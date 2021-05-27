// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) ASPEED Technology Inc.
 */

#include <linux/poll.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/completion.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <asm/uaccess.h>

#define MCTP_DESC_SIZE			4096	//for tx/rx descript
#define MCTP_TX_BUFF_SIZE		4096
#define MCTP_RX_BUFF_POOL_SIZE		16384
#define MCTP_G6_RX_BUFF_POOL_SIZE	(MCTP_RX_BUFF_POOL_SIZE * 4)
#define MCTP_TX_FIFO_NUM		1
#define MCTP_G6_TX_FIFO_NUM		8
#define MCTP_G6_RX_DEFAULT_PAYLOAD	64
#define MCTP_G6_TX_DEFAULT_PAYLOAD	64
#define MCTP_DMA_SIZE			(MCTP_DESC_SIZE * 2 + MCTP_TX_BUFF_SIZE * MCTP_TX_FIFO_NUM + MCTP_RX_BUFF_POOL_SIZE)
#define MCTP_G6_DMA_SIZE		(MCTP_DESC_SIZE * 2 + MCTP_TX_BUFF_SIZE * MCTP_G6_TX_FIFO_NUM + MCTP_G6_RX_BUFF_POOL_SIZE)
#define MCTP_G6A3_DMA_SIZE		(MCTP_DESC_SIZE * 2 + MCTP_TX_BUFF_SIZE * MCTP_G6_TX_FIFO_NUM + MCTP_RX_BUFF_POOL_SIZE)
#define MCTP_RX_DESC_BUFF_NUM		8

#define G4_DRAM_BASE_ADDR		0x40000000
#define G5_DRAM_BASE_ADDR		0x80000000
#define G6_DRAM_BASE_ADDR		0x80000000

#define ASPEED_MCTP_CTRL		0x00
#define  MCTP_GET_CUR_CMD_CNT(x)	(((x) >> 24) & 0x3f)
#define  MCTP_RX_PCIE_IDLE		BIT(21)
#define  MCTP_RX_DMA_IDLE		BIT(20)
#define  MCTP_TX_PCIE_IDLE		BIT(17)
#define  MCTP_TX_DMA_IDLE		BIT(16)
#define  MCTP_CPL2_ENABLE		BIT(15)
#define  MCTP_2600A3_PATCH_DISABLE	BIT(15)
#define  MCTP_MATCH_EID			BIT(9)
#define  MCTP_RX_CMD_RDY		BIT(4)
#define  MCTP_TX_TRIGGER		BIT(0)
#define ASPEED_MCTP_TX_CMD		0x04
#define ASPEED_MCTP_RX_CMD		0x08
#define ASPEED_MCTP_ISR			0x0c
#define ASPEED_MCTP_IER			0x10
#define  MCTP_MSG_OBFF_STS_CHG		BIT(27)
#define  MCTP_MSG_OBFF_ACTIVE		BIT(26)
#define  MCTP_MSG_OBFF_IDLE		BIT(25)
#define  MCTP_MSG_OBFF_STATE		BIT(24)
#define  MCTP_WAKE_OBFF_ACTIVE		BIT(19)
#define  MCTP_WAKE_POST_OBFF		BIT(18)
#define  MCTP_WAKE_OBFF_STATE		BIT(17)
#define  MCTP_WAKE_OBFF_IDLE		BIT(16)
#define  MCTP_RX_NO_CMD			BIT(9)
#define  MCTP_RX_COMPLETE		BIT(8)
#define  MCTP_TX_CMD_WRONG		BIT(2)	//ast-g6 mctp
#define  MCTP_TX_LAST			BIT(1)
#define  MCTP_TX_COMPLETE		BIT(0)
#define ASPEED_MCTP_EID			0x14
#define ASPEED_MCTP_OBFF_CTRL		0x18
/* aspeed g6 mctp */
#define ASPEED_MCTP_CTRL1		0x1C
#define  MCTP_FIFO_FULL			BIT(19)
#define  MCTP_OBFF_DMA_ENABLE		BIT(18)
#define  MCTP_OBFF_MONITOR		BIT(17)
#define  MCTP_6KB_TX_2KB_RX_FIFO	(0 << 8)
#define  MCTP_4KB_TX_4KB_RX_FIFO	BIT(8)
#define  MCTP_2KB_TX_6KB_RX_FIFO	(2 << 8)
#define  MCTP_RX_PAYLOAD_64BYTE		(0 << 4)
#define  MCTP_RX_PAYLOAD_128BYTE	BIT(4)
#define  MCTP_RX_PAYLOAD_256BYTE	(2 << 4)
#define  MCTP_RX_PAYLOAD_512BYTE	(3 << 4)
#define  MCTP_TX_PAYLOAD_64BYTE		(0)
#define  MCTP_TX_PAYLOAD_128BYTE	(1)
#define  MCTP_TX_PAYLOAD_256BYTE	(2)
#define  MCTP_TX_PAYLOAD_512BYTE	(3)
#define ASPEED_MCTP_RX_DESC_ADDR	0x20
#define ASPEED_MCTP_RX_DESC_NUM		0x24
#define ASPEED_MCTP_RX_WRITE_PT		0x28
#define  MCTP_HW_READ_PT_UPDATE		0x80000000
#define  MCTP_HW_READ_PT_NUM_MASK	0xfff
#define ASPEED_MCTP_RX_READ_PT		0x2C
/* ast2600 mctp use tx cmd descript */
#define ASPEED_MCTP_TX_DESC_ADDR	0x30
#define ASPEED_MCTP_TX_DESC_NUM		0x34
#define ASPEED_MCTP_TX_READ_PT		0x38
#define ASPEED_MCTP_TX_WRITE_PT		0x3C

#define ASPEED_MCTP_2400		0
#define ASPEED_MCTP_2500		5
#define ASPEED_MCTP_2600		6
#define ASPEED_MCTP_2600A3		7

#define ASPEED_REVISION_ID0		0x04
#define ASPEED_REVISION_ID1		0x14
#define ID0_AST2600A0			0x05000303
#define ID1_AST2600A0			0x05000303
#define ID0_AST2600A1			0x05010303
#define ID1_AST2600A1			0x05010303
#define ID0_AST2600A2			0x05010303
#define ID1_AST2600A2			0x05020303
#define ID0_AST2600A3			0x05030303
#define ID1_AST2600A3			0x05030303
#define ID0_AST2620A1			0x05010203
#define ID1_AST2620A1			0x05010203
#define ID0_AST2620A2			0x05010203
#define ID1_AST2620A2			0x05020203
#define ID0_AST2620A3			0x05030203
#define ID1_AST2620A3			0x05030203
#define ID0_AST2605A2			0x05010103
#define ID1_AST2605A2			0x05020103
#define ID0_AST2605A3			0x05030103
#define ID1_AST2605A3			0x05030103
#define ID0_AST2625A3			0x05030403
#define ID1_AST2625A3			0x05030403

//TX CMD desc0 : ast-g4, ast-g5
#define BUS_NO(x)			(((x) & 0xff) << 24)
#define DEV_NO(x)			(((x) & 0x1f) << 19)
#define FUN_NO(x)			(((x) & 0x7) << 16)
#define INT_ENABLE			BIT(15)

//ast-g5
/* 0: route to RC, 1: route by ID, 2/3: broadcast from RC */
#define G5_ROUTING_TYPE_L(x)		(((x) & 0x1) << 14)
#define G5_ROUTING_TYPE_H(x)		((((x) & 0x2) >> 1) << 12)
//ast old version
#define ROUTING_TYPE(x)			(((x) & 0x1) << 14)

#define TAG_OWN(x)			((x) << 13)

//bit 12:2 is packet in 4bytes
//ast2400 bit 12 can be use.
//ast2500 bit 12 can't be used. 0: 1024 * 4 = 4096
#define G5_PKG_SIZE(x)			(((x) & 0x3ff) << 2)
#define PKG_SIZE(x)			(((x) & 0x7ff) << 2)

#define PADDING_LEN(x)			((x) & 0x3)
//TX CMD desc1
#define LAST_CMD			BIT(31)
//ast-g5
#define G5_TX_DATA_ADDR(x)		((((x) >> 7) & 0x7fffff) << 8)
//ast old version
#define TX_DATA_ADDR(x)			((((x) >> 6) & 0x7fffff) << 8)

#define DEST_EP_ID(x)			((x) & 0xff)

//RX CMD desc0
#define GET_PKG_LEN(x)			(((x) >> 24) & 0x7f)
#define GET_SRC_EPID(x)			(((x) >> 16) & 0xff)
#define GET_ROUTING_TYPE(x)		(((x) >> 14) & 0x7)
#define GET_SEQ_NO(x)			(((x) >> 11) & 0x3)
#define GET_MSG_TAG(x)			(((x) >> 8) & 0x3)
#define MCTP_SOM			BIT(7)
#define GET_MCTP_SOM(x)			(((x) >> 7) & 0x1)
#define MCTP_EOM			BIT(6)
#define GET_MCTP_EOM(x)			(((x) >> 6) & 0x1)
#define GET_PADDING_LEN(x)		(((x) >> 4) & 0x3)
#define CMD_UPDATE			BIT(0)
//RX CMD desc1
#define LAST_CMD			BIT(31)
#define RX_DATA_ADDR(x)			((x) & 0x3fffff80)

#define G6_TX_DATA_ADDR(x)		((((x) >> 2) & 0x1fffffff) << 2)
#define G6_TX_DESC_VALID		(0x1)

struct aspeed_mctp_cmd_desc {
	unsigned int desc0;
	unsigned int desc1;
};

//ast2400 tx cmd desc

//ast2500 tx cmd desc
//[31:24]:dest pcie bus#,[23:19]:dest pcie dev#, [18:16]:dest pcie fun# [15]: Int_en, [14] : Routing type 0: RC, 1 by ID,  [13]: MCTP tag owner, [12:2]: package size in 4 bytes, [1:0]: Padding length
//[31]: last cmd, [30:8] data address ,[7:0]: dest EP ID

//ast2600 tx cmd desc
//[16]: stop, [15]: Int, [12:2] payload size 4bytes, [0]: 0
//[31]: read only , [30:2]vdm address, resv, [1]: 1

//ast2400 rx cmd desc

//ast2500 rx cmd desc
//[31:24]:dest pcie bus#,[23:19]:dest pcie dev#, [18:16]:dest pcie fun# [15]: Int_en, [14] : Routing type 0: RC, 1 by ID,  [13]: MCTP tag owner, [12:2]: package size in 4 bytes, [1:0]: Padding length
//[30:24] payload size 4bytes, [23:16]: src EPID, [15:14]: routing type, [13]: tag owner ,[12:11]: mctp seq# ,[10:8]: mctp Tag, [7]: mctp SOM, [6]: mctp EOM, [5:1] reserved ,[0]: cmd updated

//ast2600 rx cmd desc

/*************************************************************************************/
#define ASPEED_MCTP_XFER_SIZE 4096

#define MCTPIOC_BASE 'M'

#define ASPEED_MCTP_IOCTX	_IOW(MCTPIOC_BASE, 0, struct aspeed_mctp_xfer*)
#define ASPEED_MCTP_IOCRX	_IOR(MCTPIOC_BASE, 1, struct aspeed_mctp_xfer*)

struct pcie_vdm_header {
	__u32		length : 10,
			revd0 : 2,
			attr : 2,
			ep : 1,
			td : 1,
			revd1 : 4,
			tc : 3,
			revd2 : 1,
			type_routing : 5,
			fmt : 2,
			revd3 : 1;
	__u8		message_code;
	__u8		vdm_code : 4,
			pad_len : 2,
			tag_revd : 2;
	__u16		pcie_req_id;
	__u16		vender_id;
	__u16		pcie_target_id;
	__u8		msg_tag : 3,
			to : 1,
			pkt_seq : 2,
			eom : 1,
			som : 1;
	__u8		src_epid;
	__u8		dest_epid;
	__u8		header_ver : 4,
			rsvd : 4;
};

struct aspeed_mctp_xfer {
	unsigned char *xfer_buff;
	struct pcie_vdm_header header;
};

// #define ASPEED_MCTP_DEBUG

#ifdef ASPEED_MCTP_DEBUG
#define MCTP_DBUG(fmt, args...) pr_info("%s() " fmt, __func__, ## args)
#else
#define MCTP_DBUG(fmt, args...)
#endif

#define MCTP_MSG(fmt, args...) pr_info(fmt, ## args)

struct aspeed_mctp_info {
	void __iomem *reg_base;
	void __iomem *pci_bdf_regs;
	struct miscdevice misc_dev;
	bool is_open;
	int irq;
	int pcie_irq;
	int mctp_version;
	struct reset_control *reset;
	u32 dram_base;

	/* mctp tx info */
	struct completion tx_complete;

	int tx_fifo_num;
	int tx_idx;
	int tx_payload;

	void *tx_pool;
	dma_addr_t tx_pool_dma;

	struct aspeed_mctp_cmd_desc *tx_cmd_desc;
	dma_addr_t tx_cmd_desc_dma;

	/* mctp rx info */
	void *rx_pool;
	dma_addr_t rx_pool_dma;

	int rx_dma_pool_size;
	int rx_fifo_size;
	int rx_fifo_num;

	void *rx_cmd_desc;
	dma_addr_t rx_cmd_desc_dma;

	int rx_idx;
	int rx_hw_idx;
	int rx_full;
	int rx_payload;

	/* for ast2600 workaround, due to the hw read ptr is not point correctly,
	 * should use other variable to track the actual cmd ptr.
	 * rx_cmd_num should be 4 times of rx_fifo_num, and rx_fifo_num should
	 * be 4n+1.
	 */
	int rx_cmd_num;
	int rx_reboot;
	int rx_first_loop;
	int rx_recv_idx;
};

/******************************************************************************/

static inline u32
aspeed_mctp_readl(struct aspeed_mctp_info *aspeed_mctp, u32 reg)
{
	u32 val;

	val = readl(aspeed_mctp->reg_base + reg);
//	MCTP_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
}

static inline void
aspeed_mctp_writel(struct aspeed_mctp_info *aspeed_mctp, u32 val, u32 reg)
{
//	MCTP_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, aspeed_mctp->reg_base + reg);
}

static int aspeed_mctp_g6_tx_xfer(struct aspeed_mctp_info *aspeed_mctp, struct aspeed_mctp_xfer *mctp_xfer)
{
	struct pcie_vdm_header *vdm_header1 = &mctp_xfer->header;
	struct pcie_vdm_header *vdm_header2;
	void *tx_buff1;
	void *tx_buff2;
	unsigned long byte_length = 0;
	int needs_fifo;
	u32 hw_read_pt;
	u32 ctrl_cmd = 0;

	byte_length = vdm_header1->length * 4 - vdm_header1->pad_len;

	MCTP_DBUG("xfer byte_length = %ld, padding len = %d\n", byte_length, vdm_header1->pad_len);

	/* In some condition, tx som and eom will not match expected result.
	 * e.g. When max payload size (MPL) set to 64 byte, and then transfer
	 * size set between 61 ~ 124 (MPL-3 ~ 2*MPL-4), the engine will set all
	 * packet vdm header eom to 1, nomether what it setted. To fix that
	 * issue, when driver receive the size between (MPL-3 ~ 2*MPL-4), it
	 * set MPL to next level(e.g. 64 to 128), and then separate the packet
	 * manually.
	 */
	switch (aspeed_mctp->tx_payload) {
	case 64:
		if (byte_length >= 61 &&  byte_length <= 64) {
			ctrl_cmd = MCTP_TX_PAYLOAD_128BYTE;
			needs_fifo = 1;
		} else if (byte_length > 64 &&  byte_length <= 124) {
			ctrl_cmd = MCTP_TX_PAYLOAD_128BYTE;
			needs_fifo = 2;
		} else {
			ctrl_cmd = MCTP_TX_PAYLOAD_64BYTE;
			needs_fifo = 1;
		}
		break;
	case 128:
		if (byte_length >= 125 &&  byte_length <= 128) {
			ctrl_cmd = MCTP_TX_PAYLOAD_256BYTE;
			needs_fifo = 1;
		} else if (byte_length > 128 &&  byte_length <= 252) {
			ctrl_cmd = MCTP_TX_PAYLOAD_256BYTE;
			needs_fifo = 2;
		} else {
			ctrl_cmd = MCTP_TX_PAYLOAD_128BYTE;
			needs_fifo = 1;
		}
		break;
	case 256:
		if (byte_length >= 253 &&  byte_length <= 256) {
			ctrl_cmd = MCTP_TX_PAYLOAD_512BYTE;
			needs_fifo = 1;
		} else if (byte_length > 256 &&  byte_length <= 508) {
			ctrl_cmd = MCTP_TX_PAYLOAD_512BYTE;
			needs_fifo = 2;
		} else {
			ctrl_cmd = MCTP_TX_PAYLOAD_256BYTE;
			needs_fifo = 1;
		}
		break;
	default:
		return -EFAULT;
	}

	/*
	 * write MCTP_HW_READ_PT_UPDATE and wait until that bit back to 0,
	 * and then do it again, these steps can guarantee hw read pt will
	 * update.
	 */
	aspeed_mctp_writel(aspeed_mctp, MCTP_HW_READ_PT_UPDATE, ASPEED_MCTP_TX_READ_PT);
	while (aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_TX_READ_PT) & MCTP_HW_READ_PT_UPDATE)
		;
	aspeed_mctp_writel(aspeed_mctp, MCTP_HW_READ_PT_UPDATE, ASPEED_MCTP_TX_READ_PT);
	while (aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_TX_READ_PT) & MCTP_HW_READ_PT_UPDATE)
		;
	hw_read_pt = aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_TX_READ_PT) & MCTP_HW_READ_PT_NUM_MASK;

	if (((aspeed_mctp->tx_idx + needs_fifo) % MCTP_G6_TX_FIFO_NUM) == hw_read_pt) {
		pr_info("TX FIFO full\n");
		return 0;
	}

	if (needs_fifo == 1) {
		tx_buff1 = aspeed_mctp->tx_pool + (MCTP_TX_BUFF_SIZE * aspeed_mctp->tx_idx);
		aspeed_mctp->tx_cmd_desc[aspeed_mctp->tx_idx].desc0 = PKG_SIZE(vdm_header1->length);
		aspeed_mctp->tx_idx++;
		aspeed_mctp->tx_idx %= MCTP_G6_TX_FIFO_NUM;

		//ast2600 support vdm header transfer
		vdm_header1->pcie_req_id = (readl(aspeed_mctp->pci_bdf_regs + 0xc4) & 0x1fff) << 3;
		memcpy(tx_buff1, vdm_header1, sizeof(struct pcie_vdm_header));
		if (copy_from_user(tx_buff1 + sizeof(struct pcie_vdm_header), mctp_xfer->xfer_buff, byte_length))
			return -EFAULT;

	} else {
		tx_buff1 = aspeed_mctp->tx_pool + (MCTP_TX_BUFF_SIZE * aspeed_mctp->tx_idx);
		aspeed_mctp->tx_cmd_desc[aspeed_mctp->tx_idx].desc0 = PKG_SIZE(aspeed_mctp->tx_payload / 4);
		aspeed_mctp->tx_idx++;
		aspeed_mctp->tx_idx %= MCTP_G6_TX_FIFO_NUM;
		tx_buff2 = aspeed_mctp->tx_pool + (MCTP_TX_BUFF_SIZE * aspeed_mctp->tx_idx);
		vdm_header2 = tx_buff2;
		memcpy(vdm_header2, vdm_header1, sizeof(struct pcie_vdm_header));

		vdm_header1->eom = 0;
		vdm_header1->length = aspeed_mctp->tx_payload / 4;
		vdm_header1->pad_len = 0;
		vdm_header1->pcie_req_id = (readl(aspeed_mctp->pci_bdf_regs + 0xc4) & 0x1fff) << 3;

		vdm_header2->som = 0;
		vdm_header2->pkt_seq++;
		vdm_header2->length = vdm_header2->length - aspeed_mctp->tx_payload / 4;
		vdm_header2->pcie_req_id = vdm_header1->pcie_req_id;

		aspeed_mctp->tx_cmd_desc[aspeed_mctp->tx_idx].desc0 = PKG_SIZE(vdm_header2->length);
		aspeed_mctp->tx_idx++;
		aspeed_mctp->tx_idx %= MCTP_G6_TX_FIFO_NUM;

		//ast2600 support vdm header transfer
		memcpy(tx_buff1, vdm_header1, sizeof(struct pcie_vdm_header));
		if (copy_from_user(tx_buff1 + sizeof(struct pcie_vdm_header), mctp_xfer->xfer_buff, aspeed_mctp->tx_payload))
			return -EFAULT;

		//ast2600 support vdm header transfer
		if (copy_from_user(tx_buff2 + sizeof(struct pcie_vdm_header), mctp_xfer->xfer_buff + aspeed_mctp->tx_payload, byte_length - aspeed_mctp->tx_payload))
			return -EFAULT;
	}

	// //add TEST for interrupt and stop
	// aspeed_mctp->tx_cmd_desc[aspeed_mctp->tx_idx].desc0 |= BIT(15) | BIT(16);
	// //
	// MCTP_DBUG("tx idx [%d] desc0 %x , desc1 %x\n", aspeed_mctp->tx_idx,
	//	aspeed_mctp->tx_cmd_desc[aspeed_mctp->tx_idx].desc0,
	//	aspeed_mctp->tx_cmd_desc[aspeed_mctp->tx_idx].desc1);

	aspeed_mctp_writel(aspeed_mctp, (aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_CTRL1) & ~(0x3)) | ctrl_cmd, ASPEED_MCTP_CTRL1);

	//trigger write pt;
	aspeed_mctp_writel(aspeed_mctp, aspeed_mctp->tx_idx, ASPEED_MCTP_TX_WRITE_PT);

	//trigger tx
	aspeed_mctp_writel(aspeed_mctp, aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_CTRL) | MCTP_TX_TRIGGER, ASPEED_MCTP_CTRL);

	return 0;
}

static int aspeed_mctp_tx_xfer(struct aspeed_mctp_info *aspeed_mctp, struct aspeed_mctp_xfer *mctp_xfer)
{
	struct pcie_vdm_header *vdm_header = &mctp_xfer->header;
	unsigned long byte_length = 0;
	u32 routing_type = vdm_header->type_routing;

	if (!vdm_header->length)
		byte_length = 4096 - vdm_header->pad_len;
	else
		byte_length = vdm_header->length * 4 - vdm_header->pad_len;

	//ast2500 only support 4096, g5 is not supporting vdm_length is 1024
	if (aspeed_mctp->mctp_version == ASPEED_MCTP_2500 && !mctp_xfer->header.length)
		return 1;

	MCTP_DBUG("xfer byte_length = %ld, padding len = %d\n", byte_length, vdm_header->pad_len);

	if (copy_from_user(aspeed_mctp->tx_pool, mctp_xfer->xfer_buff, byte_length))
		return -EFAULT;

	//old ast2400/ast2500 only one tx fifo and wait for tx complete
	init_completion(&aspeed_mctp->tx_complete);

	//if use ast2400/ast2500 need to check vdm header support
	if (vdm_header->som != vdm_header->eom) {
		pr_err("can't support som eom different som %d , eom %d\n", vdm_header->som, vdm_header->eom);
		return 1;
	}

	switch (routing_type & 0x7) {
	case 0:	//route to rc
		routing_type = 0;
		break;
	case 2:	//route to ID
		routing_type = 1;
		break;
	default:
		pr_err("not supported routing type %x ", routing_type);
		break;
	}

	switch (aspeed_mctp->mctp_version) {
	case ASPEED_MCTP_2400:
		//routing type bit 14
		//bit 15 : interrupt enable
		if (!vdm_header->length)
			aspeed_mctp->tx_cmd_desc->desc0 = INT_ENABLE | TAG_OWN(vdm_header->to) | (routing_type << 14) |
							  PKG_SIZE(0x400) | (vdm_header->pcie_target_id << 16) |
							  PADDING_LEN(vdm_header->pad_len);
		else
			aspeed_mctp->tx_cmd_desc->desc0 = INT_ENABLE | TAG_OWN(vdm_header->to) | (routing_type << 14) |
							  PKG_SIZE(vdm_header->length) | (vdm_header->pcie_target_id << 16) |
							  PADDING_LEN(vdm_header->pad_len);

		aspeed_mctp->tx_cmd_desc->desc1 = LAST_CMD | DEST_EP_ID(vdm_header->dest_epid) | TX_DATA_ADDR(aspeed_mctp->tx_pool_dma);
		aspeed_mctp_writel(aspeed_mctp, aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_IER) | MCTP_TX_LAST, ASPEED_MCTP_IER);
		break;
	case ASPEED_MCTP_2500:
		//routing type [desc0 bit 12, desc0 bit 14], but bug at bit 12, don't use
		//bit 15 : interrupt enable
		aspeed_mctp->tx_cmd_desc->desc0 = INT_ENABLE | TAG_OWN(vdm_header->to) | (routing_type << 14) |
						  G5_PKG_SIZE(vdm_header->length) | (vdm_header->pcie_target_id << 16) |
						  PADDING_LEN(vdm_header->pad_len);
		aspeed_mctp->tx_cmd_desc->desc1 = LAST_CMD | DEST_EP_ID(vdm_header->dest_epid) | G5_TX_DATA_ADDR(aspeed_mctp->tx_pool_dma);
		aspeed_mctp_writel(aspeed_mctp, aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_IER) | MCTP_TX_LAST, ASPEED_MCTP_IER);
		break;
	}

	//trigger tx
	aspeed_mctp_writel(aspeed_mctp, aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_CTRL) | MCTP_TX_TRIGGER, ASPEED_MCTP_CTRL);

	wait_for_completion(&aspeed_mctp->tx_complete);

	return 0;
}

static void aspeed_mctp_ctrl_init(struct aspeed_mctp_info *aspeed_mctp)
{
	int i = 0;

	MCTP_DBUG("dram base %x\n", aspeed_mctp->dram_base);
	aspeed_mctp_writel(aspeed_mctp, (aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_EID) & 0xff) |
			   aspeed_mctp->dram_base, ASPEED_MCTP_EID);

	aspeed_mctp->tx_idx = 0;

	if (aspeed_mctp->mctp_version == ASPEED_MCTP_2600 || aspeed_mctp->mctp_version == ASPEED_MCTP_2600A3) {
		for (i = 0; i < aspeed_mctp->tx_fifo_num; i++)
			aspeed_mctp->tx_cmd_desc[i].desc1 = ((aspeed_mctp->tx_pool_dma + (MCTP_TX_BUFF_SIZE * i)) & 0x7ffffff0) | 0x1;
		aspeed_mctp->tx_cmd_desc[aspeed_mctp->tx_fifo_num - 1].desc1 |= LAST_CMD;
		aspeed_mctp_writel(aspeed_mctp, aspeed_mctp->tx_cmd_desc_dma, ASPEED_MCTP_TX_DESC_ADDR);
		aspeed_mctp_writel(aspeed_mctp, aspeed_mctp->tx_fifo_num, ASPEED_MCTP_TX_DESC_NUM);
		aspeed_mctp_writel(aspeed_mctp, 0, ASPEED_MCTP_TX_WRITE_PT);
	} else {
		aspeed_mctp_writel(aspeed_mctp, aspeed_mctp->tx_cmd_desc_dma, ASPEED_MCTP_TX_CMD);
	}

	aspeed_mctp->rx_idx = 0;
	aspeed_mctp->rx_hw_idx = 0;

	//rx fifo data
	if (aspeed_mctp->mctp_version == ASPEED_MCTP_2600) {
		//ast2600 : each 16 bytes align and configurable 64/128/256/512 bytes can receive
		u32 *rx_cmd_desc = aspeed_mctp->rx_cmd_desc;
		u32 *rx_cmd_header;

		aspeed_mctp->rx_recv_idx = 0;
		aspeed_mctp->rx_first_loop = 1;
		aspeed_mctp->rx_reboot = 1;

		/* reserved 4 cmd for reboot issue workaround, it will be minused
		 * after first circle.
		 */
		aspeed_mctp->rx_cmd_num = aspeed_mctp->rx_fifo_num * 4 + 4;

		for (i = 0; i < aspeed_mctp->rx_cmd_num; i++) {
			rx_cmd_header = (u32 *)(aspeed_mctp->rx_pool + (aspeed_mctp->rx_fifo_size * i));
			*rx_cmd_header = 0;
			rx_cmd_desc[i] = (u32)aspeed_mctp->rx_pool_dma + (aspeed_mctp->rx_fifo_size * i);
			MCTP_DBUG("Rx [%d]: desc: %x ,\n", i, rx_cmd_desc[i]);
		}
		aspeed_mctp_writel(aspeed_mctp, aspeed_mctp->rx_cmd_desc_dma, ASPEED_MCTP_RX_DESC_ADDR);
		aspeed_mctp_writel(aspeed_mctp, aspeed_mctp->rx_fifo_num, ASPEED_MCTP_RX_DESC_NUM);
		aspeed_mctp_writel(aspeed_mctp, 0, ASPEED_MCTP_RX_READ_PT);
		aspeed_mctp_writel(aspeed_mctp, MCTP_TX_CMD_WRONG | MCTP_RX_NO_CMD, ASPEED_MCTP_IER);
		aspeed_mctp_writel(aspeed_mctp, aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_CTRL) | MCTP_RX_CMD_RDY, ASPEED_MCTP_CTRL);
		aspeed_mctp_writel(aspeed_mctp, MCTP_HW_READ_PT_UPDATE, ASPEED_MCTP_RX_WRITE_PT);
	} else if (aspeed_mctp->mctp_version == ASPEED_MCTP_2600A3) {
		//ast2600 : each 16 bytes align and configurable 64/128/256/512 bytes can receive
		u32 *rx_cmd_desc = aspeed_mctp->rx_cmd_desc;
		u32 *rx_cmd_header;
		u32 mctp_ctrl;

		for (i = 0; i < aspeed_mctp->rx_fifo_num + 20; i++) {
			rx_cmd_header = (u32 *)(aspeed_mctp->rx_pool + (aspeed_mctp->rx_fifo_size * i));
			*rx_cmd_header = 0;
			rx_cmd_desc[i] = (u32)aspeed_mctp->rx_pool_dma + (aspeed_mctp->rx_fifo_size * i);
			MCTP_DBUG("Rx [%d]: desc: %x ,\n", i, rx_cmd_desc[i]);
		}
		mctp_ctrl = aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_CTRL);
		mctp_ctrl |= MCTP_RX_CMD_RDY;
		aspeed_mctp_writel(aspeed_mctp, aspeed_mctp->rx_cmd_desc_dma, ASPEED_MCTP_RX_DESC_ADDR);
		aspeed_mctp_writel(aspeed_mctp, aspeed_mctp->rx_fifo_num, ASPEED_MCTP_RX_DESC_NUM);
		aspeed_mctp_writel(aspeed_mctp, 0, ASPEED_MCTP_RX_READ_PT);
		aspeed_mctp_writel(aspeed_mctp, MCTP_TX_CMD_WRONG | MCTP_RX_NO_CMD, ASPEED_MCTP_IER);
		aspeed_mctp_writel(aspeed_mctp, mctp_ctrl, ASPEED_MCTP_CTRL);
		aspeed_mctp_writel(aspeed_mctp, MCTP_HW_READ_PT_UPDATE, ASPEED_MCTP_RX_WRITE_PT);
	} else {
		//ast2400/ast2500 : each 128 bytes align, and only 64 bytes can receive
		struct aspeed_mctp_cmd_desc *rx_cmd_desc = aspeed_mctp->rx_cmd_desc;

		for (i = 0; i < aspeed_mctp->rx_fifo_num; i++) {
			rx_cmd_desc[i].desc0 = 0;
			rx_cmd_desc[i].desc1 = RX_DATA_ADDR(aspeed_mctp->rx_pool_dma + (aspeed_mctp->rx_fifo_size * i));
			if (i == (aspeed_mctp->rx_fifo_num - 1))
				rx_cmd_desc[i].desc1 |= LAST_CMD;
			MCTP_DBUG("Rx [%d]: desc0: %x , desc1: %x\n", i, rx_cmd_desc[i].desc0, rx_cmd_desc[i].desc1);
		}
		aspeed_mctp_writel(aspeed_mctp, aspeed_mctp->rx_cmd_desc_dma, ASPEED_MCTP_RX_CMD);
		aspeed_mctp_writel(aspeed_mctp, MCTP_RX_COMPLETE | MCTP_RX_NO_CMD, ASPEED_MCTP_IER);
		aspeed_mctp_writel(aspeed_mctp, aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_CTRL) | MCTP_RX_CMD_RDY, ASPEED_MCTP_CTRL);
	}
}

static int aspeed_mctp_rx_xfer(struct aspeed_mctp_info *aspeed_mctp, struct aspeed_mctp_xfer *mctp_xfer, char __user *buf)
{
	struct aspeed_mctp_cmd_desc *rx_cmd_desc = aspeed_mctp->rx_cmd_desc;
	u32 desc0 = rx_cmd_desc[aspeed_mctp->rx_idx].desc0;
	unsigned int pci_bdf;
	int recv_length;
	int ret;

	if (aspeed_mctp->rx_idx == aspeed_mctp->rx_hw_idx && !desc0) {
		if (aspeed_mctp->rx_full) {
			MCTP_DBUG("re-trigger\n");
			aspeed_mctp_ctrl_init(aspeed_mctp);
			aspeed_mctp->rx_full = 0;
		}
		return 0;
	}

	if (!desc0)
		return 0;

	mctp_xfer->header.length = GET_PKG_LEN(desc0);
	MCTP_DBUG("mctp_xfer->header.length %d\n", mctp_xfer->header.length);

	if (mctp_xfer->header.length != 0 && mctp_xfer->header.length < GET_PKG_LEN(desc0))
		return -EINVAL;

	mctp_xfer->header.pad_len = GET_PADDING_LEN(desc0);
	mctp_xfer->header.src_epid = GET_SRC_EPID(desc0);
	mctp_xfer->header.type_routing = GET_ROUTING_TYPE(desc0);
	mctp_xfer->header.pkt_seq = GET_SEQ_NO(desc0);
	mctp_xfer->header.msg_tag = GET_MSG_TAG(desc0);
	mctp_xfer->header.eom = GET_MCTP_EOM(desc0);
	mctp_xfer->header.som = GET_MCTP_SOM(desc0);
	// 0x1e6ed0c4[4:0]: Dev#
	// 0x1e6ed0c4[12:5]: Bus#
	// Fun# always 0
	if (aspeed_mctp->mctp_version != ASPEED_MCTP_2400) {
		pci_bdf = readl(aspeed_mctp->pci_bdf_regs + 0xc4);
		mctp_xfer->header.pcie_target_id = (pci_bdf & 0x1f) << 3 |
						   (pci_bdf >> 5 & 0xff) << 8;
	}
	recv_length = (mctp_xfer->header.length * 4);

	ret = copy_to_user(mctp_xfer->xfer_buff,
			   aspeed_mctp->rx_pool + (aspeed_mctp->rx_fifo_size * aspeed_mctp->rx_idx),
			   recv_length);
	if (ret)
		return ret;

	ret = copy_to_user(buf, mctp_xfer, sizeof(struct aspeed_mctp_xfer));
	if (ret)
		return ret;

	rx_cmd_desc[aspeed_mctp->rx_idx].desc0 = 0;
	aspeed_mctp->rx_idx++;
	aspeed_mctp->rx_idx %= aspeed_mctp->rx_fifo_num;

	return 0;
}

static int aspeed_mctp_g6_rx_xfer(struct aspeed_mctp_info *aspeed_mctp, struct aspeed_mctp_xfer *mctp_xfer, char __user *buf)
{
	struct pcie_vdm_header *vdm;
	u32 hw_read_pt;
	u32 *rx_buffer;
	u32 *header_dw;
	int recv_length;
	int ret;

	aspeed_mctp_writel(aspeed_mctp, MCTP_HW_READ_PT_UPDATE, ASPEED_MCTP_RX_WRITE_PT);
	hw_read_pt = aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_RX_WRITE_PT) & MCTP_HW_READ_PT_NUM_MASK;

	if (aspeed_mctp->rx_idx == hw_read_pt) {
		// MCTP_DBUG("No rx data\n");
		return 0;
	}

	// when reboot first round, drop old data.
	while (aspeed_mctp->rx_reboot) {
		header_dw = aspeed_mctp->rx_pool + (aspeed_mctp->rx_fifo_size * aspeed_mctp->rx_recv_idx);
		if (*header_dw != 0) {
			aspeed_mctp->rx_reboot = 0;
			break;
		}
		aspeed_mctp->rx_recv_idx++;
		aspeed_mctp->rx_recv_idx %= aspeed_mctp->rx_cmd_num;
	}

	if (aspeed_mctp->rx_first_loop) {
		int wrap_around = 0;

		header_dw = aspeed_mctp->rx_pool + (aspeed_mctp->rx_fifo_size * aspeed_mctp->rx_recv_idx);
		while (*header_dw == 0) {
			MCTP_DBUG("first loop header_dw == 0");
			MCTP_DBUG("first loop rx_recv_idx:%d", aspeed_mctp->rx_recv_idx);
			aspeed_mctp->rx_recv_idx++;
			if (aspeed_mctp->rx_recv_idx >= aspeed_mctp->rx_cmd_num)
				wrap_around = 1;
			aspeed_mctp->rx_recv_idx %= aspeed_mctp->rx_cmd_num;
			header_dw = aspeed_mctp->rx_pool + (aspeed_mctp->rx_fifo_size * aspeed_mctp->rx_recv_idx);
		}
		if (wrap_around) {
			MCTP_DBUG("wrap around");
			aspeed_mctp->rx_first_loop = 0;
			aspeed_mctp->rx_cmd_num -= 4;
		}
	} else {
		header_dw = aspeed_mctp->rx_pool + (aspeed_mctp->rx_fifo_size * aspeed_mctp->rx_recv_idx);
		while (*header_dw == 0) {
			MCTP_DBUG("header_dw == 0");
			aspeed_mctp->rx_recv_idx++;
			aspeed_mctp->rx_recv_idx %= aspeed_mctp->rx_cmd_num;
			header_dw = aspeed_mctp->rx_pool + (aspeed_mctp->rx_fifo_size * aspeed_mctp->rx_recv_idx);
		}
	}

	MCTP_DBUG("rx_recv_idx:%d, rx_idx:%d", aspeed_mctp->rx_recv_idx, aspeed_mctp->rx_idx);
	MCTP_DBUG("rx_cmd_num:%d, rx_fifo_num:%d", aspeed_mctp->rx_cmd_num, aspeed_mctp->rx_fifo_num);
	MCTP_DBUG("rx_first_loop:%d, rx_reboot:%d", aspeed_mctp->rx_first_loop, aspeed_mctp->rx_reboot);

	vdm = aspeed_mctp->rx_pool + (aspeed_mctp->rx_fifo_size * aspeed_mctp->rx_recv_idx);
	header_dw = (u32 *)vdm;
	rx_buffer = aspeed_mctp->rx_pool + (aspeed_mctp->rx_fifo_size * aspeed_mctp->rx_recv_idx) + sizeof(struct pcie_vdm_header);

	recv_length = (vdm->length * 4) + vdm->pad_len;
	mctp_xfer->header = *vdm;

	ret = copy_to_user(mctp_xfer->xfer_buff, rx_buffer, recv_length);
	if (ret)
		return ret;

	ret = copy_to_user(buf, mctp_xfer, sizeof(struct aspeed_mctp_xfer));
	if (ret)
		return ret;

	*header_dw = 0;
	aspeed_mctp->rx_recv_idx++;
	aspeed_mctp->rx_recv_idx %= aspeed_mctp->rx_cmd_num;

	aspeed_mctp->rx_idx++;
	aspeed_mctp->rx_idx %= aspeed_mctp->rx_fifo_num;
	aspeed_mctp_writel(aspeed_mctp, aspeed_mctp->rx_idx, ASPEED_MCTP_RX_READ_PT);

	return 0;
}

static int aspeed_mctp_g6a3_rx_xfer(struct aspeed_mctp_info *aspeed_mctp, struct aspeed_mctp_xfer *mctp_xfer, char __user *buf)
{
	struct pcie_vdm_header *vdm;
	u32 hw_read_pt;
	u32 *rx_buffer;
	u32 *header_dw;
	int recv_length;
	int ret;

	aspeed_mctp_writel(aspeed_mctp, MCTP_HW_READ_PT_UPDATE, ASPEED_MCTP_RX_WRITE_PT);
	hw_read_pt = aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_RX_WRITE_PT) & MCTP_HW_READ_PT_NUM_MASK;

	if (aspeed_mctp->rx_idx == hw_read_pt) {
		// MCTP_DBUG("No rx data\n");
		return 0;
	}

	vdm = aspeed_mctp->rx_pool + (aspeed_mctp->rx_fifo_size * aspeed_mctp->rx_idx);
	header_dw = (u32 *)vdm;
	rx_buffer = aspeed_mctp->rx_pool + (aspeed_mctp->rx_fifo_size * aspeed_mctp->rx_idx) + sizeof(struct pcie_vdm_header);

	recv_length = (vdm->length * 4) + vdm->pad_len;
	mctp_xfer->header = *vdm;

	ret = copy_to_user(mctp_xfer->xfer_buff, rx_buffer, recv_length);
	if (ret)
		return ret;

	ret = copy_to_user(buf, mctp_xfer, sizeof(struct aspeed_mctp_xfer));
	if (ret)
		return ret;

	aspeed_mctp->rx_idx++;
	aspeed_mctp->rx_idx %= aspeed_mctp->rx_fifo_num;
	aspeed_mctp_writel(aspeed_mctp, aspeed_mctp->rx_idx, ASPEED_MCTP_RX_READ_PT);

	return 0;
}

static irqreturn_t aspeed_pcie_raise_isr(int this_irq, void *dev_id)
{
	struct aspeed_mctp_info *aspeed_mctp = dev_id;

	MCTP_DBUG("\n");
	aspeed_mctp_ctrl_init(aspeed_mctp);
	return IRQ_HANDLED;
}

static irqreturn_t aspeed_mctp_isr(int this_irq, void *dev_id)
{
	struct aspeed_mctp_info *aspeed_mctp = dev_id;
	u32 status = aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_ISR);

	MCTP_DBUG("%x\n", status);

	if (status & MCTP_TX_LAST) {
		//only for ast2400/ast2500, ast2600 is tx fifo
		aspeed_mctp_writel(aspeed_mctp, MCTP_TX_LAST, ASPEED_MCTP_ISR);
		complete(&aspeed_mctp->tx_complete);
	}

	if (status & MCTP_TX_COMPLETE) {
		aspeed_mctp_writel(aspeed_mctp, MCTP_TX_COMPLETE, ASPEED_MCTP_ISR);
		pr_info("don't care don't use\n");
	}

	if (status & MCTP_RX_COMPLETE) {
		if (aspeed_mctp->mctp_version == ASPEED_MCTP_2600 || aspeed_mctp->mctp_version == ASPEED_MCTP_2600A3) {
			if (((aspeed_mctp->rx_hw_idx + 1) % aspeed_mctp->rx_fifo_num) == aspeed_mctp->rx_idx)
				aspeed_mctp->rx_full = 1;
			else
				aspeed_mctp->rx_hw_idx++;
			aspeed_mctp->rx_hw_idx %= aspeed_mctp->rx_fifo_num;
		} else {
			struct aspeed_mctp_cmd_desc *rx_cmd_desc = aspeed_mctp->rx_cmd_desc;

			while (rx_cmd_desc[aspeed_mctp->rx_hw_idx].desc0 & CMD_UPDATE) {
				aspeed_mctp->rx_hw_idx++;
				aspeed_mctp->rx_hw_idx %= aspeed_mctp->rx_fifo_num;
				if (aspeed_mctp->rx_hw_idx == aspeed_mctp->rx_idx)
					break;
			}
		}
		aspeed_mctp_writel(aspeed_mctp, MCTP_RX_COMPLETE, ASPEED_MCTP_ISR);
	}

	if (status & MCTP_RX_NO_CMD) {
		aspeed_mctp->rx_full = 1;
		aspeed_mctp_writel(aspeed_mctp, MCTP_RX_NO_CMD, ASPEED_MCTP_ISR);
		pr_info("MCTP_RX_NO_CMD\n");
	}

	return IRQ_HANDLED;
}

static ssize_t aspeed_mctp_write(struct file *file, const char __user *buf,
				 size_t len, loff_t *offset)
{
	int ret;
	struct miscdevice *c = file->private_data;
	struct aspeed_mctp_info *aspeed_mctp = container_of(c, struct aspeed_mctp_info, misc_dev);
	struct aspeed_mctp_xfer mctp_xfer;

	if (len != sizeof(mctp_xfer))
		return -EINVAL;

	ret = copy_from_user(&mctp_xfer, buf, len);
	if (ret)
		return ret;

	MCTP_DBUG("ASPEED_MCTP_IOCTX ver %d\n", aspeed_mctp->mctp_version);

	if (aspeed_mctp->mctp_version != ASPEED_MCTP_2400) {
		if ((readl(aspeed_mctp->pci_bdf_regs + 0xc4) & 0x1fff) == 0) {
			pr_warn("PCIE not ready\n");
			return -EFAULT;
		}
	}
	switch (aspeed_mctp->mctp_version) {
	case ASPEED_MCTP_2600A3:
	case ASPEED_MCTP_2600:
		ret = aspeed_mctp_g6_tx_xfer(aspeed_mctp, &mctp_xfer);
		break;
	case ASPEED_MCTP_2500:
	case ASPEED_MCTP_2400:
	default:
		ret = aspeed_mctp_tx_xfer(aspeed_mctp, &mctp_xfer);
	}

	if (ret)
		return -EFAULT;

	return len;
}

static ssize_t aspeed_mctp_read(struct file *file, char __user *buf,
				size_t len, loff_t *offset)
{
	int ret;
	struct miscdevice *c = file->private_data;
	struct aspeed_mctp_info *aspeed_mctp = container_of(c, struct aspeed_mctp_info, misc_dev);
	struct aspeed_mctp_xfer mctp_xfer;

	if (len != sizeof(mctp_xfer))
		return -EINVAL;

	ret = copy_from_user(&mctp_xfer, buf, len);
	if (ret)
		return ret;

	switch (aspeed_mctp->mctp_version) {
	case ASPEED_MCTP_2600:
		ret = aspeed_mctp_g6_rx_xfer(aspeed_mctp, &mctp_xfer, buf);
		break;
	case ASPEED_MCTP_2600A3:
		ret = aspeed_mctp_g6a3_rx_xfer(aspeed_mctp, &mctp_xfer, buf);
		break;
	case ASPEED_MCTP_2400:
	case ASPEED_MCTP_2500:
	default:
		ret = aspeed_mctp_rx_xfer(aspeed_mctp, &mctp_xfer, buf);
	}

	if (ret)
		return ret;
	return len;
}

static long aspeed_mctp_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_mctp_info *aspeed_mctp = container_of(c, struct aspeed_mctp_info, misc_dev);
	struct aspeed_mctp_xfer mctp_xfer;
	void __user *argp = (void __user *)arg;
	int ret;

	if (copy_from_user(&mctp_xfer, argp, sizeof(struct aspeed_mctp_xfer)))
		return -EFAULT;

	switch (cmd) {
	case ASPEED_MCTP_IOCTX:
		MCTP_DBUG("ASPEED_MCTP_IOCTX ver %d\n", aspeed_mctp->mctp_version);
		if (aspeed_mctp->mctp_version != ASPEED_MCTP_2400) {
			if ((readl(aspeed_mctp->pci_bdf_regs + 0xc4) & 0x1fff) == 0) {
				pr_warn("PCIE not ready\n");
				return -EFAULT;
			}
		}

		switch (aspeed_mctp->mctp_version) {
		case ASPEED_MCTP_2600A3:
		case ASPEED_MCTP_2600:
			ret = aspeed_mctp_g6_tx_xfer(aspeed_mctp, &mctp_xfer);
			break;
		case ASPEED_MCTP_2500:
		case ASPEED_MCTP_2400:
		default:
			ret = aspeed_mctp_tx_xfer(aspeed_mctp, &mctp_xfer);
		}

		if (ret)
			return -EFAULT;
		break;
	case ASPEED_MCTP_IOCRX:
		// MCTP_DBUG("ASPEED_MCTP_IOCRX\n");
		switch (aspeed_mctp->mctp_version) {
		case ASPEED_MCTP_2600:
			ret = aspeed_mctp_g6_rx_xfer(aspeed_mctp, &mctp_xfer, argp);
			break;
		case ASPEED_MCTP_2600A3:
			ret = aspeed_mctp_g6a3_rx_xfer(aspeed_mctp, &mctp_xfer, argp);
			break;
		case ASPEED_MCTP_2400:
		case ASPEED_MCTP_2500:
		default:
			ret = aspeed_mctp_rx_xfer(aspeed_mctp, &mctp_xfer, argp);
		}

		if (ret)
			return ret;
		break;

	default:
		MCTP_DBUG("ERROR\n");
		return -ENOTTY;
	}

	return 0;
}

static int aspeed_mctp_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_mctp_info *aspeed_mctp = container_of(c, struct aspeed_mctp_info, misc_dev);

	MCTP_DBUG("\n");
	if (aspeed_mctp->is_open)
		return -EBUSY;

	aspeed_mctp->is_open = true;
	return 0;
}

static int aspeed_mctp_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_mctp_info *aspeed_mctp = container_of(c, struct aspeed_mctp_info, misc_dev);

	MCTP_DBUG("\n");
	aspeed_mctp->is_open = false;
	return 0;
}

static const struct file_operations aspeed_mctp_fops = {
	.owner		= THIS_MODULE,
	.read		= aspeed_mctp_read,
	.write		= aspeed_mctp_write,
	.unlocked_ioctl	= aspeed_mctp_ioctl,
	.open		= aspeed_mctp_open,
	.release	= aspeed_mctp_release,
};

static const struct of_device_id aspeed_mctp_of_matches[] = {
	{ .compatible = "aspeed,ast2400-mctp", .data = (void *)ASPEED_MCTP_2400, },
	{ .compatible = "aspeed,ast2500-mctp", .data = (void *)ASPEED_MCTP_2500, },
	{ .compatible = "aspeed,ast2600-mctp", .data = (void *)ASPEED_MCTP_2600, },
	{},
};

static uint32_t chip_version(u32 revid0, u32 revid1)
{
	if (revid0 == ID0_AST2600A3 && revid1 == ID1_AST2600A3) {
		/* AST2600-A3 */
		return ASPEED_MCTP_2600A3;
	} else if (revid0 == ID0_AST2620A3 && revid1 == ID1_AST2620A3) {
		/* AST2620-A3 */
		return ASPEED_MCTP_2600A3;
	} else if (revid0 == ID0_AST2605A3 && revid1 == ID1_AST2605A3) {
		/* AST2605-A3 */
		return ASPEED_MCTP_2600A3;
	} else if (revid0 == ID0_AST2625A3 && revid1 == ID1_AST2625A3) {
		/* AST2605-A3 */
		return ASPEED_MCTP_2600A3;
	}
	return ASPEED_MCTP_2600;
}

static int reserved_idx = -1;

static int aspeed_mctp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap *scu;
	struct resource *res;
	struct aspeed_mctp_info *aspeed_mctp;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *mctp_dev_id;
	size_t dma_size;
	int rc_f;
	int max_reserved_idx;
	int idx;
	int fifo_num;
	int ret = 0;
	u32 ctrl_cmd = 0;
	u32 revid0, revid1;

	MCTP_DBUG("\n");

	aspeed_mctp = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_mctp_info), GFP_KERNEL);
	if (!aspeed_mctp) {
		ret = -EINVAL;
		goto out;
	}

	mctp_dev_id = of_match_device(aspeed_mctp_of_matches, &pdev->dev);
	if (!mctp_dev_id) {
		ret = -EINVAL;
		goto out;
	}

	aspeed_mctp->mctp_version = (unsigned long)mctp_dev_id->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -EINVAL;
		goto out;
	}

	aspeed_mctp->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!aspeed_mctp->reg_base) {
		dev_err(&pdev->dev, "ioremap fail\n");
		ret = -EIO;
		goto out;
	}

	switch (aspeed_mctp->mctp_version) {
	case ASPEED_MCTP_2400:
		aspeed_mctp->tx_fifo_num = MCTP_TX_FIFO_NUM;
		aspeed_mctp->rx_fifo_size = 128;
		aspeed_mctp->rx_fifo_num = MCTP_RX_BUFF_POOL_SIZE / aspeed_mctp->rx_fifo_size;
		aspeed_mctp->dram_base = G4_DRAM_BASE_ADDR;
		dma_size = MCTP_DMA_SIZE;
		break;
	case ASPEED_MCTP_2500:
		aspeed_mctp->tx_fifo_num = MCTP_TX_FIFO_NUM;
		aspeed_mctp->rx_fifo_size = 128;
		aspeed_mctp->rx_fifo_num = MCTP_RX_BUFF_POOL_SIZE / aspeed_mctp->rx_fifo_size;
		aspeed_mctp->dram_base = G5_DRAM_BASE_ADDR;
		dma_size = MCTP_DMA_SIZE;
		break;
	case ASPEED_MCTP_2600:
		scu = syscon_regmap_lookup_by_phandle(dev->of_node, "aspeed,scu");
		if (IS_ERR(scu)) {
			dev_err(dev, "failed to find 2600 SCU regmap\n");
			return PTR_ERR(scu);
		}

		regmap_read(scu, ASPEED_REVISION_ID0, &revid0);
		regmap_read(scu, ASPEED_REVISION_ID1, &revid1);

		aspeed_mctp->mctp_version = chip_version(revid0, revid1);

		if (of_property_read_u32(np, "rx-payload-bytes",
					 &aspeed_mctp->rx_payload)) {
			aspeed_mctp->rx_payload = MCTP_G6_RX_DEFAULT_PAYLOAD;
		}
		switch (aspeed_mctp->rx_payload) {
		case 64:
			ctrl_cmd |= MCTP_RX_PAYLOAD_64BYTE;
			break;
		case 128:
			ctrl_cmd |= MCTP_RX_PAYLOAD_128BYTE;
			break;
		case 256:
			ctrl_cmd |= MCTP_RX_PAYLOAD_256BYTE;
			break;
		case 512:
			ctrl_cmd |= MCTP_RX_PAYLOAD_512BYTE;
			break;
		default:
			dev_err(&pdev->dev, "rx payload only support 64, 128 and 256 bytes\n");
			ret = -EINVAL;
			goto out;
		}

		if (of_property_read_u32(np, "tx-payload-bytes",
					 &aspeed_mctp->tx_payload)) {
			aspeed_mctp->tx_payload = MCTP_G6_TX_DEFAULT_PAYLOAD;
		}
		switch (aspeed_mctp->tx_payload) {
		case 64:
			ctrl_cmd |= MCTP_TX_PAYLOAD_64BYTE;
			break;
		case 128:
			ctrl_cmd |= MCTP_TX_PAYLOAD_128BYTE;
			break;
		case 256:
			ctrl_cmd |= MCTP_TX_PAYLOAD_256BYTE;
			break;
		case 512:
			ctrl_cmd |= MCTP_TX_PAYLOAD_512BYTE;
			break;
		default:
			dev_err(&pdev->dev, "tx payload only support 64, 128 and 256 bytes\n");
			ret = -EINVAL;
			goto out;
		}

		aspeed_mctp_writel(aspeed_mctp, aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_CTRL1) | ctrl_cmd, ASPEED_MCTP_CTRL1);

		aspeed_mctp->tx_fifo_num = MCTP_G6_TX_FIFO_NUM;
		//must 16byte align
		aspeed_mctp->rx_fifo_size = (aspeed_mctp->rx_payload + 16);
		aspeed_mctp->dram_base = G6_DRAM_BASE_ADDR;
		dma_size = MCTP_G6_DMA_SIZE;
		if (aspeed_mctp->mctp_version == ASPEED_MCTP_2600) {
			// for ast2600 workaround, rx_fifo_num should be 4n+1
			fifo_num = ((MCTP_G6_RX_BUFF_POOL_SIZE / aspeed_mctp->rx_fifo_size) / 4) - 1;
			aspeed_mctp->rx_fifo_num = fifo_num - ((fifo_num - 1) % 4);
		} else {
			// for ast2600 workaround, rx_fifo_num should be 4n
			aspeed_mctp->rx_fifo_num = MCTP_RX_BUFF_POOL_SIZE / aspeed_mctp->rx_fifo_size / 4 * 4;
		}
		break;
	default:
		dev_err(&pdev->dev, "cannot get mctp version\n");
		ret = -EINVAL;
		goto out;
	}

	aspeed_mctp->tx_cmd_desc =
		dma_alloc_coherent(&pdev->dev, dma_size, &aspeed_mctp->tx_cmd_desc_dma, GFP_KERNEL);
	aspeed_mctp->rx_cmd_desc = (void *)aspeed_mctp->tx_cmd_desc + MCTP_DESC_SIZE;
	aspeed_mctp->rx_cmd_desc_dma = aspeed_mctp->tx_cmd_desc_dma + MCTP_DESC_SIZE;
	aspeed_mctp->tx_pool = aspeed_mctp->rx_cmd_desc + MCTP_DESC_SIZE;
	aspeed_mctp->tx_pool_dma = aspeed_mctp->rx_cmd_desc_dma + MCTP_DESC_SIZE;
	aspeed_mctp->rx_pool = aspeed_mctp->tx_pool + MCTP_TX_BUFF_SIZE * aspeed_mctp->tx_fifo_num;
	aspeed_mctp->rx_pool_dma = aspeed_mctp->tx_pool_dma + MCTP_TX_BUFF_SIZE * aspeed_mctp->tx_fifo_num;
	MCTP_DBUG("rx_pool_dma: %x\n", (unsigned int)aspeed_mctp->rx_pool_dma);
	MCTP_DBUG("rx_cmd_desc: %x\n", (unsigned int)aspeed_mctp->rx_cmd_desc);

	if (of_find_property(np, "pcie_rc", NULL) == 0)
		rc_f = 0;
	else
		rc_f = 1;

	// g4 not support pcie host controller
	if (aspeed_mctp->mctp_version != ASPEED_MCTP_2400) {
		struct resource pcie_res;
		struct device_node *pcie_np;

		if (rc_f) {
			pcie_np = of_find_compatible_node(NULL, NULL, "aspeed,ast2600-pcie");
		} else {
			pcie_np = of_find_compatible_node(NULL, NULL, "aspeed,ast2600-pcie-ep");
			// only ep has mctp interrupt
			aspeed_mctp->irq = platform_get_irq_byname(pdev, "mctp");
			ret = devm_request_irq(&pdev->dev, aspeed_mctp->irq, aspeed_mctp_isr,
					       0, dev_name(&pdev->dev), aspeed_mctp);
			if (ret) {
				dev_err(&pdev->dev, "MCTP Unable to get IRQ");
				goto free;
			}
		}

		if (of_address_to_resource(pcie_np, 0, &pcie_res)) {
			dev_err(&pdev->dev, "failed to find pcie resource\n");
			ret = -ENOMEM;
			goto free;
		}
		aspeed_mctp->pci_bdf_regs = ioremap(pcie_res.start, resource_size(&pcie_res));
		if (!aspeed_mctp->pci_bdf_regs) {
			dev_err(&pdev->dev, "failed to remap pcie\n");
			ret = -ENOMEM;
			goto free;
		}
	} else {
		aspeed_mctp->irq = platform_get_irq_byname(pdev, "mctp");
		ret = devm_request_irq(&pdev->dev, aspeed_mctp->irq, aspeed_mctp_isr,
				       0, dev_name(&pdev->dev), aspeed_mctp);
		if (ret) {
			dev_err(&pdev->dev, "MCTP Unable to get IRQ");
			goto free;
		}
	}

	aspeed_mctp->reset = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(aspeed_mctp->reset)) {
		dev_err(&pdev->dev, "can't get mctp reset\n");
		return PTR_ERR(aspeed_mctp->reset);
	}

//scu init
	if (aspeed_mctp->mctp_version == ASPEED_MCTP_2600 || aspeed_mctp->mctp_version == ASPEED_MCTP_2600A3) {
		if (rc_f || aspeed_mctp_readl(aspeed_mctp, ASPEED_MCTP_TX_DESC_ADDR) == 0) {
			pr_info("aspeed_mctp: reset.\n");
			reset_control_assert(aspeed_mctp->reset);
			reset_control_deassert(aspeed_mctp->reset);
		}
	} else {
		reset_control_assert(aspeed_mctp->reset);
		reset_control_deassert(aspeed_mctp->reset);
	}

	aspeed_mctp->pcie_irq = platform_get_irq_byname(pdev, "pcie");
	if (aspeed_mctp->pcie_irq < 0) {
		dev_err(&pdev->dev, "no pcie reset irq specified\n");
		ret = -ENOENT;
		goto free;
	} else {
		ret = devm_request_irq(&pdev->dev, aspeed_mctp->pcie_irq, aspeed_pcie_raise_isr,
				       IRQF_SHARED, dev_name(&pdev->dev), aspeed_mctp);
		if (ret) {
			dev_err(&pdev->dev, "MCTP Unable to get pcie raise IRQ");
			goto free;
		}
	}

	if (reserved_idx == -1) {
		max_reserved_idx = of_alias_get_highest_id("mctp");
		if (max_reserved_idx >= 0)
			reserved_idx = max_reserved_idx;
	}

	idx = of_alias_get_id(pdev->dev.of_node, "mctp");
	if (idx < 0)
		idx = ++reserved_idx;

	aspeed_mctp->misc_dev.minor = MISC_DYNAMIC_MINOR;
	aspeed_mctp->misc_dev.name = kasprintf(GFP_KERNEL, "aspeed-mctp%d", idx);
	aspeed_mctp->misc_dev.fops = &aspeed_mctp_fops;

	ret = misc_register(&aspeed_mctp->misc_dev);

	if (ret)
		goto free;

	platform_set_drvdata(pdev, aspeed_mctp);

	aspeed_mctp_ctrl_init(aspeed_mctp);

	pr_info("aspeed_mctp: driver successfully loaded.\n");

	return 0;

free:
	dma_free_coherent(&pdev->dev, dma_size, &aspeed_mctp->tx_cmd_desc_dma, GFP_KERNEL);
out:
	pr_warn("aspeed_mctp: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int aspeed_mctp_remove(struct platform_device *pdev)
{
	struct aspeed_mctp_info *aspeed_mctp = platform_get_drvdata(pdev);

	MCTP_DBUG("\n");
	if (aspeed_mctp->mctp_version == ASPEED_MCTP_2600)
		dma_free_coherent(&pdev->dev, MCTP_G6_DMA_SIZE, aspeed_mctp->tx_cmd_desc, GFP_KERNEL);
	else
		dma_free_coherent(&pdev->dev, MCTP_DMA_SIZE, aspeed_mctp->tx_cmd_desc, GFP_KERNEL);

	misc_deregister(&aspeed_mctp->misc_dev);

	return 0;
}

static struct platform_driver aspeed_mctp_driver = {
	.probe = aspeed_mctp_probe,
	.remove = aspeed_mctp_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_mctp_of_matches,
	},
};

module_platform_driver(aspeed_mctp_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED MCTP Driver");
MODULE_LICENSE("GPL");
