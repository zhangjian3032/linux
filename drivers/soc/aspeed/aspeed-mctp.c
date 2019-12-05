/*
 * MCTP driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */
#include <linux/poll.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <asm/uaccess.h>
/*************************************************************************************/
#define MCTP_DESC_SIZE		4096	//for tx/rx descript
#define MCTP_TX_BUFF_SIZE	4096
#define MCTP_RX_BUFF_POOL_SIZE	16384
#define MCTP_TX_FIFO_NUM	1
#define MCTP_G6_TX_FIFO_NUM	4

#define MCTP_RX_DESC_BUFF_NUM	8

#define G4_DRAM_BASE_ADDR	0x40000000
#define G5_DRAM_BASE_ADDR	0x80000000
#define G6_DRAM_BASE_ADDR	0x80000000

/*************************************************************************************/
#define ASPEED_MCTP_CTRL 		0x00
#define  MCTP_GET_CUR_CMD_CNT(x)	((x >> 24) & 0x3f)
#define  MCTP_RX_PCIE_IDLE		BIT(21)
#define  MCTP_RX_DMA_IDLE		BIT(20)
#define  MCTP_TX_PCIE_IDLE		BIT(17)
#define  MCTP_TX_DMA_IDLE		BIT(16)
#define  MCTP_CPL2_ENABLE		BIT(15)
#define  MCTP_MATCH_EID			BIT(9)
#define  MCTP_RX_CMD_RDY		BIT(4)
#define  MCTP_TX_TRIGGER		BIT(0)
#define ASPEED_MCTP_TX_CMD		0x04
#define ASPEED_MCTP_RX_CMD		0x08
#define ASPEED_MCTP_ISR 		0x0c
#define ASPEED_MCTP_IER 		0x10
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
#define ASPEED_MCTP_EID 		0x14
#define ASPEED_MCTP_OBFF_CTRL 		0x18
/* aspeed g6 mctp */
#define ASPEED_MCTP_CTRL1 		0x1C
#define  MCTP_FIFO_FULL			BIT(19)
#define  MCTP_OBFF_DMA_ENABLE		BIT(18)
#define  MCTP_OBFF_MONITOR		BIT(17)
#define  MCTP_6KB_TX_2KB_RX_FIFO	(0 << 8)
#define  MCTP_4KB_TX_4KB_RX_FIFO	(1 << 8)
#define  MCTP_2KB_TX_6KB_RX_FIFO	(2 << 8)
#define  MCTP_RX_PAYLOAD_64BYTE		(0 << 4)
#define  MCTP_RX_PAYLOAD_128BYTE	(1 << 4)
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

/*************************************************************************************/
//TX CMD desc0 : ast-g4, ast-g5
#define BUS_NO(x)			((x & 0xff) << 24)
#define DEV_NO(x)			((x & 0x1f) << 19)
#define FUN_NO(x)			((x & 0x7) << 16)
#define INT_ENABLE			BIT(15)

//ast-g5
/* 0: route to RC, 1: route by ID, 2/3: broadcast from RC */
#define G5_ROUTING_TYPE_L(x)		((x & 0x1) << 14)
#define G5_ROUTING_TYPE_H(x)		(((x & 0x2) >> 1) << 12)
//ast old version
#define ROUTING_TYPE(x)			((x & 0x1) << 14)

#define TAG_OWN(x)			(x << 13)

//bit 12:2 is packet in 4bytes
//ast2400 bit 12 can be use.
//ast2500 bit 12 can't be used. 0: 1024 * 4 = 4096
#define G5_PKG_SIZE(x)		((x & 0x3ff) << 2)
#define PKG_SIZE(x)			((x & 0x7ff) << 2)

#define PADDING_LEN(x)			(x & 0x3)
//TX CMD desc1
#define LAST_CMD			BIT(31)
//ast-g5
#define G5_TX_DATA_ADDR(x)		(((x >> 7) & 0x7fffff) << 8)
//ast old version
#define TX_DATA_ADDR(x)			(((x >> 6) & 0x7fffff) << 8)

#define DEST_EP_ID(x)			(x & 0xff)

/*************************************************************************************/
//RX CMD desc0
#define GET_PKG_LEN(x)			(((x) >> 24) & 0x7f)
#define GET_SRC_EPID(x)			(((x) >> 16) & 0xff)
#define GET_ROUTING_TYPE(x)		((x >> 14) & 0x7)
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

/*************************************************************************************/
//pcie vdm header

//pcie vdm data

//ast-g6

/*************************************************************************************/
#define G6_TX_DATA_ADDR(x)		(((x >> 2) & 0x1fffffff) << 2)
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
//[16]: stop, [15]: Int, [12:2] paload size 4bytes, [0]: 0
//[31]: read only , [30:2]vdm address, resv, [1]: 1

//ast2400 rx cmd desc

//ast2500 rx cmd desc
//[31:24]:dest pcie bus#,[23:19]:dest pcie dev#, [18:16]:dest pcie fun# [15]: Int_en, [14] : Routing type 0: RC, 1 by ID,  [13]: MCTP tag owner, [12:2]: package size in 4 bytes, [1:0]: Padding length
//[30:24] paload size 4bytes, [23:16]: src EPID, [15:14]: routing type, [13]: tag owner ,[12:11]: mctp seq# ,[10:8]: mctp Tag, [7]: mctp SOM, [6]: mctp EOM, [5:1] reserved ,[0]: cmd updated

//ast2600 rx cmd desc

/*************************************************************************************/
#define ASPEED_MCTP_XFER_SIZE 4096

#define MCTPIOC_BASE 'M'

#define ASPEED_MCTP_IOCTX	_IOW(MCTPIOC_BASE, 0, struct aspeed_mctp_xfer*)
#define ASPEED_MCTP_IOCRX	_IOR(MCTPIOC_BASE, 1, struct aspeed_mctp_xfer*)

/*************************************************************************************/
struct pcie_vdm_header {
	__u32		length: 10,
			revd0: 2,
			attr: 2,
			ep: 1,
			td: 1,
			revd1: 4,
			tc: 3,
			revd2: 1,
			type_routing: 5,
			fmt: 2,
			revd3: 1;
	__u8		message_code;
	__u8		vdm_code: 4,
			pad_len: 2,
			tag_revd: 2;
	__u16		pcie_req_id;
	__u16		vender_id;
	__u16		pcie_target_id;
	__u8		msg_tag: 3,
			to: 1,
			pkt_seq: 2,
			eom: 1,
			som: 1;
	__u8		src_epid;
	__u8		dest_epid;
	__u8		header_ver: 4,
			rsvd: 4;
};

struct aspeed_mctp_xfer {
	unsigned char *xfer_buff;
	struct pcie_vdm_header header;
};
/*************************************************************************************/
//#define ASPEED_MCTP_DEBUG

#ifdef ASPEED_MCTP_DEBUG
#define MCTP_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define MCTP_DBUG(fmt, args...)
#endif

#define MCTP_MSG(fmt, args...) printk(fmt, ## args)

struct aspeed_mctp_info {
	void __iomem *reg_base;
	void __iomem *pci_bdf_regs;
	int irq;
	bool is_open;
	int pcie_irq;
	struct reset_control *reset;
	int	mctp_version;
	u32 dram_base;

	/* mctp tx info */
	struct completion tx_complete;

	int tx_fifo_num;
	int tx_idx;

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

	u32 rx_idx;
	u32 rx_hw_idx;
	u32 rx_full;
};

/******************************************************************************/

static inline u32
aspeed_mctp_read(struct aspeed_mctp_info *aspeed_mctp, u32 reg)
{
	u32 val;

	val = readl(aspeed_mctp->reg_base + reg);
//	MCTP_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
}

static inline void
aspeed_mctp_write(struct aspeed_mctp_info *aspeed_mctp, u32 val, u32 reg)
{
//	MCTP_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, aspeed_mctp->reg_base + reg);
}

/*************************************************************************************/
static int aspeed_mctp_tx_xfer(struct aspeed_mctp_info *aspeed_mctp, struct aspeed_mctp_xfer *mctp_xfer)
{
	struct pcie_vdm_header *vdm_header = &mctp_xfer->header;
	unsigned long byte_length = 0;
	u32 routing_type = vdm_header->type_routing;

	if (!vdm_header->length)
		byte_length = 4096 - vdm_header->pad_len;
	else
		byte_length = vdm_header->length * 4 - vdm_header->pad_len;

	//ast2500 noly support 4096, g5 is not supporting vdm_length is 1024
	if ((aspeed_mctp->mctp_version == 5) && (!mctp_xfer->header.length))
		return 1;

	MCTP_DBUG("xfer byte_length = %ld, padding len = %d\n", byte_length, vdm_header->pad_len);

	if ((aspeed_mctp->mctp_version == 0) || (aspeed_mctp->mctp_version == 5)) {
		if (copy_from_user(aspeed_mctp->tx_pool, mctp_xfer->xfer_buff, byte_length))
			return -EFAULT;

		//old ast2400/ast2500 only one tx fifo and wait for tx complete
		init_completion(&aspeed_mctp->tx_complete);

		//if use ast2400/ast2500 need to check vdm header support
		if (vdm_header->som != vdm_header->eom) {
			printk("can't support som eom different som %d , eom %d \n", vdm_header->som, vdm_header->eom);
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
			printk("not supported rounting type %x ", routing_type);
			break;
		}

	} else {
		//TODO check tx is full or not
		u32 hw_read_pt;
		aspeed_mctp_write(aspeed_mctp, MCTP_HW_READ_PT_UPDATE, ASPEED_MCTP_TX_READ_PT);
		hw_read_pt = aspeed_mctp_read(aspeed_mctp, ASPEED_MCTP_TX_READ_PT) & MCTP_HW_READ_PT_NUM_MASK;
		if (((aspeed_mctp->tx_idx + 1) % MCTP_G6_TX_FIFO_NUM) == aspeed_mctp_read(aspeed_mctp, ASPEED_MCTP_TX_READ_PT)) {
			printk("TX FIFO full \n");
			return 0;
		} else {
			void *cur_tx_buff = aspeed_mctp->tx_pool + (MCTP_TX_BUFF_SIZE * aspeed_mctp->tx_idx);

			//ast2600 support vdm header transfer
			mctp_xfer->header.pcie_req_id = (readl(aspeed_mctp->pci_bdf_regs + 0xc4) & 0x1fff) << 3;
			memcpy(cur_tx_buff, &mctp_xfer->header, sizeof(struct pcie_vdm_header));
			if (copy_from_user(cur_tx_buff + sizeof(struct pcie_vdm_header), mctp_xfer->xfer_buff, byte_length))
				return -EFAULT;
		}
	}

	switch (aspeed_mctp->mctp_version) {
	case 0:
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
		aspeed_mctp_write(aspeed_mctp, aspeed_mctp_read(aspeed_mctp, ASPEED_MCTP_IER) | MCTP_TX_LAST, ASPEED_MCTP_IER);
		break;
	case 5:
		//routing type [desc0 bit 12, desc0 bit 14], but bug at bit 12, don't use
		//bit 15 : interrupt enable
		aspeed_mctp->tx_cmd_desc->desc0 = INT_ENABLE | TAG_OWN(vdm_header->to) | (routing_type << 14) |
						  G5_PKG_SIZE(vdm_header->length) | (vdm_header->pcie_target_id << 16) |
						  PADDING_LEN(vdm_header->pad_len);
		aspeed_mctp->tx_cmd_desc->desc1 = LAST_CMD | DEST_EP_ID(vdm_header->dest_epid) | G5_TX_DATA_ADDR(aspeed_mctp->tx_pool_dma);
		aspeed_mctp_write(aspeed_mctp, aspeed_mctp_read(aspeed_mctp, ASPEED_MCTP_IER) | MCTP_TX_LAST, ASPEED_MCTP_IER);
		break;
	case 6:
		if (!vdm_header->length)
			aspeed_mctp->tx_cmd_desc[aspeed_mctp->tx_idx].desc0 = PKG_SIZE(0x400);
		else
			aspeed_mctp->tx_cmd_desc[aspeed_mctp->tx_idx].desc0 = PKG_SIZE(vdm_header->length);
#if 0
		//add TEST for interrupt and stop
		aspeed_mctp->tx_cmd_desc[aspeed_mctp->tx_idx].desc0 |= BIT(15) | BIT(16);
		//
		MCTP_DBUG("tx idx [%d] desc0 %x , desc1 %x \n", aspeed_mctp->tx_idx,
			  aspeed_mctp->tx_cmd_desc[aspeed_mctp->tx_idx].desc0,
			  aspeed_mctp->tx_cmd_desc[aspeed_mctp->tx_idx].desc1);
#endif
		//trigger write pt;
		aspeed_mctp->tx_idx++;
		aspeed_mctp->tx_idx %= MCTP_G6_TX_FIFO_NUM;
		aspeed_mctp_write(aspeed_mctp, aspeed_mctp->tx_idx, ASPEED_MCTP_TX_WRITE_PT);
		break;
	}

	//trigger tx
	aspeed_mctp_write(aspeed_mctp, aspeed_mctp_read(aspeed_mctp, ASPEED_MCTP_CTRL) | MCTP_TX_TRIGGER, ASPEED_MCTP_CTRL);

	if ((aspeed_mctp->mctp_version == 0) || (aspeed_mctp->mctp_version == 5)) {
		wait_for_completion(&aspeed_mctp->tx_complete);
	}

	return 0;
}

static void aspeed_mctp_ctrl_init(struct aspeed_mctp_info *aspeed_mctp)
{
	int i = 0;

	MCTP_DBUG("dram base %x \n", aspeed_mctp->dram_base);
	aspeed_mctp_write(aspeed_mctp, aspeed_mctp->dram_base, ASPEED_MCTP_EID);

	aspeed_mctp->tx_idx = 0;

	if (aspeed_mctp->mctp_version == 6) {
		for (i = 0; i < aspeed_mctp->tx_fifo_num; i++)
			aspeed_mctp->tx_cmd_desc[i].desc1 = ((aspeed_mctp->tx_pool_dma + (MCTP_TX_BUFF_SIZE * i)) & 0x7ffffff0) | 0x1;
		aspeed_mctp->tx_cmd_desc[aspeed_mctp->tx_fifo_num - 1].desc1 |= LAST_CMD;
		aspeed_mctp_write(aspeed_mctp, aspeed_mctp->tx_cmd_desc_dma, ASPEED_MCTP_TX_DESC_ADDR);
		aspeed_mctp_write(aspeed_mctp, aspeed_mctp->tx_fifo_num, ASPEED_MCTP_TX_DESC_NUM);
		aspeed_mctp_write(aspeed_mctp, 0, ASPEED_MCTP_TX_WRITE_PT);
	} else {
		aspeed_mctp_write(aspeed_mctp, aspeed_mctp->tx_cmd_desc_dma, ASPEED_MCTP_TX_CMD);
	}

	aspeed_mctp->rx_idx = 0;
	aspeed_mctp->rx_hw_idx = 0;

	//rx fifo data
	if (aspeed_mctp->mctp_version == 6) {
		//ast2600 : each 16 bytes align and configurable 64/128/256/512 bytes can recevice
		u32 *rx_cmd_desc = aspeed_mctp->rx_cmd_desc;
		for (i = 0; i < aspeed_mctp->rx_fifo_num; i++) {
			rx_cmd_desc[i] = (u32)aspeed_mctp->rx_pool_dma + (aspeed_mctp->rx_fifo_size * i);
			MCTP_DBUG("Rx [%d]: desc: %x , \n", i, rx_cmd_desc[i]);
		}
		aspeed_mctp_write(aspeed_mctp, aspeed_mctp->rx_cmd_desc_dma, ASPEED_MCTP_RX_DESC_ADDR);
		aspeed_mctp_write(aspeed_mctp, aspeed_mctp->rx_fifo_num, ASPEED_MCTP_RX_DESC_NUM);
		aspeed_mctp_write(aspeed_mctp, 0, ASPEED_MCTP_RX_READ_PT);
		aspeed_mctp_write(aspeed_mctp, MCTP_TX_CMD_WRONG | MCTP_RX_NO_CMD, ASPEED_MCTP_IER);
	} else {
		//ast2400/ast2500 : each 128 bytes align, and only 64 bytes can recevice
		struct aspeed_mctp_cmd_desc *rx_cmd_desc = aspeed_mctp->rx_cmd_desc;
		for (i = 0; i < aspeed_mctp->rx_fifo_num; i++) {
			rx_cmd_desc[i].desc0 = 0;
			rx_cmd_desc[i].desc1 = RX_DATA_ADDR(aspeed_mctp->rx_pool_dma + (aspeed_mctp->rx_fifo_size * i));
			if (i == (aspeed_mctp->rx_fifo_num - 1))
				rx_cmd_desc[i].desc1 |= LAST_CMD;
			MCTP_DBUG("Rx [%d]: desc0: %x , desc1: %x \n", i, rx_cmd_desc[i].desc0, rx_cmd_desc[i].desc1);
		}
		aspeed_mctp_write(aspeed_mctp, aspeed_mctp->rx_cmd_desc_dma, ASPEED_MCTP_RX_CMD);
		aspeed_mctp_write(aspeed_mctp, MCTP_RX_COMPLETE | MCTP_RX_NO_CMD, ASPEED_MCTP_IER);
	}

	aspeed_mctp_write(aspeed_mctp, aspeed_mctp_read(aspeed_mctp, ASPEED_MCTP_CTRL) | MCTP_RX_CMD_RDY, ASPEED_MCTP_CTRL);
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
	u32 status = aspeed_mctp_read(aspeed_mctp, ASPEED_MCTP_ISR);

	MCTP_DBUG("%x \n", status);

	if (status & MCTP_TX_LAST) {
		//only for ast2400/ast2500, ast2600 is tx fifo
		aspeed_mctp_write(aspeed_mctp, MCTP_TX_LAST, ASPEED_MCTP_ISR);
		complete(&aspeed_mctp->tx_complete);
	}

	if (status & MCTP_TX_COMPLETE) {
		aspeed_mctp_write(aspeed_mctp, MCTP_TX_COMPLETE, ASPEED_MCTP_ISR);
		printk("don't care don't use \n");
	}

	if (status & MCTP_RX_COMPLETE) {
		if (aspeed_mctp->mctp_version == 6) {
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
		aspeed_mctp_write(aspeed_mctp, MCTP_RX_COMPLETE, ASPEED_MCTP_ISR);
	}

	if (status & MCTP_RX_NO_CMD) {
		aspeed_mctp->rx_full = 1;
		aspeed_mctp_write(aspeed_mctp, MCTP_RX_NO_CMD, ASPEED_MCTP_ISR);
		printk("MCTP_RX_NO_CMD \n");
	}

	return IRQ_HANDLED;
}

static long mctp_ioctl(struct file *file, unsigned int cmd,
		       unsigned long arg)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_mctp_info *aspeed_mctp = dev_get_drvdata(c->this_device);
	struct aspeed_mctp_xfer mctp_xfer;
	void __user *argp = (void __user *)arg;
	int recv_length;

	if (copy_from_user(&mctp_xfer, argp, sizeof(struct aspeed_mctp_xfer)))
		return -EFAULT;

	switch (cmd) {
	case ASPEED_MCTP_IOCTX:
		MCTP_DBUG("ASPEED_MCTP_IOCTX ver %d\n", aspeed_mctp->mctp_version);
		if (aspeed_mctp->mctp_version != 0) {
			if ((readl(aspeed_mctp->pci_bdf_regs + 0xc4) & 0x1fff) == 0) {
				printk("PCIE not ready \n");
				return -EFAULT;
			}
		}

		if (aspeed_mctp_tx_xfer(aspeed_mctp, &mctp_xfer))
			return -EFAULT;
		else
			return 0;

		break;
	case ASPEED_MCTP_IOCRX:
		// MCTP_DBUG("ASPEED_MCTP_IOCRX \n");
		if (aspeed_mctp->mctp_version == 6) {
			u32 hw_read_pt;
			aspeed_mctp_write(aspeed_mctp, MCTP_HW_READ_PT_UPDATE, ASPEED_MCTP_RX_WRITE_PT);
			hw_read_pt = aspeed_mctp_read(aspeed_mctp, ASPEED_MCTP_RX_WRITE_PT) & MCTP_HW_READ_PT_NUM_MASK;

			if (aspeed_mctp->rx_idx == hw_read_pt) {
				MCTP_DBUG("No rx data\n");
				return 0;
			} else {
				struct pcie_vdm_header *vdm = aspeed_mctp->rx_pool + (aspeed_mctp->rx_fifo_size * aspeed_mctp->rx_idx);
				u32 *rx_buffer = aspeed_mctp->rx_pool + (aspeed_mctp->rx_fifo_size * aspeed_mctp->rx_idx) + sizeof(struct pcie_vdm_header);
				// struct aspeed_mctp_xfer *mctp_rx_xfer = (struct aspeed_mctp_xfer *)arg;
				recv_length = (vdm->length * 4) + vdm->pad_len;
				mctp_xfer.header = *vdm;
				// if (copy_to_user(&mctp_xfer.header, vdm, sizeof(struct pcie_vdm_header)))
				// 	return -EFAULT;
				// else {
				if (copy_to_user(mctp_xfer.xfer_buff, rx_buffer, recv_length))
					return -EFAULT;
				// }
				if (copy_to_user(argp, &mctp_xfer, sizeof(struct aspeed_mctp_xfer)))
					return -EFAULT;

				aspeed_mctp->rx_idx++;
				aspeed_mctp->rx_idx %= aspeed_mctp->rx_fifo_num;
				aspeed_mctp_write(aspeed_mctp, aspeed_mctp->rx_idx, ASPEED_MCTP_RX_READ_PT);
			}
		} else {
			struct aspeed_mctp_cmd_desc *rx_cmd_desc = aspeed_mctp->rx_cmd_desc;
			u32 desc0 = rx_cmd_desc[aspeed_mctp->rx_idx].desc0;
			unsigned int pci_bdf;

			if ((aspeed_mctp->rx_idx == aspeed_mctp->rx_hw_idx) && (!desc0)) {
				if (aspeed_mctp->rx_full) {
					MCTP_DBUG("re-trigger\n");
					aspeed_mctp_ctrl_init(aspeed_mctp);
					aspeed_mctp->rx_full = 0;
				}
				return 0;
			}

			if (!desc0)
				return 0;

			mctp_xfer.header.length = GET_PKG_LEN(desc0);
			MCTP_DBUG("mctp_xfer.header.length %d \n", mctp_xfer.header.length);

			if (mctp_xfer.header.length != 0 && mctp_xfer.header.length < GET_PKG_LEN(desc0))
				return -EINVAL;

			mctp_xfer.header.pad_len = GET_PADDING_LEN(desc0);
			mctp_xfer.header.src_epid = GET_SRC_EPID(desc0);
			mctp_xfer.header.type_routing = GET_ROUTING_TYPE(desc0);
			mctp_xfer.header.pkt_seq = GET_SEQ_NO(desc0);
			mctp_xfer.header.msg_tag = GET_MSG_TAG(desc0);
			mctp_xfer.header.eom = GET_MCTP_EOM(desc0);
			mctp_xfer.header.som = GET_MCTP_SOM(desc0);
			// 0x1e6ed0c4[4:0]: Dev#
			// 0x1e6ed0c4[12:5]: Bus#
			// Fun# always 0
			if (aspeed_mctp->mctp_version != 0) {
				pci_bdf = readl(aspeed_mctp->pci_bdf_regs + 0xc4);
				mctp_xfer.header.pcie_target_id = (pci_bdf & 0x1f) << 3 |
								  (pci_bdf >> 5 & 0xff) << 8;
			}
			recv_length = (mctp_xfer.header.length * 4);

			if (copy_to_user(mctp_xfer.xfer_buff,
					 aspeed_mctp->rx_pool + (aspeed_mctp->rx_fifo_size * aspeed_mctp->rx_idx),
					 recv_length)) {
				return -EFAULT;
			}

			if (copy_to_user(argp, &mctp_xfer, sizeof(struct aspeed_mctp_xfer)))
				return -EFAULT;

			rx_cmd_desc[aspeed_mctp->rx_idx].desc0 = 0;
			aspeed_mctp->rx_idx++;
			aspeed_mctp->rx_idx %= aspeed_mctp->rx_fifo_num;

		}
		break;

	default:
		MCTP_DBUG("ERROR \n");
		return -ENOTTY;
	}

	return 0;
}

static int mctp_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_mctp_info *aspeed_mctp = dev_get_drvdata(c->this_device);

	MCTP_DBUG("\n");
	if (aspeed_mctp->is_open) {
		return -EBUSY;
	}

	aspeed_mctp->is_open = true;
	return 0;
}

static int mctp_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_mctp_info *aspeed_mctp = dev_get_drvdata(c->this_device);

	MCTP_DBUG("\n");
	aspeed_mctp->is_open = false;
	return 0;
}

static const struct file_operations aspeed_mctp_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= mctp_ioctl,
	.open		= mctp_open,
	.release	= mctp_release,
};

static struct miscdevice aspeed_mctp_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aspeed-mctp",
	.fops = &aspeed_mctp_fops,
};

static const struct of_device_id aspeed_mctp_of_matches[] = {
	{ .compatible = "aspeed,ast2400-mctp", .data = (void *) 0, },
	{ .compatible = "aspeed,ast2500-mctp", .data = (void *) 5, },
	{ .compatible = "aspeed,ast2600-mctp", .data = (void *) 6, },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_mctp_of_matches);
static int aspeed_mctp_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct aspeed_mctp_info *aspeed_mctp;
	const struct of_device_id *mctp_dev_id;
	int ret = 0;

	MCTP_DBUG("\n");

	if (!(aspeed_mctp = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_mctp_info), GFP_KERNEL))) {
		return -ENOMEM;
	}

	mctp_dev_id = of_match_device(aspeed_mctp_of_matches, &pdev->dev);
	if (!mctp_dev_id)
		return -EINVAL;

	aspeed_mctp->mctp_version = (unsigned long)mctp_dev_id->data;
	switch (aspeed_mctp->mctp_version) {
	case 0:
		aspeed_mctp->tx_fifo_num = MCTP_TX_FIFO_NUM;
		aspeed_mctp->rx_fifo_size = 128;
		aspeed_mctp->rx_fifo_num = MCTP_RX_BUFF_POOL_SIZE / aspeed_mctp->rx_fifo_size;
		aspeed_mctp->dram_base = G4_DRAM_BASE_ADDR;
		break;
	case 5:
		aspeed_mctp->tx_fifo_num = MCTP_TX_FIFO_NUM;
		aspeed_mctp->rx_fifo_size = 128;
		aspeed_mctp->rx_fifo_num = MCTP_RX_BUFF_POOL_SIZE / aspeed_mctp->rx_fifo_size;
		aspeed_mctp->dram_base = G5_DRAM_BASE_ADDR;
		break;
	case 6:
		aspeed_mctp->tx_fifo_num = MCTP_G6_TX_FIFO_NUM;
		//must 16byte align
		aspeed_mctp->rx_fifo_size = (64 + 16);	//TODO AST2600 tx/rx unit can be configurate by register
		aspeed_mctp->rx_fifo_num = MCTP_RX_BUFF_POOL_SIZE / aspeed_mctp->rx_fifo_size;
		aspeed_mctp->dram_base = G6_DRAM_BASE_ADDR;
		break;
	default:
		dev_err(&pdev->dev, "cannot get mctp version\n");
		goto out;
		break;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	aspeed_mctp->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!aspeed_mctp->reg_base) {
		ret = -EIO;
		goto out_region;
	}
	// g4 not support pcie host controller
	if (aspeed_mctp->mctp_version != 0) {
		struct resource pcie_res;
		struct device_node *pcie_np;

		pcie_np = of_find_compatible_node(NULL, NULL, "aspeed,aspeed-pcie");

		if (of_address_to_resource(pcie_np, 0, &pcie_res)) {
			dev_err(&pdev->dev, "failed to find pcie resouce\n");
			ret = -ENOMEM;
			goto out_region;
		}
		aspeed_mctp->pci_bdf_regs = ioremap(pcie_res.start, resource_size(&pcie_res));
		if (!aspeed_mctp->pci_bdf_regs) {
			dev_err(&pdev->dev, "failed to remap pcie\n");
			ret = -ENOMEM;
			goto out_region;
		}
	}

	aspeed_mctp->irq = platform_get_irq(pdev, 0);
	if (aspeed_mctp->irq < 0) {
		printk("no irq \n");
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	aspeed_mctp->pcie_irq = platform_get_irq(pdev, 1);
	if (aspeed_mctp->pcie_irq < 0) {
		dev_err(&pdev->dev, "no pcie reset irq specified\n");
	}

	aspeed_mctp->reset = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(aspeed_mctp->reset)) {
		dev_err(&pdev->dev, "can't get mctp reset\n");
		return PTR_ERR(aspeed_mctp->reset);
	}

//scu init
	reset_control_assert(aspeed_mctp->reset);
	reset_control_deassert(aspeed_mctp->reset);

	ret = misc_register(&aspeed_mctp_misc);
	if (ret) {
		printk(KERN_ERR "MCTP : failed to request interrupt\n");
		goto out_region;
	}

	platform_set_drvdata(pdev, aspeed_mctp);
	dev_set_drvdata(aspeed_mctp_misc.this_device, aspeed_mctp);

	aspeed_mctp->tx_idx = 0;

//tx desc allocate --> tx desc : 0~2048, rx desc : 2048 ~ 4096
	aspeed_mctp->tx_cmd_desc = dma_alloc_coherent(NULL,
				   MCTP_DESC_SIZE,
				   &aspeed_mctp->tx_cmd_desc_dma, GFP_KERNEL);

//tx buff pool init
//ast2400/ast2500 : 128 bytes aligned,
//ast2600 : 16 bytes aligned,
	aspeed_mctp->tx_pool = dma_alloc_coherent(NULL,
			       MCTP_TX_BUFF_SIZE * aspeed_mctp->tx_fifo_num,
			       &aspeed_mctp->tx_pool_dma, GFP_KERNEL);

//rx desc allocate : 2048 ~ 4096
	aspeed_mctp->rx_cmd_desc = (void *)aspeed_mctp->tx_cmd_desc + 2048;
	aspeed_mctp->rx_cmd_desc_dma = aspeed_mctp->tx_cmd_desc_dma + 2048;

//rx buff pool init :
//ast2400/ast2500, data address [29:7]: 0x00 , 0x80 , 0x100, 0x180,
//ast2600, data address [30:4]: 0x00 , 0x10 , 0x20, 0x30,
	aspeed_mctp->rx_pool = dma_alloc_coherent(NULL,
			       MCTP_RX_BUFF_POOL_SIZE,
			       &aspeed_mctp->rx_pool_dma, GFP_KERNEL);

	aspeed_mctp_ctrl_init(aspeed_mctp);

	ret = devm_request_irq(&pdev->dev, aspeed_mctp->irq, aspeed_mctp_isr,
			       0, dev_name(&pdev->dev), aspeed_mctp);
	if (ret) {
		printk("MCTP Unable to get IRQ");
		goto out_irq;
	}

	ret = devm_request_irq(&pdev->dev, aspeed_mctp->pcie_irq, aspeed_pcie_raise_isr,
			       IRQF_SHARED, dev_name(&pdev->dev), aspeed_mctp);
	if (ret) {
		printk("MCTP Unable to get pcie raise IRQ");
		goto out_irq;
	}

	printk(KERN_INFO "aspeed_mctp: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(aspeed_mctp->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "aspeed_mctp: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int aspeed_mctp_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct aspeed_mctp_info *aspeed_mctp = platform_get_drvdata(pdev);

	MCTP_DBUG("\n");

	misc_deregister(&aspeed_mctp_misc);

	free_irq(aspeed_mctp->irq, aspeed_mctp);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(aspeed_mctp->reg_base);

	if (aspeed_mctp->mctp_version != 0) {
		iounmap(aspeed_mctp->pci_bdf_regs);
	}

	platform_set_drvdata(pdev, NULL);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

#ifdef CONFIG_PM
static int
aspeed_mctp_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int
aspeed_mctp_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define aspeed_mctp_suspend        NULL
#define aspeed_mctp_resume         NULL
#endif

static struct platform_driver aspeed_mctp_driver = {
	.probe 		= aspeed_mctp_probe,
	.remove 		= aspeed_mctp_remove,
#ifdef CONFIG_PM
	.suspend        = aspeed_mctp_suspend,
	.resume         = aspeed_mctp_resume,
#endif
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = aspeed_mctp_of_matches,
	},
};

module_platform_driver(aspeed_mctp_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED MCTP Driver");
MODULE_LICENSE("GPL");
