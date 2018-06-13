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
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/reset.h>
#include <linux/slab.h>


#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <asm/uaccess.h>
/*************************************************************************************/
#define MCTP_DESC_SIZE		4096	//for tx/rx descript
#define MCTP_BUFF_SIZE		4096
#define MCTP_TX_FIFO_NUM	1
#define MCTP_G6_TX_FIFO_NUM	4

#define MCTP_RX_DESC_BUFF_NUM	8
#define MCTP_RX_FIFO_NUM	16
#define MCTP_G6_RX_FIFO_NUM	16

/*************************************************************************************/
#define ASPEED_MCTP_CTRL 		0x00
#define  MCTP_GET_CUR_CMD_CNT(x)	((x >> 24) & 0x3f)
#define  MCTP_RX_PCIE_IDLE		BIT(21)
#define  MCTP_RX_DMA_IDLE		BIT(20)
#define  MCTP_TX_PCIE_IDLE		BIT(17)
#define  MCTP_TX_DMA_IDLE		BIT(16)
#define  MCTP_CPL2_ENABLE		BIT(15)
#define  MCTP_MATCH_EID			BIT(9)
#define  MCTP_RX_CMD_RDY			BIT(4)
#define  MCTP_TX_TRIGGER			BIT(0)
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
#define  MCTP_6KB_TX_2KB_RX_FIFO		(0 << 8)
#define  MCTP_4KB_TX_4KB_RX_FIFO		(1 << 8)
#define  MCTP_2KB_TX_6KB_RX_FIFO		(2 << 8)
#define  MCTP_RX_PAYLOAD_64BYTE		(0 << 4)
#define  MCTP_RX_PAYLOAD_128BYTE		(1 << 4)
#define  MCTP_RX_PAYLOAD_256BYTE		(2 << 4)
#define  MCTP_RX_PAYLOAD_512BYTE		(3 << 4)
#define  MCTP_TX_PAYLOAD_64BYTE		(0)
#define  MCTP_TX_PAYLOAD_128BYTE		(1)
#define  MCTP_TX_PAYLOAD_256BYTE		(2)
#define  MCTP_TX_PAYLOAD_512BYTE		(3)
#define ASPEED_MCTP_RXBUFF_ADDR		0x20
#define ASPEED_MCTP_RXBUFF_SIZE		0x24
#define ASPEED_MCTP_WRITE_POINT		0x28
#define ASPEED_MCTP_READ_POINT		0x2C
/* ast2600 mctp use tx cmd descript */
#define ASPEED_MCTP_TXBUFF_ADDR			0x30
#define ASPEED_MCTP_TX_DESC_NUM		0x34
#define ASPEED_MCTP_TX_WRITE_PT		0x38
#define ASPEED_MCTP_TX_READ_PT		0x3C

/*************************************************************************************/
//TX CMD desc0 : ast-g4, ast-g5 
#define BUS_NO(x)				((x & 0xff) << 24)
#define DEV_NO(x)				((x & 0x1f) << 19)
#define FUN_NO(x)				((x & 0x7) << 16)
#define STOP_INT_ENABLE			BIT(15)

//ast-g5
/* 0: route to RC, 1: route by ID, 2/3: broadcast from RC */
#define G5_ROUTING_TYPE_L(x)			((x & 0x1) << 14)
#define G5_ROUTING_TYPE_H(x)			(((x & 0x2) >> 1) << 12)
//ast old version
#define ROUTING_TYPE(x)			((x & 0x1) << 14)

#define TAG_OWN					BIT(13)

//bit 12:2 is packet in 4bytes
//ast2400 bit 12 can be use. 
//ast2500 bit 12 can't be used. 0: 1024 * 4 = 4096
#define PKG_SIZE(x)				((x & 0x3ff) << 2) 
#define PADDING_LEN(x)			(x & 0x3)
//TX CMD desc1
#define LAST_CMD				BIT(31)
//ast-g5
#define G5_TX_DATA_ADDR(x)		(((x >> 7) & 0x7fffff) << 8)
//ast old version
#define TX_DATA_ADDR(x)			(((x >> 6) & 0x7fffff) << 8)

#define DEST_EP_ID(x)			(x & 0xff)

/*************************************************************************************/
//RX CMD desc0
#define GET_PKG_LEN(x)			((x >> 24) & 0x7f)
#define GET_SRC_EP_ID(x)		((x >> 16) & 0xff)
#define GET_ROUTING_TYPE(x)	((x >> 14) & 0x7)
#define GET_SEQ_NO(x)			((x >> 11) & 0x3)
#define MCTP_SOM				(1 << 7)
#define MCTP_EOM				(1 << 6)
#define GET_PADDING_LEN(x)		((x >> 4) & 0x3)
#define CMD_UPDATE				BIT(0)
//RX CMD desc1
#define LAST_CMD				BIT(31)
#define RX_DATA_ADDR(x)			(((x >> 7) & 0x3fffff) << 7)

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

/*************************************************************************************/
#define ASPEED_MCTP_XFER_SIZE 4096

#if 0
struct aspeed_mctp_xfer {
	unsigned char	*xfer_buff;
	unsigned int xfer_len;
	unsigned int ep_id;
	unsigned int bus_no;
	unsigned int dev_no;
	unsigned int fun_no;
	unsigned char rt;
};
#endif

struct aspeed_mctp_xfer {
	unsigned char	*xfer_buff;
	unsigned int xfer_len;
	int wait_for_tx_complete;	//wait for interrupt
};

#define MCTPIOC_BASE       'M'


#define ASPEED_MCTP_IOCTX			_IO(MCTPIOC_BASE, 0x00)
#define ASPEED_MCTP_IOCRX			_IO(MCTPIOC_BASE, 0x01)

/*************************************************************************************/
struct pcie_vdm_header {
	__u32		length:10,
				revd0:2,
				attr:2,
				ep:1,
				td:1,
				revd1:4,
				tc:3,
				revd2:1,
				type_routing:5,
				fmt:2,
				revd3:1;
	__u8		message_code;
	__u8		vdm_code:4,
				pad_len:2,
				tag_revd:2;
	__u16		pcie_req_id;
	__u16		vender_id;
	__u16		pcie_target_id;
	__u8		msg_tag:3,
				to:1,
				pkt_seq:2,
				eom:1,
				som:1;
	__u8		src_epid;
	__u8		dest_epid;
	__u8		header_ver:4,
				rsvd:4;
};
/*************************************************************************************/
#define ASPEED_MCTP_DEBUG

#ifdef ASPEED_MCTP_DEBUG
#define MCTP_DBUG(fmt, args...) printk("%s() " fmt,__FUNCTION__, ## args)
#else
#define MCTP_DBUG(fmt, args...)
#endif

#define MCTP_MSG(fmt, args...) printk(fmt, ## args)

enum {
	RX_FIFO_FREE = 0,
	RX_FIFO_DESC,
	RX_FIFO_HANDLE,
	RX_FIFO_READY,
};

struct mctp_list {
	u8	fifo_state;	//0: free, 1:use in dma, 2:prepare for merge handle, 3:fifo ready for take
	void *rx_buff;
	dma_addr_t rx_buff_dma;
	u16 rx_buff_size;	//if fifo state is 2, size is meanfull
};

struct aspeed_mctp_info {
	void __iomem	*reg_base;
	int 	irq;	
	int 	pcie_irq;
	struct reset_control *reset;
	int	mctp_version;
	u32 dram_base;
	struct tasklet_struct	mctp_rx_tasklet;
//
	struct mctp_list *rx_list_mem;

//
	struct tasklet_struct	crypto_tasklet;

	wait_queue_head_t mctp_wq;

///////////////////////

	int tx_fifo_size;	
	int tx_idx;

	void *tx_pool;	
	dma_addr_t tx_pool_dma;

	struct aspeed_mctp_cmd_desc *tx_cmd_desc;
	dma_addr_t tx_cmd_desc_dma;

	void *cur_tx_buff;
	dma_addr_t cur_tx_buff_dma;

///////////////////////

	void *rx_pool;	
	dma_addr_t rx_pool_dma;
///

	int rx_dma_pool_size;
	int rx_fifo_size;	


	void *rx_cmd_desc;
	dma_addr_t rx_cmd_desc_dma;

	u32 rx_idx;
	u8 *rx_fifo;
	u8 rx_fifo_index;
	u32 rx_fifo_done;

	u32 flag;
	bool is_open;
	u32 state;
	//rx
	u32 rx_len;
	u8 ep_id;
	u8 rt;
	u8 seq_no;
};

/******************************************************************************/
static DEFINE_SPINLOCK(mctp_state_lock);

/******************************************************************************/

static inline u32
aspeed_mctp_read(struct aspeed_mctp_info *aspeed_mctp, u32 reg)
{
	u32 val;

	val = readl(aspeed_mctp->reg_base + reg);
	MCTP_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
}

static inline void
aspeed_mctp_write(struct aspeed_mctp_info *aspeed_mctp, u32 val, u32 reg)
{
	MCTP_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);

	writel(val, aspeed_mctp->reg_base + reg);
}

/*************************************************************************************/
static void aspeed_mctp_wait_tx_complete(struct aspeed_mctp_info *aspeed_mctp)
{
	wait_event_interruptible(aspeed_mctp->mctp_wq, (aspeed_mctp->flag == MCTP_TX_LAST));
	MCTP_DBUG("\n");
	aspeed_mctp->flag = 0;
}

static void aspeed_mctp_wait_rx_complete(struct aspeed_mctp_info *aspeed_mctp)
{
	wait_event_interruptible(aspeed_mctp->mctp_wq, (aspeed_mctp->flag == MCTP_EOM));
	MCTP_DBUG("\n");
	aspeed_mctp->flag = 0;
}

static void aspeed_mctp_tx_xfer(struct aspeed_mctp_info *aspeed_mctp, struct aspeed_mctp_xfer *mctp_xfer)
{
	u32 packet_size = (mctp_xfer->xfer_len / 4);
	u32 padding_len;
	struct pcie_vdm_header *vdm_header = (struct pcie_vdm_header *) mctp_xfer->xfer_buff;

	if ((mctp_xfer->xfer_len % 4)) {
		packet_size++;
		padding_len = 4 - ((mctp_xfer->xfer_len) % 4);
	} else {
		padding_len = 0;
	}

	MCTP_DBUG("xfer_len = %d, padding len = %d , 4byte align packet size %d\n", mctp_xfer->xfer_len, padding_len, packet_size);

	switch(aspeed_mctp->mctp_version) {
		case 0:
			//routing type bit 14
			//bit 15 : interrupt enable
			//set default tag owner = 1;
			mctp_xfer->wait_for_tx_complete = 1;
			aspeed_mctp->tx_cmd_desc->desc0 = 0x0000a000 | ROUTING_TYPE(vdm_header->type_routing) | PKG_SIZE(packet_size) | (vdm_header->pcie_target_id <<16) | PADDING_LEN(padding_len);
			aspeed_mctp->tx_cmd_desc->desc1 = LAST_CMD | DEST_EP_ID(vdm_header->dest_epid) | TX_DATA_ADDR(aspeed_mctp->cur_tx_buff_dma);
			break;
		case 5:
			//routing type [desc0 bit 12, desc0 bit 14]
			//bit 15 : interrupt enable
			//set default tag owner = 1;
			mctp_xfer->wait_for_tx_complete = 1;
			if(mctp_xfer->xfer_len == 4096) packet_size = 0;
			aspeed_mctp->tx_cmd_desc->desc0 = 0x0000a000 | G5_ROUTING_TYPE_H(vdm_header->type_routing) | G5_ROUTING_TYPE_L(vdm_header->type_routing) | PKG_SIZE(packet_size) | (vdm_header->pcie_target_id <<16) | PADDING_LEN(padding_len);
			aspeed_mctp->tx_cmd_desc->desc1 = LAST_CMD | DEST_EP_ID(vdm_header->dest_epid) | G5_TX_DATA_ADDR(aspeed_mctp->cur_tx_buff_dma);
			break;
		case 6:
 			//bit 15 : interrupt enable
			//set default tag owner = 1;
			if(mctp_xfer->wait_for_tx_complete) 
				aspeed_mctp->tx_cmd_desc[aspeed_mctp->tx_idx].desc0 = 0x00018000 | PKG_SIZE(packet_size);
			else
				aspeed_mctp->tx_cmd_desc[aspeed_mctp->tx_idx].desc0 = 0x00000000 | PKG_SIZE(packet_size);
			aspeed_mctp->tx_cmd_desc[aspeed_mctp->tx_idx].desc1 = 0x00000001 | G6_TX_DATA_ADDR(aspeed_mctp->cur_tx_buff_dma);
			//trigger write pt;
			aspeed_mctp->tx_idx++;
			aspeed_mctp->tx_idx %= MCTP_G6_TX_FIFO_NUM;

			aspeed_mctp->cur_tx_buff = aspeed_mctp->tx_pool + (aspeed_mctp->tx_idx * MCTP_BUFF_SIZE);
			aspeed_mctp->cur_tx_buff_dma = aspeed_mctp->tx_pool_dma + (aspeed_mctp->tx_idx * MCTP_BUFF_SIZE);
			aspeed_mctp_write(aspeed_mctp, aspeed_mctp->tx_idx, ASPEED_MCTP_TX_READ_PT);
			break;
	}

	//trigger tx
	aspeed_mctp_write(aspeed_mctp, aspeed_mctp_read(aspeed_mctp, ASPEED_MCTP_CTRL) | MCTP_TX_TRIGGER, ASPEED_MCTP_CTRL);

	//wait intr
	if(mctp_xfer->wait_for_tx_complete)
		aspeed_mctp_wait_tx_complete(aspeed_mctp);
	
}

static void aspeed_g6_mctp_tasklet(unsigned long data)
{
	struct aspeed_mctp_info *aspeed_mctp = (struct aspeed_mctp_info *)data;
	u32 *rx_cmd_desc = aspeed_mctp->rx_cmd_desc;
	u32 rx_len = 0;
	u32 *rx_buff;
	u32 padding_len = 0;
	int i = 0;
	MCTP_DBUG(" \n");
	for (i = 0; i < MCTP_RX_DESC_BUFF_NUM; i++) {
		aspeed_mctp->rx_idx %= MCTP_RX_DESC_BUFF_NUM;
		
		MCTP_DBUG("[%d] buff_addr %x \n", aspeed_mctp->rx_idx, rx_cmd_desc[aspeed_mctp->rx_idx]);
		rx_buff = rx_cmd_desc[aspeed_mctp->rx_idx] & 0x8000000f;
		aspeed_mctp->rx_idx++;
		if(aspeed_mctp->rx_idx == aspeed_mctp_read(aspeed_mctp, ASPEED_MCTP_READ_POINT))
			break;
	}
	aspeed_mctp_write(aspeed_mctp, aspeed_mctp->rx_idx - 1, ASPEED_MCTP_WRITE_POINT);
}

static void aspeed_mctp_tasklet(unsigned long data)
{
	struct aspeed_mctp_info *aspeed_mctp = (struct aspeed_mctp_info *)data;
	struct aspeed_mctp_cmd_desc *rx_cmd_desc = aspeed_mctp->rx_cmd_desc;
	int i = 0;
	u32 rx_len = 0;
	u32 padding_len = 0;


	MCTP_DBUG(" \n");

	for (i = 0; i < MCTP_RX_DESC_BUFF_NUM; i++) {
		aspeed_mctp->rx_idx %= MCTP_RX_DESC_BUFF_NUM;
		MCTP_DBUG("index %d, desc0 %x \n", aspeed_mctp->rx_idx, rx_cmd_desc[aspeed_mctp->rx_idx].desc0);
		MCTP_DBUG("index %d, desc1 %x \n", aspeed_mctp->rx_idx, rx_cmd_desc[aspeed_mctp->rx_idx].desc1);

		if (rx_cmd_desc[aspeed_mctp->rx_idx].desc0 & CMD_UPDATE) {
			if (aspeed_mctp->rx_fifo_done != 1) {
				MCTP_DBUG("rx fifo index %d \n", aspeed_mctp->rx_fifo_index);
				if (MCTP_SOM & rx_cmd_desc[aspeed_mctp->rx_idx].desc0) {
					MCTP_DBUG("MCTP_SOM \n");
					aspeed_mctp->rx_fifo_index = 0;
				}

				if (MCTP_EOM & rx_cmd_desc[aspeed_mctp->rx_idx].desc0) {
					MCTP_DBUG("MCTP_EOM aspeed_mctp->rx_fifo_done \n");
					aspeed_mctp->rx_fifo_done = 1;
					padding_len = GET_PADDING_LEN(rx_cmd_desc[aspeed_mctp->rx_idx].desc0);
					aspeed_mctp->flag = MCTP_EOM;
				}

				rx_len = GET_PKG_LEN(rx_cmd_desc[aspeed_mctp->rx_idx].desc0) * 4;
				rx_len -= padding_len;
#if 0
				memcpy(aspeed_mctp->rx_fifo + (0x40 * aspeed_mctp->rx_fifo_index), aspeed_mctp->rx_data + (aspeed_mctp->rx_idx * 0x80), rx_len);
				aspeed_mctp->rx_fifo_index++;

				aspeed_mctp->rx_len += rx_len;
				aspeed_mctp->ep_id = GET_SRC_EP_ID(aspeed_mctp->rx_cmd_desc[aspeed_mctp->rx_idx].desc0);
				aspeed_mctp->rt = GET_ROUTING_TYPE(aspeed_mctp->rx_cmd_desc[aspeed_mctp->rx_idx].desc0);
				aspeed_mctp->seq_no = GET_SEQ_NO(aspeed_mctp->rx_cmd_desc[aspeed_mctp->rx_idx].desc0);
#endif				
				MCTP_DBUG("rx len = %d , epid = %d, rt = %x, seq no = %d, total_len = %d \n", rx_len, aspeed_mctp->ep_id, aspeed_mctp->rt, aspeed_mctp->seq_no, aspeed_mctp->rx_len);

			} else {
				MCTP_DBUG("drop \n");
			}

			//RX CMD desc0
			rx_cmd_desc[aspeed_mctp->rx_idx].desc0 = 0;
			aspeed_mctp->rx_idx++;

		} else {
			MCTP_DBUG("index %d break\n", aspeed_mctp->rx_idx);
			break;
		}
	}

}

static irqreturn_t aspeed_pcie_raise_isr(int this_irq, void *dev_id)
{
	MCTP_DBUG("\n");
	return IRQ_HANDLED;
}

static irqreturn_t aspeed_mctp_isr(int this_irq, void *dev_id)
{
	struct aspeed_mctp_info *aspeed_mctp = dev_id;
	u32 status = aspeed_mctp_read(aspeed_mctp, ASPEED_MCTP_ISR);

	MCTP_DBUG("%x \n", status);

	if (status & MCTP_TX_LAST) {
		aspeed_mctp_write(aspeed_mctp, MCTP_TX_LAST | (status), ASPEED_MCTP_ISR);
		aspeed_mctp->flag = MCTP_TX_LAST;
	}

	if (status & MCTP_TX_COMPLETE) {
		aspeed_mctp_write(aspeed_mctp, MCTP_TX_COMPLETE | (status), ASPEED_MCTP_ISR);
//		aspeed_mctp->flag = MCTP_TX_COMPLETE;
	}

	if (status & MCTP_RX_COMPLETE) {
		aspeed_mctp_write(aspeed_mctp, MCTP_RX_COMPLETE | (status), ASPEED_MCTP_ISR);
		//disable interrupt trigger combine 
//		aspeed_mctp_rx_combine_data(aspeed_mctp);
		tasklet_schedule(&aspeed_mctp->mctp_rx_tasklet);
		return IRQ_HANDLED;
	}

	if (status & MCTP_RX_NO_CMD) {
		aspeed_mctp_write(aspeed_mctp, MCTP_RX_NO_CMD | (status), ASPEED_MCTP_ISR);
		aspeed_mctp->flag = MCTP_RX_NO_CMD;
		printk("MCTP_RX_NO_CMD \n");
	}

	if (aspeed_mctp->flag) {
		wake_up_interruptible(&aspeed_mctp->mctp_wq);
		return IRQ_HANDLED;
	} else {
		printk("TODO Check MCTP's interrupt %x\n", status);
		return IRQ_NONE;
	}

}

static void aspeed_mctp_ctrl_init(struct aspeed_mctp_info *aspeed_mctp)
{
	MCTP_DBUG("dram base %x \n", aspeed_mctp->dram_base);
	aspeed_mctp_write(aspeed_mctp, aspeed_mctp->dram_base, ASPEED_MCTP_EID);

	aspeed_mctp->tx_idx = 0;
	aspeed_mctp->cur_tx_buff = aspeed_mctp->tx_pool;
	aspeed_mctp->cur_tx_buff_dma = aspeed_mctp->tx_pool_dma;
	aspeed_mctp_write(aspeed_mctp, aspeed_mctp->tx_cmd_desc_dma, ASPEED_MCTP_TX_CMD);

	aspeed_mctp->rx_idx = 0;
	aspeed_mctp_write(aspeed_mctp, aspeed_mctp->rx_cmd_desc_dma, ASPEED_MCTP_RX_CMD);

	aspeed_mctp_write(aspeed_mctp, aspeed_mctp_read(aspeed_mctp, ASPEED_MCTP_CTRL) | MCTP_RX_CMD_RDY, ASPEED_MCTP_CTRL);

	aspeed_mctp_write(aspeed_mctp, MCTP_RX_COMPLETE | MCTP_TX_LAST, ASPEED_MCTP_IER);
}

static long mctp_ioctl(struct file *file, unsigned int cmd,
		       unsigned long arg)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_mctp_info *aspeed_mctp = dev_get_drvdata(c->this_device);
	void __user *argp = (void __user *)arg;
	struct aspeed_mctp_xfer mctp_xfer;

	switch (cmd) {
		case ASPEED_MCTP_IOCTX:
			MCTP_DBUG("ASPEED_MCTP_IOCTX ver %d\n", aspeed_mctp->mctp_version);
			if (copy_from_user(&mctp_xfer, argp, sizeof(struct aspeed_mctp_xfer))) {
				MCTP_DBUG("copy_from_user fail\n");
				return -EFAULT;
			} else {
				MCTP_DBUG("copy tx buffer %x len %d\n", (u32)aspeed_mctp->cur_tx_buff, mctp_xfer.xfer_len);
				copy_from_user(aspeed_mctp->cur_tx_buff, mctp_xfer.xfer_buff, mctp_xfer.xfer_len);
				mctp_xfer.xfer_buff = aspeed_mctp->cur_tx_buff;
				aspeed_mctp_tx_xfer(aspeed_mctp, &mctp_xfer);
			}
			break;
		case ASPEED_MCTP_IOCRX:
			MCTP_DBUG("ASPEED_MCTP_IOCRX \n");
			//wait intr
	//			if(!aspeed_mctp->rx_fifo_done) {
	//				MCTP_DBUG("wait for rx fifo done \n");
			aspeed_mctp_wait_rx_complete(aspeed_mctp);
	//			}
			mctp_xfer.xfer_len = aspeed_mctp->rx_len;
			memcpy(mctp_xfer.xfer_buff, aspeed_mctp->rx_fifo,  aspeed_mctp->rx_len);
			if (copy_to_user(argp, &mctp_xfer, sizeof(struct aspeed_mctp_xfer)))
				return -EFAULT;
			else {
				aspeed_mctp->rx_fifo_done = 0;
				aspeed_mctp->rx_len = 0;
				memset(aspeed_mctp->rx_fifo, 0,  1024);
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
	spin_lock(&mctp_state_lock);

	if (aspeed_mctp->is_open) {
		spin_unlock(&mctp_state_lock);
		return -EBUSY;
	}

	aspeed_mctp->is_open = true;

	spin_unlock(&mctp_state_lock);

	return 0;
}

static int mctp_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_mctp_info *aspeed_mctp = dev_get_drvdata(c->this_device);

	MCTP_DBUG("\n");
	spin_lock(&mctp_state_lock);

	aspeed_mctp->is_open = false;

	spin_unlock(&mctp_state_lock);

	return 0;
}

static const struct file_operations aspeed_mctp_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= mctp_ioctl,
	.open			= mctp_open,
	.release			= mctp_release,
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
	int i = 0;

	MCTP_DBUG("\n");

	if (!(aspeed_mctp = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_mctp_info), GFP_KERNEL))) {
		return -ENOMEM;
	}

	mctp_dev_id = of_match_device(aspeed_mctp_of_matches, &pdev->dev);
	if (!mctp_dev_id)
		return -EINVAL;

	aspeed_mctp->mctp_version = (unsigned long)mctp_dev_id->data;
	switch(aspeed_mctp->mctp_version) {
		case 0:
		case 5:
			aspeed_mctp->tx_fifo_size = MCTP_TX_FIFO_NUM;
			aspeed_mctp->rx_fifo_size = MCTP_RX_FIFO_NUM;
			break;
		case 6:
			aspeed_mctp->tx_fifo_size = MCTP_G6_TX_FIFO_NUM;
			aspeed_mctp->rx_fifo_size = MCTP_G6_RX_FIFO_NUM;
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

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_BUS\n");
		ret = -ENOENT;
		goto out_region;
	}

	aspeed_mctp->dram_base = (u32)res->start;

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

	aspeed_mctp->flag = 0;
	init_waitqueue_head(&aspeed_mctp->mctp_wq);

	ret = misc_register(&aspeed_mctp_misc);
	if (ret) {
		printk(KERN_ERR "MCTP : failed to request interrupt\n");
		goto out_region;
	}

	platform_set_drvdata(pdev, aspeed_mctp);
	dev_set_drvdata(aspeed_mctp_misc.this_device, aspeed_mctp);

	//memory allocation
	//1st 2048 : cmd desc --> 0~2048 : tx desc , 512 ~ 1024 : rx desc
	aspeed_mctp->tx_idx = 0;
	//tx desc
	aspeed_mctp->tx_cmd_desc = dma_alloc_coherent(NULL,
						MCTP_DESC_SIZE,
						&aspeed_mctp->tx_cmd_desc_dma, GFP_KERNEL);
		
	//tx buff pool init 
	aspeed_mctp->tx_pool = dma_alloc_coherent(NULL,
						MCTP_BUFF_SIZE * aspeed_mctp->tx_fifo_size,
						&aspeed_mctp->tx_pool_dma, GFP_KERNEL);
	
	aspeed_mctp->cur_tx_buff = aspeed_mctp->tx_pool;
	aspeed_mctp->cur_tx_buff_dma = aspeed_mctp->tx_pool_dma;

	//rx desc
	//2nd 2048 : cmd desc --> 2048~4096 : 2048 ~ 4096 : rx desc 
	aspeed_mctp->rx_cmd_desc = (struct aspeed_mctp_cmd_desc *)(aspeed_mctp->tx_cmd_desc + 2048);
	aspeed_mctp->rx_cmd_desc_dma = aspeed_mctp->tx_cmd_desc_dma + 2048;
	
	//rx desc buff pool
	//3rd 1024 : rx data --> 8 - 0x00 , 0x80 , 0x100, 0x180, each 128 align
	aspeed_mctp->rx_pool = dma_alloc_coherent(NULL,
						MCTP_BUFF_SIZE * aspeed_mctp->rx_fifo_size,
						&aspeed_mctp->rx_pool_dma, GFP_KERNEL);

	//rx fifo data
	//4th 4096 * n : rx data combine
	aspeed_mctp->rx_list_mem = kmalloc(sizeof(struct mctp_list) * aspeed_mctp->rx_fifo_size, GFP_KERNEL);
	for(i = 0; i < aspeed_mctp->rx_fifo_size; i++) {
		aspeed_mctp->rx_list_mem[i].fifo_state = RX_FIFO_FREE;
		aspeed_mctp->rx_list_mem[i].rx_buff = aspeed_mctp->rx_pool + (i * MCTP_BUFF_SIZE);
		aspeed_mctp->rx_list_mem[i].rx_buff_dma = aspeed_mctp->rx_pool_dma + (i * MCTP_BUFF_SIZE);
	}

	//prepare rx desc and rx buffer : init 8 numbers
	aspeed_mctp->rx_idx = 0;
	if(aspeed_mctp->mctp_version == 6) {
		u32 *rx_cmd_desc = aspeed_mctp->rx_cmd_desc;
		for (i = 0; i < MCTP_RX_DESC_BUFF_NUM; i++) {
			rx_cmd_desc[i] = RX_DATA_ADDR(aspeed_mctp->rx_list_mem[i].rx_buff_dma);
			aspeed_mctp->rx_list_mem[i].fifo_state = RX_FIFO_DESC;
			MCTP_DBUG("Rx [%d]: desc: %x \n", i, rx_cmd_desc[i]);
		}
	} else {
		//old mctp rx data must 128 byte align --> 8 - 0x00 , 0x80 , 0x100, 0x180, each 128 align
		struct aspeed_mctp_cmd_desc *rx_cmd_desc = aspeed_mctp->rx_cmd_desc;
		for (i = 0; i < MCTP_RX_DESC_BUFF_NUM; i++) {
			rx_cmd_desc[i].desc0 = 0;
			rx_cmd_desc[i].desc1 = RX_DATA_ADDR(aspeed_mctp->rx_list_mem[i].rx_buff_dma);
			aspeed_mctp->rx_list_mem[i].fifo_state = RX_FIFO_DESC;
			if (i == 7)
				rx_cmd_desc[i].desc1 |= LAST_CMD;
			MCTP_DBUG("Rx [%d]: desc0: %x , desc1: %x \n", i, rx_cmd_desc[i].desc0, rx_cmd_desc[i].desc1);
		}
	}

	if(aspeed_mctp->mctp_version == 6) 
		tasklet_init(&aspeed_mctp->mctp_rx_tasklet, 
					aspeed_g6_mctp_tasklet, (unsigned long)aspeed_mctp);
	else 
		tasklet_init(&aspeed_mctp->mctp_rx_tasklet, 
					aspeed_mctp_tasklet, (unsigned long)aspeed_mctp);
	
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
