/*
 * ast-udc.c - UDC driver for the Aspeed SoC
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/prefetch.h>
#include <linux/clk.h>
#include <linux/usb/ch9.h>
#include <linux/delay.h>

#include <linux/usb/gadget.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <linux/dma-mapping.h>


/*************************************************************************************/
#define AST_VHUB_CTRL 				0x00
#define AST_VHUB_CONF 				0x04
#define AST_VHUB_IER 				0x08
#define AST_VHUB_ISR 				0x0C
#define AST_VHUB_EP_ACK_IER 		0x10
#define AST_VHUB_EP_NAK_IER 		0x14
#define AST_VHUB_EP_ACK_ISR 		0x18
#define AST_VHUB_EP_NAK_ISR 		0x1C
#define AST_VHUB_DEV_RESET 			0x20
#define AST_VHUB_USB_STS 			0x24
#define AST_VHUB_EP_DATA 			0x28
#define AST_VHUB_ISO_TX_FAIL		0x2C
#define AST_VHUB_EP0_CTRL			0x30
#define AST_VHUB_EP0_DATA_BUFF		0x34
#define AST_VHUB_EP1_CTRL			0x38
#define AST_VHUB_EP1_STS_CHG		0x3C

#define AST_VHUB_SETUP_DATA0		0x80
#define AST_VHUB_SETUP_DATA1		0x84

/*  ************************************************************************************/
/* AST_VHUB_CTRL 		0x00 */
#define ROOT_PHY_CLK_EN				(1 << 31)
#define ROOT_PHY_SELF_TEST_EN			(1 << 25)
#define ROOT_DN_15K_EN					(1 << 24)
#define ROOT_DP_15K_EN					(1 << 23)
#define ROOT_FIFO_DYNP_EN				(1 << 19)
#define ROOT_EP_LONG_DESC				(1 << 18)
#define ROOT_ISO_IN_NULL_RESP			(1 << 17)
#define ROOT_SPLIT_IN					(1 << 16)
#define ROOT_LOOP_TEST_PASS			(1 << 15)
#define ROOT_LOOP_TEST_FINISH			(1 << 14)
#define ROOT_BIST_TEST_PASS				(1 << 13)
#define ROOT_BIST_ON					(1 << 12)
#define ROOT_PHY_RESET_DIS				(1 << 11)
#define ROOT_TEST_MODE(x)				((x) << 8)
#define ROOT_FORCE_TIMER_HS			(1 << 7)
#define ROOT_FORCE_HS					(1 << 6)
#define ROOT_REMOTE_WAKEUP_12MS		(1 << 5)
#define ROOT_REMOTE_WAKEUP_EN			(1 << 4)
#define ROOT_AUTO_REMOTE_WAKEUP_EN	(1 << 3)
#define ROOT_STOP_CLK_IN_SUPEND		(1 << 2)
#define ROOT_UPSTREAM_FS				(1 << 1)
#define ROOT_UPSTREAM_EN				(1)

/* AST_VHUB_CONF 		0x04 */
#define CONF_GET_DMA_STS(x)				((x >> 16) & 0xff)
#define CONF_GET_DEV_ADDR(x)			(x & 0x7f)

/* AST_VHUB_IER 		0x08	*/
#define ISR_USB_CMD_DEADLOCK			(1 << 18)

#define ISR_EP_NAK						(1 << 17)
#define ISR_EP_ACK_STALL				(1 << 16)
#define ISR_DEVICE5						(1 << 13)
#define ISR_DEVICE4						(1 << 12)
#define ISR_DEVICE3						(1 << 11)
#define ISR_DEVICE2						(1 << 10)
#define ISR_DEVICE1						(1 << 9)
#define ISR_SUSPEND_RESUME				(1 << 8)
#define ISR_BUS_SUSPEND 				(1 << 7)
#define ISR_BUS_RESET 					(1 << 6)
#define ISR_HUB_EP1_IN_DATA_ACK		(1 << 5)
#define ISR_HUB_EP0_IN_DATA_NAK		(1 << 4)
#define ISR_HUB_EP0_IN_ACK_STALL		(1 << 3)
#define ISR_HUB_EP0_OUT_NAK			(1 << 2)
#define ISR_HUB_EP0_OUT_ACK_STALL		(1 << 1)
#define ISR_HUB_EP0_SETUP				(1)

/* AST_VHUB_EP_ACK_IER 		0x10 */
#define EP14_ISR							(1 << 14)
#define EP13_ISR							(1 << 13)
#define EP12_ISR							(1 << 12)
#define EP11_ISR							(1 << 11)
#define EP10_ISR							(1 << 10)
#define EP9_ISR							(1 << 9)
#define EP8_ISR							(1 << 8)
#define EP7_ISR							(1 << 7)
#define EP6_ISR							(1 << 6)
#define EP5_ISR							(1 << 5)
#define EP4_ISR							(1 << 4)
#define EP3_ISR							(1 << 3)
#define EP2_ISR							(1 << 2)
#define EP1_ISR							(1 << 1)
#define EP0_ISR							(1)

/* AST_VHUB_DEV_RESET_ISR 		0x20 */
#define DEV5_SOFT_RESET					(1 << 5)
#define DEV4_SOFT_RESET					(1 << 4)
#define DEV3_SOFT_RESET					(1 << 3)
#define DEV2_SOFT_RESET					(1 << 2)
#define DEV1_SOFT_RESET					(1 << 1)
#define ROOT_HUB_SOFT_RESET			(1)


/* AST_VHUB_EP0_CTRL			0x30 */
#define EP0_GET_RX_LEN(x)				((x >> 16) & 0x7f)
#define EP0_TX_LEN(x)						((x & 0x7f) << 8)
#define EP0_RX_BUFF_RDY					(1 << 2)
#define EP0_TX_BUFF_RDY					(1 << 1)
#define EP0_STALL						(1)

#define AST_UDC_DEV_CTRL 				0x00
#define AST_UDC_DEV_ISR				0x04
#define AST_UDC_DEV_EP0_CTRL			0x08
#define AST_UDC_DEV_EP0_DATA_BUFF	0x0C

/*  ************************************************************************************/
//#define AST_UDC_DEV_CTRL 				0x00
#define DEV_CTRL_DEV_ADDR_MASK		(0x3f << 8)
#define DEV_CTRL_IN_NAK_EN				(1 << 6)
#define DEV_CTRL_IN_ACK_STALL_EN		(1 << 5)
#define DEV_CTRL_OUT_NAK_EN			(1 << 4)
#define DEV_CTRL_OUT_ACK_STALL_EN		(1 << 3)
#define DEV_CTRL_SETUP_EN				(1 << 2)
#define DEV_CTRL_HIGH_SPEED_MODE		(1 << 1)
#define DEV_CTRL_DEV_EN				(1)

//define AST_UDC_DEV_ISR					0x04
#define DEV_CTRL_IN_NAK_INT			(1 << 4)
#define DEV_CTRL_IN_ACK_STALL_INT		(1 << 3)
#define DEV_CTRL_OUT_NAK_INT			(1 << 2)
#define DEV_CTRL_OUT_ACK_STALL_INT	(1 << 1)
#define DEV_CTRL_SETUP_INT				(1)

//#define AST_UDC_DEV_EP0_CTRL			0x08
#define DEV_EP0_GET_RX_SIZE(x)			((x >> 16) & 0x7f)
#define DEV_EP0_TX_SIZE_MASK			(0x7f << 8)
#define DEV_EP0_SET_TX_SIZE(x)			((x & 0x7f) << 8)
#define DEV_EP0_RX_BUFF_RDY			(1 << 2)
#define DEV_EP0_TX_BUFF_RDY			(1 << 1)
#define DEV_EP0_STALL					(1)

/*************************************************************************************/
#define AST_EP_CONFIG					0x00
#define AST_EP_DMA_CTRL				0x04
#define AST_EP_DMA_BUFF				0x08
#define AST_EP_DMA_STS					0x0C

/*************************************************************************************/
//#define AST_EP_CONFIG					0x00
#define EP_SET_MAX_PKT(x)				((x & 0x3ff) << 16)

#define EP_AUTO_DATA_DISABLE			(0x1 << 13)
#define EP_SET_EP_STALL					(0x1 << 12)

#define EP_SET_EP_NUM(x)				((x & 0xf) << 8)

#define EP_SET_TYPE_MASK(x)				((x) << 4)
#define EP_TYPE_BULK_IN					(0x2 << 4)
#define EP_TYPE_BULK_OUT				(0x3 << 4)
#define EP_TYPE_INT_IN					(0x4 << 4)
#define EP_TYPE_INT_OUT					(0x5 << 4)
#define EP_TYPE_ISO_IN					(0x6 << 4)
#define EP_TYPE_ISO_OUT					(0x7 << 4)

#define EP_ALLOCATED_MASK				(0x7 << 1)

#define EP_ENABLE						(1)

//#define AST_EP_DMA_STS					0x0C


//#define AST_EP_DMA_CTRL				0x04

#define EP_SINGLE_DMA_MODE			(0x1 << 1)

/*************************************************************************************/
struct ast_udc_config {
	u8 end_point_num;
};

struct ast_udc_request {
	struct usb_request		req;
	struct list_head		queue;
	unsigned mapped:1;	
};

struct ast_udc_ep {
	struct usb_ep			ep;
	struct list_head		queue;
	struct ast_udc	*udc;
	void __iomem			*ep_reg;
	unsigned				stopped:1;
	u8						ep_dir;
	void					*ep_buf;
	dma_addr_t				ep_dma;
	const struct usb_endpoint_descriptor	*desc;	
};

/*
 * driver is non-SMP, and just blocks IRQs whenever it needs
 * access protection for chip registers or driver state
 */
struct ast_udc {
	void __iomem			*reg;
	struct clk 				*clk;	
	struct usb_gadget			gadget;
	//struct ast_udc_ep			ep[AST_NUM_ENDPOINTS];
	struct ast_udc_ep			*ep;
	struct ast_udc_config		*udc_config;
	struct usb_gadget_driver	*driver;
	unsigned			suspended:1;
	unsigned			req_pending:1;
	unsigned			wait_for_addr_ack:1;
	unsigned			wait_for_config_ack:1;
	unsigned			active_suspend:1;
	u8				addr;
	struct platform_device	*pdev;
	struct proc_dir_entry	*pde;
	struct usb_ctrlrequest *root_setup;	
	int					irq;
	spinlock_t			lock;
	void					*ep0_ctrl_buf;
	dma_addr_t			ep0_ctrl_dma;
	
};

static const char * const ast_ep_name[] = {
	"ep0", "ep1", "ep2", "ep3", "ep4","ep5", "ep6", "ep7", "ep8", "ep9","ep10", "ep11", "ep12", "ep13", "ep14", "ep15"};

#define AST_UDC_DEBUG
//#define AST_BUS_DEBUG
//#define AST_SETUP_DEBUG
//#define AST_EP_DEBUG
//#define AST_ISR_DEBUG

#ifdef AST_BUS_DEBUG
#define BUS_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define BUS_DBG(fmt, args...)
#endif

#ifdef AST_SETUP_DEBUG
//#define SETUP_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#define SETUP_DBG(fmt, args...) printk("%s() " fmt,__FUNCTION__, ## args)
#else
#define SETUP_DBG(fmt, args...)
#endif

#ifdef AST_EP_DEBUG
//#define EP_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#define EP_DBG(fmt, args...) printk("%s() " fmt,__FUNCTION__, ## args)

#else
#define EP_DBG(fmt, args...)
#endif

#ifdef AST_UDC_DEBUG
#define UDC_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define UDC_DBG(fmt, args...)
#endif

#ifdef AST_ISR_DEBUG
//#define ISR_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#define ISR_DBG(fmt, args...) printk("%s() " fmt,__FUNCTION__, ## args)
#else
#define ISR_DBG(fmt, args...)
#endif

/*-------------------------------------------------------------------------*/
#define ast_udc_read(udc, offset) \
	__raw_readl((udc)->reg + (offset))
#define ast_udc_write(udc, val, offset) \
	__raw_writel((val), (udc)->reg + (offset))

#define ast_ep_read(ep, reg) \
	__raw_readl((ep)->ep_reg + (reg))
#define ast_ep_write(ep, val, reg) \
	__raw_writel((val), (ep)->ep_reg + (reg))
/*-------------------------------------------------------------------------*/
static void ast_udc_done(struct ast_udc_ep *ep, struct ast_udc_request *req, int status)
{
	struct ast_udc *udc = ep->udc;

	EP_DBG("%s len (%d/%d) buf %x, dir %x\n",ep->ep.name, req->req.actual, req->req.length, req->req.buf, ep->ep_dir);
//	list_del_init(&req->queue);
	list_del(&req->queue);
	
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;
	
	if (status && status != -ESHUTDOWN)
		EP_DBG("%s done %p, status %d\n", ep->ep.name, req, status);

	spin_unlock(&udc->lock);
	usb_gadget_giveback_request(&ep->ep, &req->req);
	spin_lock(&udc->lock);
//	printk("done end\n");

}

static void ast_udc_nuke(struct ast_udc_ep *ep, int status)
{
        /* Sanity check */
        if (&ep->queue == NULL)
                return;

        while (!list_empty(&ep->queue)) {
                struct ast_udc_request *req;
                req = list_entry(ep->queue.next, struct ast_udc_request,
                                queue);
                ast_udc_done(ep, req, status);
        }
}

/**
 * Stop activity on all endpoints.
 * Device controller for which EP activity is to be stopped.
 *
 * All the endpoints are stopped and any pending transfer requests if any on
 * the endpoint are terminated.
 */
static void ast_udc_stop_activity(struct ast_udc *udc)
{
	int epnum;
	struct ast_udc_ep *ep;

	for (epnum = 0; epnum < 16; epnum++) {
		ep = &udc->ep[epnum];
		ep->stopped = 1;
		ast_udc_nuke(ep, -ESHUTDOWN);
	}
}

/*-------------------------------------------------------------------------*/

static int ast_udc_ep_enable(struct usb_ep *_ep,
				const struct usb_endpoint_descriptor *desc)
{
	struct ast_udc_ep	*ep = container_of(_ep, struct ast_udc_ep, ep);
	struct ast_udc *udc = ep->udc;
	u16	maxpacket = usb_endpoint_maxp(desc);
	u8	epnum = usb_endpoint_num(desc);	
	unsigned long	flags;
	u32 		ep_conf = 0;
	u8		type;
	u8		dir_in;

	EP_DBG("%s, set ep #%d, maxpacket %d ,wmax %d \n", ep->ep.name, epnum, maxpacket, le16_to_cpu(desc->wMaxPacketSize));

	if (!_ep || !ep || !desc || desc->bDescriptorType != USB_DT_ENDPOINT) {
		printk("bad ep or descriptor %s %d , maxpacket %d, ep maxpacket %d \n", _ep->name, desc->bDescriptorType, maxpacket, ep->ep.maxpacket);
		return -EINVAL;
	}

	if (!udc->driver) {
		printk("bogus device state\n");
		return -ESHUTDOWN;
	}
	
	spin_lock_irqsave(&udc->lock, flags);

	ep->desc = desc;
	ep->stopped = 0;
	ep->ep.maxpacket = maxpacket;

	if(maxpacket > 1024) {
		printk("TODO check size ~~~~ \n");
		maxpacket = 1024;
	}
	
	if(maxpacket == 1024) {
		ep_conf = 0;
	} else {
		ep_conf = EP_SET_MAX_PKT(maxpacket);
	}
	
	ep_conf |= EP_SET_EP_NUM(epnum);
	
	type = usb_endpoint_type(desc);
	dir_in = usb_endpoint_dir_in(desc);
	ep->ep_dir = dir_in;

	EP_DBG("epnum %d, type %d, dir_in %d \n", epnum, type, dir_in);
	switch(type) {
		case USB_ENDPOINT_XFER_ISOC:
			if(dir_in) {
				ep_conf |= EP_TYPE_ISO_IN;
			} else {
				ep_conf |= EP_TYPE_ISO_OUT;
			}
			break;
		case USB_ENDPOINT_XFER_BULK:
			if(dir_in) {
				ep_conf |= EP_TYPE_BULK_IN;
			} else {
				ep_conf |= EP_TYPE_BULK_OUT;
			}			
			break;
		case USB_ENDPOINT_XFER_INT:
			if(dir_in) {
				ep_conf |= EP_TYPE_INT_IN;
			} else {
				ep_conf |= EP_TYPE_INT_OUT;
			}			
			break;
	}

	ast_ep_write(ep, 0x4, AST_EP_DMA_CTRL);
	ast_ep_write(ep, 0x2, AST_EP_DMA_CTRL); 
	ast_ep_write(ep, 0, AST_EP_DMA_STS);
	ast_ep_write(ep, ep_conf | EP_ENABLE, AST_EP_CONFIG);

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static int ast_udc_ep_disable (struct usb_ep * _ep)
{
	struct ast_udc_ep	*ep = container_of(_ep, struct ast_udc_ep, ep);
	struct ast_udc		*udc = ep->udc;	

	unsigned long	flags;

	EP_DBG("%s\n", _ep->name);

	spin_lock_irqsave(&udc->lock, flags);

	ep->ep.desc = NULL;
	ep->stopped = 1;

	ast_udc_nuke(ep, -ESHUTDOWN);

	ast_ep_write(ep, 0, AST_EP_CONFIG);
	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}

static struct usb_request *
ast_udc_ep_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct ast_udc_request *req;
	EP_DBG("%s\n", _ep->name);

	req = kzalloc(sizeof (struct ast_udc_request), gfp_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

static void ast_udc_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct ast_udc_request *req;
	EP_DBG("%s\n", _ep->name);

	req = container_of(_req, struct ast_udc_request, req);
	kfree(req);
}

static void ast_udc_ep_dma(struct ast_udc_ep *ep, struct ast_udc_request *req)
{
	u16	tx_len;

	if((req->req.length - req->req.actual) > ep->ep.maxpacket) {
		tx_len = ep->ep.maxpacket; 
	} else {
		tx_len = req->req.length - req->req.actual;
	}

#if 0
	if(ep->ep_dir) {
		EP_DBG("%s : tx dma %x - len %d:(%d/%d) :",ep->ep.name, req->req.dma + req->req.actual, tx_len, req->req.actual, req->req.length);
#if 0			
		for(i = 0; i < tx_len; i++)
			printk(" %x", buf[i]);
		printk("\n");
#else
//		printk("\n");
#endif 			
	} else {
		EP_DBG("%s : rx dma %x - len %d:(%d/%d) \n",ep->ep.name, req->req.dma + req->req.actual, tx_len, req->req.actual, req->req.length);		
	}
#endif

#if 1
	if(tx_len > 1024) printk("*************************************************8\n");
	EP_DBG("dma: %s : len : %d dir %x\n", ep->ep.name, tx_len, ep->ep_dir);

	if((req->req.dma % 4) != 0) {
//		printk("1: %s : %x len (%d/%d) dir %x\n", ep->ep.name, req->req.dma, req->req.actual, req->req.length, ep->ep_dir);
		if((ep->ep_dir) && (!req->req.actual)) {
			memcpy(ep->ep_buf, req->req.buf, req->req.length);
		}
		ast_ep_write(ep, ep->ep_dma + req->req.actual, AST_EP_DMA_BUFF);
	} else {
		ast_ep_write(ep, req->req.dma + req->req.actual, AST_EP_DMA_BUFF);
	}
	//trigger
	ast_ep_write(ep, tx_len << 16, AST_EP_DMA_STS);
	ast_ep_write(ep, tx_len << 16 | 0x1, AST_EP_DMA_STS);		

#else
	ast_ep_write(ep, req->req.dma + req->req.actual, AST_EP_DMA_BUFF);
	//trigger
	ast_ep_write(ep, tx_len << 16, AST_EP_DMA_STS);
	ast_ep_write(ep, tx_len << 16 | 0x1, AST_EP_DMA_STS);	
#endif

}

static void ast_udc_ep0_queue(struct ast_udc_ep *ep, struct ast_udc_request *req)
{
	struct ast_udc		*udc = ep->udc;
	u16	tx_len;

//	if(req->req.length > 64) printk("ep0 queue len %d ***************************\n", req->req.length);

	if((req->req.length - req->req.actual) > ep->ep.maxpacket) {
		tx_len = ep->ep.maxpacket; 
	} else {
		tx_len = req->req.length - req->req.actual;
	}

	ast_udc_write(udc, req->req.dma + req->req.actual, AST_VHUB_EP0_DATA_BUFF);
	
	if(ep->ep_dir) {
		SETUP_DBG("ep0 in addr buf %x, dma %x , txlen %d:(%d/%d) ,dir %d \n", (u32)req->req.buf, req->req.dma + req->req.actual, tx_len, req->req.actual, req->req.length, ep->ep_dir);
		req->req.actual += tx_len;		
		ast_udc_write(udc, EP0_TX_LEN(tx_len), AST_VHUB_EP0_CTRL);
		ast_udc_write(udc, EP0_TX_LEN(tx_len) | EP0_TX_BUFF_RDY, AST_VHUB_EP0_CTRL);
	} else {
		SETUP_DBG("ep0 out ~~ addr buf %x, dma %x , (%d/%d) ,dir %d \n", (u32)req->req.buf, req->req.dma + req->req.actual, req->req.actual, req->req.length, ep->ep_dir);
		if(!req->req.length) {
			ast_udc_write(udc, EP0_TX_BUFF_RDY, AST_VHUB_EP0_CTRL);
			ep->ep_dir = 0x80;
		} else {
			ast_udc_write(udc, EP0_RX_BUFF_RDY, AST_VHUB_EP0_CTRL); 
		}

	}
}

static int ast_udc_ep_queue(struct usb_ep *_ep,
			struct usb_request *_req, gfp_t gfp_flags)
{
	struct ast_udc_request	*req = container_of(_req, struct ast_udc_request, req);
	struct ast_udc_ep		*ep = container_of(_ep, struct ast_udc_ep, ep);
	struct ast_udc		*udc = ep->udc;
	unsigned long		flags;
	int request = 0;	

	if (unlikely(!_req || !_req->complete || !_req->buf || !_ep))
		return -EINVAL;

	if(ep->stopped) {
		printk("%s : is stop \n", _ep->name);
		return 1;
	}
	EP_DBG("%s : len : %d \n", _ep->name, _req->length);

	spin_lock_irqsave(&udc->lock, flags);

	list_add_tail(&req->queue, &ep->queue);

	req->req.actual = 0;
	req->req.status = -EINPROGRESS;

	if(usb_gadget_map_request(&udc->gadget, &req->req, ep->ep_dir)) {
		printk("map ERROR ~~ \n");
		return 1;
	}

	if (ep->ep.desc == NULL) {	/* ep0 */
		if((req->req.dma % 4) != 0) {printk("EP0 dma  %x error ~~~~ \n", req->req.dma); while(1); }
		ast_udc_ep0_queue(ep, req);
	} else {
		if (list_is_singular(&ep->queue)) {
//			if(ep->ep.name == "ep1") printk("%s dma req %x \n", ep->ep.name, req);
			ast_udc_ep_dma(ep, req);
		} else {
//			printk("%s just add req %x buf %x \n", ep->ep.name, req, req->req.buf);
		}
	}

	spin_unlock_irqrestore(&udc->lock, flags);
	
	return 0;

}

static int ast_udc_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct ast_udc_ep		*ep = container_of(_ep, struct ast_udc_ep, ep);
	struct ast_udc_request	*req;
	unsigned long		flags;
	struct ast_udc		*udc = ep->udc;

	UDC_DBG("%s \n", _ep->name);

	if (!_ep || ep->ep.name == ast_ep_name[0])
		return -EINVAL;

	spin_lock_irqsave(&udc->lock, flags);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req) {
			list_del_init(&req->queue);
			_req->status = -ECONNRESET;			
			break;
		}
	}
	if (&req->req != _req) {
		spin_unlock_irqrestore(&udc->lock, flags);
		return -EINVAL;
	}

	ast_udc_done(ep, req, -ESHUTDOWN);
	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static int ast_udc_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct ast_udc_ep	*ep = container_of(_ep, struct ast_udc_ep, ep);
	struct ast_udc	*udc = ep->udc;
	unsigned long	flags;

	UDC_DBG("%s \n", _ep->name);
	printk("%s : %d\n", _ep->name, value);
	if (!_ep)
		return -EINVAL;

	spin_lock_irqsave(&udc->lock, flags);

	if(_ep->name == ast_ep_name[0]) {
		if(value)
			ast_udc_write(udc, ast_udc_read(udc, AST_VHUB_EP0_CTRL) | EP0_STALL, AST_VHUB_EP0_CTRL);
		else
			ast_udc_write(udc, ast_udc_read(udc, AST_VHUB_EP0_CTRL) & ~EP0_STALL, AST_VHUB_EP0_CTRL);
	} else {
		if(value) {
			ast_ep_write(ep, ast_ep_read(ep, AST_EP_CONFIG) | EP_SET_EP_STALL, AST_EP_CONFIG);
		} else {
			ast_ep_write(ep, ast_ep_read(ep, AST_EP_CONFIG) & ~EP_SET_EP_STALL, AST_EP_CONFIG);
		}
	}
	ep->stopped = value ? 1:0;
	
	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}

static const struct usb_ep_ops ast_udc_ep_ops = {
	.enable		= ast_udc_ep_enable,
	.disable		= ast_udc_ep_disable,
	.alloc_request	= ast_udc_ep_alloc_request,
	.free_request	= ast_udc_ep_free_request,
	.queue		= ast_udc_ep_queue,
	.dequeue		= ast_udc_ep_dequeue,
	.set_halt		= ast_udc_ep_set_halt,
	/* there's only imprecise fifo status reporting */
};

/*************************************************************************************************************************************************/
void ast_udc_ep0_rx(struct ast_udc *udc)
{
	SETUP_DBG("\n");

	ast_udc_write(udc, udc->ep0_ctrl_dma, AST_VHUB_EP0_DATA_BUFF);
	//trigger
	ast_udc_write(udc, EP0_RX_BUFF_RDY, AST_VHUB_EP0_CTRL); 		

}

void ast_udc_ep0_tx(struct ast_udc *udc)
{
	SETUP_DBG("\n");

	ast_udc_write(udc, udc->ep0_ctrl_dma, AST_VHUB_EP0_DATA_BUFF);
	//trigger
	ast_udc_write(udc, EP0_TX_BUFF_RDY, AST_VHUB_EP0_CTRL); 		

}

void ast_udc_ep0_out(struct ast_udc *udc)
{
	struct ast_udc_ep *ep = &udc->ep[0];
	struct ast_udc_request	*req;
/*	int i;*/
	u16 rx_len = EP0_GET_RX_LEN(ast_udc_read(udc, AST_VHUB_EP0_CTRL));	
	u8 *buf;

	SETUP_DBG("\n");

	if (list_empty(&ep->queue)) {
		return;
	} else {
		req = list_entry(ep->queue.next, struct ast_udc_request, queue);
	}

	buf = req->req.buf;

	req->req.actual += rx_len;
	
	if((rx_len < ep->ep.maxpacket) || (req->req.actual == req->req.length)) {
		ast_udc_ep0_tx(udc);
		if(!ep->ep_dir) {
			ast_udc_done(ep, req, 0);
		}		
	} else {
		if(rx_len > req->req.length) {
//			printk("ep0 out Check %d %d %d dir %x \n",req->req.length,  rx_len,  ep->ep.maxpacket, ep->ep_dir);
#if 0
			printk("error ~~~~~ ~~~~~~ `~~~~~ %x \n", ast_udc_read(udc, AST_VHUB_EP0_CTRL));
			printk("%x \n", ast_udc_read(udc, AST_VHUB_EP0_CTRL));
			for(i = 0; i < rx_len; i++) {
				printk("%x ", buf[i]);
			}
			printk("\n");
#endif			
			//Issue Fix 
			printk("Issue Fix (%d/%d)\n", req->req.actual, req->req.length);
			ast_udc_ep0_tx(udc);
			ast_udc_done(ep, req, 0);
			return;
		}	
		ep->ep_dir = 0;
		ast_udc_ep0_queue(ep, req);
	}
}

void ast_udc_ep0_in(struct ast_udc *udc)
{
	struct ast_udc_ep *ep = &udc->ep[0];
	struct ast_udc_request	*req;

	SETUP_DBG("\n");

	if (list_empty(&ep->queue)) {
		return;
	} else {
		req = list_entry(ep->queue.next, struct ast_udc_request, queue);
	}

	if(req->req.length == req->req.actual) {
		ast_udc_ep0_rx(udc);
		if(ep->ep_dir) {
			ast_udc_done(ep, req, 0);
		}
	} else {
		ast_udc_ep0_queue(ep, req);
	}
}

void ast_udc_ep_handle(struct ast_udc *udc, u16 ep_num)
{
	struct ast_udc_ep *ep = &udc->ep[ep_num];
	struct ast_udc_request	*req;
	u16 len = 0;

	if(list_empty(&ep->queue))
		return;

//	req = list_entry(ep->queue.next, struct ast_udc_request, queue);
	req = list_first_entry(&ep->queue, struct ast_udc_request, queue);

//	printk("%s handle req %x \n", ep->ep.name, req);
//	printk("req = %x \n",req);
//	buf = (u8 *) req->req.buf;

	len = (ast_ep_read(ep, AST_EP_DMA_STS) >> 16) & 0x7ff;
	
//	if((len == 0) || (len < 1024)) {
//		printk("TODO ~~~~~~ len = %d\n", len);
//	}
	req->req.actual += len;
	
//	EP_DBG(" %s : actual %d \n", ep->ep.name, req->req.actual);
	
	if((req->req.length == req->req.actual)){
#if 1		
		usb_gadget_unmap_request(&udc->gadget, &req->req, ep->ep_dir);
		if((req->req.dma % 4) != 0) {
			if(!ep->ep_dir) {
				prefetchw(req->req.buf);
//				printk("%s : rx done %x len (%d/%d) \n", ep->ep.name, req->req.dma, req->req.actual, req->req.length);
				memcpy(req->req.buf, ep->ep_buf, req->req.actual);
			} else {
//				printk("%s : tx done %x len (%d/%d) \n", ep->ep.name, req->req.dma, req->req.actual, req->req.length);
			}
		}
#else
		usb_gadget_unmap_request(&udc->gadget, &req->req, ep->ep_dir);

#endif
		ast_udc_done(ep, req, 0);
		if(!list_empty(&ep->queue)) {
//			req = list_entry(ep->queue.next, struct ast_udc_request, queue);
			req = list_first_entry(&ep->queue, struct ast_udc_request, queue);
//			printk("%s next req %x \n", ep->ep.name, req);
			ast_udc_ep_dma(ep, req);
		}		
	} else {
		if((len < ep->ep.maxpacket)) {
#if 1			
			usb_gadget_unmap_request(&udc->gadget, &req->req, ep->ep_dir);
			if((req->req.dma % 4) != 0) {
				if(!ep->ep_dir) {
//					printk("%s : rx done %x len (%d/%d) \n", ep->ep.name, req->req.dma, req->req.actual, req->req.length);
					memcpy(req->req.buf, ep->ep_buf, req->req.actual);
				} else {
//					printk("%s : tx done %x len (%d/%d) \n", ep->ep.name, req->req.dma, req->req.actual, req->req.length);
				}
			}
#else		
			usb_gadget_unmap_request(&udc->gadget, &req->req, ep->ep_dir);			

#endif		
			ast_udc_done(ep, req, 0);
			if(!list_empty(&ep->queue)) {
//				req = list_entry(ep->queue.next, struct ast_udc_request, queue);
				req = list_first_entry(&ep->queue, struct ast_udc_request, queue);
//				printk("%s next req %x \n", ep->ep.name, req);
				ast_udc_ep_dma(ep, req);
			}			
		} else {
			//next 			
			ast_udc_ep_dma(ep, req);
		}
	}

	

}

void ast_udc_setup_handle(struct ast_udc *udc)
{
	u16 ep_num = 0;
	SETUP_DBG("type : %x, req : %x, val : %x, idx: %x, len : %d  \n", 
		udc->root_setup->bRequestType, 
		udc->root_setup->bRequest, 
		udc->root_setup->wValue,
		udc->root_setup->wIndex,
		udc->root_setup->wLength);

	udc->ep[0].ep_dir = udc->root_setup->bRequestType & USB_DIR_IN;

	if ((udc->root_setup->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD) {
		switch (udc->root_setup->bRequest) {
			case USB_REQ_SET_ADDRESS:
				if (ast_udc_read(udc, AST_VHUB_USB_STS) & (0x1 << 27))
					udc->gadget.speed = USB_SPEED_HIGH;
				else
					udc->gadget.speed = USB_SPEED_FULL;
				
				SETUP_DBG("set addr %x \n", udc->root_setup->wValue);
				ast_udc_write(udc, udc->root_setup->wValue, AST_VHUB_CONF); 
				ast_udc_write(udc, EP0_TX_BUFF_RDY, AST_VHUB_EP0_CTRL);
				break;
			case USB_REQ_CLEAR_FEATURE:
				ep_num = udc->root_setup->wIndex & USB_ENDPOINT_NUMBER_MASK;
				printk("USB_REQ_CLEAR_FEATURE ep-%d\n", ep_num);
				ast_udc_write(udc, (ep_num - 1), AST_VHUB_EP_DATA);
				ast_ep_write(&udc->ep[ep_num], ast_ep_read(&udc->ep[ep_num], AST_EP_CONFIG) & ~EP_SET_EP_STALL, AST_EP_CONFIG);
				ast_udc_write(udc, EP0_TX_BUFF_RDY, AST_VHUB_EP0_CTRL);				
				break;
			case USB_REQ_SET_FEATURE:
				printk("USB_REQ_SET_FEATURE ep-%d\n", udc->root_setup->wIndex & USB_ENDPOINT_NUMBER_MASK);
				break;
			default:
				spin_unlock(&udc->lock);
				if(udc->driver->setup(&udc->gadget, udc->root_setup) < 0) {
					ast_udc_write(udc, ast_udc_read(udc, AST_VHUB_EP0_CTRL) | EP0_STALL, AST_VHUB_EP0_CTRL);
					printk("TODO XXXXXXXXXXXXXXXXXXXXXXXXXX udc->root_setup->bRequest %d \n", udc->root_setup->bRequest);
				}
				spin_lock(&udc->lock);
			break;
		}
	} else {
		switch (udc->root_setup->bRequest) {
			default:
				spin_unlock(&udc->lock);
				if(udc->driver->setup(&udc->gadget, udc->root_setup) < 0) {
					ast_udc_write(udc, ast_udc_read(udc, AST_VHUB_EP0_CTRL) | EP0_STALL, AST_VHUB_EP0_CTRL);
					printk("CLASS TODO udc->root_setup->bRequest %d \n", udc->root_setup->bRequest);
				}
				spin_lock(&udc->lock);
				break;
		}
	}	
}

static irqreturn_t ast_udc_irq (int irq, void *data)
{
	struct ast_udc	*udc = (struct ast_udc*) data;
	u32 isr = ast_udc_read(udc, AST_VHUB_ISR);
	u32 ep_isr = 0;
	int i = 0;
	
	spin_lock(&udc->lock);

	if(isr & ISR_BUS_RESET) {
		BUS_DBG("ISR_BUS_RESET \n");
		ast_udc_write(udc, ISR_BUS_RESET, AST_VHUB_ISR);
		udc->gadget.speed = USB_SPEED_UNKNOWN;
		if (udc->driver && udc->driver->reset) {
			spin_unlock(&udc->lock);
			udc->driver->reset(&udc->gadget);
			spin_lock(&udc->lock);
		}		
	}

	if(isr & ISR_BUS_SUSPEND) {
		//Suspend, we don't handle this in sample
		BUS_DBG("ISR_BUS_SUSPEND \n");
		ast_udc_write(udc, ISR_BUS_SUSPEND, AST_VHUB_ISR);
		if (udc->driver && udc->driver->suspend) {
			spin_unlock(&udc->lock);
			udc->driver->suspend(&udc->gadget);
			spin_lock(&udc->lock);
		}
	}

	if(isr & ISR_SUSPEND_RESUME) {
		//Suspend, we don't handle this in sample
		BUS_DBG("ISR_SUSPEND_RESUME \n");
		ast_udc_write(udc, ISR_SUSPEND_RESUME, AST_VHUB_ISR);
		if (udc->driver && udc->driver->resume) {
			spin_unlock(&udc->lock);
			udc->driver->resume(&udc->gadget);
			spin_lock(&udc->lock);
		}
	}

	if(isr & ISR_HUB_EP0_IN_ACK_STALL) {
		ISR_DBG("ISR_HUB_EP0_IN_ACK_STALL \n");
		ast_udc_write(udc, ISR_HUB_EP0_IN_ACK_STALL, AST_VHUB_ISR);
		ast_udc_ep0_in(udc);
	}

	if(isr & ISR_HUB_EP0_OUT_ACK_STALL) {
		ISR_DBG("ISR_HUB_EP0_OUT_ACK_STALL \n");
		ast_udc_write(udc, ISR_HUB_EP0_OUT_ACK_STALL, AST_VHUB_ISR);
		ast_udc_ep0_out(udc);
	}

	if(isr & ISR_HUB_EP0_OUT_NAK) {
//		ISR_DBG("ISR_HUB_EP0_OUT_NAK \n");
		ast_udc_write(udc, ISR_HUB_EP0_OUT_NAK, AST_VHUB_ISR);
	}

	if(isr & ISR_HUB_EP0_IN_DATA_NAK) {
		//IN NAK, we don't handle this in sample
//		ISR_DBG("ISR_HUB_EP0_IN_DATA_ACK \n");
		ast_udc_write(udc, ISR_HUB_EP0_IN_DATA_NAK, AST_VHUB_ISR);
	}

	if(isr & ISR_HUB_EP0_SETUP) {
		ISR_DBG("SETUP \n");
		ast_udc_write(udc, ISR_HUB_EP0_SETUP, AST_VHUB_ISR);
		ast_udc_setup_handle(udc);
	}

	if(isr & ISR_HUB_EP1_IN_DATA_ACK) {
		//HUB Bitmap control
		printk("ERROR EP1 IN\n");
		ast_udc_write(udc, ISR_HUB_EP1_IN_DATA_ACK, AST_VHUB_ISR);
		ast_udc_write(udc, 0x00, AST_VHUB_EP1_STS_CHG);
	}

	if(isr & ISR_DEVICE1) {
		printk("ISR_DEVICE1\n");
	}

	if(isr & ISR_DEVICE2) {
		printk("ISR_DEVICE2\n");
	}

	if(isr & ISR_DEVICE3) {
		printk("ISR_DEVICE3 \n");
	}

	if(isr & ISR_DEVICE4) {
		printk("ISR_DEVICE4 \n");
	}

	if(isr & ISR_DEVICE5) {
		printk("ISR_DEVICE5 \n");
	}

	if(isr & ISR_EP_ACK_STALL) {
//		EP_DBG("ISR_EP_ACK_STALL\n");
//		ast_udc_write(udc, ISR_EP_ACK_STALL, AST_VHUB_ISR); 
		ep_isr = ast_udc_read(udc, AST_VHUB_EP_ACK_ISR);
		for(i = 0; i < 15; i++) {
			if(ep_isr & (0x1 << i)) {
				ast_udc_write(udc, 0x1 <<  i, AST_VHUB_EP_ACK_ISR); 
				ast_udc_ep_handle(udc, i + 1);			
			}
		}
	}

	if(isr & ISR_EP_NAK) {
		ISR_DBG("ISR_EP_NAK ****************************************\n");
		ast_udc_write(udc, ISR_EP_NAK, AST_VHUB_ISR);
	}

	spin_unlock(&udc->lock);
	return IRQ_HANDLED;

}
/*-------------------------------------------------------------------------*/
static int ast_udc_gadget_getframe(struct usb_gadget *gadget)
{
	struct ast_udc	*udc = container_of(gadget, struct ast_udc, gadget);

	UDC_DBG("\n");
	return (ast_udc_read(udc, AST_VHUB_USB_STS) >> 16) & 0x7ff;
}

static int ast_udc_wakeup(struct usb_gadget *gadget)
{
	UDC_DBG("TODO ~~~~~\n");
	return 0;
}

/*
 * activate/deactivate link with host; minimize power usage for
 * inactive links by cutting clocks and transceiver power.
 */

static int ast_udc_pullup(struct usb_gadget *gadget, int is_on)
{
	struct ast_udc	*udc = container_of(gadget, struct ast_udc, gadget);

	UDC_DBG("%d \n", is_on);

	if(is_on)
		ast_udc_write(udc, ast_udc_read(udc, AST_VHUB_CTRL) | ROOT_UPSTREAM_EN, AST_VHUB_CTRL);	
	else
		ast_udc_write(udc, ast_udc_read(udc, AST_VHUB_CTRL) & ~ROOT_UPSTREAM_EN, AST_VHUB_CTRL);	

	return 0;
}

static int ast_udc_start(struct usb_gadget *gadget,
		struct usb_gadget_driver *driver)
{
	struct ast_udc	*udc = container_of(gadget, struct ast_udc, gadget);

	UDC_DBG("\n");

	if (!udc)
		return -ENODEV;

	if (udc->driver)
		return -EBUSY;

	udc->driver = driver;

	udc->gadget.dev.of_node = udc->pdev->dev.of_node;
//	udc->gadget.is_selfpowered = 1;

	return 0;
}

static int ast_udc_stop(struct usb_gadget *gadget,
		struct usb_gadget_driver *driver)
{
	struct ast_udc *udc = container_of(gadget, struct ast_udc, gadget);
	unsigned long	flags;

	UDC_DBG("\n");

	spin_lock_irqsave(&udc->lock, flags);
	ast_udc_write(udc, ast_udc_read(udc, AST_VHUB_CTRL) & ~ROOT_UPSTREAM_EN, AST_VHUB_CTRL);	
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	ast_udc_stop_activity(udc);
	spin_unlock_irqrestore(&udc->lock, flags);

	udc->driver = NULL;

	return 0;
}

static const struct usb_gadget_ops ast_udc_ops = {
	.get_frame		= ast_udc_gadget_getframe,
	.wakeup			= ast_udc_wakeup,
	.pullup			= ast_udc_pullup,
	.udc_start		= ast_udc_start,
	.udc_stop		= ast_udc_stop,
};
/*-------------------------------------------------------------------------*/
static void ast_udc_init(struct ast_udc *udc) 
{

	ast_udc_write(udc, ROOT_PHY_CLK_EN | ROOT_PHY_RESET_DIS, AST_VHUB_CTRL);
	
	udelay(1);
	ast_udc_write(udc, 0, AST_VHUB_DEV_RESET);

	ast_udc_write(udc, 0x1ffff, AST_VHUB_IER);
	
	ast_udc_write(udc, 0x1ffff, AST_VHUB_ISR);

	ast_udc_write(udc, 0x7ffff, AST_VHUB_EP_ACK_ISR);		
	
	ast_udc_write(udc, 0x7ffff, AST_VHUB_EP_ACK_IER);	
	ast_udc_write(udc, 0, AST_VHUB_EP0_CTRL);
//	ast_udc_write(udc, 0x1, AST_VHUB_EP1_CTRL);
	ast_udc_write(udc, 0, AST_VHUB_EP1_CTRL);
}

static const struct ast_udc_config ast_config = { 
	.end_point_num = 16, 
};

static const struct ast_udc_config ast1220_config = { 
	.end_point_num = 5, 
};

static const struct of_device_id ast_udc_of_dt_ids[] = {
	{ .compatible = "aspeed,ast-udc", 		.data = &ast_config, },
	{ .compatible = "aspeed,ast1220-udc",	.data = &ast1220_config, },
	{}
};

MODULE_DEVICE_TABLE(of, ast_udc_of_dt_ids);

#define EP_DMA_SIZE		2048

static int ast_udc_probe(struct platform_device *pdev)
{
	struct ast_udc	*udc;
	struct ast_udc_ep	*ep;		
	int				ret = 0;
	const struct of_device_id *udc_dev_id;	
	struct resource	*res;

	int i;

	UDC_DBG(" \n");

	udc = devm_kzalloc(&pdev->dev, sizeof(struct ast_udc), GFP_KERNEL);
	if (!udc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("platform_get_resource error.\n");
		return -ENODEV;
	}

	udc->irq = platform_get_irq(pdev, 0);
	if (!udc->irq) {
		pr_err("platform_get_resource IORESOURCE_IRQ error.\n");
		return -ENODEV;
	}

	udc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(udc->clk)) {
		dev_err(&pdev->dev, "no clock defined \n");
		return -ENODEV;
	}

	clk_prepare_enable(udc->clk);

	udc_dev_id = of_match_node(ast_udc_of_dt_ids, pdev->dev.of_node);
	if (!udc_dev_id)
		return -EINVAL;

	udc->udc_config = (struct ast_udc_config*) udc_dev_id->data;

	/* init software state */
	udc->pdev = pdev;

	spin_lock_init(&udc->lock);
	udc->ep = devm_kzalloc(&pdev->dev, sizeof(struct ast_udc_ep) * udc->udc_config->end_point_num,GFP_KERNEL);
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->gadget.max_speed = USB_SPEED_HIGH;
	udc->gadget.dev.parent = &pdev->dev;
	udc->gadget.ops = &ast_udc_ops;
	udc->gadget.ep0 = &udc->ep[0].ep;
	udc->gadget.name = "ast-udc";
	udc->gadget.dev.init_name = "gadget";

	udc->reg = devm_ioremap_resource(&pdev->dev, res);
	if (udc->reg == NULL) {
		pr_err("ioremap error.\n");
		goto err_map;
	}

	udc->root_setup = udc->reg + 0x80;

	INIT_LIST_HEAD(&udc->gadget.ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);

	//1024 * 16
	udc->ep0_ctrl_buf = dma_alloc_coherent(udc->gadget.dev.parent,
					EP_DMA_SIZE * udc->udc_config->end_point_num, &udc->ep0_ctrl_dma, GFP_KERNEL);

	for (i = 0; i < udc->udc_config->end_point_num; i++) {
		ep = &udc->ep[i];
		ep->ep.name = ast_ep_name[i];
		if(i == 0) {
			ep->ep.caps.type_control = true;
		} else {
			ep->ep.caps.type_iso = true;
			ep->ep.caps.type_bulk = true;
			ep->ep.caps.type_int = true;
		}
		ep->ep.caps.dir_in = true;
		ep->ep.caps.dir_out = true;

		ep->ep.ops = &ast_udc_ep_ops;
		ep->udc = udc;
		if(i) {
			ep->ep_reg = udc->reg + 0x200 + (0x10 * (i - 1));

			ep->ep_buf = udc->ep0_ctrl_buf + (i * EP_DMA_SIZE);
			ep->ep_dma = udc->ep0_ctrl_dma + (i * EP_DMA_SIZE);
			usb_ep_set_maxpacket_limit(&ep->ep, 1024);
		} else {
			ep->ep_reg = 0;
			usb_ep_set_maxpacket_limit(&ep->ep, 64);		
		}
		printk("%s : maxpacket  %d, dma: %x \n ", ep->ep.name, ep->ep.maxpacket, ep->ep_dma);

		if(i) 
			list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);

		INIT_LIST_HEAD(&ep->queue);				
	}

	//ast udc initial 
	ast_udc_init(udc);

	/* request UDC and maybe VBUS irqs */
	ret = devm_request_irq(&pdev->dev, udc->irq, ast_udc_irq, 0,
			      KBUILD_MODNAME, udc);
	if (ret < 0) {
		printk("request irq %d failed\n", udc->irq);
		goto fail1;
	}

	ret = usb_add_gadget_udc(&pdev->dev, &udc->gadget);
	if (ret)
		goto fail1;
	dev_set_drvdata(&pdev->dev, udc);
	device_init_wakeup(&pdev->dev, 1);

	printk(KERN_INFO "ast_udc: driver successfully loaded.\n");
	return 0;

fail1:
	free_irq(udc->irq, udc);

err_map:
	if (udc->reg)
		iounmap(udc->reg);

	kfree(udc);

	printk("ast udc probe failed, %d\n", ret);
	
	return ret;
}

static int __exit ast_udc_remove(struct platform_device *pdev)
{
	struct ast_udc *udc = platform_get_drvdata(pdev);
	struct resource *res;
	unsigned long	flags;

	usb_del_gadget_udc(&udc->gadget);
	if (udc->driver)
		return -EBUSY;

	spin_lock_irqsave(&udc->lock, flags);
	ast_udc_write(udc, ast_udc_read(udc, AST_VHUB_CTRL) & ~ROOT_UPSTREAM_EN, AST_VHUB_CTRL);
	spin_unlock_irqrestore(&udc->lock, flags);

	device_init_wakeup(&pdev->dev, 0);
	free_irq(udc->irq, udc);

	iounmap(udc->reg);
	devm_kfree(&pdev->dev, udc->ep);
	devm_kfree(&pdev->dev, udc);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));


	return 0;
}

#ifdef CONFIG_PM
static int ast_udc_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct ast_udc *udc = platform_get_drvdata(pdev);
	int		wake = udc->driver && device_may_wakeup(&pdev->dev);
	unsigned long	flags;
	printk("TODO ~~");
	/* Unless we can act normally to the host (letting it wake us up
	 * whenever it has work for us) force disconnect.  Wakeup requires
	 * PLLB for USB events (signaling for reset, wakeup, or incoming
	 * tokens) and VBUS irqs (on systems which support them).
	 */
	if ((!udc->suspended && udc->addr)
			|| !wake) {
		spin_lock_irqsave(&udc->lock, flags);
		pullup(udc, 0);
		wake = 0;
		spin_unlock_irqrestore(&udc->lock, flags);
	} else
		enable_irq_wake(udc->irq);

	udc->active_suspend = wake;
	if (gpio_is_valid(udc->board.vbus_pin) && !udc->board.vbus_polled && wake)
		enable_irq_wake(udc->board.vbus_pin);
	return 0;
}

static int ast_udc_resume(struct platform_device *pdev)
{
	struct ast_udc *udc = platform_get_drvdata(pdev);
	unsigned long	flags;
	printk("TODO ~~");
	if (!udc->board.vbus_polled &&
	    udc->active_suspend)
		disable_irq_wake(udc->board.vbus_pin);

	/* maybe reconnect to host; if so, clocks on */
	if (udc->active_suspend)
		disable_irq_wake(udc->irq);
	else {
		spin_lock_irqsave(&udc->lock, flags);
		pullup(udc, 1);
		spin_unlock_irqrestore(&udc->lock, flags);
	}
	return 0;
}
#endif

static struct platform_driver ast_udc_driver = {
	.probe 		= ast_udc_probe,
	.remove		= ast_udc_remove,
#ifdef CONFIG_PM		
	.suspend		= ast_udc_suspend,
	.resume		= ast_udc_resume,
#endif	
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table	= ast_udc_of_dt_ids,
	},
};

module_platform_driver(ast_udc_driver);

MODULE_DESCRIPTION("AST UDC driver");
MODULE_AUTHOR("Ryan Chen");
MODULE_LICENSE("GPL");

#if 0
/* hid descriptor for a keyboard */
static struct hidg_func_descriptor my_keyboard_data = {
	.subclass		= 0, /* No subclass */
	.protocol		= 1, /* Keyboard */
	.report_length		= 8,
	.report_desc_length	= 63,
	.report_desc		= {
		0x05, 0x01,	/* USAGE_PAGE (Generic Desktop)	          */
		0x09, 0x06,	/* USAGE (Keyboard)                       */
		0xa1, 0x01,	/* COLLECTION (Application)               */
		0x05, 0x07,	/*   USAGE_PAGE (Keyboard)                */
		0x19, 0xe0,	/*   USAGE_MINIMUM (Keyboard LeftControl) */
		0x29, 0xe7,	/*   USAGE_MAXIMUM (Keyboard Right GUI)   */
		0x15, 0x00,	/*   LOGICAL_MINIMUM (0)                  */
		0x25, 0x01,	/*   LOGICAL_MAXIMUM (1)                  */
		0x75, 0x01,	/*   REPORT_SIZE (1)                      */
		0x95, 0x08,	/*   REPORT_COUNT (8)                     */
		0x81, 0x02,	/*   INPUT (Data,Var,Abs)                 */
		0x95, 0x01,	/*   REPORT_COUNT (1)                     */
		0x75, 0x08,	/*   REPORT_SIZE (8)                      */
		0x81, 0x03,	/*   INPUT (Cnst,Var,Abs)                 */
		0x95, 0x05,	/*   REPORT_COUNT (5)                     */
		0x75, 0x01,	/*   REPORT_SIZE (1)                      */
		0x05, 0x08,	/*   USAGE_PAGE (LEDs)                    */
		0x19, 0x01,	/*   USAGE_MINIMUM (Num Lock)             */
		0x29, 0x05,	/*   USAGE_MAXIMUM (Kana)                 */
		0x91, 0x02,	/*   OUTPUT (Data,Var,Abs)                */
		0x95, 0x01,	/*   REPORT_COUNT (1)                     */
		0x75, 0x03,	/*   REPORT_SIZE (3)                      */
		0x91, 0x03,	/*   OUTPUT (Cnst,Var,Abs)                */
		0x95, 0x06,	/*   REPORT_COUNT (6)                     */
		0x75, 0x08,	/*   REPORT_SIZE (8)                      */
		0x15, 0x00,	/*   LOGICAL_MINIMUM (0)                  */
		0x25, 0x65,	/*   LOGICAL_MAXIMUM (101)                */
		0x05, 0x07,	/*   USAGE_PAGE (Keyboard)                */
		0x19, 0x00,	/*   USAGE_MINIMUM (Reserved)             */
		0x29, 0x65,	/*   USAGE_MAXIMUM (Keyboard Application) */
		0x81, 0x00,	/*   INPUT (Data,Ary,Abs)                 */
		0xc0		/* END_COLLECTION                         */
	}
};

static struct platform_device my_keyboard_hid = {
	.name			= "hidg",
	.id			= 0,
	.num_resources		= 0,
	.resource		= 0,
	.dev.platform_data	= &my_keyboard_data,
};

static struct hidg_func_descriptor my_mouse_data = {
	.subclass = 0,
	.protocol = 2,
	.report_length = 4,
	.report_desc_length= 52,
	.report_desc={
		0x05,0x01,	/*Usage Page (Generic Desktop Controls)*/
		0x09,0x02,	/*Usage (Mouse)*/
		0xa1,0x01,	/*Collction (Application)*/
		0x09,0x01,	/*Usage (pointer)*/
		0xa1,0x00,	/*Collction (Physical)*/
		0x05,0x09,	/*Usage Page (Button)*/
		0x19,0x01,	/*Usage Minimum(1)*/
		0x29,0x03,	/*Usage Maximum(3) */ 
		0x15,0x00,	/*Logical Minimum(1)*/
		0x25,0x01,	/*Logical Maximum(1)*/
		0x95,0x03,	/*Report Count(5)  */
		0x75,0x01,	/*Report Size(1)*/
		0x81,0x02,	/*Input(Data,Variable,Absolute,BitFiled)*/
		0x95,0x01,	/*Report Count(1)*/
		0x75,0x05,	/*Report Size(5) */
		0x81,0x01,	/*Input(Constant,Array,Absolute,BitFiled) */
		0x05,0x01,	/*Usage Page (Generic Desktop Controls)*/
		0x09,0x30,	/*Usage(x)*/
		0x09,0x31,	/*Usage(y)*/
		0x09,0x38,	/*Usage(Wheel)*/
		0x15,0x81,	/*Logical Minimum(-127)*/
		0x25,0x7f,	/*Logical Maximum(127)*/
		0x75,0x08,	/*Report Size(8)*/
		0x95,0x02,	/*Report Count(2)  */
		0x81,0x06,	/*Input(Data,Variable,Relative,BitFiled)*/
		0xc0,	/*End Collection*/
		0xc0	/*End Collection*/
	}
};

static struct platform_device my_mouse_hid = {
        .name = "hidg",
        .id = 1,
        .num_resources = 0,
        .resource = 0,
        .dev = {
                .platform_data = &my_mouse_data,
        }
};

#endif
