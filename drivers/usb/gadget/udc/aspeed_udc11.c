// SPDX-License-Identifier: GPL-2.0+
/*
 * aspeed-vhub -- Driver for Aspeed SoC USB 1.1 HID gadget
 *
 *  Copyright (C) ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
#include <linux/usb/gadget.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_data/atmel.h>
#include <linux/regmap.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/prefetch.h>
#include <linux/io.h>

#include <linux/usb.h>
#include <linux/usb/gadget.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>


#define	AST_UDC11_CTRL			0x00	/* Function Control and Status Register */
#define	AST_UDC11_CONF			0x04	/* Function Configuration Setting Register */
#define	AST_UDC11_REST			0x08	/* Endpoint Toggle Bit Reset Register */
#define	AST_UDC11_STS			0x0C	/* USB Status Register */
#define	AST_UDC11_IER			0x10	/* Interrupt Control Register */
#define	AST_UDC11_ISR			0x14	/* Interrupt Status Register */
#define AST_UDC11_EP0_CTRL		0x18	/* Endpoint 0 Control and Status Register */
#define AST_UDC11_EP1_CTRL		0x1C	/* Endpoint 1 Control and Status Register */
#define AST_UDC11_EP2_CTRL		0x20	/* Endpoint 2 Control and Status Register */
#define AST_UDC11_EP0_SETUP0	0x24	/* Endpoint 0 Setup/OUT Data Buffer LOW Register */
#define AST_UDC11_EP0_SETUP1	0x28	/* Endpoint 0 Setup/OUT Data Buffer HIGH Register */
#define AST_UDC11_EP0_DATA0		0x2C	/* Endpoint 0 IN DATA Buffer LOW Register */
#define AST_UDC11_EP0_DATA1		0x30	/* Endpoint 0 IN DATA Buffer HIGH Register */
#define AST_UDC11_EP1_DATA0		0x34	/* Endpoint 1 IN DATA Buffer LOW Register */
#define AST_UDC11_EP1_DATA1		0x38	/* Endpoint 1 IN DATA Buffer HIGH Register */
#define AST_UDC11_EP2_DATA0		0x3C	/* Endpoint 2 IN DATA Buffer LOW Register */
#define AST_UDC11_EP2_DATA1		0x40	/* Endpoint 2 IN DATA Buffer HIGH Register */

/* AST_UDC11_CTRL			0x00		Function Control and Status Register */
#define UDC11_CTRL_TEST_RESULT			(1 << 10)
#define UDC11_CTRL_TEST_STS				(1 << 9)
#define UDC11_CTRL_TEST_MODE(x)			((x) << 6)
#define UDC11_CTRL_WKP(x)				((x) << 4)
#define UDC11_CTRL_WKP_EN				(1 << 3)
#define UDC11_CTRL_CLK_STOP				(1 << 2)
#define UDC11_CTRL_LS_EN				(1 << 1)
#define UDC11_CTRL_CONNECT_EN			(1)

/* AST_UDC11_CONF			0x04		Function Configuration Setting Register */
#define UDC11_CONF_ADDR_MASK			(0x3f << 1)
#define UDC11_CONF_SET_ADDR(x)			(x << 1)
#define UDC11_CONF_SET_CONF				(1)

/* AST_UDC11_REST			0x08		Endpoint Toggle Bit Reset Register */
#define UDC11_REST_EP2					(1 << 1)
#define UDC11_REST_EP1					(1)


/* AST_UDC11_STS			0x0C	USB Status Register */
#define UDC11_STS_SUSPEND				(1 << 31)
#define UDC11_STS_BUS_RST				(1 << 30)
#define UDC11_STS_LINE_DP				(1 << 29)
#define UDC11_STS_LINE_DN				(1 << 28)
#define UDC11_STS_FRAM_NUM_MASK			(0x7ff << 16)
#define UDC11_STS_GET_FRAM_NUM(x)		((x >> 16) & 0x7ff)
#define UDC11_STS_LAST_ADDR				(0x7f << 4)
#define UDC11_STS_LAST_EP				(0xf)

/* AST_UDC11_IER			0x10		Interrupt Control Register */
/* AST_UDC11_ISR			0x14		Interrupt Status Register */
#define UDC11_EP0_OUT				(1 << 9)
#define UDC11_EP0_NAK				(1 << 8)
#define UDC11_EP2_IN_ACK			(1 << 7)
#define UDC11_EP1_IN_ACK			(1 << 6)
#define UDC11_EP0_IN_ACK			(1 << 5)
#define UDC11_EP0_OUT_ACK			(1 << 4)
#define UDC11_EP0_SETUP				(1 << 3)
#define UDC11_SUSPEND_RESUME		(1 << 2)
#define UDC11_SUSPEND_ENTRY			(1 << 1)
#define UDC11_BUS_REST				(1)

/* AST_UDC11_EP0_CTRL		0x18		Endpoint 0 Control and Status Register */
/* AST_UDC11_EP1_CTRL		0x1C	Endpoint 1 Control and Status Register */
/* AST_UDC11_EP2_CTRL		0x20		Endpoint 2 Control and Status Register */
#define GET_EP_OUT_RX_LEN(x)		((x & 0xf) >> 8)	//only for EP0
#define GET_EP_IN_TX_LEN(x)			((x & 0xf) >> 4)	
#define SET_EP_IN_TX_LEN(x)			((x & 0xf) << 4)
#define EP_OUT_BUFF_RX_RDY			(1 << 2)		//only for EP0
#define EP_IN_BUFF_TX_RDY			(1 << 1)
#define EP_CTRL_STALL

static const char ep0name[] = "ep0";

/*
 * controller driver data structures
 */

#define	NUM_ENDPOINTS	3

enum ep0_state {
        EP0_IDLE,
        EP0_IN_DATA_PHASE,
        EP0_OUT_DATA_PHASE,
        EP0_END_XFER,
        EP0_STALL,
};

struct ast_ep {
	struct usb_ep			ep;
	struct list_head		queue;
	struct aspeed_hid			*udc;
	u8				int_mask;
	unsigned			is_pingpong:1;
	unsigned			stopped:1;
	const struct usb_endpoint_descriptor
					*desc;
};

/*
 * driver is non-SMP, and just blocks IRQs whenever it needs
 * access protection for chip registers or driver state
 */
struct aspeed_hid {
	struct usb_gadget		gadget;
	struct ast_ep			ep[NUM_ENDPOINTS];
	struct usb_gadget_driver	*driver;
struct clk                      *clk;	
	unsigned			enabled:1;
	unsigned			suspended:1;
	unsigned			active_suspend:1;

	struct platform_device		*pdev;
	struct proc_dir_entry		*pde;
	void __iomem			*reg_base;
	int				udp_irq;
	spinlock_t			lock;
	u16				addr;	
	u8				ep0state;	
	u8				vbus;
};

struct ast_udc_req {
	struct usb_request		req;
	struct list_head		queue;
};

static inline struct aspeed_hid *to_udc(struct usb_gadget *g)
{
	return container_of(g, struct aspeed_hid, gadget);
}


/*-------------------------------------------------------------------------*/
#define AST_UDC11_DEBUG 1
#ifdef AST_UDC11_DEBUG
#define UDC_DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#define PKG_DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define UDC_DBG(fmt, args...)
#define PKG_DBG(fmt, args...)
#endif

#define VBUS_POLL_TIMEOUT	msecs_to_jiffies(1000)

static inline void
ast_udc_write(struct aspeed_hid *udc, u32 val, u32 reg)
{
	writel(val, udc->reg_base+ reg);
}

static inline u32
ast_udc_read(struct aspeed_hid *udc, u32 reg)
{
#if 0
	u32 val = readl(udc->reg_base + reg);
	printk("R : reg %x , val: %x \n",reg, val);
	return val;
#else	
	return readl(udc->reg_base + reg);
#endif
}

/*-------------------------------------------------------------------------*/

//#ifdef CONFIG_USB_GADGET_DEBUG_FILES
#if 0
#include <linux/seq_file.h>

static const char debug_filename[] = "driver/udc";

#define FOURBITS "%s%s%s%s"
#define EIGHTBITS FOURBITS FOURBITS

static void proc_ep_show(struct seq_file *s, struct ast_ep *ep)
{
	static char		*types[] = {
		"control", "in-int", "in-int", "in-int"};
	u32			csr;
	struct ast_udc_req	*req;
	unsigned long	flags;
	struct aspeed_hid	*udc = ep->udc;

	spin_lock_irqsave(&udc->lock, flags);

	csr = __raw_readl(ep->creg);

	/* NOTE:  not collecting per-endpoint irq statistics... */

	seq_printf(s, "\n");
	seq_printf(s, "%s, maxpacket %d %s%s %s%s\n",
			ep->ep.name, ep->ep.maxpacket,
			ep->is_in ? "in" : "out",
			ep->is_pingpong
				? (ep->fifo_bank ? "pong" : "ping")
				: "",
			ep->stopped ? " stopped" : "");
	seq_printf(s, "csr %08x rxbytes=%d %s %s %s" EIGHTBITS "\n",
		csr,
		(csr & 0x07ff0000) >> 16,
		(csr & (1 << 15)) ? "enabled" : "disabled",
		(csr & (1 << 11)) ? "DATA1" : "DATA0",
		types[(csr & 0x700) >> 8],

		/* iff type is control then print current direction */
		(!(csr & 0x700))
			? ((csr & (1 << 7)) ? " IN" : " OUT")
			: "",
		(csr & (1 << 6)) ? " rxdatabk1" : "",
		(csr & (1 << 5)) ? " forcestall" : "",
		(csr & (1 << 4)) ? " txpktrdy" : "",

		(csr & (1 << 3)) ? " stallsent" : "",
		(csr & (1 << 2)) ? " rxsetup" : "",
		(csr & (1 << 1)) ? " rxdatabk0" : "",
		(csr & (1 << 0)) ? " txcomp" : "");
	if (list_empty (&ep->queue))
		seq_printf(s, "\t(queue empty)\n");

	else list_for_each_entry (req, &ep->queue, queue) {
		unsigned	length = req->req.actual;

		seq_printf(s, "\treq %p len %d/%d buf %p\n",
				&req->req, length,
				req->req.length, req->req.buf);
	}
	spin_unlock_irqrestore(&udc->lock, flags);
}

static void proc_irq_show(struct seq_file *s, const char *label, u32 mask)
{
	int i;

	seq_printf(s, "%s %04x:%s%s" FOURBITS, label, mask,
		(mask & (1 << 13)) ? " wakeup" : "",
		(mask & (1 << 12)) ? " endbusres" : "",

		(mask & (1 << 11)) ? " sofint" : "",
		(mask & (1 << 10)) ? " extrsm" : "",
		(mask & (1 << 9)) ? " rxrsm" : "",
		(mask & (1 << 8)) ? " rxsusp" : "");
	for (i = 0; i < 8; i++) {
		if (mask & (1 << i))
			seq_printf(s, " ep%d", i);
	}
	seq_printf(s, "\n");
}

static int proc_udc_show(struct seq_file *s, void *unused)
{
	struct aspeed_hid	*udc = s->private;
	struct ast_ep	*ep;
	u32		tmp;

	seq_printf(s, "%s: \n", "aspeed_hid");

	seq_printf(s, "vbus %s, pullup %s, %s powered%s, gadget %s\n\n",
		udc->vbus ? "present" : "off",
		udc->enabled
			? (udc->vbus ? "active" : "enabled")
			: "disabled",
		udc->selfpowered ? "self" : "VBUS",
		udc->suspended ? ", suspended" : "",
		udc->driver ? udc->driver->driver.name : "(none)");

	/* don't access registers when interface isn't clocked */
	if (!udc->clocked) {
		seq_printf(s, "(not clocked)\n");
		return 0;
	}

	tmp = ast_udc_read(udc, AT91_UDP_FRM_NUM);
	seq_printf(s, "frame %05x:%s%s frame=%d\n", tmp,
		(tmp & AT91_UDP_FRM_OK) ? " ok" : "",
		(tmp & AT91_UDP_FRM_ERR) ? " err" : "",
		(tmp & AT91_UDP_NUM));

	tmp = ast_udc_read(udc, AT91_UDP_GLB_STAT);
	seq_printf(s, "glbstate %02x:%s" FOURBITS "\n", tmp,
		(tmp & AT91_UDP_RMWUPE) ? " rmwupe" : "",
		(tmp & AT91_UDP_RSMINPR) ? " rsminpr" : "",
		(tmp & AT91_UDP_ESR) ? " esr" : "",
		(tmp & AT91_UDP_CONFG) ? " confg" : "",
		(tmp & AT91_UDP_FADDEN) ? " fadden" : "");

	tmp = ast_udc_read(udc, AT91_UDP_FADDR);
	seq_printf(s, "faddr   %03x:%s fadd=%d\n", tmp,
		(tmp & AT91_UDP_FEN) ? " fen" : "",
		(tmp & AT91_UDP_FADD));

	proc_irq_show(s, "imr   ", ast_udc_read(udc, AT91_UDP_IMR));
	proc_irq_show(s, "isr   ", ast_udc_read(udc, AT91_UDP_ISR));

	if (udc->enabled && udc->vbus) {
		proc_ep_show(s, &udc->ep[0]);
		list_for_each_entry (ep, &udc->gadget.ep_list, ep.ep_list) {
			if (ep->desc)
				proc_ep_show(s, ep);
		}
	}
	return 0;
}

static int proc_udc_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_udc_show, PDE(inode)->data);
}

static const struct file_operations proc_ops = {
	.owner		= THIS_MODULE,
	.open		= proc_udc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debug_file(struct aspeed_hid *udc)
{
	udc->pde = proc_create_data(debug_filename, 0, NULL, &proc_ops, udc);
}

static void remove_debug_file(struct aspeed_hid *udc)
{
	if (udc->pde)
		remove_proc_entry(debug_filename, NULL);
}

#else

static inline void create_debug_file(struct aspeed_hid *udc) {}
static inline void remove_debug_file(struct aspeed_hid *udc) {}

#endif


/*-------------------------------------------------------------------------*/

static void ast_udc_done(struct ast_ep *ep, struct ast_udc_req *req, int status)
{
	unsigned	stopped = ep->stopped;
	struct aspeed_hid	*udc = ep->udc;

	printk("%s\n", __func__ );

	printk("del queue \n");
	list_del_init(&req->queue);
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;
	
	if (status && status != -ESHUTDOWN)
		UDC_DBG("%s done %p, status %d\n", ep->ep.name, req, status);

	ep->stopped = 1;
	spin_unlock(&udc->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock(&udc->lock);
	ep->stopped = stopped;

}

/*-------------------------------------------------------------------------*/
static void nuke(struct ast_ep *ep, int status)
{
	struct ast_udc_req 	*req;
	printk("%s\n", __func__ );
	
	// terminer chaque requete dans la queue
	ep->stopped = 1;
	if (list_empty(&ep->queue))
		return;

	UDC_DBG("%s %s\n", __func__, ep->ep.name);
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct ast_udc_req, queue);
		ast_udc_done(ep, req, status);
	}
}

/*-------------------------------------------------------------------------*/
static int ast_ep_enable(struct usb_ep *_ep,
				const struct usb_endpoint_descriptor *desc)
{
	struct ast_ep	*ep = container_of(_ep, struct ast_ep, ep);
	struct aspeed_hid	*udc = ep->udc;
	unsigned long	flags;
	printk("*************************************** %s : %s -----\n", __func__, _ep->name);

	/* initialize endpoint to match this descriptor */
//	ep->is_in = usb_endpoint_dir_in(desc);

	ep->stopped = 0;
	ep->desc = desc;
	ep->ep.maxpacket = 8;
	udc->gadget.speed = USB_SPEED_FULL;

	return 0;
}

static int ast_ep_disable (struct usb_ep * _ep)
{
	struct ast_ep	*ep = container_of(_ep, struct ast_ep, ep);
	struct aspeed_hid	*udc = ep->udc;
	unsigned long	flags;

	printk("%s\n", __func__ );

	nuke(ep, -ESHUTDOWN);

	/* restore the endpoint's pristine config */
	ep->desc = NULL;
	ep->stopped = 1;

	return 0;
}

static int ast_udc_read_fifo(struct ast_ep *ep, struct ast_udc_req *req)
{
    unsigned        total, count;

	UDC_DBG("%s : read_fifo , actual %d , len = %d, \n",ep->ep.name, req->req.actual, req->req.length);

	if(req->req.actual == req->req.length) {
		if(req->req.length == 0) {
			UDC_DBG("TODO ....... \n");
			ast_udc_write(ep->udc, 0, AST_UDC11_EP1_CTRL);
			ast_udc_write(ep->udc, SET_EP_IN_TX_LEN(0) | EP_IN_BUFF_TX_RDY, AST_UDC11_EP1_CTRL);
			return 0;
			
		} else {
			UDC_DBG("complete done \n");
			ast_udc_done(ep, req, 0);
			return 1;
		}
	}

	if (!req->req.length) {
		printk("zero len \n");			
		return 1;
	}

	total = req->req.length - req->req.actual;

	if (ep->ep.maxpacket < total) {
		count = ep->ep.maxpacket;
	} else {
		count = total;
	}

	if(ep->ep.name == ep0name) {
		printk("ep 0 read fifo error \n");
	} else {
		printk("ep read fifo eeeee\n");
	}

	return 0;
}
static int ast_udc_write_fifo(struct ast_ep *ep, struct ast_udc_req *req)
{
    unsigned        tx_len, count;

	UDC_DBG("%s : write_fifo , actual %d , len = %d, \n",ep->ep.name, req->req.actual, req->req.length);

	if(req->req.actual == req->req.length) {
		if(req->req.length == 0) {
			UDC_DBG("TX zero len package : list add queue \n");
			list_add_tail(&req->queue, &ep->queue);
			ast_udc_write(ep->udc, 0, AST_UDC11_EP0_CTRL);
			ast_udc_write(ep->udc, SET_EP_IN_TX_LEN(0) | EP_IN_BUFF_TX_RDY, AST_UDC11_EP0_CTRL);
			while(!ast_udc_read(ep->udc, AST_UDC11_ISR)& UDC11_EP0_IN_ACK); 
			ast_udc_write(ep->udc, UDC11_EP0_NAK | UDC11_EP0_IN_ACK, AST_UDC11_ISR);	
			ast_udc_done(ep, req, 0);
			return 1;
			
		} else {
			UDC_DBG("Please check ... \n");
//			ast_udc_done(ep, req, 0);
			//trigger rx 
//			ast_udc_write(ep->udc, EP_OUT_BUFF_RX_RDY, AST_UDC11_EP0_CTRL);
			return 0;
		}
	}
resend:
	tx_len = req->req.length - req->req.actual;

    if (ep->ep.maxpacket < tx_len) {
		count = ep->ep.maxpacket;
    } else {
        count = tx_len;
    }

#if 0 
	//TODO .....
	int i;
	for(i=0;i< count;i++ )
			printk(" %02x ",*(u8 * )(req->req.buf+i));

	printk("\n");
	
#endif
	printk("ep->ep.name %s , ep0name %s \n", ep->ep.name, ep0name);

	if(ep->ep.name == "ep0") {
		printk("ep 0 \n");
		while(ast_udc_read(ep->udc, AST_UDC11_EP0_CTRL)& EP_IN_BUFF_TX_RDY);
		ast_udc_write(ep->udc, *(u32 * )(req->req.buf + req->req.actual), AST_UDC11_EP0_DATA0);
		ast_udc_write(ep->udc, *(u32 * )(req->req.buf + req->req.actual + 4), AST_UDC11_EP0_DATA1);
		PKG_DBG("tx count %d , ep0 fifo : %x %x \n", count, ast_udc_read(ep->udc, AST_UDC11_EP0_DATA0), ast_udc_read(ep->udc, AST_UDC11_EP0_DATA1));
		req->req.actual += count;
		ast_udc_write(ep->udc, 0, AST_UDC11_EP0_CTRL);
		ast_udc_write(ep->udc, SET_EP_IN_TX_LEN(count) | EP_IN_BUFF_TX_RDY, AST_UDC11_EP0_CTRL);
		while(!ast_udc_read(ep->udc, AST_UDC11_ISR)& UDC11_EP0_IN_ACK);
		ast_udc_write(ep->udc, UDC11_EP0_NAK | UDC11_EP0_IN_ACK, AST_UDC11_ISR);	
	} else if (ep->ep.name == "ep1-int") {
		printk("ep 1 int \n");	
		while(ast_udc_read(ep->udc, AST_UDC11_EP1_CTRL)& EP_IN_BUFF_TX_RDY);
		ast_udc_write(ep->udc, *(u32 * )(req->req.buf + req->req.actual), AST_UDC11_EP1_DATA0);
		ast_udc_write(ep->udc, *(u32 * )(req->req.buf + req->req.actual + 4), AST_UDC11_EP1_DATA1);
		PKG_DBG("tx count %d, ep1 fifo : %x %x \n", count, ast_udc_read(ep->udc, AST_UDC11_EP1_DATA0), ast_udc_read(ep->udc, AST_UDC11_EP1_DATA1));
		req->req.actual += count;
		ast_udc_write(ep->udc, 0, AST_UDC11_EP1_CTRL);
		ast_udc_write(ep->udc, SET_EP_IN_TX_LEN(count) | EP_IN_BUFF_TX_RDY, AST_UDC11_EP1_CTRL);
		while(!ast_udc_read(ep->udc, AST_UDC11_ISR)& UDC11_EP1_IN_ACK); 
		ast_udc_write(ep->udc, UDC11_EP1_IN_ACK, AST_UDC11_ISR);	
		
	} else if (ep->ep.name == "ep2-int") {
		printk("ep 2 int \n");		
		while(ast_udc_read(ep->udc, AST_UDC11_EP2_CTRL)& EP_IN_BUFF_TX_RDY);	
		ast_udc_write(ep->udc, *(u32 * )(req->req.buf + req->req.actual), AST_UDC11_EP2_DATA0);
		ast_udc_write(ep->udc, *(u32 * )(req->req.buf + req->req.actual + 4), AST_UDC11_EP2_DATA1);
		PKG_DBG("tx count %d, ep2 fifo : %x %x \n",count, ast_udc_read(ep->udc, AST_UDC11_EP2_DATA0), ast_udc_read(ep->udc, AST_UDC11_EP2_DATA1));
		req->req.actual += count;
		ast_udc_write(ep->udc, 0, AST_UDC11_EP2_CTRL);
		ast_udc_write(ep->udc, SET_EP_IN_TX_LEN(count) | EP_IN_BUFF_TX_RDY, AST_UDC11_EP2_CTRL);
		while(!ast_udc_read(ep->udc, AST_UDC11_ISR)& UDC11_EP2_IN_ACK); 
		ast_udc_write(ep->udc, UDC11_EP2_IN_ACK, AST_UDC11_ISR);	
	} else {
		printk("TODO  ep->ep.name %s , ep0name %s \n", ep->ep.name, ep0name);

	}

	if(req->req.actual != req->req.length)
		goto resend;

	return 0;
}
/*
 * this is a PIO-only driver, so there's nothing
 * interesting for request or buffer allocation.
 */

static struct usb_request *
ast_ep_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct ast_udc_req *req;
	UDC_DBG(" %s \n", _ep->name);
	req = kzalloc(sizeof (struct ast_udc_req), gfp_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

static void ast_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct ast_udc_req *req;
	UDC_DBG(" %s \n", _ep->name);

	req = container_of(_req, struct ast_udc_req, req);
	BUG_ON(!list_empty(&req->queue));
	kfree(req);
}

static int ast_udc_queue(struct usb_ep *_ep,
			struct usb_request *_req, gfp_t gfp_flags)
{
	struct ast_udc_req	*req = container_of(_req, struct ast_udc_req, req);
	struct ast_ep		*ep = container_of(_ep, struct ast_ep, ep);
	struct aspeed_hid	*udc;
	unsigned long		flags, sts=0;

//	printk("xxx %x , %x , ep->ep.name :%s , ep0name: %s \n",_ep, ep->ep.desc, ep->ep.name, ep0name);

#if 0
	if (!_req || !_req->complete
				|| !_req->buf || !list_empty(&req->queue)) {
		printk("invalid request\n");
		return -EINVAL;
	}

	if (!_ep || (!ep->ep.desc && ep->ep.name != ep0name)) {
		printk("invalid ep\n");
		return -EINVAL;
	}
#endif
	if (!_req || !_req->complete
			|| !_req->buf || !list_empty(&req->queue)) {
		printk("invalid request ========== \n");
		if(!_req)
			printk("! req \n");
		if(!_req->complete)
			printk("!_req->complete \n");
		if(!_req->buf)
			printk("!_req->buf \n");
		if(!list_empty(&req->queue))
			printk("!list_empty(&req->queue) ???? \n");
		
		return -EINVAL;
	}

	udc = ep->udc;

	if (!udc || !udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		printk("invalid device ------------------------ \n");
		return -EINVAL;
	}

//	spin_lock_irqsave(&udc->lock, flags);

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	UDC_DBG("%s : len %d\n", _ep->name, _req->length);

	/* try to kickstart any empty and idle queue */
	if (list_empty(&ep->queue) && !ep->stopped) {
		if (_ep->name == ep0name /* ep0 */) {
			if(ast_udc_read(ep->udc, AST_UDC11_ISR) & UDC11_EP0_OUT) {
				sts = ast_udc_read_fifo(ep, req);
			} else {
				sts = ast_udc_write_fifo(ep, req);
			}
			
		} else {
			//ep 1, ep2 is use int in 
			ast_udc_write_fifo(ep, req);
		}
	} else {
		printk("udc11 Please check \n");
	}
	
	if(!sts) {
		list_add_tail(&req->queue, &ep->queue);
	} 

//	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;

}

static int ast_udc_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct ast_ep		*ep;
	struct ast_udc_req	*req;
	unsigned long		flags;
	struct aspeed_hid		*udc;

	printk("%s ====================== TODO ...\n", __func__ );
#if 0
	ep = container_of(_ep, struct ast_ep, ep);
	if (!_ep || ep->ep.name == ep0name)
		return -EINVAL;

	udc = ep->udc;

	spin_lock_irqsave(&udc->lock, flags);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		spin_unlock_irqrestore(&udc->lock, flags);
		return -EINVAL;
	}

	done(ep, req, -ECONNRESET);
	spin_unlock_irqrestore(&udc->lock, flags);
#endif	
	return 0;
}

static int ast_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct ast_ep	*ep = container_of(_ep, struct ast_ep, ep);
	struct aspeed_hid	*udc = ep->udc;
	u32 __iomem	*creg;
	u32		csr;
	unsigned long	flags;
	int		status = 0;
	printk("%s TODO ..\n", __func__ );

	if (!_ep)
		return -EINVAL;
#if 0
	creg = ep->creg;
	spin_lock_irqsave(&udc->lock, flags);

	csr = __raw_readl(creg);

	/*
	 * fail with still-busy IN endpoints, ensuring correct sequencing
	 * of data tx then stall.  note that the fifo rx bytecount isn't
	 * completely accurate as a tx bytecount.
	 */
	if (ep->is_in && (!list_empty(&ep->queue) || (csr >> 16) != 0))
		status = -EAGAIN;
	else {
		csr |= CLR_FX;
		csr &= ~SET_FX;
		if (value) {
			csr |= AT91_UDP_FORCESTALL;
			UDC_DBG("halt %s\n", ep->ep.name);
		} else {
			ast_udc_write(udc, AT91_UDP_RST_EP, ep->int_mask);
			ast_udc_write(udc, AT91_UDP_RST_EP, 0);
			csr &= ~AT91_UDP_FORCESTALL;
		}
		__raw_writel(csr, creg);
	}

	spin_unlock_irqrestore(&udc->lock, flags);
#endif	
	return status;
}

static const struct usb_ep_ops ast_ep_ops = {
	.enable		= ast_ep_enable,
	.disable	= ast_ep_disable,
	
	.alloc_request	= ast_ep_alloc_request,
	.free_request	= ast_ep_free_request,
	
	.queue		= ast_udc_queue,
	.dequeue	= ast_udc_dequeue,
	
	.set_halt	= ast_ep_set_halt,
	// there's only imprecise fifo status reporting
};

/*-------------------------------------------------------------------------*/

static int ast_get_frame(struct usb_gadget *gadget)
{
	struct aspeed_hid *udc = to_udc(gadget);
	printk("%s\n", __func__ );
	return UDC11_STS_GET_FRAM_NUM(ast_udc_read(udc, AST_UDC11_STS));
}

static void ast_udc_enable(struct aspeed_hid *udc)
{
	//AST UDC initial 
	ast_udc_write(udc, 0x1ff, AST_UDC11_IER);
	ast_udc_write(udc, 0x0, AST_UDC11_EP0_CTRL);
	ast_udc_write(udc, 0x0, AST_UDC11_EP1_CTRL);
	ast_udc_write(udc, 0x0, AST_UDC11_EP2_CTRL);	
	ast_udc_write(udc, UDC11_CTRL_CLK_STOP | UDC11_CTRL_LS_EN | UDC11_CTRL_CONNECT_EN, AST_UDC11_CTRL);
}

static void ast_udc_disable(struct aspeed_hid *udc)
{
	printk("%s ====\n", __func__ );

	ast_udc_write(udc, ast_udc_read(udc, AST_UDC11_CTRL) & ~UDC11_CTRL_CONNECT_EN, AST_UDC11_CTRL);
	udc->addr = 0;

}

static int ast_wakeup(struct usb_gadget *gadget)
{
	struct aspeed_hid	*udc = to_udc(gadget);
	int		status = -EINVAL;
	unsigned long	flags;

	printk("%s\n", __func__ );
	spin_lock_irqsave(&udc->lock, flags);

	spin_unlock_irqrestore(&udc->lock, flags);
	return status;
}

/* reinit == restore inital software state */
static void aspeed_hid_reinit(struct aspeed_hid *udc)
{
	u32 i;
	printk("%s --------------------- \n", __func__ );
	INIT_LIST_HEAD(&udc->gadget.ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);

	for (i = 0; i < NUM_ENDPOINTS; i++) {
		printk("%s ep %d \n", __func__, i);
		struct ast_ep *ep = &udc->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
		ep->desc = NULL;
		ep->stopped = 0;
//		usb_ep_set_maxpacket_limit(&ep->ep, 8);
		// initialiser une queue par endpoint
		INIT_LIST_HEAD(&ep->queue);
	}
}

/* vbus is here!  turn everything on that's ready */
static int ast_vbus_session(struct usb_gadget *gadget, int is_active)
{
	struct aspeed_hid	*udc = to_udc(gadget);
	unsigned long	flags;

	UDC_DBG("vbus %s == \n", is_active ? "on" : "off");
	spin_lock_irqsave(&udc->lock, flags);
	udc->vbus = (is_active != 0);
	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}


static int ast_udc_pullup(struct usb_gadget *gadget, int is_on)
{
	struct aspeed_hid	*udc = to_udc(gadget);
	unsigned long	flags;
	printk("%s : %d\n", __func__ ,is_on);

	spin_lock_irqsave(&udc->lock, flags);
	if (is_on)
		ast_udc_enable(udc);
	else {
		if (udc->gadget.speed != USB_SPEED_UNKNOWN) {
			if (udc->driver && udc->driver->disconnect)
				udc->driver->disconnect(&udc->gadget);
	
		}
		ast_udc_disable(udc);
	}

	spin_unlock_irqrestore(&udc->lock, flags);


	return 0;
}

static int ast_udc_start(struct usb_gadget *gadget,
		struct usb_gadget_driver *driver)
{
	struct aspeed_hid	*udc;

	udc = container_of(gadget, struct aspeed_hid, gadget);
	udc->driver = driver;
	udc->gadget.dev.of_node = udc->pdev->dev.of_node;
	udc->enabled = 1;
	udc->gadget.is_selfpowered = 1;

	return 0;
}

static int ast_udc_stop(struct usb_gadget *gadget)
{
	struct aspeed_hid *udc;
	unsigned long	flags;

	udc = container_of(gadget, struct aspeed_hid, gadget);
	spin_lock_irqsave(&udc->lock, flags);
	udc->enabled = 0;
//	at91_udp_write(udc, AT91_UDP_IDR, ~0);
	spin_unlock_irqrestore(&udc->lock, flags);

	udc->driver = NULL;

	return 0;
}

static const struct usb_gadget_ops aspeed_hid_ops = {
	.get_frame		= ast_get_frame,
	.wakeup			= ast_wakeup,
	.vbus_session	= ast_vbus_session,
	.pullup			= ast_udc_pullup,
	.udc_start		= ast_udc_start,
	.udc_stop		= ast_udc_stop,
	
};

#if 0
static const struct usb_gadget_ops at91_udc_ops = {
	.get_frame		= at91_get_frame,
	.wakeup			= at91_wakeup,
	.set_selfpowered	= at91_set_selfpowered,
	.vbus_session		= at91_vbus_session,
	.pullup			= at91_pullup,

	/*
	 * VBUS-powered devices may also also want to support bigger
	 * power budgets after an appropriate SET_CONFIGURATION.
	 */
	/* .vbus_power		= at91_vbus_power, */
};

#endif

/*-------------------------------------------------------------------------*/
static void ast_udc_ep0_setup(struct aspeed_hid *udc)
{
	struct ast_ep	*ep0 = &udc->ep[0];

	struct usb_ctrlrequest  setup_request;
	u32     setup_request_data[2];
	
	struct ast_udc_req 	*req;
	u32     ret;

	if(udc->driver == NULL) {
		printk("error return setup \n");
		return;
	}

	if (udc->gadget.dev.driver == NULL) {
		printk("setup structure null \n");
		return;
	}

	udc->ep0state = EP0_IN_DATA_PHASE;
	setup_request_data[0] = ast_udc_read(udc, AST_UDC11_EP0_SETUP0);
	setup_request_data[1] = ast_udc_read(udc, AST_UDC11_EP0_SETUP1);
	memcpy(&setup_request, setup_request_data, sizeof(setup_request));

	switch (setup_request.bRequest) {
	case USB_REQ_SET_CONFIGURATION:
		UDC_DBG("USB_REQ_SET_CONFIGURATION ... \n");
		if (setup_request.bRequestType == USB_RECIP_DEVICE) {
			if(!(ast_udc_read(udc, AST_UDC11_CONF) & UDC11_CONF_SET_CONF))
				ast_udc_write(udc, ast_udc_read(udc, AST_UDC11_CONF) | 
							UDC11_CONF_SET_CONF, AST_UDC11_CONF);
		}
		break;
	case USB_REQ_SET_ADDRESS:
		if (setup_request.bRequestType == USB_RECIP_DEVICE) {
			udc->addr = setup_request.wValue & 0x7F;
			UDC_DBG("USB_REQ_SET_ADDRESS ... %x \n", udc->addr);
			//work around 
			ast_udc_write(udc, 0, AST_UDC11_EP0_CTRL);
			ast_udc_write(udc, SET_EP_IN_TX_LEN(0) | EP_IN_BUFF_TX_RDY, AST_UDC11_EP0_CTRL);
		}
		return;
		break;

	}

	if (!udc->driver) {
		printk("no udc gadget driver \n");
		return;
	}

	ret = udc->driver->setup(&udc->gadget, &setup_request);

	if (ret < 0) {
		  printk("setup fail set ep0 STALL ========\n");
	}

}

static void ast_udc_ep2_in_intr(struct aspeed_hid *udc)
{
	struct ast_ep		*ep2 = &udc->ep[2];
	struct ast_udc_req	*req;

	UDC_DBG("ast_udc_ep2_in_intr\n");
	if (list_empty(&ep2->queue)) {
		req = NULL;
		printk("!! list_empty ========ERROR =======\n");
		return;
	} else {
		req = list_entry(ep2->queue.next, struct ast_udc_req, queue);
		ast_udc_done(ep2, req, 0);
	}
}

static void ast_udc_ep1_in_intr(struct aspeed_hid *udc)
{
	struct ast_ep		*ep1 = &udc->ep[1];
	struct ast_udc_req	*req;

	UDC_DBG("ast_udc_ep1_in_intr\n");
	if (list_empty(&ep1->queue)) {
		req = NULL;
		printk("!! list_empty ========ERROR =======\n");
		return;
	} else {
		req = list_entry(ep1->queue.next, struct ast_udc_req, queue);
		ast_udc_done(ep1, req, 0);
	}
}

static void ast_udc_ep0_in_intr(struct aspeed_hid *udc)
{
	struct ast_ep		*ep0 = &udc->ep[0];
	struct ast_udc_req	*req;

//	UDC_DBG("ast_udc_ep0_in_intr\n");
	if (list_empty(&ep0->queue)) {
		req = NULL;
//		printk("!! list_empty ======== TODO ======= %d , %x \n", udc->addr, ast_udc_read(udc, AST_UDC11_CONF));
		//Ryan workaround for ours set address mode ...
		if((udc->addr != 0) && (ast_udc_read(udc, AST_UDC11_CONF) == 0)) {
			printk("SET ADDR  = %d \n",udc->addr);
			ast_udc_write(udc, (ast_udc_read(udc, AST_UDC11_CONF) & ~UDC11_CONF_ADDR_MASK) | 
							UDC11_CONF_SET_ADDR(udc->addr), AST_UDC11_CONF);
		}
	} else {
		printk("ep0 in xx\n");
		req = list_entry(ep0->queue.next, struct ast_udc_req, queue);
		ast_udc_done(ep0, req, 0);
		//trigger rx 
		udc->ep0state = EP0_OUT_DATA_PHASE;
		ast_udc_write(udc, EP_OUT_BUFF_RX_RDY, AST_UDC11_EP0_CTRL);
	}
	
	return;	
	
}

static void ast_udc_ep0_out_intr(struct aspeed_hid *udc)
{
	struct ast_ep		*ep0 = &udc->ep[0];
	struct ast_udc_req	*req;

	udc->ep0state = EP0_OUT_DATA_PHASE;

	UDC_DBG("ast_udc_ep0_out_intr rcv len %d \n", GET_EP_OUT_RX_LEN(ast_udc_read(udc, AST_UDC11_EP0_CTRL)));

	if(GET_EP_OUT_RX_LEN(ast_udc_read(udc, AST_UDC11_EP0_CTRL)) == 0)
		return;
	
	if (list_empty(&ep0->queue)) {
		req = NULL;
		printk("!! list_empty ========ERROR =======\n");
		return;
	} else {
		req = list_entry(ep0->queue.next, struct ast_udc_req, queue);
		ast_udc_read_fifo(ep0, req);
	}
}

static irqreturn_t aspeed_hid_irq (int irq, void *_udc)
{
	struct aspeed_hid	*udc = _udc;
	unsigned long		flags;	
	u32 sts = ast_udc_read(udc, AST_UDC11_ISR);
//	printk("%s : %x \n", __func__ , sts);

	spin_lock_irqsave(&udc->lock, flags);
	if(sts & UDC11_BUS_REST) {
		UDC_DBG("UDC11_BUS_REST \n");
		/* clear interrupt */
		udc->suspended = 0;
		ast_udc_write(udc, UDC11_BUS_REST, AST_UDC11_ISR);
		udc->ep0state = EP0_IDLE;
		udc->addr = 0;
		udc->gadget.speed = USB_SPEED_LOW;
		ast_udc_write(udc, 0, AST_UDC11_CONF);
	} 

	if(sts & UDC11_SUSPEND_ENTRY) {
//		UDC_DBG("UDC11_SUSPEND_ENTRY \n");
		ast_udc_write(udc, UDC11_SUSPEND_ENTRY, AST_UDC11_ISR);
		udc->suspended = 1;
		if (udc->driver	&& udc->driver->suspend) {
			spin_unlock(&udc->lock);
			udc->driver->suspend(&udc->gadget);
			spin_lock(&udc->lock);		
		}
		udc->ep0state = EP0_IDLE;
	}

	if(sts & UDC11_SUSPEND_RESUME) {
//		UDC_DBG("UDC11_SUSPEND_RESUME \n");
		udc->suspended = 0;
		ast_udc_write(udc, UDC11_SUSPEND_RESUME, AST_UDC11_ISR);
		if (udc->driver && udc->driver->resume) {
			spin_unlock(&udc->lock);
			udc->driver->resume(&udc->gadget);
			spin_lock(&udc->lock);
		}
	}

	if(sts & UDC11_EP0_SETUP) {
		UDC_DBG("UDC11_EP0_SETUP \n");
		ast_udc_write(udc, UDC11_EP0_SETUP, AST_UDC11_ISR);
		ast_udc_ep0_setup(udc);
	}

	if(sts & UDC11_EP0_IN_ACK) {
//		UDC_DBG("UDC11_EP0_IN_ACK \n");
		ast_udc_write(udc, UDC11_EP0_IN_ACK, AST_UDC11_ISR);	
		ast_udc_ep0_in_intr(udc);
	}

	if(sts & UDC11_EP0_OUT_ACK) {
//		UDC_DBG("UDC11_EP0_OUT_ACK \n");
		ast_udc_write(udc, UDC11_EP0_OUT_ACK, AST_UDC11_ISR);
		ast_udc_ep0_out_intr(udc);
	}

	if(sts & UDC11_EP1_IN_ACK) {
		UDC_DBG("UDC11_EP1_IN_ACK \n");
		ast_udc_write(udc, UDC11_EP1_IN_ACK, AST_UDC11_ISR);		
		ast_udc_ep1_in_intr(udc);
	}

	if(sts & UDC11_EP2_IN_ACK) {
		UDC_DBG("UDC11_EP2_IN_ACK \n");
		ast_udc_write(udc, UDC11_EP2_IN_ACK, AST_UDC11_ISR);		
		ast_udc_ep2_in_intr(udc);
	}

	if(sts & UDC11_EP0_NAK) {
//		UDC_DBG("UDC11_EP0_NAK \n");
		ast_udc_write(udc, UDC11_EP0_NAK, AST_UDC11_ISR);
	}

	spin_unlock_irqrestore(&udc->lock, flags);

	return IRQ_HANDLED;
}

/*-------------------------------------------------------------------------*/

static void nop_release(struct device *dev)
{
	/* nothing to free */
}

/*-------------------------------------------------------------------------*/

static void ast_udc_shutdown(struct platform_device *dev)
{
	struct aspeed_hid *udc = platform_get_drvdata(dev);
	unsigned long	flags;

	printk("ast_udc_shutdown \n");	

	/* force disconnect on reboot */
	spin_lock_irqsave(&udc->lock, flags);

	spin_unlock_irqrestore(&udc->lock, flags);
}

static const char driver_name [] = "aspeed_hid";

static const struct {
        const char *name;
        const struct usb_ep_caps caps;
} ep_info[] = {
#define EP_INFO(_name, _caps) \
        { \
                .name = _name, \
                .caps = _caps, \
        }

        EP_INFO("ep0",
                USB_EP_CAPS(USB_EP_CAPS_TYPE_CONTROL, USB_EP_CAPS_DIR_ALL)),
        EP_INFO("ep1-int",
                USB_EP_CAPS(USB_EP_CAPS_TYPE_INT, USB_EP_CAPS_DIR_ALL)),
        EP_INFO("ep2-int",
                USB_EP_CAPS(USB_EP_CAPS_TYPE_INT, USB_EP_CAPS_DIR_ALL)),
#undef EP_INFO
};

#define ep0name         ep_info[0].name


static int aspeed_hid_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct aspeed_hid	*udc;
	int					retval;
	struct resource		*res;
	struct ast_ep		*ep;
	int					i;

	printk("%s\n", __func__ );

	udc = devm_kzalloc(dev, sizeof(*udc), GFP_KERNEL);
	if (!udc)
		return -ENOMEM;

	/* init software state */
	udc->gadget.dev.parent = dev;
	udc->pdev = pdev;
	udc->enabled = 0;
	spin_lock_init(&udc->lock);

	udc->gadget.ops = &aspeed_hid_ops;
	udc->gadget.ep0 = &udc->ep[0].ep;
	udc->gadget.name = driver_name;
	udc->gadget.dev.init_name = "gadget";

	for (i = 0; i < NUM_ENDPOINTS; i++) {
		ep = &udc->ep[i];
		ep->ep.name = ep_info[i].name;
		ep->ep.caps = ep_info[i].caps;
		ep->ep.ops = &ast_ep_ops;
		ep->ep.maxpacket = 8;
		ep->udc = udc;
		ep->int_mask = BIT(i);
		if (i != 0)
			ep->is_pingpong = 1;
	}

	printk("%s 4\n", __func__ );

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	printk("%s 5\n", __func__ );

	udc->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(udc->reg_base))
		return PTR_ERR(udc->reg_base);

	printk("%s 6\n", __func__ );

	aspeed_hid_reinit(udc);	

	printk("%s 7\n", __func__ );

	udc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(udc->clk)) {
		retval = PTR_ERR(udc->clk);
		return retval;
	}
printk("%s 8\n", __func__ );	
	retval = clk_prepare_enable(udc->clk);
	if (retval) {
		dev_err(&pdev->dev, "Error couldn't enable clock (%d)\n", retval);
		return retval;
	}

	printk("%s 9\n", __func__ );

	ast_udc_write(udc, 0, AST_UDC11_CTRL);
	ast_udc_write(udc, 0, AST_UDC11_CONF);
	ast_udc_write(udc, UDC11_REST_EP2 | UDC11_REST_EP1, AST_UDC11_REST);	

	printk("%s 10\n", __func__ );
	
	/* Clear all pending interrupts - UDP may be used by bootloader. */
	ast_udc_write(udc, 0x1ff, AST_UDC11_ISR);

	/* request UDC and maybe VBUS irqs */
	udc->udp_irq = platform_get_irq(pdev, 0);
	retval = devm_request_irq(dev, udc->udp_irq, aspeed_hid_irq, 0, 
							driver_name, udc);
	if (retval) {
		printk("request irq %d failed\n", udc->udp_irq);
		goto fail1;
	}
	printk("%s 11\n", __func__ );

	retval = usb_add_gadget_udc(dev, &udc->gadget);
	if (retval)
		goto fail0b;

	dev_set_drvdata(dev, udc);
	device_init_wakeup(dev, 1);
	create_debug_file(udc);
printk("%s done\n", __func__ );

//	INFO("%s \n", driver_name);

	return 0;

fail1:
	device_unregister(&udc->gadget.dev);
fail0b:
	iounmap(udc->reg_base);
fail0:
	release_mem_region(res->start, resource_size(res));
	printk("ast udc  probe failed, %d\n", retval);
	return retval;
}

static int __exit ast_udc_remove(struct platform_device *pdev)
{
#if 0
	struct aspeed_hid *udc = platform_get_drvdata(pdev);
	struct resource *res;
	unsigned long	flags;

	printk("remove\n");

	if (udc->driver)
		return -EBUSY;

	spin_lock_irqsave(&udc->lock, flags);
	pullup(udc, 0);
	spin_unlock_irqrestore(&udc->lock, flags);

	device_init_wakeup(&pdev->dev, 0);
	remove_debug_file(udc);
	if (udc->board.vbus_pin > 0) {
		free_irq(udc->board.vbus_pin, udc);
		gpio_free(udc->board.vbus_pin);
	}
	free_irq(udc->udp_irq, udc);
	device_unregister(&udc->gadget.dev);

	iounmap(udc->reg_base);

	if (cpu_is_at91rm9200())
		gpio_free(udc->board.pullup_pin);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	clk_put(udc->iclk);
	clk_put(udc->fclk);
#endif
	return 0;
}

static const struct of_device_id aspeed_hid_ids[] = {
	{ .compatible = "aspeed,ast2600-hid", },
};
MODULE_DEVICE_TABLE(of, aspeed_hid_ids);

static struct platform_driver aspeed_hid_driver = {
	.driver		= {
		.name	= "aspeed-hid",
		.of_match_table = of_match_ptr(aspeed_hid_ids),
	},
	.probe		= aspeed_hid_probe,
	.remove		= __exit_p(ast_udc_remove),
	.shutdown	= ast_udc_shutdown,
};

static int __init udc_init_module(void)
{
	printk("%s --------------------------------------------------\n", __func__ );

	return platform_driver_register(&aspeed_hid_driver);
}
module_init(udc_init_module);

static void __exit udc_exit_module(void)
{
	printk("%s\n", __func__ );

	platform_driver_unregister(&aspeed_hid_driver);
}
module_exit(udc_exit_module);

MODULE_DESCRIPTION("AST UDC11 HID driver");
MODULE_AUTHOR("Ryan Chen");
MODULE_LICENSE("GPL");
