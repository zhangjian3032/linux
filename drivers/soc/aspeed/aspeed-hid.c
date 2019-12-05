// SPDX-License-Identifier: GPL-2.0+
/*
 * aspeed-hid -- Driver for Aspeed SoC HID
 *
 * Copyright (C) ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/poll.h>
#include <linux/sysfs.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <asm/uaccess.h>


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
#define UDC11_EP0_OUT				BIT(9)
#define UDC11_EP0_NAK				BIT(8)
#define UDC11_EP2_IN_ACK			BIT(7)
#define UDC11_EP1_IN_ACK			BIT(6)
#define UDC11_EP0_IN_ACK			BIT(5)
#define UDC11_EP0_OUT_ACK			BIT(4)
#define UDC11_EP0_SETUP				BIT(3)
#define UDC11_RESUME			BIT(2)
#define UDC11_SUSPEND			BIT(1)
#define UDC11_BUS_REST			BIT(0)

/* AST_UDC11_EP0_CTRL		0x18		Endpoint 0 Control and Status Register */
/* AST_UDC11_EP1_CTRL		0x1C	Endpoint 1 Control and Status Register */
/* AST_UDC11_EP2_CTRL		0x20		Endpoint 2 Control and Status Register */
#define GET_EP_OUT_RX_LEN(x)		((x & 0xf) >> 8)	//only for EP0
#define GET_EP_IN_TX_LEN(x)			((x & 0xf) >> 4)	
#define SET_EP_IN_TX_LEN(x)			((x & 0xf) << 4)
#define EP_OUT_BUFF_RX_RDY			(1 << 2)		//only for EP0
#define EP_IN_BUFF_TX_RDY			(1 << 1)
#define EP_CTRL_STALL

/***************************************************************************/
#define  IOCTL_USB11_MOUSE_PACKET   0x1112

typedef struct _IO_ACCESS_DATA {
    unsigned char Type;
    unsigned long Address;
    unsigned long Data;
    unsigned long Value;
    int      kernel_socket;
//    struct sockaddr_in address_svr;
} IO_ACCESS_DATA, *PIO_ACCESS_DATA;
/***************************************************************************/
#define AST_HID_DEBUG
	
#ifdef AST_HID_DEBUG
#define HID_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define HID_DBUG(fmt, args...)
#endif

struct ast_hid_info {
	void __iomem	*reg_base;	
	int 			irq;		
	struct clk		*clk;	
	
	wait_queue_head_t hid_waitqueue;	
	u32 flag;
	bool is_open;
	u32 state;
};

typedef struct _USB_DATA {
    u32    setup[2];
    u32    datalen;
    u32    in[32];
    u32    out[32];
    int      incnt;
    u32    outcnt, index;
} USB_DATA;

USB_DATA USB11_Data;

//80, 06, 00, 01, 00, 00
u8 Descriptor1[18] = { 0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, 0x08, 
						0x2a, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						0x00, 0x01 };

//80, 06, 00, 02, 00, 00
u8 Descriptor2[34] = { 0x09, 0x02, 0x22, 0x00, 0x01, 0x01, 0x00, 0xa0,
                         0x32, 0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01,
                         0x02, 0x00, 0x09, 0x21, 0x10, 0x01, 0x00, 0x01,
                         0x22, 0x34, 0x00, 0x07, 0x05, 0x81, 0x03, 0x04,
                         0x00, 0x0a };

//81, 06, 00, 22, 00, 00
u8 Descriptor3[52] = { 0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x09, 0x01,
                         0xa1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
                         0x15, 0x00, 0x25, 0x01, 0x95, 0x03, 0x75, 0x01,
                         0x81, 0x02, 0x95, 0x01, 0x75, 0x05, 0x81, 0x01,
                         0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
                         0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x03,
                         0x81, 0x06, 0xc0, 0xc0 };


//80, 06, 00, 01, 00, 00
u8 Keyboard_Descriptor1[18] = { 0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, 0x08, 
								0x81, 0x0a, 0x01, 0x01, 0x10, 0x01, 0x00, 0x00, 
								0x00, 0x01 };

//80, 06, 00, 02, 00, 00
u8 Keyboard_Descriptor2[59] = {
    0x09, 0x02, 0x3B, 0x00, 0x02, 0x01, 0x00, 0xa0,
    0x32, 0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01,
    0x01, 0x00, 0x09, 0x21, 0x10, 0x01, 0x21, 0x01,
    0x22, 0x41, 0x00, 0x07, 0x05, 0x81, 0x03, 0x08,
    0x00, 0x0a, 0x09, 0x04, 0x01, 0x00, 0x01, 0x03,
    0x01, 0x02, 0x00, 0x09, 0x21, 0x10, 0x01, 0x00,
    0x01, 0x22, 0x34, 0x00, 0x07, 0x05, 0x82, 0x03,
    0x04, 0x00, 0x0a };

//81, 06, 00, 22, 00, 00
u8 Keyboard_Descriptor3[65] = {
    0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x05, 0x07,
    0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x08, 0x81, 0x01, 0x95, 0x05, 0x75, 0x01,
    0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02,
    0x95, 0x03, 0x75, 0x01, 0x91, 0x01, 0x95, 0x06,
    0x75, 0x08, 0x15, 0x00, 0x26, 0xff, 0x00, 0x05,
    0x07, 0x19, 0x00, 0x2a, 0xff, 0x00, 0x81, 0x00,
    0xc0 };

DECLARE_WAIT_QUEUE_HEAD (my_queue);

int  flag = 0;

/*************************************************************************************/
static DEFINE_SPINLOCK(hid_state_lock);

/******************************************************************************/

int USB11_send_mouse_packet(USB_DATA *USB_Info)
{
    unsigned char index = 0;
    unsigned long status;
#if 0
//    printk ("IN Send MOUSE PACKET\n");
//    printk ("USB_Info->incnt = %d\n", USB_Info->incnt);
//    printk ("USB_Info->in[0] = %x\n", USB_Info->in[0]);
//    printk ("USB_Info->in[1] = %x\n", USB_Info->in[1]);

        if (USB_Info->incnt >= 8) {
//            printk ("incnt >= 8\n");
            *(u32 *)(IO_ADDRESS(USB_2TXL)) = USB_Info->in[0];
            *(u32 *)(IO_ADDRESS(USB_2TXH)) = USB_Info->in[1];
            *(u32 *)(IO_ADDRESS(USB_EP2C)) = 0x80;
            *(u32 *)(IO_ADDRESS(USB_EP2C)) = 0x82;
            USB_Info->incnt -= 8;
        }
        else if (USB_Info->incnt >= 0) {
//            printk ("incnt < 8\n");
            *(u32 *)(IO_ADDRESS(USB_2TXL)) = USB_Info->in[0];
            *(u32 *)(IO_ADDRESS(USB_2TXH)) = USB_Info->in[1];
            *(u32 *)(IO_ADDRESS(USB_EP2C)) = (USB_Info->incnt << 4);
            *(u32 *)(IO_ADDRESS(USB_EP2C)) = (USB_Info->incnt << 4) | 0x02;
            USB_Info->incnt = 0;
        }
//        printk ("Go to Sleep\n");
//Check ACK
        wait_event_interruptible (my_queue, (flag == 1));
        flag = 0;
#endif
    return 0;
}


int USB11_send_keyboard_packet(USB_DATA *USB_Info)
{
    unsigned char index = 0;
    unsigned long status;
#if 0
//    printk ("IN Send Keyboard PACKET\n");
//    printk ("USB_Info->incnt = %d\n", USB_Info->incnt);
//    printk ("USB_Info->in[0] = %x\n", USB_Info->in[0]);
//    printk ("USB_Info->in[1] = %x\n", USB_Info->in[1]);

//    do {
        if (USB_Info->incnt >= 8) {
//            printk ("incnt >= 8\n");
            *(u32 *)(IO_ADDRESS(USB_1TXL)) = USB_Info->in[0];
            *(u32 *)(IO_ADDRESS(USB_1TXH)) = USB_Info->in[1];
            *(u32 *)(IO_ADDRESS(USB_EP1C)) = 0x80;
            *(u32 *)(IO_ADDRESS(USB_EP1C)) = 0x82;
            USB_Info->incnt -= 8;
        }
        else if (USB_Info->incnt >= 0) {
//            printk ("incnt < 8\n");
            *(u32 *)(IO_ADDRESS(USB_1TXL)) = USB_Info->in[0];
            *(u32 *)(IO_ADDRESS(USB_1TXH)) = USB_Info->in[1];
            *(u32 *)(IO_ADDRESS(USB_EP1C)) = (USB_Info->incnt << 4);
            *(u32 *)(IO_ADDRESS(USB_EP1C)) = (USB_Info->incnt << 4) | 0x02;
            USB_Info->incnt = 0;
        }
//        printk ("Go to Sleep\n");
//Check ACK
        wait_event_interruptible (my_queue, (flag == 1));
        flag = 0;
//    } while (USB_Info->incnt != 0);
#endif
    return 0;
}

void Setup_Mouse_Descriptor1(USB_DATA *USB_Info)
{
    USB_Info->index = 0;
    memcpy ((unsigned char *)USB_Info->in, (unsigned char *)Descriptor1, 0x12);
//    printk ("USB_Info->in[0] = %8x\n", USB_Info->in[0]);
//    printk ("USB_Info->in[1] = %8x\n", USB_Info->in[1]);
    if (USB_Info->datalen > 0x12) {
        USB_Info->incnt = 0x12;
    }
    else {
        USB_Info->incnt = USB_Info->datalen;
    }
}

void Setup_Descriptor1(USB_DATA *USB_Info)
{
    USB_Info->index = 0;
    memcpy ((unsigned char *)USB_Info->in, (unsigned char *)Keyboard_Descriptor1, 0x12);
//    printk ("USB_Info->in[0] = %8x\n", USB_Info->in[0]);
//    printk ("USB_Info->in[1] = %8x\n", USB_Info->in[1]);
    if (USB_Info->datalen > 0x12) {
        USB_Info->incnt = 0x12;
    }
    else {
        USB_Info->incnt = USB_Info->datalen;
    }
}

void Setup_Mouse_Descriptor2(USB_DATA *USB_Info)
{

    USB_Info->index = 0;
    memcpy ((unsigned char *)USB_Info->in, (unsigned char *)Descriptor2, 0x22);
    if (USB_Info->datalen > 0x22) {
        USB_Info->incnt = 0x22;
    }
    else {
        USB_Info->incnt = USB_Info->datalen;
    }
}

void Setup_Descriptor2(USB_DATA *USB_Info)
{

    USB_Info->index = 0;
    memcpy ((unsigned char *)USB_Info->in, (unsigned char *)Keyboard_Descriptor2, 0x3B);
    if (USB_Info->datalen > 0x3B) {
        USB_Info->incnt = 0x3B;
    }
    else {
        USB_Info->incnt = USB_Info->datalen;
    }
}

void Setup_Mouse_Descriptor3(USB_DATA *USB_Info)
{

    USB_Info->index = 0;
    memcpy ((unsigned char *)USB_Info->in, (unsigned char *)Descriptor3, 0x34);
    if (USB_Info->datalen > 0x34) {
        USB_Info->incnt = 0x34;
    }
    else {
        USB_Info->incnt = USB_Info->datalen;
    }
}

void Setup_Descriptor3(USB_DATA *USB_Info)
{

    USB_Info->index = 0;
    memcpy ((unsigned char *)USB_Info->in, (unsigned char *)Keyboard_Descriptor3, 0x41);
    if (USB_Info->datalen > 0x41) {
        USB_Info->incnt = 0x41;
    }
    else {
        USB_Info->incnt = USB_Info->datalen;
    }
}

void RX_EP0_OUT_Data (struct ast_hid_info *aspeed_hid, USB_DATA *USB_Info)
{
    unsigned long status;
    unsigned char index = 0;

//Disable EP0 OUT ACK Interrupt
	writel(0x4, aspeed_hid->reg_base + AST_UDC11_EP0_CTRL);

    do {
//      status = (*(u32 *)(IO_ADDRESS(USB_INTS)) & 0x10);
        status = (readl(aspeed_hid->reg_base + AST_UDC11_EP0_CTRL) & 0x04);
    } while (status != 0x04);
    USB_Info->outcnt = (readl(aspeed_hid->reg_base + AST_UDC11_EP0_CTRL) >> 8) & 0xf;

    if (USB_Info->outcnt != 0) {
        if (USB_Info->outcnt <= 4) {
            USB_Info->out[index++] = readl(aspeed_hid->reg_base + AST_UDC11_EP0_DATA0);
        } else {
            USB_Info->out[index++] = readl(aspeed_hid->reg_base + AST_UDC11_EP0_DATA1);
        }
    }
//    printk ("EP0_OUT Done\n");
}

int TX_EP0_Data(struct ast_hid_info *aspeed_hid, USB_DATA *USB_Info)
{
    u32 status;
//    do {
        if (USB_Info->incnt >= 8) {
            printk ("incnt >= 8\n");
			writel(USB_Info->in[USB_Info->index++], aspeed_hid->reg_base + AST_UDC11_EP0_DATA0);
			writel(USB_Info->in[USB_Info->index++], aspeed_hid->reg_base + AST_UDC11_EP0_DATA1);
			printk("%x %x \n", readl(aspeed_hid->reg_base + AST_UDC11_EP0_DATA0), readl(aspeed_hid->reg_base + AST_UDC11_EP0_DATA1));
			writel(0, aspeed_hid->reg_base + AST_UDC11_EP0_CTRL);
			writel(0x82, aspeed_hid->reg_base + AST_UDC11_EP0_CTRL);
            USB_Info->incnt -= 8;
        } else if (USB_Info->incnt >= 0) {
            printk ("incnt < 8\n");
			writel(USB_Info->in[USB_Info->index++], aspeed_hid->reg_base + AST_UDC11_EP0_DATA0);
			writel(USB_Info->in[USB_Info->index++], aspeed_hid->reg_base + AST_UDC11_EP0_DATA1);
			printk("%x %x \n", readl(aspeed_hid->reg_base + AST_UDC11_EP0_DATA0), readl(aspeed_hid->reg_base + AST_UDC11_EP0_DATA1));
			writel((USB_Info->incnt << 4), aspeed_hid->reg_base + AST_UDC11_EP0_CTRL);
			writel((USB_Info->incnt << 4) | 0x02, aspeed_hid->reg_base + AST_UDC11_EP0_CTRL);
			USB_Info->incnt = 0;
//            USB_Info->incnt = -1;
        }
#if 0
		while(!readl(aspeed_hid->reg_base + AST_UDC11_ISR) & UDC11_EP0_IN_ACK);
		writel(UDC11_EP0_NAK | UDC11_EP0_IN_ACK, aspeed_hid->reg_base + AST_UDC11_ISR);
        printk ("ACK\n");
#endif		
//    } while (USB_Info->incnt != 0);


    return 0;
}

static irqreturn_t aspeed_hid_isr(int this_irq, void *dev_id)
{
	u32 status, Device_Address;

	struct ast_hid_info *aspeed_hid = dev_id;

	status = readl(aspeed_hid->reg_base + AST_UDC11_ISR);
//	printk("isr %x \n", status);
	
	if(status & UDC11_BUS_REST) {
		writel(UDC11_BUS_REST, aspeed_hid->reg_base + AST_UDC11_ISR);
	}
	
	if(status & UDC11_RESUME) {
		writel(UDC11_RESUME, aspeed_hid->reg_base + AST_UDC11_ISR);
	}
	
	if(status & UDC11_SUSPEND) {
		writel(UDC11_SUSPEND, aspeed_hid->reg_base + AST_UDC11_ISR);
	}
	
    if (status & UDC11_EP1_IN_ACK) {
		writel(UDC11_EP1_IN_ACK, aspeed_hid->reg_base + AST_UDC11_ISR);
		printk ("Wake Up Mouse\n");
//		flag = 1;
//		wake_up_interruptible (&my_queue);
    } else if (status & UDC11_EP2_IN_ACK) {
		writel(UDC11_EP2_IN_ACK, aspeed_hid->reg_base + AST_UDC11_ISR);
		printk ("Wake Up Keyboard\n");
		flag = 1;
//		wake_up_interruptible (&my_queue);
	} else if (status & UDC11_EP0_IN_ACK) {
		printk("UDC11_EP0_IN_ACK\n");
		writel(UDC11_EP0_IN_ACK, aspeed_hid->reg_base + AST_UDC11_ISR);
		TX_EP0_Data(aspeed_hid, &USB11_Data);
    } else if (status & UDC11_EP0_OUT_ACK) {
    	writel(UDC11_EP0_OUT_ACK, aspeed_hid->reg_base + AST_UDC11_ISR);
		printk ("out ack\n");
		//flag = 1;
		//wake_up_interruptible (&my_queue);
    } else if (status & UDC11_EP0_SETUP) {
			writel(UDC11_EP0_SETUP, aspeed_hid->reg_base + AST_UDC11_ISR);
			printk("Setup Packet Arrived\n");

			USB11_Data.setup[0] = readl(aspeed_hid->reg_base + AST_UDC11_EP0_SETUP0); 
			USB11_Data.setup[1] = readl(aspeed_hid->reg_base + AST_UDC11_EP0_SETUP1); 
			USB11_Data.datalen = (USB11_Data.setup[1] >> 16);
			USB11_Data.setup[1] &= 0xffff;
			
             if ((USB11_Data.setup[0] & 0xffff) == 0x0500) {
				USB11_Data.incnt = 0;
				TX_EP0_Data(aspeed_hid, &USB11_Data);
				Device_Address = ((USB11_Data.setup[0] >> 16) & 0xff);
				printk ("Set Address %d \n", Device_Address);
				writel((Device_Address << 1), aspeed_hid->reg_base + AST_UDC11_CONF);
			} else if ((USB11_Data.setup[0] == 0x01000680) &&
                      (USB11_Data.setup[1] == 0x00000000)) {
                  printk ("Device Descriptor\n");
                  Setup_Descriptor1 (&USB11_Data);
                  TX_EP0_Data(aspeed_hid, &USB11_Data);
             }
             else if ((USB11_Data.setup[0] == 0x02000680) &&
                      (USB11_Data.setup[1] == 0x00000000)) {
                  printk ("Configuration Descriptor\n");
                  Setup_Mouse_Descriptor2 (&USB11_Data);
                  Setup_Descriptor2 (&USB11_Data);
                  TX_EP0_Data (aspeed_hid, &USB11_Data);
             }
             else if ((USB11_Data.setup[0] == 0x00000a21) &&
                      (USB11_Data.setup[1] == 0x00000000)) {
                  printk ("Class Report 0\n");
                  USB11_Data.incnt = 0;
                  TX_EP0_Data (aspeed_hid, &USB11_Data);
             }
             else if ((USB11_Data.setup[0] == 0x00000a21) &&
                      (USB11_Data.setup[1] == 0x00000001)) {
                  printk ("Class Report 1\n");
                  USB11_Data.incnt = 0;
                  TX_EP0_Data (aspeed_hid, &USB11_Data);
             }
             else if ((USB11_Data.setup[0] == 0x22000681) &&
                      (USB11_Data.setup[1] == 0x00000000)) {
                  printk ("Mouse Descriptor\n");
                  Setup_Descriptor3 (&USB11_Data);
                  TX_EP0_Data (aspeed_hid, &USB11_Data);
             }
             else if ((USB11_Data.setup[0] == 0x22000681) &&
                      (USB11_Data.setup[1] == 0x00000001)) {
                  printk ("Keyboard Descriptor\n");
                  Setup_Mouse_Descriptor3 (&USB11_Data);
                  TX_EP0_Data (aspeed_hid, &USB11_Data);
             }
             else if ((USB11_Data.setup[0] & 0xffff) == 0x0900) {
                  USB11_Data.incnt = 0;
                  TX_EP0_Data (aspeed_hid, &USB11_Data);
					writel(readl(aspeed_hid->reg_base + AST_UDC11_CONF) | 0x1, aspeed_hid->reg_base + AST_UDC11_CONF);
                  printk ("Configuration Done\n");
             } else {
			 		printk("TODO　nothing \n");
             }
    } else if (status & UDC11_EP0_OUT) {
    printk("status %x \n", status);
//      printk ("OUT transaction %x %x \n", readl(aspeed_hid->reg_base + AST_UDC11_EP0_DATA0), readl(aspeed_hid->reg_base + AST_UDC11_EP0_DATA1));
//        USB11_Data.incnt = 0;
//        RX_EP0_OUT_Data(aspeed_hid, &USB11_Data);
		writel(0x4, aspeed_hid->reg_base + AST_UDC11_EP0_CTRL);

		if(status & UDC11_EP0_NAK)
			writel(UDC11_EP0_NAK, aspeed_hid->reg_base + AST_UDC11_ISR);


    } else if (status & UDC11_EP0_NAK) {
		printk("EP0 NAK \n ");		
		writel(UDC11_EP0_NAK, aspeed_hid->reg_base + AST_UDC11_ISR);
//		if(readl(aspeed_hid->reg_base + AST_UDC11_CONF) & 0x1)
//			writel(0xff, aspeed_hid->reg_base + AST_UDC11_IER);
    } else {
		printk("unknow status %x \n", status);
    }

    return IRQ_HANDLED;
}

/*************************************************************************************/
static long aspeed_hid_ioctl(struct file *file, unsigned int cmd,
		       unsigned long arg)
{
    int ret = 0;
	struct miscdevice *c = file->private_data;
	struct ast_hid_info *ast_hid = dev_get_drvdata(c->this_device);
	void __user *argp = (void __user *)arg;

#if 0
	switch (cmd) {
		case AST_JTAG_IOCDATA:
			if (copy_from_user(&data, argp, sizeof(struct dr_xfer)))
				ret = -EFAULT;
			else 
				ast_hid_dr_xfer(ast_hid ,&data);

			if (copy_to_user(argp, &data, sizeof(struct dr_xfer)))
				ret = -EFAULT;
			break;
		default:
			return -ENOTTY;
	}
#endif
    return ret;
}

static int aspeed_hid_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_hid_info *ast_hid = dev_get_drvdata(c->this_device);

	if (ast_hid->is_open) {
		return -EBUSY;
	}
	ast_hid->is_open = true;

	return 0;
}

static int aspeed_hid_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_hid_info *ast_hid = dev_get_drvdata(c->this_device);

	ast_hid->is_open = false;
	return 0;
}

void ast_udc11_init(struct ast_hid_info *aspeed_hid)
{
	writel(0, aspeed_hid->reg_base + AST_UDC11_CTRL);
	writel(0, aspeed_hid->reg_base + AST_UDC11_CONF);
	writel(0x3, aspeed_hid->reg_base + AST_UDC11_REST);
	writel(0x1ff, aspeed_hid->reg_base + AST_UDC11_ISR);
	writel(0x1ff, aspeed_hid->reg_base + AST_UDC11_IER);
	writel(0, aspeed_hid->reg_base + AST_UDC11_EP0_CTRL);
	writel(2, aspeed_hid->reg_base + AST_UDC11_EP1_CTRL);
	writel(2, aspeed_hid->reg_base + AST_UDC11_EP2_CTRL);
	writel(2, aspeed_hid->reg_base + AST_UDC11_CTRL);
}

static const struct file_operations aspeed_hid_fops = {
	.owner			= THIS_MODULE,
	.llseek 		= no_llseek,		
	.unlocked_ioctl	= aspeed_hid_ioctl,
	.open			= aspeed_hid_open,
	.release		= aspeed_hid_release,
};

struct miscdevice aspeed_hid_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aspeed-hid",
	.fops = &aspeed_hid_fops,
};

static const struct of_device_id aspeed_hid_matches[] = {
	{ .compatible = "aspeed,ast2400-hid",	},
	{ .compatible = "aspeed,ast2500-hid",	},
	{ .compatible = "aspeed,ast2600-hid",	},
	{},
};

MODULE_DEVICE_TABLE(of, aspeed_hid_matches);

static int aspeed_hid_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct ast_hid_info *aspeed_hid;
	struct resource *res;

	int ret=0;

	HID_DBUG("aspeed_hid_probe\n");

	if (!(aspeed_hid = devm_kzalloc(dev, sizeof(*aspeed_hid), GFP_KERNEL))) {
			return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	aspeed_hid->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(aspeed_hid->reg_base)) {
		ret = PTR_ERR(aspeed_hid->reg_base);
		goto out_region;
	}

	aspeed_hid->irq = platform_get_irq(pdev, 0);
	if (aspeed_hid->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	aspeed_hid->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(aspeed_hid->clk)) {
		ret = PTR_ERR(aspeed_hid->clk);
		return ret;
	}

	ret = clk_prepare_enable(aspeed_hid->clk);
	if (ret) {
		dev_err(&pdev->dev, "Error couldn't enable clock (%d)\n", ret);
		return ret;
	}

    ret = devm_request_irq(dev, aspeed_hid->irq, aspeed_hid_isr, 0, "aspeed-hid", aspeed_hid);
    if (ret) {
        printk("HID Unable to get IRQ");
		goto out_region;
    }

    init_waitqueue_head(&aspeed_hid->hid_waitqueue);

	ret = misc_register(&aspeed_hid_misc);
	if (ret){		
		printk(KERN_ERR "PECI : failed to request interrupt\n");
		goto out_irq;
	}

	platform_set_drvdata(pdev, aspeed_hid);
	dev_set_drvdata(aspeed_hid_misc.this_device, aspeed_hid);

	ast_udc11_init(aspeed_hid);
	
	printk(KERN_INFO "aspeed_hid: driver successfully loaded.\n");

	return 0;


out_irq:
	free_irq(aspeed_hid->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int aspeed_hid_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_hid_info *ast_hid = platform_get_drvdata(pdev);

	HID_DBUG("ast_hid_remove\n");

	misc_deregister(&aspeed_hid_misc);

	free_irq(ast_hid->irq, ast_hid);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_hid->reg_base);

	platform_set_drvdata(pdev, NULL);

	return 0;	
}

static struct platform_driver aspeed_hid_driver = {
	.probe		= aspeed_hid_probe,
	.remove		= aspeed_hid_remove,
	.driver		= {
		.name   = KBUILD_MODNAME,
		.of_match_table = aspeed_hid_matches,
	},
};

module_platform_driver(aspeed_hid_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("HID Driver");
MODULE_LICENSE("GPL");
