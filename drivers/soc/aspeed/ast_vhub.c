/********************************************************************************
* File Name     : ast_vhub.c 
* Author         : Ryan Chen
* Description   : AST vHub driver 
* 
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
 */
 
#include <linux/module.h>
#include <linux/io.h>
 #include <linux/interrupt.h>
#include <asm/mach/irq.h>
#include <linux/irqchip/chained_irq.h>


#include <mach/hardware.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/hwmon-sysfs.h>

#include <linux/dma-mapping.h>
#include <linux/usb.h>
#include <linux/usb/ch11.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/usb/hcd.h>

//#include <plat/regs-udc.h>
//#include <plat/ast-lpc.h>
//#include <plat/regs-vhub.h>
#include <plat/ast-vhub.h>

/*  ************************************************************************************/
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
#define AST_DEV_SETUP(x)			(0x88 + (x * 0x8))
#define AST_DEV_SETUP1(x)			(0x8C + (x * 0x8))


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
#define ISR_EP_ACK_STALL					(1 << 16)
#define ISR_DEVICE5						(1 << 13)
#define ISR_DEVICE4						(1 << 12)
#define ISR_DEVICE3						(1 << 11)
#define ISR_DEVICE2						(1 << 10)
#define ISR_DEVICE1						(1 << 9)
#define ISR_SUSPEND_RESUME				(1 << 8)
#define ISR_BUS_SUSPEND 				(1 << 7)
#define ISR_BUS_RESET 					(1 << 6)
#define ISR_HUB_EP1_IN_DATA_ACK		(1 << 5)
#define ISR_HUB_EP0_IN_DATA_ACK		(1 << 4)
#define ISR_HUB_EP0_IN_ACK_STALL		(1 << 3)
#define ISR_HUB_EP0_OUT_NAK			(1 << 2)
#define ISR_HUB_EP0_OUT_ACK_STALL		(1 << 1)
#define ISR_HUB_EP0_SETUP_ACK			(1)

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

/*************************************************************************************/
#define HIGH_SPEED			1
/*************************************************************************************/
//80, 06, 00, 01, 00, 00
//Deivce Descriptor
u8 Hub_Descriptor1[18] = { 0x12, 0x01, 0x00, 0x02, 0x09, 0x00, 0x01, 0x40,
                             0x2a, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                             0x00, 0x01 };

//80, 06, 00, 02, 00, 00
//Configuration Descriptor
u8 Hub_Descriptor2[25] = { 0x09, 0x02, 0x19, 0x00, 0x01, 0x01, 0x00, 0xe0,
                             0x32, 0x09, 0x04, 0x00, 0x00, 0x01, 0x09, 0x00,
                             0x00, 0x00, 0x07, 0x05, 0x81, 0x03, 0x01, 0x00,
                             0x0c };

//A0, 06, 00, 00, 00, 00
//Class-Specific Descriptor - hub descriptor
//				desc len / type /port number /
u8 Hub_Descriptor3[9] = { 0x09, 0x29, 0x01, 0xE0, 0x00, 0x32, 0x64, 0x00, 0xff };
//u8 Hub_Descriptor3[9] = { 0x09, 0x29, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff };


//A0, 00, 00, 00, 00, 00
//Get Device Status
//03 = The device is self powered and remote wakeup
u8 Hub_Descriptor4[2] = { 0x03, 0x00 };

//A3, 00, 00, 00, 01, 00
//Port 01 Status
//03 = A device is present on the port and it is disabled
//05 = This port is not in the power-off state and high-speed device is attached to this port
//10 = Reset completed
#if (HIGH_SPEED)
u8 Hub_Descriptor5[4] = { 0x03, 0x05, 0x10, 0x00 };
#else
u8 Hub_Descriptor5[4] = { 0x03, 0x03, 0x10, 0x00 };
#endif


//A3, 00, 00, 00, 01, 00
//Port 01 Status
//Port 01 Status
//03 = A device is present on the port and it is disabled
//05 = This port is not in the power-off state and high-speed device is attached to this port
#if (HIGH_SPEED)
u8 Hub_Descriptor6[4] = { 0x03, 0x05, 0x00, 0x00 };
#else
u8 Hub_Descriptor6[4] = { 0x03, 0x03, 0x00, 0x00 };
#endif


//A3, 00, 00, 00, 01, 00
//Port 01 Status
//01 = A device is present on the port and it is disabled
//05 = This port is not in the power-off state and high-speed device is attached to this port
#if (HIGH_SPEED)
u8 Hub_Descriptor7[4] = { 0x01, 0x05, 0x00, 0x00 };
#else
u8 Hub_Descriptor7[4] = { 0x03, 0x03, 0x00, 0x00 };
#endif

u8 Hub_Descriptor8[4] = { 0x00, 0x00, 0x00, 0x00 };

//01 = This port is not in the power-off state 
u8 Hub_Descriptor9[4] = { 0x00, 0x01, 0x00, 0x00 };

u8 Hub_PortS0[4] = { 0x00, 0x00, 0x00, 0x00 };
u8 Hub_PortS1[4] = { 0x01, 0x05, 0x01, 0x00 };
u8 Hub_PortS2[4] = { 0x01, 0x05, 0x00, 0x00 };
u8 Hub_PortS3[4] = { 0x03, 0x05, 0x10, 0x00 };

/*************************************************************************************/
//#define AST_VHUB_DEBUG
	
#ifdef AST_VHUB_DEBUG
#define VHUB_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
//#define VHUB_DBUG(fmt, args...) printk("%s() " fmt,__FUNCTION__, ## args)

#else
#define VHUB_DBUG(fmt, args...)
#endif

struct ast_usb_ep_data {
	u8		enable;
	u8		state;
	u8 		*dma_virt_addr;
	dma_addr_t dma_phy_addr;	
};

struct ast_usb_device_data {
	u8 		present; 
	u8		state;
	
	u8 		complete; 	
	u8		enable;
	int 		addr;
	int 		set_addr;

	u8 		*dma_virt_addr;
	dma_addr_t dma_phy_addr;	
	struct usb_ctrlrequest *dev_setup;
	u8 		*tx_data;	
	u8 		ep_tx_len;
	u8 		ep_tx;	
	
u32    setup[2];
u32    datalen;
u32    in[32];
u32    out[32];
int 	 incnt;
u32    outcnt;
u32	index;
	
};

struct ast_vhub_data {
	void __iomem	*reg_base;	
	int irq;				//VHUB IRQ number 	
	struct usb_ctrlrequest *root_setup;
	struct ast_usb_device_data ast_dev[5];
	struct ast_usb_ep_data ast_ep[15];	
	u8 *vhub_dma_virt;
	dma_addr_t vhub_dma_phy;

	u16	dev_addr;
	wait_queue_head_t vhub_wq;		
	u32 flag;
	bool is_open;
	u32 state;
	//rx
	u32 rx_len;
	u8 ep_id;
	u8 rt;	
	u8 seq_no;	
	int 		addr;
	int 		set_addr;

	u32    setup[2];
	u32    datalen;
	u32    in[32];
	u32    out[32];
	int 	 incnt;
	u32    outcnt;
	u32	index;
	
};

/******************************************************************************/
static inline u32 
ast_vhub_read(struct ast_vhub_data *ast_vhub, u32 reg)
{
	u32 val = readl(ast_vhub->reg_base + reg);
//	VHUB_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
}

static inline void
ast_vhub_write(struct ast_vhub_data *ast_vhub, u32 val, u32 reg) 
{
//	VHUB_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, ast_vhub->reg_base + reg);
}
/*************************************************************************************************************************************************/
void vhub_ep0_tx(struct ast_vhub_data *ast_vhub, u8 *tx_data, u32 len)
{
	if(len) {
		memcpy(ast_vhub->vhub_dma_virt, tx_data, len);
		ast_vhub_write(ast_vhub, ast_vhub->vhub_dma_phy, AST_VHUB_EP0_DATA_BUFF);
		ast_vhub_write(ast_vhub, EP0_TX_LEN(len), AST_VHUB_EP0_CTRL);
		ast_vhub_write(ast_vhub, EP0_TX_LEN(len) | EP0_TX_BUFF_RDY, AST_VHUB_EP0_CTRL);
	} else {
		ast_vhub_write(ast_vhub, EP0_TX_BUFF_RDY, AST_VHUB_EP0_CTRL);
	}
}

void vhub_ep0_rx(struct ast_vhub_data *ast_vhub)
{
	u32 rx_len = 0;
	ast_vhub_write(ast_vhub, EP0_RX_BUFF_RDY, AST_VHUB_EP0_CTRL);
	rx_len = EP0_GET_RX_LEN(ast_vhub_read(ast_vhub, AST_VHUB_EP0_CTRL));
	VHUB_DBUG("rx_len = %d \n", rx_len);

//	if(rx_len)
//		memcpy(tx_data, ast_vhub->vhub_dma_virt, rx_len);	
}

void ast_set_port_feature(struct ast_vhub_data *ast_vhub, u8 port) {
	switch (ast_vhub->root_setup->wValue) {
		case USB_PORT_FEAT_ENABLE:
			VHUB_DBUG("USB_PORT_FEAT_ENABLE \n");
			break;
		case USB_PORT_FEAT_SUSPEND:
			VHUB_DBUG("USB_PORT_FEAT_SUSPEND \n");
			break;
		case USB_PORT_FEAT_POWER:
			VHUB_DBUG("USB_PORT_FEAT_POWER \n");
			ast_vhub->ast_dev[port].state = 1;
			//enable device TODO ~~~~~
			if(ast_vhub->ast_dev[port].present)
				ast_vhub_write(ast_vhub, ast_vhub_read(ast_vhub, 0x100 + (port * 0x10)) | 0x1, 0x100 + (port * 0x10));
			break;
		case USB_PORT_FEAT_INDICATOR:
			VHUB_DBUG("USB_PORT_FEAT_INDICATOR \n");
			break;
		case USB_PORT_FEAT_C_CONNECTION:
			VHUB_DBUG("USB_PORT_FEAT_C_CONNECTION \n");
			break;
		case USB_PORT_FEAT_RESET:
			VHUB_DBUG("USB_PORT_FEAT_RESET port change attach \n");
			ast_vhub->ast_dev[port].state = 3;
			ast_vhub_write(ast_vhub, 0x1 << (port + 1), AST_VHUB_EP1_STS_CHG);			
			break;
		case USB_PORT_FEAT_C_ENABLE:
			VHUB_DBUG("USB_PORT_FEAT_C_ENABLE \n");
			break;
		case USB_PORT_FEAT_C_SUSPEND:
			VHUB_DBUG("USB_PORT_FEAT_C_SUSPEND \n");
			break;
		case USB_PORT_FEAT_C_OVER_CURRENT:
			VHUB_DBUG("USB_PORT_FEAT_C_OVER_CURRENT \n");
			break;
		default:
			VHUB_DBUG("default ~~~~~~~~~~~~~~\n");
			break;
	}
	
}

void ast_clear_port_feature(struct ast_vhub_data *ast_vhub, u8 port) {
	switch (ast_vhub->root_setup->wValue) {
		case USB_PORT_FEAT_ENABLE:
			VHUB_DBUG("USB_PORT_FEAT_ENABLE \n");
			break;
		case USB_PORT_FEAT_SUSPEND:
			VHUB_DBUG("USB_PORT_FEAT_SUSPEND \n");
			break;
		case USB_PORT_FEAT_POWER:
			VHUB_DBUG("USB_PORT_FEAT_POWER \n");
			ast_vhub->ast_dev[port].state = 0;
			break;
		case USB_PORT_FEAT_INDICATOR:
			VHUB_DBUG("USB_PORT_FEAT_INDICATOR \n");
			/* Port inidicator not supported */
			break;
		case USB_PORT_FEAT_C_CONNECTION:
			VHUB_DBUG("USB_PORT_FEAT_C_CONNECTION \n");
			ast_vhub->ast_dev[port].state = 2;
			break;
		case USB_PORT_FEAT_RESET:
			VHUB_DBUG("USB_PORT_FEAT_RESET port change attach \n");
			break;
		case USB_PORT_FEAT_C_RESET:
			VHUB_DBUG("USB_PORT_FEAT_C_RESET \n");
			break;
		case USB_PORT_FEAT_C_ENABLE:
			VHUB_DBUG("USB_PORT_FEAT_C_ENABLE \n");
			break;
		case USB_PORT_FEAT_C_SUSPEND:
			VHUB_DBUG("USB_PORT_FEAT_C_SUSPEND \n");
			break;
		case USB_PORT_FEAT_C_OVER_CURRENT:
			VHUB_DBUG("USB_PORT_FEAT_C_OVER_CURRENT \n");
			break;
		default:
			VHUB_DBUG("default ~~~~~~~~~~~~~~\n");
			break;
	}
	
}

void ast_usb_type_class(struct ast_vhub_data *ast_vhub)
{
//	printk("class req : %x ,  val :%x, idx : %x \n",ast_vhub->root_setup->bRequest, ast_vhub->root_setup->wValue, ast_vhub->root_setup->wIndex);
	switch (ast_vhub->root_setup->bRequest) {
		case USB_REQ_GET_DESCRIPTOR:
		        switch (ast_vhub->root_setup->wValue >> 8) {
		                case USB_DT_DEVICE:
		                        VHUB_DBUG( "USB_DT_DEVICE \n");
		                        if(ast_vhub->root_setup->wLength < sizeof(Hub_Descriptor3))
		                                vhub_ep0_tx(ast_vhub, Hub_Descriptor3, ast_vhub->root_setup->wLength);
		                        else
		                                vhub_ep0_tx(ast_vhub, Hub_Descriptor3, sizeof(Hub_Descriptor3));
		                        break;
		                case USB_DT_CONFIG:
		                        VHUB_DBUG( "USB_DT_CONFIG \n");
		                        if(ast_vhub->root_setup->wLength < sizeof(Hub_Descriptor3))
		                                vhub_ep0_tx(ast_vhub, Hub_Descriptor3, ast_vhub->root_setup->wLength);
		                        else
		                                vhub_ep0_tx(ast_vhub, Hub_Descriptor3, sizeof(Hub_Descriptor3));
		                        break;

		                case USB_DT_DEVICE_QUALIFIER:
		                        VHUB_DBUG( "USB_DT_DEVICE_QUALIFIER \n");
		                        break;
		                case USB_DT_OTHER_SPEED_CONFIG:
		                        VHUB_DBUG( "USB_DT_OTHER_SPEED_CONFIG \n");
		                        break;
		                case USB_DT_STRING:
		                        VHUB_DBUG( "USB_DT_STRING \n");
		                        break;
		                case USB_DT_BOS:

		                        break;
		                default:
		                        VHUB_DBUG( "default \n");
		                        if(ast_vhub->root_setup->wLength < sizeof(Hub_Descriptor3))
		                                vhub_ep0_tx(ast_vhub, Hub_Descriptor3, ast_vhub->root_setup->wLength);
		                        else
		                                vhub_ep0_tx(ast_vhub, Hub_Descriptor3, sizeof(Hub_Descriptor3));
		                        break;
		                }
		        break;
		case USB_REQ_SET_CONFIGURATION:
		        VHUB_DBUG( "USB_REQ_SET_CONFIGURATION ...\n");
		        vhub_ep0_tx(ast_vhub, 0, 0);
		        break;

		case USB_REQ_SET_INTERFACE:
		        VHUB_DBUG( "USB_REQ_SET_INTERFACE ...\n");
		        break;
				case USB_REQ_SET_ADDRESS:
					VHUB_DBUG( "USB_REQ_SET_ADDRESS \n");
					break;
				case USB_REQ_GET_STATUS:
					VHUB_DBUG( "USB_REQ_GET_STATUS ...dev [%d] \n", ast_vhub->root_setup->wIndex);
					switch(ast_vhub->root_setup->wIndex) {
						case 0: //Hub status 
							vhub_ep0_tx(ast_vhub, Hub_Descriptor8, sizeof(Hub_Descriptor8));
							break;
						case 1: //port 1 status
							printk("sts %d \n", ast_vhub->ast_dev[ast_vhub->root_setup->wIndex - 1].state);
							if(ast_vhub->ast_dev[ast_vhub->root_setup->wIndex - 1].state == 0) {
								//power off 
								vhub_ep0_tx(ast_vhub, Hub_PortS0, sizeof(Hub_PortS0));
							} else if(ast_vhub->ast_dev[ast_vhub->root_setup->wIndex - 1 ].state == 1) {
								//power on
								vhub_ep0_tx(ast_vhub, Hub_PortS1, sizeof(Hub_PortS1));
							} else if(ast_vhub->ast_dev[ast_vhub->root_setup->wIndex - 1].state == 2) {
								vhub_ep0_tx(ast_vhub, Hub_PortS2, sizeof(Hub_PortS2));
							} else if(ast_vhub->ast_dev[ast_vhub->root_setup->wIndex - 1].state == 3) {
								vhub_ep0_tx(ast_vhub, Hub_PortS3, sizeof(Hub_PortS3));
							} else {
								printk("TODO ~~~~~~\n");
							}
												
							break;
						case 2: //port 2 staus 
							vhub_ep0_tx(ast_vhub, Hub_PortS0, sizeof(Hub_PortS0));

							break;
					}		
					break;
			case USB_REQ_CLEAR_FEATURE:
				VHUB_DBUG( "USB_REQ_CLEAR_FEATURE port [%d] \n", ast_vhub->root_setup->wIndex);
				ast_clear_port_feature(ast_vhub, ast_vhub->root_setup->wIndex - 1);
				vhub_ep0_tx(ast_vhub, 0, 0);
				break;
			case USB_REQ_SET_FEATURE:
				VHUB_DBUG("USB_REQ_SET_FEATURE port [%d] \n", ast_vhub->root_setup->wIndex);
				ast_set_port_feature(ast_vhub, ast_vhub->root_setup->wIndex - 1);
				vhub_ep0_tx(ast_vhub, 0, 0);
				break;
			default:
				VHUB_DBUG( "default 2 \n");
				if(ast_vhub->root_setup->wLength < sizeof(Hub_Descriptor3))
						vhub_ep0_tx(ast_vhub, Hub_Descriptor3, ast_vhub->root_setup->wLength);
				else
						vhub_ep0_tx(ast_vhub, Hub_Descriptor3, sizeof(Hub_Descriptor3));
				break;
	}
}

void ast_usb_type_standard(struct ast_vhub_data *ast_vhub)
{
	int i = 0;
	switch (ast_vhub->root_setup->bRequest) {
		case USB_REQ_GET_DESCRIPTOR:
			switch (ast_vhub->root_setup->wValue >> 8) {
				case USB_DT_DEVICE:
					VHUB_DBUG( "USB_REQ_GET_DESCRIPTOR ...USB_DT_DEVICE \n");
					vhub_ep0_tx(ast_vhub, Hub_Descriptor1, sizeof(Hub_Descriptor1));
					break;
				case USB_DT_CONFIG:					
					VHUB_DBUG( "USB_REQ_GET_DESCRIPTOR ...USB_DT_CONFIG \n");
					if(ast_vhub->root_setup->wLength < sizeof(Hub_Descriptor2))
						vhub_ep0_tx(ast_vhub, Hub_Descriptor2, ast_vhub->root_setup->wLength);
					else
						vhub_ep0_tx(ast_vhub, Hub_Descriptor2, sizeof(Hub_Descriptor2));
					break;
				case USB_DT_DEVICE_QUALIFIER:
					VHUB_DBUG( "USB_DT_DEVICE_QUALIFIER \n");
					break;
				case USB_DT_OTHER_SPEED_CONFIG:
					VHUB_DBUG( "USB_DT_OTHER_SPEED_CONFIG\n");
					break;
				case USB_DT_STRING:
					VHUB_DBUG( "USB_DT_STRING\n");
					break;
				case USB_DT_BOS:
					break;
				}
			break;
		case USB_REQ_SET_CONFIGURATION:
			VHUB_DBUG( "USB_REQ_SET_CONFIGURATION ...\n");
			vhub_ep0_tx(ast_vhub, 0, 0);
			break;

		case USB_REQ_SET_INTERFACE:
			VHUB_DBUG( "USB_REQ_SET_INTERFACE ...\n");
			break;

		case USB_REQ_SET_ADDRESS:
			VHUB_DBUG( "USB_REQ_SET_ADDRESS [%x]\n", ast_vhub->root_setup->wValue);
			vhub_ep0_tx(ast_vhub, 0, 0);
			ast_vhub_write(ast_vhub, ast_vhub->root_setup->wValue, AST_VHUB_CONF);		
			break;

		case USB_REQ_GET_STATUS:
			VHUB_DBUG( "USB_REQ_GET_STATUS ...\n");
			vhub_ep0_tx(ast_vhub, Hub_Descriptor4, sizeof(Hub_Descriptor4));
			break;

		case USB_REQ_CLEAR_FEATURE:
			VHUB_DBUG( "USB_REQ_CLEAR_FEATURE ...\n");
			return;

		case USB_REQ_SET_FEATURE:
			printk("USB_REQ_SET_FEATURE \n");
			return;

		default:

			break;
	}	
}
/*************************************************************************************/
void ast_setup_handle(struct ast_vhub_data *ast_vhub)
{
//	printk("vhub setup %x , %x \n", ast_vhub_read(ast_vhub, AST_VHUB_SETUP_DATA0), ast_vhub_read(ast_vhub, AST_VHUB_SETUP_DATA1));
	
	switch (ast_vhub->root_setup->bRequestType & USB_TYPE_MASK) {
		case USB_TYPE_STANDARD:
			ast_usb_type_standard(ast_vhub);
			break;	//USB_TYPE_STANDARD
		case USB_TYPE_CLASS:
			ast_usb_type_class(ast_vhub);
			break;
	}	
}

static void ast_vhub_isr(unsigned int irq, struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct ast_vhub_data *ast_vhub = irq_desc_get_chip_data(desc);
	u32 sts = ast_vhub_read(ast_vhub, AST_VHUB_ISR);
//	VHUB_DBUG("%x \n",sts);

	chained_irq_enter(chip, desc);

	if(sts & ISR_BUS_RESET) {
		//Reset, we don't handle this in sample
		VHUB_DBUG("ISR_BUS_RESET \n");
		ast_vhub_write(ast_vhub, ISR_BUS_RESET, AST_VHUB_ISR);
	}

	if(sts & ISR_BUS_SUSPEND) {
		//Suspend, we don't handle this in sample
		VHUB_DBUG("ISR_BUS_SUSPEND \n");
		ast_vhub_write(ast_vhub, ISR_BUS_SUSPEND, AST_VHUB_ISR);
	}

	if(sts & ISR_SUSPEND_RESUME) {
		//Suspend, we don't handle this in sample
		VHUB_DBUG("ISR_SUSPEND_RESUME \n");
		ast_vhub_write(ast_vhub, ISR_SUSPEND_RESUME, AST_VHUB_ISR);
	}

	if(sts & ISR_HUB_EP0_SETUP_ACK) {
		VHUB_DBUG("ISR_HUB_EP0_SETUP_ACK \n");
		ast_vhub_write(ast_vhub, ISR_HUB_EP0_SETUP_ACK, AST_VHUB_ISR);
		ast_setup_handle(ast_vhub);
	}

	if(sts & ISR_HUB_EP0_IN_ACK_STALL) {
		//Device_Set_Address[0] means HUB
//		VHUB_DBUG("ISR_HUB_EP0_IN_ACK_STALL \n");
		ast_vhub_write(ast_vhub, ISR_HUB_EP0_IN_ACK_STALL, AST_VHUB_ISR);
	}

	if(sts & ISR_HUB_EP0_OUT_ACK_STALL) {
		//OUT ACK, we don't handle this in sample
//		VHUB_DBUG("ISR_HUB_EP0_OUT_ACK_STALL \n");
		ast_vhub_write(ast_vhub, ISR_HUB_EP0_OUT_ACK_STALL, AST_VHUB_ISR);
	}

	if(sts & ISR_HUB_EP0_OUT_NAK) {
//		VHUB_DBUG("ISR_HUB_EP0_OUT_NAK \n");
		ast_vhub_write(ast_vhub, ISR_HUB_EP0_OUT_NAK, AST_VHUB_ISR);
		vhub_ep0_rx(ast_vhub);
	}

	if(sts & ISR_HUB_EP0_IN_DATA_ACK) {
		//IN NAK, we don't handle this in sample
//		VHUB_DBUG("ISR_HUB_EP0_IN_DATA_ACK \n");
		ast_vhub_write(ast_vhub, ISR_HUB_EP0_IN_DATA_ACK, AST_VHUB_ISR);
	}

	if(sts & ISR_HUB_EP0_OUT_NAK) {
		//IN NAK, we don't handle this in sample
//		VHUB_DBUG("ISR_HUB_EP0_OUT_NAK \n");
		ast_vhub_write(ast_vhub, ISR_HUB_EP0_OUT_NAK, AST_VHUB_ISR);
	}

	if(sts & ISR_HUB_EP1_IN_DATA_ACK) {
		//HUB Bitmap control
		VHUB_DBUG("ISR_HUB_EP1_IN_DATA_ACK change sts  = 0 \n");
		ast_vhub_write(ast_vhub, ISR_HUB_EP1_IN_DATA_ACK, AST_VHUB_ISR);
		ast_vhub_write(ast_vhub, 0x00, AST_VHUB_EP1_STS_CHG);
	}

	if(sts & ISR_DEVICE1) {
		VHUB_DBUG("ISR_DEVICE1 \n");
		generic_handle_irq(IRQ_VHUB_CHAIN_START);
		ast_vhub_write(ast_vhub, ISR_DEVICE1, AST_VHUB_ISR);
	}

	if(sts & ISR_DEVICE2) {
		printk("ISR_DEVICE2 *****************************\n");
		generic_handle_irq(IRQ_VHUB_CHAIN_START + 1);	
		ast_vhub_write(ast_vhub, ISR_DEVICE2, AST_VHUB_ISR);
	}

	if(sts & ISR_DEVICE3) {
		printk("ISR_DEVICE3 \n");
		generic_handle_irq(IRQ_VHUB_CHAIN_START + 2);			
		ast_vhub_write(ast_vhub, ISR_DEVICE3, AST_VHUB_ISR);
	}

	if(sts & ISR_DEVICE4) {
		printk("ISR_DEVICE4 \n");
		generic_handle_irq(IRQ_VHUB_CHAIN_START + 3);			
		ast_vhub_write(ast_vhub, ISR_DEVICE4, AST_VHUB_ISR);
	}

	if(sts & ISR_DEVICE5) {
		printk("ISR_DEVICE5 \n");
		generic_handle_irq(IRQ_VHUB_CHAIN_START + 4);			
		ast_vhub_write(ast_vhub, ISR_DEVICE5, AST_VHUB_ISR);
	}

	if(sts & ISR_EP_ACK_STALL) {
		VHUB_DBUG("ISR_EP_ACK_STALL ****************************************\n");
		ast_vhub_write(ast_vhub, ISR_EP_ACK_STALL, AST_VHUB_ISR);
	}

	chained_irq_exit(chip, desc);
}		

/*********************************************************************************************************************************/

struct ast_vhub_data *ast_vhub;

/*********************************************************************************************************************************/


static void ast_vhub_ctrl_init(struct ast_vhub_data *ast_vhub) 
{
	int i=0;
	char	*dev_dma_virt;
	dma_addr_t	dev_dma_phy;

	char	*ep_dma_virt;
	dma_addr_t	ep_dma_phy;

	ast_vhub->dev_addr = 0;
	
	//dma first ---
	ast_vhub->vhub_dma_virt = dma_alloc_coherent(NULL,
					 4096,
					 &ast_vhub->vhub_dma_phy, GFP_KERNEL);

	VHUB_DBUG("dam virt: %x , phy: %x \n", (u32)ast_vhub->vhub_dma_virt, (u32)ast_vhub->vhub_dma_phy);

	ast_vhub_write(ast_vhub, ast_vhub->vhub_dma_phy, AST_VHUB_EP0_DATA_BUFF);

	dev_dma_virt = dma_alloc_coherent(NULL,
					 4096 * 5,
					 &dev_dma_phy, GFP_KERNEL);
	
	//reg init
	ast_vhub->root_setup = (ast_vhub->reg_base + AST_VHUB_SETUP_DATA0);

	//dev * 5
	for(i = 0; i < 5;i++) {
		ast_vhub->ast_dev[i].enable = 0;
		ast_vhub->ast_dev[i].state = 0;
		ast_vhub->ast_dev[i].dma_virt_addr = dev_dma_virt + (i * 4096);
		ast_vhub->ast_dev[i].dma_phy_addr = dev_dma_phy + (i * 4096);
		ast_vhub->ast_dev[i].dev_setup = (ast_vhub->reg_base + AST_DEV_SETUP(i));
	}

	ep_dma_virt = dma_alloc_coherent(NULL,
					 1024 * 15,
					 &ep_dma_phy, GFP_KERNEL);

	//dev * 5
	for(i = 0; i < 15;i++) {
		ast_vhub->ast_ep[i].enable = 0;
		ast_vhub->ast_ep[i].state = 0;
		ast_vhub->ast_ep[i].dma_virt_addr = ep_dma_virt + (i * 1024);
		ast_vhub->ast_ep[i].dma_phy_addr = ep_dma_phy + (i * 1024);
	}

	ast_vhub_write(ast_vhub, ROOT_PHY_CLK_EN | ROOT_PHY_RESET_DIS, AST_VHUB_CTRL);
	
	udelay(1);
	ast_vhub_write(ast_vhub, 0, AST_VHUB_DEV_RESET);

#if 1
	ast_vhub_write(ast_vhub, 0x1ffff, AST_VHUB_IER);
#endif

	ast_vhub_write(ast_vhub, 0x1ffff, AST_VHUB_ISR);
	ast_vhub_write(ast_vhub, 0, AST_VHUB_EP0_CTRL);
	ast_vhub_write(ast_vhub, 0x1, AST_VHUB_EP1_CTRL);
	udelay(1);
	ast_vhub_write(ast_vhub, ast_vhub_read(ast_vhub,AST_VHUB_CTRL) | ROOT_UPSTREAM_EN, AST_VHUB_CTRL);


}

/*********************************************************************************************************************************/

static void ast_vhub_ack_irq(struct irq_data *d)
{
	struct ast_vhub_data *ast_vhub = irq_get_chip_data(d->irq);

	unsigned int vhub_irq = d->irq - IRQ_VHUB_CHAIN_START;

	VHUB_DBUG("irq[%d] , dev = %d \n",d->irq, vhub_irq);
}

static void ast_vhub_mask_irq(struct irq_data *d)
{
	struct ast_vhub_data *ast_vhub = irq_get_chip_data(d->irq);

	unsigned int vhub_irq = d->irq - IRQ_VHUB_CHAIN_START;

	VHUB_DBUG("irq[%d] , dev = %d \n",d->irq, vhub_irq);

	//disable irq
	ast_vhub_write(ast_vhub, ast_vhub_read(ast_vhub, AST_VHUB_IER) & ~( 1 << (9+vhub_irq)), AST_VHUB_IER);
}

static void ast_vhub_unmask_irq(struct irq_data *d)
{
	struct ast_vhub_data *ast_vhub = irq_get_chip_data(d->irq);
	unsigned int vhub_irq = d->irq - IRQ_VHUB_CHAIN_START;

	VHUB_DBUG("irq[%d] , dev = %d \n",d->irq, vhub_irq);
	ast_vhub_write(ast_vhub, ast_vhub_read(ast_vhub, AST_VHUB_IER) | ( 1 << (9+vhub_irq)), AST_VHUB_IER);
	ast_vhub->ast_dev[vhub_irq].present = 1;
}

static struct irq_chip ast_vhub_irq_chip = {
	.name		= "VHUB",
	.irq_ack		= ast_vhub_ack_irq,
	.irq_mask		= ast_vhub_mask_irq,
	.irq_unmask	= ast_vhub_unmask_irq,
};

/*******************************************************************************************************/
struct ast_vhub_data *ast_vhub;

static ssize_t 
store_port(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	input_val = simple_strtoul(sysfsbuf, NULL, 10);
	ast_vhub_write(ast_vhub, 1 << input_val, AST_VHUB_EP1_STS_CHG);
	return count;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, NULL, store_port);


static struct attribute *ast_vhub_attributes[] = {
	&dev_attr_enable.attr,
//	&dev_attr_buff.attr,		
	NULL
};

static const struct attribute_group vhub_attribute_group = {
	.attrs = ast_vhub_attributes,
};

/*******************************************************************************************************/
void __iomem *get_ast_udc_reg_base(void)
{
	return ast_vhub->reg_base;
}
EXPORT_SYMBOL(get_ast_udc_reg_base);

/*******************************************************************************************************/
struct miscdevice ast_vhub_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-vhub",
};

static int ast_vhub_probe(struct platform_device *pdev)
{
	struct resource *res;

	int ret=0;
	int dev=0;

	VHUB_DBUG("\n");	

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	if (!request_mem_region(res->start, resource_size(res), res->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		ret = -ENXIO;
		goto out;
	}

	if (!(ast_vhub = kzalloc(sizeof(struct ast_vhub_data), GFP_KERNEL))) {
		return -ENOMEM;
	}
	
	ast_vhub->reg_base = ioremap(res->start, resource_size(res));
	if (!ast_vhub->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	ast_vhub->irq = platform_get_irq(pdev, 0);
	if (ast_vhub->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	for(dev=0 ;dev < 5 ;dev++) {
		irq_set_chip_data(dev + IRQ_VHUB_CHAIN_START, ast_vhub);
		irq_set_chip_and_handler(dev + IRQ_VHUB_CHAIN_START, &ast_vhub_irq_chip,
					 handle_level_irq);
		set_irq_flags(dev + IRQ_VHUB_CHAIN_START, IRQF_VALID);
	}
	irq_set_chip_data(ast_vhub->irq, ast_vhub);
	irq_set_chained_handler(ast_vhub->irq, ast_vhub_isr);

	ret = misc_register(&ast_vhub_misc);
	if (ret){		
		printk(KERN_ERR "VHUB : failed to request interrupt\n");
		goto out_irq;
	}

	platform_set_drvdata(pdev, ast_vhub);
	
	dev_set_drvdata(ast_vhub_misc.this_device, ast_vhub);

	ast_vhub_ctrl_init(ast_vhub);

	ret = sysfs_create_group(&pdev->dev.kobj, &vhub_attribute_group);
	if (ret)
		goto out_irq;
	

	printk(KERN_INFO "ast_vhub: driver successfully loaded.\n");

	return 0;
out_irq:
	free_irq(ast_vhub->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_vhub_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_vhub_data *ast_vhub = platform_get_drvdata(pdev);

	VHUB_DBUG("\n");

	misc_deregister(&ast_vhub_misc);

	free_irq(ast_vhub->irq, ast_vhub);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_vhub->reg_base);

	platform_set_drvdata(pdev, NULL);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;	
}

#ifdef CONFIG_PM
static int 
ast_vhub_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int 
ast_vhub_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define ast_vhub_suspend        NULL
#define ast_vhub_resume         NULL
#endif

static struct platform_driver ast_vhub_driver = {
	.remove 		= ast_vhub_remove,
	.suspend        = ast_vhub_suspend,
	.resume         = ast_vhub_resume,
	.driver         = {
		.name   = "ast-vhub",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver_probe(ast_vhub_driver, ast_vhub_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST VHUB Driver");
MODULE_LICENSE("GPL");
