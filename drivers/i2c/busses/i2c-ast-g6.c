/*
 * i2c-ast.c - I2C driver for the Aspeed SoC
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
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/dma-mapping.h>
/* I2C Register */
#define AST_I2CC_FUN_CTRL			0x00	
#define AST_I2CC_AC_TIMING			0x04
#define AST_I2CC_BUFF				0x08
#define AST_I2CC_BUFF_CTRL			0x0C

#define AST_I2CM_IER			0x10
#define AST_I2CM_ISR			0x14
#define AST_I2CM_CMD_STS			0x18
#define AST_I2CM_DMA_LEN			0x1C

#define AST_I2CS_IER			0x20
#define AST_I2CS_ADDR			0x20
#define AST_I2CS_ISR			0x24
#define AST_I2CS_CMD_STS			0x28
#define AST_I2CS_DMA_LEN			0x2C

#define AST_I2CM_TX_DMA		0x30 	/* I2CM Master DMA Tx Buffer Register   */
#define AST_I2CM_RX_DMA		0x34	/* I2CM Master DMA Rx Buffer Register   */
#define AST_I2CS_TX_DMA		0x38 	/* I2CS Slave DMA Tx Buffer Register   */
#define AST_I2CS_RX_DMA		0x3C	/* I2CS Slave DMA Rx Buffer Register   */

/* 0x00 : I2CC Master/Slave Function Control Register  */
#define AST_I2CC_PAGE_SEL				(0x1 << 31)
#define AST_I2CC_WRITE_DIS				(0x1 << 30)

#define AST_I2CC_SLAVE_ADDR_RX_EN 		(0x1 << 19)
#define AST_I2CC_MASTER_RETRY 			(0x1 << 18)

#define AST_I2CC_BUS_AUTO_RELEASE		(0x1 << 17)	//only support ast_soc_g5
#define AST_I2CC_M_SDA_LOCK_EN			(0x1 << 16)
#define AST_I2CC_MULTI_MASTER_DIS		(0x1 << 15)
#define AST_I2CC_M_SCL_DRIVE_EN			(0x1 << 14)
#define AST_I2CC_MSB_STS				(0x1 << 9)
#define AST_I2CC_SDA_DRIVE_1T_EN		(0x1 << 8)
#define AST_I2CC_M_SDA_DRIVE_1T_EN		(0x1 << 7)
#define AST_I2CC_M_HIGH_SPEED_EN		(0x1 << 6)

#define AST_I2CC_SLAVE_EN				(0x1 << 1)
#define AST_I2CC_MASTER_EN				(0x1 )

/* 0x04 : I2CC Master/Slave Clock and AC Timing Control Register #1 */
#define AST_I2CC_tTIMEOUT(x)			((x) << 24)	// 0~7

#define AST_I2CC_tCKHIGHMin(x)			((x) << 20)	// 0~f 
#define AST_I2CC_tCKHIGH(x)				((x) << 16)	// 0~7 
#define AST_I2CC_tCKLOW(x)				((x) << 12)	// 0~7 
#define AST_I2CC_tHDDAT(x)				((x) << 10)	// 0~7 
#define AST_I2CC_toutBaseCLK(x)			((x) << 8)	
#define AST_I2CC_tBaseCLK(x)			(x)			// 0~0xf

/* 0x08 : I2CC Master/Skave Transmit/Receive Byte Buffer Register */
#define AST_I2CC_TX_DIR_MASK			(0x7 << 29)
#define AST_I2CC_SDA_OE					(0x1 << 28)
#define AST_I2CC_SDA_O					(0x1 << 27)
#define AST_I2CC_SCL_OE					(0x1 << 26)
#define AST_I2CC_SCL_O					(0x1 << 25)
#define AST_I2CC_TX_TIMING				(0x1 << 24)	// 0 ~3
#define AST_I2CC_TX_STATUS				(0x1 << 23)
// Tx State Machine
#define AST_I2CM_IDLE	 				0x0
#define AST_I2CM_STARTH					0x1
#define AST_I2CC_STARTW 				0x2
#define AST_I2CC_STARTR 				0x3
#define AST_I2CC_SRXD 					0x4
#define AST_I2CC_STXACK 				0x5
#define AST_I2CC_STXD					0x6
#define AST_I2CC_SRXACK 				0x7

#define AST_I2CM_MACTIVE				0x8

#define AST_I2CC_MSTART					0x9
#define AST_I2CC_MSTARTR				0xa
#define AST_I2CC_MSTOP					0xb
#define AST_I2CC_MTXD					0xc
#define AST_I2CC_MRXACK					0xd
#define AST_I2CC_MRXD 					0xe
#define AST_I2CC_MTXACK 				0xf

#define AST_I2CC_SCL_LINE_STS				(0x1 << 18)
#define AST_I2CC_SDA_LINE_STS				(0x1 << 17)
#define AST_I2CC_BUS_BUSY_STS				(0x1 << 16)

#define AST_I2CC_RX_DMA_LEN					(0xff << 8)
#define AST_I2CC_TX_DMA_LEN					(0xff)

/* 0x0C : I2CC Master/Slave Pool Buffer Control Register  */
#define AST_I2CC_RX_BUF_ADDR_GET(x)			((x>> 24)& 0xff)
#define AST_I2CC_RX_BUF_END_ADDR_SET(x)			(x << 16)
#define AST_I2CC_TX_DATA_BUF_END_SET(x)			((x&0xff) << 8)
#define AST_I2CC_TX_DATA_BUF_GET(x)			((x >>8) & 0xff)
#define AST_I2CC_BUF_BASE_ADDR_SET(x)			(x & 0x3f)

/* 0x10 : I2CM Master Interrupt Control Register */
#define AST_I2CC_SDA_DL_TO				(0x1 << 14)
#define AST_I2CC_BUS_RECOVER			(0x1 << 13)
#define AST_I2CC_SMBUS_ALT				(0x1 << 12)

#define AST_I2CC_SCL_LOW_TO				(0x1 << 6)
#define AST_I2CC_ABNORMAL_COND			(0x1 << 5)
#define AST_I2CC_NORMAL_STOP			(0x1 << 4)
#define AST_I2CC_ARBIT_LOSS				(0x1 << 3)
#define AST_I2CC_RX_DOWN				(0x1 << 2)
#define AST_I2CC_TX_NAK					(0x1 << 1)
#define AST_I2CC_TX_ACK					(0x1)

/* 0x14 : I2CM Master Interrupt Status Register   : WC */
// Tx State Machine
#define AST_I2CM_IDLE	 				0x0
#define AST_I2CM_MACTIVE				0x8
#define AST_I2CM_MSTART					0x9
#define AST_I2CM_MSTARTR				0xa
#define AST_I2CM_MSTOP					0xb
#define AST_I2CM_MTXD					0xc
#define AST_I2CM_MRXACK					0xd
#define AST_I2CM_MRXD 					0xe
#define AST_I2CM_MTXACK 				0xf
#define AST_I2CM_SWAIT					0x1
#define AST_I2CM_SRXD 					0x4
#define AST_I2CM_STXACK 				0x5
#define AST_I2CM_STXD					0x6
#define AST_I2CM_SRXACK 				0x7
#define AST_I2CM_RECOVER 				0x3

#define AST_I2CM_PKT_CMD_TO				(0x1 << 18)
#define AST_I2CM_PKT_CMD_ERR			(0x1 << 17)
#define AST_I2CM_PKT_CMD_DONE			(0x1 << 16)
#define AST_I2CM_BUS_RECOVER_ERR		(0x1 << 15)

/* 0x18 : I2CM Master Command/Status Register   */
#define AST_I2CM_PKT_ADDR(x)			((x) << 24)	
#define AST_I2CM_PKT_EN					(0x1 << 16)
#define AST_I2CM_SDA_OE_OUT_DIR			(0x1 << 15)
#define AST_I2CM_SDA_O_OUT_DIR			(0x1 << 14)
#define AST_I2CM_SCL_OE_OUT_DIR			(0x1 << 13)
#define AST_I2CM_SCL_O_OUT_DIR			(0x1 << 12)
#define AST_I2CM_RECOVER_CMD_EN			(0x1 << 11)

#define AST_I2CM_RX_DMA_EN				(0x1 << 9)
#define AST_I2CM_TX_DMA_EN				(0x1 << 8)
#define AST_I2CM_RX_BUFF_EN				(0x1 << 7)
#define AST_I2CM_TX_BUFF_EN				(0x1 << 6)
#define AST_I2CM_STOP_CMD				(0x1 << 5)
#define AST_I2CM_RX_CMD_LAST			(0x1 << 4)
#define AST_I2CM_RX_CMD				(0x1 << 3)
#define AST_I2CM_TX_CMD				(0x1 << 1)
#define AST_I2CM_START_CMD			(0x1 )

/* 0x1C : I2CM Master DMA Transfer Length Register   */
#define AST_I2CM_RX_DMA_LEN(x)			((x) << 12)	/* 1 ~ 4095 */
#define AST_I2CM_TX_DMA_LEN(x)			(x)			/* 1 ~ 4095 */

#define AST_I2CM_GET_RX_DMA_LEN(x)		(((x) & 0x1FFF) >> 16)	

/* 0x20 : I2CS Page 0 Slave Interrupt Control Register   */
#define AST_I2CS_SLAVE2_ENABLE			(0x1 << 31)
#define AST_I2CS_GET_SLAVE2_ADDR(x)		((x >> 24) & 0x3F)

#define AST_I2CS_SLAVE1_ENABLE			(0x1 << 16)

#define AST_I2CS_INACTIVE_TO_ISR		(0x1 << 15)
#define AST_I2CS_ABNORMAL_SP_ISR		(0x1 << 5)
#define AST_I2CS_STOP_ISR				(0x1 << 4)
#define AST_I2CS_MATCH_NAK_ISR			(0x1 << 3)
#define AST_I2CS_RX_DONE_ISR			(0x1 << 2)
#define AST_I2CS_NAK_ISR				(0x1 << 1)
#define AST_I2CS_ACK_ISR				(0x1)

/* 0x20 : I2CS Page 1 Slave Address Register   */
#define AST_I2CS_SLAVE3_ENABLE			(0x1 << 23)
#define AST_I2CS_SLAVE3_ADDR(x)			(x << 16)

#define AST_I2CD_SLAVE2_ENABLE			(0x1 << 15)
#define AST_I2CS_SLAVE2_ADDR(x)			(x << 8)

#define AST_I2CS_SLAVE1_ADDR(x)			(x)

/* 0x24 : I2CS Slave Interrupt Status Register   */

#define AST_I2CS3_NAK_ISR				(0x1 << 22)
#define AST_I2CS2_NAK_ISR				(0x1 << 21)
#define AST_I2CS1_NAK_ISR				(0x1 << 20)

#define AST_I2CS_PKT_CMD_ERR_ISR		(0x1 << 17)
#define AST_I2CS_PKT_CMD_DONE_ISR		(0x1 << 16)
#define AST_I2CS_INACTIVE_TO_ISR		(0x1 << 15)


#define AST_I2CS_SLAVE_MATCH_ISR		(0x1 << 7)


#define AST_I2CS_ABNORMAL_SP_ISR		(0x1 << 5)
#define AST_I2CS_NORMAL_P_ISR			(0x1 << 4)
#define AST_I2CS_MACH_NAK_ISR			(0x1 << 3)
#define AST_I2CS_RX_DONE_ISR			(0x1 << 2)
#define AST_I2CS_TX_NAK_ISR				(0x1 << 1)
#define AST_I2CS_TX_ACK_ISR				(0x1)

/* 0x28 : I2CS Slave CMD/Status Register   */
#define AST_I2CS_ACTIVE_ALL				(0x3 << 17)
#define AST_I2CS_PKT_EN					(0x1 << 16)
#define AST_I2CS_AUTO_NAK_NA_EN			(0x1 << 15)
#define AST_I2CS_AUTO_NAK_EN			(0x1 << 14)

#define AST_I2CS_ALT_EN					(0x1 << 10)
#define AST_I2CS_RX_DMA_EN				(0x1 << 9)
#define AST_I2CS_TX_DMA_EN				(0x1 << 8)
#define AST_I2CS_RX_BUFF_EN				(0x1 << 7)
#define AST_I2CS_TX_BUFF_EN				(0x1 << 6)

#define AST_I2CS_RX_CMD_LAST			(0x1 << 4)

#define AST_I2CS_TX_CMD					(0x1 << 2)

/* 0x2C : I2CS SLAVE DMA Transfer Length Register   */
#define AST_I2CS_RX_DMA_LEN(x)			((x) << 12)	/* 1 ~ 4095 */
#define AST_I2CS_TX_DMA_LEN(x)			(x)			/* 1 ~ 4095 */

/***************************************************************************/
//#define AST_I2CC_DEBUG 1
#ifdef AST_I2CC_DEBUG
#define I2CC_DBUG(fmt, args...) printk(fmt, ## args)
#else
#define I2CC_DBUG(fmt, args...)
#endif

//#define AST_I2CS_DEBUG 1
#ifdef AST_I2CS_DEBUG
#define I2CS_DBUG(fmt, args...) printk(fmt, ## args)
#else
#define I2CS_DBUG(fmt, args...)
#endif

//#define AST_I2CM_DEBUG 1

#ifdef AST_I2CM_DEBUG
#define I2CM_DBUG(fmt, args...) printk(fmt, ## args)
#else
#define I2CM_DBUG(fmt, args...)
#endif

/* Use platform_data instead of module parameters */
/* Fast Mode = 400 kHz, Standard = 100 kHz */
//static int clock = 100; /* Default: 100 kHz */
/***************************************************************************/
#define I2C_S_BUF_SIZE 		256
#define I2C_S_RX_BUF_NUM 	20

#define AST_LOCKUP_DETECTED 	(0x1 << 15)
#define AST_I2C_LOW_TIMEOUT 	0x07
/***************************************************************************/
#define AST_G6_I2C_DMA_SIZE	4096
/***************************************************************************/
struct ast_i2c_timing_table {
	u32 divisor;
	u32 timing;
};

static struct ast_i2c_timing_table ast_g5_i2c_timing_table[] = {
	/* Divisor : Base Clock : tCK High : tCK Low  */
	/* Divisor :	  [3:0]    :   [19:16]:   [15:12] */
	{6,	0x77700300 | (0x0) | (0x2 << 16) | (0x2 << 12) },
	{7,	0x77700300 | (0x0) | (0x3 << 16) | (0x2 << 12) },
	{8,	0x77700300 | (0x0) | (0x3 << 16) | (0x3 << 12) },
	{9,	0x77700300 | (0x0) | (0x4 << 16) | (0x3 << 12) },
	{10, 	0x77700300 | (0x0) | (0x4 << 16) | (0x4 << 12) },
	{11, 	0x77700300 | (0x0) | (0x5 << 16) | (0x4 << 12) },
	{12, 	0x77700300 | (0x0) | (0x5 << 16) | (0x5 << 12) },
	{13, 	0x77700300 | (0x0) | (0x6 << 16) | (0x5 << 12) },
	{14, 	0x77700300 | (0x0) | (0x6 << 16) | (0x6 << 12) },
	{15, 	0x77700300 | (0x0) | (0x7 << 16) | (0x6 << 12) },
	{16, 	0x77700300 | (0x0) | (0x7 << 16) | (0x7 << 12) },
	{17, 	0x77700300 | (0x0) | (0x8 << 16) | (0x7 << 12) },
	{18, 	0x77700300 | (0x0) | (0x8 << 16) | (0x8 << 12) },
	{19, 	0x77700300 | (0x0) | (0x9 << 16) | (0x8 << 12) },
	{20, 	0x77700300 | (0x0) | (0x9 << 16) | (0x9 << 12) },
	{21, 	0x77700300 | (0x0) | (0xa << 16) | (0x9 << 12) },
	{22, 	0x77700300 | (0x0) | (0xa << 16) | (0xa << 12) },
	{23, 	0x77700300 | (0x0) | (0xb << 16) | (0xa << 12) },
	{24, 	0x77700300 | (0x0) | (0xb << 16) | (0xb << 12) },
	{25, 	0x77700300 | (0x0) | (0xc << 16) | (0xb << 12) },
	{26, 	0x77700300 | (0x0) | (0xc << 16) | (0xc << 12) },
	{27, 	0x77700300 | (0x0) | (0xd << 16) | (0xc << 12) },
	{28, 	0x77700300 | (0x0) | (0xd << 16) | (0xd << 12) },
	{29, 	0x77700300 | (0x0) | (0xe << 16) | (0xd << 12) },
	{30, 	0x77700300 | (0x0) | (0xe << 16) | (0xe << 12) },
	{31, 	0x77700300 | (0x0) | (0xf << 16) | (0xe << 12) },
	{32, 	0x77700300 | (0x0) | (0xf << 16) | (0xf << 12) },

	{34, 	0x77700300 | (0x1) | (0x8 << 16) | (0x7 << 12) },
	{36, 	0x77700300 | (0x1) | (0x8 << 16) | (0x8 << 12) },
	{38, 	0x77700300 | (0x1) | (0x9 << 16) | (0x8 << 12) },
	{40, 	0x77700300 | (0x1) | (0x9 << 16) | (0x9 << 12) },
	{42, 	0x77700300 | (0x1) | (0xa << 16) | (0x9 << 12) },
	{44, 	0x77700300 | (0x1) | (0xa << 16) | (0xa << 12) },
	{46, 	0x77700300 | (0x1) | (0xb << 16) | (0xa << 12) },
	{48, 	0x77700300 | (0x1) | (0xb << 16) | (0xb << 12) },
	{50, 	0x77700300 | (0x1) | (0xc << 16) | (0xb << 12) },
	{52, 	0x77700300 | (0x1) | (0xc << 16) | (0xc << 12) },
	{54, 	0x77700300 | (0x1) | (0xd << 16) | (0xc << 12) },
	{56, 	0x77700300 | (0x1) | (0xd << 16) | (0xd << 12) },
	{58, 	0x77700300 | (0x1) | (0xe << 16) | (0xd << 12) },
	{60, 	0x77700300 | (0x1) | (0xe << 16) | (0xe << 12) },
	{62, 	0x77700300 | (0x1) | (0xf << 16) | (0xe << 12) },
	{64, 	0x77700300 | (0x1) | (0xf << 16) | (0xf << 12) },

	{68, 	0x77700300 | (0x2) | (0x8 << 16) | (0x7 << 12) },
	{72, 	0x77700300 | (0x2) | (0x8 << 16) | (0x8 << 12) },
	{76, 	0x77700300 | (0x2) | (0x9 << 16) | (0x8 << 12) },
	{80, 	0x77700300 | (0x2) | (0x9 << 16) | (0x9 << 12) },
	{84, 	0x77700300 | (0x2) | (0xa << 16) | (0x9 << 12) },
	{88, 	0x77700300 | (0x2) | (0xa << 16) | (0xa << 12) },
	{92, 	0x77700300 | (0x2) | (0xb << 16) | (0xa << 12) },
	{96, 	0x77700300 | (0x2) | (0xb << 16) | (0xb << 12) },
	{100, 	0x77700300 | (0x2) | (0xc << 16) | (0xb << 12) },
	{104, 	0x77700300 | (0x2) | (0xc << 16) | (0xc << 12) },
	{108, 	0x77700300 | (0x2) | (0xd << 16) | (0xc << 12) },
	{112, 	0x77700300 | (0x2) | (0xd << 16) | (0xd << 12) },
	{116, 	0x77700300 | (0x2) | (0xe << 16) | (0xd << 12) },
	{120, 	0x77700300 | (0x2) | (0xe << 16) | (0xe << 12) },
	{124, 	0x77700300 | (0x2) | (0xf << 16) | (0xe << 12) },
	{128, 	0x77700300 | (0x2) | (0xf << 16) | (0xf << 12) },

	{136, 	0x77700300 | (0x3) | (0x8 << 16) | (0x7 << 12) },
	{144, 	0x77700300 | (0x3) | (0x8 << 16) | (0x8 << 12) },
	{152, 	0x77700300 | (0x3) | (0x9 << 16) | (0x8 << 12) },
	{160, 	0x77700300 | (0x3) | (0x9 << 16) | (0x9 << 12) },
	{168, 	0x77700300 | (0x3) | (0xa << 16) | (0x9 << 12) },
	{176, 	0x77700300 | (0x3) | (0xa << 16) | (0xa << 12) },
	{184, 	0x77700300 | (0x3) | (0xb << 16) | (0xa << 12) },
	{192, 	0x77700300 | (0x3) | (0xb << 16) | (0xb << 12) },
	{200, 	0x77700300 | (0x3) | (0xc << 16) | (0xb << 12) },
	{208, 	0x77700300 | (0x3) | (0xc << 16) | (0xc << 12) },
	{216, 	0x77700300 | (0x3) | (0xd << 16) | (0xc << 12) },
	{224, 	0x77700300 | (0x3) | (0xd << 16) | (0xd << 12) },
	{232, 	0x77700300 | (0x3) | (0xe << 16) | (0xd << 12) },
	{240, 	0x77700300 | (0x3) | (0xe << 16) | (0xe << 12) },
	{248, 	0x77700300 | (0x3) | (0xf << 16) | (0xe << 12) },
	{256, 	0x77700300 | (0x3) | (0xf << 16) | (0xf << 12) },

	{272, 	0x77700300 | (0x4) | (0x8 << 16) | (0x7 << 12) },
	{288, 	0x77700300 | (0x4) | (0x8 << 16) | (0x8 << 12) },
	{304, 	0x77700300 | (0x4) | (0x9 << 16) | (0x8 << 12) },
	{320, 	0x77700300 | (0x4) | (0x9 << 16) | (0x9 << 12) },
	{336, 	0x77700300 | (0x4) | (0xa << 16) | (0x9 << 12) },
	{352, 	0x77700300 | (0x4) | (0xa << 16) | (0xa << 12) },
	{368, 	0x77700300 | (0x4) | (0xb << 16) | (0xa << 12) },
	{384, 	0x77700300 | (0x4) | (0xb << 16) | (0xb << 12) },
	{400, 	0x77700300 | (0x4) | (0xc << 16) | (0xb << 12) },
	{416, 	0x77700300 | (0x4) | (0xc << 16) | (0xc << 12) },
	{432, 	0x77700300 | (0x4) | (0xd << 16) | (0xc << 12) },
	{448, 	0x77700300 | (0x4) | (0xd << 16) | (0xd << 12) },
	{464, 	0x77700300 | (0x4) | (0xe << 16) | (0xd << 12) },
	{480, 	0x77700300 | (0x4) | (0xe << 16) | (0xe << 12) },
	{496, 	0x77700300 | (0x4) | (0xf << 16) | (0xe << 12) },
	{512, 	0x77700300 | (0x4) | (0xf << 16) | (0xf << 12) },

	{544, 	0x77700300 | (0x5) | (0x8 << 16) | (0x7 << 12) },
	{576, 	0x77700300 | (0x5) | (0x8 << 16) | (0x8 << 12) },
	{608, 	0x77700300 | (0x5) | (0x9 << 16) | (0x8 << 12) },
	{640, 	0x77700300 | (0x5) | (0x9 << 16) | (0x9 << 12) },
	{672, 	0x77700300 | (0x5) | (0xa << 16) | (0x9 << 12) },
	{704, 	0x77700300 | (0x5) | (0xa << 16) | (0xa << 12) },
	{736, 	0x77700300 | (0x5) | (0xb << 16) | (0xa << 12) },
	{768, 	0x77700300 | (0x5) | (0xb << 16) | (0xb << 12) },
	{800, 	0x77700300 | (0x5) | (0xc << 16) | (0xb << 12) },
	{832, 	0x77700300 | (0x5) | (0xc << 16) | (0xc << 12) },
	{864, 	0x77700300 | (0x5) | (0xd << 16) | (0xc << 12) },
	{896, 	0x77700300 | (0x5) | (0xd << 16) | (0xd << 12) },
	{928, 	0x77700300 | (0x5) | (0xe << 16) | (0xd << 12) },
	{960, 	0x77700300 | (0x5) | (0xe << 16) | (0xe << 12) },
	{992, 	0x77700300 | (0x5) | (0xf << 16) | (0xe << 12) },
	{1024, 	0x77700300 | (0x5) | (0xf << 16) | (0xf << 12) },

	{1088, 	0x77700300 | (0x6) | (0x8 << 16) | (0x7 << 12) },
	{1152, 	0x77700300 | (0x6) | (0x8 << 16) | (0x8 << 12) },
	{1216, 	0x77700300 | (0x6) | (0x9 << 16) | (0x8 << 12) },
	{1280, 	0x77700300 | (0x6) | (0x9 << 16) | (0x9 << 12) },
	{1344, 	0x77700300 | (0x6) | (0xa << 16) | (0x9 << 12) },
	{1408, 	0x77700300 | (0x6) | (0xa << 16) | (0xa << 12) },
	{1472, 	0x77700300 | (0x6) | (0xb << 16) | (0xa << 12) },
	{1536, 	0x77700300 | (0x6) | (0xb << 16) | (0xb << 12) },
	{1600, 	0x77700300 | (0x6) | (0xc << 16) | (0xb << 12) },
	{1664, 	0x77700300 | (0x6) | (0xc << 16) | (0xc << 12) },
	{1728, 	0x77700300 | (0x6) | (0xd << 16) | (0xc << 12) },
	{1792, 	0x77700300 | (0x6) | (0xd << 16) | (0xd << 12) },
	{1856, 	0x77700300 | (0x6) | (0xe << 16) | (0xd << 12) },
	{1920, 	0x77700300 | (0x6) | (0xe << 16) | (0xe << 12) },
	{1984, 	0x77700300 | (0x6) | (0xf << 16) | (0xe << 12) },
	{2048, 	0x77700300 | (0x6) | (0xf << 16) | (0xf << 12) },

	{2176, 	0x77700300 | (0x7) | (0x8 << 16) | (0x7 << 12) },
	{2304, 	0x77700300 | (0x7) | (0x8 << 16) | (0x8 << 12) },
	{2432, 	0x77700300 | (0x7) | (0x9 << 16) | (0x8 << 12) },
	{2560, 	0x77700300 | (0x7) | (0x9 << 16) | (0x9 << 12) },
	{2688, 	0x77700300 | (0x7) | (0xa << 16) | (0x9 << 12) },
	{2816, 	0x77700300 | (0x7) | (0xa << 16) | (0xa << 12) },
	{2944, 	0x77700300 | (0x7) | (0xb << 16) | (0xa << 12) },
	{3072, 	0x77700300 | (0x7) | (0xb << 16) | (0xb << 12) },
};

struct ast_i2c_bus {
//	struct ast_i2c_driver_data *ast_i2c_data;
	struct device			*dev;
	void __iomem			*reg_base;		/* virtual */
	int 				irq;			//I2C IRQ number
	struct clk 			*clk;
	u32				apb_clk;
	u32				base_clk1;
	u32				base_clk2;
	u32				base_clk3;
	u32				base_clk4;
	u32				bus_frequency;
//buff mode info
	u32				bus_clk;
	u32				bus_recover;
	struct i2c_adapter		adap;
//master
	struct completion	cmd_complete;
	int				cmd_err;
//Slave structure
	struct i2c_msg	*slave_msgs; 		//cur slave xfer msgs
	void	(*do_slave_xfer)(struct ast_i2c_bus *i2c_bus);
	u8				slave_en;
#ifdef CONFIG_AST_I2C_SLAVE_MODE
	unsigned char		*slave_dma_buf;
	dma_addr_t			slave_dma_addr;
	u8				slave_rx_full;
	u8				slave_tx_full;
	u8				slave_rx_idx;
	u8				slave_ioctl_idx;
	struct i2c_msg			slave_rx_msg[I2C_S_RX_BUF_NUM];
	struct i2c_msg			slave_tx_msg;

#endif
};



static inline void
ast_i2c_write(struct ast_i2c_bus *i2c_bus, u32 val, u32 reg)
{
//	dev_dbg(i2c_bus->dev, "ast_i2c_write : val: %x , reg : %x \n",val,reg);
	writel(val, i2c_bus->reg_base + reg);
}

static inline u32
ast_i2c_read(struct ast_i2c_bus *i2c_bus, u32 reg)
{
#if 0
	u32 val = readl(i2c_bus->reg_base + reg);
	printk("R : reg %x , val: %x \n", reg, val);
	return val;
#else
	return readl(i2c_bus->reg_base + reg);
#endif
}

#ifdef CONFIG_AST_I2C_SLAVE_MODE
static void ast_slave_issue_alert(struct ast_i2c_bus *i2c_bus, u8 enable)
{
	//only support dev0~3
	if (i2c_bus->adap.nr > 3)
		return;
	else {
		if (enable)
			ast_i2c_write(i2c_bus, ast_i2c_read(i2c_bus, AST_I2CS_CMD_STS) | AST_I2CS_ALT_EN,
				      AST_I2CS_CMD_STS);
		else
			ast_i2c_write(i2c_bus, ast_i2c_read(i2c_bus, AST_I2CS_CMD_STS) & ~AST_I2CS_ALT_EN,
				      AST_I2CS_CMD_STS);
	}
}

/*
msgs->buf[0] = enable /disable
msgs->buf[1] = address idx
*/
static void ast_slave_mode_enable(struct ast_i2c_bus *i2c_bus,
				  struct i2c_msg *msgs)
{
	//set page 1 for set address 
	ast_i2c_write(i2c_bus, ast_i2c_read(i2c_bus,
			AST_I2CC_FUN_CTRL) | AST_I2CC_PAGE_SEL, AST_I2CC_FUN_CTRL);

	if (msgs->buf[0] == 1) {
		if (msgs->len == 1) {
			ast_i2c_write(i2c_bus, ast_i2c_read(i2c_bus,
						AST_I2CS_IER) | msgs->addr, AST_I2CS_IER);
		} else {
			
			switch(msgs->buf[1]) {
				case 0:
					ast_i2c_write(i2c_bus, ast_i2c_read(i2c_bus,
								AST_I2CS_IER) | msgs->addr, AST_I2CS_IER);
					break;
				case 1:
					ast_i2c_write(i2c_bus, ast_i2c_read(i2c_bus,
								AST_I2CS_IER) | AST_I2CS_SLAVE2_ADDR(msgs->addr) | AST_I2CS_SLAVE2_ENABLE, AST_I2CS_IER);
					break;
				case 2:
					ast_i2c_write(i2c_bus, ast_i2c_read(i2c_bus,
								AST_I2CS_IER) | AST_I2CS_SLAVE3_ADDR(msgs->addr) | AST_I2CS_SLAVE3_ENABLE, AST_I2CS_IER);
					break;
				default:
					printk("ERROR \n");
					break;
			}
			
		}
		ast_i2c_write(i2c_bus, ast_i2c_read(i2c_bus, AST_I2CC_FUN_CTRL) | AST_I2CC_SLAVE_EN, AST_I2CC_FUN_CTRL);
		//prepare for rx 
		ast_i2c_write(i2c_bus, AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_EN | AST_I2CS_RX_DMA_EN | AST_I2CS_RX_CMD_LAST, AST_I2CS_CMD_STS);
	} else {
		if (msgs->len == 1) {
				ast_i2c_write(i2c_bus, ast_i2c_read(i2c_bus,
							AST_I2CC_FUN_CTRL) & ~AST_I2CC_SLAVE_EN, AST_I2CC_FUN_CTRL);
		} else {
			switch(msgs->buf[1]) {
				case 0:
					ast_i2c_write(i2c_bus, ast_i2c_read(i2c_bus,
								AST_I2CC_FUN_CTRL) & ~AST_I2CC_SLAVE_EN, AST_I2CC_FUN_CTRL);
					break;
				case 1:
					ast_i2c_write(i2c_bus, ast_i2c_read(i2c_bus,
								AST_I2CS_IER) & ~AST_I2CS_SLAVE2_ENABLE, AST_I2CS_IER);
					break;
				case 2:
					ast_i2c_write(i2c_bus, ast_i2c_read(i2c_bus,
								AST_I2CS_IER) & ~AST_I2CS_SLAVE3_ENABLE, AST_I2CS_IER);
					break;
				default:
					printk("ERROR \n");
					break;
			}
			
		}
	}
}

#define AST_I2C_DMA_SIZE 4096
/*
	0 ~ I2C_S_RX_BUF_NUM : is rx dma
	I2C_S_RX_BUF_NUM : is tx dma
*/
static void ast_i2c_slave_buff_init(struct ast_i2c_bus *i2c_bus)
{
	int i;
	
	i2c_bus->slave_dma_buf = dma_alloc_coherent(NULL, AST_I2C_DMA_SIZE * (I2C_S_RX_BUF_NUM + 1), &i2c_bus->slave_dma_addr, GFP_KERNEL);

	//Rx buf 4
	for (i = 0; i < I2C_S_RX_BUF_NUM; i++) {
		i2c_bus->slave_rx_msg[i].addr = 0;	//mean ~BUFF_ONGOING
		i2c_bus->slave_rx_msg[i].flags = 0;	//mean empty buffer
		i2c_bus->slave_rx_msg[i].len = I2C_S_BUF_SIZE;
		i2c_bus->slave_rx_msg[i].buf = i2c_bus->slave_dma_buf + (AST_I2C_DMA_SIZE * i);
	}

	//Tx buf  1
	i2c_bus->slave_tx_msg.len = I2C_S_BUF_SIZE;
	i2c_bus->slave_tx_msg.buf = i2c_bus->slave_dma_buf + (AST_I2C_DMA_SIZE * I2C_S_RX_BUF_NUM);

	//assign cur msg is #0
	i2c_bus->slave_msgs = &i2c_bus->slave_rx_msg[0];
	i2c_bus->slave_rx_full = 0;
	i2c_bus->slave_rx_idx = 0;
	i2c_bus->slave_ioctl_idx = 0;

	//assign dma rx and addr
	ast_i2c_write(i2c_bus, i2c_bus->slave_dma_addr, AST_I2CS_RX_DMA);
	ast_i2c_write(i2c_bus, (I2C_S_BUF_SIZE << 12), AST_I2CS_DMA_LEN);

	ast_i2c_write(i2c_bus, (u32) (i2c_bus->slave_dma_buf + (AST_I2C_DMA_SIZE * I2C_S_RX_BUF_NUM)), AST_I2CS_TX_DMA);

	/* Set slave IER */
	ast_i2c_write(i2c_bus, 
			AST_I2CS_INACTIVE_TO_ISR |
			AST_I2CS_ABNORMAL_SP_ISR |
			AST_I2CS_STOP_ISR |
			AST_I2CS_MATCH_NAK_ISR |
			AST_I2CS_RX_DONE_ISR |
			AST_I2CS_NAK_ISR |
			AST_I2CS_ACK_ISR,
			AST_I2CS_IER);	
}

static int ast_i2c_slave_ioctl_xfer(struct i2c_adapter *adap,
				    struct i2c_msg *msgs)
{
	struct ast_i2c_bus *i2c_bus = adap->algo_data;
	struct i2c_msg *slave_rx_msg = &i2c_bus->slave_rx_msg[i2c_bus->slave_ioctl_idx];
	int ret = 0;
//	int i;
	switch (msgs->flags) {
	case 0:
		if ((slave_rx_msg->addr == 0) && (slave_rx_msg->flags == BUFF_FULL)) {
			dev_dbg(i2c_bus->dev, "I2C_SLAVE_EVENT_STOP buf idx %d : len %d \n",
				i2c_bus->slave_ioctl_idx, slave_rx_msg->len);
#if 0
			printk("slave rx buff \n");
			for (i = 0; i < slave_rx_msg->len; i++)
				printk("%x ", slave_rx_msg->buf[i]);
			printk("\n");
#endif
			if (slave_rx_msg->len != 0)
				memcpy(msgs->buf, slave_rx_msg->buf, slave_rx_msg->len);
			msgs->len = slave_rx_msg->len;
			slave_rx_msg->len = 0;
			slave_rx_msg->flags = 0;
			i2c_bus->slave_ioctl_idx++;
			i2c_bus->slave_ioctl_idx %= I2C_S_RX_BUF_NUM;
			if (i2c_bus->slave_rx_full) {
				dev_err(i2c_bus->dev, "slave re-enable \n");
				i2c_bus->slave_rx_full = 0;
				ast_i2c_write(i2c_bus, ast_i2c_read(i2c_bus, AST_I2CC_FUN_CTRL) | AST_I2CC_SLAVE_EN,
					      AST_I2CC_FUN_CTRL);
			}
		} else {
//			printk("%d-No Buff\n", i2c_bus->adap.nr);
			msgs->len = 0;
			ret = -1;
		}
		break;
	case I2C_M_RD:	//slave tx
		dev_dbg(i2c_bus->dev, "slave write \n");
		if(i2c_bus->slave_tx_full)
			return -1;
		memcpy(msgs->buf, i2c_bus->slave_tx_msg.buf, msgs->len);
		//tx dma enable and size 
		ast_i2c_write(i2c_bus, ast_i2c_read(i2c_bus, (AST_I2CS_DMA_LEN) & (0xfff << 12)) | (msgs->len), AST_I2CS_DMA_LEN);
		
		//trigger tx : Issue can I do this ??
		i2c_bus->slave_tx_full = 1;
		ast_i2c_write(i2c_bus, AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_EN | AST_I2CS_TX_DMA_EN, AST_I2CS_CMD_STS);
		break;
	case I2C_S_EN:
		if ((msgs->addr < 0x1) || (msgs->addr > 0xff)) {
			ret = -1;
			printk("addrsss not correct !! \n");
			ret = -1;
		} else {
			ast_slave_mode_enable(i2c_bus, msgs);
		}
		break;
	case I2C_S_ALT:
		printk("slave issue alt\n");
		if (msgs->len != 1) printk("ERROR \n");
		if (msgs->buf[0] == 1)
			ast_slave_issue_alert(i2c_bus, 1);
		else
			ast_slave_issue_alert(i2c_bus, 0);
		break;
	default:
		printk("slave xfer error \n");
		break;

	}
	return ret;
}

static void ast_g6_i2c_slave_handler(struct ast_i2c_bus *i2c_bus)
{
	u32 sisr = ast_i2c_read(i2c_bus, AST_I2CS_ISR);

	dev_dbg(i2c_bus->dev, "sISR : %x\n", sisr);

	if(sisr & AST_I2CS_PKT_CMD_DONE_ISR) {
		//check tx or rx 
		if(sisr & AST_I2CS_RX_DONE_ISR) {
			i2c_bus->slave_msgs->addr = 0;
			i2c_bus->slave_msgs->flags = BUFF_FULL;
			if (i2c_bus->slave_rx_msg[(i2c_bus->slave_rx_idx + 1) % I2C_S_RX_BUF_NUM].flags == BUFF_FULL) {
				i2c_bus->slave_rx_full = 1;
				//no trigger next rx
				printk("slave full check !!! \n");
			} else {
				i2c_bus->slave_rx_idx++;
				i2c_bus->slave_rx_idx %= I2C_S_RX_BUF_NUM;
				i2c_bus->slave_msgs = &i2c_bus->slave_rx_msg[i2c_bus->slave_rx_idx];
				
				ast_i2c_write(i2c_bus, (u32) (i2c_bus->slave_dma_buf + (AST_I2C_DMA_SIZE * i2c_bus->slave_rx_idx)), AST_I2CS_RX_DMA);

				ast_i2c_write(i2c_bus, (I2C_S_BUF_SIZE << 12), AST_I2CS_DMA_LEN);

				//trigger next rx 
				ast_i2c_write(i2c_bus, AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_EN | AST_I2CS_RX_DMA_EN, AST_I2CS_CMD_STS);
			}
		}

		if(sisr & AST_I2CS_TX_ACK_ISR) {
			i2c_bus->slave_tx_full = 0;
		}
	}

	if(sisr & AST_I2CS_PKT_CMD_ERR_ISR) {
		printk("AST_I2CS_PKT_CMD_ERR_ISR TODO check \n");
		while(1);
	}

	if(sisr & AST_I2CS_INACTIVE_TO_ISR) {
		printk("AST_I2CS_INACTIVE_TO_ISR TODO check \n");
		while(1);
	}

	ast_i2c_write(i2c_bus, sisr, AST_I2CS_ISR);
}
#endif


static void ast_g6_i2c_master_handler(struct ast_i2c_bus *i2c_bus)
{
	u32 misr = ast_i2c_read(i2c_bus, AST_I2CM_ISR) & 0x0fffffff;

	dev_dbg(i2c_bus->dev, "mISR : %x \n", misr);

	if(misr & AST_I2CM_BUS_RECOVER_ERR) {
		printk("AST_I2CM_BUS_RECOVER_ERR \n");
		i2c_bus->cmd_err = misr;
	}

	if(misr & AST_I2CM_PKT_CMD_DONE)
		printk("AST_I2CM_PKT_CMD_DONE \n");


	if(misr & AST_I2CM_PKT_CMD_ERR) {
		printk("AST_I2CM_PKT_CMD_ERR \n");
		i2c_bus->cmd_err = misr;
	}

	ast_i2c_write(i2c_bus, misr, AST_I2CM_ISR);

	complete(&i2c_bus->cmd_complete);
}

static irqreturn_t ast_g6_i2c_handler(int irq, void *dev_id)
{
	struct ast_i2c_bus *i2c_bus = dev_id;

	u32 misr = ast_i2c_read(i2c_bus, AST_I2CM_ISR);

//	dev_dbg(i2c_bus->dev, "mISR : %x, sISR : %x\n", misr, sisr);

	if(misr & 0x0fffffff)
		ast_g6_i2c_master_handler(i2c_bus);
	else
		ast_g6_i2c_slave_handler(i2c_bus);

	return IRQ_HANDLED;

}

#if 0
 Issue ##
I2C_M_TEN	/* this is a ten bit chip address */
I2C_M_RECV_LEN		/* length will be first received byte */
I2C_M_NO_RD_ACK		/* if I2C_FUNC_PROTOCOL_MANGLING */
I2C_M_IGNORE_NAK	/* if I2C_FUNC_PROTOCOL_MANGLING */
I2C_M_REV_DIR_ADDR	/* if I2C_FUNC_PROTOCOL_MANGLING */
I2C_M_NOSTART		/* if I2C_FUNC_NOSTART */
I2C_M_STOP		/* if I2C_FUNC_PROTOCOL_MANGLING */
#endif

static int ast_g6_i2c_do_msgs_xfer(struct ast_i2c_bus *i2c_bus,
				struct i2c_msg *msgs, int num)
{
	int i;
	int ret = 0;
	long timeout = 0;
	dma_addr_t		dma_addr;
	u32 cmd;
	struct i2c_msg 	*master_msgs;
	dev_dbg(i2c_bus->dev, "ast_i2c_do_msgs_xfer\n");

	if(num > 2) printk("TODO Check ~~~~~ \n");

	for (i = 0; i < num; i++) {
		master_msgs = &msgs[i];

		i2c_bus->cmd_err = 0;

		//send start
		dev_dbg(i2c_bus->dev, " %sing %d byte%s %s 0x%02x\n",
			master_msgs->flags & I2C_M_RD ? "read" : "write",
			master_msgs->len, master_msgs->len > 1 ? "s" : "",
			master_msgs->flags & I2C_M_RD ? "from" : "to",
			master_msgs->addr);

		if (master_msgs->flags & I2C_M_RD) {
			dma_addr = dma_map_single(i2c_bus->dev, master_msgs->buf, master_msgs->len, DMA_FROM_DEVICE);
			if (dma_mapping_error(i2c_bus->dev, dma_addr)) {
				dev_err(i2c_bus->dev, "rx dma map failed ======== TODO \n");
				return -1;
			}
			ast_i2c_write(i2c_bus, dma_addr, AST_I2CM_RX_DMA);
			ast_i2c_write(i2c_bus, (master_msgs->len - 1) << 12, AST_I2CM_DMA_LEN);
			cmd = AST_I2CM_PKT_ADDR(master_msgs->addr) | AST_I2CM_PKT_EN | AST_I2CM_RX_CMD | AST_I2CM_START_CMD;
		} else {
			dma_addr = dma_map_single(i2c_bus->dev, master_msgs->buf, master_msgs->len, DMA_TO_DEVICE);
			if (dma_mapping_error(i2c_bus->dev, dma_addr)) {
				dev_err(i2c_bus->dev, "tx dma map failed ======== TODO \n");
				return -1;
			}
			ast_i2c_write(i2c_bus, dma_addr, AST_I2CM_TX_DMA);
			ast_i2c_write(i2c_bus, (master_msgs->len - 1), AST_I2CM_DMA_LEN);
			cmd = AST_I2CM_PKT_ADDR(master_msgs->addr) | AST_I2CM_PKT_EN | AST_I2CM_TX_CMD | AST_I2CM_START_CMD;
		}

		if (num == i + 1)
			cmd |= AST_I2CM_STOP_CMD;

		init_completion(&i2c_bus->cmd_complete);

		dev_dbg(i2c_bus->dev, "trigger cmd %x, dma_addr %x \n", cmd, dma_addr);	 
		ast_i2c_write(i2c_bus, cmd, AST_I2CM_CMD_STS);

		timeout = wait_for_completion_timeout(&i2c_bus->cmd_complete,
						      i2c_bus->adap.timeout * HZ);

		if (master_msgs->flags & I2C_M_RD) {
			dma_unmap_single(i2c_bus->dev, dma_addr,
					 master_msgs->len, DMA_FROM_DEVICE);
		} else {
			dma_unmap_single(i2c_bus->dev, dma_addr,
					 master_msgs->len, DMA_TO_DEVICE);			
		}

		if (timeout <= 0) {
			if (timeout == 0) {
				dev_err(i2c_bus->dev, "controller timed out\n");
				ret = -ETIMEDOUT;
			} else if (timeout == -ERESTARTSYS) {
				dev_err(i2c_bus->dev, "controller ERESTARTSYS\n");
				ret = -ERESTARTSYS;
			}
			dev_dbg(i2c_bus->dev, "timeout = %lx, master state [%x]\n", timeout, ast_i2c_read(i2c_bus, AST_I2CM_CMD_STS) >> 28);
			goto out;
		}

		if (i2c_bus->cmd_err != 0) {
			dev_dbg(i2c_bus->dev, "i2c_bus->cmd_err = %x, master state [%x]\n", i2c_bus->cmd_err, ast_i2c_read(i2c_bus, AST_I2CM_CMD_STS) >> 28);
#if 0
			if (i2c_bus->cmd_err == (AST_I2CD_INTR_STS_TX_NAK |
						 AST_I2CD_INTR_STS_NORMAL_STOP)) {
				dev_dbg(i2c_bus->dev, "go out \n");
				ret = -ETIMEDOUT;
				goto out;
			} else if (i2c_bus->cmd_err == AST_I2CD_INTR_STS_ABNORMAL) {
				dev_dbg(i2c_bus->dev, "go out \n");
				ret = -ETIMEDOUT;
				goto out;
			} else {
				dev_dbg(i2c_bus->dev, "send stop \n");
				ret = -EAGAIN;
				goto stop;
			}
#else
			if(i2c_bus->cmd_err) {
				printk("TODO \n");
				goto out;
			}
#endif			
		}
		ret++;
	}	

	if (i2c_bus->cmd_err == 0)
		goto out;
#if 0	
stop:
	init_completion(&i2c_bus->cmd_complete);
	ast_i2c_write(i2c_bus, AST_I2CM_STOP_CMD, AST_I2CM_CMD_STS);
	timeout = wait_for_completion_timeout(&i2c_bus->cmd_complete,
					      i2c_bus->adap.timeout * HZ);
	if (timeout == 0) {
		dev_dbg(i2c_bus->dev, "send stop timed out\n");
		ret = -ETIMEDOUT;
	}
#endif
out:
	dev_dbg(i2c_bus->dev, "end xfer ret = %d \n", ret);
	return ret;

}

/*
 Issue #1 MS -> MP -> SS
 Issue #2 Should I wait for slave ??
*/
static int ast_g6_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct ast_i2c_bus *i2c_bus = adap->algo_data;
	int ret = 0, i;

	/*No Wait for the bus to become free just send when master not idle but sw timeout should set with max */
	if((ast_i2c_read(i2c_bus, AST_I2CM_ISR) >> 28) != 0) {
		dev_dbg(i2c_bus->dev, "i2c_ast: waiting for master free\n");
		goto out;
	}

	for (i = adap->retries; i >= 0; i--) {
		ret = ast_g6_i2c_do_msgs_xfer(i2c_bus, msgs, num);
		if (ret != -EAGAIN)
			goto out;
		dev_dbg(i2c_bus->dev, "Retrying transmission [%d]\n", i);
	}

	ret = -EREMOTEIO;
out:
	dev_dbg(i2c_bus->dev, "%d-m: end \n", i2c_bus->adap.nr);
	return ret;
}

/*
 *	Issue #1 I2CC00 page control, when init and some interrupt will have miss-match ex when re-init the i2c bus
 *	Issue #2 Check what condition isr master / slave all have 
 */
static void ast_g6_i2c_bus_init(struct ast_i2c_bus *i2c_bus)
{
	int i;
	u32 data;
	u32 timeout;

	u32 div, divider_ratio, inc;
	u32 scl_low, scl_high, scl_highmin;

	//I2CG Reset
	ast_i2c_write(i2c_bus, 0, AST_I2CC_FUN_CTRL);

	ast_i2c_write(i2c_bus, AST_I2CC_SLAVE_ADDR_RX_EN | AST_I2CC_BUS_AUTO_RELEASE | AST_I2CC_MASTER_EN,
			      AST_I2CC_FUN_CTRL);

#if 0
	/* Set AC Timing */
	//TODO  
	if((i2c_bus->apb_clk /i2c_bus->bus_frequency) <= 32) {
	  div = 0;
	  divider_ratio = i2c_bus->apb_clk /i2c_bus->bus_frequency;
	} else if((i2c_bus->base_clk1 /i2c_bus->bus_frequency) <= 32) {
	  div = 1;
	  divider_ratio = i2c_bus->base_clk2 /i2c_bus->bus_frequency;
	} else if((i2c_bus->base_clk2 /i2c_bus->bus_frequency) <= 32) {
	  div = 2;
	  divider_ratio = i2c_bus->base_clk3 /i2c_bus->bus_frequency;	  
	} else if((i2c_bus->base_clk3 /i2c_bus->bus_frequency) <= 32) {
	  div = 3;
	  divider_ratio = i2c_bus->base_clk3 /i2c_bus->bus_frequency;	  
	} else {
	  divider_ratio = i2c_bus->base_clk4 /i2c_bus->bus_frequency;	  
	  div = 4;
	  inc = 0;
	  while((divider_ratio + inc) > 32) {
		  inc |= divider_ratio & 0x1;
		  divider_ratio >> = 1;
		  div++;
	  }
	  divider_ratio += inc;
	}

	scl_low = (divider_ratio >> 1) - 1;
	scl_high = divider_ratio - scl_low -2;
//	scl_highmin = 
	ast_i2c_write(i2c_bus, select_i2c_clock(i2c_bus), AST_I2CC_AC_TIMING);
#else
	for (i = 0; i < sizeof(ast_g5_i2c_timing_table); i++) {
		if ((i2c_bus->apb_clk / ast_g5_i2c_timing_table[i].divisor) <
		    i2c_bus->bus_frequency) {
			break;
		}
	}
	divider_ratio = ast_g5_i2c_timing_table[i].timing;

	//TODO 
	timeout = 0x7;

	/* Set AC Timing */
	ast_i2c_write(i2c_bus, divider_ratio | timeout, AST_I2CC_AC_TIMING);
#endif
	
	//Clear master/slave Interrupt
	ast_i2c_write(i2c_bus, 0xfffffff, AST_I2CM_ISR);
	ast_i2c_write(i2c_bus, 0xfffffff, AST_I2CS_ISR);

	/* Set master IER */
	ast_i2c_write(i2c_bus, 
			AST_I2CC_SDA_DL_TO |
			AST_I2CC_BUS_RECOVER |
			AST_I2CC_SMBUS_ALT |
			AST_I2CC_SCL_LOW_TO |
			AST_I2CC_ABNORMAL_COND |
			AST_I2CC_NORMAL_STOP |
			AST_I2CC_ARBIT_LOSS |
			AST_I2CC_RX_DOWN |
			AST_I2CC_TX_NAK |
			AST_I2CC_TX_ACK,
			AST_I2CM_IER);


#ifdef CONFIG_AST_I2C_SLAVE_MODE
	ast_i2c_slave_buff_init(i2c_bus);
#endif

}

static u32 ast_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_BLOCK_DATA;
}

static const struct i2c_algorithm i2c_ast_algorithm = {
	.master_xfer	= ast_g6_i2c_xfer,
#ifdef CONFIG_AST_I2C_SLAVE_MODE
	.slave_xfer		= ast_i2c_slave_ioctl_xfer,
#endif
	.functionality	= ast_i2c_functionality,
};

static int ast_i2c_probe(struct platform_device *pdev)
{
	struct ast_i2c_bus *i2c_bus;
	struct resource *res;
	int bus_nr;
	int ret = 0;

	dev_dbg(&pdev->dev, "ast_g6_i2c_probe \n");

	i2c_bus = devm_kzalloc(&pdev->dev, sizeof(struct ast_i2c_bus), GFP_KERNEL);
	if (!i2c_bus) {
		return -ENOMEM;
	}

	i2c_bus->dev = &pdev->dev;
	init_completion(&i2c_bus->cmd_complete);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENODEV;
		goto free_mem;
	}

	i2c_bus->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!i2c_bus->reg_base) {
		ret = -EIO;
		goto release_mem;
	}

	i2c_bus->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (i2c_bus->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -i2c_bus->irq;
		goto free_irq;
	}

	platform_set_drvdata(pdev, i2c_bus);

	i2c_bus->clk = devm_clk_get(i2c_bus->dev, NULL);
	if (IS_ERR(i2c_bus->clk)) {
		dev_err(i2c_bus->dev, "no clock defined\n");
		return -ENODEV;
	}
	i2c_bus->apb_clk = clk_get_rate(i2c_bus->clk);
	dev_dbg(i2c_bus->dev, "i2c_bus->apb_clk %d \n", i2c_bus->apb_clk);

	ret = of_property_read_u32(pdev->dev.of_node,
				   "bus-frequency", &i2c_bus->bus_frequency);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Could not read bus-frequency property\n");
		i2c_bus->bus_frequency = 100000;
	}

	/* Initialize the I2C adapter */
	i2c_set_adapdata(&i2c_bus->adap, i2c_bus);
	i2c_bus->adap.owner = THIS_MODULE;
	i2c_bus->adap.class = I2C_CLASS_DEPRECATED;
	i2c_bus->adap.algo = &i2c_ast_algorithm;
	i2c_bus->adap.dev.parent = i2c_bus->dev;
	i2c_bus->adap.timeout = 3;
	i2c_bus->adap.dev.of_node = pdev->dev.of_node;
	i2c_bus->adap.algo_data = i2c_bus;
	i2c_bus->adap.retries = 3;

	/* Try to set the I2C adapter number from dt */
	bus_nr = of_alias_get_id(pdev->dev.of_node, "i2c");

	snprintf(i2c_bus->adap.name, sizeof(i2c_bus->adap.name), "ast_i2c.%u",
		 bus_nr);

	ast_g6_i2c_bus_init(i2c_bus);

#ifdef CONFIG_AST_I2C_SLAVE_MODE
	ast_i2c_slave_buff_init(i2c_bus);
#endif

	ret = devm_request_irq(&pdev->dev, i2c_bus->irq, ast_g6_i2c_handler,
			       0, dev_name(&pdev->dev), i2c_bus);
	if (ret) {
		printk(KERN_INFO "I2C: Failed request irq %d\n", i2c_bus->irq);
		goto unmap;
	}

	ret = i2c_add_adapter(&i2c_bus->adap);
	if (ret < 0) {
		printk(KERN_INFO "I2C: Failed to add bus\n");
		goto free_irq;
	}

	printk(KERN_INFO "I2C: %s [%d]: AST I2C adapter [%d khz] \n",
	       pdev->dev.of_node->name, i2c_bus->adap.nr,
	       i2c_bus->bus_frequency / 1000);

	return 0;

unmap:
	free_irq(i2c_bus->irq, i2c_bus);
free_irq:
	iounmap(i2c_bus->reg_base);
release_mem:
	release_mem_region(res->start, resource_size(res));
free_mem:
	kfree(i2c_bus);

	return ret;
}

static int ast_i2c_remove(struct platform_device *pdev)
{
	struct ast_i2c_bus *i2c_bus = platform_get_drvdata(pdev);
	struct resource *res;

	platform_set_drvdata(pdev, NULL);
	i2c_del_adapter(&i2c_bus->adap);

	free_irq(i2c_bus->irq, i2c_bus);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(i2c_bus->reg_base);
	release_mem_region(res->start, res->end - res->start + 1);

	kfree(i2c_bus);

	return 0;
}

#ifdef CONFIG_PM
static int ast_i2c_suspend(struct platform_device *pdev, pm_message_t state)
{
	//TODO
//	struct ast_i2c_bus *i2c_bus = platform_get_drvdata(pdev);
	return 0;
}

static int ast_i2c_resume(struct platform_device *pdev)
{
	//TODO
//	struct ast_i2c_bus *i2c_bus = platform_get_drvdata(pdev);
	//Should reset i2c ???
	return 0;
}
#endif

static struct platform_driver ast_i2c_bus_driver = {
	.probe		= ast_i2c_probe,
	.remove		= ast_i2c_remove,
#ifdef CONFIG_PM
	.suspend	= ast_i2c_suspend,
	.resume		= ast_i2c_resume,
#endif
	.driver		= {
		.name	= KBUILD_MODNAME,
	},
};

module_platform_driver(ast_i2c_bus_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED AST G6 I2C Bus Driver");
MODULE_LICENSE("GPL");
