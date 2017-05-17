/* arch/arm/mach-aspeed/include/mach/regs-scu.h
 *
 * Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *   History      : 
 *    1. 2017/02/20 Ryan Chen Create
 * 
********************************************************************************/
#ifndef __AST_CAM_SCU_H
#define __AST_CAM_SCU_H                     1

/* Register for SCU  */
#define AST_SCU_PROTECT			0x00		/*	protection key register	*/
#define AST_SCU_RESET				0x04		/*	system reset control register */
#define AST_SCU_CLK_SEL				0x08		/*	clock selection register	*/
#define AST_SCU_CLK_SEL2			0xD8		/*	clock selection register Set 2*/
#define AST_SCU_CLK_STOP			0x0C		/*	clock stop control register	*/
#define AST_SCU_CLK_STOP2			0xDC		/*	clock stop control register set 2*/
#define AST_SCU_COUNT_CTRL			0x10		/*	frequency counter control register	*/
#define AST_SCU_COUNT_PRANG		0x14		/*	Frequency counter comparison range 	*/

#define AST_SCU_D2_PLL				0x1C		/*	D2-PLL Parameter  register	*/

#define AST_SCU_M_PLL				0x20		/*	M-PLL Parameter register		*/
#define AST_SCU_H_PLL				0x24		/*	H-PLL Parameter register		*/
#define AST_SCU_D_PLL				0x28		/*	D-PLL Parameter  register	*/
#define AST_SCU_MISC1_CTRL			0x2C		/*	Misc. Control register */
#define AST_SCU_PCI_CONF1			0x30		/*	PCI configuration setting register#1 */
#define AST_SCU_PCI_CONF2			0x34		/*	PCI configuration setting register#2 */
#define AST_SCU_PCI_CONF3			0x38		/*	PCI configuration setting register#3 */
#define AST_SCU_SYS_CTRL			0x3C		/*	System reset contrl/status register*/
#define AST_SCU_SOC_SCRATCH0		0x40		/*	SOC scratch 0~31 register */
#define AST_SCU_SOC_SCRATCH1		0x44		/*	SOC scratch 32~63 register */
#define AST_SCU_MAC_CLK			0x48		/*	MAC interface clock delay setting register */
#define AST_SCU_MISC2_CTRL			0x4C		/*	Misc. 2 Control register */
#define AST_SCU_VGA_SCRATCH0		0x50		/*	VGA Scratch register */
#define AST_SCU_VGA_SCRATCH1		0x54		/*	VGA Scratch register */
#define AST_SCU_VGA_SCRATCH2		0x58		/*	VGA Scratch register */
#define AST_SCU_VGA_SCRATCH3		0x5c		/*	VGA Scratch register */
#define AST_SCU_VGA_SCRATCH4		0x60		/*	VGA Scratch register */
#define AST_SCU_VGA_SCRATCH5		0x64		/*	VGA Scratch register */
#define AST_SCU_VGA_SCRATCH6		0x68		/*	VGA Scratch register */
#define AST_SCU_VGA_SCRATCH7		0x6c		/*	VGA Scratch register */
#define AST_SCU_HW_STRAP1			0x70		/*	hardware strapping register */
#define AST_SCU_RAMDOM_GEN		0x74		/*	random number generator register */
#define AST_SCU_RAMDOM_DATA		0x78		/*	random number generator data output*/
#define AST_SCU_REVISION_ID		0x7C		/*	Silicon revision ID register */
#define AST_SCU_FUN_PIN_CTRL1		0x80		/*	Multi-function Pin Control#1*/
#define AST_SCU_FUN_PIN_CTRL2		0x84		/*	Multi-function Pin Control#2*/
#define AST_SCU_FUN_PIN_CTRL3		0x88		/*	Multi-function Pin Control#3*/
#define AST_SCU_FUN_PIN_CTRL4		0x8C		/*	Multi-function Pin Control#4*/
#define AST_SCU_FUN_PIN_CTRL5		0x90		/*	Multi-function Pin Control#5*/
#define AST_SCU_FUN_PIN_CTRL6		0x94		/*	Multi-function Pin Control#6*/
#define AST_SCU_WDT_RESET			0x9C		/*	Watchdog Reset Selection */
#define AST_SCU_FUN_PIN_CTRL7		0xA0		/*	Multi-function Pin Control#7*/
#define AST_SCU_FUN_PIN_CTRL8		0xA4		/*	Multi-function Pin Control#8*/
#define AST_SCU_FUN_PIN_CTRL9		0xA8		/*	Multi-function Pin Control#9*/
#define AST_SCU_FUN_PIN_CTRL10		0xAC		/*	Multi-function Pin Control#10*/
#define AST_SCU_MAC_CLK_DELAY_100M	0xB8		/*	MAC interface clock delay 100M setting*/
#define AST_SCU_MAC_CLK_DELAY_10M		0xBC		/*	MAC interface clock delay 10M setting*/
#define AST_SCU_PWR_SAVING_EN		0xC0		/*	Power Saving Wakeup Enable*/
#define AST_SCU_PWR_SAVING_CTRL	0xC4		/*	Power Saving Wakeup Control*/
#define AST_SCU_HW_STRAP2			0xD0		/*	Haardware strapping register set 2*/
#define AST_SCU_COUNTER			0xE0		/*	SCU Free Run Counter Read Back */
#define AST_SCU_COUNTER_EXT		0xE4		/*	SCU Free Run Counter Extended Read Back */
#define AST_SCU_CLK_DUTY			0xE8		/*	Clock Duty Measurement Control */
#define AST_SCU_CLK_DUTY_RESULT	0xEC		/*	Clock Duty Measurement Result */
#define AST_SCU_SECURITY_IP_CTRL	0xF0		/*	Security IP Control  */


#define AST_SCU_MAC_CLK_DUTY		0x1DC		/*	Clock Duty Selection */

#define AST_SCU_CPU2_BASE0_ADDR	0x200		/*	CPU2 Base Address for Segment 0x00:0000~0x0F:FFFF*/
#define AST_SCU_CPU2_BASE1_ADDR	0x204		/*	CPU2 Base Address for Segment 0x10:0000~0x1F:FFFF*/
#define AST_SCU_CPU2_BASE2_ADDR	0x208		/*	CPU2 Base Address for Segment 0x20:0000~0x2F:FFFF*/
#define AST_SCU_CPU2_BASE3_ADDR	0x20C		/*	CPU2 Base Address for Segment 0x30:0000~0x3F:FFFF*/
#define AST_SCU_CPU2_BASE4_ADDR	0x210		/*	CPU2 Base Address for Segment 0x40:0000~0x4F:FFFF*/
#define AST_SCU_CPU2_BASE5_ADDR	0x214		/*	CPU2 Base Address for Segment 0x50:0000~0x5F:FFFF*/
#define AST_SCU_CPU2_BASE6_ADDR	0x218		/*	CPU2 Base Address for Segment 0x60:0000~0x6F:FFFF*/
#define AST_SCU_CPU2_BASE7_ADDR	0x21C		/*	CPU2 Base Address for Segment 0x70:0000~0x7F:FFFF*/
#define AST_SCU_CPU2_BASE8_ADDR	0x220		/*	CPU2 Base Address for Segment 0x80:0000~0x8F:FFFF*/
#define AST_SCU_CPU2_CACHE_CTRL	0x224		/*	CPU2 Cache Function Control */

#define AST_SCU_CPU3_BASE0_ADDR	0x230		/*	CPU3 Base Address for Segment 0x00:0000~0x0F:FFFF*/
#define AST_SCU_CPU3_BASE1_ADDR	0x234		/*	CPU3 Base Address for Segment 0x10:0000~0x1F:FFFF*/
#define AST_SCU_CPU3_BASE2_ADDR	0x238		/*	CPU3 Base Address for Segment 0x20:0000~0x2F:FFFF*/
#define AST_SCU_CPU3_BASE3_ADDR	0x23C		/*	CPU3 Base Address for Segment 0x30:0000~0x3F:FFFF*/
#define AST_SCU_CPU3_BASE4_ADDR	0x240		/*	CPU3 Base Address for Segment 0x40:0000~0x4F:FFFF*/
#define AST_SCU_CPU3_BASE5_ADDR	0x244		/*	CPU3 Base Address for Segment 0x50:0000~0x5F:FFFF*/
#define AST_SCU_CPU3_BASE6_ADDR	0x248		/*	CPU3 Base Address for Segment 0x60:0000~0x6F:FFFF*/
#define AST_SCU_CPU3_BASE7_ADDR	0x24C		/*	CPU3 Base Address for Segment 0x70:0000~0x7F:FFFF*/
#define AST_SCU_CPU3_BASE8_ADDR	0x250		/*	CPU3 Base Address for Segment 0x80:0000~0x8F:FFFF*/
#define AST_SCU_CPU3_CACHE_CTRL	0x254		/*	CPU3 Cache Function Control */

#define AST_SCU_ZSP_BASE0_ADDR	0x260		/*	CPU3 Base Address for Segment 0x00:0000~0x0F:FFFF*/
#define AST_SCU_ZSP_BASE1_ADDR	0x264		/*	CPU3 Base Address for Segment 0x10:0000~0x1F:FFFF*/
#define AST_SCU_ZSP_BASE2_ADDR	0x268		/*	CPU3 Base Address for Segment 0x20:0000~0x2F:FFFF*/
#define AST_SCU_ZSP_BASE3_ADDR	0x26C		/*	CPU3 Base Address for Segment 0x30:0000~0x3F:FFFF*/
#define AST_SCU_ZSP_BASE4_ADDR	0x270		/*	CPU3 Base Address for Segment 0x40:0000~0x4F:FFFF*/
#define AST_SCU_ZSP_BASE5_ADDR	0x274		/*	CPU3 Base Address for Segment 0x50:0000~0x5F:FFFF*/
#define AST_SCU_ZSP_BASE6_ADDR	0x278		/*	CPU3 Base Address for Segment 0x60:0000~0x6F:FFFF*/
#define AST_SCU_ZSP_BASE7_ADDR	0x27C		/*	CPU3 Base Address for Segment 0x70:0000~0x7F:FFFF*/
#define AST_SCU_ZSP_BASE8_ADDR	0x280		/*	CPU3 Base Address for Segment 0x80:0000~0xFF:FFFF*/
#define AST_SCU_ZSP_CACHE_CTRL	0x284		/*	CPU3 Cache Function Control */



/*	AST_SCU_PROTECT: 0x00  - protection key register */
#define SCU_PROTECT_UNLOCK			0x1688A8A8

/*	AST_SCU_RESET :0x04	 - system reset control register */
#define SCU_RESET_SC				(0x1 << 18)
#define SCU_RESET_OSD				(0x1 << 17) 
#define SCU_RESET_EE				(0x1 << 16)
#define SCU_RESET_WDR				(0x1 << 15)
#define SCU_RESET_HE				(0x1 << 14)
#define SCU_RESET_PP				(0x1 << 13)
#define SCU_RESET_UDC0				(0x1 << 12)
#define SCU_RESET_MAC0				(0x1 << 11)
#define SCU_RESET_SDHCI				(0x1 << 10)
#define SCU_RESET_SDIO				(0x1 << 9)
#define SCU_RESET_I2S				(0x1 << 8)
#define SCU_RESET_H264				(0x1 << 7)
#define SCU_RESET_JPEG				(0x1 << 6)
#define SCU_RESET_3D				(0x1 << 5)
#define SCU_RESET_HAC				(0x1 << 4)
#define SCU_RESET_ISP				(0x1 << 3)
#define SCU_RESET_I2C				(0x1 << 2)
#define SCU_RESET_AHB				(0x1 << 1)
#define SCU_RESET_SDRAM_CTRL		(0x1 << 0)

/*	AST_SCU_CLK_SEL	 : 0x08 - clock selection register	*/
#define SCU_SDHCI_CLK_SOURCE(x)	((x) << 22)
#define SCU_SDIO_CLK_SOURCE(x)		((x) << 20)
#define SCU_H264_HEVCLK_SOURCE	((x) << 18)		
#define SCU_JPEG_JCLK_SOURCE(x)	((x) << 16)
#define SCU_PP_VCLK_SOURCE(x)		((x) << 14)
#define SCU_ZSP_CLK_SOURCE(x)		((x) << 12)
#define SCU_3D_ECLK_SOURCE(x)		((x) << 10)		

#define SCU_APB_PCLK_DIV(x)			((x) << 6)
#define SCU_GET_PCLK_DIV(x)			((x >> 6) & 0x7)
#define SCU_APB_PCLK_DIV_MASK		(0x7 << 6)		//limitation on PCLK .. PCLK > 0.5*LCLK (33Mhz)
#define SCU_CLK_CPU_AHB_SLOW_EN	(0x1 << 5)

#define SCU_CPU_DYNAMIC_SLOW_DIV(x)	((x) << 2)

#define SCU_CLK_CPU_AHB_SLOW_IDLE	(0x1 << 1)	/*0:128 HCLK idle, 1:256 HCLK idle */
#define SCU_CLK_CPU_AHB_DYN_SLOW_EN	(0x1)

/*	AST_SCU_CLK_SEL2	: 0xD8 - clock selection register Set 2	*/
#define SCU_MAC_AHB_CLK_DIV(x)	(x << 24)
#define SCU_SDHCI_CLK_DIV(x)		(x << 20)
#define SCU_SDIO_CLK_DIV(x)			(x << 16)
#define SCU_ZSP_CLK_DIV(x)			(x << 12)
#define SCU_H264_HEVCLK_DIV(x)		(x << 9)
#define SCU_JPEG_JCLK_DIV(x)		(x << 6)
#define SCU_PP_VCLK_DIV(x)			(x << 3)
#define SCU_3D_ECLK(x)				(x)

/*	AST_SCU_CLK_STOP : 0x0C - clock stop control register	*/
#define SCU_SC_CLK_STOP_EN			(0x1 << 27)
#define SCU_EE_CLK_STOP_EN			(0x1 << 26)
#define SCU_OSD_CLK_STOP_EN		(0x1 << 25)
#define SCU_WDR_CLK_STOP_EN		(0x1 << 24)
#define SCU_HE_CLK_STOP_EN			(0x1 << 23)
#define SCU_I2S_CLK_STOP_EN		(0x1 << 22)
#define SCU_SDHCI_CLK_STOP_EN		(0x1 << 21)
#define SCU_SDIO_CLK_STOP_EN		(0x1 << 20)
#define SCU_HEV_CLK_STOP_EN		(0x1 << 19)
#define SCU_MAC_CLK_STOP_EN		(0x1 << 18)
#define SCU_ISP_CLK_STOP_EN		(0x1 << 17)
#define SCU_YCLK_STOP_EN			(0x1 << 16)
#define SCU_UART3_CLK_STOP_EN		(0x1 << 15)
#define SCU_UART2_CLK_STOP_EN		(0x1 << 14)
#define SCU_UART1_CLK_STOP_EN		(0x1 << 13)
#define SCU_UDC20_CLK_STOP_EN		(0x1 << 12)
#define SCU_ISP6_CLK_STOP_EN		(0x1 << 11)
#define SCU_ISP5_CLK_STOP_EN		(0x1 << 10)
#define SCU_ISP4_CLK_STOP_EN		(0x1 << 9)
#define SCU_ISP3_CLK_STOP_EN		(0x1 << 8)
#define SCU_ISP2_CLK_STOP_EN		(0x1 << 7)
#define SCU_ISP1_CLK_STOP_EN		(0x1 << 6)
#define SCU_REF_CLK_STOP_EN		(0x1 << 5)
#define SCU_SACLK_STOP_EN			(0x1 << 4)
#define SCU_JCLK_STOP_EN			(0x1 << 3)
#define SCU_PPCLK_STOP_EN			(0x1 << 3)
#define SCU_ECLK_STOP_EN			(0x1 << 2)
#define SCU_MCLK_STOP_EN			(0x1 << 1)
#define SCU_ARM_CPU_STOP_EN		(0x1 << 0)

/* AST_SCU_COUNT_CTRL : 0x10 - frequency counter control register	*/
#define SCU_FREQ_MEASURE_COUNT(x)		((x) << 16) 
#define SCU_DELAY_RING_STAGE_CTRL(x)	((x) << 9) 
#define SCU_OSC_COUNT_RESULT_PIN_EN	(0x1 << 8) 
#define SCU_FREQ_COMP_RESULT			(0x1 << 7) 
#define SCU_FREQ_MEASU_FINISH			(0x1 << 6)
#define SCU_FREQ_SOURCE_FOR_MEASU(x)	(x << 2)
#define SCU_FREQ_SOURCE_FOR_MEASU_MASK	(0xf << 2)
#define SCU_FREQ_SOURCE_FOR_MEASU_12MHZ 0xC //0b1100

#define SCU_SOURCE_PCLK				0xf
#define SCU_SOURCE_VPACLK				0xe
#define SCU_SOURCE_VPBCLK				0xd
#define SCU_SOURCE_12M					0xc
#define SCU_SOURCE_LCLK				0xb
#define SCU_SOURCE_GRCLK				0xa
#define SCU_SOURCE_HCLK				0x9
#define SCU_SOURCE_MCLK				0x8
#define SCU_SOURCE_BCLK				0x7
#define SCU_SOURCE_XPCLK				0x6
#define SCU_SOURCE_D2_CLK				0x5
#define SCU_SOURCE_D_CLK				0x4
#define SCU_SOURCE_DLY32				0x3
#define SCU_SOURCE_DLY16				0x2
#define SCU_SOURCE_NAND				0x1
#define SCU_SOURCE_DEL_CELL			0x0

#define SCU_OSC_COUNT_EN				(0x1 << 1)
#define SCU_RING_OSC_EN				(0x1 << 0)


/*  AST_SCU_INTR_CTRL : 0x18 - Interrupt control and status register   */
#define INTR_LPC_H_L_RESET				(0x1 << 21)
#define INTR_LPC_L_H_RESET				(0x1 << 20)
#define INTR_PCIE_H_L_RESET				(0x1 << 19)
#define INTR_PCIE_L_H_RESET				(0x1 << 18)
#define INTR_VGA_SCRATCH_CHANGE			(0x1 << 17)
#define INTR_VGA_CURSOR_CHANGE			(0x1 << 16)
#define INTR_ISSUE_MSI					(0x1 << 6)
#define INTR_LPC_H_L_RESET_EN			(0x1 << 5)
#define INTR_LPC_L_H_RESET_EN			(0x1 << 4)
#define INTR_PCIE_H_L_RESET_EN			(0x1 << 3)
#define INTR_PCIE_L_H_RESET_EN			(0x1 << 2)
#define INTR_VGA_SCRATCH_CHANGE_EN		(0x1 << 1)
#define INTR_VGA_CURSOR_CHANGE_EN		(0x1 << 0)

/*	AST_SCU_D2_PLL: 0x1C - D2-PLL Parameter  register */
#define SCU_D2_PLL_SET_ODNUM(x)		(x << 19)
#define SCU_D2_PLL_GET_ODNUM(x)		((x >> 19) & 0x3)
#define SCU_D2_PLL_OD_MASK				(0x3 << 19)
#define SCU_D2_PLL_SET_PNUM(x)			(x << 13)
#define SCU_D2_PLL_GET_PNUM(x)			((x >>13)&0x3f)
#define SCU_D2_PLL_PNUM_MASK			(0x3f << 13)
#define SCU_D2_PLL_SET_NNUM(x)			(x << 8)
#define SCU_D2_PLL_GET_NNUM(x)			((x >>8)&0x1f)
#define SCU_D2_PLL_NNUM_MASK			(0x1f << 8)
#define SCU_D2_PLL_SET_MNUM(x)			(x)
#define SCU_D2_PLL_GET_MNUM(x)			(x & 0xff)
#define SCU_D2_PLL_MNUM_MASK			(0xff)

/*	AST_SCU_D2_PLL_EXTEND: 0x13C - D2-PLL Extender Parameter  register */
#define SCU_D2_PLL_PARAMETER0(x)		((x) << 5)
#define SCU_D2_PLL_SET_MODE(x)			((x) << 3)
#define SCU_D2_PLL_GET_MODE(x)			(((x) >> 3) & 0x3)
#define SCU_D2_PLL_RESET				(0x1 << 2)
#define SCU_D2_PLL_BYPASS				(0x1 << 1)
#define SCU_D2_PLL_OFF					(0x1)


/*	AST_SCU_M_PLL : 0x20	- M-PLL Parameter register	*/
#define SCU_M_PLL_RESET					(0x1 << 21)
#define SCU_M_PLL_BYPASS				(0x1 << 20)
#define SCU_M_PLL_OFF					(0x1 << 19)
#define SCU_M_PLL_GET_PDNUM(x)			((x >> 13) & 0x3f)
#define SCU_M_PLL_GET_MNUM(x)			((x >> 5) & 0xff)
#define SCU_M_PLL_GET_NNUM(x)			(x & 0x1f)

/*	AST_SCU_H_PLL: 0x24- H-PLL Parameter register		*/
#define SCU_H_PLL_PARAMETER0(x)			((x) << 22)
#define SCU_H_PLL_GET_PARAMETER0(x)		((x >> 22) & 0x3ff)
#define SCU_H_PLL_PARAMETER0_MASK(x)	(0x3ff << 22)
#define SCU_H_PLL_GET_PARAMETER0(x)		((x >> 22) & 0x3ff)
#define SCU_H_PLL_RESET					(0x1 << 21)
#define SCU_H_PLL_BYPASS_EN			(0x1 << 20)
#define SCU_H_PLL_OFF					(0x1 << 19)
#define SCU_H_PLL_PNUM(x)				(x << 13)
#define SCU_H_PLL_GET_PNUM(x)			((x >> 13) & 0x3f)
#define SCU_H_PLL_PNUM_MASK			(0x3f << 13)
#define SCU_H_PLL_MNUM(x)				(x << 5)
#define SCU_H_PLL_GET_MNUM(x)			((x >> 5) & 0xff)
#define SCU_H_PLL_MNUM_MASK			(0xff << 5)
#define SCU_H_PLL_NNUM(x)				(x)
#define SCU_H_PLL_GET_NNUM(x)			(x & 0xf)
#define SCU_H_PLL_NNUM_MASK			(0xf)


/* 	AST_SCU_MH_PLL_EXTEND : 0x148 - Extended Parameter of M/H-PLL register */
#define SCU_H_PLL_GET_PARAMETER1(x)		((x >> 16) & 0x3f)
#define SCU_H_PLL_PARAMETER1_MASK(x)	(0x3f << 16)
#define SCU_M_PLL_GET_PARAMETER1(x)		(x & 0x3f)
#define SCU_M_PLL_PARAMETER1_MASK(x)	(0x3f)

/*	AST_SCU_D_PLL : 0x28 - D-PLL Parameter  register	*/
#define SCU_D_PLL_GET_SIP(x)				((x >>27) & 0x1f)
#define SCU_D_PLL_GET_SIC(x)				((x >>22) & 0x1f)
#define SCU_D_PLL_GET_ODNUM(x)			((x >>19) & 0x7)
#define SCU_D_PLL_GET_PNUM(x)			((x >>13) & 0x3f)
#define SCU_D_PLL_GET_NNUM(x)			((x >>8) & 0x1f)
#define SCU_D_PLL_GET_MNUM(x)			(x & 0xff)

/*	AST_SCU_D_PLL_EXTEND : 0x130 - D-PLL Extended Parameter  register	*/
#define SCU_D_PLL_SET_MODE(x)			((x & 0x3) << 3)
#define SCU_D_PLL_RESET					(0x1 << 2)
#define SCU_D_PLL_BYPASS				(0x1 << 1)
#define SCU_D_PLL_OFF					(0x1)

/*	AST_SCU_MISC1_CTRL : 0x2C - Misc. Control register */
#define SCU_MISC_JTAG_MASTER_DIS	(0x1 << 26)
#define SCU_MISC_DRAM_W_P2A_DIS		(0x1 << 25)
#define SCU_MISC_SPI_W_P2A_DIS		(0x1 << 24)
#define SCU_MISC_SOC_W_P2A_DIS		(0x1 << 23)
#define SCU_MISC_FLASH_W_P2A_DIS	(0x1 << 22)
#define SCU_MISC_CRT_CLK_H_SOURCE		(0x1 << 21)
#define SCU_MISC_D_PLL_SOURCE			(0x1 << 20)
#define SCU_MISC_VGA_CONFIG_PREFETCH	(0x1 << 19)
#define SCU_MISC_DVO_SOURCE_CRT			(0x1 << 18) 	//0:VGA , 1:CRT
#define SCU_MISC_DAC_MASK				(0x3 << 16)
#define SCU_MISC_SET_DAC_SOURCE(x)		(x << 16)		
#define SCU_MISC_DAC_SOURCE_CRT			(0x1 << 16)		//00 VGA, 01: CRT, 1x: PASS-Through DVO
#define SCU_MISC_DAC_SOURCE_MASK		(0x3 << 16)		
#define SCU_MISC_JTAG_TO_PCIE_EN		(0x1 << 15)
#define SCU_MISC_JTAG__M_TO_PCIE_EN		(0x1 << 14)
#define SCU_MISC_VUART_TO_CTRL			(0x1 << 13)
#define SCU_MISC_DIV13_EN				(0x1 << 12)
#define SCU_MISC_Y_CLK_INVERT			(0x1 << 11)
#define SCU_MISC_OUT_DELAY				(0x1 << 9)
#define SCU_MISC_PCI_TO_AHB_DIS			(0x1 << 8)
#define SCU_MISC_2D_CRT_EN				(0x1 << 7)
#define SCU_MISC_VGA_CRT_DIS			(0x1 << 6)
#define SCU_MISC_VGA_REG_ACCESS_EN		(0x1 << 5)
#define SCU_MISC_D2_PLL_DIS				(0x1 << 4)
#define SCU_MISC_DAC_DIS				(0x1 << 3)
#define SCU_MISC_D_PLL_DIS				(0x1 << 2)
#define SCU_MISC_OSC_CLK_OUT_PIN		(0x1 << 1)
#define SCU_MISC_LPC_TO_SPI_DIS			(0x1 << 0)

/*	AST_SCU_PCI_CONF1 : 0x30 - PCI configuration setting register#1 */
#define SCU_PCI_DEVICE_ID(x)			(x << 16)
#define SCU_PCI_VENDOR_ID(x)			(x)

/*	AST_SCU_PCI_CONF2	0x34		PCI configuration setting register#2 */
#define SCU_PCI_SUB_SYS_ID(x)			(x << 16)
#define SCU_PCI_SUB_VENDOR_ID(x)		(x)

/*	AST_SCU_PCI_CONF3	0x38		PCI configuration setting register#3 */
#define SCU_PCI_CLASS_CODE(x)			(x << 8)
#define SCU_PCI_REVISION_ID(x)			(x)

/*	AST_SCU_SYS_CTRL	0x3C		System reset contrl/status register*/
#define SCU_SYS_WDT3_RESET_FLAG			(0x1 << 4)
#define SCU_SYS_WDT2_RESET_FLAG			(0x1 << 3)
#define SCU_SYS_WDT1_RESET_FLAG			(0x1 << 2)
#define SCU_SYS_EXT_RESET_FLAG			(0x1 << 1)
#define SCU_SYS_PWR_RESET_FLAG			(0x1 << 0)

/*	AST_SCU_SOC_SCRATCH0	0x40		SOC scratch 0~31 register */




/*	AST_SCU_SOC_SCRATCH1	0x44		SOC scratch 32~63 register */


/*	AST_SCU_VGA0		0x40		VGA fuction handshake register */
#define SCU_VGA_SLT_HANDSHAKE(x)		(x << 24)
#define SCU_VGA_SLT_HANDSHAKE_MASK		(0xff << 24)
#define SCU_VGA_CTM_DEF(x)				(x << 16)
#define SCU_VGA_CTM_DEF_MASK			(0xff << 16)
#define SCU_MAC0_PHY_MODE(x)			(x << 14)
#define SCU_MAC0_GET_PHY_MODE(x)		((x >> 14) & 0x3)
#define SCU_MAC0_PHY_MODE_MASK(x)		(0x3 << 14)
#define SCU_MAC1_PHY_MODE(x)			(x << 12)
#define SCU_MAC1_PHY_MODE_MASK			(0x3 << 12)
#define SCU_MAC1_GET_PHY_MODE(x)		((x >> 12) & 0x3)

#define SCU_VGA_ASPEED_DEF(x)			(x << 8)
#define SCU_VGA_ASPEED_DEF_MASK			(0xf << 8)

#define SCU_VGA_DRAM_INIT_MASK(x)		((x >> 7) & 0x1)

/*	AST_SCU_VGA1		0x44		VGA fuction handshake register */


/*	AST_SCU_MAC_CLK		0x48		MAC interface clock delay setting register */



/*	AST_SCU_MISC2_CTRL		0x4C		Misc. 2 Control register */
#define SCU_PCIE_MAPPING_HIGH			(1 << 15)
#define SCU_MALI_DTY_MODE				(1 << 8)
#define SCU_MALI_RC_MODE				(1 << 7)

/*	AST_SCU_VGA_SCRATCH0	0x50		VGA Scratch register */
/*	AST_SCU_VGA_SCRATCH1	0x54		VGA Scratch register */
/*	AST_SCU_VGA_SCRATCH2	0x58		VGA Scratch register */
/*	AST_SCU_VGA_SCRATCH3	0x5c		VGA Scratch register */
/*	AST_SCU_VGA_SCRATCH4	0x60		VGA Scratch register */
/*	AST_SCU_VGA_SCRATCH5	0x64		VGA Scratch register */
/*	AST_SCU_VGA_SCRATCH6	0x68		VGA Scratch register */
/*	AST_SCU_VGA_SCRATCH7	0x6c		VGA Scratch register */

/*	AST_SCU_HW_STRAP1			0x70		hardware strapping register */
#define SCU_HW_STRAP_ESPI_MODE			(0x1 << 25)
#define CLK_25M_IN							(0x1 << 23)

#define SCU_HW_STRAP_2ND_BOOT_WDT		(0x1 << 17)
#define SCU_HW_STRAP_SUPER_IO_CONFIG	(0x1 << 16)
#define SCU_HW_STRAP_VGA_CLASS_CODE		(0x1 << 15)
#define SCU_HW_STRAP_LPC_RESET_PIN		(0x1 << 14)
#define SCU_HW_STRAP_SPI_MODE(x)			(x << 12)
#define SCU_HW_STRAP_SPI_MODE_MASK			(0x3 << 12)
#define SCU_HW_STRAP_SPI_MASTER			(0x1 << 12)
#define SCU_HW_STRAP_SPI_M_S_EN			(0x2 << 12)
#define SCU_HW_STRAP_SPI_PASS_THROUGH	(0x3 << 12)
#define SCU_HW_STRAP_GET_AXI_AHB_RATIO(x)	((x >> 9) & 0x7)
#define SCU_HW_STRAP_GET_CPU_AXI_RATIO(x)	((x >> 8) & 0x1)
#define SCU_HW_STRAP_MAC1_RGMII		(0x1 << 7)
#define SCU_HW_STRAP_MAC0_RGMII		(0x1 << 6)
#define SCU_HW_STRAP_VGA_BIOS_ROM	(0x1 << 5)
#define SCU_HW_STRAP_SPI_WIDTH		(0x1 << 4)

#define SCU_HW_STRAP_VGA_SIZE_GET(x)	((x >> 2)& 0x3)
#define SCU_HW_STRAP_VGA_MASK			(0x3 << 3)
#define SCU_HW_STRAP_VGA_SIZE_SET(x)	(x << 2)

#define VGA_8M_DRAM				0
#define VGA_16M_DRAM			1
#define VGA_32M_DRAM			2
#define VGA_64M_DRAM			3

#define SCU_HW_STRAP_DIS_BOOT 			(1)

/*	AST_SCU_RAMDOM_GEN		0x74		random number generator register */
#define RNG_TYPE_MASK			(0x7 << 1) 
#define RNG_SET_TYPE(x)			((x) << 1) 
#define RNG_GET_TYPE(x)			(((x) >> 1)  & 0x7)
#define RNG_ENABLE				0x1
/*	AST_SCU_RAMDOM_DATA		0x78		random number generator data output*/

/*	AST_SCU_MULTI_FUNC_2	0x78 */

#define MULTI_FUNC_VIDEO_RGB18			(0x1 << 2)
#define MULTI_FUNC_VIDEO_SINGLE_EDGE	(0x1 << 0)



/*	AST_SCU_REVISION_ID		0x7C		Silicon revision ID register */
#define AST_SOC_GEN				24
#define AST1100_A0				0x00000200
#define AST1100_A1				0x00000201
#define AST1100_A2				0x00000202
#define AST1100_A3				0x00000202

#define AST2050_A0				0x00000200
#define AST2050_A1				0x00000201
#define AST2050_A2				0x00000202
#define AST2050_A3				0x00000202

#define AST2100_A0				0x00000300
#define AST2100_A1				0x00000301
#define AST2100_A2				0x00000302
#define AST2100_A3				0x00000302

#define AST2200_A0				0x00000102
#define AST2200_A1				0x00000102

#define AST2300_A0				0x01000003
#define AST2300_A1				0x01010303
#define AST1300_A1				0x01010003
#define AST1050_A1				0x01010203

#define AST2400_A0				0x02000303

#define GET_CHIP_REVISION(x)			((x & 0xff000000) >> 24)

#define GET_HW_REVISION_ID(x)	((x & 0xff0000) >> 16)

#define AST_DRAM_BASE_4		0x40000000
#define AST_DRAM_BASE_8		0x80000000

/*	AST_SCU_FUN_PIN_CTRL1		0x80		Multi-function Pin Control#1*/
#define SCU_FUN_PIN_UART4_RXD	(0x1 << 31)
#define SCU_FUN_PIN_UART4_TXD	(0x1 << 30)
#define SCU_FUN_PIN_UART4_NRTS	(0x1 << 29)
#define SCU_FUN_PIN_UART4_NDTR	(0x1 << 28)
#define SCU_FUN_PIN_UART4_NRI	(0x1 << 27)
#define SCU_FUN_PIN_UART4_NDSR	(0x1 << 26)
#define SCU_FUN_PIN_UART4_NDCD	(0x1 << 25)
#define SCU_FUN_PIN_UART4_NCTS	(0x1 << 24)
#define SCU_FUN_PIN_UART3_RXD	(0x1 << 23)
#define SCU_FUN_PIN_UART3_TXD	(0x1 << 22)
#define SCU_FUN_PIN_UART3_NRTS	(0x1 << 21)
#define SCU_FUN_PIN_UART3_NDTR	(0x1 << 20)
#define SCU_FUN_PIN_UART3_NRI	(0x1 << 19)
#define SCU_FUN_PIN_UART3_NDSR	(0x1 << 18)
#define SCU_FUN_PIN_UART3_NDCD	(0x1 << 17)
#define SCU_FUN_PIN_UART3_NCTS	(0x1 << 16)




#define SCU_FUN_PIN_MAC1_PHY_LINK	(0x1 << 1)
#define SCU_FUN_PIN_MAC0_PHY_LINK	(0x1)


/*	AST_SCU_FUN_PIN_CTRL2		0x84		Multi-function Pin Control#2*/
#define SCU_FUN_PIN_VPIB9		(0x1 << 31)
#define SCU_FUN_PIN_VPIB8		(0x1 << 30)
#define SCU_FUN_PIN_VPIB7		(0x1 << 29)
#define SCU_FUN_PIN_VPIB6		(0x1 << 28)
#define SCU_FUN_PIN_VPIB5		(0x1 << 27)
#define SCU_FUN_PIN_VPIB4		(0x1 << 26)
#define SCU_FUN_PIN_VPIB3		(0x1 << 25)
#define SCU_FUN_PIN_VPIB2		(0x1 << 24)
#define SCU_FUN_PIN_VPIB1		(0x1 << 23)
#define SCU_FUN_PIN_VPIB0		(0x1 << 22)
#define SCU_FUN_PIN_VPICLK		(0x1 << 21)
#define SCU_FUN_PIN_VPIVS		(0x1 << 20)
#define SCU_FUN_PIN_VPIHS		(0x1 << 19)
#define SCU_FUN_PIN_VPIODD		(0x1 << 18)
#define SCU_FUN_PIN_VPIDE		(0x1 << 17)

#define SCU_FUN_PIN_DDCDAT		(0x1 << 15)
#define SCU_FUN_PIN_DDCCLK		(0x1 << 14)
#define SCU_FUN_PIN_VGAVS		(0x1 << 13)
#define SCU_FUN_PIN_VGAHS		(0x1 << 12)

#define SCU_FUN_PIN_UART2_RXD	(0x1 << 31)
#define SCU_FUN_PIN_UART2_TXD	(0x1 << 30)
#define SCU_FUN_PIN_UART2_NRTS	(0x1 << 29)
#define SCU_FUN_PIN_UART2_NDTR	(0x1 << 28)
#define SCU_FUN_PIN_UART2_NRI	(0x1 << 27)
#define SCU_FUN_PIN_UART2_NDSR	(0x1 << 26)
#define SCU_FUN_PIN_UART2_NDCD	(0x1 << 25)
#define SCU_FUN_PIN_UART2_NCTS	(0x1 << 24)
#define SCU_FUN_PIN_UART1_RXD	(0x1 << 23)
#define SCU_FUN_PIN_UART1_TXD	(0x1 << 22)
#define SCU_FUN_PIN_UART1_NRTS	(0x1 << 21)
#define SCU_FUN_PIN_UART1_NDTR	(0x1 << 20)
#define SCU_FUN_PIN_UART1_NRI	(0x1 << 19)
#define SCU_FUN_PIN_UART1_NDSR	(0x1 << 18)
#define SCU_FUN_PIN_UART1_NDCD	(0x1 << 17)
#define SCU_FUN_PIN_UART1_NCTS	(0x1 << 16)

#define SCU_FUN_PIN_SGPMI		(0x1 << 11)
#define SCU_FUN_PIN_SGPMO		(0x1 << 10)
#define SCU_FUN_PIN_SGPMLD		(0x1 << 9)
#define SCU_FUN_PIN_SGPMCK		(0x1 << 8)

#define SCU_FUN_PIN_I2C4_SALT4	(0x1 << 7)
#define SCU_FUN_PIN_I2C3_SALT3	(0x1 << 6)
#define SCU_FUN_PIN_I2C2_SALT2	(0x1 << 5)
#define SCU_FUN_PIN_I2C1_SALT1	(0x1 << 4)

/*	AST_SCU_FUN_PIN_CTRL3		0x88		Multi-function Pin Control#3*/
#define SCU_FUN_PIN_MAC0_MDIO	(0x1 << 31)
#define SCU_FUN_PIN_MAC0_MDC	(0x1 << 30)
#define SCU_FUN_PIN_ROMA25		(0x1 << 29)
#define SCU_FUN_PIN_ROMA24		(0x1 << 28)
#define SCU_FUN_PIN_ROMCS4		(0x1 << 27)
#define SCU_FUN_PIN_ROMCS3		(0x1 << 26)
#define SCU_FUN_PIN_ROMCS2		(0x1 << 25)
#define SCU_FUN_PIN_ROMCS1		(0x1 << 24)
#define SCU_FUN_PIN_ROMCS(x)	(0x1 << (23+x))

#define SCU_FUN_PIN_USBP4_DN	(0x1 << 23)
#define SCU_FUN_PIN_USBP4_DP	(0x1 << 22)
#define SCU_FUN_PIN_USBP3_DN	(0x1 << 21)
#define SCU_FUN_PIN_USBP3_DP	(0x1 << 20)
//Video pin
#define SCU_FUN_PIN_VPIR9		(0x1 << 19)
#define SCU_FUN_PIN_VPIR8		(0x1 << 18)
#define SCU_FUN_PIN_VPIR7		(0x1 << 17)
#define SCU_FUN_PIN_VPIR6		(0x1 << 16)
#define SCU_FUN_PIN_VPIR5		(0x1 << 15)
#define SCU_FUN_PIN_VPIR4		(0x1 << 14)
#define SCU_FUN_PIN_VPIR3		(0x1 << 13)
#define SCU_FUN_PIN_VPIR2		(0x1 << 12)
#define SCU_FUN_PIN_VPIR1		(0x1 << 11)
#define SCU_FUN_PIN_VPIR0		(0x1 << 10)
#define SCU_FUN_PIN_VPIG9		(0x1 << 9)
#define SCU_FUN_PIN_VPIG8		(0x1 << 8)
#define SCU_FUN_PIN_VPIG7		(0x1 << 7)
#define SCU_FUN_PIN_VPIG6		(0x1 << 6)
#define SCU_FUN_PIN_VPIG5		(0x1 << 5)
#define SCU_FUN_PIN_VPIG4		(0x1 << 4)
#define SCU_FUN_PIN_VPIG3		(0x1 << 3)
#define SCU_FUN_PIN_VPIG2		(0x1 << 2)
#define SCU_FUN_PIN_VPIG1		(0x1 << 1)
#define SCU_FUN_PIN_VPIG0		(0x1 << 0)

//pwm pin
#define SCU_FUN_PIN_PWM_TACHO	(0)
/*	AST_SCU_FUN_PIN_CTRL4		0x8C		Multi-function Pin Control#4*/
#define SCU_FUN_PIN_ROMA23		(0x1 << 7)
#define SCU_FUN_PIN_ROMA22		(0x1 << 6)

#define SCU_FUN_PIN_ROMWE		(0x1 << 5)
#define SCU_FUN_PIN_ROMOE		(0x1 << 4)
#define SCU_FUN_PIN_ROMD7		(0x1 << 3)
#define SCU_FUN_PIN_ROMD6		(0x1 << 2)
#define SCU_FUN_PIN_ROMD5		(0x1 << 1)
#define SCU_FUN_PIN_ROMD4		(0x1)

/*	AST_SCU_FUN_PIN_CTRL5		0x90		Multi-function Pin Control#5*/
#define SCU_FUN_PIN_SPICS1		(0x1 << 31)
#define SCU_FUN_PIN_LPC_PLUS	(0x1 << 30)
#define SCU_FUC_PIN_USB20_HOST	(0x1 << 29)
#define SCU_FUC_PIN_USB11_PORT4	(0x1 << 28)
#define SCU_FUC_PIN_I2C14		(0x1 << 27)
#define SCU_FUC_PIN_I2C13		(0x1 << 26)
#define SCU_FUC_PIN_I2C12		(0x1 << 25)
#define SCU_FUC_PIN_I2C11		(0x1 << 24)
#define SCU_FUC_PIN_I2C10		(0x1 << 23)
#define SCU_FUC_PIN_I2C9		(0x1 << 22)
#define SCU_FUC_PIN_I2C8		(0x1 << 21)
#define SCU_FUC_PIN_I2C7		(0x1 << 20)
#define SCU_FUC_PIN_I2C6		(0x1 << 19)
#define SCU_FUC_PIN_I2C5		(0x1 << 18)
#define SCU_FUC_PIN_I2C4		(0x1 << 17)
#define SCU_FUC_PIN_I2C3		(0x1 << 16)
#define SCU_FUC_PIN_MII2_RX_DWN_DIS		(0x1 << 15)
#define SCU_FUC_PIN_MII2_TX_DWN_DIS		(0x1 << 14)
#define SCU_FUC_PIN_MII1_RX_DWN_DIS		(0x1 << 13)
#define SCU_FUC_PIN_MII1_TX_DWN_DIS		(0x1 << 12)

#define SCU_FUC_PIN_MII2_TX_DRIV(x)		(x << 10)
#define SCU_FUC_PIN_MII2_TX_DRIV_MASK	(0x3 << 10)
#define SCU_FUC_PIN_MII1_TX_DRIV(x)		(x << 8)
#define SCU_FUC_PIN_MII1_TX_DRIV_MASK	(0x3 << 8)

#define MII_NORMAL_DRIV	0x0
#define MII_HIGH_DRIV	0x2

#define SCU_FUC_PIN_UART6		(0x1 << 7)
#define SCU_FUC_PIN_ROM_16BIT	(0x1 << 6)
#define SCU_FUC_PIN_DIGI_V_OUT(x)	(x)
#define SCU_FUC_PIN_DIGI_V_OUT_MASK	(0x3)

#define VIDEO_DISABLE	0x0
#define VIDEO_12BITS	0x1
#define VIDEO_24BITS	0x2
//#define VIDEO_DISABLE	0x3

#define SCU_FUC_PIN_USB11_PORT2	(0x1 << 3)

#define SCU_FUC_PIN_SD2		(0x1 << 2)
#define SCU_FUC_PIN_SD1		(0x1 << 1)
#define SCU_FUC_PIN_SDIO		(0x1 << 0)


/*	AST_SCU_FUN_PIN_CTRL6		0x94		Multi-function Pin Control#6*/
#define SCU_FUN_PIN_USBP1_MODE(x)	(x << 13)	/* Select USB2.0 Port\#2 function mode */
#define SCU_FUN_PIN_USBP1_MASK		(0x3 << 13)	/* Select USB2.0 Port\#2 function mode */
#define USB_HID_MODE	0
#define USB_DEV_MODE	1
#define USB_HOST_MODE	2
#define SCU_FUN_PIN_SGPIOP2			(0x1 << 12)	/* Enable Slave SGPIO port 2 function pins */
#define SCU_FUN_PIN_UART13			(0x1 << 11)	/* Enable UART13 function pins */
#define SCU_FUN_PIN_UART12			(0x1 << 10)	/* Enable UART12 function pins */
#define SCU_FUN_PIN_UART11			(0x1 << 9)	/* Enable UART11 function pins */
#define SCU_FUN_PIN_UART10			(0x1 << 8)	/* Enable UART10 function pins */
#define SCU_FUN_PIN_UART9			(0x1 << 7)	/* Enable UART9 function pins */
#define SCU_FUN_PIN_UART8			(0x1 << 6)	/* Enable UART8 function pins */
#define SCU_FUN_PIN_UART7			(0x1 << 5)	/* Enable UART7 function pins */
#define SCU_FUN_PIN_I2S2			(0x1 << 4)
#define SCU_FUN_PIN_I2S1			(0x1 << 3)
#define SCU_FUN_PIN_VIDEO_SO		(0x1 << 2)
#define SCU_FUN_PIN_DVO_24BIT		(0x1)
#define SCU_VIDEO_OUT_MASK		(~0x3)

/*	AST_SCU_WDT_RESET		0x9C		Watchdog Reset Selection */
/*	AST_SCU_FUN_PIN_CTRL7		0xA0		Multi-function Pin Control#7*/
/*	AST_SCU_FUN_PIN_CTRL8		0xA4		Multi-function Pin Control#8*/
#define SCU_FUN_PIN_ROMA17		(0x1 << 31)
#define SCU_FUN_PIN_ROMA16		(0x1 << 30)
#define SCU_FUN_PIN_ROMA15		(0x1 << 29)
#define SCU_FUN_PIN_ROMA14		(0x1 << 28)
#define SCU_FUN_PIN_ROMA13		(0x1 << 27)
#define SCU_FUN_PIN_ROMA12		(0x1 << 26)
#define SCU_FUN_PIN_ROMA11		(0x1 << 25)
#define SCU_FUN_PIN_ROMA10		(0x1 << 24)
#define SCU_FUN_PIN_ROMA9		(0x1 << 23)
#define SCU_FUN_PIN_ROMA8		(0x1 << 22)
#define SCU_FUN_PIN_ROMA7		(0x1 << 21)
#define SCU_FUN_PIN_ROMA6		(0x1 << 20)
#define SCU_FUN_PIN_ROMA5		(0x1 << 19)
#define SCU_FUN_PIN_ROMA4		(0x1 << 18)
#define SCU_FUN_PIN_ROMA3		(0x1 << 17)
#define SCU_FUN_PIN_ROMA2		(0x1 << 16)

/*	AST_SCU_FUN_PIN_CTRL9		0xA8		Multi-function Pin Control#9*/
#define SCU_FUN_PIN_ROMA21		(0x1 << 3)
#define SCU_FUN_PIN_ROMA20		(0x1 << 2)
#define SCU_FUN_PIN_ROMA19		(0x1 << 1)
#define SCU_FUN_PIN_ROMA18		(0x1)

/*	AST_SCU_PWR_SAVING_EN		0xC0		Power Saving Wakeup Enable*/
/*	AST_SCU_PWR_SAVING_CTRL		0xC4		Power Saving Wakeup Control*/
/*	AST_SCU_HW_STRAP2			0xD0		Haardware strapping register set 2*/


/*	AST_SCU_COUNTER4			0xE0		SCU Free Run Counter Read Back #4*/
/*	AST_SCU_COUNTER4_EXT		0xE4		SCU Free Run Counter Extended Read Back #4*/

//CPU 2 
/*	#define AST_SCU_CPU2_BASE0_ADDR	0x200		CPU2 Base Address for Segment 0x00:0000~0x0F:FFFF*/
/*	#define AST_SCU_CPU2_BASE1_ADDR	0x204		CPU2 Base Address for Segment 0x10:0000~0x1F:FFFF*/
/*	#define AST_SCU_CPU2_BASE2_ADDR	0x208		CPU2 Base Address for Segment 0x20:0000~0x2F:FFFF*/
/*	#define AST_SCU_CPU2_BASE3_ADDR	0x20C		CPU2 Base Address for Segment 0x30:0000~0x3F:FFFF*/
/*	#define AST_SCU_CPU2_BASE4_ADDR	0x210		CPU2 Base Address for Segment 0x40:0000~0x4F:FFFF*/
/*	#define AST_SCU_CPU2_BASE5_ADDR	0x214		CPU2 Base Address for Segment 0x50:0000~0x5F:FFFF*/
/*	#define AST_SCU_CPU2_BASE6_ADDR	0x218		CPU2 Base Address for Segment 0x60:0000~0x6F:FFFF*/
/*	#define AST_SCU_CPU2_BASE7_ADDR	0x21C		CPU2 Base Address for Segment 0x70:0000~0x7F:FFFF*/
/*	#define AST_SCU_CPU2_BASE8_ADDR	0x220		CPU2 Base Address for Segment 0x80:0000~0x8F:FFFF*/

/*	#define AST_SCU_CPU2_CACHE_CTRL	0x224		CPU2 Cache Function Control */


/*	AST_SCU_UART24_REF			0x160		Generate UART 24Mhz Ref from H-PLL when CLKIN is 25Mhz */
/*	AST_SCU_PCIE_CONFIG_SET		0x180		PCI-E Configuration Setting Control Register */
/*	AST_SCU_BMC_MMIO_DEC		0x184		BMC MMIO Decode Setting Register */
/*	AST_SCU_DEC_AREA1			0x188		1st relocated controller decode area location */
/*	AST_SCU_DEC_AREA2			0x18C		2nd relocated controller decode area location */
/*	AST_SCU_MBOX_DEC_AREA		0x190		Mailbox decode area location*/
/*	AST_SCU_SRAM_DEC_AREA0		0x194		Shared SRAM area decode location*/
/*	AST_SCU_SRAM_DEC_AREA1		0x198		Shared SRAM area decode location*/
/*	AST_SCU_BMC_CLASS			0x19C		BMC device class code and revision ID */
/*	AST_SCU_BMC_DEV_ID			0x1A4		BMC device ID */

#endif

