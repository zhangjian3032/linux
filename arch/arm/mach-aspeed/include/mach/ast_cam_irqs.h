/*
 *  arch/arm/plat-aspeed/include/plat/irqs.h
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _AST_CAM_IRQS_H_
#define _AST_CAM_IRQS_H_

#define NR_IRQS							(AST_VIC_NUM + AST_FIQ_NUM  + ARCH_NR_I2C + ARCH_NR_GPIOS + ARCH_NR_SDHCI)

#define IRQ_SDHCI_CHAIN_START			(IRQ_GPIO_CHAIN_START + ARCH_NR_SDHCI)

#define IRQ_GPIO_CHAIN_START			(IRQ_I2C_CHAIN_START + ARCH_NR_I2C)

#define IRQ_I2C_CHAIN_START			(AST_VIC_NUM + AST_FIQ_NUM)

#define FIQ_START 						AST_VIC_NUM

#define AST_FIQ_NUM						64

#define AST_VIC_NUM						64

#define IRQ_CFV0						0
#define IRQ_CFV1						1
#define IRQ_SDIO							2
#define IRQ_SDHCI						3
#define IRQ_H264						4
#define IRQ_JPEG							5
#define IRQ_D3D							6
#define IRQ_PP							7
#define IRQ_ISP0							8
#define IRQ_ISP1							9
#define IRQ_ISP2							10
#define IRQ_ISP3							11
#define IRQ_ISP4							12
#define IRQ_ISP5							13
#define IRQ_ZSP							14
#define IRQ_PDM							15
#define IRQ_I2S							16
#define IRQ_CRYPTO						17
#define IRQ_MAC0						18			/* MAC 1 interrupt */
#define IRQ_UDC							19
#define IRQ_I2C							20
#define IRQ_GPIO							21
#define IRQ_TIMER0						22			/* TIMER 1 interrupt */
#define IRQ_TIMER1						23			/* TIMER 2 interrupt */
#define IRQ_TIMER2						24			/* TIMER 3 interrupt */
#define IRQ_TIMER3						25			/* TIMER 4 interrupt */
#define IRQ_TIMER4						26			/* TIMER 5 interrupt */
#define IRQ_TIMER5						27			/* TIMER 6 interrupt */
#define IRQ_TIMER6						28			/* TIMER 7 interrupt */
#define IRQ_TIMER7						29			/* TIMER 8 interrupt */
#define IRQ_UART0						30			/* UART 1 interrupt */
#define IRQ_UART1						31			/* UART 2 interrupt */
#define IRQ_UART2						32			/* UART 3 interrupt */
#define IRQ_UART_DMA					33
#define IRQ_FMC							34
#define IRQ_WDT							35
#define IRQ_RTC							36
#define IRQ_AHBC						37

#endif
