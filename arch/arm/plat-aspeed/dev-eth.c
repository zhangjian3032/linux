/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-eth.c
* Author        : Ryan Chen
* Description   : Aspeed Ethernet Device
*
* Copyright (C) ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by the Free Software Foundation;
* either version 2 of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*   History      :
*    1. 2012/08/24 Ryan Chen initial
*
********************************************************************************/
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#ifdef CONFIG_COLDFIRE
#include "../../arm/plat-aspeed/include/plat/devs.h"
#include "../../arm/mach-aspeed/include/mach/irqs.h"
#include "../../arm/mach-aspeed/include/mach/ast1010_platform.h"
#else
#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/ast-scu.h>
#endif

#if defined(CONFIG_ARCH_AST3200)
#undef AST_MAC1_BASE
#endif

/* --------------------------------------------------------------------
 *  Ethernet
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_MAC) || defined(CONFIG_AST_MAC_MODULE)
#ifdef AST_MAC0_BASE
static u64 ast_eth_dmamask = 0xffffffffUL;
static struct resource ast_mac0_resources[] = {
	[0] = {
		.start = AST_MAC0_BASE,
		.end = AST_MAC0_BASE + SZ_128K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_MAC0,
		.end = IRQ_MAC0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_eth0_device = {
	.name		= "ast_gmac",
	.id		= 0,
	.dev		= {
				.dma_mask		= &ast_eth_dmamask,
				.coherent_dma_mask	= 0xffffffff,
	},
	.resource	= ast_mac0_resources,
	.num_resources = ARRAY_SIZE(ast_mac0_resources),
};
#endif
#ifdef AST_MAC1_BASE
static struct resource ast_mac1_resources[] = {
	[0] = {
		.start = AST_MAC1_BASE,
		.end = AST_MAC1_BASE + SZ_128K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_MAC1,
		.end = IRQ_MAC1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_eth1_device = {
	.name		= "ast_gmac",
	.id		= 1,
	.dev		= {
				.dma_mask		= &ast_eth_dmamask,
				.coherent_dma_mask	= 0xffffffff,
	},
	.resource	= ast_mac1_resources,
	.num_resources = ARRAY_SIZE(ast_mac1_resources),
};
#endif

/*
 * MAC1 always has MII MDC+MDIO pins to access PHY registers.  We assume MAC1
 * always has a PHY chip, if MAC1 is enabled.
 * U-Boot can enable MAC2 MDC+MDIO pins for a 2nd PHY, or MAC2 might be
 * disabled (only one port), or it's sideband-RMII which has no PHY chip.
 *
 * Return miiPhyId==0 if the MAC cannot be accessed.
 * Return miiPhyId==1 if the MAC registers are OK but it cannot carry traffic.
 * Return miiPhyId==2 if the MAC can send/receive but it has no PHY chip.
 * Else return the PHY 22-bit vendor ID, 6-bit model and 4-bit revision.
 */

void __init ast_add_device_gmac(void)
{
#if defined(CONFIG_COLDFIRE)
#else
	ast_scu_init_eth(0);
	ast_scu_multi_func_eth(0);

	// We assume the Clock Stop register does not disable the MAC1 or MAC2 clock
	// unless Reset Control also holds the MAC in reset.
#endif
	
	platform_device_register(&ast_eth0_device);

#ifdef AST_MAC1_BASE
	ast_scu_init_eth(1);
	ast_scu_multi_func_eth(1);	

	platform_device_register(&ast_eth1_device);

#endif

}
#else
void __init ast_add_device_gmac(void) {}
#endif

