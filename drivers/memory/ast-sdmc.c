/********************************************************************************
* File Name     : arch/arm/mach-aspeed/ast-sdmc.c 
* Author         : Ryan Chen
* Description   : AST SDRAM Memory Ctrl
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


*   History      : 
*    1. 2013/03/15 Ryan Chen Create
* 
********************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/io.h>

#include <mach/platform.h>
#include <mach/hardware.h>
#include <mach/ast-sdmc.h>
/***********************  Registers for SDMC ***************************/ 
#define AST_SDMC_PROTECT	0x00		/*	protection key register	*/
#define AST_SDMC_CONFIG		0x04		/*	Configuration register */
#define AST_SDMC_MEM_REQ		0x08		/*	Graphics Memory Protection register */

/*	AST_SDMC_PROTECT: 0x00  - protection key register */
#define SDMC_PROTECT_UNLOCK			0xFC600309

#if defined(CONFIG_ARCH_AST1010)
/*	AST_SDMC_CONFIG :0x04	 - Configuration register */
#define SDMC_CONFIG_16MB			1
#else
/*	AST_SDMC_CONFIG :0x04	 - Configuration register */
#define SDMC_CONFIG_VER_NEW		(0x1 << 28)
#define SDMC_CONFIG_MEM_GET(x)		(x & 0x3)
#define SDMC_CONFIG_CACHE_EN		(0x1 << 10)
#define SDMC_CONFIG_EEC_EN			(0x1 << 7)
#endif
/**************************************************************/

//#define AST_SDMC_LOCK
//#define AST_SDMC_DEBUG

#ifdef AST_SDMC_DEBUG
#define SDMCDBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define SDMCDBUG(fmt, args...)
#endif

#define SDMCMSG(fmt, args...) printk(fmt, ## args)

void __iomem	*ast_sdmc_base = 0;

static inline u32 
ast_sdmc_read(u32 reg)
{
	u32 val;
		
	val = readl(ast_sdmc_base + reg);
	
	SDMCDBUG("ast_sdmc_read : reg = 0x%08x, val = 0x%08x\n", reg, val);
	
	return val;
}

static inline void
ast_sdmc_write(u32 val, u32 reg) 
{
	SDMCDBUG("ast_sdmc_write : reg = 0x%08x, val = 0x%08x\n", reg, val);
#ifdef CONFIG_AST_SDMC_LOCK
	//unlock 
	writel(SDMC_PROTECT_UNLOCK, ast_sdmc_base);
	writel(val, ast_sdmc_base + reg);
	//lock
	writel(0xaa,ast_sdmc_base);	
#else
	writel(SDMC_PROTECT_UNLOCK, ast_sdmc_base);

	writel(val, ast_sdmc_base + reg);
#endif
}

//***********************************Information ***********************************
extern void
ast_sdmc_disable_mem_protection(u8 req)
{
	ast_sdmc_write(ast_sdmc_read(AST_SDMC_MEM_REQ) & ~(1<< req), AST_SDMC_MEM_REQ);
}

extern u32
ast_sdmc_get_mem_size(void)
{
	u32 size=0;
	u32 conf = ast_sdmc_read(AST_SDMC_CONFIG);
	
	if(conf & SDMC_CONFIG_VER_NEW) {
		switch(SDMC_CONFIG_MEM_GET(conf)) {
			case 0:
				size = 128*1024*1024;
				break;
			case 1:
				size = 256*1024*1024;
				break;
			case 2:
				size = 512*1024*1024;
				break;
			case 3:
				size = 1024*1024*1024;
				break;
				
			default:
				SDMCMSG("error ddr size \n");
				break;
		}
		
	} else {
		switch(SDMC_CONFIG_MEM_GET(conf)) {
			case 0:
				size = 64*1024*1024;
				break;
			case 1:
				size = 128*1024*1024;
				break;
			case 2:
				size = 256*1024*1024;
				break;
			case 3:
				size = 512*1024*1024;
				break;
				
			default:
				SDMCMSG("error ddr size \n");
				break;
		}
	}
	return size;
}

extern u8
ast_sdmc_get_ecc(void)
{
	u32 conf = ast_sdmc_read(AST_SDMC_CONFIG);
	if(conf & SDMC_CONFIG_EEC_EN)
		return 1;
	else
		return 0;
}

extern u8
ast_sdmc_get_cache(void)
{
	u32 conf = ast_sdmc_read(AST_SDMC_CONFIG);
	if(conf & SDMC_CONFIG_CACHE_EN)
		return 1;
	else
		return 0;
}

static int ast_sdmc_probe(struct platform_device *pdev)
{
	struct resource *res;

	SDMCDBUG("\n");
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ast_sdmc_base = devm_ioremap_resource(&pdev->dev, res);

	return 0;
}

static const struct of_device_id ast_sdmc_of_match[] = {
	{ .compatible = "aspeed,ast-admc", },
	{ }
};

static struct platform_driver ast_sdmc_driver = {
	.probe = ast_sdmc_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = ast_sdmc_of_match,
	},
};

static int ast_bmc_scu_init(void)
{
	return platform_driver_register(&ast_sdmc_driver);
}

core_initcall(ast_sdmc_init);
