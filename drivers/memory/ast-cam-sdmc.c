/*
 * ast-cam-sdmc.c - SDMC driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/hwmon-sysfs.h>

#include <mach/ast-sdmc.h>
/************************************  Registers for SDMC ****************************************/ 
#define AST_SDMC_PROTECT				0x00		/*	protection key register	*/
#define AST_SDMC_CONFIG				0x04		/*	Configuration register */

/*	AST_SDMC_PROTECT: 0x00  - protection key register */
#define SDMC_PROTECT_UNLOCK			0xFC600309

/*	AST_SDMC_CONFIG :0x04	 - Configuration register */
#define SDMC_CONFIG_VER_NEW			(0x1 << 28)
#define SDMC_CONFIG_MEM_GET(x)		(x & 0x3)

#define SDMC_CONFIG_CACHE_EN			(0x1 << 10)
/****************************************************************************************/

//#define AST_SDMC_LOCK
//#define AST_SDMC_DEBUG

#ifdef AST_SDMC_DEBUG
#define SDMCDBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define SDMCDBUG(fmt, args...)
#endif

#define SDMCMSG(fmt, args...) printk(fmt, ## args)
/****************************************************************************************************************/
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
/****************************************************************************************************************/
extern u32
ast_sdmc_get_mem_size(void)
{
	u32 size=0;
	u32 conf = ast_sdmc_read(AST_SDMC_CONFIG);

	switch(SDMC_CONFIG_MEM_GET(conf)) {
		case 0:
			size = 512*1024*1024;
			break;
		case 1:
			size = 1024*1024*1024;
			break;
		case 2:
			size = 2048*1024*1024;
			break;
		default:
			SDMCMSG("error ddr size \n");
			break;
	}
		
	return size;
}

/************************************************** SYS FS **************************************************************/

//***********************************Information ***********************************
static int ast_cam_sdmc_probe(struct platform_device *pdev)
{
	struct resource *res;

	SDMCDBUG("\n");
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ast_sdmc_base = devm_ioremap_resource(&pdev->dev, res);

	return 0;
}

static const struct of_device_id ast_cam_sdmc_of_match[] = {
	{ .compatible = "aspeed,ast-cam-sdmc", },
	{ }
};

static struct platform_driver ast_cam_sdmc_driver = {
	.probe = ast_cam_sdmc_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = ast_cam_sdmc_of_match,
	},
};

static int ast_cam_sdmc_init(void)
{
	return platform_driver_register(&ast_cam_sdmc_driver);
}

core_initcall(ast_cam_sdmc_init);
