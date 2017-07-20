/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-spi.c
* Author        : Ryan chen
* Description   : ASPEED SPI device
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

* History      :
*    1. 2012/08/01 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <asm/io.h>

#ifdef CONFIG_COLDFIRE
#include "../../arm/plat-aspeed/include/plat/devs.h"
#include "../../arm/mach-aspeed/include/mach/irqs.h"
#include "../../arm/mach-aspeed/include/mach/ast1010_platform.h"
#else
#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/regs-fmc.h>
#include <plat/ast-scu.h>
#include <mach/ast_spi.h>
#endif



static struct flash_platform_data ast_spi_flash0_data = {
#if defined(CONFIG_ARCH_AST2400) || defined(CONFIG_ARCH_AST2500)
	.type 		  = "mx25l25635e",	//AST2400 A1
#elif defined(CONFIG_ARCH_AST3200)
	.type 		  = "w25q256",	
#else		
	.type 		  = "mx25l12805d",	//old AST2300 
#endif		
	.nr_parts       = ARRAY_SIZE(ast_spi_flash0_partitions),
	.parts          = ast_spi_flash0_partitions,
};

static struct flash_platform_data ast_spi_flash1_data = {
//	.type         	= "mx25l2005a",
	.type         	= "mx25l6405d",
	.nr_parts       	= ARRAY_SIZE(ast_spi_flash1_partitions),
	.parts          	= ast_spi_flash1_partitions,
};

static struct spi_board_info ast_spi_devices[] = {
	{
		.modalias    		= "m25p80",
		.platform_data  	= &ast_spi_flash0_data,
		.chip_select    		= 0, //.chip_select This tells your device driver which chipselect to use.
		.max_speed_hz    	= 100 * 1000 * 1000, 
		.bus_num    		= 0, //  This chooses if SPI0 or SPI1 of the SoC is used.
		.mode 			= SPI_MODE_0,
#if 0		
	}, {
		.modalias    		= "spidev",
		.chip_select    		= 0,
		.max_speed_hz    	= 30 * 1000 * 1000,
		.bus_num    		= 0,
		.mode 			= SPI_MODE_0,
#endif		
	},
};

static struct spi_board_info ast_spi1_devices[] = {
	{
		.modalias    		= "m25p80",
		.platform_data  	= &ast_spi_flash1_data,
		.chip_select    		= 0, //.chip_select This tells your device driver which chipselect to use.
		.max_speed_hz    	= 50 * 1000 * 1000, 
		.bus_num    		= 1, //  This chooses if SPI0 or SPI1 of the SoC is used.
		.mode 			= SPI_MODE_0,
	},
};

#if defined(AST_SPI1_BASE)
static struct mtd_partition ast_spi_flash1_partitions[] = {
		{
			.name	= "bios",
			.offset = 0,
			.size	= MTDPART_SIZ_FULL,
        }
};

static struct flash_platform_data ast_spi_flash1_data = {
//        .type           = "w25q64",	
	.type           = "mx25l6405d",
        .nr_parts       = ARRAY_SIZE(ast_spi_flash1_partitions),
        .parts          = ast_spi_flash1_partitions,
};


static struct spi_board_info ast_spi1_devices[] = {
    {
        .modalias    = "m25p80",
	.platform_data  = &ast_spi_flash1_data,
        .chip_select    = 0, //.chip_select This tells your device driver which chipselect to use.
        .max_speed_hz    = 100 * 1000 * 1000, 
        .bus_num    = 1, //  This chooses if SPI0 or SPI1 of the SoC is used.
     	.mode = SPI_MODE_0,
    }, 
};
#endif

#if defined(CONFIG_SPI_FMC) || defined(CONFIG_SPI_FMC_MODULE) || defined(CONFIG_SPI_AST) || defined(CONFIG_SPI_AST_MODULE)

/*-------------------------------------*/
void __init ast_add_device_spi(void)
{
#if defined(CONFIG_SPI_FMC) || defined(CONFIG_SPI_FMC_MODULE)
	platform_device_register(&ast_fmc0_spi_device);
	spi_register_board_info(ast_spi_devices, ARRAY_SIZE(ast_spi_devices));
#endif

//	platform_device_register(&ast_fmc_spi0_device);
//	platform_device_register(&ast_fmc_spi1_device);
//	spi_register_board_info(ast_spi1_devices, ARRAY_SIZE(ast_spi1_devices));

#if defined(CONFIG_SPI_AST) || defined(CONFIG_SPI_AST_MODULE)
	//pin switch by trap[13:12]	-- [0:1] Enable SPI Master 
	platform_device_register(&ast_spi_device0);
	spi_register_board_info(ast_spi1_devices, ARRAY_SIZE(ast_spi1_devices));
#endif

#if defined(AST_SPI1_BASE)
	//AST1010 SCU CONFIG TODO .......
	writel(readl(AST_SCU_BASE + 0x70) | 0x10,AST_SCU_BASE + 0x70);
	platform_device_register(&aspeed_spi_device1);
	spi_register_board_info(ast_spi1_devices, ARRAY_SIZE(ast_spi1_devices));
#endif	

}
#else
void __init ast_add_device_spi(void) {}
#endif


