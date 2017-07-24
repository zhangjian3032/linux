/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-i2c.c
* Author        : Ryan chen
* Description   : ASPEED I2C Device
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
*    1. 2012/07/30 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <asm/io.h>
#include <linux/gpio.h>


#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/ast_i2c.h>
#include <plat/devs.h>
#include <plat/regs-iic.h>
#include <plat/ast-scu.h>

#if defined(CONFIG_I2C_AST) || defined(CONFIG_I2C_AST_MODULE)

static struct ast_i2c_driver_data ast_i2c_data = {
	.master_dma = MASTER_XFER_MODE,
	.slave_dma = SLAVE_XFER_MODE,
	.request_pool_buff_page = request_pool_buff_page,
	.free_pool_buff_page = free_pool_buff_page,
	.get_i2c_clock = ast_get_pclk,
};

static struct ast_i2c_driver_data ast_i2c_data_1m = {
	.bus_clk = 100000000,		//bus clock 100KHz
	.master_dma = MASTER_XFER_MODE,
	.slave_dma = SLAVE_XFER_MODE,
	.request_pool_buff_page = request_pool_buff_page,
	.free_pool_buff_page = free_pool_buff_page,
	.get_i2c_clock = ast_get_pclk,
};


/*--------- I2C Board devices ------------*/
//AST EVB I2C Device 
static struct i2c_board_info __initdata ast_i2c_board_info_0[] = {
	{	
		I2C_BOARD_INFO("ddc", 0x50),
	},
#if defined(CONFIG_ARCH_AST3200)	
	{
		I2C_BOARD_INFO("cat66121_hdmi", 0x4C),
		.irq		= IRQ_GPIOB0,
	},
	{
		I2C_BOARD_INFO("cat66121_hdmi", 0x4D),
		.irq		= IRQ_GPIOB1,
	},	
#endif	
};

static struct i2c_board_info __initdata ast_i2c_board_info_1[] = {
#if defined(CONFIG_ARCH_AST3200)	
	{
		I2C_BOARD_INFO("cat66121_hdmi", 0x4C),
		.irq		= IRQ_GPIOB2,
	},
	{
		I2C_BOARD_INFO("cat66121_hdmi", 0x4D),
		.irq		= IRQ_GPIOB3,
	},	
#endif	
};

static struct i2c_board_info __initdata ast_i2c_board_info_2[] = {
};

//Under I2C Dev 3
static struct i2c_board_info __initdata ast_i2c_board_info_3[] = {
	{
		I2C_BOARD_INFO("24c08", 0x50),						
	},
};
//Under I2C Dev 8
static struct i2c_board_info __initdata ast_i2c_board_info_7[] = {
	{
		I2C_BOARD_INFO("lm75", 0x4d),						
	}
};

/*-------------------------------------*/
void __init ast_add_device_i2c(void)
{
	
	
	i2c_register_board_info(0, ast_i2c_board_info_0, ARRAY_SIZE(ast_i2c_board_info_0));	
	i2c_register_board_info(1, ast_i2c_board_info_1, ARRAY_SIZE(ast_i2c_board_info_1));	
	i2c_register_board_info(2, ast_i2c_board_info_2, ARRAY_SIZE(ast_i2c_board_info_2));		
	i2c_register_board_info(3, ast_i2c_board_info_3, ARRAY_SIZE(ast_i2c_board_info_3));
	i2c_register_board_info(7, ast_i2c_board_info_7, ARRAY_SIZE(ast_i2c_board_info_7));
	
}
#else

#if 0
static struct i2c_gpio_platform_data i2c_gpio_data = {
	.sda_pin		= PIN_GPIOB3,
	.scl_pin		= PIN_GPIOB2,
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.udelay			= 40,
}; /* This hasn't actually been used these pins
    * are (currently) free pins on the expansion connector */

static struct platform_device i2c_gpio_device = {
	.name		= "i2c-gpio",
	.id		= 0,
	.dev		= {
		.platform_data	= &i2c_gpio_data,
	},
};
#endif


#endif
