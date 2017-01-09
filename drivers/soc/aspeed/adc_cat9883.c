/********************************************************************************
* File Name     : drivers/video/adc_cat9883.c
* Author        : Ryan Chen 
* Description   : ADC CAT9883 driver
*
* Copyright (C) 2012-2020  ASPEED Technology Inc.
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
*    1. 2013/08/24 Ryan Chen create this file
*
********************************************************************************/

#include <linux/i2c.h>
#include <linux/delay.h>

//#include <mach/regs-cat9883.h>

#define DEVICE_NAME "cat9883"

struct cat9883_info {
	struct i2c_client *client;
};

static struct cat9883_info cat9883_device;


static int adc_cat9883_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	printk("adc_cat9883_probe \n");

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE | I2C_FUNC_I2C))
		return -ENODEV;
	
	printk("initinal adc9883 Set Hsync to negative \n");
	cat9883_device.client=client;

	i2c_smbus_write_byte_data(cat9883_device.client, 0x0e, 0x64);
	
	return 0;
}

static int __devexit adc_cat9883_remove(struct i2c_client *client)
{

	return 0;
}


static const struct i2c_device_id adc_cat9883_id[] = {
	{ DEVICE_NAME, 0 },
	{ }
};

static struct i2c_driver adc_cat9883_i2c_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
	},
	.probe = adc_cat9883_probe,
	.remove =  __exit_p(adc_cat9883_remove),
	.id_table = adc_cat9883_id,
};


void adc_enable(int en)
{
}

static int __init adc_cat9883_init(void)
{
	int ret;
	
	ret = i2c_add_driver(&adc_cat9883_i2c_driver);
	if (ret)
		printk(KERN_ERR "%s: failed to add i2c driver\n", __func__);

	return ret;
}

static void __exit adc_cat9883_exit(void)
{
	i2c_del_driver(&adc_cat9883_i2c_driver);
}

module_init(adc_cat9883_init);
module_exit(adc_cat9883_exit);

EXPORT_SYMBOL(adc_enable);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeed-tech.com>");
MODULE_DESCRIPTION("CAT9883 ADC Driver");
MODULE_LICENSE("GPL");
