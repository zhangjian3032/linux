/********************************************************************************
* File Name     : drivers/video/hdmi_cat6613.c
* Author        : Ryan Chen
* Description   : VGA EDID driver
*
* Copyright (C) Socle Tech. Corp.
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
*    1. 2011/12/18 Ryan create this file
*
********************************************************************************/
#include <linux/device.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/sched.h>

#include "display-sys.h"
#include "../edid.h"

#define CONFIG_DDC_DEBUG

#ifdef CONFIG_DDC_DEBUG
	#define DDC_DBG(fmt, args...) printk("%s(): " fmt, __FUNCTION__, ## args)
#else
	#define DDC_DBG(fmt, args...)
#endif

#define UNPLUG_TIMER 200
#define PLUG_TIMER 3000

#define DDC_ADDR	0x50
#define DEVICE_NAME "ddc"

struct vga_ddc_info {
	struct i2c_client *client;
	struct fb_info *fb_info;
//	struct monitor_info *mon_info;
	u8 state;//0:unplug 1:plug
	int count;
};

static void vga_detect_work(struct work_struct *ws);


static DECLARE_DELAYED_WORK(detect_work, vga_detect_work);
static struct workqueue_struct *detect_workqueue;

int vga_read_ddc_edid(struct ast_display_device *disp_dev, u8 *buf)
{
	int i;
	struct vga_ddc_info *vga_ddc = (struct vga_ddc_info *)disp_dev->priv_data;
	struct i2c_client *client = vga_ddc->client;
	unsigned char start = 0x0;
	struct i2c_msg msgs[] = {
		{
			.addr	= DDC_ADDR,
			.flags	= 0,
			.len	= 1,
			.buf	= &start,
		}, {
			.addr	= DDC_ADDR,
			.flags	= I2C_M_RD,
			.len	= EDID_LENGTH,
			.buf	= buf,
		}
	};

	if (i2c_transfer(client->adapter, msgs, 2) == 2) {
		vga_ddc->state = 1;
		return 1;
	} else {
		vga_ddc->state = 0;
	}
	return 0;
}

static void vga_detect_work(struct work_struct *ws)
{
//TODO ~~~ container_of
#if 0
	if(i2c_smbus_read_byte_data(vga_ddc->client, 0) != 0x00) {
		if(vga_ddc->state !=0) {
//			change_call_back;;;
		}
		vga_ddc->state = 0;
		DDC_DBG("UNPLUG \n");
		queue_delayed_work(detect_workqueue, &detect_work, UNPLUG_TIMER);
	} else {
		if(vga_ddc->state !=1) {
//			change_call_back;;;
		}
		vga_ddc->state = 1;
		DDC_DBG("PLUG \n");
		queue_delayed_work(detect_workqueue, &detect_work, PLUG_TIMER);
	}
#endif	
}

static struct ast_display_ops ddc_display_ops = {
	.getedid = vga_read_ddc_edid,
#if 0	
	.setenable = cvbs_set_enable,
	.getenable = cvbs_get_enable,
	.getstatus = cvbs_get_status,
	.getmodelist = cvbs_get_modelist,
	.setmode = cvbs_set_mode,
	.getmode = cvbs_get_mode,
#endif	
};

static int display_vga_probe(struct ast_display_device *device, void *devdata)
{
	device->owner = THIS_MODULE;
	strcpy(device->type, "VGA");
	device->name = "ddc_vga";
	device->priv_data = devdata;
	device->ops = &ddc_display_ops;
	return 1;
}


static struct ast_display_driver display_vga = {
	.probe = display_vga_probe,
	.display_no = 0,
	.name = "vga_ddc"
};

static int vga_ddc_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	DDC_DBG("\n");

	if (!(vga_ddc = kzalloc(sizeof(struct vga_ddc_info), GFP_KERNEL))) {
		return -ENOMEM;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE | I2C_FUNC_I2C))
		return -ENODEV;

	i2c_set_clientdata(client, vga_ddc);
	vga_ddc->state = 0;
	vga_ddc->client = client;

#ifdef CONFIG_DISPLAY_SUPPORT
	ast_display_device_register(&display_vga, client->dev.parent, vga_ddc);	
#endif
//	detect_workqueue = create_workque4ue("detect_wq");
//	queue_delayed_work(detect_workqueue, &detect_work, 0);
	return 0;
}

static const struct i2c_device_id vga_ddc_id[] = {
	{ DEVICE_NAME, 0 },
	{ }
};

static struct i2c_driver vga_ddc_i2c_driver = {
	.class = I2C_CLASS_DDC,	
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
	},
	.probe = vga_ddc_probe,
	.id_table = vga_ddc_id,
};

//EXPORT_SYMBOL(vga_read_ddc_edid);

int __init vga_ddc_init(void)
{
	int ret;
	ret = i2c_add_driver(&vga_ddc_i2c_driver);
	if (ret)
		printk(KERN_ERR "%s: failed to add i2c driver\n", __func__);

	return ret;
}

static void __exit vga_ddc_exit(void)
{
	i2c_del_driver(&vga_ddc_i2c_driver);
}

//Should after I2C bus driver 
subsys_initcall_sync(vga_ddc_init);
//module_init(vga_ddc_init);
module_exit(vga_ddc_exit);

#if 0
MODULE_AUTHOR("Ryan Chen");
MODULE_DESCRIPTION("VGA DDC driver");
MODULE_LICENSE("GPL");
#endif
