/*******************************************************************************
 *
 * Copyright (C) 2005-2015 Emulex. All rights reserved.
 * EMULEX is a trademark of Emulex.
 * www.emulex.com
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of version 2 of the GNU General Public License as published by the
 * Free Software Foundation.
 * This program is distributed in the hope that it will be useful. ALL EXPRESS
 * OR IMPLIED CONDITIONS, REPRESENTATIONS AND WARRANTIES, INCLUDING ANY IMPLIED
 * WARRANTY OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
 * NON-INFRINGEMENT, ARE DISCLAIMED, EXCEPT TO THE EXTENT THAT SUCH DISCLAIMERS
 * ARE HELD TO BE LEGALLY INVALID. See the GNU General Public License for more
 * details, a copy of which can be found in the file COPYING included
 * with this package.
 *
 ********************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/ide.h>
#include <linux/spinlock.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include "espi.h"

static int pilot_espi_probe(struct platform_device *pdev);
static int pilot_espi_remove(struct platform_device *pdev);

static int ESPI_MAJOR = 0;
static const char ESPI_DEVNAME[] = "eSPI";
unsigned int IRQ_ESPI_VWIRE;
unsigned int IRQ_SIO_PSR;
volatile unsigned char * SE_SYS_CLK_VA_BASE;
volatile unsigned char * SE_PILOT_SPEC_VA_BASE;
volatile unsigned char * SE_SCRATCH_128_VA_BASE;
volatile unsigned char * SE_PILOT_ESPI_VA_BASE;



static int espi_open(struct inode * inode, struct file * filp);
static int espi_close(struct inode * inode, struct file * filp);

static int espi_registered = 0;
static int espi_sys_device_added = 0;

extern long espi_vw_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
extern int espivw_init(struct device* device);
extern void cleanup_espivw_module(struct device* device);

static struct device espi_device;

static struct file_operations espi_fops = {
    owner:		THIS_MODULE,
    open:		espi_open,
    release:    espi_close,
};

static struct file_operations espi_vw_fops = {
    owner:		THIS_MODULE,
    unlocked_ioctl:		espi_vw_ioctl,
    release:    espi_close,
};

static int espi_open(struct inode * inode, struct file * filp)
{
    if(filp->f_flags & 010000000)
    {
        return 0;
    }

    switch (MINOR(inode->i_rdev)) {
        case 1:
            filp->f_op = &espi_vw_fops;
            break;

        default:
            return -ENXIO;
    }
    return 0;
}

static int espi_close(struct inode * inode, struct file * filp)
{
    return 0;
}

static void espi_dev_release(struct device *_dev)
{
    return;
}

static int pilot_espi_probe(struct platform_device *pdev)
{
	int err = 0, ret = 0;
	struct device_node *node;

	IRQ_ESPI_VWIRE = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!IRQ_ESPI_VWIRE) {
		pr_err("%s: unable to get the IRQ resource 0\n", __func__);
		return -EINVAL;
	}	
	IRQ_SIO_PSR = irq_of_parse_and_map(pdev->dev.of_node, 1);
	if (!IRQ_SIO_PSR) {
		pr_err("%s: unable to get the IRQ resource 1\n", __func__);
		return -EINVAL;
	}	

	node = of_find_compatible_node(NULL, NULL, "aspeed,pilot-espi-reg");
	SE_PILOT_SPEC_VA_BASE = of_iomap(node, 0);
	SE_SYS_CLK_VA_BASE = of_iomap(node, 1);
	SE_SCRATCH_128_VA_BASE = of_iomap(node, 2);
	SE_PILOT_ESPI_VA_BASE = of_iomap(node, 3); 	 


	// Check if eSPI strap is set. Otherwise do not insmod the driver
	if ((*(volatile unsigned long*)(SE_SYS_CLK_VA_BASE + 0x0C) & (1 << 14)) != (1 << 14))
	{
		printk("eSPI strap not set\n");
		return 0;
	}

	printk("eSPI Driver\n");
	printk("Copyright (c) 2015 Avago Technologies\n");

	/* ----  Register the character device ------------------- */
	if ((err = register_chrdev(ESPI_MAJOR, ESPI_DEVNAME, &espi_fops)) < 0)
	{
		printk ("failed to register chrdev (err %d)\n", err);
		ret = -ENODEV;
		goto fail;
	}
	espi_registered = 1;
	dev_set_name(&espi_device, "%s", ESPI_DEVNAME);
	espi_device.release = espi_dev_release;
	if ((err = device_register(&espi_device)) < 0)
	{
		printk ("failed to register device (err %d)\n", err);
		ret = -ENODEV;
		goto fail;
	}
	espi_sys_device_added = 1;

	if (espivw_init(&espi_device) < 0) {
		printk("espi vw init failed\n");
		ret = -ENODEV;
		goto fail;
	}
	return 0;

fail:
	pilot_espi_remove(pdev);
	return ret;
}
static int pilot_espi_remove(struct platform_device *pdev)
{
	/* free irq */
	cleanup_espivw_module(&espi_device);

	if (espi_sys_device_added)
	{
		device_unregister(&espi_device);
		espi_sys_device_added = 0;
	}

	/* unregister the device */
	if (espi_registered)
	{
		unregister_chrdev(ESPI_MAJOR, ESPI_DEVNAME);
		espi_registered = 0;
	}
	return 0;
}
static const struct of_device_id pilot_espi_of_match[] = {
	        { .compatible = "aspeed,pilot-espi", },
		{ }
};

static struct platform_driver pilot_espi_driver = {
	.driver = {
		.name		= "pilot_espi",
		.of_match_table = pilot_espi_of_match,
	},
	.probe		= pilot_espi_probe,
	.remove		= pilot_espi_remove,
};
module_platform_driver(pilot_espi_driver);

MODULE_DESCRIPTION("eSPI driver module.");
MODULE_LICENSE("GPL");
