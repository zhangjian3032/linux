// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) ASPEED Technology Inc.
 */

#include <linux/io.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/module.h>

#include "regs-aspeed-espi.h"

#define DEVICE_NAME     "espi-vw"

#define ESPIVWFIOC_BASE       'V'

#define ASPEED_ESPI_VW_GPIO_IOCRX			_IOWR(ESPIVWFIOC_BASE, 0, unsigned int)
#define ASPEED_ESPI_VW_SYS_IOCRX			_IOW(ESPIVWFIOC_BASE, 1, unsigned int)

struct aspeed_espi_vw {
	struct regmap 	*map;
	struct miscdevice       miscdev;
	u32 			vw_gpio;
	u32				sys_event;
	int 			irq;					//LPC IRQ number	
//	int				rest_irq;
	int				espi_version;
};

static irqreturn_t aspeed_espi_vw_irq(int irq, void *arg)
{
	u32 sys1_isr;
	u32 sts, vw_isr;
	u32 sys_evt_isr, sys_evt; 
	
	struct aspeed_espi_vw *espi_vw = arg;

	regmap_read(espi_vw->map, ASPEED_ESPI_ISR, &sts);
	vw_isr = sts & (ESPI_ISR_VIRTW_GPIO | ESPI_ISR_VIRTW_SYS | ESPI_ISR_VIRTW_SYS1);
	
	printk("aspeed_espi_vw_irq %x\n", vw_isr);

	if (vw_isr) {
		if (vw_isr & ESPI_ISR_VIRTW_GPIO) {
			printk("ESPI_ISR_VIRTW_GPIO \n");
			regmap_read(espi_vw->map, ASPEED_ESPI_GPIO_VIRTCH, &espi_vw->vw_gpio);
		}

		//sys event
		if (vw_isr & ESPI_ISR_VIRTW_SYS) {
			printk("ESPI_ISR_VIRTW_SYS \n");
			regmap_read(espi_vw->map, ASPEED_ESPI_SYS_EVENT_ISR, &sys_evt_isr);
			regmap_read(espi_vw->map, ASPEED_ESPI_SYS_EVENT, &sys_evt);
			
			printk("event isr %x, sys_event %x\n", sys_evt_isr, sys_evt);
			
			if (espi_vw->espi_version == ESPI_AST2500) {
				if (sys_evt_isr & ESPI_HOST_RST_WARN) {
					if (sys_evt & ESPI_HOST_RST_WARN)
						regmap_update_bits(espi_vw->map, ASPEED_ESPI_SYS_EVENT, ESPI_HOST_REST_ACK, ESPI_HOST_REST_ACK);
					else
						regmap_update_bits(espi_vw->map, ASPEED_ESPI_SYS_EVENT, ESPI_HOST_REST_ACK, 0);
				}
			
				if (sys_evt_isr & ESPI_OOB_RST_WARN) {
					if (sys_evt & ESPI_OOB_RST_WARN)
						regmap_update_bits(espi_vw->map, ASPEED_ESPI_SYS_EVENT, ESPI_OOB_REST_ACK, ESPI_OOB_REST_ACK);
					else
						regmap_update_bits(espi_vw->map, ASPEED_ESPI_SYS_EVENT, ESPI_OOB_REST_ACK, 0);
				}
			
				if (sys_evt_isr & ~(ESPI_OOB_RST_WARN | ESPI_HOST_RST_WARN)) {
					printk("new sts %x \n", sys_evt_isr);
				}
				regmap_write(espi_vw->map, ASPEED_ESPI_SYS_EVENT_ISR, sys_evt_isr);
			} else {
				printk("new sts %x \n", sys_evt_isr);
				regmap_write(espi_vw->map, ASPEED_ESPI_SYS_EVENT_ISR, sys_evt_isr);
			}

		}
	
		//AST2500 A1
		if (vw_isr & ESPI_ISR_VIRTW_SYS1) {
			printk("ESPI_ISR_VIRTW_SYS1 \n");
			regmap_read(espi_vw->map, ASPEED_ESPI_SYS1_INT_STS, &sys1_isr);
				
			if (sys1_isr & ESPI_SYS_SUS_WARN) {
				regmap_update_bits(espi_vw->map, ASPEED_ESPI_SYS1_EVENT, ESPI_SYS_SUS_ACK, ESPI_SYS_SUS_ACK);
			}
			
			if (sys1_isr & ~(ESPI_SYS_SUS_WARN)) {
				printk("new sys1 sts %x \n", sts);

			}
			regmap_write(espi_vw->map, ASPEED_ESPI_SYS1_INT_STS, sys1_isr);	
		}

		regmap_write(espi_vw->map, ASPEED_ESPI_ISR, vw_isr);
		return IRQ_HANDLED;		
	} else 
		return IRQ_NONE;

}

static long
espi_vw_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct miscdevice *c = file->private_data;
	struct aspeed_espi_vw *espi_vw = dev_get_drvdata(c->this_device);

	switch (cmd) {
	case ASPEED_ESPI_VW_GPIO_IOCRX:
		regmap_read(espi_vw->map, ASPEED_ESPI_GPIO_VIRTCH, &espi_vw->vw_gpio);
		ret = __put_user(espi_vw->vw_gpio, (u32 __user *)arg);
		espi_vw->vw_gpio = 0;
		break;
	case ASPEED_ESPI_VW_SYS_IOCRX:
		regmap_read(espi_vw->map, ASPEED_ESPI_SYS_EVENT, &espi_vw->sys_event);
		ret = __put_user(espi_vw->sys_event, (u32 __user *)arg);
		espi_vw->sys_event = 0;
		break;
	default:
		printk("ERROR \n");
		return -ENOTTY;
	}

	return ret;
}

static ssize_t show_sw_gpio(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	u32 espi_ctrl;
	struct aspeed_espi_vw *espi_vw = dev_get_drvdata(dev);

	regmap_read(espi_vw->map, ASPEED_ESPI_CTRL, &espi_ctrl);

	return sprintf(buf, "%s Mode\n", espi_ctrl & ESPI_CTRL_SW_GPIO_VIRTCH ? "1:SW" : "0:HW");
}

static ssize_t store_sw_gpio(struct device *dev,
							 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_espi_vw *espi_vw = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		regmap_update_bits(espi_vw->map, ASPEED_ESPI_CTRL, ESPI_CTRL_SW_GPIO_VIRTCH, ESPI_CTRL_SW_GPIO_VIRTCH);
	else
		regmap_update_bits(espi_vw->map, ASPEED_ESPI_CTRL, ESPI_CTRL_SW_GPIO_VIRTCH, 0);

	return count;
}

static DEVICE_ATTR(sw_gpio, S_IRUGO | S_IWUSR, show_sw_gpio, store_sw_gpio);

static struct attribute *espi_vw_sysfs_entries[] = {
	&dev_attr_sw_gpio.attr,
	NULL
};

static struct attribute_group espi_vw_attribute_group = {
	.attrs = espi_vw_sysfs_entries,
};

static const struct file_operations aspeed_espi_vw_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl		= espi_vw_ioctl,
};

static const struct of_device_id aspeed_espi_vw_match[] = {
	{ .compatible = "aspeed,ast2600-espi-virtial-wire", .data = (void *) ESPI_AST2600, },
	{ .compatible = "aspeed,ast2500-espi-virtial-wire", .data = (void *) ESPI_AST2500, },
	{ }
};
MODULE_DEVICE_TABLE(of, aspeed_espi_vw_match);

static int aspeed_espi_vw_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aspeed_espi_vw *espi_vw;
	const struct of_device_id *dev_id;	
	int rc;

	espi_vw = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_espi_vw), GFP_KERNEL);
	if (!espi_vw)
		return -ENOMEM;

	dev_id = of_match_device(aspeed_espi_vw_match, &pdev->dev);
	if (!dev_id)
		return -EINVAL;

	espi_vw->espi_version = (unsigned long)dev_id->data;

	espi_vw->map = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(espi_vw->map)) {
		dev_err(dev, "Couldn't get regmap\n");
		return -ENODEV;
	}

	espi_vw->irq = platform_get_irq(pdev, 0);
	if (espi_vw->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		return espi_vw->irq;
	}

	rc = sysfs_create_group(&pdev->dev.kobj, &espi_vw_attribute_group);
	if (rc) {
		printk(KERN_ERR "aspeed_espi_vw: failed to create sysfs device attributes.\n");
		return -1;
	}
	
	dev_set_drvdata(dev, espi_vw);

	
	rc = devm_request_irq(&pdev->dev, espi_vw->irq, aspeed_espi_vw_irq, IRQF_SHARED,
				dev_name(&pdev->dev), espi_vw);
	if (rc) {
		printk("espi vw Unable to get IRQ \n");
		return rc;
	}
#if 0
	espi_vw->rest_irq = platform_get_irq(pdev, 1);
	if (espi_vw->rest_irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		return espi_vw->irq;
	}
	rc = devm_request_irq(&pdev->dev, espi_vw->rest_irq, aspeed_espi_vw_reset_irq, IRQF_SHARED,
				dev_name(&pdev->dev), espi_vw);
	if (rc) {
		printk("espi vw Unable to get reset IRQ \n");
		return rc;
	}
#endif
	espi_vw->miscdev.minor = MISC_DYNAMIC_MINOR;
	espi_vw->miscdev.name = DEVICE_NAME;
	espi_vw->miscdev.fops = &aspeed_espi_vw_fops;
	espi_vw->miscdev.parent = dev;
	rc = misc_register(&espi_vw->miscdev);
	if (rc) {
		dev_err(dev, "Unable to register device\n");
		return rc;
	}

	pr_info("aspeed espi-vw loaded \n");

	return 0;
}

static int aspeed_espi_vw_remove(struct platform_device *pdev)
{
	struct aspeed_espi_vw *espi_vw = dev_get_drvdata(&pdev->dev);

	misc_deregister(&espi_vw->miscdev);
	return 0;
}


static struct platform_driver aspeed_espi_vw_driver = {
	.driver = {
		.name           = DEVICE_NAME,
		.of_match_table = aspeed_espi_vw_match,
	},
	.probe  = aspeed_espi_vw_probe,
	.remove = aspeed_espi_vw_remove,
};
module_platform_driver(aspeed_espi_vw_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("Aspeed ESPI VW Driver");
