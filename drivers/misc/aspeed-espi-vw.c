// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) ASPEED Technology Inc.
 */

#include <linux/atomic.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <linux/poll.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/of_device.h>

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>

#include "regs-aspeed-espi.h"

#define ESPIVWFIOC_BASE       'V'


#define ASPEED_ESPI_VW_GPIO_IOCRX			_IOWR(ESPIVWFIOC_BASE, 0, unsigned int)
#define ASPEED_ESPI_VW_SYS_IOCRX			_IOW(ESPIVWFIOC_BASE, 1, unsigned int)

struct aspeed_espi_vw {
	struct regmap *map;
	bool 					is_open;
	u32 					vw_gpio;
	u32						sys_event;
	int 					irq;					//LPC IRQ number	
	int		espi_version;

};

static u32 aspeed_espi_vw_read(struct aspeed_espi_vw *espi_vw, u32 reg)
{

	u32 val = 0;
	int rc;

	rc = regmap_read(espi_vw->map, reg, &val);
	WARN(rc != 0, "regmap_read() failed: %d\n", rc);

	return rc == 0 ? val : 0;
}

static void aspeed_espi_vw_write(struct aspeed_espi_vw *espi_vw, u32 reg, u32 data)
{

	int rc;

	rc = regmap_write(espi_vw->map, reg, data);
	WARN(rc != 0, "regmap_write() failed: %d\n", rc);
}

static void
aspeed_sys_event(struct aspeed_espi_vw *espi_vw)
{
	u32 sts = aspeed_espi_vw_read(espi_vw, ASPEED_ESPI_SYS_EVENT_ISR);
	u32 sys_event = aspeed_espi_vw_read(espi_vw, ASPEED_ESPI_SYS_EVENT);
	printk("sts %x, sys_event %x\n", sts, sys_event);

	if (espi_vw->espi_version == 5) {
		if (sts & ESPI_HOST_RST_WARN) {
			if (sys_event & ESPI_HOST_RST_WARN)
				aspeed_espi_vw_write(espi_vw, ASPEED_ESPI_SYS_EVENT, sys_event | ESPI_HOST_REST_ACK);
			else
				aspeed_espi_vw_write(espi_vw, ASPEED_ESPI_SYS_EVENT, sys_event & ~ESPI_HOST_REST_ACK);
			aspeed_espi_vw_write(espi_vw, ASPEED_ESPI_SYS_EVENT_ISR, ESPI_HOST_RST_WARN);
		}

		if (sts & ESPI_OOB_RST_WARN) {
			if (sys_event & ESPI_OOB_RST_WARN)
				aspeed_espi_vw_write(espi_vw, ASPEED_ESPI_SYS_EVENT, sys_event | ESPI_OOB_REST_ACK);
			else
				aspeed_espi_vw_write(espi_vw, ASPEED_ESPI_SYS_EVENT, sys_event & ~ESPI_OOB_REST_ACK);
			aspeed_espi_vw_write(espi_vw, ASPEED_ESPI_SYS_EVENT_ISR, ESPI_OOB_RST_WARN);
		}

		if (sts & ~(ESPI_OOB_RST_WARN | ESPI_HOST_RST_WARN)) {
			printk("new sts %x \n", sts);
			aspeed_espi_vw_write(espi_vw, ASPEED_ESPI_SYS_EVENT_ISR, sts);
		}

	} else {
		printk("new sts %x \n", sts);
		aspeed_espi_vw_write(espi_vw, ASPEED_ESPI_SYS_EVENT_ISR, sts);
	}

}

static void
aspeed_sys1_event(struct aspeed_espi_vw *espi_vw)
{
	u32 sts = aspeed_espi_vw_read(espi_vw, ASPEED_ESPI_SYS1_INT_STS);
	if (sts & ESPI_SYS_SUS_WARN) {
		aspeed_espi_vw_write(espi_vw, ASPEED_ESPI_SYS1_EVENT, aspeed_espi_vw_read(espi_vw, ASPEED_ESPI_SYS1_EVENT) | ESPI_SYS_SUS_ACK);
		//TODO  polling bit 20 is 1
		aspeed_espi_vw_write(espi_vw, ASPEED_ESPI_SYS1_INT_STS, ESPI_SYS_SUS_WARN);
	}

	if (sts & ~(ESPI_SYS_SUS_WARN)) {
		printk("new sys1 sts %x \n", sts);
		aspeed_espi_vw_write(espi_vw, ASPEED_ESPI_SYS1_INT_STS, sts);
	}

}

static irqreturn_t aspeed_espi_vw_irq(int irq, void *arg)
{
	struct aspeed_espi_vw *espi_vw = arg;

	u32 sts = aspeed_espi_vw_read(espi_vw, ASPEED_ESPI_ISR);
	printk("aspeed_espi_vw_irq %x\n", sts);

	if (sts & (ESPI_ISR_VIRTW_GPIO | ESPI_ISR_VIRTW_SYS | ESPI_ISR_VIRTW_SYS1)) {
		if (sts & ESPI_ISR_VIRTW_GPIO) {
			printk("ESPI_ISR_VIRTW_GPIO \n");
			espi_vw->vw_gpio = aspeed_espi_vw_read(espi_vw, ASPEED_ESPI_GPIO_VIRTCH);
			aspeed_espi_vw_write(espi_vw, ASPEED_ESPI_ISR, ESPI_ISR_VIRTW_GPIO);
		}
	
		if (sts & ESPI_ISR_VIRTW_SYS) {
			printk("ESPI_ISR_VIRTW_SYS \n");
	//		aspeed_espi->sys_event = aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT);
			aspeed_sys_event(espi_vw);
			aspeed_espi_vw_write(espi_vw, ASPEED_ESPI_ISR, ESPI_ISR_VIRTW_SYS);
		}
	
		//AST2500 A1
		if (sts & ESPI_ISR_VIRTW_SYS1) {
			printk("ESPI_ISR_VIRTW_SYS1 \n");
			aspeed_sys1_event(espi_vw);
			aspeed_espi_vw_write(espi_vw, ASPEED_ESPI_ISR, ESPI_ISR_VIRTW_SYS1);
		}
	
		return IRQ_HANDLED;		
	} else 
		return IRQ_NONE;

}

static void
aspeed_espi_vw_init(struct aspeed_espi_vw *espi_vw) {

//controller init 
}

static DEFINE_SPINLOCK(espi_vw_state_lock);


static long
espi_vw_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct miscdevice *c = file->private_data;
	struct aspeed_espi_vw *espi_vw = dev_get_drvdata(c->this_device);

	switch (cmd) {
	case ASPEED_ESPI_VW_GPIO_IOCRX:
		printk(" \n");
		espi_vw->vw_gpio = aspeed_espi_vw_read(espi_vw, ASPEED_ESPI_GPIO_VIRTCH);
		ret = __put_user(espi_vw->vw_gpio, (u32 __user *)arg);
		espi_vw->vw_gpio = 0;
		break;
	case ASPEED_ESPI_VW_SYS_IOCRX:
		printk(" \n");
		espi_vw->sys_event = aspeed_espi_vw_read(espi_vw, ASPEED_ESPI_SYS_EVENT) & 0xffff;
		ret = __put_user(espi_vw->sys_event, (u32 __user *)arg);
		espi_vw->sys_event = 0;
		break;
	default:
		printk("ERROR \n");
		return -ENOTTY;
	}

	return ret;
}

static int espi_vw_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_espi_vw *espi_vw = dev_get_drvdata(c->this_device);

	printk("\n");
	spin_lock(&espi_vw_state_lock);

	if (espi_vw->is_open) {
		spin_unlock(&espi_vw_state_lock);
		return -EBUSY;
	}

	espi_vw->is_open = true;

	spin_unlock(&espi_vw_state_lock);

	return 0;
}

static int espi_vw_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_espi_vw *espi_vw = dev_get_drvdata(c->this_device);

	printk("\n");
	spin_lock(&espi_vw_state_lock);

	espi_vw->is_open = false;
	spin_unlock(&espi_vw_state_lock);

	return 0;
}

static ssize_t show_sw_gpio(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	struct aspeed_espi_vw *espi_vw = dev_get_drvdata(dev);

	return sprintf(buf, "%s Mode\n", aspeed_espi_vw_read(espi_vw, ASPEED_ESPI_CTRL) & ESPI_CTRL_SW_GPIO_VIRTCH ? "1:SW" : "0:HW");
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
	.open			= espi_vw_open,
	.release			= espi_vw_release,
};

struct miscdevice aspeed_espi_vw_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "espi-vw",
	.fops = &aspeed_espi_vw_fops,
};

static const struct of_device_id aspeed_espi_vw_match[] = {
	{ .compatible = "aspeed,ast2600-espi-virtial-wire", .data = (void *) 6, },
	{ .compatible = "aspeed,ast2500-espi-virtial-wire", .data = (void *) 5, },
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

	espi_vw->map = syscon_node_to_regmap(pdev->dev.of_node);
	if (IS_ERR(espi_vw->map)) {
		dev_err(dev, "Couldn't get regmap\n");
		return -ENODEV;
	}

	rc = sysfs_create_group(&pdev->dev.kobj, &espi_vw_attribute_group);
	if (rc) {
		printk(KERN_ERR "aspeed_espi_vw: failed to create sysfs device attributes.\n");
		return -1;
	}
	
	dev_set_drvdata(dev, espi_vw);

	espi_vw->irq = platform_get_irq(pdev, 0);
	if (espi_vw->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		return espi_vw->irq;
	}
	
	rc = devm_request_irq(&pdev->dev, espi_vw->irq, aspeed_espi_vw_irq, IRQF_SHARED,
				dev_name(&pdev->dev), espi_vw);

	if (rc) {
		printk("espi oob Unable to get IRQ \n");
		return rc;
	}

	rc = misc_register(&aspeed_espi_vw_misc);
	if (rc) {
		dev_err(dev, "Unable to register device\n");
		return rc;
	}

	pr_info("aspeed espi vw loaded \n");

	return 0;
}

static int aspeed_espi_vw_remove(struct platform_device *pdev)
{
//	struct aspeed_espi_vw *espi_vw = dev_get_drvdata(&pdev->dev);

	misc_deregister(&aspeed_espi_vw_misc);

	return 0;
}


static struct platform_driver aspeed_espi_vw_driver = {
	.driver = {
		.name           = "aspeed-espi-vw",
		.of_match_table = aspeed_espi_vw_match,
	},
	.probe  = aspeed_espi_vw_probe,
	.remove = aspeed_espi_vw_remove,
};
module_platform_driver(aspeed_espi_vw_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("Aspeed ESPI VW Driver");
