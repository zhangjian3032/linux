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

#define DEVICE_NAME     "espi-flash"

struct espi_flash_xfer {
	/* [23:12]: len, [11:8]: tag, [7:0]: cycle type */
	unsigned int header;
	unsigned char xfer_buf[68];
};

#define FLASH_READ			0x00
#define FLASH_WRITE			0x01
#define FLASH_ERASE			0x02
#define FLASH_CMPLT_W_DATA_MIDDLE	0x09
#define FLASH_CMPLT_W_DATA_FIRST	0x0b
#define FLASH_CMPLT_W_DATA_LAST		0x0d
#define FLASH_CMPLT_W_DATA_ONLY		0x0f

struct aspeed_espi_flash {
	struct regmap *map;
	struct miscdevice miscdev;

	struct bin_attribute bin;

	u8 *rx_buff;
	dma_addr_t rx_dma_addr;

	u8 *tx_buff;
	dma_addr_t tx_dma_addr;
	
	int irq;
	int rest_irq;
	
	int espi_version;
	int dma_mode;
	int dma_rx;

	u8 rx_ready;
	wait_queue_head_t wq;
};

static irqreturn_t aspeed_espi_flash_reset_irq(int irq, void *arg)
{
	struct aspeed_espi_flash *espi_flash = arg;

	if(espi_flash->dma_mode) {
		espi_flash->dma_rx = 0;
		regmap_write(espi_flash->map, ASPEED_ESPI_FLASH_RX_DMA, espi_flash->rx_dma_addr);
		regmap_write(espi_flash->map, ASPEED_ESPI_FLASH_TX_DMA, espi_flash->tx_dma_addr);
		regmap_update_bits(espi_flash->map, ASPEED_ESPI_CTRL, GENMASK(23, 22), ESPI_CTRL_FLASH_RX_DMA | ESPI_CTRL_FLASH_TX_DMA);
	}
	return IRQ_HANDLED;
}

static irqreturn_t aspeed_espi_flash_irq(int irq, void *arg)
{
	u32 sts, flash_isr;
	struct aspeed_espi_flash *espi_flash = arg;

	regmap_read(espi_flash->map, ASPEED_ESPI_ISR, &sts);
	printk("aspeed_espi_flash_irq %x\n", sts);

	flash_isr = sts & (ESPI_ISR_FLASH_TX_ERR | ESPI_ISR_FLASH_TX_ABORT | ESPI_ISR_FLASH_RX_ABORT | ESPI_ISR_FLASH_TX_COMP | ESPI_ISR_FLASH_RX_COMP);
	if (flash_isr) {
		if (flash_isr & ESPI_ISR_FLASH_TX_ERR) {
			printk("ESPI_ISR_FLASH_TX_ERR \n");
		}
	
		if (flash_isr & ESPI_ISR_FLASH_TX_ABORT) {
			printk("ESPI_ISR_FLASH_TX_ABORT \n");
		}
	
		if (flash_isr & ESPI_ISR_FLASH_RX_ABORT) {
			printk("ESPI_ISR_FLASH_RX_ABORT \n");
		}

		if (flash_isr & ESPI_ISR_FLASH_TX_COMP) {
			printk("ESPI_ISR_FLASH_TX_COMP \n");
		}
	
		if (flash_isr & ESPI_ISR_FLASH_RX_COMP) {
			printk("ESPI_ISR_FLASH_RX_COMP \n");

			espi_flash->rx_ready = 1;
			wake_up_interruptible(&espi_flash->wq);
		}

		regmap_write(espi_flash->map, ASPEED_ESPI_ISR, flash_isr);
		return IRQ_HANDLED;		
	} else 
		return IRQ_NONE;

}

static ssize_t show_sw_flash_read(struct device *dev,
								  struct device_attribute *attr, char *buf)
{
	u32 ctrl;
	struct aspeed_espi_flash *espi_flash = dev_get_drvdata(dev);

	regmap_read(espi_flash->map, ASPEED_ESPI_CTRL, &ctrl);

	if(espi_flash->espi_version == ESPI_AST2500)
		return sprintf(buf, "%s Mode\n", ctrl & ESPI_CTRL_SW_FLASH_READ ? "1:SW" : "0:HW");
	else
		return sprintf(buf, "%d (0: mix mode, 1: sw mode, 2:hw mode) \n", (ctrl >> 10) & 0x3);
}

static ssize_t store_sw_flash_read(struct device *dev,
								   struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_espi_flash *espi_flash = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);

	if(espi_flash->espi_version == ESPI_AST2500) {
		if (val)
			regmap_update_bits(espi_flash->map, ASPEED_ESPI_CTRL, ESPI_CTRL_SW_FLASH_READ, ESPI_CTRL_SW_FLASH_READ);
		else
			regmap_update_bits(espi_flash->map, ASPEED_ESPI_CTRL, ESPI_CTRL_SW_FLASH_READ, 0);
	} else {
		regmap_update_bits(espi_flash->map, ASPEED_ESPI_CTRL, GENMASK(11, 10), val << 10);
	}
	return count;
}

static DEVICE_ATTR(sw_flash_read, S_IRUGO | S_IWUSR, show_sw_flash_read, store_sw_flash_read);

static struct attribute *espi_flash_sysfs_entries[] = {
	&dev_attr_sw_flash_read.attr,
	NULL
};

static struct attribute_group espi_flash_attribute_group = {
	.attrs = espi_flash_sysfs_entries,
};

static ssize_t espi_flash_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	int i, rc;
	u32 rx_ctrl, rx_data;
	struct espi_flash_xfer xfer;
	size_t xfer_len;

	struct aspeed_espi_flash *espi_flash = container_of(
			file->private_data,
			struct aspeed_espi_flash,
			miscdev);

	if (count != sizeof(xfer))
		return -EINVAL;

	if (!espi_flash->rx_ready) {
		if (file->f_flags & O_NONBLOCK)
			return -ENODATA;
		rc = wait_event_interruptible(espi_flash->wq,
				espi_flash->rx_ready);
		if (rc == -ERESTARTSYS)
			return -EINTR;
	}

	regmap_read(espi_flash->map, ASPEED_ESPI_FLASH_RX_CTRL, &rx_ctrl);
	printk("cycle type=0x%02x, tag=0x%02x, len=%d\n",
			ESPI_GET_CYCLE_TYPE(rx_ctrl),
			ESPI_GET_TAG(rx_ctrl),
			ESPI_GET_LEN(rx_ctrl));

	xfer.header = rx_ctrl & ~ESPI_TRIGGER_PACKAGE;

	switch (ESPI_GET_CYCLE_TYPE(rx_ctrl)) {
		case FLASH_READ:
		case FLASH_ERASE:
			xfer_len = 4;
			break;
		case FLASH_WRITE:
			xfer_len = ESPI_GET_LEN(rx_ctrl) + 4;
			break;
		case FLASH_CMPLT_W_DATA_MIDDLE:
		case FLASH_CMPLT_W_DATA_FIRST:
		case FLASH_CMPLT_W_DATA_LAST:
		case FLASH_CMPLT_W_DATA_ONLY:
			xfer_len = ESPI_GET_LEN(rx_ctrl);
			break;
		default:
			xfer_len = 0;
			break;
	}

	if (espi_flash->dma_mode)
		memcpy(xfer.xfer_buf, espi_flash->rx_buff, xfer_len);
	else {
		for (i = 0; i < xfer_len; ++i) {
			regmap_read(espi_flash->map,
					ASPEED_ESPI_FLASH_RX_DATA,
					&rx_data);
			xfer.xfer_buf[i] = (unsigned char)((rx_data & 0xff));
		}
	}

	if (copy_to_user(buffer, &xfer, sizeof(xfer)))
		return -EFAULT;

	espi_flash->rx_ready = 0;
	regmap_write(espi_flash->map, ASPEED_ESPI_FLASH_RX_CTRL, ESPI_TRIGGER_PACKAGE);

	return count;
}

static ssize_t espi_flash_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int i;
	u32 tx_ctrl;
	struct espi_flash_xfer xfer;
	size_t xfer_len;

	struct aspeed_espi_flash *espi_flash = container_of(
			file->private_data,
			struct aspeed_espi_flash,
			miscdev);

	if (count != sizeof(xfer))
		return -EINVAL;

	regmap_read(espi_flash->map, ASPEED_ESPI_FLASH_TX_CTRL, &tx_ctrl);
	if (tx_ctrl & ESPI_TRIGGER_PACKAGE)
		return -EBUSY;

	if (copy_from_user(&xfer, buffer, sizeof(xfer)))
		return -EFAULT;

	printk("cycle type=0x%02x, tag=0x%02x, len=%d\n",
			ESPI_GET_CYCLE_TYPE(xfer.header),
			ESPI_GET_TAG(xfer.header),
			ESPI_GET_LEN(xfer.header));

	switch (ESPI_GET_CYCLE_TYPE(xfer.header)) {
		case FLASH_READ:
		case FLASH_ERASE:
			xfer_len = 4;
			break;
		case FLASH_WRITE:
			xfer_len = ESPI_GET_LEN(xfer.header) + 4;
			break;
		case FLASH_CMPLT_W_DATA_MIDDLE:
		case FLASH_CMPLT_W_DATA_FIRST:
		case FLASH_CMPLT_W_DATA_LAST:
		case FLASH_CMPLT_W_DATA_ONLY:
			xfer_len = ESPI_GET_LEN(xfer.header);
			break;
		default:
			xfer_len = 0;
			break;
	}

	if (espi_flash->dma_mode)
		memcpy(espi_flash->tx_buff, xfer.xfer_buf, xfer_len);
	else
		for (i = 0; i < xfer_len; ++i)
			regmap_write(espi_flash->map,
					ASPEED_ESPI_FLASH_TX_DATA,
					xfer.xfer_buf[i]);

	regmap_write(espi_flash->map, ASPEED_ESPI_FLASH_TX_CTRL, ESPI_TRIGGER_PACKAGE | xfer.header);

	return count;
}

static __poll_t espi_flash_poll(struct file *file, struct poll_table_struct *pt)
{
	struct aspeed_espi_flash *espi_flash = container_of(
			file->private_data,
			struct aspeed_espi_flash,
			miscdev);

	poll_wait(file, &espi_flash->wq, pt);
	return (espi_flash->rx_ready)? POLLIN : 0;
}

static const struct file_operations aspeed_espi_flash_fops = {
	.owner = THIS_MODULE,
	.read = espi_flash_read,
	.write = espi_flash_write,
	.poll = espi_flash_poll,
};

static const struct of_device_id aspeed_espi_flash_match[] = {
	{ .compatible = "aspeed,ast2600-espi-flash", .data = (void *) ESPI_AST2600, },
	{ .compatible = "aspeed,ast2500-espi-flash", .data = (void *) ESPI_AST2500, },
	{ }
};
MODULE_DEVICE_TABLE(of, aspeed_espi_flash_match);

static int aspeed_espi_flash_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aspeed_espi_flash *espi_flash;
	const struct of_device_id *dev_id;	
	int rc;

	espi_flash = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_espi_flash), GFP_KERNEL);
	if (!espi_flash)
		return -ENOMEM;

	dev_id = of_match_device(aspeed_espi_flash_match, &pdev->dev);
	if (!dev_id)
		return -EINVAL;

	espi_flash->espi_version = (unsigned long)dev_id->data;
	espi_flash->rx_ready = 0;

	if (of_property_read_bool(pdev->dev.of_node, "dma-mode"))
		espi_flash->dma_mode = 1;

	espi_flash->map = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(espi_flash->map)) {
		dev_err(dev, "Couldn't get regmap\n");
		return -ENODEV;
	}

	espi_flash->irq = platform_get_irq(pdev, 0);
	if (espi_flash->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		return espi_flash->irq;
	}

	espi_flash->rest_irq = platform_get_irq(pdev, 1);
	if (espi_flash->rest_irq < 0) {
		dev_err(&pdev->dev, "no rest_irq specified\n");
		return espi_flash->rest_irq;
	}

	rc = sysfs_create_group(&pdev->dev.kobj, &espi_flash_attribute_group);
	if (rc) {
		printk(KERN_ERR "aspeed_espi_flash: failed to create sysfs device attributes.\n");
		return -1;
	}

	if(espi_flash->dma_mode) {
		espi_flash->dma_rx = 0;
		espi_flash->tx_buff = dma_alloc_coherent(NULL,
									  (MAX_XFER_BUFF_SIZE * 2),
									  &espi_flash->tx_dma_addr, GFP_KERNEL);
		espi_flash->rx_buff = espi_flash->tx_buff + MAX_XFER_BUFF_SIZE;
		espi_flash->rx_dma_addr = espi_flash->tx_dma_addr + MAX_XFER_BUFF_SIZE;

		regmap_write(espi_flash->map, ASPEED_ESPI_FLASH_RX_DMA, espi_flash->rx_dma_addr);
		regmap_write(espi_flash->map, ASPEED_ESPI_FLASH_TX_DMA, espi_flash->rx_dma_addr);

		regmap_update_bits(espi_flash->map, ASPEED_ESPI_CTRL, GENMASK(23, 22), ESPI_CTRL_FLASH_RX_DMA | ESPI_CTRL_FLASH_TX_DMA);	
	} else {
		espi_flash->tx_buff = kzalloc(MAX_XFER_BUFF_SIZE * 2, GFP_KERNEL);
		espi_flash->rx_buff = espi_flash->tx_buff + MAX_XFER_BUFF_SIZE;
	}

	rc = devm_request_irq(&pdev->dev, espi_flash->irq, aspeed_espi_flash_irq, IRQF_SHARED,
				dev_name(&pdev->dev), espi_flash);
	if (rc) {
		printk("espi flash Unable to get IRQ \n");
		return rc;
	}

	rc = devm_request_irq(&pdev->dev, espi_flash->rest_irq, aspeed_espi_flash_reset_irq, IRQF_SHARED,
				dev_name(&pdev->dev), espi_flash);
	if (rc) {
		printk("espi flash Unable to get IRQ \n");
		return rc;
	}

	init_waitqueue_head(&espi_flash->wq);

	espi_flash->miscdev.minor = MISC_DYNAMIC_MINOR;
	espi_flash->miscdev.parent = dev;
	espi_flash->miscdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s", DEVICE_NAME);
	espi_flash->miscdev.fops = &aspeed_espi_flash_fops;
	rc = misc_register(&espi_flash->miscdev);
	if (rc) {
		dev_err(dev, "Unable to register device\n");
		return rc;
	}

	platform_set_drvdata(pdev, espi_flash);

	pr_info("aspeed espi-flash loaded \n");

	return 0;
}

static int aspeed_espi_flash_remove(struct platform_device *pdev)
{
	struct aspeed_espi_flash *espi_flash = dev_get_drvdata(&pdev->dev);

	misc_deregister(&espi_flash->miscdev);
	return 0;
}

static struct platform_driver aspeed_espi_flash_driver = {
	.driver = {
		.name           = DEVICE_NAME,
		.of_match_table = aspeed_espi_flash_match,
	},
	.probe  = aspeed_espi_flash_probe,
	.remove = aspeed_espi_flash_remove,
};
module_platform_driver(aspeed_espi_flash_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("Aspeed ESPI flash driver");
