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

struct aspeed_espi_flash {
	struct regmap 			*map;
	struct miscdevice       miscdev;

	struct bin_attribute	bin;
	struct kernfs_node		*kn;	
	struct espi_ch_data		flash_rx_channel;
	struct espi_ch_data		flash_tx_channel;
	
	int 					irq;					//LPC IRQ number	
	int 					rest_irq;					//espi reset irq	
	
	int						espi_version;
	int						dma_mode;		/* o:disable , 1:enable */	
};

//buf[0] : cycle type, buf[1] : tag, buf[2] : len, buf [2 ~ len] : data
static ssize_t aspeed_flash_channel_rx(struct file *filp, struct kobject *kobj,
                struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	u32 rx_ctrl, rx_data, rx_len;
	int i = 0;
	struct aspeed_espi_flash *espi_flash = dev_get_drvdata(container_of(kobj, struct device, kobj));
	regmap_read(espi_flash->map, ASPEED_ESPI_FLASH_RX_CTRL, &rx_ctrl);

	if(rx_ctrl & ESPI_TRIGGER_PACKAGE) {
		printk("aspeed_flash_channel_rx : cycle type = %x , tag = %x, len = %d byte \n", 
				ESPI_GET_CYCLE_TYPE(rx_ctrl), ESPI_GET_TAG(rx_ctrl), ESPI_GET_LEN(rx_ctrl));
		if ((ESPI_GET_CYCLE_TYPE(rx_ctrl) == 0x00) || (ESPI_GET_CYCLE_TYPE(rx_ctrl) == 0x02))
			rx_len = 4;
		else if (ESPI_GET_CYCLE_TYPE(rx_ctrl) == 0x01)
			rx_len = ESPI_GET_LEN(rx_ctrl) + 4;
		else if ((ESPI_GET_CYCLE_TYPE(rx_ctrl) & 0x09) == 0x09)
			rx_len = ESPI_GET_LEN(rx_ctrl);
		else
			rx_len = 0;

		buf[0] = ESPI_GET_CYCLE_TYPE(rx_ctrl); 
		buf[1] = ESPI_GET_TAG(rx_ctrl);
		buf[2] = ESPI_GET_LEN(rx_ctrl);
		if(espi_flash->dma_mode) {
			memcpy(&buf[2], espi_flash->flash_rx_channel.buff, rx_len);
		} else {
			//PIO mode
			for (i = 0; i < rx_len; i++) {
				regmap_read(espi_flash->map, ASPEED_ESPI_FLASH_RX_DATA, &rx_data);
				buf[i + 2] = rx_data;
			}
		}
		regmap_write(espi_flash->map, ASPEED_ESPI_FLASH_RX_CTRL, ESPI_TRIGGER_PACKAGE);
		return count;
	} else 
		return 0;
}

//buf[0] : cycle type, buf[1] : tag, buf[2] : len, buf [3 ~ len] : data
static ssize_t aspeed_flash_channel_tx(struct file *filp, struct kobject *kobj,
                struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	int i = 0;
	u32 ch_sts;	
	struct aspeed_espi_flash *espi_flash = dev_get_drvdata(container_of(kobj, struct device, kobj));
	regmap_read(espi_flash->map, ASPEED_ESPI_CH3CAP_CONFIG, &ch_sts);
	
	if(!(ch_sts & BIT(0))) {
		printk("channel not enable \n");
		return -1;
	}

	if(count > MAX_XFER_BUFF_SIZE)
		return 0;
	
	if(espi_flash->dma_mode) {
		memcpy(espi_flash->flash_tx_channel.buff, &buf[3], count);
	} else {
		for (i = 0; i < count; i++)
			regmap_write(espi_flash->map, ASPEED_ESPI_FLASH_TX_DATA, buf[i + 3]);
	}
	regmap_write(espi_flash->map, ASPEED_ESPI_FLASH_TX_CTRL, ESPI_TRIGGER_PACKAGE | (buf[2] << 12) | (buf[1] << 8) | buf[0]);
	return count;
}

static irqreturn_t aspeed_espi_flash_reset_irq(int irq, void *arg)
{
	struct aspeed_espi_flash *espi_flash = arg;

	if(espi_flash->dma_mode) {
		regmap_write(espi_flash->map, ASPEED_ESPI_FLASH_RX_DMA, espi_flash->flash_rx_channel.dma_addr);
		regmap_write(espi_flash->map, ASPEED_ESPI_FLASH_TX_DMA, espi_flash->flash_tx_channel.dma_addr);
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

	if (of_property_read_bool(pdev->dev.of_node, "dma-mode"))
		espi_flash->dma_mode = 1;

	espi_flash->map = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(espi_flash->map)) {
		dev_err(dev, "Couldn't get regmap\n");
		return -ENODEV;
	}

	sysfs_bin_attr_init(&espi_flash->bin);
	espi_flash->bin.attr.name = DEVICE_NAME;
	espi_flash->bin.attr.mode = S_IRUSR | S_IWUSR;
	espi_flash->bin.read = aspeed_flash_channel_rx;
	espi_flash->bin.write = aspeed_flash_channel_tx;
	espi_flash->bin.size = MAX_XFER_BUFF_SIZE;

	rc = sysfs_create_bin_file(&pdev->dev.kobj, &espi_flash->bin);
	if (rc) {
		printk("error for bin file ");
		return rc;
	}

	espi_flash->kn = kernfs_find_and_get(dev->kobj.sd, espi_flash->bin.attr.name);
	if (!espi_flash->kn) {
		sysfs_remove_bin_file(&dev->kobj, &espi_flash->bin);
		return -EFAULT;
	}

	rc = sysfs_create_group(&pdev->dev.kobj, &espi_flash_attribute_group);
	if (rc) {
		printk(KERN_ERR "aspeed_espi_flash: failed to create sysfs device attributes.\n");
		return -1;
	}

	dev_set_drvdata(dev, espi_flash);

	if(espi_flash->dma_mode) {
		espi_flash->flash_rx_channel.buff = dma_alloc_coherent(NULL,
									  (MAX_XFER_BUFF_SIZE * 2),
									  &espi_flash->flash_rx_channel.dma_addr, GFP_KERNEL);
		espi_flash->flash_tx_channel.buff = espi_flash->flash_rx_channel.buff + MAX_XFER_BUFF_SIZE;
		espi_flash->flash_tx_channel.dma_addr = espi_flash->flash_rx_channel.dma_addr + MAX_XFER_BUFF_SIZE;

		regmap_write(espi_flash->map, ASPEED_ESPI_FLASH_RX_DMA, espi_flash->flash_rx_channel.dma_addr);
		regmap_write(espi_flash->map, ASPEED_ESPI_FLASH_TX_DMA, espi_flash->flash_tx_channel.dma_addr);

		regmap_update_bits(espi_flash->map, ASPEED_ESPI_CTRL, GENMASK(23, 22), ESPI_CTRL_FLASH_RX_DMA | ESPI_CTRL_FLASH_TX_DMA);	
	} else {
		// non-dma mode 
		espi_flash->flash_rx_channel.buff = kzalloc(MAX_XFER_BUFF_SIZE * 2, GFP_KERNEL);
		espi_flash->flash_tx_channel.buff = espi_flash->flash_rx_channel.buff + MAX_XFER_BUFF_SIZE;
	}

	espi_flash->irq = platform_get_irq(pdev, 0);
	if (espi_flash->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		return espi_flash->irq;
	}
	rc = devm_request_irq(&pdev->dev, espi_flash->irq, aspeed_espi_flash_irq, IRQF_SHARED,
				dev_name(&pdev->dev), espi_flash);
	if (rc) {
		printk("espi flash Unable to get IRQ \n");
		return rc;
	}

	espi_flash->rest_irq = platform_get_irq(pdev, 1);
	if (espi_flash->rest_irq < 0) {
		dev_err(&pdev->dev, "no rest_irq specified\n");
		return espi_flash->rest_irq;
	}
	rc = devm_request_irq(&pdev->dev, espi_flash->rest_irq, aspeed_espi_flash_reset_irq, IRQF_SHARED,
				dev_name(&pdev->dev), espi_flash);
	if (rc) {
		printk("espi flash Unable to get IRQ \n");
		return rc;
	}

	espi_flash->miscdev.minor = MISC_DYNAMIC_MINOR;
	espi_flash->miscdev.name = DEVICE_NAME;
//	espi_flash->miscdev.fops = &aspeed_espi_flash_fops;
	espi_flash->miscdev.parent = dev;
	rc = misc_register(&espi_flash->miscdev);
	if (rc) {
		dev_err(dev, "Unable to register device\n");
		return rc;
	}

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
