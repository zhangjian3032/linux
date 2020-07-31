/*
 * eSPI driver for the Aspeed SoC
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
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>

#include <linux/io.h>
#include <linux/module.h>

#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/reset.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/sysfs.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/regmap.h>


#include <dt-bindings/gpio/aspeed-gpio.h>
#include <linux/irqdomain.h>

#include "regs-aspeed-espi.h"

#define DEVICE_NAME "aspeed-espi"

/*************************************************************************************/
#define ESPIIOC_BASE       'E'

#define ASPEED_ESPI_RX_IOCSTS			_IOR(ESPIIOC_BASE, 0x0, unsigned int)
/*************************************************************************************/
#define MAX_XFER_BUFF_SIZE	0x80		//128
/*
ast2500 dma/pio mode only support 1 fifo 
ast2600 pio mode support 1 fifo 
ast2600 dma mode support 8 fifo 
*/

struct aspeed_espi_data {
	struct device			*dev;
	struct regmap			*map;	
	struct miscdevice 		miscdev;	
	
	int 					irq;					//LPC IRQ number
	int 					gpio_irq;					//GPIO IRQ number	
	int						espi_version;
	struct reset_control 	*reset;
	u32 					irq_sts;
	u32 					espi_bus_rest_ier;

	struct irq_domain *irq_domain;
	struct fasync_struct 	*async_queue;
};

static void aspeed_espi_ctrl_init(struct aspeed_espi_data *aspeed_espi)
{
	//ast2600 a0 workaround for oob free init before espi reset 
	if(aspeed_espi->espi_version == ESPI_AST2600)
		regmap_update_bits(aspeed_espi->map, ASPEED_ESPI_CTRL, 0xef, 0xef); 
	else
		regmap_update_bits(aspeed_espi->map, ASPEED_ESPI_CTRL, 0xff, 0xff); 

	//TODO for function interrpt type
	regmap_write(aspeed_espi->map, ASPEED_ESPI_SYS_INT_T0, 0);
	regmap_write(aspeed_espi->map, ASPEED_ESPI_SYS_INT_T1, 0);

	if (aspeed_espi->espi_version == ESPI_AST2500)
		regmap_write(aspeed_espi->map, ASPEED_ESPI_SYS_INT_T2, ESPI_HOST_RST_WARN | ESPI_OOB_RST_WARN);

	regmap_update_bits(aspeed_espi->map, ASPEED_ESPI_IER, BIT(31), BIT(31));
	regmap_write(aspeed_espi->map, ASPEED_ESPI_SYS_IER, 0xffffffff);

	//A1
	regmap_write(aspeed_espi->map, ASPEED_ESPI_SYS1_IER, 1);
	regmap_write(aspeed_espi->map, ASPEED_ESPI_SYS1_INT_T0, 1);
}

static irqreturn_t aspeed_espi_reset_isr(int this_irq, void *dev_id)
{
	u32 ctrl, sw_mode = 0;
	struct aspeed_espi_data *aspeed_espi = dev_id;
	unsigned int bus_irq;

	printk("aspeed_espi_reset_isr\n");

	regmap_read(aspeed_espi->map, ASPEED_ESPI_CTRL, &ctrl);
	sw_mode = ctrl & ESPI_CTRL_SW_FLASH_READ;

#if 0	//temporarily remove the scu reset
	//scu init
	reset_control_assert(aspeed_espi->reset);
	reset_control_deassert(aspeed_espi->reset);
#endif

	aspeed_espi_ctrl_init(aspeed_espi);

	regmap_update_bits(aspeed_espi->map, ASPEED_ESPI_CTRL, sw_mode, sw_mode); 

	//for all channel reset 
	bus_irq = irq_find_mapping(aspeed_espi->irq_domain, 4);
	generic_handle_irq(bus_irq);

	return IRQ_HANDLED;
}

static void aspeed_espi_handler(struct irq_desc *desc)
{
	struct aspeed_espi_data *aspeed_espi = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 status, ier;
	unsigned int bus_irq;

	chained_irq_enter(chip, desc);
	regmap_read(aspeed_espi->map, ASPEED_ESPI_ISR, &status);
	regmap_read(aspeed_espi->map, ASPEED_ESPI_IER, &ier);

	printk("isr status %x \n", status);

	if (status & (GENMASK(17, 16) | GENMASK(13, 10))) {
		bus_irq = irq_find_mapping(aspeed_espi->irq_domain, 0);
		generic_handle_irq(bus_irq);
	}

	if (status & (BIT(22) | GENMASK(9, 8))) {
		bus_irq = irq_find_mapping(aspeed_espi->irq_domain, 1);
		generic_handle_irq(bus_irq);
	}

	//OOB
	if (ier & (BIT(23) | BIT(20) | BIT(18) | BIT(14) | GENMASK(5, 4))) {
		if (status & (BIT(23) | BIT(20) | BIT(18) | BIT(14) | GENMASK(5, 4) | ESPI_ISR_HW_RESET)) {
			bus_irq = irq_find_mapping(aspeed_espi->irq_domain, 2);
			generic_handle_irq(bus_irq);
		}
	}
	
	if (status & (BIT(21) | BIT(19) | BIT(15) | GENMASK(7, 6))) {
		bus_irq = irq_find_mapping(aspeed_espi->irq_domain, 3);
		generic_handle_irq(bus_irq);
	}

	if (status & ESPI_ISR_HW_RESET) {
		printk("ESPI_ISR_HW_RESET \n");
		regmap_update_bits(aspeed_espi->map, ASPEED_ESPI_SYS_EVENT, ESPI_BOOT_STS | ESPI_BOOT_DWN, ESPI_BOOT_STS | ESPI_BOOT_DWN); 
		//6:flash ready ,4: oob ready , 0: perp ready
//		regmap_update_bits(aspeed_espi->map, ASPEED_ESPI_CTRL, BIT(4) | BIT(6), BIT(4) | BIT(6)); 
		regmap_write(aspeed_espi->map, ASPEED_ESPI_ISR, ESPI_ISR_HW_RESET);	
	}
	
	chained_irq_exit(chip, desc);
}

static void aspeed_espi_mask_irq(struct irq_data *data)
{
	struct aspeed_espi_data *aspeed_espi = irq_data_get_irq_chip_data(data);

	switch(data->hwirq) {
		case 0:	//peripheral channel
			regmap_update_bits(aspeed_espi->map, ASPEED_ESPI_IER, GENMASK(17, 16) | GENMASK(13, 10), 0);
			break;
		case 1:	//virtual wire
			regmap_update_bits(aspeed_espi->map, ASPEED_ESPI_IER, BIT(22) | GENMASK(9, 8), 0);
			break;
		case 2:	//oob
			regmap_update_bits(aspeed_espi->map, ASPEED_ESPI_IER, BIT(23) | BIT(20) | BIT(18) | BIT(14) | GENMASK(5, 4), 0);
			break;
		case 3: //flash
			regmap_update_bits(aspeed_espi->map, ASPEED_ESPI_IER, BIT(21) | BIT(19) | BIT(15) | GENMASK(7, 6), 0);
			break;		
		case 4: //espi channel reset
			aspeed_espi->espi_bus_rest_ier = 0;
			break;
	}
}

static void aspeed_espi_unmask_irq(struct irq_data *data)
{
	struct aspeed_espi_data *aspeed_espi = irq_data_get_irq_chip_data(data);

	switch(data->hwirq) {
		case 0:	//peripheral channel
			regmap_update_bits(aspeed_espi->map, ASPEED_ESPI_IER, GENMASK(17, 16) | GENMASK(13, 10), GENMASK(17, 16) | GENMASK(13, 10));
			break;
		case 1:	//virtual wire
			regmap_update_bits(aspeed_espi->map, ASPEED_ESPI_IER, BIT(22) | GENMASK(9, 8), BIT(22) | GENMASK(9, 8));
			break;
		case 2:	//oob
			regmap_update_bits(aspeed_espi->map, ASPEED_ESPI_IER, BIT(23) | BIT(20) | BIT(18) | BIT(14) | GENMASK(5, 4), BIT(23) | BIT(20) | BIT(18) | BIT(14) | GENMASK(5, 4));
			break;
		case 3: //flash
			regmap_update_bits(aspeed_espi->map, ASPEED_ESPI_IER, BIT(21) | BIT(19) | BIT(15) | GENMASK(7, 6), BIT(21) | BIT(19) | BIT(15) | GENMASK(7, 6));
			break;
		case 4: //espi bus reset
			aspeed_espi->espi_bus_rest_ier = 1;
			break;
	}
}

struct irq_chip aspeed_espi_irq_chip = {
	.name		= "espi-irq",
	.irq_mask	= aspeed_espi_mask_irq,
	.irq_unmask	= aspeed_espi_unmask_irq,
};

static int aspeed_espi_map_irq_domain(struct irq_domain *domain,
				  unsigned int irq, irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &aspeed_espi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops aspeed_espi_irq_domain_ops = {
	.map = aspeed_espi_map_irq_domain,
};

static long
espi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct miscdevice *c = file->private_data;
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(c->this_device);
	struct aspeed_espi_xfer xfer;

	if (copy_from_user(&xfer, (void*)arg, sizeof(struct aspeed_espi_xfer)))
		return -EFAULT; 

	switch (cmd) {
	case ASPEED_ESPI_RX_IOCSTS:
		ret = __put_user(aspeed_espi->irq_sts, (u32 __user *)arg);
		aspeed_espi->irq_sts = 0;
		break;

	default:
		printk("ERROR \n");
		return -ENOTTY;
	}

	return ret;
}

static int espi_fasync(int fd, struct file *file, int mode)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(c->this_device);
	return fasync_helper(fd, file, mode, &aspeed_espi->async_queue);
}

static ssize_t show_op_freq(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	u32 config;
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);
	
	regmap_read(aspeed_espi->map, ASPEED_ESPI_GCAP_CONFIG, &config);

	switch (GET_GCAP_OP_FREQ(config)) {
	case 0:
		return sprintf(buf, "20Mhz\n");
		break;
	case 1:
		return sprintf(buf, "25Mhz\n");
		break;
	case 2:
		return sprintf(buf, "33Mhz\n");
		break;
	case 3:
		return sprintf(buf, "50Mhz\n");
		break;
	case 4:
		return sprintf(buf, "66Mhz\n");
		break;
	}
	return sprintf(buf, "Unknow\n");
}

static DEVICE_ATTR(op_freq, S_IRUGO, show_op_freq, NULL);

static ssize_t show_io_mode(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	u32 config;
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);

	regmap_read(aspeed_espi->map, ASPEED_ESPI_GCAP_CONFIG, &config);

	switch (GET_GCAP_IO_MODE(config)) {
	case 0:
		return sprintf(buf, "Single IO\n");
		break;
	case 1:
		return sprintf(buf, "Dual IO\n");
		break;
	case 2:
		return sprintf(buf, "Qual IO\n");
		break;
	}
	return sprintf(buf, "Unknow\n");
}

static DEVICE_ATTR(io_mode, S_IRUGO, show_io_mode, NULL);

static struct attribute *espi_sysfs_entries[] = {
	&dev_attr_io_mode.attr,
	&dev_attr_op_freq.attr,
	NULL
};

static struct attribute_group espi_attribute_group = {
	.attrs = espi_sysfs_entries,
};

static const struct file_operations aspeed_espi_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl		= espi_ioctl,
	.fasync			= espi_fasync,
};

static const struct of_device_id aspeed_espi_of_matches[] = {
	{ .compatible = "aspeed,ast2500-espi", .data = (void *) ESPI_AST2500, },
	{ .compatible = "aspeed,ast2600-espi", .data = (void *) ESPI_AST2600, },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_espi_of_matches);


static int aspeed_espi_probe(struct platform_device *pdev)
{
	static struct aspeed_espi_data *aspeed_espi;
	struct device *dev = &pdev->dev;	
	u32 delay_timing = 0;
	int ret = 0;
	unsigned long espi_version;
	u32 reg;
	struct regmap *scu_regmap;

	espi_version = (unsigned long)of_device_get_match_data(dev);

	scu_regmap = syscon_regmap_lookup_by_compatible("aspeed,aspeed-scu");
	if (IS_ERR_OR_NULL(scu_regmap)) {
		dev_err(dev, "failed to find SCU regmap\n");
		return PTR_ERR(scu_regmap);
	}

	/* abort eSPI probe if LPC mode is selected */
	if (espi_version == ESPI_AST2500) {
		regmap_read(scu_regmap, 0x70, &reg);
		if ((reg & 0x2000000) == 0)
			return -EPERM;
	}
	else if (espi_version == ESPI_AST2600) {
		regmap_read(scu_regmap, 0x510, &reg);
		if (reg & 0x40)
			return -EPERM;
	}
	else {
		dev_err(dev, "unknown eSPI version\n");
		return -EINVAL;
	}

	aspeed_espi = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_espi_data), GFP_KERNEL);
	if (aspeed_espi == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	aspeed_espi->espi_version = espi_version;
	aspeed_espi->dev = &pdev->dev;
	aspeed_espi->map = syscon_node_to_regmap(pdev->dev.of_node);
	if (IS_ERR(aspeed_espi->map)) {
		printk("aspeed_espi->map ERROR \n");
		return PTR_ERR(aspeed_espi->map);	
	}

	if(delay_timing)
		regmap_update_bits(aspeed_espi->map, ASPEED_ESPI_CTRL2, GENMASK(19, 16), BIT(19) | (delay_timing << 16)); 
	
	aspeed_espi->irq = platform_get_irq(pdev, 0);
	if (aspeed_espi->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		return aspeed_espi->irq;
	}

	aspeed_espi->irq_domain = irq_domain_add_linear(
					pdev->dev.of_node, 5,
					&aspeed_espi_irq_domain_ops, aspeed_espi);
	if (!aspeed_espi->irq_domain) {
		ret = -ENOMEM;

	}

	irq_set_chained_handler_and_data(aspeed_espi->irq,
					 aspeed_espi_handler, aspeed_espi);

	aspeed_espi->reset = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(aspeed_espi->reset)) {
		dev_err(&pdev->dev, "can't get espi reset\n");
		return PTR_ERR(aspeed_espi->reset);
	}

	aspeed_espi->gpio_irq = platform_get_irq(pdev, 1);

	ret = devm_request_irq(&pdev->dev, aspeed_espi->gpio_irq, aspeed_espi_reset_isr,
						   0, dev_name(&pdev->dev), aspeed_espi);
	if (ret) {
		printk("AST ESPI Unable to get GPIO IRQ %d\n", ret);
		return ret;
	}

	aspeed_espi_ctrl_init(aspeed_espi);

	aspeed_espi->miscdev.minor = MISC_DYNAMIC_MINOR;
	aspeed_espi->miscdev.name = DEVICE_NAME;
	aspeed_espi->miscdev.fops = &aspeed_espi_fops;
	aspeed_espi->miscdev.parent = dev;
	ret = misc_register(&aspeed_espi->miscdev);
	if (ret) {
		printk(KERN_ERR "ESPI : failed misc_register\n");
		return ret;
	}

	dev_set_drvdata(dev, aspeed_espi);

	ret = sysfs_create_group(&pdev->dev.kobj, &espi_attribute_group);
	if (ret) {
		printk(KERN_ERR "aspeed_espi: failed to create sysfs device attributes.\n");
		return -1;
	}

	printk(KERN_INFO "aspeed_espi: driver successfully loaded.\n");

	return 0;
}

static int aspeed_espi_remove(struct platform_device *pdev)
{
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(&pdev->dev);

	misc_deregister(&aspeed_espi->miscdev);
	free_irq(aspeed_espi->gpio_irq, aspeed_espi);
	free_irq(aspeed_espi->irq, aspeed_espi);
	sysfs_remove_group(&pdev->dev.kobj, &espi_attribute_group);

	kfree(aspeed_espi);
	return 0;
}

static struct platform_driver aspeed_espi_driver = {
	.probe		= aspeed_espi_probe,
	.remove		= aspeed_espi_remove,
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = aspeed_espi_of_matches,
	},
};

module_platform_driver(aspeed_espi_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST eSPI driver");
MODULE_LICENSE("GPL");
