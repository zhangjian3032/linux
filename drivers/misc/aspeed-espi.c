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
/*************************************************************************************/
#define ASPEED_ESPI_DEBUG

#ifdef ASPEED_ESPI_DEBUG
#define ESPI_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define ESPI_DBUG(fmt, args...)
#endif

struct aspeed_espi_data {
	struct device			*dev;
//	void __iomem			*reg_base;			/* virtual */
	
	struct regmap		*map;	
	
	int 					irq;					//LPC IRQ number
	int 					gpio_irq;					//GPIO IRQ number	
	int						espi_version;
	int						dma_mode;		/* o:disable , 1:enable */
	struct reset_control 	*reset;
	u32 					irq_sts;
	bool 					is_open;

	struct irq_domain *irq_domain;
	
	
	struct fasync_struct 	*async_queue;
};

/******************************************************************************/
static DEFINE_SPINLOCK(espi_state_lock);
/******************************************************************************/

static inline u32
aspeed_espi_read(struct aspeed_espi_data *aspeed_espi, u32 reg)
{
	u32 val;
	int rc;	
#if 0
	val = readl(aspeed_espi->reg_base + reg);
	ESPI_DBUG("aspeed_espi_read : reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
#else
	rc = regmap_read(aspeed_espi->map, reg, &val);
	WARN(rc != 0, "regmap_read() failed: %d\n", rc);

	return rc == 0 ? val : 0;
#endif
}

static inline void
aspeed_espi_write(struct aspeed_espi_data *aspeed_espi, u32 val, u32 reg)
{
	int rc;

	rc = regmap_write(aspeed_espi->map, reg, val);
	WARN(rc != 0, "regmap_write() failed: %d\n", rc);
}

/******************************************************************************/

static void aspeed_espi_ctrl_init(struct aspeed_espi_data *aspeed_espi)
{

	//ast2600 a0 workaround for oob free init before espi reset 
	if(aspeed_espi->espi_version == 6)
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_CTRL) | 0xef, ASPEED_ESPI_CTRL);
	else
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_CTRL) | 0xff, ASPEED_ESPI_CTRL);

	//TODO for function interrpt type
	aspeed_espi_write(aspeed_espi, 0, ASPEED_ESPI_SYS_INT_T0);
	aspeed_espi_write(aspeed_espi, 0, ASPEED_ESPI_SYS_INT_T1);

	if (aspeed_espi->espi_version == 5)
		aspeed_espi_write(aspeed_espi, ESPI_HOST_RST_WARN | ESPI_OOB_RST_WARN, ASPEED_ESPI_SYS_INT_T2);

	aspeed_espi_write(aspeed_espi, 0xffffffff, ASPEED_ESPI_IER);
	aspeed_espi_write(aspeed_espi, 0xffffffff, ASPEED_ESPI_SYS_IER);

	//A1
	aspeed_espi_write(aspeed_espi, 0x1, ASPEED_ESPI_SYS1_IER);
	aspeed_espi_write(aspeed_espi, 0x1, ASPEED_ESPI_SYS1_INT_T0);



}

static irqreturn_t aspeed_espi_reset_isr(int this_irq, void *dev_id)
{
	u32 sw_mode = 0;
	struct aspeed_espi_data *aspeed_espi = dev_id;

	ESPI_DBUG("aspeed_espi_reset_isr\n");

	sw_mode = aspeed_espi_read(aspeed_espi, ASPEED_ESPI_CTRL) & ESPI_CTRL_SW_FLASH_READ;

#if 0	//temporarily remove the scu reset
	//scu init
	reset_control_assert(aspeed_espi->reset);
	reset_control_deassert(aspeed_espi->reset);
#endif

	//ast2600 a0 workaround for oob free init before espi reset 
	if(aspeed_espi->espi_version == 6)
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_CTRL) | ESPI_CTRL_OOB_FW_RDY, ASPEED_ESPI_CTRL);

	aspeed_espi_ctrl_init(aspeed_espi);

	aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_CTRL) | sw_mode, ASPEED_ESPI_CTRL);

	return IRQ_HANDLED;
}
static irqreturn_t aspeed_espi_isr(int this_irq, void *dev_id)
{
	struct aspeed_espi_data *aspeed_espi = dev_id;
	u32 sts = aspeed_espi_read(aspeed_espi, ASPEED_ESPI_ISR);
	ESPI_DBUG("sts : %x\n", sts);

	if (sts & ESPI_ISR_HW_RESET) {
		printk("ESPI_ISR_HW_RESET \n");
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) | ESPI_BOOT_STS | ESPI_BOOT_DWN, ASPEED_ESPI_SYS_EVENT);

		//6:flash ready ,4: oob ready , 0: perp ready
//		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_CTRL) |BIT(4) | BIT(6), ASPEED_ESPI_CTRL);
		aspeed_espi_write(aspeed_espi, ESPI_ISR_HW_RESET, ASPEED_ESPI_ISR);
	}

	aspeed_espi->irq_sts |= sts;
	if (aspeed_espi->async_queue)
		kill_fasync(&aspeed_espi->async_queue, SIGIO, POLL_IN);

	return IRQ_HANDLED;
}

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
		ESPI_DBUG("ERROR \n");
		return -ENOTTY;
	}

	return ret;
}

static int espi_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(c->this_device);

	ESPI_DBUG("\n");
	spin_lock(&espi_state_lock);

	if (aspeed_espi->is_open) {
		spin_unlock(&espi_state_lock);
		return -EBUSY;
	}

	aspeed_espi->is_open = true;

	spin_unlock(&espi_state_lock);

	return 0;
}

static int espi_fasync(int fd, struct file *file, int mode)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(c->this_device);
	return fasync_helper(fd, file, mode, &aspeed_espi->async_queue);
}

static int espi_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(c->this_device);

	ESPI_DBUG("\n");
	spin_lock(&espi_state_lock);

	aspeed_espi->is_open = false;
//	espi_fasync(-1, file, 0);
	spin_unlock(&espi_state_lock);

	return 0;
}

static ssize_t show_fatal_err(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) & ESPI_FATEL_ERR ? "0" : "1");
}

static ssize_t store_fatal_err(struct device *dev,
							   struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) | ESPI_FATEL_ERR, ASPEED_ESPI_SYS_EVENT);
	else
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) & ~ESPI_FATEL_ERR, ASPEED_ESPI_SYS_EVENT);

	return count;
}

static DEVICE_ATTR(fatal_err, S_IWUSR | S_IWUSR, show_fatal_err, store_fatal_err);

static ssize_t show_nfatal_err(struct device *dev,
							   struct device_attribute *attr, char *buf)
{
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) & ESPI_NFATEL_ERR ? "0" : "1");
}

static ssize_t store_nfatal_err(struct device *dev,
								struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) | ESPI_NFATEL_ERR, ASPEED_ESPI_SYS_EVENT);
	else
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) & ~ESPI_NFATEL_ERR, ASPEED_ESPI_SYS_EVENT);

	return count;
}

static DEVICE_ATTR(nfatal_err, S_IWUSR | S_IWUSR, show_nfatal_err, store_nfatal_err);

static ssize_t show_rest_cpu(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) & ESPI_REST_CPU_INIT ? "0" : "1");
}

static ssize_t store_rest_cpu(struct device *dev,
							  struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) | ESPI_REST_CPU_INIT, ASPEED_ESPI_SYS_EVENT);
	else
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) & ~ESPI_REST_CPU_INIT, ASPEED_ESPI_SYS_EVENT);

	return count;
}

static DEVICE_ATTR(rest_cpu,  S_IWUSR | S_IWUSR, show_rest_cpu, store_rest_cpu);

static ssize_t show_host_rest_ack(struct device *dev,
								  struct device_attribute *attr, char *buf)
{
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) & ESPI_HOST_REST_ACK ? "0" : "1");
}

static ssize_t store_host_rest_ack(struct device *dev,
								   struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) | ESPI_HOST_REST_ACK, ASPEED_ESPI_SYS_EVENT);
	else
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) & ~ESPI_HOST_REST_ACK, ASPEED_ESPI_SYS_EVENT);

	return count;
}

static DEVICE_ATTR(host_rest_ack, S_IWUSR | S_IWUSR, show_host_rest_ack, store_host_rest_ack);

static ssize_t show_oob_rest_ack(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) & ESPI_OOB_REST_ACK ? "0" : "1");
}

static ssize_t store_oob_rest_ack(struct device *dev,
								  struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) | ESPI_OOB_REST_ACK, ASPEED_ESPI_SYS_EVENT);
	else
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) & ~ESPI_OOB_REST_ACK, ASPEED_ESPI_SYS_EVENT);

	return count;
}

static DEVICE_ATTR(oob_rest_ack, S_IWUSR | S_IWUSR, show_oob_rest_ack, store_oob_rest_ack);

static ssize_t show_boot_sts(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) & ESPI_BOOT_STS ? "0" : "1");
}

static ssize_t store_boot_sts(struct device *dev,
							  struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) | ESPI_BOOT_STS, ASPEED_ESPI_SYS_EVENT);
	else
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) & ~ESPI_BOOT_STS, ASPEED_ESPI_SYS_EVENT);

	return count;
}

static DEVICE_ATTR(boot_sts, S_IWUSR | S_IWUSR, show_boot_sts, store_boot_sts);

static ssize_t show_boot_dwn(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) & ESPI_BOOT_DWN ? "0" : "1");
}

static ssize_t store_boot_dwn(struct device *dev,
							  struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) | ESPI_BOOT_DWN, ASPEED_ESPI_SYS_EVENT);
	else
		aspeed_espi_write(aspeed_espi, aspeed_espi_read(aspeed_espi, ASPEED_ESPI_SYS_EVENT) & ~ESPI_BOOT_DWN, ASPEED_ESPI_SYS_EVENT);

	return count;
}

static DEVICE_ATTR(boot_dwn, S_IWUSR | S_IWUSR, show_boot_dwn, store_boot_dwn);

static ssize_t show_op_freq(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);

	switch (GET_GCAP_OP_FREQ(aspeed_espi_read(aspeed_espi, ASPEED_ESPI_GCAP_CONFIG))) {
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
	struct aspeed_espi_data *aspeed_espi = dev_get_drvdata(dev);

	switch (GET_GCAP_IO_MODE(aspeed_espi_read(aspeed_espi, ASPEED_ESPI_GCAP_CONFIG))) {
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
	&dev_attr_boot_sts.attr,
	&dev_attr_boot_dwn.attr,
	&dev_attr_oob_rest_ack.attr,
	&dev_attr_fatal_err.attr,
	&dev_attr_nfatal_err.attr,
	&dev_attr_rest_cpu.attr,
	&dev_attr_host_rest_ack.attr,
	NULL
};

static struct attribute_group espi_attribute_group = {
	.attrs = espi_sysfs_entries,
};

static const struct file_operations aspeed_espi_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl		= espi_ioctl,
	.open			= espi_open,
	.release			= espi_release,
	.fasync			= espi_fasync,
};

struct miscdevice aspeed_espi_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aspeed-espi",
	.fops = &aspeed_espi_fops,
};

static const struct of_device_id aspeed_espi_of_matches[] = {
	{ .compatible = "aspeed,ast2500-espi", .data = (void *) 5, },
	{ .compatible = "aspeed,ast2600-espi", .data = (void *) 6, },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_espi_of_matches);


static int aspeed_espi_probe(struct platform_device *pdev)
{
	static struct aspeed_espi_data *aspeed_espi;
	const struct of_device_id *dev_id;
	struct resource *res;
	void __iomem *regs;
	int ret = 0, i = 0;

	ESPI_DBUG("\n");
	
	aspeed_espi = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_espi_data), GFP_KERNEL);
	if (aspeed_espi == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	dev_id = of_match_device(aspeed_espi_of_matches, &pdev->dev);
	if (!dev_id)
		return -EINVAL;

	aspeed_espi->espi_version = (unsigned long)dev_id->data;

	aspeed_espi->dev = &pdev->dev;

	if (of_device_is_compatible(pdev->dev.of_node, "aspeed,aspeed-espi-dma"))
		aspeed_espi->dma_mode = 1;
	else
		aspeed_espi->dma_mode = 0;

//
	aspeed_espi->map = syscon_node_to_regmap(pdev->dev.of_node);

	if (IS_ERR(aspeed_espi->map)) {
		printk("aspeed_espi->map ERROR \n");
		return PTR_ERR(aspeed_espi->map);	
	}

	aspeed_espi->irq = platform_get_irq(pdev, 0);
	if (aspeed_espi->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		return aspeed_espi->irq;
	}
#if 1
	ret = devm_request_irq(&pdev->dev, aspeed_espi->irq, aspeed_espi_isr,
						   IRQF_SHARED, dev_name(&pdev->dev), aspeed_espi);
	if (ret) {
		printk("AST ESPI Unable to get IRQ");
		return ret;
	}
#endif
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

#if 0
	aspeed_espi->irq_domain = irq_domain_add_linear(
					pdev->dev.of_node , 4,
					&aspeed_espi_irq_domain_ops, aspeed_espi->irq);
	if (!aspeed_espi->irq_domain) {
		ret = -ENOMEM;
		printk("irq_domain fail ---------------- \n");
	}

	irq_set_chained_handler_and_data(aspeed_espi->irq,
					 aspeed_espi_irq_handler, aspeed_espi);
#endif
	ret = misc_register(&aspeed_espi_misc);
	if (ret) {
		printk(KERN_ERR "ESPI : failed misc_register\n");
		return ret;
	}

	platform_set_drvdata(pdev, aspeed_espi);
	dev_set_drvdata(aspeed_espi_misc.this_device, aspeed_espi);

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
	struct aspeed_espi_data *aspeed_espi;
	struct resource *res;

	ESPI_DBUG("\n");
	aspeed_espi = platform_get_drvdata(pdev);
	if (aspeed_espi == NULL)
		return -ENODEV;

	free_irq(aspeed_espi->gpio_irq, aspeed_espi);
	free_irq(aspeed_espi->irq, aspeed_espi);
//	iounmap(aspeed_espi->reg_base);
	sysfs_remove_group(&pdev->dev.kobj, &espi_attribute_group);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

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
