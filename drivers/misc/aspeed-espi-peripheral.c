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

#define ESPIPIOC_BASE       'P'


#define ASPEED_ESPI_PERIPHERAL_IOCRX			_IOWR(ESPIPIOC_BASE, 0x0, struct aspeed_espi_xfer)			//post rx
#define ASPEED_ESPI_PERIPHERAL_IOCTX			_IOW(ESPIPIOC_BASE, 0x1, struct aspeed_espi_xfer)				//post tx
#define ASPEED_ESPI_PERINP_IOCTX				_IOW(ESPIPIOC_BASE, 0x2, struct aspeed_espi_xfer)				//non-post tx

struct aspeed_espi_peripheral {

	struct regmap *map;
	bool 					is_open;

	struct bin_attribute	bin;
	struct kernfs_node	*kn;	

	struct espi_ch_data		p_rx_channel;
	struct espi_ch_data		p_tx_channel;
	struct espi_ch_data		np_tx_channel;
	
	int 					irq;					//LPC IRQ number	
	int		espi_version;
	int						dma_mode;		/* o:disable , 1:enable */	
};

static u32 aspeed_espi_peripheral_read(struct aspeed_espi_peripheral *espi_peripheral, u32 reg)
{

	u32 val = 0;
	int rc;

	rc = regmap_read(espi_peripheral->map, reg, &val);
	WARN(rc != 0, "regmap_read() failed: %d\n", rc);

	return rc == 0 ? val : 0;
}

static void aspeed_espi_peripheral_write(struct aspeed_espi_peripheral *espi_peripheral, u32 reg, u32 data)
{

	int rc;

	rc = regmap_write(espi_peripheral->map, reg, data);
	WARN(rc != 0, "regmap_write() failed: %d\n", rc);
}

static void
aspeed_espi_pcp_rx(struct aspeed_espi_peripheral *espi_peripheral)
{
	int i = 0;
	u32 ctrl = aspeed_espi_peripheral_read(espi_peripheral, ASPEED_ESPI_PCP_RX_CTRL);
	printk("cycle type = %x , tag = %x, len = %d byte \n", ESPI_GET_CYCLE_TYPE(ctrl), ESPI_GET_TAG(ctrl), ESPI_GET_LEN(ctrl));

	espi_peripheral->p_rx_channel.header = ctrl;

	//Message
	if ((ESPI_GET_CYCLE_TYPE(ctrl) & 0x10) == 0x10) {		//message
		espi_peripheral->p_rx_channel.buf_len = 5;
		if (ESPI_GET_CYCLE_TYPE(ctrl) & 0x1)	//message with data
			espi_peripheral->p_rx_channel.buf_len += ESPI_GET_LEN(ctrl);
	} else if ((ESPI_GET_CYCLE_TYPE(ctrl) & 0x09) == 0x09)	//success com with data
		espi_peripheral->p_rx_channel.buf_len = ESPI_GET_LEN(ctrl);
	else
		espi_peripheral->p_rx_channel.buf_len = 0;

	if(!espi_peripheral->dma_mode) {
		for (i = 0; i < espi_peripheral->p_rx_channel.buf_len; i++)
			espi_peripheral->p_rx_channel.buff[i] = aspeed_espi_peripheral_read(espi_peripheral, ASPEED_ESPI_PCP_RX_DATA);
	}
	//wait for up get package
}

static void
aspeed_espi_pcp_tx(struct aspeed_espi_peripheral *espi_peripheral)
{
	int i = 0;
	printk("\n");

	if(!espi_peripheral->dma_mode) {
		for (i = 0; i < espi_peripheral->p_tx_channel.buf_len; i++)
			aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_PCP_TX_DATA, espi_peripheral->p_tx_channel.buff[i]);
	}
	
	aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_PCP_TX_CTRL, ESPI_TRIGGER_PACKAGE | espi_peripheral->p_tx_channel.header);
}

static void
aspeed_espi_pcnp_tx(struct aspeed_espi_peripheral *espi_peripheral)
{
	int i = 0;
	printk("\n");

	if(!espi_peripheral->dma_mode) {
		for (i = 0; i < espi_peripheral->np_tx_channel.buf_len; i++)
			aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_PCNP_TX_DATA, espi_peripheral->np_tx_channel.buff[i]);
	}
	aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_PCNP_TX_CTRL, ESPI_TRIGGER_PACKAGE | espi_peripheral->np_tx_channel.header);
}


static irqreturn_t aspeed_espi_peripheral_irq(int irq, void *arg)
{
	struct aspeed_espi_peripheral *espi_peripheral = arg;


	u32 sts = aspeed_espi_peripheral_read(espi_peripheral, ASPEED_ESPI_ISR);
	printk("aspeed_espi_peripheral_irq %x\n", sts);

	if (sts & (ESPI_ISR_FLASH_TX_ERR | ESPI_ISR_FLASH_TX_ABORT | ESPI_ISR_FLASH_RX_ABORT | ESPI_ISR_FLASH_TX_COMP | ESPI_ISR_FLASH_RX_COMP)) {

		if (sts & ESPI_ISR_PCNP_TX_ABORT) {
			printk("ESPI_ISR_PCNP_TX_ABORT\n");
			aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_ISR, ESPI_ISR_PCNP_TX_ABORT);
		}
		
		if (sts & ESPI_ISR_PCP_TX_ABORT) {
			printk("ESPI_ISR_PCP_TX_ABORT\n");
			aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_ISR, ESPI_ISR_PCP_TX_ABORT);
		}
		
		
		if (sts & ESPI_ISR_PCNP_RX_ABORT) {
			printk("ESPI_ISR_PCNP_RX_ABORT\n");
			aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_ISR, ESPI_ISR_PCNP_RX_ABORT);
		}
		
		if (sts & ESPI_ISR_PCP_RX_ABORT) {
			printk("ESPI_ISR_PCP_RX_ABORT \n");
			aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_ISR, ESPI_ISR_PCP_RX_ABORT);
		}
		
		if (sts & ESPI_ISR_PCNP_TX_ERR) {
			printk("ESPI_ISR_PCNP_TX_ERR \n");
			aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_ISR, ESPI_ISR_PCNP_TX_ERR);
		}
		
		if (sts & ESPI_ISR_PCP_TX_ERR) {
			printk("ESPI_ISR_PCP_TX_ERR \n");
			aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_ISR, ESPI_ISR_PCP_TX_ERR);
		}
		
		if (sts & ESPI_ISR_PCNP_TX_COMP) {
			printk("ESPI_ISR_PCNP_TX_COMP \n");
			aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_ISR, ESPI_ISR_PCNP_TX_COMP);
		}
		
		if (sts & ESPI_ISR_PCP_TX_COMP) {
			printk("ESPI_ISR_PCP_TX_COMP \n");
			aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_ISR, ESPI_ISR_PCP_TX_COMP);
		}
		
		if (sts & ESPI_ISR_PCP_RX_COMP) {
			printk("ESPI_ISR_PCP_RX_COMP \n");
			aspeed_espi_pcp_rx(espi_peripheral);
			aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_ISR, ESPI_ISR_PCP_RX_COMP);
		}

	
		return IRQ_HANDLED;		
	} else 
		return IRQ_NONE;

}

static void
aspeed_espi_peripheral_init(struct aspeed_espi_peripheral *espi_peripheral) {

//controller init 
	if(espi_peripheral->dma_mode) {
		aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_PCP_RX_DMA, espi_peripheral->p_rx_channel.dma_addr);
		aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_PCP_TX_DMA, espi_peripheral->p_tx_channel.dma_addr);

		aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_PCNP_TX_DMA, espi_peripheral->np_tx_channel.dma_addr);

		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_CTRL, ESPI_CTRL_PCNP_TX_DMA | ESPI_CTRL_PCP_RX_DMA | ESPI_CTRL_PCP_TX_DMA, 
			ESPI_CTRL_PCNP_TX_DMA | ESPI_CTRL_PCP_RX_DMA | ESPI_CTRL_PCP_TX_DMA);
	}

}

static DEFINE_SPINLOCK(espi_peripheral_state_lock);


static long
espi_peripheral_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct miscdevice *c = file->private_data;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(c->this_device);
	struct aspeed_espi_xfer xfer;

	if (copy_from_user(&xfer, (void*)arg, sizeof(struct aspeed_espi_xfer)))
		return -EFAULT; 

	switch (cmd) {
	case ASPEED_ESPI_PERIPHERAL_IOCRX:
		printk(" \n");
		xfer.header = espi_peripheral->p_rx_channel.header;
		xfer.buf_len = espi_peripheral->p_rx_channel.buf_len;
		if (copy_to_user(xfer.xfer_buf, espi_peripheral->p_rx_channel.buff, espi_peripheral->p_rx_channel.buf_len))
			ret = -EFAULT;
		aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_PCP_RX_CTRL, ESPI_TRIGGER_PACKAGE);
		break;
	case ASPEED_ESPI_PERIPHERAL_IOCTX:
		printk(" \n");
		espi_peripheral->p_tx_channel.header = xfer.header;
		espi_peripheral->p_tx_channel.buf_len = xfer.buf_len;
		if (copy_from_user(espi_peripheral->p_tx_channel.buff, xfer.xfer_buf, xfer.buf_len)) {
			printk("copy_from_user  fail\n");
			ret = -EFAULT;
		} else
			aspeed_espi_pcp_tx(espi_peripheral);
		break;
	case ASPEED_ESPI_PERINP_IOCTX:
		printk(" \n");
		espi_peripheral->np_tx_channel.header = xfer.header;
		espi_peripheral->np_tx_channel.buf_len = xfer.buf_len;
		if (copy_from_user(espi_peripheral->np_tx_channel.buff, xfer.xfer_buf, xfer.buf_len)) {
			printk("copy_from_user  fail\n");
			ret = -EFAULT;
		} else
			aspeed_espi_pcnp_tx(espi_peripheral);
		break;

	default:
		printk("ERROR \n");
		return -ENOTTY;
	}

	return ret;
}

static int espi_peripheral_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(c->this_device);

	printk("\n");
	spin_lock(&espi_peripheral_state_lock);

	if (espi_peripheral->is_open) {
		spin_unlock(&espi_peripheral_state_lock);
		return -EBUSY;
	}

	espi_peripheral->is_open = true;

	spin_unlock(&espi_peripheral_state_lock);

	return 0;
}

static int espi_peripheral_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(c->this_device);

	printk("\n");
	spin_lock(&espi_peripheral_state_lock);

	espi_peripheral->is_open = false;
//	espi_fasync(-1, file, 0);
	spin_unlock(&espi_peripheral_state_lock);

	return 0;
}

static const struct file_operations aspeed_espi_peripheral_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl		= espi_peripheral_ioctl,
	.open			= espi_peripheral_open,
	.release			= espi_peripheral_release,
};

struct miscdevice aspeed_espi_peripheral_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "espi-peripheral",
	.fops = &aspeed_espi_peripheral_fops,
};

static const struct of_device_id aspeed_espi_peripheral_match[] = {
	{ .compatible = "aspeed,ast2600-espi-peripheral", .data = (void *) 6, },
	{ .compatible = "aspeed,ast2500-espi-peripheral", .data = (void *) 5, },
	{ }
};
MODULE_DEVICE_TABLE(of, aspeed_espi_peripheral_match);

static int aspeed_espi_peripheral_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aspeed_espi_peripheral *espi_peripheral;
	const struct of_device_id *dev_id;	
	int rc, i;
	int irq;
printk("aspeed_espi_peripheral_probe \n");
	espi_peripheral = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_espi_peripheral), GFP_KERNEL);
	if (!espi_peripheral)
		return -ENOMEM;

	dev_id = of_match_device(aspeed_espi_peripheral_match, &pdev->dev);
	if (!dev_id)
		return -EINVAL;

	espi_peripheral->espi_version = (unsigned long)dev_id->data;

	if (of_property_read_bool(pdev->dev.of_node, "dma-mode"))
		espi_peripheral->dma_mode = 1;

	espi_peripheral->map = syscon_node_to_regmap(pdev->dev.of_node);
	if (IS_ERR(espi_peripheral->map)) {
		dev_err(dev, "Couldn't get regmap\n");
		return -ENODEV;
	}
#if 0
	rc = sysfs_create_group(&pdev->dev.kobj, &espi_peripheral_attribute_group);
	if (rc) {
		printk(KERN_ERR "aspeed_espi_peripheral: failed to create sysfs device attributes.\n");
		return -1;
	}
#endif	
///
	printk("espi flash read %x \n", aspeed_espi_peripheral_read(espi_peripheral, 0x0));
	dev_set_drvdata(dev, espi_peripheral);

	if(espi_peripheral->dma_mode) {
		espi_peripheral->p_rx_channel.buff = dma_alloc_coherent(NULL,
									  (MAX_XFER_BUFF_SIZE * 3),
									  &espi_peripheral->p_rx_channel.dma_addr, GFP_KERNEL);

		espi_peripheral->p_tx_channel.buff = espi_peripheral->p_rx_channel.buff  + MAX_XFER_BUFF_SIZE;
		espi_peripheral->p_tx_channel.dma_addr = espi_peripheral->p_rx_channel.dma_addr + MAX_XFER_BUFF_SIZE;

		espi_peripheral->np_tx_channel.buff = espi_peripheral->p_tx_channel.buff  + MAX_XFER_BUFF_SIZE;
		espi_peripheral->np_tx_channel.dma_addr = espi_peripheral->p_tx_channel.dma_addr + MAX_XFER_BUFF_SIZE;


		aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_PCP_RX_DMA, espi_peripheral->p_rx_channel.dma_addr);
		aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_PCP_TX_DMA, espi_peripheral->p_tx_channel.dma_addr);

		aspeed_espi_peripheral_write(espi_peripheral, ASPEED_ESPI_PCNP_TX_DMA, espi_peripheral->np_tx_channel.dma_addr);
	
		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_CTRL, ESPI_CTRL_PCNP_TX_DMA | ESPI_CTRL_PCP_RX_DMA | ESPI_CTRL_PCP_TX_DMA,
						ESPI_CTRL_PCNP_TX_DMA | ESPI_CTRL_PCP_RX_DMA | ESPI_CTRL_PCP_TX_DMA);	
	} else {
		// non-dma mode 
		espi_peripheral->p_rx_channel.buff = kzalloc(MAX_XFER_BUFF_SIZE * 3, GFP_KERNEL);
		espi_peripheral->p_tx_channel.buff = espi_peripheral->p_rx_channel.buff  + MAX_XFER_BUFF_SIZE;
		espi_peripheral->np_tx_channel.buff = espi_peripheral->p_tx_channel.buff  + MAX_XFER_BUFF_SIZE;
		
	}


printk("request p isr ******************************************************\n");
	espi_peripheral->irq = platform_get_irq(pdev, 0);
	if (espi_peripheral->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		return espi_peripheral->irq;
	}
	
	rc = devm_request_irq(&pdev->dev, espi_peripheral->irq, aspeed_espi_peripheral_irq, IRQF_SHARED,
				dev_name(&pdev->dev), espi_peripheral);

	if (rc) {
		printk("espi oob Unable to get IRQ \n");
		return rc;
	}
printk("request p ira ******************************************************1\n");	
	rc = misc_register(&aspeed_espi_peripheral_misc);
	if (rc) {
		dev_err(dev, "Unable to register device\n");
		return rc;
	}

	pr_info("aspeed espi flash loaded \n");

	return 0;
}

static int aspeed_espi_peripheral_remove(struct platform_device *pdev)
{
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(&pdev->dev);

	misc_deregister(&aspeed_espi_peripheral_misc);

	return 0;
}


static struct platform_driver aspeed_espi_peripheral_driver = {
	.driver = {
		.name           = "aspeed-espi-peripheral",
		.of_match_table = aspeed_espi_peripheral_match,
	},
	.probe  = aspeed_espi_peripheral_probe,
	.remove = aspeed_espi_peripheral_remove,
};
module_platform_driver(aspeed_espi_peripheral_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("Aspeed device interface to the KCS BMC device");
