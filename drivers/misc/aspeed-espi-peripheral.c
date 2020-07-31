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

#define DEVICE_NAME		"espi-peripheral"

#define ESPIPIOC_BASE	'P'

#define ASPEED_ESPI_PERIPHERAL_IOCRX			_IOWR(ESPIPIOC_BASE, 0x0, struct aspeed_espi_xfer)			//post rx
#define ASPEED_ESPI_PERIPHERAL_IOCTX			_IOW(ESPIPIOC_BASE, 0x1, struct aspeed_espi_xfer)				//post tx
#define ASPEED_ESPI_PERINP_IOCTX				_IOW(ESPIPIOC_BASE, 0x2, struct aspeed_espi_xfer)				//non-post tx

#define MQ_MSGBUF_SIZE		4096
#define MQ_QUEUE_SIZE		16
#define MQ_QUEUE_NEXT(x)	(((x) + 1) & (MQ_QUEUE_SIZE - 1))

struct mq_msg {
	int	len;
	u8	*buf;
};

struct aspeed_espi_peripheral {
	struct regmap 			*map;
	struct miscdevice       miscdev;
	int						espi_version;

	struct bin_attribute	bin;
	struct kernfs_node		*kn;

	struct espi_ch_data		p_rx_channel;
	struct espi_ch_data		p_tx_channel;
	struct espi_ch_data		np_tx_channel;

	int 					irq;					//LPC IRQ number
	int 					rest_irq;					//espi reset irq
	int						dma_mode;		/* 0:disable, 1:enable */
	int 					mmbi_mode;		/* 0:disable, 1:enable */

	phys_addr_t			host_mapping_mem_base;
	phys_addr_t			mapping_mem_base;
	resource_size_t		mapping_mem_size;
	void 				*mapping_mem_virt;

	struct mq_msg		*write_curr;
	int			wtruncated; /* drop current if truncated */
	struct mq_msg		write_queue[MQ_QUEUE_SIZE];
	spinlock_t wbuffer_lock;	/* spinlock for queue index handling */

	struct mq_msg		*read_curr;
	int			rtruncated; /* drop current if truncated */
	struct mq_msg		read_queue[MQ_QUEUE_SIZE];
	spinlock_t rbuffer_lock;	/* spinlock for queue index handling */
};

static void
aspeed_espi_pcp_rx(struct aspeed_espi_peripheral *espi_peripheral)
{
	int i = 0;
	u32 rx_ctrl, rx_data;

	regmap_read(espi_peripheral->map, ASPEED_ESPI_PCP_RX_CTRL, &rx_ctrl);
	printk("cycle type = %x , tag = %x, len = %d byte \n", ESPI_GET_CYCLE_TYPE(rx_ctrl), ESPI_GET_TAG(rx_ctrl), ESPI_GET_LEN(rx_ctrl));

	espi_peripheral->p_rx_channel.header = rx_ctrl;

	//Message
	if ((ESPI_GET_CYCLE_TYPE(rx_ctrl) & 0x10) == 0x10) {		//message
		espi_peripheral->p_rx_channel.buf_len = 5;
		if (ESPI_GET_CYCLE_TYPE(rx_ctrl) & 0x1)	//message with data
			espi_peripheral->p_rx_channel.buf_len += ESPI_GET_LEN(rx_ctrl);
	} else if ((ESPI_GET_CYCLE_TYPE(rx_ctrl) & 0x09) == 0x09)	//success com with data
		espi_peripheral->p_rx_channel.buf_len = ESPI_GET_LEN(rx_ctrl);
	else
		espi_peripheral->p_rx_channel.buf_len = 0;

	if(!espi_peripheral->dma_mode) {
		for (i = 0; i < espi_peripheral->p_rx_channel.buf_len; i++) {
			regmap_read(espi_peripheral->map, ASPEED_ESPI_PCP_RX_DATA, &rx_data);
			espi_peripheral->p_rx_channel.buff[i] = (u8) rx_data;
		}
	}
}

static void
aspeed_espi_pcp_tx(struct aspeed_espi_peripheral *espi_peripheral)
{
	int i = 0;

	if(!espi_peripheral->dma_mode) {
		for (i = 0; i < espi_peripheral->p_tx_channel.buf_len; i++)
			regmap_write(espi_peripheral->map, ASPEED_ESPI_PCP_TX_DATA, espi_peripheral->p_tx_channel.buff[i]);
	}

	regmap_write(espi_peripheral->map, ASPEED_ESPI_PCP_TX_CTRL, ESPI_TRIGGER_PACKAGE | espi_peripheral->p_tx_channel.header);
}

static void
aspeed_espi_pcnp_tx(struct aspeed_espi_peripheral *espi_peripheral)
{
	int i = 0;

	if(!espi_peripheral->dma_mode) {
		for (i = 0; i < espi_peripheral->np_tx_channel.buf_len; i++)
			regmap_write(espi_peripheral->map, ASPEED_ESPI_PCNP_TX_DATA, espi_peripheral->np_tx_channel.buff[i]);
	}

	regmap_write(espi_peripheral->map, ASPEED_ESPI_PCNP_TX_CTRL, ESPI_TRIGGER_PACKAGE | espi_peripheral->np_tx_channel.header);
}

static irqreturn_t aspeed_espi_peripheral_reset_irq(int irq, void *arg)
{
	struct aspeed_espi_peripheral *espi_peripheral = arg;

	if(espi_peripheral->dma_mode) {
		regmap_write(espi_peripheral->map, ASPEED_ESPI_PCP_RX_DMA, espi_peripheral->p_rx_channel.dma_addr);
		regmap_write(espi_peripheral->map, ASPEED_ESPI_PCP_TX_DMA, espi_peripheral->p_tx_channel.dma_addr);
		regmap_write(espi_peripheral->map, ASPEED_ESPI_PCNP_TX_DMA, espi_peripheral->np_tx_channel.dma_addr);

		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_CTRL, ESPI_CTRL_PCNP_TX_DMA | ESPI_CTRL_PCP_RX_DMA | ESPI_CTRL_PCP_TX_DMA,
			ESPI_CTRL_PCNP_TX_DMA | ESPI_CTRL_PCP_RX_DMA | ESPI_CTRL_PCP_TX_DMA);
	}
	return IRQ_HANDLED;
}

static irqreturn_t aspeed_espi_peripheral_irq(int irq, void *arg)
{
	u32 sts, pp_isr;
	struct aspeed_espi_peripheral *espi_peripheral = arg;

	regmap_read(espi_peripheral->map, ASPEED_ESPI_ISR, &sts);

	pp_isr = sts & (ESPI_ISR_PCNP_TX_ABORT | ESPI_ISR_PCP_TX_ABORT |
		ESPI_ISR_PCNP_TX_ERR | ESPI_ISR_PCP_TX_ERR |
		ESPI_ISR_PCNP_TX_COMP | ESPI_ISR_PCP_TX_COMP |
		ESPI_ISR_PCNP_RX_ABORT | ESPI_ISR_PCP_RX_ABORT |
		ESPI_ISR_PCP_RX_COMP);

	printk("aspeed_espi_peripheral_irq %x\n", pp_isr);

	if (pp_isr) {
		if (pp_isr & ESPI_ISR_PCP_RX_COMP) {
			printk("ESPI_ISR_PCP_RX_COMP \n");
			aspeed_espi_pcp_rx(espi_peripheral);
		}
		regmap_write(espi_peripheral->map, ASPEED_ESPI_ISR, pp_isr);
		return IRQ_HANDLED;
	} else
		return IRQ_NONE;

}

static struct aspeed_espi_peripheral *file_aspeed_espi_peripherial(struct file *file)
{
	return container_of(file->private_data, struct aspeed_espi_peripheral,
			miscdev);
}

static int aspeed_espi_peripheral_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct aspeed_espi_peripheral *espi_peripheral = file_aspeed_espi_peripherial(file);
	unsigned long vsize = vma->vm_end - vma->vm_start;
	pgprot_t prot = vma->vm_page_prot;

	if (espi_peripheral->mmbi_mode)
		return -EPERM;

	if (vma->vm_pgoff + vsize > espi_peripheral->mapping_mem_base + espi_peripheral->mapping_mem_size)
		return -EINVAL;

	prot = pgprot_noncached(prot);

	if (remap_pfn_range(vma, vma->vm_start,
		(espi_peripheral->mapping_mem_base >> PAGE_SHIFT) + vma->vm_pgoff,
		vsize, prot))
		return -EAGAIN;

	return 0;
}

static long
aspeed_espi_peripheral_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct aspeed_espi_peripheral *espi_peripheral = file_aspeed_espi_peripherial(file);
	struct aspeed_espi_xfer xfer;

	if (copy_from_user(&xfer, (void*)arg, sizeof(struct aspeed_espi_xfer)))
		return -EFAULT;

	switch (cmd) {
	case ASPEED_ESPI_PERIPHERAL_IOCRX:
		printk("ASPEED_ESPI_PERIPHERAL_IOCRX \n");
		xfer.header = espi_peripheral->p_rx_channel.header;
		xfer.buf_len = espi_peripheral->p_rx_channel.buf_len;
		if (copy_to_user(xfer.xfer_buf, espi_peripheral->p_rx_channel.buff, espi_peripheral->p_rx_channel.buf_len))
			ret = -EFAULT;
		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_PCP_RX_CTRL, ESPI_TRIGGER_PACKAGE, ESPI_TRIGGER_PACKAGE);
		break;
	case ASPEED_ESPI_PERIPHERAL_IOCTX:
		printk("ASPEED_ESPI_PERIPHERAL_IOCTX \n");
		espi_peripheral->p_tx_channel.header = xfer.header;
		espi_peripheral->p_tx_channel.buf_len = xfer.buf_len;
		if (copy_from_user(espi_peripheral->p_tx_channel.buff, xfer.xfer_buf, xfer.buf_len)) {
			printk("copy_from_user  fail\n");
			ret = -EFAULT;
		} else
			aspeed_espi_pcp_tx(espi_peripheral);
		break;
	case ASPEED_ESPI_PERINP_IOCTX:
		printk("ASPEED_ESPI_PERINP_IOCTX \n");
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

/* for sw mode memory read */
static ssize_t aspeed_espi_peripheral_bin_read(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(container_of(kobj, struct device, kobj));
	unsigned long flags;

	printk("aspeed_espi_peripheral_bin_read\n");

	spin_lock_irqsave(&espi_peripheral->rbuffer_lock, flags);
//	memcpy(buf, &espi_peripheral->buffer[off], count);
	spin_unlock_irqrestore(&espi_peripheral->rbuffer_lock, flags);

	return count;
}

/* for sw mode memory write */
static ssize_t aspeed_espi_peripheral_bin_write(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(container_of(kobj, struct device, kobj));
	unsigned long flags;

	printk("aspeed_espi_peripheral_bin_write\n");

	spin_lock_irqsave(&espi_peripheral->wbuffer_lock, flags);
//	memcpy(&espi_peripheral->buffer[off], buf, count);
	spin_unlock_irqrestore(&espi_peripheral->wbuffer_lock, flags);

	return count;
}

static ssize_t show_fatal_err(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	u32 sys_evt;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(dev);

	regmap_read(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, &sys_evt);
	return sprintf(buf, "%s\n", sys_evt & ESPI_FATEL_ERR ? "0" : "1");
}

static ssize_t store_fatal_err(struct device *dev,
							   struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);

	if (val)
		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, ESPI_FATEL_ERR, ESPI_FATEL_ERR);
	else
		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, ESPI_FATEL_ERR, 0);

	return count;
}

static DEVICE_ATTR(fatal_err, S_IWUSR | S_IWUSR, show_fatal_err, store_fatal_err);

static ssize_t show_nfatal_err(struct device *dev,
							   struct device_attribute *attr, char *buf)
{
	u32 sys_evt;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(dev);

	regmap_read(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, &sys_evt);
	return sprintf(buf, "%s\n", sys_evt & ESPI_NFATEL_ERR ? "0" : "1");
}

static ssize_t store_nfatal_err(struct device *dev,
								struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, ESPI_NFATEL_ERR, ESPI_NFATEL_ERR);
	else
		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, ESPI_NFATEL_ERR, 0);

	return count;
}

static DEVICE_ATTR(nfatal_err, S_IWUSR | S_IWUSR, show_nfatal_err, store_nfatal_err);

static ssize_t show_rest_cpu(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	u32 sys_evt;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(dev);

	regmap_read(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, &sys_evt);
	return sprintf(buf, "%s\n", sys_evt & ESPI_REST_CPU_INIT ? "0" : "1");
}

static ssize_t store_rest_cpu(struct device *dev,
							  struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, ESPI_REST_CPU_INIT, ESPI_REST_CPU_INIT);
	else
		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, ESPI_REST_CPU_INIT, 0);

	return count;
}

static DEVICE_ATTR(rest_cpu,  S_IWUSR | S_IWUSR, show_rest_cpu, store_rest_cpu);

static ssize_t show_host_rest_ack(struct device *dev,
								  struct device_attribute *attr, char *buf)
{
	u32 sys_evt;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(dev);

	regmap_read(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, &sys_evt);
	return sprintf(buf, "%s\n", sys_evt & ESPI_HOST_REST_ACK ? "0" : "1");
}

static ssize_t store_host_rest_ack(struct device *dev,
								   struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, ESPI_HOST_REST_ACK, ESPI_HOST_REST_ACK);
	else
		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, ESPI_HOST_REST_ACK, 0);

	return count;
}

static DEVICE_ATTR(host_rest_ack, S_IWUSR | S_IWUSR, show_host_rest_ack, store_host_rest_ack);

static ssize_t show_oob_rest_ack(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	u32 sys_evt;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(dev);

	regmap_read(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, &sys_evt);
	return sprintf(buf, "%s\n", sys_evt & ESPI_OOB_REST_ACK ? "0" : "1");
}

static ssize_t store_oob_rest_ack(struct device *dev,
								  struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, ESPI_OOB_REST_ACK, ESPI_OOB_REST_ACK);
	else
		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, ESPI_OOB_REST_ACK, 0);

	return count;
}

static DEVICE_ATTR(oob_rest_ack, S_IWUSR | S_IWUSR, show_oob_rest_ack, store_oob_rest_ack);

static ssize_t show_boot_sts(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	u32 sys_evt;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(dev);

	regmap_read(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, &sys_evt);
	return sprintf(buf, "%s\n", sys_evt & ESPI_BOOT_STS ? "0" : "1");
}

static ssize_t store_boot_sts(struct device *dev,
							  struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, ESPI_BOOT_STS, ESPI_BOOT_STS);
	else
		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, ESPI_BOOT_STS, 0);

	return count;
}

static DEVICE_ATTR(boot_sts, S_IWUSR | S_IWUSR, show_boot_sts, store_boot_sts);

static ssize_t show_boot_dwn(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	u32 sys_evt;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(dev);

	regmap_read(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, &sys_evt);
	return sprintf(buf, "%s\n", sys_evt & ESPI_BOOT_DWN ? "0" : "1");
}

static ssize_t store_boot_dwn(struct device *dev,
							  struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, ESPI_BOOT_DWN, ESPI_BOOT_DWN);
	else
		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_SYS_EVENT, ESPI_BOOT_DWN, 0);

	return count;
}

static DEVICE_ATTR(boot_dwn, S_IWUSR | S_IWUSR, show_boot_dwn, store_boot_dwn);

static struct attribute *espi_peripheral_sysfs_entries[] = {
	&dev_attr_boot_sts.attr,
	&dev_attr_boot_dwn.attr,
	&dev_attr_oob_rest_ack.attr,
	&dev_attr_fatal_err.attr,
	&dev_attr_nfatal_err.attr,
	&dev_attr_rest_cpu.attr,
	&dev_attr_host_rest_ack.attr,
	NULL
};

static struct attribute_group espi_peripheral_attribute_group = {
	.attrs = espi_peripheral_sysfs_entries,
};

static const struct file_operations aspeed_espi_peripheral_fops = {
	.owner			= THIS_MODULE,
	.mmap			= aspeed_espi_peripheral_mmap,
	.unlocked_ioctl	= aspeed_espi_peripheral_ioctl,
};

static const struct of_device_id aspeed_espi_peripheral_match[] = {
	{ .compatible = "aspeed,ast2600-espi-peripheral",	.data = (void *) ESPI_AST2600, },
	{ .compatible = "aspeed,ast2500-espi-peripheral",	.data = (void *) ESPI_AST2500, },
	{ }
};
MODULE_DEVICE_TABLE(of, aspeed_espi_peripheral_match);

static int aspeed_espi_peripheral_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *mmbi_np;
	struct aspeed_espi_peripheral *espi_peripheral;
	const struct of_device_id *dev_id;
	int rc;

	espi_peripheral = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_espi_peripheral), GFP_KERNEL);
	if (!espi_peripheral)
		return -ENOMEM;

	dev_id = of_match_device(aspeed_espi_peripheral_match, &pdev->dev);
	if (!dev_id)
		return -EINVAL;

	espi_peripheral->espi_version = (unsigned long)dev_id->data;

	if (of_property_read_bool(pdev->dev.of_node, "dma-mode"))
		espi_peripheral->dma_mode = 1;

	espi_peripheral->map = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(espi_peripheral->map)) {
		dev_err(dev, "Couldn't get regmap\n");
		return -ENODEV;
	}

	espi_peripheral->irq = platform_get_irq(pdev, 0);
	if (espi_peripheral->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		return espi_peripheral->irq;
	}

	mmbi_np = of_parse_phandle(dev->of_node, "aspeed,espi-mmbi", 0);
	if (mmbi_np) {
		espi_peripheral->mmbi_mode = of_device_is_available(mmbi_np);
		of_node_put(mmbi_np);
	}

	if (!espi_peripheral->mmbi_mode) {
		rc = of_property_read_u32(dev->of_node, "host-map-addr", &espi_peripheral->host_mapping_mem_base);
		if (rc) {
			dev_err(dev, "Couldn't get host mapping address\n");
			return -ENODEV;
		}

		rc = of_property_read_u32(dev->of_node, "map-size", &espi_peripheral->mapping_mem_size);
		if (rc) {
			dev_err(dev, "Couldn't get mapping size\n");
			return -ENODEV;
		}

		if (espi_peripheral->mapping_mem_size < 0x10000)
			espi_peripheral->mapping_mem_size = 0x10000;

		espi_peripheral->mapping_mem_size = roundup_pow_of_two(espi_peripheral->mapping_mem_size);
		espi_peripheral->mapping_mem_virt = dma_alloc_coherent(dev, espi_peripheral->mapping_mem_size,
				&espi_peripheral->mapping_mem_base, GFP_KERNEL);
		if (IS_ERR_OR_NULL(espi_peripheral->mapping_mem_virt)) {
			dev_err(dev, "Failed to allocate memory cycle region\n");
			return -ENOMEM;
		}

		if(espi_peripheral->espi_version == ESPI_AST2500) {
			regmap_write(espi_peripheral->map, ASPEED_ESPI_PC_RX_TADDRM, 0xFEDC756E);
			regmap_write(espi_peripheral->map, ASPEED_ESPI_PC_RX_SADDR, espi_peripheral->host_mapping_mem_base);
			regmap_write(espi_peripheral->map, ASPEED_ESPI_PC_RX_TADDR, espi_peripheral->mapping_mem_base);
		} else {
			regmap_write(espi_peripheral->map, ASPEED_ESPI_PC_RX_SADDR, espi_peripheral->host_mapping_mem_base);
			regmap_write(espi_peripheral->map, ASPEED_ESPI_PC_RX_TADDR, espi_peripheral->mapping_mem_base);
			regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_PC_RX_TADDRM,
					GENMASK(31, 16), ~(espi_peripheral->mapping_mem_size - 1));
			regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_CTRL2,
					ESPI_DISABLE_PERP_MEM_READ | ESPI_DISABLE_PERP_MEM_WRITE, 0 );
		}
	}

	rc = sysfs_create_group(&pdev->dev.kobj, &espi_peripheral_attribute_group);
	if (rc) {
		printk(KERN_ERR "aspeed_espi_peripheral: failed to create sysfs device attributes.\n");
		return -1;
	}

	dev_set_drvdata(dev, espi_peripheral);

	if(espi_peripheral->dma_mode) {
		espi_peripheral->p_rx_channel.buff = dma_alloc_coherent(NULL,
									  (MAX_XFER_BUFF_SIZE * 3),
									  &espi_peripheral->p_rx_channel.dma_addr, GFP_KERNEL);

		espi_peripheral->p_tx_channel.buff = espi_peripheral->p_rx_channel.buff  + MAX_XFER_BUFF_SIZE;
		espi_peripheral->p_tx_channel.dma_addr = espi_peripheral->p_rx_channel.dma_addr + MAX_XFER_BUFF_SIZE;

		espi_peripheral->np_tx_channel.buff = espi_peripheral->p_tx_channel.buff  + MAX_XFER_BUFF_SIZE;
		espi_peripheral->np_tx_channel.dma_addr = espi_peripheral->p_tx_channel.dma_addr + MAX_XFER_BUFF_SIZE;


		regmap_write(espi_peripheral->map, ASPEED_ESPI_PCP_RX_DMA, espi_peripheral->p_rx_channel.dma_addr);
		regmap_write(espi_peripheral->map, ASPEED_ESPI_PCP_TX_DMA, espi_peripheral->p_tx_channel.dma_addr);
		regmap_write(espi_peripheral->map, ASPEED_ESPI_PCNP_TX_DMA, espi_peripheral->np_tx_channel.dma_addr);

		regmap_update_bits(espi_peripheral->map, ASPEED_ESPI_CTRL, ESPI_CTRL_PCNP_TX_DMA | ESPI_CTRL_PCP_RX_DMA | ESPI_CTRL_PCP_TX_DMA,
						ESPI_CTRL_PCNP_TX_DMA | ESPI_CTRL_PCP_RX_DMA | ESPI_CTRL_PCP_TX_DMA);
	} else {
		// non-dma mode
		espi_peripheral->p_rx_channel.buff = kzalloc(MAX_XFER_BUFF_SIZE * 3, GFP_KERNEL);
		espi_peripheral->p_tx_channel.buff = espi_peripheral->p_rx_channel.buff  + MAX_XFER_BUFF_SIZE;
		espi_peripheral->np_tx_channel.buff = espi_peripheral->p_tx_channel.buff  + MAX_XFER_BUFF_SIZE;
	}


	rc = devm_request_irq(&pdev->dev, espi_peripheral->irq, aspeed_espi_peripheral_irq, IRQF_SHARED,
				dev_name(&pdev->dev), espi_peripheral);

	if (rc) {
		printk("espi peripheral Unable to get IRQ \n");
		return rc;
	}

	espi_peripheral->rest_irq = platform_get_irq(pdev, 1);
	if (espi_peripheral->rest_irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		return espi_peripheral->irq;
	}
	rc = devm_request_irq(&pdev->dev, espi_peripheral->rest_irq, aspeed_espi_peripheral_reset_irq, IRQF_SHARED,
				dev_name(&pdev->dev), espi_peripheral);

	if (rc) {
		printk("espi peripheral Unable to get reset IRQ \n");
		return rc;
	}

	sysfs_bin_attr_init(&espi_peripheral->bin);
	espi_peripheral->bin.attr.name = "espi-channel0";
	espi_peripheral->bin.attr.mode = S_IRUSR | S_IWUSR;
	espi_peripheral->bin.read = aspeed_espi_peripheral_bin_read;
	espi_peripheral->bin.write = aspeed_espi_peripheral_bin_write;
	espi_peripheral->bin.size = MQ_MSGBUF_SIZE * MQ_QUEUE_SIZE;;

	rc = sysfs_create_bin_file(&pdev->dev.kobj, &espi_peripheral->bin);
	if (rc)
		return rc;

	espi_peripheral->miscdev.minor = MISC_DYNAMIC_MINOR;
	espi_peripheral->miscdev.name = DEVICE_NAME;
	espi_peripheral->miscdev.fops = &aspeed_espi_peripheral_fops;
	espi_peripheral->miscdev.parent = dev;
	rc = misc_register(&espi_peripheral->miscdev);
	if (rc) {
		dev_err(dev, "Unable to register device\n");
		return rc;
	}

	pr_info("aspeed espi-peripheral loaded \n");

	return 0;
}

static int aspeed_espi_peripheral_remove(struct platform_device *pdev)
{
	struct aspeed_espi_peripheral *espi_peripheral = dev_get_drvdata(&pdev->dev);

	misc_deregister(&espi_peripheral->miscdev);

	if (!IS_ERR_OR_NULL(espi_peripheral->mapping_mem_virt)) {
		dma_free_coherent(&pdev->dev, espi_peripheral->mapping_mem_size,
				espi_peripheral->mapping_mem_virt, espi_peripheral->mapping_mem_base);
	}

	return 0;
}

static struct platform_driver aspeed_espi_peripheral_driver = {
	.driver = {
		.name           = DEVICE_NAME,
		.of_match_table = aspeed_espi_peripheral_match,
	},
	.probe  = aspeed_espi_peripheral_probe,
	.remove = aspeed_espi_peripheral_remove,
};
module_platform_driver(aspeed_espi_peripheral_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("Aspeed device interface to the KCS BMC device");
