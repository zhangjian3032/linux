// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) ASPEED Technology Inc.
 */
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/kfifo.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <linux/dma-mapping.h>

#define DEVICE_NAME "aspeed-lpc-snoop-dma"

#define LHCR5	0x34
#define LHCR6	0x38
#define LHCRA	0x48
#define 	LHCRA_PAT_A_LEN_MASK	GENMASK(18, 17)
#define 	LHCRA_PAT_A_LEN_SHIFT	17
#define 	LHCRA_PAT_A_WRITE	BIT(16)
#define 	LHCRA_PAT_A_ADDR_MASK	GENMASK(15, 0)
#define 	LHCRA_PAT_A_ADDR_SHIFT	0
#define LHCRB	0x4C
#define 	LHCRB_PAT_B_LEN_MASK	GENMASK(18, 17)
#define 	LHCRB_PAT_B_LEN_SHIFT	17
#define 	LHCRB_PAT_B_WRITE	BIT(16)
#define 	LHCRB_PAT_B_ADDR_MASK	GENMASK(15, 0)
#define 	LHCRB_PAT_B_ADDR_SHIFT	0
#define PCCR0	0xB0
#define 	PCCR0_EN_DMA_INT	BIT(31)
#define 	PCCR0_EN_PAT_B_INT	BIT(23)
#define 	PCCR0_EN_PAT_B		BIT(22)
#define 	PCCR0_EN_PAT_A_INT	BIT(21)
#define 	PCCR0_EN_PAT_A		BIT(20)
#define 	PCCR0_EN_DMA_MODE	BIT(14)
#define 	PCCR0_ADDR_SELECT_MASK	GENMASK(13, 12)
#define 	PCCR0_ADDR_SELECT_SHIFT	12
#define 	PCCR0_CLR_RX_FIFO	BIT(7)
#define 	PCCR0_MODE_SELECT_MASK	GENMASK(5, 4)
#define 	PCCR0_MODE_SELECT_SHIFT	4
#define 	PCCR0_EN		BIT(0)
#define PCCR1	0xB4
#define 	PCCR1_BASE_ADDR_MASK		GENMASK(15, 0)
#define 	PCCR1_BASE_ADDR_SHIFT		0
#define 	PCCR1_DONT_CARE_BITS_MASK	GENMASK(21, 16)
#define 	PCCR1_DONT_CARE_BITS_SHIFT	16
#define PCCR2	0xB8
#define 	PCCR2_PAT_B_RST		BIT(17)
#define 	PCCR2_PAT_B_INT		BIT(16)
#define 	PCCR2_PAT_A_RST		BIT(9)
#define 	PCCR2_PAT_A_INT		BIT(8)
#define 	PCCR2_SNOOP_DMA_DONE	BIT(4)
#define PCCR4	0x50
#define PCCR5	0x54
#define PCCR6	0x44

#define SNOOP_DMA_MAX_BUFSZ	(PAGE_SIZE)
#define SNOOP_DMA_MAX_PATNM	2

enum snoop_dma_record_mode {
	SNOOP_DMA_REC_1B,
	SNOOP_DMA_REC_2B,
	SNOOP_DMA_REC_4B,
	SNOOP_DMA_REC_FULL,
};

enum snoop_dma_port_hbits_select {
	SNOOP_DMA_PORT_HBITS_SEL_NONE,
	SNOOP_DMA_PORT_HBITS_SEL_45,
	SNOOP_DMA_PORT_HBITS_SEL_67,
	SNOOP_DMA_PORT_HBITS_SEL_89,
};

struct snoop_dma_pattern {
	u32 enable;
	u32 pattern;
	u32 len;
	u32 write;
	u32 port;
};

struct aspeed_snoop_dma {
	struct device *dev;
	struct regmap *regmap;
	int irq;

	u32 rec_mode;

	u32 port;
	u32 port_xbits;
	u32 port_hbits_select;

	u32 dma_idx;
	u8 *dma_virt;
	dma_addr_t dma_addr;
	u32 dma_size;
	u32 dma_addr_reserved;

	struct snoop_dma_pattern pat_search[SNOOP_DMA_MAX_PATNM];

	struct kfifo fifo;
	wait_queue_head_t wq;

	struct miscdevice misc_dev;
	struct tasklet_struct tasklet;
};

static inline bool is_snoop_dma_enabled(struct aspeed_snoop_dma *snoop_dma)
{
	u32 reg;
	if (regmap_read(snoop_dma->regmap, PCCR0, &reg))
		return false;
	return (reg & PCCR0_EN) ? true : false;
}

static inline bool is_valid_rec_mode(u32 mode)
{
	return (mode > SNOOP_DMA_REC_FULL) ? false : true;
}

static inline bool is_valid_high_bits_select(u32 select)
{
	return (select > SNOOP_DMA_PORT_HBITS_SEL_89) ? false : true;
}

static ssize_t aspeed_snoop_dma_file_read(struct file *file, char __user *buffer,
		size_t count, loff_t *ppos)
{
	int rc;
	ssize_t copied;

	struct aspeed_snoop_dma *snoop_dma = container_of(
			file->private_data,
			struct aspeed_snoop_dma,
			misc_dev);

	if (kfifo_is_empty(&snoop_dma->fifo)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		rc = wait_event_interruptible(snoop_dma->wq,
				!kfifo_is_empty(&snoop_dma->fifo));
		if (rc == -ERESTARTSYS)
			return -EINTR;
	}

	rc = kfifo_to_user(&snoop_dma->fifo, buffer, count, &copied);
	return rc ? rc : copied;
}

static __poll_t aspeed_snoop_dma_file_poll(struct file *file,
		struct poll_table_struct *pt)
{
	struct aspeed_snoop_dma *snoop_dma = container_of(
			file->private_data,
			struct aspeed_snoop_dma,
			misc_dev);

	poll_wait(file, &snoop_dma->wq, pt);
	return !kfifo_is_empty(&snoop_dma->fifo) ? POLLIN : 0;
}

static const struct file_operations snoop_dma_fops = {
	.owner = THIS_MODULE,
	.read = aspeed_snoop_dma_file_read,
	.poll = aspeed_snoop_dma_file_poll,
};

static void aspeed_snoop_dma_tasklet(unsigned long arg)
{
	u32 reg;
	u32 pre_dma_idx;
	u32 cur_dma_idx;
	
	u8 has_data = 0;

	struct aspeed_snoop_dma *snoop_dma = (struct aspeed_snoop_dma*)arg;
	struct kfifo *fifo = &snoop_dma->fifo;

	if (!kfifo_initialized(fifo))
		return;

	if (regmap_read(snoop_dma->regmap, PCCR6, &reg))
		return;

	cur_dma_idx = reg & (SNOOP_DMA_MAX_BUFSZ - 1);
	pre_dma_idx = snoop_dma->dma_idx;
	has_data = (pre_dma_idx == cur_dma_idx) ? false : true;

	do {
		/* kick the oldest one if full */
		if (kfifo_is_full(fifo))
			kfifo_skip(fifo);
		kfifo_put(fifo, snoop_dma->dma_virt[pre_dma_idx]);
		pre_dma_idx = (pre_dma_idx + 1) % SNOOP_DMA_MAX_BUFSZ;
	} while (pre_dma_idx != cur_dma_idx);

	if (has_data)
		wake_up_interruptible(&snoop_dma->wq);

	snoop_dma->dma_idx = cur_dma_idx;
}

static irqreturn_t aspeed_snoop_dma_irq(int irq, void *arg)
{
	u32 val;
	irqreturn_t ret = IRQ_NONE;
	struct aspeed_snoop_dma *snoop_dma = (struct aspeed_snoop_dma*)arg;

	if (regmap_read(snoop_dma->regmap, PCCR2, &val))
		return ret;

	/* handle pattern search B */
	if (val & PCCR2_PAT_B_INT) {
		dev_info(snoop_dma->dev, "pattern search B interrupt\n");
		regmap_write_bits(snoop_dma->regmap, PCCR2,
			PCCR2_PAT_B_INT, PCCR2_PAT_B_INT);
		ret = IRQ_HANDLED;
	}

	/* handle pattern search A */
	if (val & PCCR2_PAT_A_INT) {
		dev_info(snoop_dma->dev, "pattern search A interrupt\n");
		regmap_write_bits(snoop_dma->regmap, PCCR2,
			PCCR2_PAT_A_INT, PCCR2_PAT_A_INT);
		ret = IRQ_HANDLED;
	}

	/* handle DMA done */
	if (val & PCCR2_SNOOP_DMA_DONE) {
		regmap_write_bits(snoop_dma->regmap, PCCR2,
			PCCR2_SNOOP_DMA_DONE, PCCR2_SNOOP_DMA_DONE);
		tasklet_schedule(&snoop_dma->tasklet);
		ret = IRQ_HANDLED;
	}

	return ret;
}

static void aspeed_snoop_dma_config(struct aspeed_snoop_dma *snoop_dma)
{
	struct snoop_dma_pattern* pat_search = snoop_dma->pat_search;

	/* record mode */
	regmap_update_bits(snoop_dma->regmap, PCCR0,
			PCCR0_MODE_SELECT_MASK,
			snoop_dma->rec_mode << PCCR0_MODE_SELECT_SHIFT);

	/* port address */
	regmap_update_bits(snoop_dma->regmap, PCCR1,
			PCCR1_BASE_ADDR_MASK,
			snoop_dma->port << PCCR1_BASE_ADDR_SHIFT);

	/* port address high bits selection or parser control */
	regmap_update_bits(snoop_dma->regmap, PCCR0,
			PCCR0_ADDR_SELECT_MASK,
			snoop_dma->port_hbits_select << PCCR0_ADDR_SELECT_SHIFT);

	/* port address dont care bits */
	regmap_update_bits(snoop_dma->regmap, PCCR1,
			PCCR1_DONT_CARE_BITS_MASK,
			snoop_dma->port_xbits << PCCR1_DONT_CARE_BITS_SHIFT);

	/* pattern search state reset */
	regmap_write_bits(snoop_dma->regmap, PCCR2,
			PCCR2_PAT_B_RST | PCCR2_PAT_A_RST,
			PCCR2_PAT_B_RST | PCCR2_PAT_A_RST);

	/* pattern A to search */
	regmap_write(snoop_dma->regmap, LHCR5, pat_search[0].pattern);
	regmap_update_bits(snoop_dma->regmap, LHCRA,
			LHCRA_PAT_A_LEN_MASK,
			(pat_search[0].len - 1) << LHCRA_PAT_A_LEN_SHIFT);
	regmap_update_bits(snoop_dma->regmap, LHCRA,
			LHCRA_PAT_A_WRITE,
			(pat_search[0].write) ? LHCRA_PAT_A_WRITE : 0);
	regmap_update_bits(snoop_dma->regmap, LHCRA,
			LHCRA_PAT_A_ADDR_MASK,
			pat_search[0].port << LHCRA_PAT_A_ADDR_SHIFT);
	regmap_update_bits(snoop_dma->regmap, PCCR0,
			PCCR0_EN_PAT_A_INT | PCCR0_EN_PAT_A,
			(pat_search[0].enable) ? PCCR0_EN_PAT_A_INT | PCCR0_EN_PAT_A : 0);

	/* pattern B to search */
	regmap_write(snoop_dma->regmap, LHCR6, pat_search[1].pattern);
	regmap_update_bits(snoop_dma->regmap, LHCRB,
			LHCRB_PAT_B_LEN_MASK,
			(pat_search[1].len - 1) << LHCRB_PAT_B_LEN_SHIFT);
	regmap_update_bits(snoop_dma->regmap, LHCRB,
			LHCRB_PAT_B_WRITE,
			(pat_search[1].write) ? LHCRB_PAT_B_WRITE : 0);
	regmap_update_bits(snoop_dma->regmap, LHCRB,
			LHCRB_PAT_B_ADDR_MASK,
			pat_search[1].port << LHCRB_PAT_B_ADDR_SHIFT);
	regmap_update_bits(snoop_dma->regmap, PCCR0,
			PCCR0_EN_PAT_B_INT | PCCR0_EN_PAT_B,
			PCCR0_EN_PAT_B_INT | PCCR0_EN_PAT_B);
	regmap_update_bits(snoop_dma->regmap, PCCR0,
			PCCR0_EN_PAT_B_INT | PCCR0_EN_PAT_B,
			(pat_search[1].enable) ? PCCR0_EN_PAT_B_INT | PCCR0_EN_PAT_B : 0);

	/* DMA address and size (4-bytes unit) */
	regmap_write(snoop_dma->regmap, PCCR4, snoop_dma->dma_addr);
	regmap_write(snoop_dma->regmap, PCCR5, snoop_dma->dma_size / 4);
}

static int aspeed_snoop_dma_enable(struct aspeed_snoop_dma *snoop_dma, struct device *dev)
{
	int rc;

	/* map reserved memory or allocate a new one for DMA use */
	if (snoop_dma->dma_addr_reserved) {
		if (snoop_dma->dma_size > SNOOP_DMA_MAX_BUFSZ) {
			rc = -EINVAL;
			goto err_ret;
		}

		snoop_dma->dma_virt = ioremap(snoop_dma->dma_addr,
					      snoop_dma->dma_size);
		if (snoop_dma->dma_virt == NULL) {
			rc = -ENOMEM;
			goto err_ret;
		}
	}
	else {
		snoop_dma->dma_size = SNOOP_DMA_MAX_BUFSZ;
		snoop_dma->dma_virt = dma_alloc_coherent(NULL,
				snoop_dma->dma_size,
				&snoop_dma->dma_addr,
				GFP_KERNEL);
		if (snoop_dma->dma_virt == NULL) {
			rc = -ENOMEM;
			goto err_ret;
		}
	}

	rc = kfifo_alloc(&snoop_dma->fifo, snoop_dma->dma_size, GFP_KERNEL);
	if (rc)
		goto err_free_dma;

	snoop_dma->misc_dev.parent = dev;
	snoop_dma->misc_dev.name = devm_kasprintf(dev, GFP_KERNEL, "%s", DEVICE_NAME);
	snoop_dma->misc_dev.fops = &snoop_dma_fops;
	rc = misc_register(&snoop_dma->misc_dev);
	if (rc)
		goto err_free_kfifo;

	aspeed_snoop_dma_config(snoop_dma);

	/* skip FIFO cleanup if already enabled */
	if (!is_snoop_dma_enabled(snoop_dma))
		regmap_write_bits(snoop_dma->regmap, PCCR0,
				PCCR0_CLR_RX_FIFO, PCCR0_CLR_RX_FIFO);

	regmap_update_bits(snoop_dma->regmap, PCCR0,
		PCCR0_EN_DMA_INT | PCCR0_EN_DMA_MODE | PCCR0_EN,
		PCCR0_EN_DMA_INT | PCCR0_EN_DMA_MODE | PCCR0_EN);

	return 0;

err_free_kfifo:
	kfifo_free(&snoop_dma->fifo);
err_free_dma:
	if (snoop_dma->dma_addr_reserved)
		iounmap(snoop_dma->dma_virt);
	else
		dma_free_coherent(dev, snoop_dma->dma_size,
				snoop_dma->dma_virt, snoop_dma->dma_addr);
err_ret:
	return rc;
}

static int aspeed_snoop_dma_disable(struct aspeed_snoop_dma *snoop_dma, struct device *dev)
{
	regmap_update_bits(snoop_dma->regmap, PCCR0,
		PCCR0_EN_DMA_INT | PCCR0_EN_DMA_MODE | PCCR0_EN, 0);

	if (snoop_dma->dma_addr_reserved)
		iounmap(snoop_dma->dma_virt);
	else
		dma_free_coherent(dev, snoop_dma->dma_size,
				snoop_dma->dma_virt, snoop_dma->dma_addr);

	misc_deregister(&snoop_dma->misc_dev);
	kfifo_free(&snoop_dma->fifo);

	return 0;
}

static int aspeed_snoop_dma_probe(struct platform_device *pdev)
{
	int rc;

	struct aspeed_snoop_dma *snoop_dma;

	struct device *dev = &pdev->dev;
	struct device_node *node;

	struct resource res;

	snoop_dma = devm_kzalloc(&pdev->dev, sizeof(*snoop_dma), GFP_KERNEL);
	if (!snoop_dma) {
		dev_err(dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	snoop_dma->regmap = syscon_node_to_regmap(pdev->dev.parent->of_node);
	if (IS_ERR(snoop_dma->regmap)) {
		dev_err(dev, "failed to get regmap\n");
		return -ENODEV;
	}

	rc = of_property_read_u32(dev->of_node, "port-addr", &snoop_dma->port);
	if (rc) {
		dev_err(dev, "failed to get port base address\n");
		return rc;
	}

	/*
	 * optional, reserved memory for the DMA buffer
	 * if not specified, the DMA buffer is allocated
	 * dynamically.
	 */
	node = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (node) {
		rc = of_address_to_resource(node, 0, &res);
		if (rc) {
			dev_err(dev, "failed to get reserved memory region\n");
			return -ENOMEM;
		}
		snoop_dma->dma_addr = res.start;
		snoop_dma->dma_size = resource_size(&res);
		snoop_dma->dma_addr_reserved = 1;
		of_node_put(node);
	}

	/* optional, by default: 0 -> 1-Byte mode */
	of_property_read_u32(dev->of_node, "rec-mode", &snoop_dma->rec_mode);
	if (!is_valid_rec_mode(snoop_dma->rec_mode)) {
		dev_err(dev, "invalid record mode: %u\n",
				snoop_dma->rec_mode);
		return -EINVAL;
	}

	/* optional, by default: 0 -> no don't care bits */
	of_property_read_u32(dev->of_node, "port-addr-xbits", &snoop_dma->port_xbits);

	/* 
	 * optional, by default: 0 -> no high address bits 
	 *
	 * Note that when record mode is set to 1-Byte, this
	 * property is ignored and the corresponding HW bits
	 * behave as read/write cycle parser control with the
	 * value set to 0b11
	 */
	if (snoop_dma->rec_mode) {
		of_property_read_u32(dev->of_node, "port-addr-hbits-select", &snoop_dma->port_hbits_select);
		if (!is_valid_high_bits_select(snoop_dma->port_hbits_select)) {
			dev_err(dev, "invalid high address bits selection: %u\n",
				snoop_dma->port_hbits_select);
			return -EINVAL;
		}
	}
	else
		snoop_dma->port_hbits_select = 0x3;

	/* optional, pattern search A */
	if (of_property_read_bool(dev->of_node, "pattern-a-en")) {
		of_property_read_u32(dev->of_node, "pattern-a", &snoop_dma->pat_search[0].pattern);
		of_property_read_u32(dev->of_node, "pattern-a-len", &snoop_dma->pat_search[0].len);
		of_property_read_u32(dev->of_node, "pattern-a-write", &snoop_dma->pat_search[0].write);
		of_property_read_u32(dev->of_node, "pattern-a-port", &snoop_dma->pat_search[0].port);
		snoop_dma->pat_search[0].enable = 1;
	}

	/* optional, pattern search B */
	if (of_property_read_bool(dev->of_node, "pattern-b-en")) {
		of_property_read_u32(dev->of_node, "pattern-b", &snoop_dma->pat_search[1].pattern);
		of_property_read_u32(dev->of_node, "pattern-b-len", &snoop_dma->pat_search[1].len);
		of_property_read_u32(dev->of_node, "pattern-b-write", &snoop_dma->pat_search[1].write);
		of_property_read_u32(dev->of_node, "pattern-b-port", &snoop_dma->pat_search[1].port);
		snoop_dma->pat_search[1].enable = 1;
	}

	snoop_dma->irq = platform_get_irq(pdev, 0);
	if (!snoop_dma->irq) {
		dev_err(dev, "failed to get IRQ\n");
		return -ENODEV;
	}

	/*
	 * as snoop DMA may have been enabled in early stage,
	 * we have disable interrupts befor requesting IRQ to
	 * prevent kernel crash
	 */
	regmap_update_bits(snoop_dma->regmap, PCCR0,
			PCCR0_EN_DMA_INT | PCCR0_EN_PAT_A_INT | PCCR0_EN_PAT_B_INT
			, 0);

	rc = devm_request_irq(dev, snoop_dma->irq,
			aspeed_snoop_dma_irq, IRQF_SHARED,
			DEVICE_NAME, snoop_dma);
	if (rc < 0) {
		dev_err(dev, "failed to request IRQ handler\n");
		return rc;
	}

	tasklet_init(&snoop_dma->tasklet, aspeed_snoop_dma_tasklet,
			(unsigned long)snoop_dma);

	init_waitqueue_head(&snoop_dma->wq);

	rc = aspeed_snoop_dma_enable(snoop_dma, dev);
	if (rc) {
		dev_err(dev, "failed to enable snoop DMA\n");
		return rc;
	}

	snoop_dma->dev = dev;
	dev_set_drvdata(&pdev->dev, snoop_dma);
	return 0;
}

static int aspeed_snoop_dma_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aspeed_snoop_dma *snoop_dma = dev_get_drvdata(dev);
	aspeed_snoop_dma_disable(snoop_dma, dev);
	return 0;
}

static const struct of_device_id aspeed_snoop_dma_table[] = {
	{ .compatible = "aspeed,ast2500-lpc-snoop-dma" },
	{ .compatible = "aspeed,ast2600-lpc-snoop-dma" },
};

static struct platform_driver aspeed_snoop_dma_driver = {
	.driver = {
		.name = "aseepd-snoop-dma",
		.of_match_table = aspeed_snoop_dma_table,
	},
	.probe = aspeed_snoop_dma_probe,
	.remove = aspeed_snoop_dma_remove,
};

module_platform_driver(aspeed_snoop_dma_driver);

MODULE_AUTHOR("Chia-Wei Wang <chiawei_wang@aspeedtech.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver for Aspeed SNOOP DMA");
