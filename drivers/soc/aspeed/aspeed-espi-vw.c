// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) ASPEED Technology Inc.
 */
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/uaccess.h>

#include <soc/aspeed/espi.h>
#include <uapi/linux/aspeed-espi.h>

#define DEVICE_NAME	"aspeed-espi-vw"

struct aspeed_espi_vw {
	struct regmap *map;

	int irq;
	int irq_reset;

	struct miscdevice mdev;

	uint32_t version;
};

static long aspeed_espi_vw_ioctl(struct file *fp, unsigned int cmd,
				    unsigned long arg)
{
	uint32_t val;

	struct aspeed_espi_vw *espi_vw = container_of(
			fp->private_data,
			struct aspeed_espi_vw,
			mdev);

	switch (cmd) {
	case ASPEED_ESPI_VW_GET_GPIO_VAL:
		regmap_read(espi_vw->map, ESPI_VW_GPIO_VAL, &val);
		if (put_user(val, (uint32_t __user *)arg))
			return -EFAULT;
		break;

	case ASPEED_ESPI_VW_PUT_GPIO_VAL:
		if (get_user(val, (uint32_t __user *)arg))
			return -EFAULT;
		regmap_write(espi_vw->map, ESPI_VW_GPIO_VAL, val);
		break;

	default:
		return -EINVAL;
	};

	return 0;
}

static const struct file_operations aspeed_espi_vw_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = aspeed_espi_vw_ioctl,
};

static irqreturn_t aspeed_espi_vw_isr(int irq, void *arg)
{
	uint32_t sts;
	uint32_t sysevt_sts;

	struct aspeed_espi_vw *espi_vw = arg;

	regmap_read(espi_vw->map, ESPI_INT_STS, &sts);

	if (!(sts & ESPI_INT_STS_VW_BITS))
		return IRQ_NONE;

	if (sts & ESPI_INT_STS_VW_SYSEVT) {
		regmap_read(espi_vw->map, ESPI_SYSEVT_INT_STS, &sysevt_sts);

		if (espi_vw->version == ESPI_AST2500) {
			if (sysevt_sts & ESPI_SYSEVT_INT_STS_HOST_RST_WARN)
				regmap_update_bits(espi_vw->map, ESPI_SYSEVT,
						   ESPI_SYSEVT_HOST_RST_ACK,
						   ESPI_SYSEVT_HOST_RST_ACK);

			if (sysevt_sts & ESPI_SYSEVT_INT_STS_OOB_RST_WARN)
				regmap_update_bits(espi_vw->map, ESPI_SYSEVT,
						   ESPI_SYSEVT_OOB_RST_ACK,
						   ESPI_SYSEVT_OOB_RST_ACK);
		}

		regmap_write(espi_vw->map, ESPI_SYSEVT_INT_STS, sysevt_sts);
	}

	if (sts & ESPI_INT_STS_VW_SYSEVT1) {
		regmap_read(espi_vw->map, ESPI_SYSEVT1_INT_STS, &sysevt_sts);

		if (sysevt_sts & ESPI_SYSEVT1_INT_STS_SUSPEND_WARN)
			regmap_update_bits(espi_vw->map, ESPI_SYSEVT1,
					   ESPI_SYSEVT1_SUSPEND_ACK,
					   ESPI_SYSEVT1_SUSPEND_ACK);

		regmap_write(espi_vw->map, ESPI_SYSEVT1_INT_STS, sysevt_sts);
	}

	regmap_write(espi_vw->map, ESPI_INT_STS, sts & ESPI_INT_STS_VW_BITS);

	return IRQ_HANDLED;

}

static irqreturn_t aspeed_espi_vw_reset_isr(int irq, void *arg)
{
	return IRQ_HANDLED;
}

static int aspeed_espi_vw_init(struct platform_device *pdev,
		struct aspeed_espi_vw *espi_vw)
{
	int rc = 0;
	struct device *dev;

	dev = &pdev->dev;

	rc = devm_request_irq(dev, espi_vw->irq, aspeed_espi_vw_isr,
			0, DEVICE_NAME, espi_vw);
	if (rc) {
		dev_err(dev, "cannot request eSPI virtual wire channel irq\n");
		return rc;
	}

	rc = devm_request_irq(dev, espi_vw->irq_reset, aspeed_espi_vw_reset_isr,
			IRQF_SHARED, DEVICE_NAME, espi_vw);
	if (rc) {
		dev_err(dev, "cannot request eSPI channel reset irq\n");
		return rc;
	}

	return 0;
}

static int aspeed_espi_vw_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device *dev;
	struct aspeed_espi_vw *espi_vw;

	dev = &pdev->dev;

	espi_vw = devm_kzalloc(dev, sizeof(*espi_vw), GFP_KERNEL);
	if (!espi_vw)
		return -ENOMEM;

	espi_vw->map = syscon_node_to_regmap(
			dev->parent->of_node);
	if (IS_ERR(espi_vw->map)) {
		dev_err(dev, "cannot get regmap\n");
		return -ENODEV;
	}

	espi_vw->irq = platform_get_irq(pdev, 0);
	if (espi_vw->irq < 0)
		return espi_vw->irq;

	espi_vw->irq_reset = platform_get_irq(pdev, 1);
	if (espi_vw->irq_reset < 0)
		return espi_vw->irq_reset;

	espi_vw->version = (uint32_t)of_device_get_match_data(dev);

	rc = aspeed_espi_vw_init(pdev, espi_vw);
	if (rc)
		return rc;

	espi_vw->mdev.parent = dev;
	espi_vw->mdev.minor = MISC_DYNAMIC_MINOR;
	espi_vw->mdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s", DEVICE_NAME);
	espi_vw->mdev.fops = &aspeed_espi_vw_fops;
	rc = misc_register(&espi_vw->mdev);
	if (rc) {
		dev_err(dev, "cannot register device\n");
		return rc;
	}

	dev_set_drvdata(dev, espi_vw);

	dev_info(dev, "module loaded\n");

	return 0;
}

static int aspeed_espi_vw_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id aspeed_espi_vw_match[] = {
	{ .compatible = "aspeed,ast2500-espi-vw", .data = (void *)ESPI_AST2500, },
	{ .compatible = "aspeed,ast2600-espi-vw", .data = (void *)ESPI_AST2600, },
	{ }
};

static struct platform_driver aspeed_espi_vw_driver = {
	.driver = {
		.name           = DEVICE_NAME,
		.of_match_table = aspeed_espi_vw_match,
	},
	.probe  = aspeed_espi_vw_probe,
	.remove = aspeed_espi_vw_remove,
};

module_platform_driver(aspeed_espi_vw_driver);

MODULE_AUTHOR("Chia-Wei Wang <chiawei_wang@aspeedtech.com>");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("Control of eSPI virtual wire channel");
MODULE_LICENSE("GPL v2");
