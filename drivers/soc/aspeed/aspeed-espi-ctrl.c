// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2020 Aspeed Technology Inc.
 */
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include <soc/aspeed/espi.h>

#define DEVICE_NAME "aspeed-espi-ctrl"

struct aspeed_espi_ctrl {
	struct regmap *map;
	struct clk *clk;
	struct reset_control *reset;

	int irq;
	int irq_reset;

	uint32_t version;
};

static void aspeed_espi_ctrl_init(struct aspeed_espi_ctrl *espi_ctrl)
{
	regmap_write(espi_ctrl->map, ESPI_SYSEVT_INT_T0, 0x0);
	regmap_write(espi_ctrl->map, ESPI_SYSEVT_INT_T1, 0x0);

	regmap_write(espi_ctrl->map, ESPI_SYSEVT_INT_EN, 0xffffffff);

	regmap_write(espi_ctrl->map, ESPI_SYSEVT1_INT_T0, 0x1);
	regmap_write(espi_ctrl->map, ESPI_SYSEVT1_INT_EN, 0x1);

	if (espi_ctrl->version == ESPI_AST2500) {
		regmap_write(espi_ctrl->map, ESPI_SYSEVT_INT_T2,
			     ESPI_SYSEVT_INT_T2_HOST_RST_WARN
			     | ESPI_SYSEVT_INT_T2_OOB_RST_WARN);
		regmap_update_bits(espi_ctrl->map, ESPI_CTRL, 0xff, 0xff);
	}
	else {
		regmap_update_bits(espi_ctrl->map, ESPI_CTRL, 0xef, 0xef);
	}
}

static irqreturn_t aspeed_espi_ctrl_isr(int irq, void *arg)
{
	uint32_t sts;
	struct aspeed_espi_ctrl *espi_ctrl = (struct aspeed_espi_ctrl*)arg;

	regmap_read(espi_ctrl->map, ESPI_INT_STS, &sts);
	if (!(sts & ESPI_INT_STS_HW_RST_DEASSERT))
		return IRQ_NONE;

	regmap_update_bits(espi_ctrl->map, ESPI_SYSEVT,
			   ESPI_SYSEVT_SLV_BOOT_STS | ESPI_SYSEVT_SLV_BOOT_DONE,
			   ESPI_SYSEVT_SLV_BOOT_STS | ESPI_SYSEVT_SLV_BOOT_DONE);
	regmap_write(espi_ctrl->map, ESPI_INT_STS, ESPI_INT_STS_HW_RST_DEASSERT);

	return IRQ_HANDLED;
}

static irqreturn_t aspeed_espi_ctrl_reset_isr(int irq, void *arg)
{
	struct aspeed_espi_ctrl *espi_ctrl = (struct aspeed_espi_ctrl*)arg;

	/* for AST2600 A1/A0, DO NOT toggle reset in case OOB_FREE cannot be set */
	//reset_control_assert(espi_ctrl->reset);
	//reset_control_deassert(espi_ctrl->reset);
	aspeed_espi_ctrl_init(espi_ctrl);

	return IRQ_HANDLED;
}

static int aspeed_espi_ctrl_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct aspeed_espi_ctrl *espi_ctrl;
	struct device *dev;

	dev = &pdev->dev;

	espi_ctrl = devm_kzalloc(dev, sizeof(*espi_ctrl), GFP_KERNEL);
	if (!espi_ctrl)
		return -ENOMEM;

	espi_ctrl->map = syscon_node_to_regmap(
			dev->parent->of_node);
	if (IS_ERR(espi_ctrl->map)) {
		dev_err(dev, "cannot get remap\n");
		return -ENODEV;
	}

	espi_ctrl->irq = platform_get_irq(pdev, 0);
	if (espi_ctrl->irq < 0)
		return espi_ctrl->irq;

	espi_ctrl->irq_reset = platform_get_irq(pdev, 1);
	if (espi_ctrl->irq_reset < 0)
		return espi_ctrl->irq_reset;

	espi_ctrl->reset = devm_reset_control_get(dev, NULL);
	if (IS_ERR(espi_ctrl->reset)) {
		dev_err(dev, "cannot get reset\n");
		return -ENODEV;
	}

	espi_ctrl->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(espi_ctrl->clk)) {
		dev_err(dev, "cannot get clock\n");
		return -ENODEV;
	}

	rc = clk_prepare_enable(espi_ctrl->clk);
	if (rc) {
		dev_err(dev, "cannot enable clock\n");
		return rc;
	}

	espi_ctrl->version = (uint32_t)of_device_get_match_data(dev);

	rc = devm_request_irq(dev, espi_ctrl->irq,
			      aspeed_espi_ctrl_isr,
			      0, DEVICE_NAME, espi_ctrl);
	if (rc) {
		dev_err(dev, "failed to request IRQ\n");
		return rc;
	}

	rc = devm_request_irq(dev, espi_ctrl->irq_reset,
			      aspeed_espi_ctrl_reset_isr,
			      IRQF_SHARED, DEVICE_NAME, espi_ctrl);
	if (rc) {
		dev_err(dev, "failed to request reset IRQ\n");
		return rc;
	}

	aspeed_espi_ctrl_init(espi_ctrl);

	dev_set_drvdata(dev, espi_ctrl);

	dev_info(dev, "module loaded\n");

	return 0;
}

static int aspeed_espi_ctrl_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aspeed_espi_ctrl *espi_ctrl = dev_get_drvdata(dev);

	devm_free_irq(dev, espi_ctrl->irq, espi_ctrl);
	devm_free_irq(dev, espi_ctrl->irq_reset, espi_ctrl);
	devm_kfree(dev, espi_ctrl);
	return 0;
}

static const struct of_device_id aspeed_espi_ctrl_of_matches[] = {
	{ .compatible = "aspeed,ast2500-espi-ctrl", .data = (void *)ESPI_AST2500 },
	{ .compatible = "aspeed,ast2600-espi-ctrl", .data = (void *)ESPI_AST2600 },
	{ },
};

static struct platform_driver aspeed_espi_ctrl_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = aspeed_espi_ctrl_of_matches,
	},
	.probe = aspeed_espi_ctrl_probe,
	.remove = aspeed_espi_ctrl_remove,
};

module_platform_driver(aspeed_espi_ctrl_driver);

MODULE_AUTHOR("Chia-Wei Wang <chiawei_wang@aspeedtech.com>");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("Control of Aspeed eSPI reset and clocks");
MODULE_LICENSE("GPL v2");
