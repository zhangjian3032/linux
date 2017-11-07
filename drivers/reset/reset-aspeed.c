/*
 * reset-aspeed.c - Reset Controller Driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/regmap.h>
#include <linux/types.h>

struct aspeed_reset_data {
	struct regmap *slcr;
	struct reset_controller_dev rcdev;
	u32 offset;
};

#define to_aspeed_reset_data(p)		\
	container_of((p), struct aspeed_reset_data, rcdev)

static int aspeed_reset_assert(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	struct aspeed_reset_data *priv = to_aspeed_reset_data(rcdev);
	
	pr_debug("%s: %s reset id %ld\n", KBUILD_MODNAME, __func__, id);

	return regmap_update_bits(priv->slcr,
					priv->offset,
					BIT(id),
					BIT(id));
}

static int aspeed_reset_deassert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	struct aspeed_reset_data *priv = to_aspeed_reset_data(rcdev);

	pr_debug("%s: %s reset id %ld\n", KBUILD_MODNAME, __func__, id);

	return regmap_update_bits(priv->slcr,
					priv->offset,
					BIT(id),
					~BIT(id));
}

static int aspeed_reset_status(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	struct aspeed_reset_data *priv = to_aspeed_reset_data(rcdev);
	int ret;
	u32 reg;

	pr_debug("%s: %s reset id %ld\n", KBUILD_MODNAME, __func__, id);

	ret = regmap_read(priv->slcr, priv->offset, &reg);
	if (ret)
		return ret;

	return !!(reg & BIT(id));
}

static const struct reset_control_ops aspeed_reset_ops = {
	.assert		= aspeed_reset_assert,
	.deassert	= aspeed_reset_deassert,
	.status		= aspeed_reset_status,
};

static int aspeed_reset_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct aspeed_reset_data *priv;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	platform_set_drvdata(pdev, priv);

	priv->slcr = syscon_node_to_regmap(pdev->dev.of_node->parent);
//	priv->slcr = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
//						     "syscon");

	if (IS_ERR(priv->slcr)) {
		dev_err(&pdev->dev, "unable to get ast-slcr regmap");
		return PTR_ERR(priv->slcr);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "missing IO resource\n");
		return -ENODEV;
	}

	priv->offset = res->start;

	priv->rcdev.owner = THIS_MODULE;
	priv->rcdev.nr_resets = resource_size(res) / 4 * BITS_PER_LONG;
	priv->rcdev.ops = &aspeed_reset_ops;
	priv->rcdev.of_node = pdev->dev.of_node;
	printk("aspeed_reset_probe end res->start %x, priv->rcdev.nr_resets %d \n", res->start, priv->rcdev.nr_resets);

	return devm_reset_controller_register(&pdev->dev, &priv->rcdev);
}

static const struct of_device_id aspeed_reset_dt_ids[] = {
	{ .compatible = "aspeed,ast-reset", },
	{ /* sentinel */ },
};

static struct platform_driver aspeed_reset_driver = {
	.probe	= aspeed_reset_probe,
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= aspeed_reset_dt_ids,
	},
};
module_platform_driver(aspeed_reset_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED Reset Controller Driver");
