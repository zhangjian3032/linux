// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 Aspeed Technology Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <asm/io.h>

static int ast_phy_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	void __iomem *uphya_reg;
	void __iomem *uphyb_reg;

	uphya_reg = of_iomap(node, 0);
	uphyb_reg = of_iomap(node, 1);

	writel(readl(uphya_reg) | BIT(10), uphya_reg);
	writel(readl(uphyb_reg) | BIT(8), uphyb_reg);

	dev_info(&pdev->dev, "Initialized USB PHYA/B\n");

	return 0;
}

static const struct of_device_id ast_phy_dt_ids[] = {
	{
		.compatible = "aspeed,ast2600-usb-phy",
	},
};

static struct platform_driver ast_phy_driver = {
	.probe		= ast_phy_probe,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table	= ast_phy_dt_ids,
	},
};
module_platform_driver(ast_phy_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Neal Liu <neal_liu@aspeedtech.com>");
