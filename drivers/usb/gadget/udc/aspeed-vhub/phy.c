// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 Aspeed Technology Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <asm/io.h>

static int ast_phy_probe(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *reg;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	reg = devm_ioremap_resource(&pdev->dev, res);

	writel(readl(reg) | BIT(10), reg);

	dev_info(&pdev->dev, "Initialized USB PHYA\n");

	return 0;
}

static const struct of_device_id ast_phy_dt_ids[] = {
	{
		.compatible = "aspeed,ast2600-usb-phya",
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
