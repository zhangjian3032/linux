// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright ASPEED Tech Corporation
 */

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/dma-mapping.h>


#define DEVICE_NAME	"aspeed-dac_ctrl"

enum aspeed_dac_version {
	DAC_AST2400,
	DAC_AST2500,
	DAC_AST2600,
};

struct aspeed_dac_ctrl {
	int				version;
	int 			pcie_l2h_irq;
	int 			pcie_h2l_irq;
	struct regmap	*scu;
	struct regmap	*pcie;
};

static irqreturn_t aspeed_pcie_l2h_handle(int irq, void *arg)
{
	struct aspeed_dac_ctrl *dac = arg;
	u32 val0, val1;
	int rc;

	rc = regmap_read(dac->pcie, 0xc4, &val0);
	if (rc)
		return IRQ_NONE;

	rc = regmap_read(dac->pcie, 0xc0, &val1);
	if (rc)
		return IRQ_NONE;

	/* 1: pcie host power on */
	printk("pcie host power on %x %x version %d \n", val0, val1, dac->version);

	if(dac->version == DAC_AST2600)
		regmap_update_bits(dac->scu, 0xc0, BIT(16), 0);
	else
		regmap_update_bits(dac->scu, 0x2c, BIT(16), 0);

	return IRQ_HANDLED;
}

static irqreturn_t aspeed_pcie_h2l_handle(int irq, void *arg)
{
	struct aspeed_dac_ctrl *dac = arg;
	u32 val0, val1;
	int rc;

	rc = regmap_read(dac->pcie, 0xc4, &val0);
	if (rc)
		return IRQ_NONE;

	rc = regmap_read(dac->pcie, 0xc0, &val1);
	if (rc)
		return IRQ_NONE;

	/* 1: pcie host power on */
	printk("pcie host power on %x %x version %d \n", val0, val1, dac->version);

	if(dac->version == DAC_AST2600)
		regmap_update_bits(dac->scu, 0xc0, BIT(16), BIT(16));
	else
		regmap_update_bits(dac->scu, 0x2c, BIT(16), BIT(16));

	return IRQ_HANDLED;
}

static ssize_t dac_mux_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct aspeed_dac_ctrl *dac = dev_get_drvdata(dev);

	u32 val;
	int rc;

	if(regmap_read(dac->pcie, 0xc4, &val))
		return 0;

	/* 1: pcie host power on */
	if(val & BIT(19))
		return 0;
	
	rc = kstrtou32(buf, 0, &val);
	if (rc)
		return rc;

	if (val > 3)
		return -EINVAL;

	if(dac->version == DAC_AST2600)
		rc = regmap_update_bits(dac->scu, 0xc0, BIT(16), val << 16);
	else 
		rc = regmap_update_bits(dac->scu, 0x2c, BIT(16), val << 16);
	if (rc < 0)
		return 0;

	return count;
}

static ssize_t dac_mux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aspeed_dac_ctrl *dac = dev_get_drvdata(dev);
	u32 reg;
	int rc;

	if(dac->version == DAC_AST2600)
		rc = regmap_read(dac->scu, 0xc0, &reg);
	else 
		rc = regmap_read(dac->scu, 0x2c, &reg);
	
	if (rc)
		return rc;

	return sprintf(buf, "%u\n", (reg >> 16) & 0x3);
}
static DEVICE_ATTR_RW(dac_mux);

static ssize_t
vga_pw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct aspeed_dac_ctrl *dac = dev_get_drvdata(dev);
	u32 reg;
	int rc;

	if(dac->version == DAC_AST2600)
		rc = regmap_read(dac->scu, 0xe00, &reg);
	else
		rc = regmap_read(dac->scu, 0x50, &reg);
	if (rc)
		return rc;

	return sprintf(buf, "%u\n", reg & 1);
}
static DEVICE_ATTR_RO(vga_pw);

static struct attribute *dac_sysfs_entries[] = {
	&dev_attr_vga_pw.attr,
	&dev_attr_dac_mux.attr,
	NULL,
};

static struct attribute_group dac_sysfs_attr_group = {
	.attrs = dac_sysfs_entries,
};

static const struct of_device_id aspeed_dac_ctrl_match[] = {
	{ .compatible = "aspeed,ast2400-dac-ctrl", .data = (void *) DAC_AST2400},
	{ .compatible = "aspeed,ast2500-dac-ctrl", .data = (void *) DAC_AST2500},
	{ .compatible = "aspeed,ast2600-dac-ctrl", .data = (void *) DAC_AST2600},
	{ },
};

static int aspeed_dac_ctrl_probe(struct platform_device *pdev)
{
	struct aspeed_dac_ctrl *dac_ctrl;
	const struct of_device_id *dev_id;	
	struct device *dev;
	int rc;

	dev = &pdev->dev;

	dac_ctrl = devm_kzalloc(dev, sizeof(*dac_ctrl), GFP_KERNEL);
	if (!dac_ctrl)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, dac_ctrl);

	dev_id = of_match_device(aspeed_dac_ctrl_match, &pdev->dev);
	if (!dev_id)
		return -EINVAL;

	dac_ctrl->version = (int)dev_id->data;

	dac_ctrl->pcie_l2h_irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (dac_ctrl->pcie_l2h_irq < 0) {
		dev_err(&pdev->dev, "no l2h irq specified\n");
		return dac_ctrl->pcie_l2h_irq;
	}

	dac_ctrl->pcie_h2l_irq = irq_of_parse_and_map(pdev->dev.of_node, 1);
	if (dac_ctrl->pcie_h2l_irq < 0) {
		dev_err(&pdev->dev, "no h2l irq specified\n");
		return dac_ctrl->pcie_h2l_irq;
	}

	dac_ctrl->scu = syscon_regmap_lookup_by_compatible("aspeed,aspeed-scu");
	if (IS_ERR(dac_ctrl->scu)) {
		dev_err(&pdev->dev, "failed to find SCU regmap\n");
		return PTR_ERR(dac_ctrl->scu);
	}

	dac_ctrl->pcie = syscon_regmap_lookup_by_compatible("aspeed,aspeed-pcie");
	if (IS_ERR(dac_ctrl->pcie)) {
		dev_err(&pdev->dev, "failed to find PCIe regmap\n");
		return PTR_ERR(dac_ctrl->pcie);
	}

	rc = sysfs_create_group(&pdev->dev.kobj, &dac_sysfs_attr_group);
	if (rc)
		return rc;

	rc = devm_request_irq(&pdev->dev, dac_ctrl->pcie_l2h_irq, aspeed_pcie_l2h_handle, IRQF_SHARED,
				dev_name(&pdev->dev), dac_ctrl);
	if (rc) {
		printk("Unable to get l2h IRQ \n");
		return rc;
	}

	rc = devm_request_irq(&pdev->dev, dac_ctrl->pcie_h2l_irq, aspeed_pcie_h2l_handle, IRQF_SHARED,
				dev_name(&pdev->dev), dac_ctrl);
	if (rc) {
		printk("Unable to get h2l IRQ \n");
		return rc;
	}

	return 0;
}

static int aspeed_dac_ctrl_remove(struct platform_device *pdev)
{
	struct aspeed_dac_ctrl *dac_ctrl = dev_get_drvdata(&pdev->dev);

	free_irq(dac_ctrl->pcie_l2h_irq, dac_ctrl);
	free_irq(dac_ctrl->pcie_h2l_irq, dac_ctrl);

	return 0;
}

static struct platform_driver aspeed_dac_ctrl_driver = {
	.driver = {
		.name		= DEVICE_NAME,
		.of_match_table = aspeed_dac_ctrl_match,
	},
	.probe = aspeed_dac_ctrl_probe,
	.remove = aspeed_dac_ctrl_remove,
};

module_platform_driver(aspeed_dac_ctrl_driver);

MODULE_DEVICE_TABLE(of, aspeed_dac_ctrl_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("Control for aspeed DAC switch driver");
