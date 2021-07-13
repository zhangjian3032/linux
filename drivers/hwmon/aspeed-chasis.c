// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2021 ASPEED Technology Inc.
 *
 * CHASIS driver for the Aspeed SoC
 */
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/interrupt.h>
/******************************************************************************/
union chasis_ctrl_register {
	uint32_t value;
	struct {
		uint32_t intrusion_status_clear : 1; /*[0]*/
		uint32_t intrusion_int_enable : 1; /*[1]*/
		uint32_t intrusion_status : 1; /*[2]*/
		uint32_t battery_power_good : 1; /*[3]*/
		uint32_t chasis_raw_status : 1; /*[4]*/
		uint32_t reserved0 : 3; /*[5-7]*/
		uint32_t io_power_status_clear : 1; /*[8]*/
		uint32_t io_power_int_enable : 1; /*[9]*/
		uint32_t core_power_status : 1; /*[10]*/
		uint32_t reserved1 : 5; /*[11-15]*/
		uint32_t core_power_status_clear : 1; /*[16]*/
		uint32_t core_power_int_enable : 1; /*[17]*/
		uint32_t io_power_status : 1; /*[18]*/
		uint32_t reserved2 : 13; /*[19-31]*/
	} fields;
};

struct aspeed_chasis {
	struct device *dev;
	void __iomem *base;
	int irq;
	/* for hwmon */
	const struct attribute_group *groups[2];
};

static ssize_t
intrusion_store(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	unsigned long val;
	struct aspeed_chasis *chasis = dev_get_drvdata(dev);
	union chasis_ctrl_register chasis_ctrl;

	if (kstrtoul(buf, 10, &val) < 0 || val != 0)
		return -EINVAL;

	chasis_ctrl.value = readl(chasis->base);
	chasis_ctrl.fields.intrusion_status_clear = 1;
	writel(chasis_ctrl.value, chasis->base);
	chasis_ctrl.fields.intrusion_status_clear = 0;
	writel(chasis_ctrl.value, chasis->base);
	return count;
}

static ssize_t intrusion_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int index = sensor_attr->index;
	struct aspeed_chasis *chasis = dev_get_drvdata(dev);
	union chasis_ctrl_register chasis_ctrl;
	uint8_t ret;

	chasis_ctrl.value = readl(chasis->base);

	switch (index) {
	case 0:
		ret = chasis_ctrl.fields.core_power_status;
		break;
	case 1:
		ret = chasis_ctrl.fields.io_power_status;
		break;
	case 2:
		ret = chasis_ctrl.fields.intrusion_status;
		break;
	}

	return sprintf(buf, "%d\n", ret);
}

static SENSOR_DEVICE_ATTR_RO(core_power, intrusion, 0);
static SENSOR_DEVICE_ATTR_RO(io_power, intrusion, 1);
static SENSOR_DEVICE_ATTR_RW(intrusion0_alarm, intrusion, 2);

static struct attribute *intrusion_dev_attrs[] = {
	&sensor_dev_attr_core_power.dev_attr.attr,
	&sensor_dev_attr_io_power.dev_attr.attr,
	&sensor_dev_attr_intrusion0_alarm.dev_attr.attr, NULL
};

static const struct attribute_group intrusion_dev_group = {
	.attrs = intrusion_dev_attrs,
	.is_visible = NULL,
};

static void aspeed_chasis_status_check(struct aspeed_chasis *chasis)
{
	union chasis_ctrl_register chasis_ctrl;

	chasis_ctrl.value = readl(chasis->base);
	if (chasis_ctrl.fields.intrusion_status) {
		dev_info(chasis->dev, "CHASI# pin has been pulled low");
		chasis_ctrl.fields.intrusion_status_clear = 1;
		writel(chasis_ctrl.value, chasis->base);
		chasis_ctrl.fields.intrusion_status_clear = 0;
		writel(chasis_ctrl.value, chasis->base);
	}
	chasis_ctrl.value = readl(chasis->base);
	if (chasis_ctrl.fields.core_power_status) {
		dev_info(chasis->dev, "Core power has been pulled low");
		chasis_ctrl.fields.core_power_status_clear = 1;
		writel(chasis_ctrl.value, chasis->base);
		chasis_ctrl.fields.core_power_status_clear = 0;
		writel(chasis_ctrl.value, chasis->base);
	}

	chasis_ctrl.value = readl(chasis->base);
	if (chasis_ctrl.fields.io_power_status) {
		dev_info(chasis->dev, "IO power has been pulled low");
		chasis_ctrl.fields.io_power_status_clear = 1;
		writel(chasis_ctrl.value, chasis->base);
		chasis_ctrl.fields.io_power_status_clear = 0;
		writel(chasis_ctrl.value, chasis->base);
	}
}

static irqreturn_t aspeed_chasis_isr(int this_irq, void *dev_id)
{
	struct aspeed_chasis *chasis = dev_id;

	aspeed_chasis_status_check(chasis);
	return IRQ_HANDLED;
}

static const struct of_device_id aspeed_chasis_of_table[] = {
	{ .compatible = "aspeed,ast2600-chasis" },
	{}
};
MODULE_DEVICE_TABLE(of, aspeed_chasis_of_table);

static int aspeed_chasis_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aspeed_chasis *priv;
	struct device *hwmon;
	int ret;

	dev_info(dev, "aspeed_chasis_probe\n");

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0) {
		dev_err(dev, "no irq specified\n");
		ret = -ENOENT;
		return ret;
	}

	ret = devm_request_irq(dev, priv->irq, aspeed_chasis_isr, 0,
			       dev_name(dev), priv);
	if (ret) {
		dev_err(dev, "Chasis Unable to get IRQ");
		return ret;
	}

	priv->groups[0] = &intrusion_dev_group;
	priv->groups[1] = NULL;

	hwmon = devm_hwmon_device_register_with_groups(dev, "aspeed_chasis",
						       priv, priv->groups);

	dev_info(dev, "chasis driver probe done.\n");

	return PTR_ERR_OR_ZERO(hwmon);
}

static struct platform_driver aspeed_chasis_driver = {
	.probe		= aspeed_chasis_probe,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = aspeed_chasis_of_table,
	},
};

module_platform_driver(aspeed_chasis_driver);

MODULE_AUTHOR("Billy Tsai<billy_tsai@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED CHASIS Driver");
MODULE_LICENSE("GPL v2");
