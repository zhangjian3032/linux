/*
 * Copyright (C) ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or later as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/sysfs.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/thermal.h>
/**********************************************************
 * Tach HW register offset define
 *********************************************************/
//TACH Control Register
#define ASPEED_TACHO_CTRL_CH(ch)		((ch * 0x10) + 0x08)
//TACH Status Register
#define ASPEED_TACHO_STS_CH(x)			((x * 0x10) + 0x0C)
/**********************************************************
 * Tach register Bit field 
 *********************************************************/
/*PWM_TACHO_CTRL */
#define  TACHO_IER						BIT(31)	//enable tacho interrupt
#define  TACHO_INVERS_LIMIT				BIT(30) //inverse tacho limit comparison
#define  TACHO_LOOPBACK					BIT(29) //tacho loopback
#define  TACHO_ENABLE					BIT(28)	//{enable tacho}
#define  TACHO_DEBOUNCE_MASK			(0x3 << 26) //{tacho de-bounce}
#define  TACHO_DEBOUNCE_BIT				(26) //{tacho de-bounce}
#define  TECHIO_EDGE_MASK				(0x3 << 24) //tacho edge}
#define  TECHIO_EDGE_BIT				(24) //tacho edge}
#define  TACHO_CLK_DIV_T_MASK			(0xf << 20) 
#define  TACHO_CLK_DIV_BIT				(20)
#define  TACHO_THRESHOLD_MASK			(0xfffff)	//tacho threshold bit
/* [27:26] */
#define DEBOUNCE_3_CLK 0x00 /* 10b */
#define DEBOUNCE_2_CLK 0x01 /* 10b */
#define DEBOUNCE_1_CLK 0x02 /* 10b */
#define DEBOUNCE_0_CLK 0x03 /* 10b */
/* [25:24] */
#define F2F_EDGES 0x00 /* 10b */
#define R2R_EDGES 0x01 /* 10b */
#define BOTH_EDGES 0x02 /* 10b */
/* [23:20] */
/* Cover rpm range 5~5859375 */
#define  DEFAULT_TACHO_DIV 5

/*PWM_TACHO_STS */
#define  TACHO_ISR			BIT(31)	//interrupt status and clear
#define  PWM_OUT			BIT(25)	//{pwm_out}
#define  PWM_OEN			BIT(24)	//{pwm_oeN}
#define  TACHO_DEB_INPUT	BIT(23)	//tacho deB input
#define  TACHO_RAW_INPUT	BIT(22) //tacho raw input}
#define  TACHO_VALUE_UPDATE	BIT(21)	//tacho value updated since the last read
#define  TACHO_FULL_MEASUREMENT	BIT(20) //{tacho full measurement}
#define  TACHO_VALUE_MASK	0xfffff	//tacho value bit [19:0]}
/**********************************************************
 * Software setting
 *********************************************************/
#define DEFAULT_FAN_MIN_RPM 1000
#define DEFAULT_FAN_PULSE_PR 2

struct aspeed_tacho_channel_params {
	int limited_inverse;
	u16 threshold;
	u8	tacho_edge;
	u8	tacho_debounce;
	u8  pulse_pr;
	u32 min_rpm;
	u32 divide;
	u32 sample_period; /* unit is us */
};

struct aspeed_tach_data {
	struct regmap *regmap;
	unsigned long clk_freq;
	struct reset_control *reset;
	bool tach_present[16];
	struct aspeed_tacho_channel_params *tacho_channel;
	/* for hwmon */
	const struct attribute_group *groups[1];
};

static int regmap_aspeed_tach_reg_write(void *context, unsigned int reg,
					     unsigned int val)
{
	void __iomem *regs = (void __iomem *)context;

	writel(val, regs + reg);
	return 0;
}

static int regmap_aspeed_tach_reg_read(void *context, unsigned int reg,
					    unsigned int *val)
{
	void __iomem *regs = (void __iomem *)context;

	*val = readl(regs + reg);
	return 0;
}

static const struct regmap_config aspeed_tach_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x100,
	.reg_write = regmap_aspeed_tach_reg_write,
	.reg_read = regmap_aspeed_tach_reg_read,
	.fast_io = true,
};

static u32
aspeed_get_fan_tach_sample_period(struct aspeed_tach_data *priv, u8 fan_tach_ch)
{
	u32 tach_period_us;
	u8 pulse_pr = priv->tacho_channel[fan_tach_ch].pulse_pr;
	u32 min_rpm = priv->tacho_channel[fan_tach_ch].min_rpm;
	/* 
	 * min(Tach input clock) = (PulsePR * minRPM) / 60
	 * max(Tach input period) = 60 / (PulsePR * minRPM)
	 * Tach sample period > 2 * max(Tach input period) = (2*60) / (PulsePR * minRPM)
	 */
	tach_period_us = (1000000 * 2 * 60) / (pulse_pr * min_rpm);
	/* Add the margin (about 1.2) of tach sample period to avoid sample miss */
	tach_period_us = (tach_period_us * 1200) >> 10;
	printk(KERN_DEBUG "tach%d sample period = %dus", fan_tach_ch, tach_period_us);
	return tach_period_us;
}

static void
aspeed_set_fan_tach_ch_enable(struct aspeed_tach_data *priv,
			      u8 fan_tach_ch, bool enable, u32 tacho_div)
{
	u32 reg_value = 0;

	if(enable) {
		/* divide = 2^(tacho_div*2) */
		priv->tacho_channel[fan_tach_ch].divide = 1 << (tacho_div << 1);

		reg_value = TACHO_ENABLE | 
				(priv->tacho_channel[fan_tach_ch].tacho_edge << TECHIO_EDGE_BIT) |
				(tacho_div << TACHO_CLK_DIV_BIT) |
				(priv->tacho_channel[fan_tach_ch].tacho_debounce << TACHO_DEBOUNCE_BIT);

		if(priv->tacho_channel[fan_tach_ch].limited_inverse)
			reg_value |= TACHO_INVERS_LIMIT;

		if(priv->tacho_channel[fan_tach_ch].threshold)
			reg_value |= (TACHO_IER | priv->tacho_channel[fan_tach_ch].threshold); 

		regmap_write(priv->regmap, ASPEED_TACHO_CTRL_CH(fan_tach_ch), reg_value);

		priv->tacho_channel[fan_tach_ch].sample_period =
			aspeed_get_fan_tach_sample_period(priv, fan_tach_ch);
	} else
		regmap_update_bits(priv->regmap, ASPEED_TACHO_CTRL_CH(fan_tach_ch),  TACHO_ENABLE, 0);
}

static int aspeed_get_fan_tach_ch_rpm(struct aspeed_tach_data *priv,
				      u8 fan_tach_ch)
{
	u32 raw_data, tach_div, clk_source, usec, val;
	int ret;

	usec = priv->tacho_channel[fan_tach_ch].sample_period;
	ret = regmap_read_poll_timeout(
		priv->regmap, ASPEED_TACHO_STS_CH(fan_tach_ch), val,
		(val & TACHO_FULL_MEASUREMENT) && (val & TACHO_VALUE_UPDATE), 0,
		usec);

	/* return -ETIMEDOUT if we didn't get an answer. */
	if (ret)
		return ret;
	
	raw_data = val & TACHO_VALUE_MASK;
	/*
	 * We need the mode to determine if the raw_data is double (from
	 * counting both edges).
	 */
	if (priv->tacho_channel[fan_tach_ch].tacho_edge == BOTH_EDGES)
		raw_data <<= 1;
	
	tach_div = raw_data * (priv->tacho_channel[fan_tach_ch].divide) * (priv->tacho_channel[fan_tach_ch].pulse_pr);

//	printk("clk %ld, raw_data %d , tach_div %d  \n", priv->clk_freq, raw_data, tach_div);
	
	clk_source = priv->clk_freq;

	if (tach_div == 0)
		return -EDOM;

	return ((clk_source / tach_div) * 60);

}

static ssize_t show_rpm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int index = sensor_attr->index;
	int rpm;
	struct aspeed_tach_data *priv = dev_get_drvdata(dev);

	rpm = aspeed_get_fan_tach_ch_rpm(priv, index);
	if (rpm < 0)
		return rpm;

	return sprintf(buf, "%d\n", rpm);
}

static umode_t fan_dev_is_visible(struct kobject *kobj,
				  struct attribute *a, int index)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct aspeed_tach_data *priv = dev_get_drvdata(dev);

	if (!priv->tach_present[index])
		return 0;
	return a->mode;
}

static SENSOR_DEVICE_ATTR(fan0, 0444,
		show_rpm, NULL, 0);
static SENSOR_DEVICE_ATTR(fan1, 0444,
		show_rpm, NULL, 1);
static SENSOR_DEVICE_ATTR(fan2, 0444,
		show_rpm, NULL, 2);
static SENSOR_DEVICE_ATTR(fan3, 0444,
		show_rpm, NULL, 3);
static SENSOR_DEVICE_ATTR(fan4, 0444,
		show_rpm, NULL, 4);
static SENSOR_DEVICE_ATTR(fan5, 0444,
		show_rpm, NULL, 5);
static SENSOR_DEVICE_ATTR(fan6, 0444,
		show_rpm, NULL, 6);
static SENSOR_DEVICE_ATTR(fan7, 0444,
		show_rpm, NULL, 7);
static SENSOR_DEVICE_ATTR(fan8, 0444,
		show_rpm, NULL, 8);
static SENSOR_DEVICE_ATTR(fan9, 0444,
		show_rpm, NULL, 9);
static SENSOR_DEVICE_ATTR(fan10, 0444,
		show_rpm, NULL, 10);
static SENSOR_DEVICE_ATTR(fan11, 0444,
		show_rpm, NULL, 11);
static SENSOR_DEVICE_ATTR(fan12, 0444,
		show_rpm, NULL, 12);
static SENSOR_DEVICE_ATTR(fan13, 0444,
		show_rpm, NULL, 13);
static SENSOR_DEVICE_ATTR(fan14, 0444,
		show_rpm, NULL, 14);
static SENSOR_DEVICE_ATTR(fan15, 0444,
		show_rpm, NULL, 15);
static struct attribute *fan_dev_attrs[] = {
	&sensor_dev_attr_fan0.dev_attr.attr,
	&sensor_dev_attr_fan1.dev_attr.attr,
	&sensor_dev_attr_fan2.dev_attr.attr,
	&sensor_dev_attr_fan3.dev_attr.attr,
	&sensor_dev_attr_fan4.dev_attr.attr,
	&sensor_dev_attr_fan5.dev_attr.attr,
	&sensor_dev_attr_fan6.dev_attr.attr,
	&sensor_dev_attr_fan7.dev_attr.attr,
	&sensor_dev_attr_fan8.dev_attr.attr,
	&sensor_dev_attr_fan9.dev_attr.attr,
	&sensor_dev_attr_fan10.dev_attr.attr,
	&sensor_dev_attr_fan11.dev_attr.attr,
	&sensor_dev_attr_fan12.dev_attr.attr,
	&sensor_dev_attr_fan13.dev_attr.attr,
	&sensor_dev_attr_fan14.dev_attr.attr,
	&sensor_dev_attr_fan15.dev_attr.attr,
	NULL
};

static const struct attribute_group fan_dev_group = {
	.attrs = fan_dev_attrs,
	.is_visible = fan_dev_is_visible,
};

static void
aspeed_create_fan_tach_channel(struct aspeed_tach_data *priv,
			       u32 tach_ch, int count, u32 fan_pulse_pr,
			       u32 fan_min_rpm, u32 tacho_div)
{
    priv->tach_present[tach_ch] = true;
    priv->tacho_channel[tach_ch].pulse_pr = fan_pulse_pr;
    priv->tacho_channel[tach_ch].min_rpm = fan_min_rpm;
    priv->tacho_channel[tach_ch].limited_inverse = 0;
    priv->tacho_channel[tach_ch].threshold = 0;
    priv->tacho_channel[tach_ch].tacho_edge = F2F_EDGES;
    priv->tacho_channel[tach_ch].tacho_debounce = DEBOUNCE_3_CLK;
    aspeed_set_fan_tach_ch_enable(priv, tach_ch, true, tacho_div);
}

static int aspeed_tach_create_fan(struct device *dev,
			     struct device_node *child,
			     struct aspeed_tach_data *priv)
{
	u32 fan_pulse_pr, fan_min_rpm;
	u32 tacho_div;
	u32 tach_channel;
	int ret, count;

	ret = of_property_read_u32(child, "aspeed,tach-ch", &tach_channel);
	if (ret)
		return ret;

	ret = of_property_read_u32(child, "aspeed,pulse-pr", &fan_pulse_pr);
	if (ret)
		fan_pulse_pr = DEFAULT_FAN_PULSE_PR;

	ret = of_property_read_u32(child, "aspeed,min-rpm", &fan_min_rpm);
	if (ret)
		fan_min_rpm = DEFAULT_FAN_MIN_RPM;

	ret = of_property_read_u32(child, "aspeed,tach-div", &tacho_div);
	if (ret)
		tacho_div = DEFAULT_TACHO_DIV;

	aspeed_create_fan_tach_channel(priv, tach_channel, count, fan_pulse_pr,
				       fan_min_rpm, tacho_div);

	return 0;
}

static int aspeed_tach_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np, *child;
	struct aspeed_tach_data *priv;
	struct device *hwmon;
	struct clk *clk;
	int ret;
	np = dev->of_node;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->tacho_channel =
		devm_kzalloc(dev, 16 * sizeof(*priv->tacho_channel), GFP_KERNEL);

	priv->regmap = syscon_node_to_regmap(
			pdev->dev.parent->of_node);
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "Couldn't get regmap\n");
		return -ENODEV;
	}

	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk))
		return -ENODEV;
	priv->clk_freq = clk_get_rate(clk);

	priv->reset = reset_control_get_shared(&pdev->dev, NULL);
	if (IS_ERR(priv->reset)) {
		dev_err(&pdev->dev, "can't get aspeed_pwm_tacho reset\n");
		return PTR_ERR(priv->reset);
	}

	//scu init
	reset_control_deassert(priv->reset);

	for_each_child_of_node(np, child) {
		ret = aspeed_tach_create_fan(dev, child, priv);
		if (ret) {
			of_node_put(child);
			return ret;
		}
	}

	priv->groups[0] = &fan_dev_group;
	priv->groups[1] = NULL;
	dev_info(dev, "tach probe done\n");
	hwmon = devm_hwmon_device_register_with_groups(dev,
						       "aspeed_tach",
						       priv, priv->groups);

	return PTR_ERR_OR_ZERO(hwmon);
}

static const struct of_device_id of_stach_match_table[] = {
	{ .compatible = "aspeed,ast2600-tach", },
	{},
};
MODULE_DEVICE_TABLE(of, of_stach_match_table);

static struct platform_driver aspeed_tach_driver = {
	.probe		= aspeed_tach_probe,
	.driver		= {
		.name	= "aspeed_tach",
		.of_match_table = of_stach_match_table,
	},
};

module_platform_driver(aspeed_tach_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED Fan tach device driver");
MODULE_LICENSE("GPL");
