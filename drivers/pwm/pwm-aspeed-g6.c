/*
 * Copyright (C) ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or later as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/pwm.h>
/**********************************************************
 * PWM HW register offset define
 *********************************************************/
//PWM Control Register
#define ASPEED_PWM_CTRL_CH(ch) ((ch * 0x10) + 0x00)
//PWM Duty Cycle Register
#define ASPEED_PWM_DUTY_CYCLE_CH(ch) ((ch * 0x10) + 0x04)
/**********************************************************
 * PWM register Bit field
 *********************************************************/
/*PWM_CTRL */
#define PWM_LOAD_SEL_AS_WDT_BIT (19) //load selection as WDT
#define PWM_DUTY_LOAD_AS_WDT_EN BIT(18) //enable PWM duty load as WDT
#define PWM_DUTY_SYNC_DIS BIT(17) //disable PWM duty sync
#define PWM_CLK_ENABLE BIT(16) //enable PWM clock
#define PWM_LEVEL_OUTPUT BIT(15) //output PWM level
#define PWM_INVERSE BIT(14) //inverse PWM pin
#define PWM_OPEN_DRAIN_EN BIT(13) //enable open-drain
#define PWM_PIN_EN BIT(12) //enable PWM pin
#define PWM_CLK_DIV_H_MASK (0xf << 8) //PWM clock division H bit [3:0]
#define PWM_CLK_DIV_L_MASK (0xff) //PWM clock division H bit [3:0]
/* [19] */
#define LOAD_SEL_FALLING 0
#define LOAD_SEL_RIGING 1

/*PWM_DUTY_CYCLE */
#define PWM_PERIOD_BIT (24) //pwm period bit [7:0]
#define PWM_PERIOD_BIT_MASK (0xff << 24) //pwm period bit [7:0]
#define PWM_RISING_FALLING_AS_WDT_BIT (16)
#define PWM_RISING_FALLING_AS_WDT_MASK                                         \
	(0xff << 16) //pwm rising/falling point bit [7:0] as WDT
#define PWM_RISING_FALLING_MASK (0xffff)
#define PWM_FALLING_POINT_BIT (8) //pwm falling point bit [7:0]
#define PWM_RISING_POINT_BIT (0) //pwm rising point bit [7:0]
/* [31:24] */
#define DEFAULT_PWM_PERIOD 0xff
/**********************************************************
 * Software setting
 *********************************************************/
#define DEFAULT_TARGET_PWM_FREQ 25000
#define DEFAULT_DUTY_PT 10
#define DEFAULT_WDT_RELOAD_DUTY_PT 16

struct aspeed_pwm_data {
	struct regmap *regmap;
	unsigned long clk_freq;
	struct reset_control *reset;
	/* for pwm */
	struct pwm_chip chip;
};

static int regmap_aspeed_pwm_reg_write(void *context, unsigned int reg,
				       unsigned int val)
{
	void __iomem *regs = (void __iomem *)context;

	writel(val, regs + reg);
	return 0;
}

static int regmap_aspeed_pwm_reg_read(void *context, unsigned int reg,
				      unsigned int *val)
{
	void __iomem *regs = (void __iomem *)context;

	*val = readl(regs + reg);
	return 0;
}

static const struct regmap_config aspeed_pwm_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x100,
	.reg_write = regmap_aspeed_pwm_reg_write,
	.reg_read = regmap_aspeed_pwm_reg_read,
	.fast_io = true,
};

static void aspeed_set_pwm_channel_enable(struct regmap *regmap, u8 pwm_channel,
					  bool enable)
{
	regmap_update_bits(regmap, ASPEED_PWM_CTRL_CH(pwm_channel),
			   (PWM_CLK_ENABLE | PWM_PIN_EN),
			   enable ? (PWM_CLK_ENABLE | PWM_PIN_EN) : 0);
}

/**
 * struct aspeed_pwm - per-PWM driver data
 * @freq: cached pwm freq
 */
struct aspeed_pwm {
	u32 freq;
};
/*
 * The PWM frequency = HCLK(200Mhz) / (clock division L bit *
 * clock division H bit * (period bit + 1))
 */
static void aspeed_set_pwm_freq(struct aspeed_pwm_data *priv,
				struct pwm_device *pwm, u32 freq)
{
	u32 ctrl_value;
	u32 target_div, cal_freq;
	u32 tmp_div_h, tmp_div_l, diff, min_diff = INT_MAX;
	u32 div_h = BIT(5) - 1, div_l = BIT(8) - 1;
	u8 div_found;
	u32 index = pwm->hwpwm;
	struct aspeed_pwm *channel = pwm_get_chip_data(pwm);

	regmap_read(priv->regmap, ASPEED_PWM_CTRL_CH(index), &ctrl_value);

	cal_freq = priv->clk_freq / (DEFAULT_PWM_PERIOD + 1);
	target_div = DIV_ROUND_UP(cal_freq, freq);
	div_found = 0;
	/* calculate for target frequence */
	for (tmp_div_h = 0; tmp_div_h < 0x10; tmp_div_h++) {
		tmp_div_l = target_div / BIT(tmp_div_h) - 1;

		if (tmp_div_l < 0 || tmp_div_l > 255)
			continue;

		diff = freq - cal_freq / (BIT(tmp_div_h) * (tmp_div_l + 1));
		if (abs(diff) < abs(min_diff)) {
			min_diff = diff;
			div_l = tmp_div_l;
			div_h = tmp_div_h;
			div_found = 1;
			if (diff == 0)
				break;
		}
	}
	if (div_found == 0) {
		pr_debug("target freq: %d too slow set minimal frequency\n",
			 freq);
	}
	channel->freq = cal_freq / (BIT(div_h) * (div_l + 1));
	pr_debug("div h %x, l : %x pwm out clk %d\n", div_h, div_l,
		 channel->freq);
	pr_debug("hclk %ld, target pwm freq %d, real pwm freq %d\n",
		 priv->clk_freq, freq, channel->freq);
	ctrl_value &= ~GENMASK(11, 0);
	ctrl_value |= (div_h << 8) | div_l;
	regmap_write(priv->regmap, ASPEED_PWM_CTRL_CH(index), ctrl_value);
}

static void aspeed_set_pwm_duty(struct aspeed_pwm_data *priv,
				struct pwm_device *pwm, u32 duty_pt)
{
	u32 duty_value;
	u32 index = pwm->hwpwm;
	if (duty_pt == 0) {
		aspeed_set_pwm_channel_enable(priv->regmap, index, false);
	} else {
		regmap_read(priv->regmap, ASPEED_PWM_DUTY_CYCLE_CH(index),
			    &duty_value);
		duty_value &= ~GENMASK(15, 8);
		duty_value |= (duty_pt << PWM_FALLING_POINT_BIT);
		regmap_write(priv->regmap, ASPEED_PWM_DUTY_CYCLE_CH(index),
			     duty_value);
		aspeed_set_pwm_channel_enable(priv->regmap, index, true);
	}
}

static void aspeed_set_pwm_polarity(struct aspeed_pwm_data *priv,
				    struct pwm_device *pwm, u8 polarity)
{
	u32 ctrl_value;
	u32 index = pwm->hwpwm;
	regmap_read(priv->regmap, ASPEED_PWM_CTRL_CH(index), &ctrl_value);
	ctrl_value &= ~PWM_INVERSE;
	ctrl_value |= (polarity) ? PWM_INVERSE : 0;
	regmap_write(priv->regmap, ASPEED_PWM_CTRL_CH(index), ctrl_value);
}

static int aspeed_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct device *dev = chip->dev;
	struct aspeed_pwm_data *priv = dev_get_drvdata(dev);
	struct aspeed_pwm *channel;
	u32 duty_value;
	u32 index = pwm->hwpwm;

	regmap_read(priv->regmap, ASPEED_PWM_DUTY_CYCLE_CH(index), &duty_value);
	duty_value &= ~GENMASK(31, 24);
	duty_value &= ~GENMASK(7, 0);
	duty_value |= (DEFAULT_PWM_PERIOD << PWM_PERIOD_BIT);
	regmap_write(priv->regmap, ASPEED_PWM_DUTY_CYCLE_CH(index), duty_value);

	channel = devm_kzalloc(dev, sizeof(*channel), GFP_KERNEL);
	if (!channel)
		return -ENOMEM;

	pwm_set_chip_data(pwm, channel);

	return 0;
}

static void aspeed_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct device *dev = chip->dev;
	struct aspeed_pwm *channel = pwm_get_chip_data(pwm);

	devm_kfree(dev, channel);
}

static int aspeed_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	struct device *dev = chip->dev;
	struct aspeed_pwm_data *priv = dev_get_drvdata(dev);
	struct pwm_state *cur_state = &pwm->state;
	u32 freq = DIV_ROUND_UP(1000000000, state->period);
	u32 duty_pt = DIV_ROUND_UP(state->duty_cycle * 256, state->period);
	dev_dbg(dev, "freq: %d, duty_pt: %d", freq, duty_pt);
	if (state->enabled) {
		aspeed_set_pwm_freq(priv, pwm, freq);
		aspeed_set_pwm_duty(priv, pwm, duty_pt);
		aspeed_set_pwm_polarity(priv, pwm, state->polarity);
	} else {
		aspeed_set_pwm_duty(priv, pwm, 0);
	}
	cur_state->period = state->period;
	cur_state->duty_cycle = state->duty_cycle;
	cur_state->polarity = state->polarity;
	cur_state->enabled = state->enabled;

	return 0;
}
static const struct pwm_ops aspeed_pwm_ops = {
	.request = aspeed_pwm_request,
	.free = aspeed_pwm_free,
	.apply = aspeed_pwm_apply,
	.owner = THIS_MODULE,
};

static int aspeed_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk *clk;
	int ret;
	struct aspeed_pwm_data *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regmap = syscon_node_to_regmap(pdev->dev.parent->of_node);
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

	priv->chip.dev = dev;
	priv->chip.ops = &aspeed_pwm_ops;
	priv->chip.base = -1;
	priv->chip.npwm = 16;
	priv->chip.of_xlate = of_pwm_xlate_with_flags;
	priv->chip.of_pwm_n_cells = 3;

	ret = pwmchip_add(&priv->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add PWM chip: %d\n", ret);
		return ret;
	}
	dev_set_drvdata(dev, priv);
	return ret;
}

static const struct of_device_id of_pwm_match_table[] = {
	{
		.compatible = "aspeed,ast2600-pwm",
	},
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_match_table);

static struct platform_driver aspeed_pwm_driver = {
	.probe		= aspeed_pwm_probe,
	.driver		= {
		.name	= "aspeed_pwm",
		.of_match_table = of_pwm_match_table,
	},
};

module_platform_driver(aspeed_pwm_driver);

MODULE_AUTHOR("Billy Tsai <billy_tsai@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED PWM device driver");
MODULE_LICENSE("GPL");
