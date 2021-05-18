// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2021 ASPEED Technology Inc.
 *
 * PWM controller driver for Aspeed ast26xx SoCs.
 * This drivers doesn't support earlier version of the IP.
 *
 * The formula of pwm frequency:
 * PWM frequency = CLK Source / ((DIV_L + 1) * BIT(DIV_H) * (PERIOD + 1))
 *
 * The software driver fixes the period to 255, which causes the high-frequency
 * precision of the PWM to be coarse, in exchange for the fineness of the duty cycle.
 *
 * Register usage:
 * PIN_ENABLE: When it is unset the pwm controller will always output low to the extern.
 * Use to determine whether the PWM channel is enabled or disabled
 * CLK_ENABLE: When it is unset the pwm controller will reset the duty counter to 0 and
 * output low to the PIN_ENABLE mux after that the driver can still change the pwm period
 * and duty and the value will apply when CLK_ENABLE be set again.
 * Use to determin whether duty_cycle bigger than 0.
 * PWM_ASPEED_CTRL_INVERSE: When it is toggled the output value will inverse immediately.
 * PWM_ASPEED_DUTY_CYCLE_FALLING_POINT/PWM_ASPEED_DUTY_CYCLE_RISING_POINT: When these two
 * values are equal it means the duty cycle = 100%.
 *
 * Limitations:
 * - When changing both duty cycle and period, we cannot prevent in
 *   software that the output might produce a period with mixed
 *   settings.
 *
 * Improvements:
 * - When changing the duty cycle or period, our pwm controller will not
 *   generate the glitch, the configure will change at next cycle of pwm.
 *   This improvement can disable/enable through PWM_ASPEED_CTRL_DUTY_SYNC_DISABLE.
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
#include <linux/bitfield.h>
#include <linux/slab.h>
#include <linux/pwm.h>
#include <linux/math64.h>

/* The channel number of Aspeed pwm controller */
#define PWM_ASPEED_NR_PWMS 16

/* PWM Control Register */
#define PWM_ASPEED_CTRL_CH(ch) (((ch)*0x10) + 0x00)
#define PWM_ASPEED_CTRL_LOAD_SEL_RISING_AS_WDT BIT(19)
#define PWM_ASPEED_CTRL_DUTY_LOAD_AS_WDT_ENABLE BIT(18)
#define PWM_ASPEED_CTRL_DUTY_SYNC_DISABLE BIT(17)
#define PWM_ASPEED_CTRL_CLK_ENABLE BIT(16)
#define PWM_ASPEED_CTRL_LEVEL_OUTPUT BIT(15)
#define PWM_ASPEED_CTRL_INVERSE BIT(14)
#define PWM_ASPEED_CTRL_OPEN_DRAIN_ENABLE BIT(13)
#define PWM_ASPEED_CTRL_PIN_ENABLE BIT(12)
#define PWM_ASPEED_CTRL_CLK_DIV_H GENMASK(11, 8)
#define PWM_ASPEED_CTRL_CLK_DIV_L GENMASK(7, 0)

/* PWM Duty Cycle Register */
#define PWM_ASPEED_DUTY_CYCLE_CH(ch) (((ch)*0x10) + 0x04)
#define PWM_ASPEED_DUTY_CYCLE_PERIOD GENMASK(31, 24)
#define PWM_ASPEED_DUTY_CYCLE_POINT_AS_WDT GENMASK(23, 16)
#define PWM_ASPEED_DUTY_CYCLE_FALLING_POINT GENMASK(15, 8)
#define PWM_ASPEED_DUTY_CYCLE_RISING_POINT GENMASK(7, 0)

/* PWM fixed value */
#define PWM_ASPEED_FIXED_PERIOD 0xff

struct aspeed_pwm_data {
	struct pwm_chip chip;
	struct clk *clk;
	struct regmap *regmap;
	struct reset_control *reset;
};

static inline struct aspeed_pwm_data *
aspeed_pwm_chip_to_data(struct pwm_chip *c)
{
	return container_of(c, struct aspeed_pwm_data, chip);
}

static u32 aspeed_pwm_get_period(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct aspeed_pwm_data *priv = aspeed_pwm_chip_to_data(chip);
	unsigned long rate;
	u32 index = pwm->hwpwm;
	u32 val;
	u64 period, div_h, div_l, clk_period;

	rate = clk_get_rate(priv->clk);
	regmap_read(priv->regmap, PWM_ASPEED_CTRL_CH(index), &val);
	div_h = FIELD_GET(PWM_ASPEED_CTRL_CLK_DIV_H, val);
	div_l = FIELD_GET(PWM_ASPEED_CTRL_CLK_DIV_L, val);
	regmap_read(priv->regmap, PWM_ASPEED_DUTY_CYCLE_CH(index), &val);
	clk_period = FIELD_GET(PWM_ASPEED_DUTY_CYCLE_PERIOD, val);
	period = (NSEC_PER_SEC * BIT(div_h) * (div_l + 1) * (clk_period + 1));
	period = DIV_ROUND_UP_ULL(period, rate);

	return (u32)period;
}

static int aspeed_pwm_set_period(struct pwm_chip *chip, struct pwm_device *pwm,
				 const struct pwm_state *state)
{
	struct device *dev = chip->dev;
	struct aspeed_pwm_data *priv = aspeed_pwm_chip_to_data(chip);
	unsigned long rate;
	u64 div_h, div_l, divisor;
	u32 index = pwm->hwpwm;

	rate = clk_get_rate(priv->clk);
	/*
	 * Pick the smallest value for div_h so that div_l can be the biggest
	 * which results in a finer resolution near the target period value.
	 */
	divisor = (u64)NSEC_PER_SEC * (PWM_ASPEED_FIXED_PERIOD + 1) *
		  (PWM_ASPEED_CTRL_CLK_DIV_L + 1);
	div_h = order_base_2(div64_u64((u64)rate * state->period, divisor));
	if (div_h > 0xf)
		div_h = 0xf;

	divisor = ((u64)NSEC_PER_SEC * (PWM_ASPEED_FIXED_PERIOD + 1)) << div_h;
	div_l = div64_u64((u64)rate * state->period, divisor);

	if (div_l == 0)
		return -ERANGE;

	div_l -= 1;

	if (div_l > 255)
		div_l = 255;

	dev_dbg(dev, "clk source: %ld div_h %lld, div_l : %lld\n", rate, div_h,
		div_l);

	regmap_update_bits(
		priv->regmap, PWM_ASPEED_CTRL_CH(index),
		(PWM_ASPEED_CTRL_CLK_DIV_H | PWM_ASPEED_CTRL_CLK_DIV_L),
		FIELD_PREP(PWM_ASPEED_CTRL_CLK_DIV_H, div_h) |
			FIELD_PREP(PWM_ASPEED_CTRL_CLK_DIV_L, div_l));
	return 0;
}

static void aspeed_pwm_set_duty(struct pwm_chip *chip, struct pwm_device *pwm,
				const struct pwm_state *state)
{
	struct device *dev = chip->dev;
	struct aspeed_pwm_data *priv = aspeed_pwm_chip_to_data(chip);
	u32 duty_pt;
	u32 index = pwm->hwpwm;
	u32 cur_period;

	cur_period = aspeed_pwm_get_period(chip, pwm);
	duty_pt = DIV_ROUND_DOWN_ULL(
		(u64)state->duty_cycle * (PWM_ASPEED_FIXED_PERIOD + 1), cur_period);
	dev_dbg(dev, "cur_period = %d, duty_cycle = %d, duty_pt = %d\n",
		cur_period, state->duty_cycle, duty_pt);
	if (duty_pt == 0) {
		regmap_update_bits(priv->regmap, PWM_ASPEED_CTRL_CH(index),
				   PWM_ASPEED_CTRL_CLK_ENABLE, 0);
	} else {
		if (duty_pt >= (PWM_ASPEED_FIXED_PERIOD + 1))
			duty_pt = 0;
		regmap_update_bits(
			priv->regmap, PWM_ASPEED_DUTY_CYCLE_CH(index),
			PWM_ASPEED_DUTY_CYCLE_FALLING_POINT,
			FIELD_PREP(PWM_ASPEED_DUTY_CYCLE_FALLING_POINT,
				   duty_pt));
		regmap_update_bits(priv->regmap, PWM_ASPEED_CTRL_CH(index),
				   PWM_ASPEED_CTRL_CLK_ENABLE,
				   PWM_ASPEED_CTRL_CLK_ENABLE);
	}
}

static void aspeed_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				 struct pwm_state *state)
{
	struct device *dev = chip->dev;
	struct aspeed_pwm_data *priv = aspeed_pwm_chip_to_data(chip);
	u32 index = pwm->hwpwm;
	bool polarity, ch_en, clk_en;
	u32 duty_pt, val;

	regmap_read(priv->regmap, PWM_ASPEED_CTRL_CH(index), &val);
	polarity = FIELD_GET(PWM_ASPEED_CTRL_INVERSE, val);
	ch_en = FIELD_GET(PWM_ASPEED_CTRL_PIN_ENABLE, val);
	clk_en = FIELD_GET(PWM_ASPEED_CTRL_CLK_ENABLE, val);
	regmap_read(priv->regmap, PWM_ASPEED_DUTY_CYCLE_CH(index), &val);
	duty_pt = FIELD_GET(PWM_ASPEED_DUTY_CYCLE_FALLING_POINT, val);

	state->period = aspeed_pwm_get_period(chip, pwm);
	if (clk_en && duty_pt)
		state->duty_cycle = DIV_ROUND_UP_ULL(
			state->period * duty_pt, PWM_ASPEED_FIXED_PERIOD + 1);
	else
		state->duty_cycle = clk_en ? state->period : 0;
	state->polarity = polarity;
	state->enabled = ch_en;
	dev_dbg(dev, "get period: %dns, duty_cycle: %dns", state->period,
		state->duty_cycle);
}

static int aspeed_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	struct device *dev = chip->dev;
	struct aspeed_pwm_data *priv = aspeed_pwm_chip_to_data(chip);
	u32 index = pwm->hwpwm;
	int ret;

	dev_dbg(dev, "apply period: %dns, duty_cycle: %dns", state->period,
		state->duty_cycle);

	regmap_update_bits(priv->regmap, PWM_ASPEED_CTRL_CH(index),
			   PWM_ASPEED_CTRL_PIN_ENABLE,
			   state->enabled ? PWM_ASPEED_CTRL_PIN_ENABLE : 0);
	/*
	 * Fixed the period to the max value and rising point to 0
	 * for high resolution and simplify frequency calculation.
	 */
	regmap_update_bits(priv->regmap, PWM_ASPEED_DUTY_CYCLE_CH(index),
			   (PWM_ASPEED_DUTY_CYCLE_PERIOD |
			    PWM_ASPEED_DUTY_CYCLE_RISING_POINT),
			   FIELD_PREP(PWM_ASPEED_DUTY_CYCLE_PERIOD,
				      PWM_ASPEED_FIXED_PERIOD));

	ret = aspeed_pwm_set_period(chip, pwm, state);
	if (ret)
		return ret;
	aspeed_pwm_set_duty(chip, pwm, state);
	regmap_update_bits(priv->regmap, PWM_ASPEED_CTRL_CH(index),
			   PWM_ASPEED_CTRL_INVERSE,
			   FIELD_PREP(PWM_ASPEED_CTRL_INVERSE,
				      state->polarity));
	return 0;
}

static const struct pwm_ops aspeed_pwm_ops = {
	.apply = aspeed_pwm_apply,
	.get_state = aspeed_pwm_get_state,
	.owner = THIS_MODULE,
};

static int aspeed_pwm_channel_config(struct aspeed_pwm_data *priv,
				     struct device_node *child)
{
	u32 index, wdt_reload_duty;
	bool wdt_reload_en;

	int ret;

	ret = of_property_read_u32(child, "reg", &index);
	if (ret)
		return ret;
	ret = of_property_read_u32(child, "aspeed,wdt-reload-duty-point",
				   &wdt_reload_duty);
	if (ret)
		return ret;
	wdt_reload_en =
		of_property_read_bool(child, "aspeed,wdt-reload-enable");

	regmap_update_bits(
		priv->regmap, PWM_ASPEED_CTRL_CH(index),
		PWM_ASPEED_CTRL_DUTY_LOAD_AS_WDT_ENABLE,
		wdt_reload_en ? PWM_ASPEED_CTRL_DUTY_LOAD_AS_WDT_ENABLE : 0);

	regmap_update_bits(priv->regmap, PWM_ASPEED_DUTY_CYCLE_CH(index),
			   PWM_ASPEED_DUTY_CYCLE_POINT_AS_WDT,
			   FIELD_PREP(PWM_ASPEED_DUTY_CYCLE_POINT_AS_WDT,
				      wdt_reload_duty));

	return 0;
}

static int aspeed_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;
	struct aspeed_pwm_data *priv;
	struct device_node *np, *child;
	struct platform_device *parent_dev;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	np = pdev->dev.parent->of_node;
	if (!of_device_is_compatible(np, "aspeed,ast2600-pwm-tach")) {
		dev_err(dev, "unsupported pwm device binding\n");
		return -ENODEV;
	}

	priv->regmap = syscon_node_to_regmap(np);
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "Couldn't get regmap\n");
		return PTR_ERR(priv->regmap);
	}

	parent_dev = of_find_device_by_node(np);
	priv->clk = devm_clk_get(&parent_dev->dev, 0);
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "get clock failed\n");
		return PTR_ERR(priv->clk);
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(dev, "couldn't enable clock\n");
		return ret;
	}

	priv->reset = of_reset_control_get_shared(np, NULL);
	if (IS_ERR(priv->reset)) {
		dev_err(dev, "get reset failed\n");
		return PTR_ERR(priv->reset);
	}

	ret = reset_control_deassert(priv->reset);
	if (ret) {
		dev_err(dev, "cannot deassert reset control: %pe\n",
			ERR_PTR(ret));
		goto err_disable_clk;
	}

	priv->chip.dev = dev;
	priv->chip.ops = &aspeed_pwm_ops;
	priv->chip.npwm = PWM_ASPEED_NR_PWMS;
	priv->chip.of_xlate = of_pwm_xlate_with_flags;
	priv->chip.of_pwm_n_cells = 3;

	ret = pwmchip_add(&priv->chip);
	if (ret < 0) {
		dev_err(dev, "failed to add PWM chip: %pe\n", ERR_PTR(ret));
		goto err_assert_reset;
	}
	for_each_child_of_node(dev->of_node, child) {
		ret = aspeed_pwm_channel_config(priv, child);
		if (ret) {
			of_node_put(child);
			return ret;
		}
	}

	dev_set_drvdata(dev, priv);
	return 0;
err_assert_reset:
	reset_control_assert(priv->reset);
err_disable_clk:
	clk_disable_unprepare(priv->clk);
	return ret;
}

static int aspeed_pwm_remove(struct platform_device *dev)
{
	struct aspeed_pwm_data *priv = platform_get_drvdata(dev);

	pwmchip_remove(&priv->chip);
	reset_control_assert(priv->reset);
	clk_disable_unprepare(priv->clk);

	return 0;
}

static const struct of_device_id of_pwm_match_table[] = {
	{
		.compatible = "aspeed,ast2600-pwm",
	},
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_match_table);

static struct platform_driver aspeed_pwm_driver = {
	.probe = aspeed_pwm_probe,
	.remove	= aspeed_pwm_remove,
	.driver	= {
		.name = "aspeed_pwm",
		.of_match_table = of_pwm_match_table,
	},
};

module_platform_driver(aspeed_pwm_driver);

MODULE_AUTHOR("Billy Tsai <billy_tsai@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED PWM device driver");
MODULE_LICENSE("GPL v2");
