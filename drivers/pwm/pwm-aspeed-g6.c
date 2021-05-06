// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2021 ASPEED Technology Inc.
 *
 * PWM controller driver for Aspeed ast26xx SoCs.
 * This drivers doesn't rollback to previous version of aspeed SoCs.
 *
 * The formula of pwm frequency:
 * PWM frequency = CLK Source / ((DIV_L + 1) * BIT(DIV_H) * (PERIOD + 1))
 *
 * The software driver fixes the period to 256, which causes the high-frequency
 * precision of the PWM to be coarse, in exchange for the fineness of the duty cycle.
 *
 * Register usage:
 * PIN_ENABLE: When it is unset the pwm controller will always output low to the extern.
 * Use to determin PWM channel enable/disable.
 * CLK_ENABLE: When it is unset the pwm controller will reset the duty counter to 0 and
 * output low to the PIN_ENABLE mux after that the driver can still change the pwm period
 * and duty and the value will apply when CLK_ENABLE be set again.
 * Use to determin whether duty_cycle bigger than 0.
 * PWM_ASPEED_INVERSE: When it is toggled the output value will inverse immediately.
 *
 * Limitations:
 * - When changing both duty cycle and period, we cannot prevent in
 *   software that the output might produce a period with mixed
 *   settings.
 *
 * Improvements:
 * - When changing the duty cycle or period, our pwm controller will not
 *   generate the glitch, the configure will change at next cycle of pwm.
 *   This improvement can disable/enable through PWM_ASPEED_DUTY_SYNC_DISABLE.
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

/* The channel number of Aspeed pwm controller */
#define PWM_ASPEED_NR_PWMS 16

/* PWM Control Register */
#define PWM_ASPEED_CTRL_CH(ch) (((ch * 0x10) + 0x00))
#define PWM_ASPEED_LOAD_SEL_RISING_AS_WDT BIT(19)
#define PWM_ASPEED_DUTY_LOAD_AS_WDT_ENABLE BIT(18)
#define PWM_ASPEED_DUTY_SYNC_DISABLE BIT(17)
#define PWM_ASPEED_CLK_ENABLE BIT(16)
#define PWM_ASPEED_LEVEL_OUTPUT BIT(15)
#define PWM_ASPEED_INVERSE BIT(14)
#define PWM_ASPEED_OPEN_DRAIN_ENABLE BIT(13)
#define PWM_ASPEED_PIN_ENABLE BIT(12)
#define PWM_ASPEED_CLK_DIV_H GENMASK(11, 8)
#define PWM_ASPEED_CLK_DIV_L GENMASK(7, 0)

/* PWM Duty Cycle Register */
#define PWM_ASPEED_DUTY_CYCLE_CH(ch) (((ch * 0x10) + 0x04))
#define PWM_ASPEED_PERIOD GENMASK(31, 24)
#define PWM_ASPEED_POINT_AS_WDT GENMASK(23, 16)
#define PWM_ASPEED_FALLING_POINT GENMASK(15, 8)
#define PWM_ASPEED_RISING_POINT GENMASK(7, 0)

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

static void aspeed_set_pwm_clk_enable(struct regmap *regmap, u8 pwm_channel,
				      bool enable)
{
	regmap_update_bits(regmap, PWM_ASPEED_CTRL_CH(pwm_channel),
			   PWM_ASPEED_CLK_ENABLE,
			   enable ? PWM_ASPEED_CLK_ENABLE : 0);
}

static u32 apseed_get_pwm_freq(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct aspeed_pwm_data *priv = aspeed_pwm_chip_to_data(chip);
	unsigned long rate;
	u32 index = pwm->hwpwm;
	u32 div_h, div_l, cur_freq, val;

	rate = clk_get_rate(priv->clk);
	regmap_read(priv->regmap, PWM_ASPEED_CTRL_CH(index), &val);
	div_h = FIELD_GET(PWM_ASPEED_CLK_DIV_H, val);
	div_l = FIELD_GET(PWM_ASPEED_CLK_DIV_L, val);

	cur_freq = DIV_ROUND_DOWN_ULL(rate, (BIT(div_h) * (div_l + 1) *
					     (PWM_ASPEED_FIXED_PERIOD + 1)));
	return cur_freq;
}

static int aspeed_set_pwm_freq(struct pwm_chip *chip, struct pwm_device *pwm,
			       const struct pwm_state *state)
{
	struct device *dev = chip->dev;
	struct aspeed_pwm_data *priv = aspeed_pwm_chip_to_data(chip);
	unsigned long rate;
	u32 div_h, div_l, freq;
	u32 index = pwm->hwpwm;
	/* Get the smallest value for div_h  */
	freq = DIV_ROUND_UP_ULL(NSEC_PER_SEC, state->period);
	rate = clk_get_rate(priv->clk);
	div_h = DIV_ROUND_DOWN_ULL(rate, ((PWM_ASPEED_CLK_DIV_L + 1) * freq *
					  (PWM_ASPEED_FIXED_PERIOD + 1)));
	div_h = order_base_2(div_h);
	if (div_h > 0xf)
		div_h = 0xf;

	div_l = DIV_ROUND_DOWN_ULL(rate >> div_h,
				   (freq * (PWM_ASPEED_FIXED_PERIOD + 1)));
	if (div_l == 0) {
		dev_err(dev, "Period too small, cannot implement it");
		return -ERANGE;
	}

	div_l -= 1;

	if (div_l > 255)
		div_l = 255;

	dev_dbg(dev, "clk source: %ld div h %x, l : %x\n", rate, div_h, div_l);

	regmap_update_bits(priv->regmap, PWM_ASPEED_CTRL_CH(index),
			   (PWM_ASPEED_CLK_DIV_H | PWM_ASPEED_CLK_DIV_L),
			   FIELD_PREP(PWM_ASPEED_CLK_DIV_H, div_h) |
				   FIELD_PREP(PWM_ASPEED_CLK_DIV_L, div_l));
	return 0;
}

static void aspeed_set_pwm_duty(struct pwm_chip *chip, struct pwm_device *pwm,
				const struct pwm_state *state)
{
	struct aspeed_pwm_data *priv = aspeed_pwm_chip_to_data(chip);
	u32 duty_pt;
	u32 index = pwm->hwpwm;
	u32 cur_freq;
	u64 cur_period;

	cur_freq = apseed_get_pwm_freq(chip, pwm);
	cur_period = DIV_ROUND_DOWN_ULL(NSEC_PER_SEC, cur_freq);
	duty_pt = DIV_ROUND_DOWN_ULL(
		state->duty_cycle * (PWM_ASPEED_FIXED_PERIOD + 1), cur_period);
	if (duty_pt == 0) {
		aspeed_set_pwm_clk_enable(priv->regmap, index, false);
	} else {
		if (duty_pt >= (PWM_ASPEED_FIXED_PERIOD + 1))
			duty_pt = 0;
		/* When duty_pt = 0 it mean our duty cycle = 100% */
		regmap_update_bits(
			priv->regmap, PWM_ASPEED_DUTY_CYCLE_CH(index),
			PWM_ASPEED_FALLING_POINT,
			FIELD_PREP(PWM_ASPEED_FALLING_POINT, duty_pt));
		aspeed_set_pwm_clk_enable(priv->regmap, index, true);
	}
}

static void aspeed_set_pwm_polarity(struct pwm_chip *chip,
				    struct pwm_device *pwm,
				    const struct pwm_state *state)
{
	struct aspeed_pwm_data *priv = aspeed_pwm_chip_to_data(chip);
	u32 index = pwm->hwpwm;

	regmap_update_bits(priv->regmap, PWM_ASPEED_CTRL_CH(index),
			   PWM_ASPEED_INVERSE,
			   FIELD_PREP(PWM_ASPEED_INVERSE, state->polarity));
}

static void aspeed_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				 struct pwm_state *state)
{
	struct device *dev = chip->dev;
	struct aspeed_pwm_data *priv = aspeed_pwm_chip_to_data(chip);
	u32 index = pwm->hwpwm;
	bool polarity, ch_en, clk_en;
	u32 duty_pt, val;
	u32 cur_freq;

	regmap_read(priv->regmap, PWM_ASPEED_CTRL_CH(index), &val);
	polarity = FIELD_GET(PWM_ASPEED_INVERSE, val);
	ch_en = FIELD_GET(PWM_ASPEED_PIN_ENABLE, val);
	clk_en = FIELD_GET(PWM_ASPEED_CLK_ENABLE, val);
	regmap_read(priv->regmap, PWM_ASPEED_DUTY_CYCLE_CH(index), &val);
	duty_pt = FIELD_GET(PWM_ASPEED_FALLING_POINT, val);

	cur_freq = apseed_get_pwm_freq(chip, pwm);

	state->period = DIV_ROUND_DOWN_ULL(NSEC_PER_SEC, cur_freq);
	if (clk_en && duty_pt)
		state->duty_cycle = DIV_ROUND_DOWN_ULL(
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
	struct pwm_state cur_state;
	int ret;

	aspeed_pwm_get_state(chip, pwm, &cur_state);
	dev_dbg(dev, "cur period: %dns, cur duty_cycle: %dns",
		cur_state.period, cur_state.duty_cycle);
	dev_dbg(dev, "apply period: %dns, duty_cycle: %dns", state->period,
		state->duty_cycle);
	regmap_update_bits(priv->regmap, PWM_ASPEED_CTRL_CH(index),
			   PWM_ASPEED_PIN_ENABLE,
			   state->enabled ? PWM_ASPEED_PIN_ENABLE : 0);
	if (cur_state.period != state->period) {
		ret = aspeed_set_pwm_freq(chip, pwm, state);
		if (ret)
			return ret;
		aspeed_set_pwm_duty(chip, pwm, state);
	} else if (cur_state.duty_cycle != state->duty_cycle)
		aspeed_set_pwm_duty(chip, pwm, state);
	aspeed_set_pwm_polarity(chip, pwm, state);
	return 0;
}

static const struct pwm_ops aspeed_pwm_ops = {
	.apply = aspeed_pwm_apply,
	.get_state = aspeed_pwm_get_state,
	.owner = THIS_MODULE,
};

static int aspeed_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret, index;
	struct aspeed_pwm_data *priv;
	struct device_node *np;

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
		return -ENODEV;
	}

	priv->clk = of_clk_get(np, 0);
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
		clk_disable_unprepare(priv->clk);
		return ret;
	}

	priv->chip.dev = dev;
	priv->chip.ops = &aspeed_pwm_ops;
	priv->chip.npwm = PWM_ASPEED_NR_PWMS;
	priv->chip.of_xlate = of_pwm_xlate_with_flags;
	priv->chip.of_pwm_n_cells = 3;

	/*
	 * Fixed the period to the max value and rising point to 0
	 * for high resolution and simplified frequency calculation.
	 */
	for (index = 0; index < PWM_ASPEED_NR_PWMS; index++) {
		regmap_update_bits(
			priv->regmap, PWM_ASPEED_DUTY_CYCLE_CH(index),
			PWM_ASPEED_PERIOD,
			FIELD_PREP(PWM_ASPEED_PERIOD, PWM_ASPEED_FIXED_PERIOD));
		regmap_update_bits(priv->regmap,
				   PWM_ASPEED_DUTY_CYCLE_CH(index),
				   PWM_ASPEED_RISING_POINT, 0);
	}

	ret = pwmchip_add(&priv->chip);
	if (ret < 0) {
		dev_err(dev, "failed to add PWM chip: %pe\n", ERR_PTR(ret));
		reset_control_assert(priv->reset);
		clk_disable_unprepare(priv->clk);
		return ret;
	}
	dev_set_drvdata(dev, priv);
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
	.probe		= aspeed_pwm_probe,
	.remove		= aspeed_pwm_remove,
	.driver		= {
		.name	= "aspeed_pwm",
		.of_match_table = of_pwm_match_table,
	},
};

module_platform_driver(aspeed_pwm_driver);

MODULE_AUTHOR("Billy Tsai <billy_tsai@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED PWM device driver");
MODULE_LICENSE("GPL v2");
