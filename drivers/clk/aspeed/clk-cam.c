/*
 * Copyright ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include "clk-aspeed.h"
//#define AST_CLK_DEBUG

#ifdef AST_CLK_DEBUG
//#define CLK_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#define CLK_DBUG(fmt, args...) printk( "%s() " fmt, __FUNCTION__, ## args)

#else
#define CLK_DBUG(fmt, args...)
#endif

/*	AST_SCU_PLL: 0x18/0x1C/0x20/0x24/0x28	E/A/M/H/D-PLL Parameter register	*/
#define SCU_PLL_ENSAT				(0x1 << 28)
#define SCU_PLL_FAST				(0x1 << 27)
#define SCU_PLL_TEST				(0x1 << 26)
#define SCU_PLL_RESET				(0x1 << 25)
#define SCU_PLL_BYPASS				(0x1 << 24)
#define SCU_PLL_OFF					(0x1 << 23)
#define SCU_PLL_PNUM(x)				(x << 19)
#define SCU_PLL_GET_PNUM(x)			((x >> 19) & 0xf)
#define SCU_PLL_PNUM_MASK			(0xf << 19)
#define SCU_PLL_NNUM(x)				(x << 13)
#define SCU_PLL_GET_NNUM(x)			((x >> 13) & 0x3f)
#define SCU_PLL_NNUM_MASK			(0x3f << 5)
#define SCU_PLL_MNUM(x)				(x)
#define SCU_PLL_GET_MNUM(x)			(x & 0x1fff)
#define SCU_PLL_MNUM_MASK			(0x1fff)

static unsigned long aspeed_clk_pll_recalc_rate(struct clk_hw *hw,
		unsigned long clkin_rate)
{
	struct aspeed_clk *pll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	int p, m, n;
	CLK_DBUG("aspeed_clk_pll_recalc_rate %s\n", clk_hw_get_name(hw));

	ret = regmap_read(pll->map, pll->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if (div & SCU_PLL_OFF)
		return 0;

	if (div & SCU_PLL_BYPASS) {
		return clkin_rate;
	} else {
		//P = SCU24[22:19]
		//N = SCU24[18:13]
		//M = SCU24[12:0]
		//hpll = clkin * [(M+1) /(N+1)] / (P+1)
		m = SCU_PLL_GET_MNUM(div);
		n = SCU_PLL_GET_NNUM(div);
		p = SCU_PLL_GET_PNUM(div);
		rate = ((clkin_rate * (m + 1)) / (n + 1)) / (p + 1);
	}
	CLK_DBUG(" === %ld === \n", rate);
	return rate;
}

#define SCU_HW_STRAP_GET_AHB_RATIO(x)	((x >> 8) & 0x7)

static unsigned long aspeed_clk_ahb_recalc_rate(struct clk_hw *hw,
		unsigned long hpll_rate)
{
	struct aspeed_clk *ahb = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	u32 ahb_div;
	CLK_DBUG("aspeed_clk_ahb_recalc_rate\n");

	/* Strap register SCU70 */
	ret = regmap_read(ahb->map, ahb->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	ahb_div = (SCU_HW_STRAP_GET_AHB_RATIO(div) * 2);
	if (!ahb_div) ahb_div = 4;

	rate = (hpll_rate / ahb_div);

	return rate;
}

#define SCU_GET_PCLK_DIV(x)				(((x) >> 6) & 0x7)

static unsigned long aspeed_clk_apb_recalc_rate(struct clk_hw *hw,
		unsigned long hpll_rate)
{
	struct aspeed_clk *apb = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	u32 apb_div;
	CLK_DBUG("aspeed_clk_apb_recalc_rate\n");

	/* Clock selection register SCU08 */
	ret = regmap_read(apb->map, apb->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	apb_div = ((SCU_GET_PCLK_DIV(div) + 1) * 4);

	rate = (hpll_rate / apb_div);

	return rate;
}

#define SCU_CLK_SD_DIV(x)			(x << 20)
#define SCU_CLK_SD_GET_DIV(x)		((x >> 20) & 0xf)
#define SCU_CLK_SD_MASK				(0xf << 20)
//0x08
#define SCU_SDCLK_STOP_EN			(0x1 << 21)

static unsigned long aspeed_sdclk_recalc_rate(struct clk_hw *hw,
		unsigned long clkin_rate)
{
	struct aspeed_clk *sdclk = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	u32 enable;
	u32 sd_div;
	CLK_DBUG("aspeed_sdclk_recalc_rate %s =======\n", clk_hw_get_name(hw));

	ret = regmap_read(sdclk->map, sdclk->div, &div);

	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	/* SCU0C: SD EN Register */
	ret = regmap_read(sdclk->map, sdclk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if (enable & SCU_SDCLK_STOP_EN) {
		printk("error ---------------------------- 0 \n");
		return 0;

	}
	sd_div = SCU_CLK_SD_GET_DIV(div);
	sd_div = (sd_div + 1) << 2;
	rate = clkin_rate /= sd_div;
	printk("== rate %ld ==\n", rate);
	return rate;
}

static int aspeed_sdclk_prepare(struct clk_hw *hw)
{
	struct aspeed_clk *sdclk = to_aspeed_clk(hw);
	int ret;
	u32 div;
	CLK_DBUG("aspeed_sdclk_prepare \n");

	ret = regmap_read(sdclk->map, sdclk->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	// SDCLK = G4  H-PLL / 4, G5 = H-PLL /8
//	ret = regmap_write(sdclk->map, sdclk->div, (div & ~SCU_CLK_SD_MASK) | SCU_CLK_SD_DIV(1));

	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;
}

static int aspeed_sdclk_enable(struct clk_hw *hw)
{
	struct aspeed_clk *sdclk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	CLK_DBUG("aspeed_sdclk_enable \n");
	/* SCU0C: SD EN Register */
	ret = regmap_read(sdclk->map, sdclk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	ret = regmap_write(sdclk->map, sdclk->enable, enable & ~SCU_SDCLK_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;
}

static void aspeed_sdclk_disable(struct clk_hw *hw)
{
	struct aspeed_clk *sdclk = to_aspeed_clk(hw);
	int ret;
	u32 enable;
	CLK_DBUG("aspeed_sdclk_disable \n");

	/* SCU0C: SD EN Register */
	ret = regmap_read(sdclk->map, sdclk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}

	ret = regmap_write(sdclk->map, sdclk->enable, enable | SCU_SDCLK_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}

}

#define SCU_CLK_SDIO_DIV(x)			(x << 16)
#define SCU_CLK_SDIO_GET_DIV(x)		((x >> 16) & 0xf)
#define SCU_CLK_SDIO_MASK				(0xf << 16)
//0x08
#define SCU_SDIOCLK_STOP_EN			(0x1 << 20)

static unsigned long aspeed_sdioclk_recalc_rate(struct clk_hw *hw,
		unsigned long clkin_rate)
{
	struct aspeed_clk *sdioclk = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	u32 enable;
	u32 sdio_div;
	CLK_DBUG("aspeed_sdioclk_recalc_rate %s =======\n", clk_hw_get_name(hw));

	ret = regmap_read(sdioclk->map, sdioclk->div, &div);

	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	/* SCU0C: SDIO EN Register */
	ret = regmap_read(sdioclk->map, sdioclk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if (enable & SCU_SDIOCLK_STOP_EN) {
		printk("error ---------------------------- 0 \n");
		return 0;

	}
	sdio_div = SCU_CLK_SDIO_GET_DIV(div);
	sdio_div = (sdio_div + 1) << 2;
	rate = clkin_rate /= sdio_div;
	printk("== rate %ld ==\n", rate);
	return rate;
}

static int aspeed_sdioclk_prepare(struct clk_hw *hw)
{
	struct aspeed_clk *sdioclk = to_aspeed_clk(hw);
	int ret;
	u32 div;
	CLK_DBUG("aspeed_sdioclk_prepare \n");

	ret = regmap_read(sdioclk->map, sdioclk->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	// SDIOCLK = G4  H-PLL / 4, G5 = H-PLL /8
//	ret = regmap_write(sdioclk->map, sdioclk->div, (div & ~SCU_CLK_SDIO_MASK) | SCU_CLK_SDIO_DIV(1));

	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;
}

static int aspeed_sdioclk_enable(struct clk_hw *hw)
{
	struct aspeed_clk *sdioclk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	CLK_DBUG("aspeed_sdioclk_enable \n");
	/* SCU0C: SDIO EN Register */
	ret = regmap_read(sdioclk->map, sdioclk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	ret = regmap_write(sdioclk->map, sdioclk->enable, enable & ~SCU_SDIOCLK_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;
}

static void aspeed_sdioclk_disable(struct clk_hw *hw)
{
	struct aspeed_clk *sdioclk = to_aspeed_clk(hw);
	int ret;
	u32 enable;
	CLK_DBUG("aspeed_sdioclk_disable \n");

	/* SCU0C: SDIO EN Register */
	ret = regmap_read(sdioclk->map, sdioclk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}

	ret = regmap_write(sdioclk->map, sdioclk->enable, enable | SCU_SDIOCLK_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}

}
#define SCU_RSACLK_STOP_EN			(0x1 << 24)
#define SCU_YCLK_STOP_EN			(0x1 << 16)

static unsigned long aspeed_yclk_recalc_rate(struct clk_hw *hw,
		unsigned long clkin_rate)
{
	struct aspeed_clk *yclk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	ret = regmap_read(yclk->map, yclk->enable, &enable);

	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if (enable & SCU_YCLK_STOP_EN)
		return 0;
	else
		return clkin_rate;
}

static int aspeed_yclk_enable(struct clk_hw *hw)
{
	struct aspeed_clk *yclk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	ret = regmap_read(yclk->map, yclk->enable, &enable);

	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	ret = regmap_write(yclk->map, yclk->enable, enable & ~(SCU_YCLK_STOP_EN | SCU_RSACLK_STOP_EN));
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;
}

static void aspeed_yclk_disable(struct clk_hw *hw)
{
	struct aspeed_clk *yclk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	ret = regmap_read(yclk->map, yclk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}

	ret = regmap_write(yclk->map, yclk->enable, enable | SCU_YCLK_STOP_EN | SCU_RSACLK_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}
}

#define SCU_MAC0CLK_STOP_EN		(0x1 << 20)

static int aspeed_mac_clk_enable(struct clk_hw *hw)
{
	struct aspeed_clk *mac_clk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	ret = regmap_read(mac_clk->map, mac_clk->enable, &enable);

	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	ret = regmap_write(mac_clk->map, mac_clk->enable, enable & ~SCU_MAC0CLK_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;
}

static void aspeed_mac_clk_disable(struct clk_hw *hw)
{
	struct aspeed_clk *mac_clk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	ret = regmap_read(mac_clk->map, mac_clk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}

	ret = regmap_write(mac_clk->map, mac_clk->enable, enable | SCU_MAC0CLK_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}
}

#define SCU_USB20_PHY_CLK_EN		(0x1 << 12)

static int aspeed_usb20p1_clk_enable(struct clk_hw *hw)
{
	struct aspeed_clk *usb20p1_clk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	CLK_DBUG("aspeed_usb20p1_clk_enable \n");

	ret = regmap_read(usb20p1_clk->map, usb20p1_clk->enable, &enable);

	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	ret = regmap_write(usb20p1_clk->map, usb20p1_clk->enable, enable & ~SCU_USB20_PHY_CLK_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;
}

static void aspeed_usb20p1_clk_disable(struct clk_hw *hw)
{
	struct aspeed_clk *usb20p1_clk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	ret = regmap_read(usb20p1_clk->map, usb20p1_clk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}

	ret = regmap_write(usb20p1_clk->map, usb20p1_clk->enable, enable | SCU_USB20_PHY_CLK_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}
}

static const struct clk_ops aspeed_clk_epll_ops = {
	.recalc_rate = aspeed_clk_pll_recalc_rate,
};
static const struct clk_ops aspeed_clk_apll_ops = {
	.recalc_rate = aspeed_clk_pll_recalc_rate,
};
static const struct clk_ops aspeed_clk_mpll_ops = {
	.recalc_rate = aspeed_clk_pll_recalc_rate,
};
static const struct clk_ops aspeed_clk_hpll_ops = {
	.recalc_rate = aspeed_clk_pll_recalc_rate,
};
static const struct clk_ops aspeed_clk_dpll_ops = {
	.recalc_rate = aspeed_clk_pll_recalc_rate,
};
static const struct clk_ops aspeed_clk_ahb_ops = {
	.recalc_rate = aspeed_clk_ahb_recalc_rate,
};
static const struct clk_ops aspeed_clk_apb_ops = {
	.recalc_rate = aspeed_clk_apb_recalc_rate,
};
static const struct clk_ops aspeed_sdclk_ops = {
	.recalc_rate = aspeed_sdclk_recalc_rate,
	.prepare = aspeed_sdclk_prepare,
	.enable = aspeed_sdclk_enable,
	.disable = aspeed_sdclk_disable,
};
static const struct clk_ops aspeed_sdioclk_ops = {
	.recalc_rate = aspeed_sdioclk_recalc_rate,
	.prepare = aspeed_sdioclk_prepare,
	.enable = aspeed_sdioclk_enable,
	.disable = aspeed_sdioclk_disable,
};
static const struct clk_ops aspeed_yclk_ops = {
	.recalc_rate = aspeed_yclk_recalc_rate,
	.enable = aspeed_yclk_enable,
	.disable = aspeed_yclk_disable,
};
static const struct clk_ops aspeed_mac_clk_ops = {
	.enable = aspeed_mac_clk_enable,
	.disable = aspeed_mac_clk_disable,
};
static const struct clk_ops aspeed_usb20p1_clk_ops = {
	.enable = aspeed_usb20p1_clk_enable,
	.disable = aspeed_usb20p1_clk_disable,
};

static void __init aspeed_clk_epll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_epll_ops, 0);
}
CLK_OF_DECLARE(aspeed_epll_clk, "aspeed,cam-epll-clock", aspeed_clk_epll_init);
static void __init aspeed_clk_apll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_apll_ops, 0);
}
CLK_OF_DECLARE(aspeed_apll_clk, "aspeed,cam-apll-clock", aspeed_clk_apll_init);
static void __init aspeed_clk_mpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_mpll_ops, 0);
}
CLK_OF_DECLARE(aspeed_mpll_clk, "aspeed,cam-mpll-clock", aspeed_clk_mpll_init);
static void __init aspeed_clk_hpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_hpll_ops, 0);
}
CLK_OF_DECLARE(aspeed_hpll_clk, "aspeed,cam-hpll-clock", aspeed_clk_hpll_init);
static void __init aspeed_clk_dpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_dpll_ops, 0);
}
CLK_OF_DECLARE(aspeed_dpll_clk, "aspeed,cam-dpll-clock", aspeed_clk_dpll_init);
static void __init aspeed_clk_ahb_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_ahb_ops, 0);
}
CLK_OF_DECLARE(aspeed_ahb_clk, "aspeed,cam-ahb-clock", aspeed_clk_ahb_init);
static void __init aspeed_clk_apb_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_apb_ops, 0);
}
CLK_OF_DECLARE(aspeed_apb_clk, "aspeed,cam-apb-clock", aspeed_clk_apb_init);
static void __init aspeed_sdclk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_sdclk_ops, CLK_GET_RATE_NOCACHE);
}
CLK_OF_DECLARE(aspeed_sdclk, "aspeed,cam-sdclock", aspeed_sdclk_init);
static void __init aspeed_sdioclk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_sdioclk_ops, CLK_GET_RATE_NOCACHE);
}
CLK_OF_DECLARE(aspeed_sdioclk, "aspeed,cam-sdioclock", aspeed_sdioclk_init);
static void __init aspeed_yclk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_yclk_ops, 0);
}
CLK_OF_DECLARE(aspeed_yclk, "aspeed,cam-yclock", aspeed_yclk_init);
static void __init aspeed_mac_clk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_mac_clk_ops, 0);
}
CLK_OF_DECLARE(aspeed_mac_clk, "aspeed,cam-mac-clock", aspeed_mac_clk_init);
static void __init aspeed_usb20p1_clk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_usb20p1_clk_ops, 0);
}
CLK_OF_DECLARE(aspeed_usb20p1_clk, "aspeed,cam-usb20p1clock", aspeed_usb20p1_clk_init);