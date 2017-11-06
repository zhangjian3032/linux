/*
 * Copyright 2017 IBM Corporation
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

#define CLK_25M_IN					(0x1 << 23)
#define CLK_25M_MASK(x)				((x >> 23) & 0x1)
#define USBCLK_48M_IN				(0x1 << 18)
#define USBCLK_48M_MASK(x)			((x >> 18) & 0x1)
#define CLK_24M_IN					0
#define CLK_48M_IN					1
#define CLK_25M_IN_24M_USB_CKI		2
#define CLK_25M_IN_48M_USB_CKI		3
static unsigned long aspeed_clk_clkin_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct aspeed_clk *clkin = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 reg;
	u8 clk;

	/* SCU70: Hardware Strapping Register  */
	ret = regmap_read(clkin->map, clkin->reg, &reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	
	clk = CLK_25M_MASK(reg) | USBCLK_48M_MASK(reg);
	
	switch(clk) {
		case 0:
			rate = 24 * 1000 * 1000;
			break;
		case 1:
			rate = 48 * 1000 * 1000;
			break;
		case 2:
		case 3:
			rate = 25 * 1000 * 1000;
			break;
	}

	return rate;
}

#define SCU_H_PLL_PARAMETER				(0x1 << 18)
#define SCU_H_PLL_BYPASS_EN				(0x1 << 17)
#define SCU_H_PLL_OFF					(0x1 << 16)
#define SCU_H_PLL_NUM(x)				(x << 5)
#define SCU_H_PLL_GET_NUM(x)			((x >> 5) & 0x3f)
#define SCU_H_PLL_NUM_MASK				(0x3f << 5)
#define SCU_H_PLL_OUT_DIV				(0x1 << 4)
#define SCU_H_PLL_GET_DIV(x)			((x >> 4) & 0x1)
#define SCU_H_PLL_DENUM(x)				(x)
#define SCU_H_PLL_GET_DENUM(x)			(x & 0xf)
#define SCU_H_PLL_DENUM_MASK			(0xf)

static unsigned long aspeed_clk_hpll_recalc_rate(struct clk_hw *hw,
						 unsigned long clkin_rate)
{
	struct aspeed_clk *hpll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 reg;
	int de, n, od;
	printk("TODO ~~~~~~~~~~~~~~~~~~~~~~~~~ aspeed_clk_hpll_recalc_rate \n");
#if 0	
	/* SCU24: HPLL */
	ret = regmap_read(hpll->map, hpll->reg, &reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if(reg & SCU_H_PLL_OFF)
		return 0;

	if(reg & SCU_H_PLL_PARAMETER) {
		if(reg & SCU_H_PLL_BYPASS_EN) {
			return clkin_rate;
		} else {
			od = SCU_H_PLL_GET_DIV(reg);	//OD == SCU24[4]
			n = SCU_H_PLL_GET_NUM(reg);		//Numerator == SCU24[10:5]
			de = SCU_H_PLL_GET_DENUM(reg);	//Denumerator == SCU24[3:0]
			//hpll = clkin * (2-OD) * ((Numerator+2)/(Denumerator+1))
			rate = ((clkin_rate * (2-od) * (n+2))/(de+1));
		}
	} else {	// HW Trap
		clk = SCU_HW_STRAP_GET_H_PLL_CLK(scu70);
		switch (clk) {
			case 0:
				rate = 384 * 1000 * 1000; 
				break;
			case 1:
				rate = 360 * 1000 * 1000; 
				break;
			case 2:
				rate = 336 * 1000 * 1000; 
				break;
			case 3:
				rate = 408 * 1000 * 1000; 
				break;
			default:
				BUG(); 
				break;
		}		
	}
#endif	
	return rate;
}

#define SCU_HW_STRAP_GET_CPU_AHB_RATIO(x)		((x >> 10) & 3)

static unsigned long aspeed_clk_ahb_recalc_rate(struct clk_hw *hw, 
						unsigned long hpll_rate)
{
	struct aspeed_clk *ahb = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	u32 reg;

	/* Strap register SCU70 */
	ret = regmap_read(ahb->map, ahb->reg, &reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	/* bits 11:10 define the CPU/AHB clock frequency ratio */
	div = SCU_HW_STRAP_GET_CPU_AHB_RATIO(reg);

	rate = hpll_rate / (div + 1);

	return rate;
}

#define SCU_PCLK_APB_DIV(x)			(x << 23)
#define SCU_GET_PCLK_DIV(x)			((x >> 23) & 0x7)
#define SCU_PCLK_APB_DIV_MASK		(0x7 << 23)		//limitation on PCLK .. PCLK > 0.5*LCLK (33Mhz)

static unsigned long aspeed_clk_apb_recalc_rate(struct clk_hw *hw,
						unsigned long hpll_rate)
{
	struct aspeed_clk *apb = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	u32 reg;

	/* SCU08: Clock Selection Register */
	ret = regmap_read(apb->map, apb->reg, &reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	div = SCU_GET_PCLK_DIV(reg);
	div = (div+1) << 1;

	rate = hpll_rate / div;

	return rate;
}

#define SCU_M_PLL_BYPASS				(0x1 << 17)
#define SCU_M_PLL_OFF					(0x1 << 16)
#define SCU_M_PLL_NUM(x)				(x << 5)
#define SCU_M_PLL_GET_NUM(x)			((x >> 5) & 0x3f)
#define SCU_M_PLL_NUM_MASK				(0x3f << 5)
#define SCU_M_PLL_OUT_DIV				(0x1 << 4)
#define SCU_M_PLL_GET_DIV(x)			((x >> 4) & 0x1)
#define SCU_M_PLL_DENUM(x)				(x)
#define SCU_M_PLL_GET_DENUM(x)			(x & 0xf)

static unsigned long aspeed_clk_mpll_recalc_rate(struct clk_hw *hw,
						unsigned long clkin_rate)
{
	struct aspeed_clk *mpll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 reg;
	int od, n, de;

	/* SCU20: M-PLL Parameter Register */
	ret = regmap_read(mpll->map, mpll->reg, &reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if(reg & SCU_M_PLL_OFF)
		return 0;

	if(reg & SCU_M_PLL_BYPASS)
		return clkin_rate;

	od = SCU_M_PLL_GET_DIV(reg);	//OD == SCU24[4]
	n = SCU_M_PLL_GET_NUM(reg);		//Numerator == SCU24[10:5]
	de = SCU_M_PLL_GET_DENUM(reg);	//Denumerator == SCU24[3:0]
	
	//mpll = clkin * (2-OD) * ((n+2)/(de+1))
	rate = (clkin_rate * (2-od) * ((n+2)/(de+1)));

	return rate;
}

static unsigned long aspeed_clk_dpll_recalc_rate(struct clk_hw *hw,
						unsigned long clkin_rate)
{
	struct aspeed_clk *dpll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 reg;
	u32 ext_reg;
	int p, m, n, od;
#if 0 	
	/* SCU28: D-PLL Parameter Register */
	ret = regmap_read(dpll->map, dpll->reg, &reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	/* SCU130: D-PLL Parameter Register */
	ret = regmap_read(dpll->map, dpll->ext_reg, &ext_reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if(ext_reg & SCU_D_PLL_OFF)
		return 0;

	if(ext_reg & SCU_D_PLL_BYPASS)
		return clkin_rate;

	m = SCU_D_PLL_GET_MNUM(reg);
	n = SCU_D_PLL_GET_NNUM(reg);
	p = SCU_D_PLL_GET_PNUM(reg);
	od = SCU_D_PLL_GET_ODNUM(reg);

	//dpll = clkin * [(M + 1) /(N + 1)] / (P + 1) / (OD + 1)
	rate = (clkin_rate * (m + 1)) / (n + 1) / (p + 1) / (od + 1);
#endif
	return rate;
}

#define SCU_D2_PLL_SET_PD2(x)			(x << 19)
#define SCU_D2_PLL_GET_PD2(x)			((x >> 19)&0x7)
#define SCU_D2_PLL_PD2_MASK				(0x7 << 19)
#define SCU_D2_PLL_BYPASS				(0x1 << 18)
#define SCU_D2_PLL_OFF					(0x1 << 17)
#define SCU_D2_PLL_SET_PD(x)			(x << 15)
#define SCU_D2_PLL_GET_PD(x)			((x >> 15) &0x3)
#define SCU_D2_PLL_PD_MASK				(0x3 << 15)
#define SCU_D2_PLL_SET_OD(x)			(x << 13)
#define SCU_D2_PLL_GET_OD(x)			((x >> 13) & 0x3)
#define SCU_D2_PLL_OD_MASK				(0x3 << 13)
#define SCU_D2_PLL_SET_DENUM(x)			(x << 8)
#define SCU_D2_PLL_GET_DENUM(x)			((x >>8)&0x1f)
#define SCU_D2_PLL_DENUM_MASK			(0x1f << 8)
#define SCU_D2_PLL_SET_NUM(x)			(x)
#define SCU_D2_PLL_GET_NUM(x)			(x & 0xff)
#define SCU_D2_PLL_NUM_MASK				(0xff)

static unsigned long aspeed_clk_d2pll_recalc_rate(struct clk_hw *hw,
						unsigned long clkin_rate)
{
	struct aspeed_clk *d2pll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 reg;
	u32 ext_reg;
	int n, de, od, pd, pd2;

	/* SCU1C: D2-PLL Parameter Register */
	ret = regmap_read(d2pll->map, d2pll->reg, &reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
#if 0
	/* SCU13C: D2-PLL Parameter Register */
	ret = regmap_read(d2pll->map, d2pll->ext_reg, &ext_reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if(ext_reg & SCU_D2_PLL_OFF)
		return 0;

	if(ext_reg & SCU_D2_PLL_BYPASS) 
		return clkin_rate;

	n = SCU_D2_PLL_GET_NNUM(reg);
	de = SCU_D2_PLL_GET_DENUM(reg);
	od = 1 << (SCU_D2_PLL_GET_OD(reg) - 1);
	pd= SCU_D2_PLL_GET_PD(reg) + 1;
	pd2= SCU_D2_PLL_GET_PD2(reg) + 1;

	//d2pll = clkin * (Numerator * 2) / (Denumerator * OD * PD * PD2)
	rate = (clkin_rate * NUM * 2) / (de * od * pd * pd2);
#endif
	return rate;
}

static long aspeed_clk_d2pll_round_rate(struct clk_hw *hw,
			unsigned long drate, unsigned long *prate)
{
	struct aspeed_clk *d2pll = to_aspeed_clk(hw);
	printk("TODO aspeed_clk_d2pll_round_rate \n");
}
			
static int aspeed_clk_d2pll_set_rate(struct clk_hw *hw, unsigned long rate,
							unsigned long parent_rate)
{
	struct aspeed_clk *d2pll = to_aspeed_clk(hw);
	int ret;
	u32 reg;
	u32 ext_reg;
	int p, m, n, od;
	printk("TODO aspeed_clk_d2pll_set_rate \n");
	/* SCU1C: D2-PLL Parameter Register */
	ret = regmap_read(d2pll->map, d2pll->reg, &reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	
	/* SCU13C: D2-PLL Parameter Register */
	ret = regmap_read(d2pll->map, d2pll->reg, &ext_reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	printk("TODO ~~ \n");
#if 0	
	//Off D2-PLL
//	ast_scu_write(ast_scu_read(AST_SCU_D2_PLL_EXTEND) |  SCU_D2_PLL_OFF | SCU_D2_PLL_RESET , AST_SCU_D2_PLL_EXTEND);
	ast_scu_write(0x585, AST_SCU_D2_PLL_EXTEND);

	//set D2-PLL parameter 
	ast_scu_write(pll_setting, AST_SCU_D2_PLL);

	//enable D2-PLL
//	ast_scu_write(ast_scu_read(AST_SCU_D2_PLL_EXTEND) &  ~(SCU_D2_PLL_OFF | SCU_D2_PLL_RESET) , AST_SCU_D2_PLL_EXTEND);
	ast_scu_write(0x580, AST_SCU_D2_PLL_EXTEND);
#endif	
}

#define SCU_GET_LHCLK_DIV(x)		((x >> 20) & 0x7)
#define SCU_SET_LHCLK_DIV(x)		(x << 20)
#define SCU_LHCLK_DIV_MASK			(0x7 << 20)
#define SCU_LHCLK_SOURCE_EN			(0x1 << 19)		//0: ext , 1:internel

static unsigned long aspeed_clk_lhpll_recalc_rate(struct clk_hw *hw,
						unsigned long clkin_rate)
{
	struct aspeed_clk *lhpll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 reg;
	u32 div;

	/* SCU1C: D2-PLL Parameter Register */
	ret = regmap_read(lhpll->map, lhpll->reg, &reg);
	
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if(SCU_LHCLK_SOURCE_EN & reg) {
		div = SCU_GET_LHCLK_DIV(reg);
		div = (div+1) << 1;
		rate = (clkin_rate/div);
	} else { 
		// come from external source
		rate = 0; 
	}
		
	return rate;
}

#define SCU_CLK_SD_DIV(x)			(x << 12)
#define SCU_CLK_SD_GET_DIV(x)		((x >> 12) & 0x7)
#define SCU_CLK_SD_MASK				(0x7 << 12)

static unsigned long aspeed_clk_sdpll_recalc_rate(struct clk_hw *hw,
						unsigned long clkin_rate)
{
	struct aspeed_clk *sdpll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 reg;
	u32 div;

	ret = regmap_read(sdpll->map, sdpll->reg, &reg);
	
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	div = SCU_CLK_SD_GET_DIV(reg);
	div = (div+1) << 1;
	clkin_rate /= div;
		
	return rate;
}						

static const struct clk_ops aspeed_clk_clkin_ops = {
	.recalc_rate = aspeed_clk_clkin_recalc_rate,
};
static const struct clk_ops aspeed_clk_hpll_ops = {
	.recalc_rate = aspeed_clk_hpll_recalc_rate,
};
static const struct clk_ops aspeed_clk_apb_ops = {
	.recalc_rate = aspeed_clk_apb_recalc_rate,
};
static const struct clk_ops aspeed_clk_ahb_ops = {
	.recalc_rate = aspeed_clk_ahb_recalc_rate,
};
static const struct clk_ops aspeed_clk_mpll_ops = {
	.recalc_rate = aspeed_clk_mpll_recalc_rate,
};	
static const struct clk_ops aspeed_clk_dpll_ops = {
	.recalc_rate = aspeed_clk_dpll_recalc_rate,
};
static const struct clk_ops aspeed_clk_d2pll_ops = {
	.recalc_rate = aspeed_clk_d2pll_recalc_rate,
	.round_rate = aspeed_clk_d2pll_round_rate,
	.set_rate = aspeed_clk_d2pll_set_rate,
};
static const struct clk_ops aspeed_clk_lhpll_ops = {
	.recalc_rate = aspeed_clk_lhpll_recalc_rate,
};
static const struct clk_ops aspeed_clk_sdpll_ops = {
	.recalc_rate = aspeed_clk_sdpll_recalc_rate,
};

static void __init aspeed_clk_clkin_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_clkin_ops);
}
CLK_OF_DECLARE(aspeed_clkin_clk, "aspeed,g4-clkin-clock", aspeed_clk_clkin_init);

static void __init aspeed_clk_hpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_hpll_ops);
}
CLK_OF_DECLARE(aspeed_hpll_clk, "aspeed,g4-hpll-clock", aspeed_clk_hpll_init);

static void __init aspeed_clk_apb_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_apb_ops);
}
CLK_OF_DECLARE(aspeed_apb_clk, "aspeed,g4-apb-clock", aspeed_clk_apb_init);

static void __init aspeed_clk_ahb_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_ahb_ops);
}
CLK_OF_DECLARE(aspeed_ahb_clk, "aspeed,g4-ahb-clock", aspeed_clk_ahb_init);
static void __init aspeed_clk_mpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_mpll_ops);
}
CLK_OF_DECLARE(aspeed_mpll_clk, "aspeed,g4-mpll-clock", aspeed_clk_mpll_init);
static void __init aspeed_clk_dpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_dpll_ops);
}
CLK_OF_DECLARE(aspeed_dpll_clk, "aspeed,g4-dpll-clock", aspeed_clk_dpll_init);
static void __init aspeed_clk_d2pll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_d2pll_ops);
}
CLK_OF_DECLARE(aspeed_d2pll_clk, "aspeed,g4-d2pll-clock", aspeed_clk_d2pll_init);
static void __init aspeed_clk_lhpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_lhpll_ops);
}
CLK_OF_DECLARE(aspeed_lhpll_clk, "aspeed,g4-lhpll-clock", aspeed_clk_lhpll_init);
static void __init aspeed_clk_sdpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_sdpll_ops);
}
CLK_OF_DECLARE(aspeed_sdpll_clk, "aspeed,g4-sdpll-clock", aspeed_clk_sdpll_init);
