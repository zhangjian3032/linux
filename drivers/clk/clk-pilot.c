// SPDX-License-Identifier: GPL-2.0
/*
 * Clock definitions for pilot platform.
 *
 * Copyright (C) 2018 Aspeed Technology Inc
 * Shivah Shankar S <shivahshankar.shankarnaraynarao@aspeedtech.com>
 *
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/slab.h>
#include "clk.h"

static struct clk *clks[1];
static struct clk_hw_onecell_data *clk_data;
struct pilot_clk_priv{
	struct clk_hw	hw;
	unsigned char enabled;
	struct clk *gclk;
};
#define to_pilot_clk(_hw) container_of(_hw, struct pilot_clk_priv, hw)
int pilot_clk_is_enabled(struct clk_hw *hw)
{
	struct pilot_clk_priv	*clk;
	clk = to_pilot_clk(hw);
	clk->enabled = 1;
	//We dont have to do anything here
	return 0;
};
static int pilot_clk_enable(struct clk_hw *hw)
{
	struct pilot_clk_priv	*clk;

	printk(" %s\n", __func__);
	clk = to_pilot_clk(hw);
	return clk->enabled;
}
static void pilot_clk_disable(struct clk_hw *hw)
{
	struct pilot_clk_priv	*clk;
	printk(" %s\n", __func__);
	clk = to_pilot_clk(hw);
	clk->enabled = 0;
}

struct clk_ops clk_pilot_smp_twd_ops = 
{
	.enable		= &pilot_clk_enable,
	.disable	= &pilot_clk_disable,
	.is_enabled	= &pilot_clk_is_enabled,
};
static struct pilot_clk_priv * pilot_plk_register(struct device_node *np)
{
	struct pilot_clk_priv	*clk;
	struct clk *nclk;
	const char *parent_names[2];
	struct clk_init_data initd = {
		.name = "pclk",
		.parent_names = parent_names,
		.ops = &clk_pilot_smp_twd_ops,
		.num_parents = 1,
		.flags = 0
	};
	parent_names[0] = "internalclk";
	clk = kzalloc(sizeof(*clk), GFP_KERNEL);
	if (!clk)
		return ERR_PTR(-ENOMEM);

	clk->hw.init = &initd;
	clk_hw_register(NULL, &clk->hw);
	return clk;
}

static void pilot_clk_init(struct device_node *np)
{
	struct pilot_clk_priv	*clk, *gclk;
	struct clk *twd_clk, *glbl_clk;
	struct device_node *child = NULL;

	if (!of_node_cmp(np->name, "pclk")){
		clk = pilot_plk_register(np);
		clk_data = kzalloc(sizeof(struct clk_hw_onecell_data) + sizeof(struct clk_hw), GFP_KERNEL);
		clk_data->hws[0] = &clk->hw;	
		clk_data->num = 1; 
		of_clk_add_hw_provider(np, of_clk_hw_simple_get, &clk->hw);
	}
}
CLK_OF_DECLARE(pilot_clks, "aspeed,pilot-clks", pilot_clk_init);
