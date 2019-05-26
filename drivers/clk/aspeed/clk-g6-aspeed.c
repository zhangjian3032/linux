// SPDX-License-Identifier: GPL-2.0+

#define pr_fmt(fmt) "clk-aspeed: " fmt

#include <linux/clk-provider.h>
#include <linux/mfd/syscon.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <dt-bindings/clock/aspeed-g6-clock.h>
#include "clk-aspeed.h"

#define ASPEED_G6_NUM_CLKS		49

#define ASPEED_RESET2_OFFSET	32
#define ASPEED_CLK2_OFFSET		32
#define ASPEED_G6_RESET_CTRL		0x40
#define ASPEED_G6_RESET_CTRL2		0x50


#define ASPEED_G6_CLK_SELECTION		0x300
#define ASPEED_G6_CLK_SELECTION2	0x304
#define ASPEED_G6_CLK_SELECTION3	0x310

#define ASPEED_G6_CLK_STOP_CTRL		0x80
#define ASPEED_G6_CLK_STOP_CTRL2	0x90

#define ASPEED_G6_MISC_CTRL			0xC0
#define  UART_DIV13_EN		BIT(12)

#define ASPEED_MPLL_PARAM	0x220

/* Globally visible clocks */
static DEFINE_SPINLOCK(aspeed_clk_lock);

/* Keeps track of all clocks */
static struct clk_hw_onecell_data *aspeed_g6_clk_data;

static void __iomem *scu_g6_base;


static const struct aspeed_gate_data aspeed_g6_gates[] = {
	/*				 			  clk rst   name			parent	flags */
	[ASPEED_CLK_GATE_ECLK] =	{  1, -1, "eclk-gate",		"eclk",	0 }, /* Video Engine */
	[ASPEED_CLK_GATE_GCLK] =	{  2,  7, "gclk-gate",		NULL,	0 }, /* 2D engine */
	[ASPEED_CLK_GATE_MCLK] =	{  0, -1, "mclk-gate",		"mpll",	CLK_IS_CRITICAL }, /* SDRAM */
	[ASPEED_CLK_GATE_VCLK] =	{  3,  6, "vclk-gate",		NULL,	0 }, /* Video Capture */
	[ASPEED_CLK_GATE_BCLK] =	{  4,  8, "bclk-gate",		"bclk",	CLK_IS_CRITICAL }, /* PCIe/PCI */
	[ASPEED_CLK_GATE_DCLK] =	{  5, -1, "dclk-gate",		NULL,	CLK_IS_CRITICAL }, /* DAC */
	[ASPEED_CLK_GATE_REFCLK] =	{  6, -1, "refclk-gate",	"clkin", CLK_IS_CRITICAL },
	[ASPEED_CLK_GATE_USBPORT2CLK] =	{  7,  3, "usb-port2-gate",	NULL,	0 }, /* USB2.0 Host port 2 */
	
	[ASPEED_CLK_GATE_LCLK] =	{  ASPEED_CLK2_OFFSET + 0,  32, "lclk-gate",		NULL,	CLK_IS_CRITICAL }, /* LPC */
	
	[ASPEED_CLK_GATE_USBUHCICLK] =	{  9, 15, "usb-uhci-gate",	NULL,	0 }, /* USB1.1 (requires port 2 enabled) */
	[ASPEED_CLK_GATE_D1CLK] =	{ 10, 13, "d1clk-gate",		NULL,	0 }, /* GFX CRT */
	[ASPEED_CLK_GATE_YCLK] =	{ 13,  4, "yclk-gate",		NULL,	0 }, /* HAC */
	[ASPEED_CLK_GATE_USBPORT1CLK] = { 14, 14, "usb-port1-gate",	NULL,	0 }, /* USB2 hub/USB2 host port 1/USB1.1 dev */

	[ASPEED_CLK_GATE_UART1CLK] =	{ ASPEED_CLK2_OFFSET + 16, -1, "uart1clk-gate",	"uart",	0 }, /* UART1 */
	[ASPEED_CLK_GATE_UART2CLK] =	{ ASPEED_CLK2_OFFSET + 17, -1, "uart2clk-gate",	"uart",	0 }, /* UART2 */

	[ASPEED_CLK_GATE_UART5CLK] =	{ 15, -1, "uart5clk-gate",	"uart",	0 }, /* UART5 */
	[ASPEED_CLK_GATE_ESPICLK] =	{ ASPEED_CLK2_OFFSET + 1, -1, "espiclk-gate",	NULL,	CLK_IS_CRITICAL }, /* eSPI */
	
	[ASPEED_CLK_GATE_MAC1CLK] =	{ 20, 11, "mac1clk-gate",	"mac",	0 }, /* MAC1 */
	[ASPEED_CLK_GATE_MAC2CLK] =	{ 21, 12, "mac2clk-gate",	"mac",	0 }, /* MAC2 */
	[ASPEED_CLK_GATE_RSACLK] =	{ 24,  4, "rsaclk-gate",	NULL,	0 }, /* HAC */
	
	[ASPEED_CLK_GATE_UART3CLK] =	{ ASPEED_CLK2_OFFSET + 18, -1, "uart3clk-gate",	"uart",	0 }, /* UART3 */
	[ASPEED_CLK_GATE_UART4CLK] =	{ ASPEED_CLK2_OFFSET + 19, -1, "uart4clk-gate",	"uart",	0 }, /* UART4 */
	
	[ASPEED_CLK_GATE_SDCLK] =	{ ASPEED_CLK2_OFFSET + 4, ASPEED_RESET2_OFFSET + 24, "sdclk-gate",		NULL,	0 }, /* SDIO/SD */
	
	[ASPEED_CLK_GATE_LHCCLK] =	{ ASPEED_CLK2_OFFSET + 5, -1, "lhclk-gate",		"lhclk", 0 }, /* LPC master/LPC+ */
	
	[ASPEED_CLK_GATE_SDEXTCLK] = { 31, -1, "sdextclk-gate",		"sdio",	0 }, /* For card clk scu310*/
	
	[ASPEED_CLK_GATE_EMMCCLK] = { 30, 16, "emmcclk-gate",		NULL,	0 }, /* For card clk */
	[ASPEED_CLK_GATE_EMMCEXTCLK] = { 27, -1, "emmcextclk-gate",		"emmc",	0 }, /* For card clk scu300*/

	[ASPEED_CLK_GATE_UART6CLK] =	{ 22, -1, "uart6clk-gate",	"uartx",	0 }, /* UART6 */
	[ASPEED_CLK_GATE_UART7CLK] =	{ 23, -1, "uart7clk-gate",	"uartx",	0 }, /* UART7 */
	[ASPEED_CLK_GATE_UART8CLK] =	{ 24, -1, "uart8clk-gate",	"uartx",	0 }, /* UART8 */
	[ASPEED_CLK_GATE_UART9CLK] =	{ 25, -1, "uart9clk-gate",	"uartx",	0 }, /* UART9 */
	[ASPEED_CLK_GATE_UART10CLK] =	{ 26, -1, "uart10clk-gate",	"uartx",	0 }, /* UART10 */
	[ASPEED_CLK_GATE_UART11CLK] =	{ 27, -1, "uart11clk-gate",	"uartx",	0 }, /* UART11 */
	[ASPEED_CLK_GATE_UART12CLK] =	{ 28, -1, "uart12clk-gate",	"uartx",	0 }, /* UART12 */
	[ASPEED_CLK_GATE_UART13CLK] =	{ 29, -1, "uart13clk-gate",	"uartx",	0 }, /* UART13 */
	
};

static const char * const eclk_parent_names[] = {
	"mpll",
	"hpll",
	"dpll",
};

static const struct clk_div_table ast2600_eclk_div_table[] = {
	{ 0x0, 2 },
	{ 0x1, 2 },
	{ 0x2, 3 },
	{ 0x3, 4 },
	{ 0x4, 5 },
	{ 0x5, 6 },
	{ 0x6, 7 },
	{ 0x7, 8 },
	{ 0 }
};

static const struct clk_div_table ast2600_mac_div_table[] = {
	{ 0x0, 4 }, /* Yep, really. Aspeed confirmed this is correct */
	{ 0x1, 4 },
	{ 0x2, 6 },
	{ 0x3, 8 },
	{ 0x4, 10 },
	{ 0x5, 12 },
	{ 0x6, 14 },
	{ 0x7, 16 },
	{ 0 }
};

static const struct clk_div_table ast2600_div_table[] = {
	{ 0x0, 4 },
	{ 0x1, 8 },
	{ 0x2, 12 },
	{ 0x3, 16 },
	{ 0x4, 20 },
	{ 0x5, 24 },
	{ 0x6, 28 },
	{ 0x7, 32 },
	{ 0 }
};

static struct clk_hw *aspeed_ast2600_calc_pll(const char *name, u32 val)
{
#if 0
	unsigned int mult, div;

	if (val & AST2400_HPLL_BYPASS_EN) {
		/* Pass through mode */
		mult = div = 1;
	} else {
		/* F = 24Mhz * (2-OD) * [(N + 2) / (D + 1)] */
		u32 n = (val >> 5) & 0x3f;
		u32 od = (val >> 4) & 0x1;
		u32 d = val & 0xf;

		mult = (2 - od) * (n + 2);
		div = d + 1;
	}
	return clk_hw_register_fixed_factor(NULL, name, "clkin", 0,
			mult, div);
#else
	//TODO
#endif	
};


struct aspeed_g6_clk_soc_data {
	const struct clk_div_table *div_table;
	const struct clk_div_table *eclk_div_table;
	const struct clk_div_table *mac_div_table;
	struct clk_hw *(*calc_pll)(const char *name, u32 val);
	unsigned int nr_resets;
};

static const struct aspeed_clk_soc_data ast2600_data = {
	.div_table = ast2600_div_table,
	.mac_div_table = ast2600_mac_div_table,
	.eclk_div_table = ast2600_eclk_div_table,	
	.calc_pll = aspeed_ast2600_calc_pll,
};



static int aspeed_g6_clk_is_enabled(struct clk_hw *hw)
{
	u32 clk = 0;
	u32 rst = 0;
	u32 reg;	
	u32 enval = 0;
	struct aspeed_clk_gate *gate = to_aspeed_clk_gate(hw);

	if(gate->reset_idx & 0x1f)
		rst = BIT((gate->reset_idx));
	else
		rst = BIT(gate->reset_idx - 32);

	if(gate->clock_idx & 0x1f)
		clk = BIT((gate->clock_idx));
	else
		clk = BIT(gate->clock_idx - 32);

	enval = (gate->flags & CLK_GATE_SET_TO_DISABLE) ? 0 : clk;

	printk("aspeed_g6_clk_is_enabled gate->clock_idx %d, reset_idx %d, envalx %x\n", gate->clock_idx, gate->reset_idx, enval);

	/*
	 * If the IP is in reset, treat the clock as not enabled,
	 * this happens with some clocks such as the USB one when
	 * coming from cold reset. Without this, aspeed_clk_enable()
	 * will fail to lift the reset.
	 */
	if (gate->reset_idx >= 0) {
		if(gate->reset_idx & 0x1f)
			regmap_read(gate->map, ASPEED_G6_RESET_CTRL, &reg);
		else
			regmap_read(gate->map, ASPEED_G6_RESET_CTRL2, &reg);

		if (reg & rst)
			return 0;
	}

	if(gate->clock_idx & 0x1f)
		regmap_read(gate->map, ASPEED_G6_CLK_STOP_CTRL, &reg);
	else
		regmap_read(gate->map, ASPEED_G6_CLK_STOP_CTRL2, &reg);

	return ((reg & clk) == enval) ? 1 : 0;
}

static int aspeed_g6_clk_enable(struct clk_hw *hw)
{
	struct aspeed_clk_gate *gate = to_aspeed_clk_gate(hw);
	unsigned long flags;
	u32 enval;
	u32 clk = 0;
	u32 rst = 0;
	
	if(gate->reset_idx & 0x1f)
		rst = BIT((gate->reset_idx));
	else
		rst = BIT(gate->reset_idx - 32);

	if(gate->clock_idx & 0x1f)
		clk = BIT((gate->clock_idx));
	else
		clk = BIT(gate->clock_idx - 32);

	spin_lock_irqsave(gate->lock, flags);

	if (aspeed_g6_clk_is_enabled(hw)) {
		printk("is enable return ~~~~ xxxxxxxxxxxxxxxxxxxxxxxx need check\n");
		spin_unlock_irqrestore(gate->lock, flags);
		return 0;
	}

	if (gate->reset_idx >= 0) {
		printk("put reset gate->reset_idx %d, rst %x \n", gate->reset_idx, rst);
		/* Put IP in reset */
		if(gate->reset_idx & 0x1f)		
			regmap_update_bits(gate->map, ASPEED_G6_RESET_CTRL, rst, rst);
		else
			regmap_update_bits(gate->map, ASPEED_G6_RESET_CTRL2, rst, rst);
		/* Delay 100us */
		udelay(100);
	}

	/* Enable clock */
	if(gate->clock_idx == aspeed_g6_gates[ASPEED_CLK_GATE_SDEXTCLK].clock_idx) {
		/* sd ext clk */
//		regmap_update_bits(gate->map, ASPEED_G6_CLK_SELECTION, ASPEED_SDIO_CLK_EN, ASPEED_SDIO_CLK_EN);
		printk("TODO ~~~ enable sd card clk xxxxx\n");
	} else {
		enval = (gate->flags & CLK_GATE_SET_TO_DISABLE) ? 0 : clk;
		printk("enval %x gate->flags %x enable g6 clk gate->clock_idx %d gate->reset_idx %d \n",enval, gate->flags, gate->clock_idx, gate->reset_idx);
		if(enval) {
			printk("write 80 to enable clk  %x \n", clk);
			if(gate->reset_idx & 0x1f)			
				regmap_write(gate->map, ASPEED_G6_CLK_STOP_CTRL, clk);
			else
				regmap_write(gate->map, ASPEED_G6_CLK_STOP_CTRL2, clk);
		} else {
			printk("write 84 clr to enable clk mask %x val %x\n", clk, enval);
			if(gate->reset_idx & 0x1f)
				regmap_write(gate->map, ASPEED_G6_CLK_STOP_CTRL + 0x04, clk);
			else
				regmap_write(gate->map, ASPEED_G6_CLK_STOP_CTRL2 + 0x04, clk);
		}
	}

	if (gate->reset_idx >= 0) {
		/* A delay of 10ms is specified by the ASPEED docs */
		mdelay(10);
		printk("clear reset rst %x\n", rst);
		/* Take IP out of reset */
		if(gate->reset_idx & 0x1f)
			regmap_write(gate->map, ASPEED_G6_RESET_CTRL + 0x04, rst);
		else
			regmap_write(gate->map, ASPEED_G6_RESET_CTRL2 + 0x04, rst);
	}

	spin_unlock_irqrestore(gate->lock, flags);

	return 0;
}

static void aspeed_g6_clk_disable(struct clk_hw *hw)
{
	struct aspeed_clk_gate *gate = to_aspeed_clk_gate(hw);
	unsigned long flags;
	u32 clk;
	u32 enval;

	if(gate->clock_idx & 0x1f)
		clk = BIT((gate->clock_idx));
	else
		clk = BIT(gate->clock_idx - 32);

	printk("aspeed_g6_clk_disable gate->clock_idx %d, reset_idx %d\n", gate->clock_idx, gate->reset_idx);

	spin_lock_irqsave(gate->lock, flags);

	enval = (gate->flags & CLK_GATE_SET_TO_DISABLE) ? clk : 0;

	if(enval) {
		if(gate->clock_idx & 0x1f)
			regmap_write(gate->map, ASPEED_G6_CLK_STOP_CTRL, clk);
		else
			regmap_write(gate->map, ASPEED_G6_CLK_STOP_CTRL2, clk);
	} else {
		if(gate->clock_idx & 0x1f)
			regmap_write(gate->map, ASPEED_G6_CLK_STOP_CTRL + 0x04, clk);
		else
			regmap_write(gate->map, ASPEED_G6_CLK_STOP_CTRL2 + 0x04, clk);
	}
	
	spin_unlock_irqrestore(gate->lock, flags);
}

static const struct clk_ops aspeed_g6_clk_gate_ops = {
	.enable = aspeed_g6_clk_enable,
	.disable = aspeed_g6_clk_disable,
	.is_enabled = aspeed_g6_clk_is_enabled,
};

static const u8 aspeed_g6_resets[] = {
	/* SCU40 resets */
//	[ASPEED_RESET_GRAPHIC] = 26, ???
	[ASPEED_RESET_XDMA]	= 25,
	[ASPEED_RESET_MCTP]	= 24,
	[ASPEED_RESET_P2X]	= 31,	
//	reserved bit 23	
	[ASPEED_RESET_JTAG_MASTER] = 22,
//	reserved bit 21
//	reserved bit 20	
//	reserved bit 19	
	[ASPEED_RESET_MIC]	= 18,
//	reserved bit 17
	[ASPEED_RESET_SDHCI]	= ASPEED_RESET2_OFFSET + 24,
	[ASPEED_RESET_UHCI]	= 15,
	[ASPEED_RESET_EHCI_P1]	= 14,
	[ASPEED_RESET_CRT]		= 13,
	[ASPEED_RESET_MAC2]	= 12,
	[ASPEED_RESET_MAC1]	= 11,
//	reserved bit 10	
//	reserved bit 9 
//	[ASPEED_RESET_PCI_VGA]	= 8,	???
	[ASPEED_RESET_2D]	= 7,
	[ASPEED_RESET_VIDEO]	= 6,
//	reserved bit 5
	[ASPEED_RESET_HACE]	= 4,
	[ASPEED_RESET_EHCI_P2]	= 3,
//	reserved bit 2
	[ASPEED_RESET_AHB]	=  1,	
	[ASPEED_RESET_SRAM_CTRL]	=  0,

	/*
	 * SCU50 resets start at an offset to separate them from
	 */
//	[ASPEED_RESET_PCIE_DIR] = 21,
//	[ASPEED_RESET_PCIE] 	= 20,
	
	[ASPEED_RESET_ADC]	= ASPEED_RESET2_OFFSET + 23,
//	[ASPEED_RESET_JTAG_MASTER2fs] = ASPEED_RESET2_OFFSET + 22,
//	[ASPEED_RESET_MAC4]	= ASPEED_RESET2_OFFSET + 21,
//	[ASPEED_RESET_MAC3]	= ASPEED_RESET2_OFFSET + 20,
//	reserved bit 6	
	[ASPEED_RESET_PWM]	= ASPEED_RESET2_OFFSET + 5,	 
	[ASPEED_RESET_PECI] = ASPEED_RESET2_OFFSET + 4,	
//	[ASPEED_RESET_MII] = ASPEED_RESET2_OFFSET + 3,
	[ASPEED_RESET_I2C]	= ASPEED_RESET2_OFFSET + 2,	
//	reserved bit 1
	[ASPEED_RESET_LPC]	= ASPEED_RESET2_OFFSET + 0,	
	[ASPEED_RESET_ESPI]	= ASPEED_RESET2_OFFSET + 0,
	[ASPEED_RESET_EMMC]	= 16,
};
static int aspeed_g6_reset_deassert(struct reset_controller_dev *rcdev,
				 unsigned long id)
{
	struct aspeed_reset *ar = to_aspeed_reset(rcdev);
	u32 reg = ASPEED_G6_RESET_CTRL;
	u32 bit = aspeed_g6_resets[id];

	if (bit >= ASPEED_RESET2_OFFSET) {
		bit -= ASPEED_RESET2_OFFSET;
		reg = ASPEED_G6_RESET_CTRL2;
	}

	reg += 0x04;
	return regmap_update_bits(ar->map, reg, BIT(bit), BIT(bit));
}

static int aspeed_g6_reset_assert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	struct aspeed_reset *ar = to_aspeed_reset(rcdev);
	u32 reg = ASPEED_G6_RESET_CTRL;
	u32 bit = aspeed_g6_resets[id];

	if (bit >= ASPEED_RESET2_OFFSET) {
		bit -= ASPEED_RESET2_OFFSET;
		reg = ASPEED_G6_RESET_CTRL2;
	}

	return regmap_update_bits(ar->map, reg, BIT(bit), BIT(bit));
}

static int aspeed_g6_reset_status(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	struct aspeed_reset *ar = to_aspeed_reset(rcdev);
	u32 reg = ASPEED_G6_RESET_CTRL;
	u32 bit = aspeed_g6_resets[id];
	int ret, val;

	if (bit >= ASPEED_RESET2_OFFSET) {
		bit -= ASPEED_RESET2_OFFSET;
		reg = ASPEED_G6_RESET_CTRL2;
	}

	ret = regmap_read(ar->map, reg, &val);
	if (ret)
		return ret;

	return !!(val & BIT(bit));
}

static const struct reset_control_ops aspeed_g6_reset_ops = {
	.assert = aspeed_g6_reset_assert,
	.deassert = aspeed_g6_reset_deassert,
	.status = aspeed_g6_reset_status,
};

static struct clk_hw *aspeed_g6_clk_hw_register_gate(struct device *dev,
		const char *name, const char *parent_name, unsigned long flags,
		struct regmap *map, u8 clock_idx, u8 reset_idx,
		u8 clk_gate_flags, spinlock_t *lock)
{
	struct aspeed_clk_gate *gate;
	struct clk_init_data init;
	struct clk_hw *hw;
	int ret;

	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &aspeed_g6_clk_gate_ops;
	init.flags = flags;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	gate->map = map;
	gate->clock_idx = clock_idx;
	gate->reset_idx = reset_idx;
	gate->flags = clk_gate_flags;
	gate->lock = lock;
	gate->hw.init = &init;

	hw = &gate->hw;
	ret = clk_hw_register(dev, hw);
	if (ret) {
		kfree(gate);
		hw = ERR_PTR(ret);
	}

	return hw;
}

static int aspeed_g6_clk_probe(struct platform_device *pdev)
{
	const struct aspeed_clk_soc_data *soc_data;
	struct device *dev = &pdev->dev;
	struct aspeed_reset *ar;
	struct regmap *map;
	struct clk_hw *hw;
	u32 val, rate;
	int i, ret;

	map = syscon_node_to_regmap(dev->of_node);
	if (IS_ERR(map)) {
		dev_err(dev, "no syscon regmap\n");
		return PTR_ERR(map);
	}

	ar = devm_kzalloc(dev, sizeof(*ar), GFP_KERNEL);
	if (!ar)
		return -ENOMEM;

	ar->map = map;

	ar->rcdev.owner = THIS_MODULE;
	ar->rcdev.nr_resets = ARRAY_SIZE(aspeed_g6_resets);
	ar->rcdev.ops = &aspeed_g6_reset_ops;
	ar->rcdev.of_node = dev->of_node;

	ret = devm_reset_controller_register(dev, &ar->rcdev);
	if (ret) {
		dev_err(dev, "could not register reset controller\n");
		return ret;
	}

	/* SoC generations share common layouts but have different divisors */
	soc_data = of_device_get_match_data(dev);
	if (!soc_data) {
		dev_err(dev, "no match data for platform\n");
		return -EINVAL;
	}

	/* UART clock div13 setting */
	regmap_read(map, ASPEED_G6_MISC_CTRL, &val);
	if (val & UART_DIV13_EN)
		rate = 24000000 / 13;
	else
		rate = 24000000;
	/* TODO: Find the parent data for the uart clock */
	hw = clk_hw_register_fixed_rate(dev, "uart", NULL, 0, rate);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	aspeed_g6_clk_data->hws[ASPEED_CLK_UART] = hw;

	/* UART6~13 clock div13 setting */
	regmap_read(map, 0x80, &val);

	if (val & BIT(31))
		rate = 24000000 / 13;
	else
		rate = 24000000;
	/* TODO: Find the parent data for the uart clock */
	hw = clk_hw_register_fixed_rate(dev, "uartx", NULL, 0, rate);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	aspeed_g6_clk_data->hws[ASPEED_CLK_UARTX] = hw;

	/*
	 * Memory controller (M-PLL) PLL. This clock is configured by the
	 * bootloader, and is exposed to Linux as a read-only clock rate.
	 */
	regmap_read(map, ASPEED_MPLL_PARAM, &val);
	hw = soc_data->calc_pll("mpll", val);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	aspeed_g6_clk_data->hws[ASPEED_CLK_MPLL] =	hw;

	/* EMMC clock divider (TODO: There's a gate too) */
#if 0	
	hw = clk_hw_register_divider_table(dev, "emmc", "hpll", 0,
			scu_g6_base + 0x300, 12, 3, 0,
			soc_data->div_table,
			&aspeed_clk_lock);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	aspeed_g6_clk_data->hws[ASPEED_CLK_EMMC] = hw;

	/* SD/SDIO clock divider (TODO: There's a gate too) */
	hw = clk_hw_register_divider_table(dev, "sdio", "hpll", 0,
			scu_g6_base + 0x310, 28, 3, 0,
			soc_data->div_table,
			&aspeed_clk_lock);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	aspeed_g6_clk_data->hws[ASPEED_CLK_SDIO] = hw;
		
#else		
	hw = clk_hw_register_fixed_rate(NULL, "emmc", NULL, 0, 100000000);
	aspeed_g6_clk_data->hws[ASPEED_CLK_EMMC] = hw;

	hw = clk_hw_register_fixed_rate(NULL, "sdio", NULL, 0, 200000000);
	aspeed_g6_clk_data->hws[ASPEED_CLK_SDIO] = hw;
		
#endif


	/* MAC AHB bus clock divider */
	hw = clk_hw_register_divider_table(dev, "mac", "hpll", 0,
			scu_g6_base + ASPEED_G6_CLK_SELECTION, 16, 3, 0,
			soc_data->mac_div_table,
			&aspeed_clk_lock);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	aspeed_g6_clk_data->hws[ASPEED_CLK_MAC] = hw;

//	ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~(SCU_ECLK_SOURCE_MASK | SCU_CLK_VIDEO_SLOW_EN)) | SCU_ECLK_SOURCE(2), AST_SCU_CLK_SEL);

	/* LPC Host (LHCLK) clock divider */
	hw = clk_hw_register_divider_table(dev, "lhclk", "hpll", 0,
			scu_g6_base + ASPEED_G6_CLK_SELECTION, 20, 3, 0,
			soc_data->div_table,
			&aspeed_clk_lock);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	aspeed_g6_clk_data->hws[ASPEED_CLK_LHCLK] = hw;

	/* P-Bus (BCLK) clock divider */
	hw = clk_hw_register_divider_table(dev, "bclk", "hpll", 0,
			scu_g6_base + ASPEED_G6_CLK_SELECTION2, 0, 2, 0,
			soc_data->div_table,
			&aspeed_clk_lock);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	aspeed_g6_clk_data->hws[ASPEED_CLK_BCLK] = hw;

	/* Fixed 24MHz clock */
	hw = clk_hw_register_fixed_rate(NULL, "fixed-24m", "clkin",
					0, 24000000);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	aspeed_g6_clk_data->hws[ASPEED_CLK_24M] = hw;

	/* Video Engine clock divider */
	hw = clk_hw_register_divider_table(dev, "eclk", NULL, 0,
			scu_g6_base + ASPEED_G6_CLK_SELECTION, 28, 3, 0,
			soc_data->eclk_div_table,
			&aspeed_clk_lock);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	aspeed_g6_clk_data->hws[ASPEED_CLK_ECLK] = hw;
	
	/*
	 * TODO: There are a number of clocks that not included in this driver
	 * as more information is required:
	 *   D2-PLL
	 *   D-PLL
	 *   YCLK
	 *   RGMII
	 *   RMII
	 *   UART[1..5] clock source mux
	 */

	for (i = 0; i < ARRAY_SIZE(aspeed_g6_gates); i++) {
		const struct aspeed_gate_data *gd = &aspeed_g6_gates[i];
		u32 gate_flags;

		/* Special case: the USB port 1 clock (bit 14) is always
		 * working the opposite way from the other ones.
		 */
		gate_flags = (gd->clock_idx == 14) ? 0 : CLK_GATE_SET_TO_DISABLE;
		hw = aspeed_g6_clk_hw_register_gate(dev,
				gd->name,
				gd->parent_name,
				gd->flags,
				map,
				gd->clock_idx,
				gd->reset_idx,
				gate_flags,
				&aspeed_clk_lock);
		if (IS_ERR(hw))
			return PTR_ERR(hw);
		aspeed_g6_clk_data->hws[i] = hw;
	}

	return 0;
};

static const struct of_device_id aspeed_g6_clk_dt_ids[] = {
	{ .compatible = "aspeed,ast2600-scu", .data = &ast2600_data },
	{ }
};

static struct platform_driver aspeed_g6_clk_driver = {
	.probe  = aspeed_g6_clk_probe,
	.driver = {
		.name = "aspeed-clk",
		.of_match_table = aspeed_g6_clk_dt_ids,
		.suppress_bind_attrs = true,
	},
};

static int __init aspeed_g6_clk_init(void)
{
	return platform_driver_register(&aspeed_g6_clk_driver);
}
core_initcall(aspeed_g6_clk_init);

static void __init aspeed_ast2600_cc(struct regmap *map)
{
	struct clk_hw *hw;
	u32 val, freq, div;

	freq = 25000000;

	hw = clk_hw_register_fixed_rate(NULL, "clkin", NULL, 0, freq);
	pr_debug("clkin @%u MHz\n", freq / 1000000);

#if 1//def CONFIG_ASPEED_FPGA
	/*
	 * High-speed PLL clock derived from the crystal. This the CPU clock,
	 * and we assume that it is enabled
	 */
	hw = clk_hw_register_fixed_rate(NULL, "hpll", NULL, 0, freq * 2);
	aspeed_g6_clk_data->hws[ASPEED_CLK_HPLL] = hw;

	hw = clk_hw_register_fixed_rate(NULL, "ahb", NULL, 0, freq);
	aspeed_g6_clk_data->hws[ASPEED_CLK_AHB] = hw;

	hw = clk_hw_register_fixed_rate(NULL, "apb", NULL, 0, freq);
	aspeed_g6_clk_data->hws[ASPEED_CLK_APB] = hw;

#else
	/*
	 * High-speed PLL clock derived from the crystal. This the CPU clock,
	 * and we assume that it is enabled
	 */
//	regmap_read(map, ASPEED_HPLL_PARAM, &val);
//	aspeed_g6_clk_data->hws[ASPEED_CLK_HPLL] = aspeed_ast2600_calc_pll("hpll", val);

	/* Strap bits 11:9 define the AXI/AHB clock frequency ratio (aka HCLK)*/
	regmap_read(map, ASPEED_STRAP, &val);
	val = (val >> 9) & 0x7;
	WARN(val == 0, "strapping is zero: cannot determine ahb clock");
	div = 2 * (val + 1);
	hw = clk_hw_register_fixed_factor(NULL, "ahb", "hpll", 0, 1, div);
	aspeed_g6_clk_data->hws[ASPEED_CLK_AHB] = hw;

	/* APB clock clock selection register SCU08 (aka PCLK) */
	regmap_read(map, ASPEED_G6_CLK_SELECTION, &val);
	val = (val >> 23) & 0x7;
	div = 4 * (val + 1);
	hw = clk_hw_register_fixed_factor(NULL, "apb", "hpll", 0, 1, div);
	aspeed_g6_clk_data->hws[ASPEED_CLK_APB] = hw;
#endif
};

static void __init aspeed_g6_cc_init(struct device_node *np)
{
	struct regmap *map;
	u32 val;
	int ret;
	int i;

	scu_g6_base = of_iomap(np, 0);
	if (!scu_g6_base)
		return;

	aspeed_g6_clk_data = kzalloc(struct_size(aspeed_g6_clk_data, hws,
				      ASPEED_G6_NUM_CLKS), GFP_KERNEL);
	if (!aspeed_g6_clk_data)
		return;

	/*
	 * This way all clocks fetched before the platform device probes,
	 * except those we assign here for early use, will be deferred.
	 */
	for (i = 0; i < ASPEED_G6_NUM_CLKS; i++)
		aspeed_g6_clk_data->hws[i] = ERR_PTR(-EPROBE_DEFER);

	map = syscon_node_to_regmap(np);
	if (IS_ERR(map)) {
		pr_err("no syscon regmap\n");
		return;
	}
	/*
	 * We check that the regmap works on this very first access,
	 * but as this is an MMIO-backed regmap, subsequent regmap
	 * access is not going to fail and we skip error checks from
	 * this point.
	 */
#if 0	 
	ret = regmap_read(map, ASPEED_STRAP, &val);
	if (ret) {
		pr_err("failed to read strapping register\n");
		return;
	}
#endif
	aspeed_ast2600_cc(map);	
	aspeed_g6_clk_data->num = ASPEED_G6_NUM_CLKS;
	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, aspeed_g6_clk_data);
	if (ret)
		pr_err("failed to add DT provider: %d\n", ret);
};
CLK_OF_DECLARE_DRIVER(aspeed_cc_g6, "aspeed,ast2600-scu", aspeed_g6_cc_init);
