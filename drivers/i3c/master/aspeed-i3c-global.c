// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 ASPEED Technology Inc.
 *
 * Author: Dylan Hung <dylan_hung@aspeedtech.com>
 * Based on a work from: Ryan Chen <ryan_chen@aspeedtech.com>
 */
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/reset.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define I3C_GLOBAL_REG0(x)		(0x10 + (x * 0x10))
#define  SDK_PULLUP_EN_MASK		GENMASK(29, 28)
#define  SDA_PULLUP_EN_2K		(0x1 << 28)
#define  SDA_PULLUP_EN_750		(0x2 << 28)
#define  SDA_PULLUP_EN_545		(0x3 << 28)

#define I3C_GLOBAL_REG1(x)		(0x14 + (x * 0x10))
union i3c_global_reg1 {
	uint32_t value;
	struct {
		unsigned int i2c_mode : 1;	/* bit[0] */
		unsigned int test_mode : 1;	/* bit[1] */
		unsigned int act_mode : 2;	/* bit[ 3: 2] */
		unsigned int pending_int : 4;	/* bit[ 7: 4] */
#define DEF_SLV_STATIC_ADDR		0x74
		unsigned int sa : 7;		/* bit[14: 8] */
		unsigned int sa_en : 1;		/* bit[15] */
#define DEF_SLV_INST_ID			0x4
		unsigned int inst_id : 4;	/* bit[19:16] */
		unsigned int rsvd : 12;		/* bit[31:20] */
	} fields;
};


struct aspeed_i3c_global {
	void __iomem		*base;
	struct reset_control	*rst;
};

static const struct of_device_id aspeed_i3c_of_match[] = {
	{ .compatible = "aspeed,ast2600-i3c-global", },
	{},
};

static int aspeed_i3c_global_probe(struct platform_device *pdev)
{
	struct aspeed_i3c_global *i3c_global;
	struct device_node *node = pdev->dev.of_node;
	union i3c_global_reg1 reg1;
	u32 reg0, num_of_i3cs;
	u32 *pullup_r_conf;
	int i, ret;

	i3c_global = kzalloc(sizeof(*i3c_global), GFP_KERNEL);
	if (!i3c_global)
		return -ENOMEM;

	i3c_global->base = of_iomap(node, 0);
	if (!i3c_global->base)
		return -ENOMEM;

	i3c_global->rst = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(i3c_global->rst)) {
		dev_err(&pdev->dev,
			"missing or invalid reset controller device tree entry");
		return PTR_ERR(i3c_global->rst);
	}

	reset_control_assert(i3c_global->rst);
	udelay(3);
	reset_control_deassert(i3c_global->rst);

	ret = of_property_read_u32(pdev->dev.of_node, "ni3cs", &num_of_i3cs);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to get number of i3c devices");
		return -ENOMEM;
	}

	pullup_r_conf = kmalloc(sizeof(u32) * num_of_i3cs, GFP_KERNEL);
	if (!pullup_r_conf)
		return -ENOMEM;

	ret = of_property_read_u32_array(pdev->dev.of_node, "pull-up-resistors",
					 pullup_r_conf, num_of_i3cs);
	if (ret < 0) {
		dev_warn(&pdev->dev,
			 "use 2K Ohm SDA pull up resistor by default");
		for (i = 0; i < num_of_i3cs; i++)
			pullup_r_conf[i] = 2000;
	}

	reg1.value = 0;
	reg1.fields.inst_id = DEF_SLV_INST_ID;
	reg1.fields.sa = DEF_SLV_STATIC_ADDR;
	reg1.fields.pending_int = 0xc;
	reg1.fields.act_mode = 0x1;

	for (i = 0; i < num_of_i3cs; i++) {
		reg0 = readl(i3c_global->base + I3C_GLOBAL_REG0(i));
		reg0 &= SDK_PULLUP_EN_MASK;
		switch (pullup_r_conf[i]) {
		case 750:
			reg0 |= SDA_PULLUP_EN_750;
			break;
		case 545:
			reg0 |= SDA_PULLUP_EN_545;
			break;
		case 2000:
		default:
			reg0 |= SDA_PULLUP_EN_2K;
			break;
		}
		writel(reg0, i3c_global->base + I3C_GLOBAL_REG0(i));
		writel(reg1.value, i3c_global->base + I3C_GLOBAL_REG1(i));
	}

	kfree(pullup_r_conf);

	return 0;
}

static struct platform_driver aspeed_i3c_driver = {
	.probe  = aspeed_i3c_global_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_i3c_of_match,
	},
};

static int __init aspeed_i3c_global_init(void)
{
	return platform_driver_register(&aspeed_i3c_driver);
}
postcore_initcall(aspeed_i3c_global_init);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_AUTHOR("Dylan Hung <dylan_hung@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED I3C Global Driver");
MODULE_LICENSE("GPL v2");
