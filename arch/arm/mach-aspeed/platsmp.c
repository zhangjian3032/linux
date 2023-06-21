// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) ASPEED Technology Inc.
// Copyright IBM Corp.

#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/smp.h>

#define BOOT_ADDR	0x00
#define BOOT_SIG	0x04
#define BOOT_READY	0x08

static struct device_node *secboot_node;

static int aspeed_g6_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	void __iomem *base;

	base = of_iomap(secboot_node, 0);
	if (!base) {
		pr_err("could not map the secondary boot base!");
		return -ENODEV;
	}

	writel_relaxed(0, base + BOOT_ADDR);
	writel_relaxed(__pa_symbol(secondary_startup_arm), base + BOOT_ADDR);
	writel_relaxed((0xABBAAB00 | (cpu & 0xff)), base + BOOT_SIG);

	dsb_sev();

	iounmap(base);

	return 0;
}

static void __init aspeed_g6_smp_prepare_cpus(unsigned int max_cpus)
{
	void __iomem *base;
	u32 ret;

	secboot_node = of_find_compatible_node(NULL, NULL, "aspeed,ast2600-smpmem");
	if (!secboot_node) {
		pr_err("secboot device node found!!\n");
		return;
	}

	base = of_iomap(secboot_node, 0);
	if (!base) {
		pr_err("could not map the secondary boot base!");
		return;
	}
	__raw_writel(0xBADABABA, base + BOOT_SIG);
	
	ret = __raw_readl(base + BOOT_READY);
	if (ret != 0XBABECAFE) {
		// if cpu1 not ready, notify again
		__raw_writel(0XBABECAFE, base + BOOT_READY);
		dsb_sev();
		 /* ensure going to next smp_go loop */
		mdelay(1);
	}

	iounmap(base);
}

static const struct smp_operations aspeed_smp_ops __initconst = {
	.smp_prepare_cpus	= aspeed_g6_smp_prepare_cpus,
	.smp_boot_secondary	= aspeed_g6_boot_secondary,
};

CPU_METHOD_OF_DECLARE(aspeed_smp, "aspeed,ast2600-smp", &aspeed_smp_ops);
