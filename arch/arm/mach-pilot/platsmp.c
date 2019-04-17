// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018 Aspeed Technology Inc.
// Shivah Shankar S <shivahshankar.shankarnarayanrao@aspeedtech.com>
//
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <asm/cacheflush.h>
#include <asm/smp.h>
#include <asm/smp_plat.h>
#include <asm/smp_scu.h>

#define PILOT_BOOT_SIG_REG_OFFSET 0xC
#define PILOT_BOOT_ADDR_REG_OFFSET 0x4

extern void pilot_secondary_startup(void);

unsigned char *secboot_base;
static int pilot_smp_boot_secondary(unsigned int cpu,
				      struct task_struct *idle)
{
	struct device_node *secboot_node;
	int retval = 0;
	unsigned int timeout;	

	secboot_node = of_find_compatible_node(NULL, NULL, "aspeed,pilot-secboot-reg");
	if (!secboot_node) {
		pr_err("secboot device node not found!!\n");
		retval = -ENODEV;
		goto out;
	}
	pr_err("secboot device node found!!\n");
	secboot_base = (unsigned char *) of_iomap(secboot_node, 0);
	if (!secboot_base) {
		pr_err("could not map the secondary boot base!");
		retval = -ENOMEM;
		goto out;
	}

        __raw_writel(0, secboot_base + PILOT_BOOT_ADDR_REG_OFFSET);
	__raw_writel(__pa_symbol(pilot_secondary_startup), secboot_base + PILOT_BOOT_ADDR_REG_OFFSET);
	__raw_writel(0xABBAADDA, secboot_base + PILOT_BOOT_SIG_REG_OFFSET);
	wmb();
	
	/* barrier it to make sure everyone sees it */
	dsb_sev();

	/*
	 * Wait for the other CPU to boot, but timeout if it doesn't
	 */
	
	timeout = jiffies + (1 * HZ);
	while ((__raw_readl(secboot_base + PILOT_BOOT_SIG_REG_OFFSET) !=
				0xBADABABE) &&
				(time_before(jiffies, timeout)))
		rmb();
		
	printk("Returning from secondary successfully\n");
	iounmap(secboot_base);
out:
	return retval;
}

static void __init pilot_smp_prepare_cpus(unsigned int max_cpus)
{
	struct device_node *scu_node;
	void __iomem *scu_base;

	scu_node = of_find_compatible_node(NULL, NULL, "arm,cortex-a9-scu");
	if (!scu_node) {
		pr_err("no pilot scu device node found!\n");
		return;
	}
	scu_base = of_iomap(scu_node, 0);
	if (!scu_base) {
		pr_err("could not map pilot scubase !\n");
		return;
	}

	unsigned int scu_ctrl = 0;
	scu_ctrl = readl_relaxed(scu_base + 0x0);
	/* already enabled? */
		if (scu_ctrl & 0x1)
			printk("ASPEED : Already enabled yet\n");
		else
			printk("ASPEED : Not enabled yet\n");
	scu_ctrl &= ~(0x1);

	writel_relaxed(scu_ctrl, scu_base + 0x0);

	printk("SCU COntrol %x\n", readl_relaxed(scu_base + 0x0));
	printk("SCU COnfig %x\n", readl_relaxed(scu_base + 0x4));
	printk("SCU COntrol %x\n", readl_relaxed(scu_base + 0x8));
	printk("SCU COnfig %x\n", readl_relaxed(scu_base + 0xC));
	printk("SCU COntrol %x\n", readl_relaxed(scu_base + 0x40));
	printk("SCU COnfig %x\n", readl_relaxed(scu_base + 0x44));
	printk("SCU COntrol %x\n", readl_relaxed(scu_base + 0x50));
	printk("SCU COnfig %x\n", readl_relaxed(scu_base + 0x54));

	scu_enable(scu_base);

	iounmap(scu_base);
}

static void pilot_smp_secondary_init (unsigned int cpu){
	printk("SECONDARY is writing a sign\n\n");
	 __raw_writel(0xBADABABE, secboot_base + PILOT_BOOT_SIG_REG_OFFSET);
}


static struct smp_operations pilot_smp_ops __initdata = {
	.smp_prepare_cpus = pilot_smp_prepare_cpus,
	.smp_secondary_init = pilot_smp_secondary_init,
	.smp_boot_secondary = pilot_smp_boot_secondary,
};

CPU_METHOD_OF_DECLARE(pilot_smp, "aspeed,pilot-smp", &pilot_smp_ops);
