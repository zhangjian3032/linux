/*
 * Copyright 2018 ASpeed Technology.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Secondary CPU kernel startup is a 2 step process. The primary CPU
 * starts the secondary CPU by giving it the address of the kernel and
 * then sending it an event to wake it up. The secondary CPU then
 * starts the kernel and tells the primary CPU it's up and running.
 */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <asm/cacheflush.h>
#include <asm/smp_scu.h>
#include <asm/smp_plat.h>

#include "pilot4_asic.h"
static DEFINE_SPINLOCK(boot_lock);

extern void __iomem *p4smp_base_addr;
extern void __iomem *p4smp_scu_base;

void platform_secondary_init(unsigned int cpu)
{
#ifdef PILOT_F0
	__raw_writel(BOOT_STATUS_CPU1_UP, OCM_HIGH_BASE + BOOT_STATUS_OFFSET);
#else
	p4smp_base_addr = ioremap(PILOT4_SMP_BASE_ADDR, SZ_128);	
	__raw_writel(0xBADABABA, p4smp_base_addr + 0xC);
#endif
	wmb();
	//Synchronize with boot thread
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

int pilot4_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;
        /*
	 * set synchronisation state between this boot processor
	 * and the secondary one
	 */
        spin_lock(&boot_lock);
#ifdef PILOT_F0
	/* Initialize the boot status and give the secondary core
	 * the start address of the kernel, let the write buffer drain
	 */
	 __raw_writel(virt_to_phys(secondary_startup),
				        OCM_HIGH_BASE + BOOT_ADDR_OFFSET);
#else
        __raw_writel(0, p4smp_base_addr + 4);
	__raw_writel(virt_to_phys(secondary_startup), p4smp_base_addr + 4);
	__raw_writel(0xABBAADDA, p4smp_base_addr + 0xC);
#endif
	wmb();
	printk("Sending event to CPU1, wake up lazy bones!!!!\n");
	/*
	 * Send an event to wake the secondary core from WFE state.
	 */
	sev();
	/*
	 * Wait for the other CPU to boot, but timeout if it doesn't
	 */
	 timeout = jiffies + (1 * HZ);
#ifdef PILOT_F0
	while ((__raw_readl(OCM_HIGH_BASE + BOOT_STATUS_OFFSET) !=
				BOOT_STATUS_CPU1_UP) &&
				(time_before(jiffies, timeout)))
		rmb();
#else
	while ((__raw_readl(p4smp_base_addr+ 0xC) !=
				0xBADABABA) &&
				(time_before(jiffies, timeout)))
		rmb();
#endif
	printk("Second Guy up, lot to do!!!!\n");
	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return 0;
}




	
/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
static void __init pilot4_smp_init_cpus(void)
{
	int i, ncores;

	ncores = scu_get_core_count(p4smp_scu_base);

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);

}

void __init pilot4_smp_prepare_cpus(unsigned int max_cpus)
{
	scu_enable(p4smp_scu_base);
}

struct smp_operations pilot4_smp_ops __initdata = {
	.smp_init_cpus		= pilot4_smp_init_cpus,
	.smp_prepare_cpus	= pilot4_smp_prepare_cpus,
	.smp_boot_secondary	= pilot4_boot_secondary,
	.smp_secondary_init	= platform_secondary_init,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= NULL,
	.cpu_kill		= NULL,
#endif
};

CPU_METHOD_OF_DECLARE(pilot4_asic_smp, "aspeed,pilot4-asic-smp", &pilot4_smp_ops);
