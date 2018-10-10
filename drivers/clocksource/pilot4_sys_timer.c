/*
 * Copyright (C) 2018 ASpeed Technology.
 *
 * Author:
 *	Shivah Shankar<shivahshankar.shankarnarayanro@aspeedtech.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>
#include <linux/delay.h>

#include <asm/mach/time.h>
#include <asm/smp_twd.h>

static void __iomem *pilot4_timer_base;
#define SE_TIMER_VA_BASE	((unsigned char *)pilot4_timer_base)
#define CLOCK_TICK_RATE                    (25*1000*1000)
#define OUR_LATCH  ((CLOCK_TICK_RATE + HZ/2) / HZ)      /* For divider */
static struct clock_event_device pilot_clockevent;
static cycle_t notrace pilot4_read_cycles(struct clocksource *cs)
{
	return (cycle_t) (0xffffffff - *((volatile unsigned long *)(SE_TIMER_VA_BASE + 0x14 +0x4)));
}
static struct clocksource pilot_clk_src = {
	.name           = "pilot4_timer2",
	.rating         = 200,
	.read           = pilot4_read_cycles,
	.mask           = CLOCKSOURCE_MASK(32),
	.flags          = CLOCK_SOURCE_IS_CONTINUOUS,
};
static int pilot_set_next_event(unsigned long evt,
			struct clock_event_device *unused)
{
	unsigned long ctrl = 1;//Timer enable
	*((volatile unsigned long *)(SE_TIMER_VA_BASE+0x08)) = 0;//Disable
	*((volatile unsigned long *)(SE_TIMER_VA_BASE)) = evt;
	/* Enable Interrupts and timer */
	*((volatile unsigned long *)(SE_TIMER_VA_BASE+0x08)) = ctrl;
	return 0;
}
static int pilot_set_periodic(struct clock_event_device *clk)
{
	volatile unsigned long dummy_read;
	unsigned long ctrl;
	printk("CLOCK_EVT_MODE_PERIODIC OUR_LATCH %x CLOCK_TICK_RATE %d\n",  OUR_LATCH, CLOCK_TICK_RATE);
	/* timer load already set up */
	/* Disable timer and interrups */
	*((volatile unsigned long *)(SE_TIMER_VA_BASE+0x08)) = 0x0;
	dummy_read = *((volatile unsigned long*) (SE_TIMER_VA_BASE+0x0c));
	/* Load counter values */
	*((volatile unsigned long *)(SE_TIMER_VA_BASE)) = (OUR_LATCH);
	ctrl = 0x3;
	*((volatile unsigned long *)(SE_TIMER_VA_BASE+0x08)) = ctrl;
	return 0;
}
static int pilot_set_oneshot(struct clock_event_device *clk)
{
	volatile unsigned long dummy_read;
	unsigned long ctrl;
	printk("CLOCK_EVT_MODE_ONESHOT OUR_LATCH %x CLOCK_TICK_RATE %d\n",  OUR_LATCH, CLOCK_TICK_RATE);
	dummy_read = *((volatile unsigned long*) (SE_TIMER_VA_BASE+0x0c));
	/* Load counter values */
	/* period set, and timer enabled in 'next_event' hook */
	ctrl = 0x0;//One shot
	*((volatile unsigned long *)(SE_TIMER_VA_BASE+0x08)) = ctrl;
	return 0;
}
static int pilot_shutdown(struct clock_event_device *clk)
{
	unsigned long ctrl = 0;
	*((volatile unsigned long *)(SE_TIMER_VA_BASE+0x08)) = ctrl;
	return 0;
}
static struct clock_event_device pilot_clockevent = {
	.name           = "pilot_sys_timer1",
#ifdef CONFIG_SMP
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
#else
	.features       = CLOCK_EVT_FEAT_PERIODIC,
#endif
	.set_next_event = pilot_set_next_event,
	.set_state_shutdown = pilot_shutdown,
	.set_state_periodic = pilot_set_periodic,
	.set_state_oneshot = pilot_set_oneshot,
	.rating         = 300,
};

static void __init clk_src_pilot_init(void)
{

       /* timer load already set up */
       /* Disable timer and interrups */
       *((volatile unsigned long *)(SE_TIMER_VA_BASE+0x08 +0x14)) = 0x0;
       /* Load counter values */
       *((volatile unsigned long *)(SE_TIMER_VA_BASE + 0x14)) = (0xffffffff);
        printk("Registering clock source timercrnt addr %x\n", (SE_TIMER_VA_BASE + 0x14 +0x4));
       *((volatile unsigned long *)(SE_TIMER_VA_BASE + 0x8 + 0x14)) = 5;//no interrupt ,free running , start
        clocksource_register_hz(&pilot_clk_src, 25000000);
}
static irqreturn_t pilot_timer_interrupt(int irq, void *dev_id)
{
	volatile unsigned long dummy_read;
	struct clock_event_device *evt = &pilot_clockevent;
	printk("pilot_timer_interrupt irq %d\n", irq);
	/* Clear Interrupt */
	dummy_read = *((volatile unsigned long*) (SE_TIMER_VA_BASE+0x0c));

	evt->event_handler(evt);

	return IRQ_HANDLED;
}
static irqreturn_t pilot_timer_interrupt(int irq, void *dev_id);
static struct irqaction se_pilot4_timer_irq = {
	.name		= "Pilot4 Timer Tick",
	.handler	= pilot_timer_interrupt,
	.flags		= IRQF_TIMER | IRQF_IRQPOLL,
};

void __init se_pilot4_timer_init(struct device_node *np)
{
	int  err, irq;

	pilot4_timer_base = of_iomap(np, 0);

	pilot_clockevent.cpumask = cpumask_of(smp_processor_id());
	clockevents_config_and_register(&pilot_clockevent, 25*1000*1000,
					0xff, 0xffffffff);
	irq = irq_of_parse_and_map(np, 0);
	if (irq <= 0) {
		pr_err("Can't parse IRQ");
		err = -EINVAL;
		goto out_unmap;
	}

	/* Setup Timer Interrupt routine */
	err = setup_irq(irq, &se_pilot4_timer_irq);
	printk("mult %x shift %x max_delta_ns %llx min_delta_ns %llx\n",pilot_clockevent.mult, pilot_clockevent.shift, pilot_clockevent.max_delta_ns, pilot_clockevent.min_delta_ns);
	printk("Pilot-4 Timer configured\n");
	clk_src_pilot_init();
out_unmap:
	return;
}


CLOCKSOURCE_OF_DECLARE(pilot, "pilot,p4-timer", se_pilot4_timer_init);
