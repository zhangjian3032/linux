// SPDX-License-Identifier: GPL-2.0
/*
 * drivers/watchdog/pilot_wdt.c
 * Watchdog driver for pilot processors
 *
 * Copyright (c) 2018, Aspeed Technology Inc.
 * Author: Ashok Reddy Soma <ashok.soma@aspeedtech.com>
 *
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of_device.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/smp.h>
#include <linux/sched.h>

/*
 * Watchdog timer block registers.
 */
#define SYSWCR			0x00
#define SYSWRERL		0x04
#define SYSWRERH		0x08
#define SYSWCFR			0x0C

#define SEC             	381
#define WDT_EN			0x800000
#define PRE_TRIG_EN		0x400000
#define TRIG_WDT		0x1
#define RST_PULSE_WIDTH		0xC	/*using default value 0xC*(40.96us)=491.52us*/
#define WDT_MAX_CYCLE_COUNT	0xffff
#define WDT_DEFAULT_TIMEOUT	120

struct pilot_wdt {
	struct watchdog_device	wdd;
	u32 iobase;
};

static int heartbeat = -1;		/* module parameter (seconds) */
module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Initial watchdog heartbeat in seconds");

static int pretimeout = 20;		/* module parameter (seconds) */
module_param(pretimeout, int, 0);
MODULE_PARM_DESC(pretimeout, "Initial watchdog pretimeout in seconds");

static unsigned int wdt_max_duration;	/* (seconds) */
static DEFINE_SPINLOCK(wdt_lock);

static int pilot_wdt_update_timeout(int new_time);

static struct pilot_wdt *to_pilot_wdt(struct watchdog_device *wdd)
{
	return container_of(wdd, struct pilot_wdt, wdd);
}

static int pilot_wdt_ping(struct watchdog_device *wdd)
{
	volatile u32 reg;
	volatile u32 wdt_cnt;
	struct pilot_wdt *wdt = to_pilot_wdt(wdd);

	spin_lock(&wdt_lock);

	/* Reload watchdog duration */
	wdt_cnt = ((SEC * pretimeout) << 16) | (SEC * heartbeat);
	writel(wdt_cnt, wdt->iobase + SYSWCFR);

	/* Trigger reset on watchdog */
	reg = readl(wdt->iobase + SYSWCR);
	reg |= WDT_EN;
	writel(reg, wdt->iobase + SYSWCR);

	spin_unlock(&wdt_lock);

	return 0;
}

static void pilot_wdt_enable(struct watchdog_device *wdd)
{
	volatile u32 reg;
	volatile u32 wdt_cnt;
	struct pilot_wdt *wdt = to_pilot_wdt(wdd);

	spin_lock(&wdt_lock);

	/* Set watchdog duration */
	wdt_cnt = ((SEC * pretimeout) << 16) | (SEC * heartbeat);
	writel(wdt_cnt, wdt->iobase + SYSWCFR);

	/* Enable ARM reset on watchdog */
	writel(0x3, wdt->iobase + SYSWRERL);
	writel(0x0, wdt->iobase + SYSWRERH);

	/* Trigger reset on watchdog */
	reg = readl(wdt->iobase + SYSWCR);
	reg |= WDT_EN;
	writel(reg, wdt->iobase + SYSWCR);

	/* Enable watchdog timer */
	reg = readl(wdt->iobase + SYSWCR);
	reg &=~(0xff<<8);
	reg |= PRE_TRIG_EN;
	reg |= TRIG_WDT;
	reg |= RST_PULSE_WIDTH << 8;

	writel(reg, wdt->iobase + SYSWCR);

	spin_unlock(&wdt_lock);
}

static int pilot_wdt_start(struct watchdog_device *wdd)
{
	pilot_wdt_update_timeout(wdd->timeout);
        pilot_wdt_enable(wdd);
        return 0;
}

static int pilot_wdt_stop(struct watchdog_device *wdd)
{
	volatile u32 reg;
	struct pilot_wdt *wdt = to_pilot_wdt(wdd);

	spin_lock(&wdt_lock);

	/* Disable watchdog timer */
	reg = readl(wdt->iobase + SYSWCR);
	reg &= ~(0x1);
	writel(reg, wdt->iobase + SYSWCR);

	spin_unlock(&wdt_lock);

	return 0;
}

static int pilot_wdt_set_timeout(struct watchdog_device *wdd,
                                  unsigned int timeout)
{
	wdd->timeout = timeout;
	pilot_wdt_update_timeout(wdd->timeout);
        pilot_wdt_enable(wdd);

	return 0;
}

static int pilot_wdt_restart(struct watchdog_device *wdd,
                              unsigned long action, void *data)
{
        pilot_wdt_enable(wdd);
        return 0;
}

static int pilot_wdt_update_timeout(int new_time)
{
	if ((new_time <= 0) || (new_time > wdt_max_duration))
		return -EINVAL;

	/* Set new watchdog time to be used when
	 * pilot_wdt_enable() or pilot_wdt_ping() is called. */
	heartbeat = new_time;
	return 0;
}

static int pilot_wdt_set_pretimeout (int new_pretimeout)
{
	if ((new_pretimeout <= 0) || (new_pretimeout > wdt_max_duration))
		return -EINVAL;

	/* Set new watchdog pre timeout to be used when
	 * pilot_wdt_enable() or pilot_wdt_ping() is called. */
	pretimeout = new_pretimeout;
	return 0;
}

static const struct watchdog_ops pilot_wdt_fops = {
	.start		= pilot_wdt_start,
	.stop		= pilot_wdt_stop,
	.ping		= pilot_wdt_ping,
	.set_timeout	= pilot_wdt_set_timeout,
	.restart	= pilot_wdt_restart,
	.owner		= THIS_MODULE,
};

static const struct watchdog_info pilot_wdt_info = {
        .options        = WDIOF_KEEPALIVEPING
                        | WDIOF_MAGICCLOSE
                        | WDIOF_SETTIMEOUT,
        .identity       = KBUILD_MODNAME,
};

static int pilot_wdt_probe(struct platform_device *pdev)
{
	struct pilot_wdt *pilot;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	int ret;

	printk(KERN_DEBUG "Entered %s \n", __FUNCTION__);

	pilot = (struct pilot_wdt *)devm_kzalloc(&pdev->dev, sizeof(struct pilot_wdt), GFP_KERNEL);
	if (!pilot)
		return -ENOMEM;

	wdt_max_duration = WDT_MAX_CYCLE_COUNT / SEC;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pilot->iobase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pilot->iobase))
		return PTR_ERR(pilot->iobase);

	pilot->wdd.info = &pilot_wdt_info;
        pilot->wdd.ops = &pilot_wdt_fops;
        pilot->wdd.max_hw_heartbeat_ms = (wdt_max_duration * 1000);
        pilot->wdd.parent = &pdev->dev;

	pilot->wdd.timeout = WDT_DEFAULT_TIMEOUT;
        watchdog_init_timeout(&pilot->wdd, 0, &pdev->dev);

	of_property_read_u32(np, "pretimeout-sec", &pretimeout);
	if (pilot_wdt_set_pretimeout(pretimeout))
		pretimeout = wdt_max_duration;

	of_property_read_u32(np, "timeout-sec", &heartbeat);
	if (pilot_wdt_update_timeout(heartbeat))
		heartbeat = wdt_max_duration;

	pr_info("Initial timeout %d sec pretimeout %d sec\n",
		heartbeat, pretimeout);

        ret = devm_watchdog_register_device(&pdev->dev, &pilot->wdd);
        if (ret) {
                dev_err(&pdev->dev, "failed to register\n");
                return ret;
        }

	return 0;
}

static const struct of_device_id pilot_wdt_of_match[] = {
	{ .compatible = "aspeed,pilot-wdt-1" },
	{ .compatible = "aspeed,pilot-wdt-2" },
	{ .compatible = "aspeed,pilot-wdt-3" },
	{ }
};

MODULE_DEVICE_TABLE(of, pilot_wdt_of_match);

static struct platform_driver pilot_wdt_driver = {
	.probe		= pilot_wdt_probe,
	.driver		= {
		.name		= KBUILD_MODNAME,
		.of_match_table =  pilot_wdt_of_match,
	},
};

module_platform_driver(pilot_wdt_driver);

MODULE_AUTHOR("Ashok Reddy Soma<ashok.soma@aspeedtech.com>");
MODULE_DESCRIPTION("Pilot Watchdog");
MODULE_LICENSE("GPL");
