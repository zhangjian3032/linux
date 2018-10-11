/*
 * drivers/watchdog/pilot_wdt.c
 *
 * Watchdog driver for pilot processors
 *
 * Author: Sylver Bruneau <sylver.bruneau@googlemail.com>
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

/******************************************************************************
 *
 * Copyright (c) 2010-2014, Emulex Corporation.
 *
 * Modifications made by Emulex Corporation under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 *****************************************************************************/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
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

#define SYSWCR				0x80
#define SYSWRERL			0x84
#define SYSWRERH			0x88
#define SYSWCFR				0x8C

#define SEC             	381
#define WDT_EN				0x800000
#define PRE_TRIG_EN			0x400000
#define TRIG_WDT			0x1
#define RST_PULSE_WIDTH		0xC		/*using default value 0xC*(40.96us)=491.52us*/
#define WDT_MAX_CYCLE_COUNT	0xffff
#define WDT_IN_USE			0
#define WDT_OK_TO_CLOSE		1

struct pilot_wdt_data {
	u32 iobase;
	u32 irq;
};
struct pilot_wdt_data * pilot_wdt;

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static int heartbeat = -1;		/* module parameter (seconds) */
module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Initial watchdog heartbeat in seconds");

static int pretimeout = 20;		/* module parameter (seconds) */
module_param(pretimeout, int, 0);
MODULE_PARM_DESC(pretimeout, "Initial watchdog pretimeout in seconds");

static unsigned int wdt_max_duration;	/* (seconds) */
static unsigned int wdt_tclk;
static void __iomem *wdt_reg;
static unsigned long wdt_status;
static DEFINE_SPINLOCK(wdt_lock);

static void pilot_wdt_ping(void)
{
	volatile u32 reg;
	volatile u32 wdt_cnt;
	printk(KERN_DEBUG "Entered %s \n", __FUNCTION__);

	spin_lock(&wdt_lock);

	/* Reload watchdog duration */
	wdt_cnt = ((wdt_tclk * pretimeout) << 16) | (wdt_tclk * heartbeat);
	writel(wdt_cnt, wdt_reg + SYSWCFR);

	/* Trigger reset on watchdog */
	reg = readl(wdt_reg + SYSWCR);
	reg |= WDT_EN;
	writel(reg, wdt_reg + SYSWCR);

	spin_unlock(&wdt_lock);
}

static void pilot_wdt_enable(void)
{
	volatile u32 reg;
	volatile u32 wdt_cnt;
	printk(KERN_DEBUG "Entered %s \n", __FUNCTION__);

	spin_lock(&wdt_lock);

	/* Set watchdog duration */
	wdt_cnt = ((wdt_tclk * pretimeout) << 16) | (wdt_tclk * heartbeat);
	writel(wdt_cnt, wdt_reg + SYSWCFR);

	/* Enable ARM reset on watchdog */
	writel(0x3, wdt_reg + SYSWRERL);
	writel(0x0, wdt_reg + SYSWRERH);

	/* Trigger reset on watchdog */
	reg = readl(wdt_reg + SYSWCR);
	reg |= WDT_EN;
	writel(reg, wdt_reg + SYSWCR);

	/* Enable watchdog timer */
	reg = readl(wdt_reg + SYSWCR);
	reg &=~(0xff<<8);
	reg |= PRE_TRIG_EN;
	reg |= TRIG_WDT;
	reg |= RST_PULSE_WIDTH << 8;

	writel(reg, wdt_reg + SYSWCR);

	spin_unlock(&wdt_lock);
}

static void pilot_wdt_disable(void)
{
	volatile u32 reg;
	printk(KERN_DEBUG "Entered %s \n", __FUNCTION__);

	spin_lock(&wdt_lock);

	/* Disable watchdog timer */
	reg = readl(wdt_reg + SYSWCR);
	reg &= ~(0x1);
	writel(reg, wdt_reg + SYSWCR);

	spin_unlock(&wdt_lock);
}

static int pilot_wdt_get_timeleft(int *time_left)
{
	volatile u32 reg=0;
	volatile u32 timeleft=0;
	printk(KERN_DEBUG "Entered %s \n", __FUNCTION__);

	spin_lock(&wdt_lock);

	reg = readl(wdt_reg + SYSWCFR);
	printk(KERN_DEBUG "wdt cnt=%x \n", reg);
	timeleft = (reg & 0xffff) / wdt_tclk;
	printk(KERN_DEBUG "timeleft=%d \n", timeleft);
	*time_left = timeleft;

	spin_unlock(&wdt_lock);

	return 0;
}

static int pilot_wdt_open(struct inode *inode, struct file *file)
{
	printk(KERN_DEBUG "Entered %s \n", __FUNCTION__);
	if (test_and_set_bit(WDT_IN_USE, &wdt_status))
		return -EBUSY;

	clear_bit(WDT_OK_TO_CLOSE, &wdt_status);
	pilot_wdt_enable();

	return nonseekable_open(inode, file);
}

static ssize_t pilot_wdt_write(struct file *file, const char *data,
					size_t len, loff_t *ppos)
{
	if (len) {
		if (!nowayout) {
			size_t i;

			clear_bit(WDT_OK_TO_CLOSE, &wdt_status);
			for (i = 0; i != len; i++) {
				char c;

				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					set_bit(WDT_OK_TO_CLOSE, &wdt_status);
			}
		}
		pilot_wdt_ping();
	}
	return len;
}

static int pilot_wdt_settimeout(int new_time)
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

static const struct watchdog_info ident = {
	.options	= WDIOF_MAGICCLOSE | WDIOF_SETTIMEOUT |
			  WDIOF_KEEPALIVEPING,
	.identity	= "Pilot Watchdog",
};

static long pilot_wdt_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	int ret = -ENOTTY;
	int time;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		ret = copy_to_user((struct watchdog_info *)arg, &ident,
				   sizeof(ident)) ? -EFAULT : 0;
		break;

	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		ret = put_user(0, (int *)arg);
		break;

	case WDIOC_KEEPALIVE:
		pilot_wdt_ping();
		ret = 0;
		break;

	case WDIOC_SETTIMEOUT:
		ret = get_user(time, (int *)arg);
		if (ret)
			break;

		if (pilot_wdt_settimeout(time)) {
			ret = -EINVAL;
			break;
		}
		pilot_wdt_ping();
		/* Fall through */

	case WDIOC_GETTIMEOUT:
		ret = put_user(heartbeat, (int *)arg);
		break;

	case WDIOC_GETTIMELEFT:
		if (pilot_wdt_get_timeleft(&time)) {
			ret = -EINVAL;
			break;
		}
		ret = put_user(time, (int *)arg);
		break;

	case WDIOC_SETPRETIMEOUT:
		ret = get_user(time, (int *)arg);
		if (ret)
			return -EFAULT;
		ret = pilot_wdt_set_pretimeout(time);
		if (ret)
			return ret;
		pilot_wdt_ping();
		break;

	case WDIOC_GETPRETIMEOUT:
		ret = put_user(pretimeout, (int *)arg);
		break;
	}
	return ret;
}

extern void smp_send_wfe(int);
extern void reinit_spi(void);

static inline irqreturn_t pilot_wdt_irq(int irq, void *dev_id)
{
	int status;	

	printk(KERN_DEBUG "Entered %s \n", __FUNCTION__);

	status = readl(wdt_reg + SYSWCR);
	if( (irq==pilot_wdt->irq) && ((status & (1<<18))==(1<<18)) )
	{
		pr_crit("Initiating system reboot in %d seconds\n", pretimeout);
		status |= 1<<18;
		writel(status, wdt_reg + SYSWCR);
	}

	return IRQ_HANDLED;
}

static int pilot_wdt_release(struct inode *inode, struct file *file)
{
	printk(KERN_DEBUG "Entered %s \n", __FUNCTION__);

	if (test_bit(WDT_OK_TO_CLOSE, &wdt_status))
		pilot_wdt_disable();
	else
		pr_crit("Device closed unexpectedly - timer will not stop\n");
	clear_bit(WDT_IN_USE, &wdt_status);
	clear_bit(WDT_OK_TO_CLOSE, &wdt_status);

	return 0;
}

static const struct file_operations pilot_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= pilot_wdt_write,
	.unlocked_ioctl	= pilot_wdt_ioctl,
	.open		= pilot_wdt_open,
	.release	= pilot_wdt_release,
};

static struct miscdevice pilot_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &pilot_wdt_fops,
};

static int pilot_wdt_probe(struct platform_device *pdev)
{
	struct pilot_wdt_data *pilot;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	int ret;
	int status;

	printk(KERN_DEBUG "Entered %s \n", __FUNCTION__);

	pilot = (struct pilot_wdt_data *)kmalloc(sizeof(struct pilot_wdt_data), GFP_KERNEL);

	wdt_tclk = SEC;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pilot->iobase = res->start;
	wdt_reg = ioremap(pilot->iobase, resource_size(res));

	pilot_wdt = pilot;

#if 0
    // Not clearing the status before registering for the interrupt. This will allow the
    // interrupt handler to fire in case pre-trigger happened just before registering
    // the interrupt.
	status = readl(wdt_reg + SYSWCR);
	status |= 1<<18;
	writel(status, wdt_reg + SYSWCR);
#endif

	pilot->irq = platform_get_irq(pdev, 0);
	ret = request_irq(pilot->irq, pilot_wdt_irq, IRQF_SHARED, np->name, pilot);
	if (ret)
	{
		printk(KERN_DEBUG "ERROR: Couldn't get int %d: %d\n", pilot->irq, ret);
		return -ENXIO;
	}

	if (pilot_wdt_miscdev.parent)
		return -EBUSY;
	pilot_wdt_miscdev.parent = &pdev->dev;

	wdt_max_duration = WDT_MAX_CYCLE_COUNT / wdt_tclk;

	of_property_read_u32(np, "pretimeout-sec", &pretimeout);
	if (pilot_wdt_set_pretimeout(pretimeout))
		pretimeout = wdt_max_duration;

	of_property_read_u32(np, "timeout-sec", &heartbeat);
	if (pilot_wdt_settimeout(heartbeat))
		heartbeat = wdt_max_duration;

	ret = misc_register(&pilot_wdt_miscdev);
	if (ret)
		return ret;

	pr_info("Initial timeout %d sec pretimeout %d sec%s\n",
		heartbeat, pretimeout, nowayout ? ", nowayout" : "");
	return 0;
}

static int pilot_wdt_remove(struct platform_device *pdev)
{
	if (test_bit(WDT_IN_USE, &wdt_status)) {
		pilot_wdt_disable();
		clear_bit(WDT_IN_USE, &wdt_status);
	}

	misc_deregister(&pilot_wdt_miscdev);

	free_irq(pilot_wdt->irq, pilot_wdt);
	iounmap(wdt_reg);
	kfree(pilot_wdt);

	return 0;
}

static void pilot_wdt_shutdown(struct platform_device *pdev)
{
	if (test_bit(WDT_IN_USE, &wdt_status))
		pilot_wdt_disable();
}

static const struct of_device_id pilot_wdt_of_match[] = {
	{ .compatible = "pilot,wdt-1" },
	{ }
};
MODULE_DEVICE_TABLE(of, pilot_wdt_of_match);

static struct platform_driver pilot_wdt_driver = {
	.probe		= pilot_wdt_probe,
	.remove		= pilot_wdt_remove,
	.shutdown	= pilot_wdt_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "pilot-wdt",
		.of_match_table = pilot_wdt_of_match,
	},
};

module_platform_driver(pilot_wdt_driver);

MODULE_AUTHOR("Ashok Reddy Soma<ashok.soma@aspeedtech.com>");
MODULE_DESCRIPTION("Pilot Watchdog");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_LICENSE("GPL");
