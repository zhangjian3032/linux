/*
 * CHASIS driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Billy Tsai<billy_tsai@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/poll.h>
#include <linux/sysfs.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <asm/uaccess.h>
/******************************************************************************/
/* ASPEED_INTRUSION_CTRL */
#define INTRUSION_STATUS_CLEAR  BIT(0)
#define INTRUSION_INT_ENABLE    BIT(1)
#define INTRUSION_STATUS        BIT(2)
#define BATTERY_POWER_GOOD      BIT(3)
#define CHASIS_RAW_STATUS       BIT(4)
#define CORE_POWER_STATUS_CLEAR BIT(8)
#define CORE_POWER_INT_ENABLE   BIT(9)
#define CORE_POWER_STATUS       BIT(10)
#define IO_POWER_STATUS_CLEAR   BIT(16)
#define IO_POWER_INT_ENABLE     BIT(17)
#define IO_POWER_STATUS         BIT(18)

typedef union
{
	volatile uint32_t value;
	struct
	{
		volatile uint32_t intrusion_status_clear:1; /*[0]*/
        volatile uint32_t intrusion_int_enable:1; /*[1]*/ 
        volatile uint32_t intrusion_status:1; /*[2]*/ 
        volatile uint32_t battery_power_good:1; /*[3]*/ 
        volatile uint32_t chasis_raw_status:1; /*[4]*/
	    volatile uint32_t : 3; /*[5-7]*/
	    volatile uint32_t io_power_status_clear:1; /*[8]*/ 
        volatile uint32_t io_power_int_enable:1; /*[9]*/ 
        volatile uint32_t core_power_status:1; /*[10]*/
	    volatile uint32_t : 5; /*[11-15]*/
	    volatile uint32_t core_power_status_clear:1; /*[16]*/  
        volatile uint32_t core_power_int_enable:1; /*[17]*/ 
        volatile uint32_t io_power_status:1; /*[18]*/
	    volatile uint32_t : 13; /*[19-31]*/
	} fields;
} chasis_ctrl_register_t;


struct aspeed_chasis {
	struct device *dev;
	void __iomem *base;
	int irq;
};

static void aspeed_chasis_status_check(struct aspeed_chasis *chasis)
{
    chasis_ctrl_register_t chasis_ctrl;
    chasis_ctrl.value = readl(chasis->base);
    if (chasis_ctrl.fields.intrusion_status) {
	    dev_info(chasis->dev, "CHASI# pin has been pulled low");
	    chasis_ctrl.fields.intrusion_status_clear = 1;
	    writel(chasis_ctrl.value, chasis->base);
        chasis_ctrl.fields.intrusion_status_clear = 0;
        writel(chasis_ctrl.value, chasis->base);
    }
    chasis_ctrl.value = readl(chasis->base);
    if (chasis_ctrl.fields.core_power_status) {
	    dev_info(chasis->dev, "Core power has been pulled low");
	    chasis_ctrl.fields.core_power_status_clear = 1;
	    writel(chasis_ctrl.value, chasis->base);
        chasis_ctrl.fields.core_power_status_clear = 0;
        writel(chasis_ctrl.value, chasis->base);
    }

    chasis_ctrl.value = readl(chasis->base);
    if (chasis_ctrl.fields.io_power_status) {
	    dev_info(chasis->dev, "IO power has been pulled low");
	    chasis_ctrl.fields.io_power_status_clear = 1;
	    writel(chasis_ctrl.value, chasis->base);
        chasis_ctrl.fields.io_power_status_clear = 0;
        writel(chasis_ctrl.value, chasis->base);
    }
}

static irqreturn_t aspeed_chasis_isr(int this_irq, void *dev_id)
{
	struct aspeed_chasis *chasis = dev_id;
	aspeed_chasis_status_check(chasis);
	return IRQ_HANDLED;
}

static const struct of_device_id aspeed_chasis_of_table[] = {
	{ .compatible = "aspeed,ast2600-chasis" },
	{}
};
MODULE_DEVICE_TABLE(of, aspeed_chasis_of_table);

static int aspeed_chasis_probe(struct platform_device *pdev)
{
	struct aspeed_chasis *chasis;
	chasis_ctrl_register_t chasis_ctrl;
    int ret;

	dev_info(&pdev->dev, "aspeed_chasis_probe\n");

    chasis = devm_kzalloc(&pdev->dev, sizeof(*chasis), GFP_KERNEL);
	if (!chasis)
		return -ENOMEM;

    chasis->dev = &pdev->dev;
    chasis->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(chasis->base))
		return PTR_ERR(chasis->base);
	
	chasis->irq = platform_get_irq(pdev, 0);
	if (chasis->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		return ret;
	}

	ret = devm_request_irq(&pdev->dev, chasis->irq, aspeed_chasis_isr,
			       0, dev_name(&pdev->dev), chasis);
	if (ret) {
		dev_err(&pdev->dev, "Chasis Unable to get IRQ");
		return ret;
	}
    
    aspeed_chasis_status_check(chasis);

	// enable interrupt
    chasis_ctrl.value = readl(chasis->base);
    chasis_ctrl.fields.io_power_int_enable = 1;
    chasis_ctrl.fields.intrusion_int_enable = 1;
    chasis_ctrl.fields.core_power_int_enable = 1;
    writel(chasis_ctrl.value, chasis->base);
    dev_info(&pdev->dev, "Chasis ctrl value 0x%08x", chasis_ctrl.value);

    dev_info(&pdev->dev, "chasis driver successfully loaded.\n");

    return 0;
}


static struct platform_driver aspeed_chasis_driver = {
	.probe		= aspeed_chasis_probe,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = aspeed_chasis_of_table,
	},
};

module_platform_driver(aspeed_chasis_driver);

MODULE_AUTHOR("Billy Tsai<billy_tsai@aspeedtech.com>");
MODULE_DESCRIPTION("AST CHASIS Driver");
MODULE_LICENSE("GPL");
