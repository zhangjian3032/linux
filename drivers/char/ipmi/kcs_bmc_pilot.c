// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018, Aspeed Technology Inc.
 * Shivah Shankar S <shivahs@aspeedtech.com>
 */

#define pr_fmt(fmt) "pilot-kcs-bmc: " fmt

#include <linux/atomic.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>

#include "kcs_bmc.h"

#define LPC_IDR1 0x1200
#define LPC_IDR2 0x1300
#define LPC_IDR3 0x1400

#define LPC_ODR1 LPC_IDR1
#define LPC_ODR2 LPC_IDR2
#define LPC_ODR3 LPC_IDR3

#define LPC_STR1 0x1203
#define LPC_STR2 0x1303
#define LPC_STR3 0x1403

#define KCSIER	0x1439
#define KCSISR	0x1438

#define DEVICE_NAME     "pilot-kcs-bmc"

#define KCS_CHANNEL_MAX     3

struct pilot_kcs_bmc {
	struct regmap *map;
	void __iomem *base;
	int irq;
	int swc_irq;
	int lpcrst_irq;
};
static void pilot_regmap_read8(struct regmap * map, 
					u32 offset, u8 * val)
{
	u32 temp_val;
	
	regmap_read(map, offset, &temp_val);
	*val = temp_val & 0xFF;
	//printk("Read val at offset %x is %x\n", offset, *val);
}
static void pilot_regmap_write8(struct regmap * map,
					u32 offset, u8 mask, u8 val)
{
	u32 temp_val;
	temp_val = val;
	regmap_write(map, offset, temp_val);
}
static u8 pilot_kcs_inb(struct kcs_bmc *kcs_bmc, u32 reg)
{
	struct pilot_kcs_bmc *priv = kcs_bmc_priv(kcs_bmc);
	u8 val = 0;
	int rc;
	bool change;

	/* Pilot has a different CMD register that contains the command
	 * since the framework expects command to be in Data register
	 * So when the framework tries to read the data register with 
	 * CMD bit set in status register we will actually read the CMD 
	 * register and provide the data
	 */ 
	if((reg & 0x3 ) == 0x0){
		pilot_regmap_read8(priv->map, (reg | 0x3), &val);
		//Check if the command bit is set
		if(( val & 0x8 ) == (0x8)){
			pilot_regmap_read8(priv->map, (reg | 0x2), &val);
			regmap_update_bits_base(priv->map, reg|0x3, 0x2, 0x2, 
							&change, false, true);
			//printk("CMD reg Value read is %x\n", val);
			return val;
		}else{
			/* Note - Pilot expects a W1C to the IBF to clear it
			 * this cannot be cleared until we read the data
			 * that was written by host, so we ensure to clear it
			 * after the data is read
			 */
			pilot_regmap_read8(priv->map, reg, &val);
			regmap_update_bits_base(priv->map, reg|0x3, 0x2, 0x2, 
							&change, false, true);
			return val;
		}
	}
	pilot_regmap_read8(priv->map, reg, &val);
	return val;
}

static void pilot_kcs_outb(struct kcs_bmc *kcs_bmc, u32 reg, u8 data)
{
	struct pilot_kcs_bmc *priv = kcs_bmc_priv(kcs_bmc);
	int rc;
	u8 val;
	bool change;

	pilot_regmap_write8(priv->map, reg, 0xFF, data);
}
/*
 * Pilot4 Specification (section 6.3.4.1)
 * KCS1	-	Is actually named KCS3
 * 		Default Address - 0xC62
 * KCS2	- 	Is actually named KCS4
 * 		Default Address - 0xCA2
 * KCS3	-	Is actually named KCS5
 * 		Default Address - 0xC68
 */
static void pilot_kcs_set_address(struct kcs_bmc *kcs_bmc, u16 addr)
{
	/* NOTE: You can set the addres only from Host side*/
}
#define PILOT_KCS_ENABLE_CHAN1 (1<<2)
#define PILOT_KCS_ENABLE_CHAN2 (1<<3)
#define PILOT_KCS_ENABLE_CHAN3 (1<<4)
static void pilot_kcs_enable_channel(struct kcs_bmc *kcs_bmc, u8 enable)
{
	struct pilot_kcs_bmc *priv = kcs_bmc_priv(kcs_bmc);
	u8 val = 0;
        int rc;
	bool change;
//TODo - Enabled the channel here
	switch (kcs_bmc->channel) {
	case 1:
		printk("Enabling KCS channel %d\n", kcs_bmc->channel);
		regmap_update_bits_base(priv->map, KCSIER, PILOT_KCS_ENABLE_CHAN1,
			enable ? PILOT_KCS_ENABLE_CHAN1: 0, &change, false, true);
		/*
		pilot_regmap_write8(priv->map, KCSIER, PILOT_KCS_ENABLE_CHAN1,
					enable ? PILOT_KCS_ENABLE_CHAN1: 0);
		*/
		pilot_regmap_read8(priv->map, KCSIER, &val);
		printk("Value of KCSIER after enable is %x and %x\n", val, rc);
		break;
	case 2:
		printk("Enabling KCS channel %d\n", kcs_bmc->channel);
		regmap_update_bits_base(priv->map, KCSIER, PILOT_KCS_ENABLE_CHAN2,
			enable ? PILOT_KCS_ENABLE_CHAN2: 0, &change, false, true);
/*
		pilot_regmap_write8(priv->map, KCSIER, PILOT_KCS_ENABLE_CHAN2,
				enable ? PILOT_KCS_ENABLE_CHAN2: 0);
*/
		pilot_regmap_read8(priv->map, KCSIER, &val);
		printk("Value of KCSIER after enable is %x and %x\n", val, rc);
		break;
	case 3:
		printk("Enabling KCS channel %d\n", kcs_bmc->channel);
		regmap_update_bits_base(priv->map, KCSIER, PILOT_KCS_ENABLE_CHAN3,
			enable ? PILOT_KCS_ENABLE_CHAN3: 0, &change, false, true);
/*
		pilot_regmap_write8(priv->map, KCSIER, PILOT_KCS_ENABLE_CHAN3,
				enable ? PILOT_KCS_ENABLE_CHAN3: 0);
*/
		pilot_regmap_read8(priv->map, KCSIER, &val);
		printk("Value of KCSIER after enable is %x and %x\n", val, rc);
		break;
	default:
		printk("Error Enabling KCS channel %d\n", kcs_bmc->channel);
		break;
	}
}
static irqreturn_t pilot_lpcrst_irq(int irq, void *arg)
{
	struct kcs_bmc *kcs_bmc = arg;
	struct pilot_kcs_bmc *priv = kcs_bmc_priv(kcs_bmc);
	u8 swc_val;
	bool change;
	pilot_regmap_read8(priv->map, 0x1523, &swc_val);
	if((swc_val & 0x1) != 0x1){
                printk("LPC reset NOT active in KCS\n");
                regmap_update_bits_base(priv->map, 0x1523,
                                (0xf7), ((swc_val)),
                                &change, false, true);
                return IRQ_HANDLED;
        }
        printk("LPC reset active in KCS\n");
	regmap_update_bits_base(priv->map, 0x1523, (0xf7), ((swc_val )),
				&change, false, true);
	pilot_kcs_enable_channel(kcs_bmc, 1);
	return IRQ_HANDLED;
}
/* NOTE: The LPC registers KCS, BT, Mailbox etc cannot be written 
 * unless the host is powered on when the host powers on the SWC
 * interrupt is fired, we will enable the KCS channel then
 */
static irqreturn_t pilot_swc_irq(int irq, void *arg)
{
	struct kcs_bmc *kcs_bmc = arg;
	struct pilot_kcs_bmc *priv = kcs_bmc_priv(kcs_bmc);
	u8 swc_val;
	bool change;

	pilot_regmap_read8(priv->map, 0xa, &swc_val);
	//Acknowledge the SWC interrupt
	regmap_update_bits_base(priv->map, 0xa, (1<<2), (1<<2),
				&change, false, true);
	if(swc_val & 0x8){
		//Enable KCS
		pilot_kcs_enable_channel(kcs_bmc, 1);
	}else{
		//Disable KCS
		pilot_kcs_enable_channel(kcs_bmc, 0);
	}
	return IRQ_HANDLED;
}
static irqreturn_t pilot_kcs_irq(int irq, void *arg)
{
	struct kcs_bmc *kcs_bmc = arg;

	if (!kcs_bmc_handle_event(kcs_bmc))
		return IRQ_HANDLED;

	return IRQ_NONE;
}

static int pilot_kcs_config_irq(struct kcs_bmc *kcs_bmc,
                        struct platform_device *pdev)
{
        struct device *dev = &pdev->dev;
	struct pilot_kcs_bmc *priv = kcs_bmc_priv(kcs_bmc);;
        int irq;
	int rc;
	u8 lpc_val;
	bool change;

	rc = request_irq(priv->irq, &pilot_kcs_irq, IRQF_SHARED, dev_name(dev), kcs_bmc);
	if(rc < 0)
		goto err1;

	/* Here enable the SWC interrupt*/
	/*pilot_regmap_write8(priv->map, 0x4, 0x1, 0x1);*/
	regmap_update_bits_base(priv->map, 0x4, 0x1, ((0x1)),
				&change, false, true);

	rc = request_irq(priv->swc_irq, &pilot_swc_irq, IRQF_SHARED, dev_name(dev), kcs_bmc);
	if(rc < 0)
		goto err2;

	pilot_regmap_read8(priv->map, 0x1523, &lpc_val);
	//Enable the lpcreset interrupt
	regmap_update_bits_base(priv->map, 0x1523, 0xf7, (lpc_val | (1<<2)),
				&change, false, true);
/*
	regmap_update_bits_base(priv->map, 0x1520, (0xf7000000), ((lpc_val | (1<<2)) << 23),
				&change, false, true);
*/
	rc = request_irq(priv->lpcrst_irq, &pilot_lpcrst_irq, IRQF_SHARED, dev_name(dev), kcs_bmc);
	if(rc < 0)
		goto err2;

	return rc;

err1:
	printk("Couldnt map KCS irq\n");
	
err2:
	printk("Couldnt map KCS irq\n");

	return rc;
}

static const struct kcs_ioreg pilot_kcs_bmc_ioregs[KCS_CHANNEL_MAX] = {
	{ .idr = LPC_IDR1, .odr = LPC_ODR1, .str = LPC_STR1 },
	{ .idr = LPC_IDR2, .odr = LPC_ODR2, .str = LPC_STR2 },
	{ .idr = LPC_IDR3, .odr = LPC_ODR3, .str = LPC_STR3 },
};
static struct regmap_config pilot_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.reg_stride = 1,
};
static int pilot_kcs_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pilot_kcs_bmc *priv;
	struct kcs_bmc *kcs_bmc;
	u32 chan, addr;
	int rc;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *node;
	

	rc = of_property_read_u32(dev->of_node, "kcs_chan", &chan);
	if ((rc != 0) || (chan == 0 || chan > KCS_CHANNEL_MAX)) {
		dev_err(dev, "no valid 'kcs_chan' configured\n");
		return -ENODEV;
	}
	
	kcs_bmc = kcs_bmc_alloc(dev, sizeof(*priv), chan);
	if (!kcs_bmc)
		return -ENOMEM;

	priv = kcs_bmc_priv(kcs_bmc);
	/*
	priv->map = syscon_node_to_regmap(dev->parent->of_node);
	priv->map = syscon_regmap_lookup_by_phandle(np, "syscon-lpc");
	if (IS_ERR(priv->map)) {
		dev_err(dev, "Couldn't get regmap\n");
		return -ENODEV;
	}
	*/
	node = of_find_compatible_node(NULL, NULL, "aspeed,pilot-syscon");
	
	priv->base = of_iomap(node, 0);

	priv->map = regmap_init_mmio(NULL, priv->base, &pilot_config);

	kcs_bmc->ioreg = pilot_kcs_bmc_ioregs[chan - 1];
	kcs_bmc->io_inputb = pilot_kcs_inb;
	kcs_bmc->io_outputb = pilot_kcs_outb;

	dev_set_drvdata(dev, kcs_bmc);

	pilot_kcs_enable_channel(kcs_bmc, 1);
	priv->irq = irq_of_parse_and_map(np, 0);
	priv->swc_irq = irq_of_parse_and_map(np, 1);
	priv->lpcrst_irq = irq_of_parse_and_map(np, 2);
	rc = pilot_kcs_config_irq(kcs_bmc, pdev);
	if (rc)
		return rc;

	rc = misc_register(&kcs_bmc->miscdev);
	if (rc) {
		dev_err(dev, "Unable to register device\n");
		return rc;
	}

	pr_info("channel=%u addr=0x%x idr=0x%x odr=0x%x str=0x%x\n",
		chan, addr,
		kcs_bmc->ioreg.idr, kcs_bmc->ioreg.odr, kcs_bmc->ioreg.str);

	return 0;
}

static int pilot_kcs_remove(struct platform_device *pdev)
{
	struct kcs_bmc *kcs_bmc = dev_get_drvdata(&pdev->dev);

	misc_deregister(&kcs_bmc->miscdev);

	return 0;
}

static const struct of_device_id pilot_kcs_bmc_match[] = {
	{ .compatible = "aspeed,pilot-kcs-bmc" },
	{ }
};
MODULE_DEVICE_TABLE(of, pilot_kcs_bmc_match);

static struct platform_driver pilot_kcs_bmc_driver = {
	.driver = {
		.name           = DEVICE_NAME,
		.of_match_table = pilot_kcs_bmc_match,
	},
	.probe  = pilot_kcs_probe,
	.remove = pilot_kcs_remove,
};
module_platform_driver(pilot_kcs_bmc_driver);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Shivah Shankar S<shivahs@aspeedtech.com>");
MODULE_DESCRIPTION("Aspeed device interface to the Pilot's KCS BMC device");
