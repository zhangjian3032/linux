// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018 Aspeed Technology Inc
 * Shivah Shankar S <shivahs@aspeedtech.com>
 *
 */

#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>

/* Provides a simple driver to control the Pilot LPC snoop interface which
 * allows the BMC to listen on and save the data written by
 * the host to an arbitrary LPC I/O port.
 *
 * Typically used by the BMC to "watch" host boot progress via port
 * 0x80 writes made by the BIOS during the boot process.
 */
#if 0
#define DEVICE_NAME	"pilot-lpc-snoop"
#endif
#define DEVICE_NAME	"aspeed-lpc-snoop"
#define NUM_SNOOP_CHANNELS 1
#define SNOOP_FIFO_SIZE 2048

#define PORT_CAPTURE_DISABLE (1<<0)
#define FIFO_OVERLFW_INTR_ENBLE (1<<5)
#define FIFO_UPDATE_INTR_ENBLE	(1<<2)

#define PCDR	0x1540
#define PCARL	0x1541
#define PCSR	0x1542
#define PCMR	0x1543
#define PCCR	0x1544
#define PCCT	0x1545
#define PCARM	0x1546

struct pilot_lpc_snoop {
	struct regmap		*regmap;
	void __iomem		*base;
	int			irq;
	int			swc_irq;
	struct kfifo		snoop_fifo[NUM_SNOOP_CHANNELS];
};
static const struct of_device_id pilot_lpc_snoop_match[] = {
	{ .compatible = "aspeed,pilot-lpc-snoop"},
	{ }
};
static void pilot_regmap_read8(struct regmap * map, 
					u32 offset, u8 * val)
{
	u32 temp_val;
#if 0
	u8 b_pos = offset & 3;
	regmap_read(map, (offset&~(3)), &temp_val);
	printk("Read val is %x\n", temp_val);
	*val = ((temp_val) >> (b_pos << 3)) & 0xFF;
#endif
	regmap_read(map, offset,  &temp_val);
	*val = (temp_val & 0xFF);
}
static void pilot_regmap_write8(struct regmap * map,
					u32 offset, u8 mask, u8 val)
{
	u32 temp_val;
	bool change;
#if 0
	u32 msk=0;
	u8 b_pos = offset & 3;
	regmap_read(map, offset&~(3), &temp_val);
	temp_val |= (((val) << (b_pos << 3)));
	printk("The value written is %x to be written is %x\n", temp_val, val);
	temp_val =0;
	temp_val |= (((val) << (b_pos << 3)));
	msk |= (((mask) << (b_pos << 3)));
	printk("mask is %x tval is %x val is %x\n", msk, temp_val, val);
	regmap_update_bits_base(map, (offset&~(3)), msk, temp_val,
				&change, false, true);
	/*regmap_update_bits(map, (offset&~(3)), temp_val, temp_val);*/
#endif
	temp_val = val;
	regmap_update_bits_base(map, offset, mask, temp_val,
				&change, false, true);
}
/* Save a byte to a FIFO and discard the oldest byte if FIFO is full */
static void put_fifo_with_discard(struct kfifo *fifo, u8 val)
{
	if (!kfifo_initialized(fifo))
		return;
	if (kfifo_is_full(fifo))
		kfifo_skip(fifo);
	kfifo_put(fifo, val);
}

static void pilot_lpc_enable_snoop(struct pilot_lpc_snoop *lpc_snoop, int enable)
{
	bool change;
	if(enable){
		regmap_update_bits_base(lpc_snoop->regmap, PCCT, 
					PORT_CAPTURE_DISABLE, (0),
					&change, false, true);
		regmap_update_bits_base(lpc_snoop->regmap, PCCT, 
				(FIFO_OVERLFW_INTR_ENBLE |
                                 FIFO_UPDATE_INTR_ENBLE),
				(FIFO_OVERLFW_INTR_ENBLE |
                                 FIFO_UPDATE_INTR_ENBLE),
					&change, false, true);
#if 0
		pilot_regmap_write8(lpc_snoop->regmap, PCCT, PORT_CAPTURE_DISABLE, 
						0);
		pilot_regmap_write8(lpc_snoop->regmap, PCCT, 
				(FIFO_OVERLFW_INTR_ENBLE | 
				 FIFO_UPDATE_INTR_ENBLE), 
				(FIFO_OVERLFW_INTR_ENBLE |
                                 FIFO_UPDATE_INTR_ENBLE));
#endif
	} else {
#if 0
		pilot_regmap_write8(lpc_snoop->regmap, PCCT, 
#endif
		regmap_update_bits_base(lpc_snoop->regmap, PCCT, 
					PORT_CAPTURE_DISABLE, 
					PORT_CAPTURE_DISABLE,
					&change, false, true);
#if 0
		pilot_regmap_write8(lpc_snoop->regmap, PCCT, 
#endif
		regmap_update_bits_base(lpc_snoop->regmap, PCCT, 
				(FIFO_OVERLFW_INTR_ENBLE | 
				 FIFO_UPDATE_INTR_ENBLE), 0,
					&change, false, true);
	}
}

static irqreturn_t pilot_lpc_snoop_irq(int irq, void *arg)
{
	struct pilot_lpc_snoop *lpc_snoop = arg;
	u8 reg, data;
	u8 swc_val;
	bool change;

	printk("Got LPC reset interrupt in snoop\n");
	pilot_regmap_read8(lpc_snoop->regmap, 0x1523, &swc_val);
	if((swc_val & 0x1) != 0x1){
                printk("LPC reset active in snoop\n");
                regmap_update_bits_base(lpc_snoop->regmap, 0x1523,
                                (0xf7), ((swc_val)),
                                &change, false, true);
                return IRQ_HANDLED;
        }
	pilot_lpc_enable_snoop(lpc_snoop, 1);
        regmap_update_bits_base(lpc_snoop->regmap, 0x1523, (0xf7), ((swc_val)),
                                		&change, false, true);

	//Read the FIFO Not empty bit to check if the data in  the
	//FIFO is valid
	pilot_regmap_read8(lpc_snoop->regmap, PCSR, &reg);
	while(reg & 0x1){
		pilot_regmap_read8(lpc_snoop->regmap, PCDR, &data);
		printk("Snoop data is %x\n", data);
		put_fifo_with_discard(&lpc_snoop->snoop_fifo[0], data);
		pilot_regmap_read8(lpc_snoop->regmap, PCSR, &reg);
	}
	return IRQ_HANDLED;
}
static irqreturn_t pilot_swc_irq(int irq, void *arg)
{
	struct pilot_lpc_snoop *lpc_snoop = arg;
	u8 swc_val;
	bool change;

	pilot_regmap_read8(lpc_snoop->regmap, 0xa, &swc_val);
	//Acknowledge the SWC interrupt
	regmap_update_bits_base(lpc_snoop->regmap, 0xa, (1<<2), (1<<2),
				&change, false, true);
#if 0
	regmap_update_bits_base(lpc_snoop->regmap, 0x8, (1<<18), (1<<18),
				&change, false, true);
#endif
	if(swc_val & 0x8){
		pilot_lpc_enable_snoop(lpc_snoop, 1);
	}else{
		pilot_lpc_enable_snoop(lpc_snoop, 0);
	}

	return IRQ_HANDLED;
}
static int pilot_lpc_snoop_config_irq(struct pilot_lpc_snoop *lpc_snoop,
				       struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc;
	bool change;

	lpc_snoop->irq = platform_get_irq(pdev, 0);
	if (!lpc_snoop->irq)
		return -ENODEV;

	rc = devm_request_irq(dev, lpc_snoop->irq,
			      pilot_lpc_snoop_irq, IRQF_SHARED,
			      DEVICE_NAME, lpc_snoop);
	if (rc < 0) {
		dev_warn(dev, "Unable to request IRQ %d\n", lpc_snoop->irq);
		lpc_snoop->irq = 0;
		return rc;
	}
	lpc_snoop->swc_irq = platform_get_irq(pdev, 1);
	if (!lpc_snoop->swc_irq)
		return -ENODEV;
	regmap_update_bits_base(lpc_snoop->regmap, + 0x4,
				0x1, 0x1, &change, false, true);

	rc = devm_request_irq(dev, lpc_snoop->swc_irq,
			      pilot_swc_irq, IRQF_SHARED,
			      DEVICE_NAME, lpc_snoop);
	if (rc < 0) {
		dev_warn(dev, "Unable to request IRQ %d\n", lpc_snoop->swc_irq);
		lpc_snoop->swc_irq = 0;
		return rc;
	}

	return 0;
}

static struct regmap_config pilot_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.reg_stride = 1,
};
static int pilot_lpc_snoop_probe(struct platform_device *pdev)
{
	struct pilot_lpc_snoop *lpc_snoop;
	struct device *dev;
	u32 port;
	int rc;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *node;

	dev = &pdev->dev;

	lpc_snoop = devm_kzalloc(dev, sizeof(*lpc_snoop), GFP_KERNEL);
	if (!lpc_snoop)
		return -ENOMEM;
#if 0
	lpc_snoop->regmap = syscon_node_to_regmap(
			pdev->dev.parent->of_node);
	lpc_snoop->regmap = syscon_regmap_lookup_by_phandle(np, "syscon-lpc");
	if (IS_ERR(lpc_snoop->regmap)) {
		dev_err(dev, "Couldn't get regmap\n");
		return -ENODEV;
	}
#endif
	node = of_find_compatible_node(NULL, NULL, "aspeed,pilot-syscon");
	
	lpc_snoop->base = of_iomap(node, 0);

	lpc_snoop->regmap = regmap_init_mmio(NULL, lpc_snoop->base, &pilot_config);

	dev_set_drvdata(&pdev->dev, lpc_snoop);

	rc = pilot_lpc_snoop_config_irq(lpc_snoop, pdev);
	if (rc)
		return rc;

	pilot_lpc_enable_snoop(lpc_snoop, 1);
	return rc;
}

static int pilot_lpc_snoop_remove(struct platform_device *pdev)
{
	struct pilot_lpc_snoop *lpc_snoop = dev_get_drvdata(&pdev->dev);

	pilot_lpc_enable_snoop(lpc_snoop, 0);
	return 0;
}

static struct platform_driver pilot_lpc_snoop_driver = {
	.driver = {
		.name		= DEVICE_NAME,
		.of_match_table = pilot_lpc_snoop_match,
	},
	.probe = pilot_lpc_snoop_probe,
	.remove = pilot_lpc_snoop_remove,
};

module_platform_driver(pilot_lpc_snoop_driver);

MODULE_DEVICE_TABLE(of, pilot_lpc_snoop_match);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Shivah Shankar S <shivahs@aspeedtech.com>");
MODULE_DESCRIPTION("Linux driver to control Pilot's LPC snoop functionality");
