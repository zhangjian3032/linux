/*
 * Copyright (c) 2020 Johnny Huang <johnny_huang@aspeedtech.com>
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/hw_random.h>
#include <linux/platform_device.h>

#define TRNG_CTL	0x00
#define   TRNG_EN	0x0
#define   TRNG_MODE	0x04
#define   TRNG_RDY	0x1f
#define TRNG_ODATA	0x04


struct aspeed_trng {
	void __iomem *base;
	struct hwrng rng;
};

static int aspeed_trng_read(struct hwrng *rng, void *buf, size_t max,
			    bool wait)
{
	struct aspeed_trng *trng = container_of(rng, struct aspeed_trng, rng);
	u32 *data = buf;
	size_t read = 0;
	int timeout = max / 4 + 1;

	while (read < max) {
		if (!(readl(trng->base + TRNG_CTL) & (1 << TRNG_RDY))) {
			if (wait) {
				if (timeout-- == 0)
					return read;
			} else {
				return 0;
			}
		} else {
			*data = readl(trng->base + TRNG_ODATA);
			data++;
			read += 4;
		}
	}

	return read;
}

static void aspeed_trng_enable(struct aspeed_trng *trng)
{
	u32 ctl;

	ctl = readl(trng->base + TRNG_CTL);
	ctl = ctl & ~(1 << TRNG_EN); /* enable rng */
	ctl = ctl | (3 << TRNG_MODE); /* select mode */
	writel(ctl, trng->base + TRNG_CTL);
}

static void aspeed_trng_disable(struct aspeed_trng *trng)
{
	writel(1, trng->base + TRNG_CTL);
}

static int aspeed_trng_probe(struct platform_device *pdev)
{
	struct aspeed_trng *trng;
	struct resource *res;
	int ret;

	trng = devm_kzalloc(&pdev->dev, sizeof(*trng), GFP_KERNEL);
	if (!trng)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	trng->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(trng->base))
		return PTR_ERR(trng->base);

	aspeed_trng_enable(trng);
	trng->rng.name = pdev->name;
	trng->rng.read = aspeed_trng_read;
	trng->rng.quality = 900;

	ret = devm_hwrng_register(&pdev->dev, &trng->rng);
	if (ret)
		goto err_register;

	platform_set_drvdata(pdev, trng);

	return 0;

err_register:
	return ret;
}

static int aspeed_trng_remove(struct platform_device *pdev)
{
	struct aspeed_trng *trng = platform_get_drvdata(pdev);

	aspeed_trng_disable(trng);

	return 0;
}

static const struct of_device_id aspeed_trng_dt_ids[] = {
	{ .compatible = "aspeed,ast2600-trng" },
	{}
};
MODULE_DEVICE_TABLE(of, aspeed_trng_dt_ids);

static struct platform_driver aspeed_trng_driver = {
	.probe		= aspeed_trng_probe,
	.remove		= aspeed_trng_remove,
	.driver		= {
		.name	= "aspeed-trng",
		.of_match_table = aspeed_trng_dt_ids,
	},
};

module_platform_driver(aspeed_trng_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Johnny Huang <johnny_huang@aspeedtech.com>");
MODULE_DESCRIPTION("Aspeed true random number generator driver");
