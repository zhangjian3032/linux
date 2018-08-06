/*
 * Crypto driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/reset.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of.h>

#include "aspeed-crypto.h"

// #define ASPEED_CRYPTO_DEBUG

#ifdef ASPEED_CRYPTO_DEBUG
//#define CRYPTO_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#define CRYPTO_DBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define CRYPTO_DBUG(fmt, args...)
#endif

int aspeed_crypto_handle_queue(struct aspeed_crypto_dev *crypto_dev,
			       struct crypto_async_request *new_areq)
{
	struct crypto_async_request *areq, *backlog;
	unsigned long flags;
	int err, ret = 0;

	CRYPTO_DBUG("\n");
	spin_lock_irqsave(&crypto_dev->lock, flags);
	if (new_areq)
		ret = crypto_enqueue_request(&crypto_dev->queue, new_areq);
	if (crypto_dev->flags & CRYPTO_FLAGS_BUSY) {
		spin_unlock_irqrestore(&crypto_dev->lock, flags);
		return ret;
	}
	backlog = crypto_get_backlog(&crypto_dev->queue);
	areq = crypto_dequeue_request(&crypto_dev->queue);
	if (areq)
		crypto_dev->flags |= CRYPTO_FLAGS_BUSY;
	spin_unlock_irqrestore(&crypto_dev->lock, flags);

	if (!areq)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	crypto_dev->is_async = (areq != new_areq);

	if (crypto_tfm_alg_type(areq->tfm) == CRYPTO_ALG_TYPE_ABLKCIPHER) {
		CRYPTO_DBUG("ablkcipher_request_cast \n");
		crypto_dev->ablk_req = ablkcipher_request_cast(areq);
		err = aspeed_crypto_ablkcipher_trigger(crypto_dev);
	} else if (crypto_tfm_alg_type(areq->tfm) == CRYPTO_ALG_TYPE_AKCIPHER) {
		CRYPTO_DBUG("akcipher_request_cast \n");
		crypto_dev->akcipher_req = container_of(areq, struct akcipher_request, base);
		err = aspeed_crypto_rsa_trigger(crypto_dev);
	} else {
		CRYPTO_DBUG("ahash_request_cast \n");
		crypto_dev->ahash_req = ahash_request_cast(areq);
		err = aspeed_crypto_ahash_trigger(crypto_dev);
		// crypto_dev->ahash_req->base.complete(&crypto_dev->ahash_req->base, err);
	}


	return (crypto_dev->is_async) ? ret : err;
}

static irqreturn_t aspeed_crypto_irq(int irq, void *dev)
{
	struct aspeed_crypto_dev *crypto_dev = (struct aspeed_crypto_dev *)dev;
	u32 sts = aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS);
	int handle = IRQ_NONE;

	CRYPTO_DBUG("aspeed_crypto_irq sts %x \n", sts);
	aspeed_crypto_write(crypto_dev, sts, ASPEED_HACE_STS);

	if (sts & HACE_CRYPTO_ISR) {
		if (crypto_dev->flags & CRYPTO_FLAGS_BUSY)
			tasklet_schedule(&crypto_dev->done_task);
		else
			dev_warn(crypto_dev->dev, "CRYPTO interrupt when no active requests.\n");
		handle = IRQ_HANDLED;
	}
	if (sts & HACE_RSA_ISR) {
		aspeed_crypto_write(crypto_dev, 0, ASPEED_HACE_RSA_CMD);
		if (crypto_dev->flags & CRYPTO_FLAGS_BUSY)
			tasklet_schedule(&crypto_dev->done_task);
		else
			dev_warn(crypto_dev->dev, "CRYPTO interrupt when no active requests.\n");
		handle = IRQ_HANDLED;
	}
	if (sts & HACE_HASH_ISR) {
		if (crypto_dev->flags & CRYPTO_FLAGS_BUSY)
			tasklet_schedule(&crypto_dev->done_task);
		else
			dev_warn(crypto_dev->dev, "CRYPTO interrupt when no active requests.\n");
		handle = IRQ_HANDLED;
	}
	return handle;
}

static void aspeed_crypto_done_task(unsigned long data)
{
	struct aspeed_crypto_dev *crypto_dev = (struct aspeed_crypto_dev *)data;

	crypto_dev->is_async = true;
	(void)crypto_dev->resume(crypto_dev);
}

static void aspeed_crypto_queue_task(unsigned long data)
{
	struct aspeed_crypto_dev *crypto_dev = (struct aspeed_crypto_dev *)data;

	aspeed_crypto_handle_queue(crypto_dev, NULL);
}

static int aspeed_crypto_register(struct aspeed_crypto_dev *crypto_dev)
{
	aspeed_register_crypto_algs(crypto_dev);
	aspeed_register_ahash_algs(crypto_dev);
	aspeed_register_akcipher_algs(crypto_dev);

	return 0;
}

static void aspeed_crypto_unregister(void)
{
#if 0
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(aspeed_cipher_algs); i++) {
		if (aspeed_cipher_algs[i]->type == ALG_TYPE_CIPHER)
			crypto_unregister_alg(&aspeed_cipher_algs[i]->alg.crypto);
		else
			crypto_unregister_ahash(&aspeed_cipher_algs[i]->alg.hash);
	}
#endif
}

static int aspeed_crypto_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int err;
	struct aspeed_crypto_dev *crypto_dev;

	crypto_dev = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_crypto_dev), GFP_KERNEL);
	if (!crypto_dev) {
		dev_err(dev, "unable to alloc data struct.\n");
		return -ENOMEM;
	}

	crypto_dev->dev = dev;

	platform_set_drvdata(pdev, crypto_dev);

	spin_lock_init(&crypto_dev->lock);
	tasklet_init(&crypto_dev->done_task, aspeed_crypto_done_task,
		     (unsigned long)crypto_dev);
	tasklet_init(&crypto_dev->queue_task, aspeed_crypto_queue_task,
		     (unsigned long)crypto_dev);
	crypto_init_queue(&crypto_dev->queue, 50);

	crypto_dev->regs = of_iomap(pdev->dev.of_node, 0);
	if (!(crypto_dev->regs)) {
		dev_err(dev, "can't ioremap\n");
		return -ENOMEM;
	}

	crypto_dev->rsa_buff = of_iomap(pdev->dev.of_node, 1);
	if (!(crypto_dev->rsa_buff)) {
		dev_err(dev, "can't rsa ioremap\n");
		return -ENOMEM;
	}

	crypto_dev->irq = platform_get_irq(pdev, 0);
	if (!crypto_dev->irq) {
		dev_err(&pdev->dev, "no memory/irq resource for crypto_dev\n");
		return -ENXIO;
	}

	crypto_dev->yclk = devm_clk_get(&pdev->dev, "yclk");
	if (IS_ERR(crypto_dev->yclk)) {
		dev_err(&pdev->dev, "no yclk clock defined\n");
		return -ENODEV;
	}

	clk_prepare_enable(crypto_dev->yclk);

	crypto_dev->rsaclk = devm_clk_get(&pdev->dev, "rsaclk");
	if (IS_ERR(crypto_dev->rsaclk)) {
		dev_err(&pdev->dev, "no rsaclk clock defined\n");
		return -ENODEV;
	}

	clk_prepare_enable(crypto_dev->rsaclk);

	if (devm_request_irq(&pdev->dev, crypto_dev->irq, aspeed_crypto_irq, 0, dev_name(&pdev->dev), crypto_dev)) {
		dev_err(dev, "unable to request aes irq.\n");
		return -EBUSY;
	}

	if (of_device_is_compatible(pdev->dev.of_node,
				    "aspeed,ast2600-crypto")) {
		crypto_dev->version = ASPEED_CRYPTO_G6;
		crypto_dev->rsa_max_buf_len = ASPEED_CRYPTO_G6_RSA_BUFF_SIZE;
	} else {
		crypto_dev->version = 0;
		crypto_dev->rsa_max_buf_len = ASPEED_CRYPTO_RSA_BUFF_SIZE;
	}

	// 8-byte aligned
	crypto_dev->cipher_addr = dma_alloc_coherent(&pdev->dev,
				  0xa000,
				  &crypto_dev->cipher_dma_addr, GFP_KERNEL);

	if (! crypto_dev->cipher_addr) {
		printk("error buff allocation\n");
		return -ENOMEM;
	}

	err = aspeed_crypto_register(crypto_dev);
	if (err) {
		dev_err(dev, "err in register alg");
		return err;
	}

	printk("ASPEED Crypto Accelerator successfully registered \n");

	return 0;
}

static int aspeed_crypto_remove(struct platform_device *pdev)
{
	struct aspeed_crypto_dev *crypto_dev = platform_get_drvdata(pdev);

	//aspeed_crypto_unregister();
	tasklet_kill(&crypto_dev->done_task);
	tasklet_kill(&crypto_dev->queue_task);
	return 0;
}

#ifdef CONFIG_PM
static int aspeed_crypto_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct aspeed_crypto_dev *crypto_dev = platform_get_drvdata(pdev);

	/*
	 * We only support standby mode. All we have to do is gate the clock to
	 * the spacc. The hardware will preserve state until we turn it back
	 * on again.
	 */
//	clk_disable(crypto_dev->clk);

	return 0;
}

static int aspeed_crypto_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct aspeed_crypto_dev *crypto_dev = platform_get_drvdata(pdev);
	return 0;
//	return clk_enable(crypto_dev->clk);
}

#endif /* CONFIG_PM */



static const struct of_device_id aspeed_crypto_of_matches[] = {
	{ .compatible = "aspeed,ast2600-crypto", },
	{ .compatible = "aspeed,ast2500-crypto", },
	{ .compatible = "aspeed,ast2400-crypto", },
	{},
};

MODULE_DEVICE_TABLE(of, aspeed_crypto_of_matches);

static struct platform_driver aspeed_crypto_driver = {
	.probe 		= aspeed_crypto_probe,
	.remove		= aspeed_crypto_remove,
#ifdef CONFIG_PM
	.suspend	= aspeed_crypto_suspend,
	.resume 	= aspeed_crypto_resume,
#endif
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = aspeed_crypto_of_matches,
	},
};

module_platform_driver(aspeed_crypto_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED Crypto driver");
MODULE_LICENSE("GPL2");
