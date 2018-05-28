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
#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <crypto/authenc.h>
#include <crypto/des.h>
#include <crypto/md5.h>
#include <crypto/sha.h>
#include <crypto/scatterwalk.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/hash.h>
#include <crypto/md5.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/skcipher.h>

#include <linux/completion.h>
#include <linux/clk.h>
#include <linux/crypto.h>
#include <linux/cryptohash.h>
#include <linux/delay.h>
#include <linux/scatterlist.h>

#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/rtnetlink.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <crypto/kpp.h>
#include <crypto/dh.h>

#include "aspeed-crypto.h"

#define ASPEED_ECDH_DEBUG

#ifdef ASPEED_ECDH_DEBUG
//#define ECDH_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#define ECDH_DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define ECDH_DBG(fmt, args...)
#endif

int aspeed_ecdh_trigger(struct aspeed_crypto_dev *crypto_dev)
{
	ECDH_DBG("\n");

	return 0;
}


static int aspeed_ecdh_compute_value(struct kpp_request *req)
{
	struct crypto_kpp *tfm = crypto_kpp_reqtfm(req);
	struct aspeed_ecdh_ctx *ctx = kpp_tfm_ctx(tfm);
//	struct qat_crypto_instance *inst = ctx->inst;

	ECDH_DBG("\n");
	
}

static int aspeed_ecdh_set_secret(struct crypto_kpp *tfm, void *buf,
			     unsigned int len)
{
	struct aspeed_ecdh_ctx *ctx = kpp_tfm_ctx(tfm);
	struct dh params;
	int ret;

	if (crypto_dh_decode_key(buf, len, &params) < 0)
		return -EINVAL;
	ECDH_DBG("\n");

#if 0
//	ret = qat_dh_set_params(ctx, &params);
//	if (ret < 0)
//		return ret;

	ctx->xa = dma_zalloc_coherent(dev, ctx->p_size, &ctx->dma_xa,
				      GFP_KERNEL);
	if (!ctx->xa) {
		qat_dh_clear_ctx(dev, ctx);
		return -ENOMEM;
	}
	memcpy(ctx->xa + (ctx->p_size - params.key_size), params.key,
	       params.key_size);
#endif
	return 0;
}


static int aspeed_ecdh_max_size(struct crypto_kpp *tfm)
{
	struct aspeed_ecdh_ctx *ctx = kpp_tfm_ctx(tfm);
	ECDH_DBG("\n");

	//return ctx->p ? ctx->p_size : -EINVAL;
	return 32;
}

static int aspeed_ecdh_init_tfm(struct crypto_kpp *tfm)
{
	struct aspeed_ecdh_ctx *ctx = kpp_tfm_ctx(tfm);
	struct crypto_alg *alg = tfm->base.__crt_alg;
	struct aspeed_crypto_alg *crypto_alg;
	
	crypto_alg = container_of(alg, struct aspeed_crypto_alg, alg.crypto);
	ctx->crypto_dev = crypto_alg->crypto_dev;
	ECDH_DBG("\n");

	return 0;
}

static void aspeed_ecdh_exit_tfm(struct crypto_tfm *tfm)
{
	//disable clk ??
	ECDH_DBG("\n");
}

struct aspeed_crypto_alg aspeed_kpp_algs[] = {
	{
		.alg.kpp = {
			.set_secret = aspeed_ecdh_set_secret,
			.generate_public_key = aspeed_ecdh_compute_value,
			.compute_shared_secret = aspeed_ecdh_compute_value,
			.max_size = aspeed_ecdh_max_size,
			.init = aspeed_ecdh_init_tfm,
			.exit = aspeed_ecdh_exit_tfm,
			.base = {
				.cra_name = "ecdh",
				.cra_driver_name = "aspeed-ecdh",
				.cra_priority = 300,
				.cra_module = THIS_MODULE,
				.cra_ctxsize = sizeof(struct aspeed_ecdh_ctx),
			},
		},
	},
};

int aspeed_register_kpp_algs(struct aspeed_crypto_dev *crypto_dev)
{
	int i;
	int err = 0;
	for (i = 0; i < ARRAY_SIZE(aspeed_kpp_algs); i++) {
		aspeed_kpp_algs[i].crypto_dev = crypto_dev;
		err = crypto_register_kpp(&aspeed_kpp_algs[i].alg.kpp);
		if (err)
			return err;
	}
	return 0;
}
