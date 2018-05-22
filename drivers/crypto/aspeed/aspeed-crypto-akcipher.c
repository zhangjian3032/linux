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
#include <crypto/akcipher.h>

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
#include "aspeed-crypto.h"

//#define ASPEED_RSA_DEBUG 

#ifdef ASPEED_RSA_DEBUG
//#define RSA_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#define RSA_DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define RSA_DBG(fmt, args...)
#endif

int aspeed_crypto_rsa_trigger(struct aspeed_crypto_dev *crypto_dev)
{

	return 0;
}

static int aspeed_rsa_enc(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
//	struct aspeed_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	RSA_DBG("\n");

	return 0;
}

static int aspeed_rsa_dec(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
//	struct aspeed_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	RSA_DBG("\n");

}

static int aspeed_rsa_setpubkey(struct crypto_akcipher *tfm, const void *key,
				unsigned int keylen)
{
//	struct aspeed_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
//	struct rsa_key raw_key = {0};
//	struct aspeed_rsa_key *rsa_key = &ctx->key;
//	int ret;
	RSA_DBG("\n");
#if 0
	/* Free the old RSA key if any */
	caam_rsa_free_key(rsa_key);

	ret = rsa_parse_pub_key(&raw_key, key, keylen);
	if (ret)
		return ret;

	/* Copy key in DMA zone */
	rsa_key->e = kzalloc(raw_key.e_sz, GFP_DMA | GFP_KERNEL);
	if (!rsa_key->e)
		goto err;

	/*
	 * Skip leading zeros and copy the positive integer to a buffer
	 * allocated in the GFP_DMA | GFP_KERNEL zone. The decryption descriptor
	 * expects a positive integer for the RSA modulus and uses its length as
	 * decryption output length.
	 */
	rsa_key->n = caam_read_raw_data(raw_key.n, &raw_key.n_sz);
	if (!rsa_key->n)
		goto err;

	if (caam_rsa_check_key_length(raw_key.n_sz << 3)) {
		caam_rsa_free_key(rsa_key);
		return -EINVAL;
	}

	rsa_key->e_sz = raw_key.e_sz;
	rsa_key->n_sz = raw_key.n_sz;

	memcpy(rsa_key->e, raw_key.e, raw_key.e_sz);

	return 0;
err:
	caam_rsa_free_key(rsa_key);
	return -ENOMEM;
#endif	
}

static int aspeed_rsa_setprivkey(struct crypto_akcipher *tfm, const void *key,
				 unsigned int keylen)
{
//	struct aspeed_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
//	struct rsa_key raw_key = {0};
//	struct aspeed_rsa_key *rsa_key = &ctx->key;
	int ret;

	RSA_DBG("\n");
#if 0
	/* Free the old RSA key if any */
	caam_rsa_free_key(rsa_key);

	ret = rsa_parse_priv_key(&raw_key, key, keylen);
	if (ret)
		return ret;

	/* Copy key in DMA zone */
	rsa_key->d = kzalloc(raw_key.d_sz, GFP_DMA | GFP_KERNEL);
	if (!rsa_key->d)
		goto err;

	rsa_key->e = kzalloc(raw_key.e_sz, GFP_DMA | GFP_KERNEL);
	if (!rsa_key->e)
		goto err;

	/*
	 * Skip leading zeros and copy the positive integer to a buffer
	 * allocated in the GFP_DMA | GFP_KERNEL zone. The decryption descriptor
	 * expects a positive integer for the RSA modulus and uses its length as
	 * decryption output length.
	 */
	rsa_key->n = caam_read_raw_data(raw_key.n, &raw_key.n_sz);
	if (!rsa_key->n)
		goto err;

	if (caam_rsa_check_key_length(raw_key.n_sz << 3)) {
		caam_rsa_free_key(rsa_key);
		return -EINVAL;
	}

	rsa_key->d_sz = raw_key.d_sz;
	rsa_key->e_sz = raw_key.e_sz;
	rsa_key->n_sz = raw_key.n_sz;

	memcpy(rsa_key->d, raw_key.d, raw_key.d_sz);
	memcpy(rsa_key->e, raw_key.e, raw_key.e_sz);

	return 0;

err:
	caam_rsa_free_key(rsa_key);
	return -ENOMEM;
#endif	
}


static int aspeed_rsa_max_size(struct crypto_akcipher *tfm)
{
//	struct aspeed_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
//	struct aspeed_rsa_key *key = &ctx->key;
	RSA_DBG("\n");
//	return (key->n) ? key->n_sz : -EINVAL;
}

static int aspeed_rsa_init_tfm(struct crypto_akcipher *tfm)
{
//	struct aspeed_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
//	struct akcipher_alg *alg = __crypto_akcipher_alg(tfm->base.__crt_alg);
//	struct aspeed_crypto_alg *algt;
	RSA_DBG("\n");

//	algt = container_of(alg, struct aspeed_crypto_alg, alg.akcipher);

//	ctx->crypto_dev = algt->crypto_dev;

	return 0;
}

static void aspeed_rsa_exit_tfm(struct crypto_akcipher *tfm)
{
	RSA_DBG("\n");

//	struct caam_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
//	struct caam_rsa_key *key = &ctx->key;

}

struct aspeed_crypto_alg aspeed_akcipher_algs[] = {
	{
		.alg.akcipher = {
			.encrypt = aspeed_rsa_enc,
			.decrypt = aspeed_rsa_dec,
			.sign = aspeed_rsa_dec,
			.verify = aspeed_rsa_enc,
			.set_pub_key = aspeed_rsa_setpubkey,
			.set_priv_key = aspeed_rsa_setprivkey,
			.max_size = aspeed_rsa_max_size,
			.init = aspeed_rsa_init_tfm,
			.exit = aspeed_rsa_exit_tfm,
			.base = {
				.cra_name = "rsa",
				.cra_driver_name = "aspeed-rsa",
				.cra_priority = 300,
				.cra_module = THIS_MODULE,
				.cra_ctxsize = sizeof(struct aspeed_rsa_ctx),
			},
		},
	},
};

int aspeed_register_akcipher_algs(struct aspeed_crypto_dev *crypto_dev)
{
	int i;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(aspeed_akcipher_algs); i++) {
		aspeed_akcipher_algs[i].crypto_dev = crypto_dev;
		err = crypto_register_ahash(&aspeed_akcipher_algs[i].alg.akcipher);
		if (err)
			return err;
	}
	return 0;
}
