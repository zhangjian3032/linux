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
#include "aspeed-crypto.h"

#define ASPEED_AES_QUEUE_LENGTH	1
#define ASPEED_HASH_BUFF_SIZE 	8192


#ifdef ASPEED_AHASH_DEBUG
#define AHASH_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#else
#define AHASH_DBG(fmt, args...)
#endif



static int aspeed_sham_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_sham_ctx *tctx = crypto_ahash_ctx(tfm);
	struct aspeed_sham_reqctx *ctx = ahash_request_ctx(req);
	struct aspeed_crypto_dev *aspeed_crypto = NULL, *tmp;

	AHASH_DBG("init: digest size: %d\n", crypto_ahash_digestsize(tfm));

	spin_lock_bh(&aspeed_drv.lock);
	if (!ctx->crypto_dev) {
		list_for_each_entry(tmp, &aspeed_drv.dev_list, list) {
			aspeed_crypto = tmp;
			break;
		}
		tctx->crypto_dev = aspeed_crypto;

	} else {
		aspeed_crypto = tctx->crypto_dev;
	}

	ctx->crypto_dev = aspeed_crypto;

	spin_unlock_bh(&aspeed_drv.lock);

	ctx->flags = 0;
	ctx->cmd = 0;

	switch (crypto_ahash_digestsize(tfm)) {
	case MD5_DIGEST_SIZE:
		ctx->cmd = HASH_CMD_MD5 | HASH_CMD_MD5_SWAP;
		ctx->block_size = SHA1_BLOCK_SIZE;
		ctx->digcnt = MD5_DIGEST_SIZE;
		break;
	case SHA1_DIGEST_SIZE:
		ctx->cmd = HASH_CMD_SHA1 | HASH_CMD_SHA_SWAP;
		ctx->block_size = SHA1_BLOCK_SIZE;
		ctx->digcnt = SHA1_DIGEST_SIZE;
		break;
	case SHA224_DIGEST_SIZE:
		ctx->cmd = HASH_CMD_SHA224 | HASH_CMD_SHA_SWAP;
		ctx->block_size = SHA224_BLOCK_SIZE;
		ctx->digcnt = SHA224_DIGEST_SIZE;
		break;
	case SHA256_DIGEST_SIZE:
		ctx->cmd = HASH_CMD_SHA256 | HASH_CMD_SHA_SWAP;
		ctx->block_size = SHA256_BLOCK_SIZE;
		ctx->digcnt = SHA256_DIGEST_SIZE;
		break;
	default:
		printk("%d not support \n", crypto_ahash_digestsize(tfm));
		return -EINVAL;
		break;
	}

	ctx->bufcnt = 0;

	if (tctx->flags & HASH_CMD_HMAC) {
		AHASH_DBG("wHMAC \n");
		ctx->cmd |= HASH_CMD_HMAC;
	}
	return 0;
}

static size_t aspeed_sha_append_sg(struct aspeed_sham_reqctx *ctx)
{
	size_t count;
	struct aspeed_crypto_dev	*aspeed_crypto = ctx->crypto_dev;
	u8	*hash_buff = aspeed_crypto->hash_src;
	AHASH_DBG("ctx->bufcnt %d, ctx->offset %d, ctx->total %d\n", ctx->bufcnt, ctx->offset, ctx->total);

	while ((ctx->bufcnt < ASPEED_HASH_BUFF_SIZE) && ctx->total) {
		count = min(ctx->sg->length - ctx->offset, ctx->total);
		count = min(count, ASPEED_HASH_BUFF_SIZE - ctx->bufcnt);

		if (count <= 0) {
			/*
			 * Check if count <= 0 because the buffer is full or
			 * because the sg length is 0. In the latest case,
			 * check if there is another sg in the list, a 0 length
			 * sg doesn't necessarily mean the end of the sg list.
			 */
			if ((ctx->sg->length == 0) && !sg_is_last(ctx->sg)) {
				ctx->sg = sg_next(ctx->sg);
				continue;
			} else
				break;
		}

		scatterwalk_map_and_copy(hash_buff + ctx->bufcnt, ctx->sg,
					 ctx->offset, count, 0);

		ctx->bufcnt += count;
		ctx->offset += count;
		ctx->total -= count;

		if (ctx->offset == ctx->sg->length) {
			ctx->sg = sg_next(ctx->sg);
			if (ctx->sg)
				ctx->offset = 0;
			else
				ctx->total = 0;
		}
	}
	AHASH_DBG("ctx->bufcnt %d, ctx->offset %d, ctx->total %d\n", ctx->bufcnt, ctx->offset, ctx->total);

	return 0;
}

static int aspeed_sha_update(struct ahash_request *req)
{
	struct aspeed_sham_reqctx *ctx = ahash_request_ctx(req);
//	struct aspeed_crypto_dev *aspeed_crypto = ctx->crypto_dev;

	if (!req->nbytes)
		return -1;

	AHASH_DBG("req->nbytes %d\n", req->nbytes);

	ctx->total = req->nbytes;
	ctx->sg = req->src;
	ctx->offset = 0;

	if (ctx->bufcnt + ctx->total <= ASPEED_HASH_BUFF_SIZE) {
		aspeed_sha_append_sg(ctx);
		return 0;
	} else {
		printk("aspeed_sha_update TODO ...ctx->bufcnt %d, ctx->total %d \n", ctx->bufcnt, ctx->total);
//		return aspeed_hash_handle_queue(aspeed_crypto, req);
	}

	return 0;
}

static int aspeed_sha_final(struct ahash_request *req)
{
	struct aspeed_sham_reqctx *ctx = ahash_request_ctx(req);
	struct aspeed_sham_ctx *tctx = crypto_tfm_ctx(req->base.tfm);
	struct aspeed_crypto_dev *aspeed_crypto = tctx->crypto_dev;

	ctx->flags = 1;
	ctx->op = 2;

	AHASH_DBG("ctx->bufcnt : %d , ctx->total : %d,  req->nbytes : %d \n", ctx->bufcnt, ctx->total, req->nbytes);
	if (req->nbytes) {
		printk("aspeed_sha_final ~~ TODO req->nbytes %d ~~~ \n", req->nbytes);
		aspeed_sha_update(req);
	}

	return aspeed_hash_handle_queue(aspeed_crypto, req);

}

static int aspeed_sha_finup(struct ahash_request *req)
{
//	struct aspeed_sham_reqctx *ctx = ahash_request_ctx(req);
	int err1, err2;

	AHASH_DBG("Final up TODO ...\n");

//	ctx->flags |= SHA_FLAGS_FINUP;

	err1 = aspeed_sha_update(req);
	if (err1 == -EINPROGRESS || err1 == -EBUSY)
		return err1;

	/*
	 * final() has to be always called to cleanup resources
	 * even if udpate() failed, except EINPROGRESS
	 */
	err2 = aspeed_sha_final(req);

	return err1 ? : err2;

}

static int aspeed_sha_digest(struct ahash_request *req)
{
	AHASH_DBG("TOTO~~~~~~\n");

	return aspeed_sham_init(req) ? : aspeed_sha_finup(req);
}

static int aspeed_sham_setkey(struct crypto_ahash *tfm, const u8 *key,
			      unsigned int keylen)
{
	struct aspeed_sham_ctx *ctx = crypto_ahash_ctx(tfm);
//	int bs = crypto_shash_blocksize(ctx->base_hash);
//	int ds = crypto_shash_digestsize(ctx->base_hash);
	struct aspeed_crypto_dev *aspeed_crypto = NULL, *tmp;
	int err = 0;

	AHASH_DBG("keylen %d\n", keylen);

	spin_lock_bh(&aspeed_drv.lock);
	if (!ctx->crypto_dev) {
		list_for_each_entry(tmp, &aspeed_drv.dev_list, list) {
			aspeed_crypto = tmp;
			break;
		}
		ctx->crypto_dev = aspeed_crypto;

	} else {
		aspeed_crypto = ctx->crypto_dev;
	}

	spin_unlock_bh(&aspeed_drv.lock);

	err = crypto_shash_setkey(ctx->fallback, key, keylen);
	if (err)
		return err;

	memcpy(aspeed_crypto->hash_key, key, keylen);

	return err;
}

/*************************************************************************************/
static int aspeed_sha_cra_init_alg(struct crypto_tfm *tfm, const char *alg_base)
{
	struct aspeed_sham_ctx *tctx = crypto_tfm_ctx(tfm);
	const char *alg_name = crypto_tfm_alg_name(tfm);

	AHASH_DBG("%s \n", alg_name);

	/* Allocate a fallback and abort if it failed. */
	tctx->fallback = crypto_alloc_shash(alg_name, 0,
					    CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(tctx->fallback)) {
		pr_err("aspeed-sham: fallback driver '%s' "
		       "could not be loaded.\n", alg_name);
		return PTR_ERR(tctx->fallback);
	}

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct aspeed_sham_reqctx));

	tctx->flags = 0;

	if (alg_base) {
		tctx->flags = HASH_CMD_HMAC;
		tctx->base_hash = crypto_alloc_shash(alg_base, 0,
						     CRYPTO_ALG_NEED_FALLBACK);
		if (IS_ERR(tctx->base_hash)) {
			pr_err("aspeed-sham: base driver '%s' "
			       "could not be loaded.\n", alg_base);
			crypto_free_shash(tctx->fallback);
			return PTR_ERR(tctx->base_hash);
		}
	}

	return 0;

}

static int aspeed_sham_cra_init(struct crypto_tfm *tfm)
{
	return aspeed_sha_cra_init_alg(tfm, NULL);
}

static int aspeed_sha_cra_sha1_init(struct crypto_tfm *tfm)
{
	return aspeed_sha_cra_init_alg(tfm, "sha1");
}

static int aspeed_sha_cra_sha224_init(struct crypto_tfm *tfm)
{
	return aspeed_sha_cra_init_alg(tfm, "sha224");
}

static int aspeed_sha_cra_sha256_init(struct crypto_tfm *tfm)
{
	return aspeed_sha_cra_init_alg(tfm, "sha256");
}

static int aspeed_sha_cra_md5_init(struct crypto_tfm *tfm)
{
	return aspeed_sha_cra_init_alg(tfm, "md5");
}

static void aspeed_sham_cra_exit(struct crypto_tfm *tfm)
{
	struct aspeed_sham_ctx *tctx = crypto_tfm_ctx(tfm);

	AHASH_DBG("\n");

	crypto_free_shash(tctx->fallback);
	tctx->fallback = NULL;

	if (tctx->base_hash) {
		crypto_free_shash(tctx->base_hash);
	}
}

static struct ahash_alg aspeed_ahash_alg[] = {
#if 1
	{
		.init			= aspeed_sham_init,
		.update 		= aspeed_sha_update,
		.final		= aspeed_sha_final,
		.finup		= aspeed_sha_finup,
		.digest 		= aspeed_sha_digest,
		.halg = {
			.digestsize = MD5_DIGEST_SIZE,
			.statesize	= sizeof(struct aspeed_sham_reqctx),
			.base	= {
				.cra_name		= "md5",
				.cra_driver_name	= "aspeed-md5",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
				CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_NEED_FALLBACK |
				CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize		= MD5_HMAC_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
				.cra_alignmask	= 0,
				.cra_module 		= THIS_MODULE,
				.cra_init			= aspeed_sham_cra_init,
				.cra_exit			= aspeed_sham_cra_exit,
			}
		}
	},
#endif
	{
		.init			= aspeed_sham_init,
		.update 		= aspeed_sha_update,
		.final		= aspeed_sha_final,
		.finup		= aspeed_sha_finup,
		.digest 		= aspeed_sha_digest,
		.halg = {
			.digestsize = SHA1_DIGEST_SIZE,
			.statesize = sizeof(struct aspeed_sham_reqctx),
			.base	= {
				.cra_name		= "sha1",
				.cra_driver_name	= "aspeed-sha1",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
				CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_NEED_FALLBACK |
				CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize		= SHA1_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
				.cra_alignmask	= 0,
				.cra_module 		= THIS_MODULE,
				.cra_init			= aspeed_sham_cra_init,
				.cra_exit			= aspeed_sham_cra_exit,
			}
		}
	},
	{
		.init			= aspeed_sham_init,
		.update 		= aspeed_sha_update,
		.final		= aspeed_sha_final,
		.finup		= aspeed_sha_finup,
		.digest 		= aspeed_sha_digest,
		.halg = {
			.digestsize = SHA256_DIGEST_SIZE,
			.statesize = sizeof(struct aspeed_sham_reqctx),
			.base	= {
				.cra_name		= "sha256",
				.cra_driver_name	= "aspeed-sha256",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
				CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_NEED_FALLBACK |
				CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize		= SHA256_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
				.cra_alignmask	= 0,
				.cra_module 		= THIS_MODULE,
				.cra_init			= aspeed_sham_cra_init,
				.cra_exit			= aspeed_sham_cra_exit,
			}
		}
	},
	{
		.init			= aspeed_sham_init,
		.update		= aspeed_sha_update,
		.final		= aspeed_sha_final,
		.finup		= aspeed_sha_finup,
		.digest		= aspeed_sha_digest,
		.halg = {
			.digestsize	= SHA224_DIGEST_SIZE,
			.statesize = sizeof(struct aspeed_sham_reqctx),
			.base	= {
				.cra_name		= "sha224",
				.cra_driver_name	= "aspeed-sha224",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
				CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_NEED_FALLBACK |
				CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize		= SHA224_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
				.cra_alignmask	= 0,
				.cra_module		= THIS_MODULE,
				.cra_init			= aspeed_sham_cra_init,
				.cra_exit			= aspeed_sham_cra_exit,
			}
		}
	},
	{
		.init			= aspeed_sham_init,
		.update		= aspeed_sha_update,
		.final		= aspeed_sha_final,
		.finup		= aspeed_sha_finup,
		.digest		= aspeed_sha_digest,
		.setkey		= aspeed_sham_setkey,
		.halg = {
			.digestsize	= MD5_DIGEST_SIZE,
			.statesize = sizeof(struct aspeed_sham_reqctx),
			.base	= {
				.cra_name		= "hmac(md5)",
				.cra_driver_name	= "aspeed-hmac-md5",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
				CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_NEED_FALLBACK |
				CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize		= MD5_HMAC_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
				.cra_alignmask	= 0,
				.cra_module		= THIS_MODULE,
				.cra_init		= aspeed_sha_cra_md5_init,
				.cra_exit		= aspeed_sham_cra_exit,
			}
		}
	},
	{
		.init			= aspeed_sham_init,
		.update		= aspeed_sha_update,
		.final		= aspeed_sha_final,
		.finup		= aspeed_sha_finup,
		.digest		= aspeed_sha_digest,
		.setkey		= aspeed_sham_setkey,
		.halg = {
			.digestsize	= SHA1_DIGEST_SIZE,
			.statesize = sizeof(struct aspeed_sham_reqctx),
			.base	= {
				.cra_name		= "hmac(sha1)",
				.cra_driver_name	= "aspeed-hmac-sha1",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
				CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_NEED_FALLBACK |
				CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize		= SHA1_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
				.cra_alignmask	= 0,
				.cra_module		= THIS_MODULE,
				.cra_init		= aspeed_sha_cra_sha1_init,
				.cra_exit		= aspeed_sham_cra_exit,
			}
		}
	},
	{
		.init			= aspeed_sham_init,
		.update 		= aspeed_sha_update,
		.final		= aspeed_sha_final,
		.finup		= aspeed_sha_finup,
		.digest 		= aspeed_sha_digest,
		.setkey 		= aspeed_sham_setkey,
		.halg = {
			.digestsize	= SHA224_DIGEST_SIZE,
			.statesize = sizeof(struct aspeed_sham_reqctx),
			.base	= {
				.cra_name		= "hmac(sha224)",
				.cra_driver_name	= "aspeed-hmac-sha224",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
				CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_NEED_FALLBACK |
				CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize		= SHA224_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
				.cra_alignmask	= 0,
				.cra_module 	= THIS_MODULE,
				.cra_init		= aspeed_sha_cra_sha224_init,
				.cra_exit		= aspeed_sham_cra_exit,
			}
		}
	},
	{
		.init			= aspeed_sham_init,
		.update 		= aspeed_sha_update,
		.final		= aspeed_sha_final,
		.finup		= aspeed_sha_finup,
		.digest 		= aspeed_sha_digest,
		.setkey 		= aspeed_sham_setkey,
		.halg = {
			.digestsize	= SHA256_DIGEST_SIZE,
			.statesize = sizeof(struct aspeed_sham_reqctx),
			.base	= {
				.cra_name		= "hmac(sha256)",
				.cra_driver_name	= "aspeed-hmac-sha256",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
				CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_NEED_FALLBACK |
				CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize		= SHA256_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
				.cra_alignmask	= 0,
				.cra_module 	= THIS_MODULE,
				.cra_init		= aspeed_sha_cra_sha256_init,
				.cra_exit		= aspeed_sham_cra_exit,
			}
		}
	},
};

int aspeed_register_ahash_algs(struct aspeed_crypto_dev *crypto_dev)
{
	int i;
	int err = 0;
	crypto_dev->ahash_algs		= &aspeed_ahash_alg;
	for (i = 0; i < ARRAY_SIZE(aspeed_ahash_alg); i++) {
		err = crypto_register_ahash(&aspeed_ahash_alg[i]);
		if (err)
			printk("aspeed_ahash_alg~~~ ERROR ~~~\n");
	}
}

