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

#include "aspeed-crypto.h"

//#define ASPEED_AHASH_DEBUG

#ifdef ASPEED_AHASH_DEBUG
//#define AHASH_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#define AHASH_DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define AHASH_DBG(fmt, args...)
#endif

int aspeed_crypto_ahash_trigger(struct aspeed_crypto_dev *crypto_dev)
{
	struct ahash_request *req = crypto_dev->ahash_req;
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_sham_ctx *ctx = crypto_ahash_ctx(tfm);
	struct aspeed_sham_reqctx *req_ctx = ahash_request_ctx(req);
//	int i = 0;
//	u8 *buff = crypto_dev->hash_digst;

//	crypto_dev->cmd |= HASH_CMD_INT_ENABLE;

	while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_HASH_BUSY);

	aspeed_crypto_write(crypto_dev, crypto_dev->hash_src_dma, ASPEED_HACE_HASH_SRC);
	AHASH_DBG("0x20 src : %x \n", crypto_dev->hash_src_dma);
	aspeed_crypto_write(crypto_dev, ctx->hash_digst_dma, ASPEED_HACE_HASH_DIGEST_BUFF);
	AHASH_DBG("0x24 gigest : %x \n", ctx->hash_digst_dma);

	if (ctx->flags)
		aspeed_crypto_write(crypto_dev, ctx->hmac_key_dma, ASPEED_HACE_HASH_KEY_BUFF);

	AHASH_DBG("0x28 key : %x \n", ctx->hmac_key_dma);
	aspeed_crypto_write(crypto_dev, req_ctx->total, ASPEED_HACE_HASH_DATA_LEN);
	AHASH_DBG("0x2c len : %x \n", req_ctx->total);

	AHASH_DBG("cmd : %x \n", req_ctx->cmd);
	aspeed_crypto_write(crypto_dev, req_ctx->cmd, ASPEED_HACE_HASH_CMD);

	while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_HASH_BUSY);
#if 0
	printk("digst %d \n", req_ctx->digcnt);
	for (i = 0; i < req_ctx->digcnt; i++)
		printk("%02x ", buff[i]);
	printk(" \n");
#endif
	memcpy(req->result, ctx->hash_digst, req_ctx->digcnt);

	AHASH_DBG("done : copy to result \n");
	return 0;
}

static int aspeed_sham_final(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_sham_reqctx *req_ctx = ahash_request_ctx(req);
	struct aspeed_sham_ctx *ctx = crypto_ahash_ctx(tfm);
	struct aspeed_crypto_dev *crypto_dev = ctx->crypto_dev;
	int err;
	unsigned long flags;

	AHASH_DBG("req->nbytes %d , req_ctx->total %d\n", req->nbytes, req_ctx->total);

	spin_lock_irqsave(&crypto_dev->lock, flags);
	err = crypto_enqueue_request(&crypto_dev->queue, &req->base);
	spin_unlock_irqrestore(&crypto_dev->lock, flags);

	tasklet_schedule(&crypto_dev->crypto_tasklet);

	return err;
}

static int aspeed_sham_update(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_sham_ctx *ctx = crypto_ahash_ctx(tfm);
	struct aspeed_sham_reqctx *req_ctx = ahash_request_ctx(req);
	struct aspeed_crypto_dev *crypto_dev = ctx->crypto_dev;
	unsigned long flags;
	int err;

	AHASH_DBG("now req_ctx->total %d, add req->nbytes %d , sg_nents %d \n", req_ctx->total, req->nbytes, sg_nents(req->src));

	if (req_ctx->total + req->nbytes <= PAGE_SIZE) {
		scatterwalk_map_and_copy(crypto_dev->hash_src + req_ctx->total, req->src,
					 0, req->nbytes, 0);
		req_ctx->total += req->nbytes;
	} else {
		printk("TODO ~~ aspeed_sha_update ctx->total %d \n", req_ctx->total);
		AHASH_DBG("TODO ~~ should trigger update accumulate mode \n");
		while (1);
	}

	if ((!req_ctx->flags) && (req_ctx->total > PAGE_SIZE)) {
		AHASH_DBG("TODO ~~ should trigger update accumulate mode \n");
		printk("TODO e\n");
		while (1);
//		AHASH_DBG("enqueue ctx->total %d req->nbytes %d req->src %x crypto dev %x ctx->flags %d\n", ctx->total, req->nbytes, req->src, crypto_dev, ctx->flags);
		spin_lock_irqsave(&crypto_dev->lock, flags);
		err = crypto_enqueue_request(&crypto_dev->queue, &req->base);
		spin_unlock_irqrestore(&crypto_dev->lock, flags);
		tasklet_schedule(&crypto_dev->crypto_tasklet);
		return err;
//		return crypto_enqueue_request(&crypto_dev->queue, &req->base);
	} else {
		return 0;
	}
}

static int aspeed_sham_finup(struct ahash_request *req)
{
	struct aspeed_sham_reqctx *reqctx = ahash_request_ctx(req);

	int err1, err2;

	AHASH_DBG("\n");
	reqctx->flags = 1;

	err1 = aspeed_sham_update(req);
	if (err1 == -EINPROGRESS || err1 == -EBUSY)
		return err1;

	/*
	 * final() has to be always called to cleanup resources
	 * even if udpate() failed, except EINPROGRESS
	 */
	err2 = aspeed_sham_final(req);

	return err1 ? : err2;
}

static int aspeed_sham_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_sham_ctx *tctx = crypto_ahash_ctx(tfm);
	struct aspeed_sham_reqctx *req_ctx = ahash_request_ctx(req);

	AHASH_DBG("digest size: %d\n", crypto_ahash_digestsize(tfm));

	req_ctx->flags = 0;

	switch (crypto_ahash_digestsize(tfm)) {
	case MD5_DIGEST_SIZE:
		req_ctx->cmd |= HASH_CMD_MD5 | HASH_CMD_MD5_SWAP;
		req_ctx->digcnt = MD5_DIGEST_SIZE;
		break;
	case SHA1_DIGEST_SIZE:
		req_ctx->cmd |= HASH_CMD_SHA1 | HASH_CMD_SHA_SWAP;
		req_ctx->digcnt = SHA1_DIGEST_SIZE;
		break;
	case SHA224_DIGEST_SIZE:
		req_ctx->cmd |= HASH_CMD_SHA224 | HASH_CMD_SHA_SWAP;
		req_ctx->digcnt = SHA224_DIGEST_SIZE;
		break;
	case SHA256_DIGEST_SIZE:
		req_ctx->cmd |= HASH_CMD_SHA256 | HASH_CMD_SHA_SWAP;
		req_ctx->digcnt = SHA256_DIGEST_SIZE;
		break;
	default:
		printk("%d not support \n", crypto_ahash_digestsize(tfm));
		return -EINVAL;
		break;
	}

	req_ctx->bufcnt = 0;
	req_ctx->total = 0;
	req_ctx->offset = 0;
	req_ctx->buflen = SHA512_BLOCK_SIZE;

	if (tctx->flags) {
		//hmac cmd
		req_ctx->cmd |= HASH_CMD_HMAC;
	}

	return 0;
}

static int aspeed_sham_digest(struct ahash_request *req)
{
	AHASH_DBG("\n");
	return aspeed_sham_init(req) ? : aspeed_sham_finup(req);
}

static int aspeed_sham_setkey(struct crypto_ahash *tfm, const u8 *key,
			      unsigned int keylen)
{
	struct aspeed_sham_ctx *ctx = crypto_ahash_ctx(tfm);
	struct aspeed_crypto_dev *crypto_dev = ctx->crypto_dev;
	int err = 0;
	size_t			digcnt;
	u32 cmd;

	AHASH_DBG("keylen %d crypto_dev %x \n", keylen, (u32)crypto_dev);

	switch (crypto_ahash_digestsize(tfm)) {
	case MD5_DIGEST_SIZE:
		cmd = HASH_CMD_MD5 | HASH_CMD_MD5_SWAP;
		digcnt = MD5_DIGEST_SIZE;
		break;
	case SHA1_DIGEST_SIZE:
		cmd = HASH_CMD_SHA1 | HASH_CMD_SHA_SWAP;
		digcnt = SHA1_DIGEST_SIZE;
		break;
	case SHA224_DIGEST_SIZE:
		cmd = HASH_CMD_SHA224 | HASH_CMD_SHA_SWAP;
		digcnt = SHA224_DIGEST_SIZE;
		break;
	case SHA256_DIGEST_SIZE:
		cmd = HASH_CMD_SHA256 | HASH_CMD_SHA_SWAP;
		digcnt = SHA256_DIGEST_SIZE;
		break;
	default:
		printk("%d not support \n", crypto_ahash_digestsize(tfm));
		return -EINVAL;
		break;
	}

#if 0
	printk("key : \n");
	print_hex_dump(KERN_CONT, "", DUMP_PREFIX_OFFSET,
		       16, 1,
		       key, keylen, false);
	printk("\n");
#endif
	memset(ctx->hash_digst, 0, 64);

	if (keylen > 64) {
		AHASH_DBG("gen hash key keylen %d crypto_dev->hash_key_dma trigger  %x \n", keylen, ctx->hash_digst_dma);
		//gen hash(key)
		memcpy(crypto_dev->hash_src, key, keylen);
		aspeed_crypto_write(crypto_dev, crypto_dev->hash_src_dma, ASPEED_HACE_HASH_SRC);
		aspeed_crypto_write(crypto_dev, ctx->hash_digst_dma, ASPEED_HACE_HASH_DIGEST_BUFF);
		aspeed_crypto_write(crypto_dev, keylen, ASPEED_HACE_HASH_DATA_LEN);
		aspeed_crypto_write(crypto_dev, cmd, ASPEED_HACE_HASH_CMD);
		while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_HASH_BUSY);
		//for workaround for SHA224 fill 32 bytes
		if (digcnt == SHA224_DIGEST_SIZE) {
			memset(ctx->hash_digst + 28, 0, 4);
		}
	} else {
		memcpy(ctx->hash_digst, key, keylen);
	}

	cmd |= HASH_CMD_HMAC;

	aspeed_crypto_write(crypto_dev, ctx->hash_digst_dma, ASPEED_HACE_HASH_SRC);
	aspeed_crypto_write(crypto_dev, ctx->hmac_key_dma, ASPEED_HACE_HASH_KEY_BUFF);
	aspeed_crypto_write(crypto_dev, 0x40, ASPEED_HACE_HASH_DATA_LEN);
	aspeed_crypto_write(crypto_dev, cmd | BIT(8), ASPEED_HACE_HASH_CMD);
	while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_HASH_BUSY);

	return err;
}

static int aspeed_sham_cra_init_alg(struct crypto_tfm *tfm, const char *alg_base)
{
	struct aspeed_sham_ctx *tctx = crypto_tfm_ctx(tfm);
	const char *alg_name = crypto_tfm_alg_name(tfm);
	struct aspeed_crypto_alg *algt;
	struct ahash_alg *alg = __crypto_ahash_alg(tfm->__crt_alg);
	algt = container_of(alg, struct aspeed_crypto_alg, alg.ahash);
	tctx->crypto_dev = algt->crypto_dev;

	AHASH_DBG("%s crypto dev %x \n", alg_name, (u32)tctx->crypto_dev);

	/* Allocate a fallback and abort if it failed. */
	tctx->fallback = crypto_alloc_shash(alg_name, 0,
					    CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(tctx->fallback)) {
		pr_err("aspeed-sham: fallback driver '%s' "
		       "could not be loaded.\n", alg_name);
		return PTR_ERR(tctx->fallback);
	}

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct aspeed_sham_reqctx) + SHA512_BLOCK_SIZE);


	tctx->hash_digst = dma_alloc_coherent(tctx->crypto_dev->dev, PAGE_SIZE, &tctx->hash_digst_dma, GFP_KERNEL);

	tctx->hmac_key = tctx->hash_digst + 2048;
	tctx->hmac_key_dma = tctx->hash_digst_dma + 2048;

	if (alg_base) {
		struct aspeed_sham_hmac_ctx *bctx = tctx->base;
		tctx->flags = 1;
		bctx->shash = crypto_alloc_shash(alg_base, 0,
						 CRYPTO_ALG_NEED_FALLBACK);
		if (IS_ERR(bctx->shash)) {
			pr_err("aspeed-sham: base driver '%s' "
			       "could not be loaded.\n", alg_base);
			crypto_free_shash(tctx->fallback);
			return PTR_ERR(bctx->shash);
		}
	}

	return 0;
}

static int aspeed_sham_cra_init(struct crypto_tfm *tfm)
{
	return aspeed_sham_cra_init_alg(tfm, NULL);
}

static int aspeed_sham_cra_sha1_init(struct crypto_tfm *tfm)
{
	return aspeed_sham_cra_init_alg(tfm, "sha1");
}

static int aspeed_sham_cra_sha224_init(struct crypto_tfm *tfm)
{
	return aspeed_sham_cra_init_alg(tfm, "sha224");
}

static int aspeed_sham_cra_sha256_init(struct crypto_tfm *tfm)
{
	return aspeed_sham_cra_init_alg(tfm, "sha256");
}

static int aspeed_sham_cra_md5_init(struct crypto_tfm *tfm)
{
	return aspeed_sham_cra_init_alg(tfm, "md5");
}

static int aspeed_sham_cra_sha384_init(struct crypto_tfm *tfm)
{
	return aspeed_sham_cra_init_alg(tfm, "sha384");
}

static int aspeed_sham_cra_sha512_init(struct crypto_tfm *tfm)
{
	return aspeed_sham_cra_init_alg(tfm, "sha512");
}

static void aspeed_sham_cra_exit(struct crypto_tfm *tfm)
{
	struct aspeed_sham_ctx *tctx = crypto_tfm_ctx(tfm);

	AHASH_DBG("\n");

	crypto_free_shash(tctx->fallback);
	tctx->fallback = NULL;
	dma_free_coherent(tctx->crypto_dev->dev, PAGE_SIZE, tctx->hash_digst, tctx->hash_digst_dma);

	if (tctx->flags) {
		struct aspeed_sham_hmac_ctx *bctx = tctx->base;
		AHASH_DBG("HMAC \n");
		crypto_free_shash(bctx->shash);
	}
}

static int aspeed_sham_export(struct ahash_request *req, void *out)
{
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);
	AHASH_DBG("rctx->bufcnt %d \n", rctx->bufcnt);

	memcpy(out, rctx, sizeof(*rctx) + rctx->bufcnt);
	return 0;
}

static int aspeed_sham_import(struct ahash_request *req, const void *in)
{
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);
	const struct aspeed_sham_reqctx *ctx_in = in;
	AHASH_DBG("ctx_in->bufcnt %d \n", ctx_in->bufcnt);

	memcpy(rctx, in, sizeof(*rctx) + ctx_in->bufcnt);
	return 0;
}

struct aspeed_crypto_alg aspeed_ahash_algs[] = {
	{
		.alg.ahash = {
			.init	= aspeed_sham_init,
			.update	= aspeed_sham_update,
			.final	= aspeed_sham_final,
			.finup	= aspeed_sham_finup,
			.digest	= aspeed_sham_digest,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = MD5_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx) + MD5_BLOCK_WORDS,
				.base = {
					.cra_name		= "md5",
					.cra_driver_name	= "aspeed-md5",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_NEED_FALLBACK |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= MD5_HMAC_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask		= 0,	//TODO check alignmask ???
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_init,
					.cra_exit		= aspeed_sham_cra_exit,
				}
			}
		},
	},
	{
		.alg.ahash = {
			.init	= aspeed_sham_init,
			.update	= aspeed_sham_update,
			.final	= aspeed_sham_final,
			.finup	= aspeed_sham_finup,
			.digest	= aspeed_sham_digest,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = SHA1_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "sha1",
					.cra_driver_name	= "aspeed-sha1",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_NEED_FALLBACK |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA1_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_init,
					.cra_exit		= aspeed_sham_cra_exit,
				}
			}
		},
	},
	{
		.alg.ahash = {
			.init	= aspeed_sham_init,
			.update	= aspeed_sham_update,
			.final	= aspeed_sham_final,
			.finup	= aspeed_sham_finup,
			.digest	= aspeed_sham_digest,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = SHA256_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "sha256",
					.cra_driver_name	= "aspeed-sha256",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_NEED_FALLBACK |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA256_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_init,
					.cra_exit		= aspeed_sham_cra_exit,
				}
			}
		},
	},
	{
		.alg.ahash = {
			.init	= aspeed_sham_init,
			.update	= aspeed_sham_update,
			.final	= aspeed_sham_final,
			.finup	= aspeed_sham_finup,
			.digest	= aspeed_sham_digest,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = SHA224_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "sha224",
					.cra_driver_name	= "aspeed-sha224",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_NEED_FALLBACK |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA224_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_init,
					.cra_exit		= aspeed_sham_cra_exit,
				}
			}
		},
	},
	{
		.alg.ahash = {
			.init	= aspeed_sham_init,
			.update	= aspeed_sham_update,
			.final	= aspeed_sham_final,
			.finup	= aspeed_sham_finup,
			.digest	= aspeed_sham_digest,
			.setkey	= aspeed_sham_setkey,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = MD5_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx) + SHA512_BLOCK_SIZE,
				.base = {
					.cra_name		= "hmac(md5)",
					.cra_driver_name	= "aspeed-hmac-md5",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_NEED_FALLBACK |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= MD5_HMAC_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_md5_init,
					.cra_exit		= aspeed_sham_cra_exit,
				}
			}
		},
	},

	{
		.alg.ahash = {
			.init	= aspeed_sham_init,
			.update	= aspeed_sham_update,
			.final	= aspeed_sham_final,
			.finup	= aspeed_sham_finup,
			.digest	= aspeed_sham_digest,
			.setkey	= aspeed_sham_setkey,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = SHA1_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "hmac(sha1)",
					.cra_driver_name	= "aspeed-hmac-sha1",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_NEED_FALLBACK |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA1_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_sha1_init,
					.cra_exit		= aspeed_sham_cra_exit,
				}
			}
		},
	},
	{
		.alg.ahash = {
			.init	= aspeed_sham_init,
			.update	= aspeed_sham_update,
			.final	= aspeed_sham_final,
			.finup	= aspeed_sham_finup,
			.digest	= aspeed_sham_digest,
			.setkey	= aspeed_sham_setkey,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = SHA224_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "hmac(sha224)",
					.cra_driver_name	= "aspeed-hmac-sha224",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_NEED_FALLBACK |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA224_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_sha224_init,
					.cra_exit		= aspeed_sham_cra_exit,
				}
			}
		},
	},
	{
		.alg.ahash = {
			.init	= aspeed_sham_init,
			.update	= aspeed_sham_update,
			.final	= aspeed_sham_final,
			.finup	= aspeed_sham_finup,
			.digest	= aspeed_sham_digest,
			.setkey	= aspeed_sham_setkey,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = SHA256_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "hmac(sha256)",
					.cra_driver_name	= "aspeed-hmac-sha256",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_NEED_FALLBACK |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA256_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_sha256_init,
					.cra_exit		= aspeed_sham_cra_exit,
				}
			}
		},
	},
};

int aspeed_register_ahash_algs(struct aspeed_crypto_dev *crypto_dev)
{
	int i;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(aspeed_ahash_algs); i++) {
		aspeed_ahash_algs[i].crypto_dev = crypto_dev;
		err = crypto_register_ahash(&aspeed_ahash_algs[i].alg.ahash);
		if (err)
			return err;
	}
	return 0;
}
