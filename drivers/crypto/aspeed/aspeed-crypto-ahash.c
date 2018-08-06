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

// #define ASPEED_AHASH_DEBUG

#ifdef ASPEED_AHASH_DEBUG
//#define AHASH_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#define AHASH_DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define AHASH_DBG(fmt, args...)
#endif

const u32 md5_iv[8] = {
	MD5_H0, MD5_H1, MD5_H2, MD5_H3,
	0, 0, 0, 0
};

const u32 sha1_iv[8] = {
	SHA1_H0, SHA1_H1, SHA1_H2, SHA1_H3,
	SHA1_H4, 0, 0, 0
};

const u32 sha224_iv[8] = {
	SHA224_H0, SHA224_H1, SHA224_H2, SHA224_H3,
	SHA224_H4, SHA224_H5, SHA224_H6, SHA224_H7
};

const u32 sha256_iv[8] = {
	SHA256_H0, SHA256_H1, SHA256_H2, SHA256_H3,
	SHA256_H4, SHA256_H5, SHA256_H6, SHA256_H7
};

static int aspeed_ahash_complete(struct aspeed_crypto_dev *crypto_dev, int err)
{
	struct ahash_request *req = crypto_dev->ahash_req;

	AHASH_DBG("\n");

	crypto_dev->flags &= ~CRYPTO_FLAGS_BUSY;

	if (crypto_dev->is_async)
		req->base.complete(&req->base, err);
	tasklet_schedule(&crypto_dev->queue_task);

	return err;
}

static int aspeed_ahash_transfer(struct aspeed_crypto_dev *crypto_dev)
{
	struct ahash_request *req = crypto_dev->ahash_req;
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_sham_ctx *tctx = crypto_ahash_ctx(tfm);
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);

	AHASH_DBG("\n");
	dma_unmap_single(tctx->crypto_dev->dev, rctx->digest_dma_addr, rctx->digsize, DMA_FROM_DEVICE);
	memcpy(req->result, rctx->digest, rctx->digsize);

	return aspeed_ahash_complete(crypto_dev, 0);
}

static inline int aspeed_ahash_wait_for_data_ready(struct aspeed_crypto_dev *crypto_dev,
		aspeed_crypto_fn_t resume)
{
#ifdef CONFIG_CRYPTO_DEV_ASPEED_AHASH_INT
	// u32 isr = aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS);
	// AHASH_DBG("\n");

	// if (unlikely(isr & HACE_HASH_ISR))
	// 	return resume(crypto_dev);

	crypto_dev->resume = resume;
	return -EINPROGRESS;
#else
	u32 sts = aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS);

	AHASH_DBG("\n");
	// printk("aspeed_ahash_wait_for_data_ready sts : %x\n", sts);
	while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_HASH_BUSY);
	aspeed_crypto_write(crypto_dev, sts, ASPEED_HACE_STS);
	return resume(crypto_dev);
#endif
}

int aspeed_crypto_ahash_trigger(struct aspeed_crypto_dev *crypto_dev)
{
	struct ahash_request *req = crypto_dev->ahash_req;
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);

	AHASH_DBG("\n");
#ifdef CONFIG_CRYPTO_DEV_ASPEED_AHASH_INT
	rctx->cmd |= HASH_CMD_INT_ENABLE;
#else
	while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_HASH_BUSY);
#endif
	if (rctx->flags & SHA_FLAGS_CPU) {
		aspeed_crypto_write(crypto_dev, rctx->buffer_dma_addr, ASPEED_HACE_HASH_SRC);
		aspeed_crypto_write(crypto_dev, rctx->digest_dma_addr, ASPEED_HACE_HASH_DIGEST_BUFF);
		aspeed_crypto_write(crypto_dev, rctx->bufcnt, ASPEED_HACE_HASH_DATA_LEN);
		aspeed_crypto_write(crypto_dev, rctx->cmd, ASPEED_HACE_HASH_CMD);
		return aspeed_ahash_wait_for_data_ready(crypto_dev, aspeed_ahash_transfer);
	}
	return 0;
}

static int aspeed_sham_final(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);
	struct aspeed_sham_ctx *tctx = crypto_ahash_ctx(tfm);
	struct aspeed_crypto_dev *crypto_dev = tctx->crypto_dev;

	AHASH_DBG("req->nbytes %d , rctx->total %d\n", req->nbytes, rctx->total);
	if (rctx->flags & SHA_FLAGS_CPU) {
		rctx->buffer_dma_addr = tctx->hash_src_dma;
		rctx->bufcnt = rctx->total;
		return aspeed_crypto_handle_queue(crypto_dev, &req->base);
	}
	// if (rctx->flags & SHA_FLAGS_SINGLE_SG) {
	// 	dma_map_sg(crypto_dev->dev, rctx->src_sg, 1, DMA_TO_DEVICE);
	// 	rctx->buffer_dma_addr = rctx->src_sg->dma_address;
	// 	printk("buffer_dma_addr = %x\n", rctx->buffer_dma_addr);
	// 	rctx->bufcnt = rctx->total;
	// 	return aspeed_crypto_handle_queue(crypto_dev, &req->base);
	// }
	return 0;
}

static int aspeed_sham_update(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_sham_ctx *tctx = crypto_ahash_ctx(tfm);
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);
	struct aspeed_crypto_dev *crypto_dev = tctx->crypto_dev;
	struct scatterlist *sg;
	int i;
	int nents = sg_nents(req->src);
	AHASH_DBG("\n");
	// AHASH_DBG("now rctx->total %d, add req->nbytes %d , sg_nents %d \n", rctx->total, req->nbytes, sg_nents(req->src));
	if (!req->nbytes)
		return 0;
	rctx->total = req->nbytes;
	rctx->src_sg = req->src;
	rctx->offset = 0;
	for_each_sg(req->src, sg, nents, i) {
		dma_map_sg(crypto_dev->dev, sg, 1, DMA_TO_DEVICE);
		AHASH_DBG("nent %d, address : %x, offset : %d, length : %d\n", i, sg->dma_address, sg->offset, sg->length);
	}
	AHASH_DBG("nents : %d, req->nbytes : %d\n", nents, req->nbytes);

	if (!(rctx->flags & (SHA_FLAGS_SINGLE_UPDATE | SHA_FLAGS_N_UPDATES))) {
		rctx->flags |= SHA_FLAGS_SINGLE_UPDATE;
		AHASH_DBG("SHA_FLAGS_SINGLE_UPDATE\n");

		if (req->nbytes <= PAGE_SIZE * 10) {
			AHASH_DBG("CPU\n");
			rctx->flags |= SHA_FLAGS_CPU;
			rctx->digcnt += rctx->total;
			sg_copy_to_buffer(rctx->src_sg, sg_nents(rctx->src_sg), tctx->hash_src, req->nbytes);
		} else {
			AHASH_DBG("Not yet support accumulative mode\n");
			return -EINVAL;
		}
		// if (sg_is_last(req->src)) {
		// 	AHASH_DBG("DMA\n");
		// 	rctx->flags |= SHA_FLAGS_SINGLE_SG;
		// } else {
		// 	rctx->flags |= SHA_FLAGS_N_SG;
		// 	if (req->nbytes <= PAGE_SIZE * 10) {
		// 		AHASH_DBG("CPU\n");
		// 		rctx->flags |= SHA_FLAGS_CPU;
		// 	}
		// }
		return 0;
	} else {
		AHASH_DBG("Not yet support multi updates\n");
		rctx->flags |= ~SHA_FLAGS_SINGLE_UPDATE;
		rctx->flags |= SHA_FLAGS_N_UPDATES;
		return -EINVAL;
	}
	return 0;
}

static int aspeed_sham_finup(struct ahash_request *req)
{
	int err1, err2;

	AHASH_DBG("\n");

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
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);

	AHASH_DBG("digest size: %d\n", crypto_ahash_digestsize(tfm));

	rctx->cmd = 0;

	switch (crypto_ahash_digestsize(tfm)) {
	case MD5_DIGEST_SIZE:
		rctx->cmd |= HASH_CMD_MD5 | HASH_CMD_MD5_SWAP;
		rctx->digsize = MD5_DIGEST_SIZE;
		rctx->block_size = MD5_HMAC_BLOCK_SIZE;
		memcpy(rctx->digest, md5_iv, 32);
		break;
	case SHA1_DIGEST_SIZE:
		rctx->cmd |= HASH_CMD_SHA1 | HASH_CMD_SHA_SWAP;
		rctx->digsize = SHA1_DIGEST_SIZE;
		rctx->block_size = SHA1_BLOCK_SIZE;
		memcpy(rctx->digest, sha1_iv, 32);
		break;
	case SHA224_DIGEST_SIZE:
		rctx->cmd |= HASH_CMD_SHA224 | HASH_CMD_SHA_SWAP;
		rctx->digsize = SHA224_DIGEST_SIZE;
		rctx->block_size = SHA224_BLOCK_SIZE;
		memcpy(rctx->digest, sha224_iv, 32);
		break;
	case SHA256_DIGEST_SIZE:
		rctx->cmd |= HASH_CMD_SHA256 | HASH_CMD_SHA_SWAP;
		rctx->digsize = SHA256_DIGEST_SIZE;
		rctx->block_size = SHA256_BLOCK_SIZE;
		memcpy(rctx->digest, sha256_iv, 32);
		break;
	default:
		printk("%d not support \n", crypto_ahash_digestsize(tfm));
		return -EINVAL;
		break;
	}
	rctx->flags = 0;
	rctx->bufcnt = 0;
	rctx->total = 0;
	rctx->digcnt = 0;
	rctx->buflen = SHA_BUFFER_LEN;
	rctx->digest_dma_addr = dma_map_single(tctx->crypto_dev->dev, rctx->digest,
					       SHA512_DIGEST_SIZE, DMA_FROM_DEVICE);
	if (dma_mapping_error(tctx->crypto_dev->dev, rctx->digest_dma_addr))
		return -EINVAL;
	//hmac cmd
	if (tctx->flags)
		rctx->cmd |= HASH_CMD_HMAC;

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
	struct aspeed_sham_ctx *tctx = crypto_ahash_ctx(tfm);
	struct aspeed_crypto_dev *crypto_dev = tctx->crypto_dev;
	int err = 0;
	size_t			digsize;
	u32 cmd;

	AHASH_DBG("keylen %d crypto_dev %x \n", keylen, (u32)crypto_dev);

	switch (crypto_ahash_digestsize(tfm)) {
	case MD5_DIGEST_SIZE:
		cmd = HASH_CMD_MD5 | HASH_CMD_MD5_SWAP;
		digsize = MD5_DIGEST_SIZE;
		break;
	case SHA1_DIGEST_SIZE:
		cmd = HASH_CMD_SHA1 | HASH_CMD_SHA_SWAP;
		digsize = SHA1_DIGEST_SIZE;
		break;
	case SHA224_DIGEST_SIZE:
		cmd = HASH_CMD_SHA224 | HASH_CMD_SHA_SWAP;
		digsize = SHA224_DIGEST_SIZE;
		break;
	case SHA256_DIGEST_SIZE:
		cmd = HASH_CMD_SHA256 | HASH_CMD_SHA_SWAP;
		digsize = SHA256_DIGEST_SIZE;
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
	// memset(tctx->hash_digst, 0, 64);

	// if (keylen > 64) {
	// 	AHASH_DBG("gen hash key keylen %d crypto_dev->hash_key_dma trigger  %x \n", keylen, tctx->hash_digst_dma);
	// 	//gen hash(key)
	// 	memcpy(crypto_dev->hash_src, key, keylen);
	// 	aspeed_crypto_write(crypto_dev, crypto_dev->hash_src_dma, ASPEED_HACE_HASH_SRC);
	// 	aspeed_crypto_write(crypto_dev, tctx->hash_digst_dma, ASPEED_HACE_HASH_DIGEST_BUFF);
	// 	aspeed_crypto_write(crypto_dev, keylen, ASPEED_HACE_HASH_DATA_LEN);
	// 	aspeed_crypto_write(crypto_dev, cmd, ASPEED_HACE_HASH_CMD);
	// 	while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_HASH_BUSY);
	// 	//for workaround for SHA224 fill 32 bytes
	// 	if (digsize == SHA224_DIGEST_SIZE) {
	// 		memset(tctx->hash_digst + 28, 0, 4);
	// 	}
	// } else {
	// 	memcpy(tctx->hash_digst, key, keylen);
	// }

	// cmd |= HASH_CMD_HMAC;

	// aspeed_crypto_write(crypto_dev, tctx->hash_digst_dma, ASPEED_HACE_HASH_SRC);
	// aspeed_crypto_write(crypto_dev, tctx->hmac_key_dma, ASPEED_HACE_HASH_KEY_BUFF);
	// aspeed_crypto_write(crypto_dev, 0x40, ASPEED_HACE_HASH_DATA_LEN);
	// aspeed_crypto_write(crypto_dev, cmd | BIT(8), ASPEED_HACE_HASH_CMD);
	// while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_HASH_BUSY);

	return err;
}

static int aspeed_sham_cra_init_alg(struct crypto_tfm *tfm, const char *alg_base)
{
	struct aspeed_sham_ctx *tctx = crypto_tfm_ctx(tfm);
	struct aspeed_crypto_alg *algt;
	struct ahash_alg *alg = __crypto_ahash_alg(tfm->__crt_alg);

	algt = container_of(alg, struct aspeed_crypto_alg, alg.ahash);
	tctx->crypto_dev = algt->crypto_dev;

	AHASH_DBG("%s crypto dev %x \n", crypto_tfm_alg_name(tfm), (u32)tctx->crypto_dev);

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct aspeed_sham_reqctx) + SHA512_BLOCK_SIZE);

	tctx->hash_src = dma_alloc_coherent(tctx->crypto_dev->dev, PAGE_SIZE * 10, &tctx->hash_src_dma, GFP_KERNEL);
	// tctx->hash_digst = dma_alloc_coherent(tctx->crypto_dev->dev, 0xa000, &tctx->hash_digst_dma, GFP_KERNEL);

	// tctx->hmac_key = tctx->hash_digst + 2048;
	// tctx->hmac_key_dma = tctx->hash_digst_dma + 2048;

	// tctx->hash_src = tctx->hash_digst + 4096;
	// tctx->hash_src_dma = tctx->hash_digst_dma + 4096;

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

// static int aspeed_sha_cra_sha384_init(struct crypto_tfm *tfm)
// {
// 	return aspeed_sham_cra_init_alg(tfm, "sha384");
// }

// static int aspeed_sha_cra_sha512_init(struct crypto_tfm *tfm)
// {
// 	return aspeed_sham_cra_init_alg(tfm, "sha512");
// }

static void aspeed_sham_cra_exit(struct crypto_tfm *tfm)
{
	struct aspeed_sham_ctx *tctx = crypto_tfm_ctx(tfm);

	AHASH_DBG("\n");

	crypto_free_shash(tctx->fallback);
	dma_free_coherent(tctx->crypto_dev->dev, PAGE_SIZE * 10, tctx->hash_src, tctx->hash_src_dma);

	if (tctx->flags) {
		struct aspeed_sha_hmac_ctx *bctx = tctx->base;
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
				.statesize = sizeof(struct aspeed_sham_reqctx),
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
				.statesize = sizeof(struct aspeed_sham_reqctx),
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
