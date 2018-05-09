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

#ifdef ASPEED_CIPHER_DEBUG
#define CIPHER_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#else
#define CIPHER_DBG(fmt, args...)
#endif

static int aspeed_rc4_crypt(struct ablkcipher_request *req, u32 cmd)
{
	struct aspeed_rc4_ctx *ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
//	struct aspeed_crypto_reqctx *rctx = ablkcipher_request_ctx(req);
	struct aspeed_crypto_dev *aspeed_crypto = NULL, *tmp;

	CIPHER_DBG("\n");

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


	cmd |= HACE_CMD_RI_WO_DATA_ENABLE |
	       HACE_CMD_CONTEXT_LOAD_ENABLE | HACE_CMD_CONTEXT_SAVE_ENABLE;

	aspeed_crypto->cmd = cmd;
	aspeed_crypto->rc4_ctx = ctx;

	return aspeed_crypto_handle_queue(aspeed_crypto, req);
}

static int aspeed_rc4_setkey(struct crypto_ablkcipher *tfm, const u8 *in_key,
			     unsigned int key_len)

{
	int i, j = 0, k = 0;
	struct aspeed_rc4_ctx *ctx = crypto_ablkcipher_ctx(tfm);
	CIPHER_DBG("keylen : %d \n", key_len);

	for (i = 0; i < 256; i++)
		ctx->rc4_key[i] = i;

	for (i = 0; i < 256; i++) {
		u32 a = ctx->rc4_key[i];
		j = (j + in_key[k] + a) & 0xff;
		ctx->rc4_key[i] = ctx->rc4_key[j];
		ctx->rc4_key[j] = a;
		if (++k >= key_len)
			k = 0;
	}

	ctx->key_len = 256;


	return 0;
}

static int aspeed_rc4_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_rc4_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_RC4);
}

static int aspeed_rc4_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_rc4_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_RC4);
}

static int aspeed_des_crypt(struct ablkcipher_request *req, u32 cmd)
{
	struct aspeed_des_ctx *ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
//	struct aspeed_crypto_reqctx *rctx = ablkcipher_request_ctx(req);
	struct aspeed_crypto_dev *aspeed_crypto = NULL, *tmp;

	CIPHER_DBG("\n");

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

	ctx->iv = req->info;

	cmd |= HACE_CMD_DES_SELECT | HACE_CMD_RI_WO_DATA_ENABLE |
	       HACE_CMD_CONTEXT_LOAD_ENABLE | HACE_CMD_CONTEXT_SAVE_ENABLE;

	aspeed_crypto->cmd = cmd;
	aspeed_crypto->des_ctx = ctx;

	return aspeed_crypto_handle_queue(aspeed_crypto, req);
}

static int aspeed_des_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
			     unsigned int keylen)
{
//	struct crypto_tfm *ctfm = crypto_ablkcipher_tfm(tfm);
	struct aspeed_aes_ctx *ctx = crypto_ablkcipher_ctx(tfm);

	CIPHER_DBG("bits : %d \n", (keylen * 8));

	if ((keylen != DES_KEY_SIZE) && (keylen != 2 * DES_KEY_SIZE) && (keylen != 3 * DES_KEY_SIZE)) {
		crypto_ablkcipher_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	memcpy(ctx->key, key, keylen);
	ctx->key_len = keylen;


	return 0;
}

static int aspeed_tdes_ctr_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CTR | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_ctr_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CTR | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_ofb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_OFB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_ofb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_OFB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_cfb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CFB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_cfb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CFB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_cbc_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CBC | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_cbc_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CBC | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_ecb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_ECB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_ecb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_ECB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ctr_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CTR | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ctr_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CTR | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ofb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_OFB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ofb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_OFB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_cfb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CFB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_cfb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CFB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_cbc_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CBC | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_cbc_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CBC | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ecb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_ECB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ecb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_ECB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}
/* AES *************************************************************************************/
static int aspeed_aes_crypt(struct ablkcipher_request *areq, u32 cmd)
{
	struct aspeed_aes_ctx *ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(areq));
//	struct aspeed_crypto_reqctx *rctx = ablkcipher_request_ctx(areq);
	struct aspeed_crypto_dev *aspeed_crypto = NULL, *tmp;

	CIPHER_DBG("\n");

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

	ctx->iv = areq->info;

	cmd |= HACE_CMD_AES_SELECT | HACE_CMD_RI_WO_DATA_ENABLE |
	       HACE_CMD_CONTEXT_LOAD_ENABLE | HACE_CMD_CONTEXT_SAVE_ENABLE;

	switch (ctx->key_len) {
	case AES_KEYSIZE_128:
		cmd |= HACE_CMD_AES128;
		break;
	case AES_KEYSIZE_192:
		cmd |= HACE_CMD_AES192;
		break;
	case AES_KEYSIZE_256:
		cmd |= HACE_CMD_AES256;
		break;
	}

	aspeed_crypto->cmd = cmd;

	aspeed_crypto->aes_ctx = ctx;

	return aspeed_crypto_handle_queue(aspeed_crypto, areq);

}

static int aspeed_aes_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
			     unsigned int keylen)
{
	struct aspeed_aes_ctx *ctx = crypto_ablkcipher_ctx(tfm);
	CIPHER_DBG("bits : %d \n", (keylen * 8));

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 &&
	    keylen != AES_KEYSIZE_256) {
		crypto_ablkcipher_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	crypto_aes_expand_key(&ctx->crypto_dev->aes_ctx, key, keylen);
//	AESKeyExpan(keylen * 8, (u32 *) key, (u32 *)ctx->key);
	ctx->key_len = keylen;

	return 0;
}

static int aspeed_aes_ctr_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CTR);
}

static int aspeed_aes_ctr_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CTR);
}

static int aspeed_aes_ofb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_OFB);
}

static int aspeed_aes_ofb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_OFB);
}

static int aspeed_aes_cfb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CFB);
}

static int aspeed_aes_cfb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CFB);
}

static int aspeed_aes_ecb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_ECB);
}

static int aspeed_aes_ecb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_ECB);
}

static int aspeed_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CBC);
}

static int aspeed_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CBC);
}

static int aspeed_crypto_cra_init(struct crypto_tfm *tfm)
{
	CIPHER_DBG("\n");

	tfm->crt_ablkcipher.reqsize = sizeof(struct aspeed_crypto_reqctx);
	return 0;
}

static void aspeed_crypto_cra_exit(struct crypto_tfm *tfm)
{
	CIPHER_DBG("\n");

}

struct crypto_alg aspeed_crypto_algs[] = {
	{
		.cra_name 		= "ecb(aes)",
		.cra_driver_name 	= "aspeed-ecb-aes",
		.cra_priority 		= 300,
		.cra_flags 		= CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_ASYNC,
		.cra_blocksize 	= AES_BLOCK_SIZE,
		.cra_ctxsize 		= sizeof(struct aspeed_aes_ctx),
		.cra_alignmask	= 0x0f,
		.cra_type 		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init 		= aspeed_crypto_cra_init,
		.cra_exit 		= aspeed_crypto_cra_exit,
		.cra_u.ablkcipher = {
			//no iv
			.min_keysize 	= AES_MIN_KEY_SIZE,
			.max_keysize 	= AES_MAX_KEY_SIZE,
			.setkey 		= aspeed_aes_setkey,
			.encrypt 	= aspeed_aes_ecb_encrypt,
			.decrypt	= aspeed_aes_ecb_decrypt,
		},
	},
	{
		.cra_name 		= "cbc(aes)",
		.cra_driver_name 	= "aspeed-cbc-aes",
		.cra_priority 		= 300,
		.cra_flags 		= CRYPTO_ALG_TYPE_ABLKCIPHER |
				CRYPTO_ALG_ASYNC,
		.cra_blocksize 	= AES_BLOCK_SIZE,
		.cra_ctxsize 		= sizeof(struct aspeed_aes_ctx),
		.cra_alignmask	= 0xf,
		.cra_type 		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init 		= aspeed_crypto_cra_init,
		.cra_exit 		= aspeed_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= AES_BLOCK_SIZE,
			.min_keysize 	= AES_MIN_KEY_SIZE,
			.max_keysize 	= AES_MAX_KEY_SIZE,
			.setkey 		= aspeed_aes_setkey,
			.encrypt 	= aspeed_aes_cbc_encrypt,
			.decrypt	= aspeed_aes_cbc_decrypt,
		},
	},
	{
		.cra_name		= "cfb(aes)",
		.cra_driver_name	= "aspeed-cfb-aes",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
		CRYPTO_ALG_ASYNC,
		.cra_blocksize	= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct aspeed_aes_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= aspeed_crypto_cra_init,
		.cra_exit		= aspeed_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= AES_BLOCK_SIZE,
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.setkey 		= aspeed_aes_setkey,
			.encrypt	= aspeed_aes_cfb_encrypt,
			.decrypt	= aspeed_aes_cfb_decrypt,
		},
	},
	{
		.cra_name		= "ofb(aes)",
		.cra_driver_name	= "aspeed-ofb-aes",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
		CRYPTO_ALG_ASYNC,
		.cra_blocksize	= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct aspeed_aes_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= aspeed_crypto_cra_init,
		.cra_exit		= aspeed_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= AES_BLOCK_SIZE,
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.setkey 		= aspeed_aes_setkey,
			.encrypt	= aspeed_aes_ofb_encrypt,
			.decrypt	= aspeed_aes_ofb_decrypt,
		},
	},
	{
		.cra_name		= "ctr(aes)",
		.cra_driver_name	= "aspeed-ctr-aes",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
		CRYPTO_ALG_ASYNC,
		.cra_blocksize	= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct aspeed_aes_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= aspeed_crypto_cra_init,
		.cra_exit		= aspeed_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= AES_BLOCK_SIZE,
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.setkey 		= aspeed_aes_setkey,
			.encrypt	= aspeed_aes_ctr_encrypt,
			.decrypt	= aspeed_aes_ctr_decrypt,
		},
	},
	{
		.cra_name		= "ecb(des)",
		.cra_driver_name	= "aspeed-ecb-des",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
		CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct aspeed_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= aspeed_crypto_cra_init,
		.cra_exit		= aspeed_crypto_cra_exit,
		.cra_u.ablkcipher = {
			//no iv
			.min_keysize	= DES_KEY_SIZE,
			.max_keysize	= DES_KEY_SIZE,
			.setkey		= aspeed_des_setkey,
			.encrypt	= aspeed_des_ecb_encrypt,
			.decrypt	= aspeed_des_ecb_decrypt,
		},
	},
	{
		.cra_name		= "cbc(des)",
		.cra_driver_name	= "aspeed-cbc-des",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
		CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct aspeed_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= aspeed_crypto_cra_init,
		.cra_exit		= aspeed_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= DES_KEY_SIZE,
			.max_keysize	= DES_KEY_SIZE,
			.setkey 	= aspeed_des_setkey,
			.encrypt	= aspeed_des_cbc_encrypt,
			.decrypt	= aspeed_des_cbc_decrypt,
		},
	},
	{
		.cra_name		= "cfb(des)",
		.cra_driver_name	= "aspeed-cfb-des",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
		CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct aspeed_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= aspeed_crypto_cra_init,
		.cra_exit		= aspeed_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= DES_KEY_SIZE,
			.max_keysize	= DES_KEY_SIZE,
			.setkey 	= aspeed_des_setkey,
			.encrypt	= aspeed_des_cfb_encrypt,
			.decrypt	= aspeed_des_cfb_decrypt,
		},
	},
	{
		.cra_name		= "ofb(des)",
		.cra_driver_name	= "aspeed-ofb-des",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
				CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct aspeed_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= aspeed_crypto_cra_init,
		.cra_exit		= aspeed_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= 2 * DES_KEY_SIZE,
			.max_keysize	= 3 * DES_KEY_SIZE,
			.setkey 	= aspeed_des_setkey,
			.encrypt	= aspeed_des_ofb_encrypt,
			.decrypt	= aspeed_des_ofb_decrypt,
		},
	},
	{
		.cra_name		= "ctr(des)",
		.cra_driver_name	= "aspeed-ctr-des",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
				CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct aspeed_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= aspeed_crypto_cra_init,
		.cra_exit		= aspeed_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= 2 * DES_KEY_SIZE,
			.max_keysize	= 3 * DES_KEY_SIZE,
			.setkey 	= aspeed_des_setkey,
			.encrypt	= aspeed_des_ctr_encrypt,
			.decrypt	= aspeed_des_ctr_decrypt,
		},
	},
	{
		.cra_name		= "ecb(des3_ede)",
		.cra_driver_name	= "aspeed-ecb-tdes",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
		CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct aspeed_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= aspeed_crypto_cra_init,
		.cra_exit		= aspeed_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= 2 * DES_KEY_SIZE,
			.max_keysize	= 3 * DES_KEY_SIZE,
			.setkey		= aspeed_des_setkey,
			.encrypt	= aspeed_tdes_ecb_encrypt,
			.decrypt	= aspeed_tdes_ecb_decrypt,
		},
	},
	{
		.cra_name		= "cbc(des3_ede)",
		.cra_driver_name	= "aspeed-cbc-tdes",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
		CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct aspeed_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= aspeed_crypto_cra_init,
		.cra_exit		= aspeed_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= 2 * DES_KEY_SIZE,
			.max_keysize	= 3 * DES_KEY_SIZE,
			.setkey 	= aspeed_des_setkey,
			.encrypt	= aspeed_tdes_cbc_encrypt,
			.decrypt	= aspeed_tdes_cbc_decrypt,
		},
	},
	{
		.cra_name		= "cfb(des3_ede)",
		.cra_driver_name	= "aspeed-cfb-tdes",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
		CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct aspeed_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= aspeed_crypto_cra_init,
		.cra_exit		= aspeed_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= 2 * DES_KEY_SIZE,
			.max_keysize	= 3 * DES_KEY_SIZE,
			.setkey 	= aspeed_des_setkey,
			.encrypt	= aspeed_tdes_cfb_encrypt,
			.decrypt	= aspeed_tdes_cfb_decrypt,
		},
	},
	{
		.cra_name		= "ofb(des3_ede)",
		.cra_driver_name	= "aspeed-ofb-tdes",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
		CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct aspeed_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= aspeed_crypto_cra_init,
		.cra_exit		= aspeed_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= 2 * DES_KEY_SIZE,
			.max_keysize	= 3 * DES_KEY_SIZE,
			.setkey 	= aspeed_des_setkey,
			.encrypt	= aspeed_tdes_ofb_encrypt,
			.decrypt	= aspeed_tdes_ofb_decrypt,
		},
	},
	{
		.cra_name		= "ctr(des3_ede)",
		.cra_driver_name	= "aspeed-ctr-tdes",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
		CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct aspeed_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= aspeed_crypto_cra_init,
		.cra_exit		= aspeed_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= 2 * DES_KEY_SIZE,
			.max_keysize	= 3 * DES_KEY_SIZE,
			.setkey 	= aspeed_des_setkey,
			.encrypt	= aspeed_tdes_ctr_encrypt,
			.decrypt	= aspeed_tdes_ctr_decrypt,
		},
	},
	{
		.cra_name		= "ecb(arc4)",
		.cra_driver_name	= "aspeed-ecb-arc4",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
		CRYPTO_ALG_ASYNC,
		.cra_blocksize		= 1,
		.cra_ctxsize		= sizeof(struct aspeed_rc4_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= aspeed_crypto_cra_init,
		.cra_exit		= aspeed_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= 1,
			.max_keysize	= 256,
			.setkey 	= aspeed_rc4_setkey,
			.encrypt	= aspeed_rc4_encrypt,
			.decrypt	= aspeed_rc4_decrypt,
		},
	},
};

int aspeed_register_crypto_algs(struct aspeed_crypto_dev *crypto_dev)
{
	int i;
	int err = 0;
	crypto_dev->crypto_algs		= (struct crypto_alg *)&aspeed_crypto_algs;
	for (i = 0; i < ARRAY_SIZE(aspeed_crypto_algs); i++) {
		err = crypto_register_alg(&aspeed_crypto_algs[i]);
		if (err)
			printk("aspeed_crypto_algs ~~~ ERROR ~~~\n");
	}
}
