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

#define ASPEED_CRYPTO_IRQ
//#define ASPEED_CRYPTO_DEBUG
//#define ASPEED_HASH_DEBUG

#define ASPEED_AES_QUEUE_LENGTH	1
#define ASPEED_HASH_BUFF_SIZE 	8192

//#define CDBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)

#ifdef ASPEED_CRYPTO_DEBUG
#define CRYPTO_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#else
#define CRYPTO_DBUG(fmt, args...)
#endif

#ifdef ASPEED_HASH_DEBUG
#define HASH_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#else
#define HASH_DBUG(fmt, args...)
#endif

struct aspeed_crypto_drv aspeed_drv = {
	.dev_list = LIST_HEAD_INIT(aspeed_drv.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(aspeed_drv.lock),
};


/*************************************************************************************/
static u8 AESSBox[256] = {
	0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
	0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
	0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
	0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
	0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
	0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
	0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
	0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
	0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
	0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
	0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
	0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
	0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
	0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
	0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
	0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16
};
static u32 aesRcon[10] = {
	0x01000000, 0x02000000, 0x04000000, 0x08000000,
	0x10000000, 0x20000000, 0x40000000, 0x80000000,
	0x1b000000, 0x36000000
};

u32 AESRotWord(u32 val)
{
	return ((val <<  8)) |
	       ((val >> 24) & 0xff) ;
};
u32 AESSubWord(u32 val)
{
	return (AESSBox[ val        & 0xff]) |
	       (AESSBox[(val >>  8) & 0xff] <<  8) |
	       (AESSBox[(val >> 16) & 0xff] << 16) |
	       (AESSBox[(val >> 24) & 0xff] << 24) ;
};
void AESKeyExpan(unsigned int bits, u32 *aeskeyin, u32 *aeskeydram)
{
	unsigned int aesNk = 0;
	unsigned int aesNr = 0;
	unsigned int i = 0;
	switch (bits) {
	case 128:
		aesNk = 4;
		aesNr = 10;
		break;
	case 192:
		aesNk = 6;
		aesNr = 12;
		break;
	case 256:
		aesNk = 8;
		aesNr = 14;
	}
	for (i = 0; i < aesNk; i++)
		aeskeydram[i] = be32_to_cpu(aeskeyin[i]);

	for (i = aesNk; i < 4 * (aesNr + 1); i++) {
		if (i % aesNk == 0)
			aeskeydram[i] = aeskeydram[i - aesNk] ^ AESSubWord(AESRotWord(aeskeydram[i - 1])) ^ aesRcon[(i / aesNk) - 1];
		else if ((aesNk > 6) && (i % aesNk == 4))
			aeskeydram[i] = aeskeydram[i - aesNk] ^ AESSubWord(aeskeydram[i - 1])                         ;
		else
			aeskeydram[i] = aeskeydram[i - aesNk] ^                       aeskeydram[i - 1]                           ;
	}

	for (i = 0; i < 4 * (aesNr + 1); i++) {
		aeskeydram[i] = be32_to_cpu(aeskeydram[i]);
	}

}
/*************************************************************************************/
static inline void
aspeed_crypto_write(struct aspeed_crypto_dev *crypto, u32 val, u32 reg)
{
//	printk("uart dma write : val: %x , reg : %x \n",val,reg);
	writel(val, crypto->regs + reg);
}

static inline u32
aspeed_crypto_read(struct aspeed_crypto_dev *crypto, u32 reg)
{
#if 0
	u32 val = readl(crypto->regs + reg);
	printk("R : reg %x , val: %x \n", reg, val);
	return val;
#else
	return readl(crypto->regs + reg);
#endif
}

/*************************************************************************************/
static int aspeed_crypto_sg_length(struct ablkcipher_request *req,
				   struct scatterlist *sg)
{
	unsigned int total = req->nbytes;
	int sg_nb;
	unsigned int len;
	struct scatterlist *sg_list;

	sg_nb = 0;
	sg_list = sg;
	total = req->nbytes;

	while (total) {
		len = min(sg_list->length, total);

		sg_nb++;
		total -= len;

		sg_list = sg_next(sg_list);
		if (!sg_list)
			total = 0;
	}

	return sg_nb;
}

/*************************************************************************************/
static int aspeed_crypto_trigger(struct aspeed_crypto_dev *aspeed_crypto)
{
	u32 nbytes = 0;
	unsigned int	nb_in_sg = 0, nb_out_sg = 0;
	struct ablkcipher_request	*req = aspeed_crypto->ablkcipher_req;
//	struct scatterlist	*org_in_sg = req->src, *org_out_sg = req->dst;
	struct scatterlist	*in_sg = req->src, *out_sg = req->dst;

	if (aspeed_crypto->cmd & HACE_CMD_RC4) {
		//RC4
		if (!aspeed_crypto->rc4_ctx->done) {
			*(u32 *)(aspeed_crypto->ctx_buf + 8) = 0x0001;
			memcpy(aspeed_crypto->ctx_buf + 16, aspeed_crypto->rc4_ctx->rc4_key, 256);
			aspeed_crypto->rc4_ctx->done = 1;
		}
	} else {
		if (aspeed_crypto->cmd & HACE_CMD_DES_SELECT) {
			//DES
			if (!aspeed_crypto->des_ctx->done) {
				if (aspeed_crypto->des_ctx->iv) {
					memcpy(aspeed_crypto->ctx_buf + 8, aspeed_crypto->des_ctx->iv, 8);
#if 0
					for (i = 0; i < 8; i++) {
						ctxbuf[i + 8] = aspeed_crypto->des_ctx->iv[i];
					}
#endif
				}
#if 0
				for (i = 0; i < 24; i++) {
					ctxbuf[0x10 + i] = aspeed_crypto->des_ctx->key[i];
				}
#else
				memcpy(aspeed_crypto->ctx_buf + 16, aspeed_crypto->des_ctx->key, 24);
#endif
				aspeed_crypto->des_ctx->done = 1;
			}
		} else {
			//AES
			if (!aspeed_crypto->aes_ctx->done) {

				if (aspeed_crypto->aes_ctx->iv) {
#if 1
					memcpy(aspeed_crypto->ctx_buf, aspeed_crypto->aes_ctx->iv, 16);
#else
					printk("Set iv: \n");
					for (i = 0; i < 0x10; i++) {
						ctxbuf[i] = aspeed_crypto->aes_ctx->iv[i];
						printk("%02x ", ctxbuf[i]);
					}
					printk("\n");
#endif
				}
#if 1
				memcpy(aspeed_crypto->ctx_buf + 16, aspeed_crypto->aes_ctx->key, 0xff);
#else
				printk("Set key: \n");
				for (i = 0; i < 0xff; i++) {
					ctxbuf[0x10 + i] = aspeed_crypto->aes_ctx->key[i];
					printk("%02x ", ctxbuf[0x10 + i]);
				}
				printk("\n");
#endif
				aspeed_crypto->aes_ctx->done = 1;
			}
		}
	}

	nb_in_sg = aspeed_crypto_sg_length(req, in_sg);
	if (!nb_in_sg)
		return -EINVAL;

	nb_out_sg = aspeed_crypto_sg_length(req, out_sg);
	if (!nb_out_sg)
		return -EINVAL;

	if (nb_in_sg != nb_out_sg) {
		printk("ERROR !!!~~~~ \n");
	}
#if 0
	err = dma_map_sg(aspeed_crypto->dev, org_in_sg, nb_in_sg,
			 DMA_TO_DEVICE);
	if (!err) {
		dev_err(aspeed_crypto->dev, "dma_map_sg() error\n");
		return -EINVAL;
	}

	err = dma_map_sg(aspeed_crypto->dev, org_out_sg, nb_out_sg,
			 DMA_FROM_DEVICE);

	if (!err) {
		dev_err(aspeed_crypto->dev, "dma_map_sg() error\n");
		return -EINVAL;
	}

//	printk("req->nbytes %d , nb_in_sg %d, nb_out_sg %d \n", req->nbytes, nb_in_sg, nb_out_sg);

	for (i = 0; i < nb_in_sg; i++)  {
//		printk("sg phy %x ", sg_phys(in_sg));

#ifdef ASPEED_CRYPTO_IRQ
		aspeed_crypto->cmd |= HACE_CMD_ISR_EN;
		printk("aspeed_crypto->cmd %x \n", aspeed_crypto->cmd);
		aspeed_crypto->isr = 0;
		init_completion(&aspeed_crypto->cmd_complete);

#endif

		aspeed_crypto_write(aspeed_crypto, sg_phys(in_sg), ASPEED_HACE_SRC);
		aspeed_crypto_write(aspeed_crypto, sg_phys(out_sg), ASPEED_HACE_DEST);
		aspeed_crypto_write(aspeed_crypto, in_sg->length, ASPEED_HACE_DATA_LEN);
		printk("src : %x , dst : %x , len %d , out len %d \n", sg_phys(in_sg), sg_phys(out_sg), in_sg->length, out_sg->length);

		aspeed_crypto_write(aspeed_crypto, aspeed_crypto->cmd, ASPEED_HACE_CMD);


#ifdef ASPEED_CRYPTO_IRQ
		wait_for_completion_interruptible(&aspeed_crypto->cmd_complete);
		printk("done \n");

#if 0
		if (!(aspeed_crypto->isr & HACE_CRYPTO_ISR)) {
			printk("INTR ERROR aspeed_crypto->isr %x \n", aspeed_crypto->isr);
		}
#endif
#else
		while (aspeed_crypto_read(aspeed_crypto, ASPEED_HACE_STS) & HACE_CRYPTO_BUSY);
#endif

		nbytes +=  in_sg->length;
		in_sg = sg_next(in_sg);
		out_sg = sg_next(out_sg);
	}

	if (nbytes != req->nbytes) {
		printk("~~~ EOOERR  nbytes %d , req->nbytes %d \n", nbytes, req->nbytes);
	}
	dma_unmap_sg(aspeed_crypto->dev, org_in_sg, nb_in_sg, DMA_TO_DEVICE);
	dma_unmap_sg(aspeed_crypto->dev, org_out_sg, nb_out_sg, DMA_FROM_DEVICE);
#else
	nbytes = sg_copy_to_buffer(in_sg, nb_in_sg, 	aspeed_crypto->buf_in, req->nbytes);
//	printk("copy nbytes %d, req->nbytes %d , nb_in_sg %d, nb_out_sg %d \n", nbytes, req->nbytes, nb_in_sg, nb_out_sg);

	if (!nbytes)
		return -EINVAL;

	if (nbytes != req->nbytes) {
		printk("~~~ EOOERR  nbytes %d , req->nbytes %d \n", nbytes, req->nbytes);
	}

#ifdef ASPEED_CRYPTO_IRQ
	aspeed_crypto->cmd |= HACE_CMD_ISR_EN;
	aspeed_crypto->isr = 0;
//	CDBUG("crypto cmd %x\n", aspeed_crypto->cmd);
	init_completion(&aspeed_crypto->cmd_complete);
#endif

	aspeed_crypto_write(aspeed_crypto, aspeed_crypto->dma_addr_in, ASPEED_HACE_SRC);
	aspeed_crypto_write(aspeed_crypto, aspeed_crypto->dma_addr_out, ASPEED_HACE_DEST);
	aspeed_crypto_write(aspeed_crypto, req->nbytes, ASPEED_HACE_DATA_LEN);

	aspeed_crypto_write(aspeed_crypto, aspeed_crypto->cmd, ASPEED_HACE_CMD);

#ifdef ASPEED_CRYPTO_IRQ
	wait_for_completion_interruptible(&aspeed_crypto->cmd_complete);
#if 0
	if (!(aspeed_crypto->isr & HACE_CRYPTO_ISR)) {
		printk("INTR ERROR aspeed_crypto->isr %x \n", aspeed_crypto->isr);
		printk("INTR ERROR aspeed_crypto->isr %x \n", aspeed_crypto->isr);
		printk("INTR ERROR aspeed_crypto->isr %x \n", aspeed_crypto->isr);
		printk("INTR ERROR aspeed_crypto->isr %x \n", aspeed_crypto->isr);
		printk("INTR ERROR aspeed_crypto->isr %x \n", aspeed_crypto->isr);
		printk("INTR ERROR aspeed_crypto->isr %x \n", aspeed_crypto->isr);
		printk("INTR ERROR aspeed_crypto->isr %x \n", aspeed_crypto->isr);
		printk("INTR ERROR aspeed_crypto->isr %x \n", aspeed_crypto->isr);
		printk("INTR ERROR aspeed_crypto->isr %x \n", aspeed_crypto->isr);
	}
	if (aspeed_crypto->isr == 0)
		CDBUG("~~~ aspeed_crypto->isr %x \n", aspeed_crypto->isr);
#endif
#else
	while (aspeed_crypto_read(aspeed_crypto, ASPEED_HACE_STS) & HACE_CRYPTO_BUSY);
#endif

	nbytes = sg_copy_from_buffer(out_sg, nb_out_sg, aspeed_crypto->buf_out, req->nbytes);

//	printk("sg_copy_from_buffer nbytes %d req->nbytes %d\n",nbytes, req->nbytes);

	if (!nbytes) {
		printk("nbytes %d req->nbytes %d\n", nbytes, req->nbytes);
		return -EINVAL;
	}

#endif

	return 0;
}

int aspeed_crypto_handle_queue(struct aspeed_crypto_dev *aspeed_crypto,
				      struct ablkcipher_request *req)
{
	struct crypto_async_request *async_req, *backlog;
//	struct aspeed_crypto_reqctx *rctx;
	unsigned long flags;
	int ret = 0;
	spin_lock_irqsave(&aspeed_crypto->lock, flags);
	if (req)
		ret = ablkcipher_enqueue_request(&aspeed_crypto->queue, req);

	if (aspeed_crypto_read(aspeed_crypto, ASPEED_HACE_STS) & HACE_CRYPTO_BUSY) {
		printk("Engine Busy \n");
		spin_unlock_irqrestore(&aspeed_crypto->lock, flags);
		return ret;
	}
	backlog = crypto_get_backlog(&aspeed_crypto->queue);
	async_req = crypto_dequeue_request(&aspeed_crypto->queue);
	spin_unlock_irqrestore(&aspeed_crypto->lock, flags);

	if (!async_req)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	req = ablkcipher_request_cast(async_req);

	/* assign new request to device */
	aspeed_crypto->ablkcipher_req = req;

	if (aspeed_crypto_trigger(aspeed_crypto) != 0)
		printk("aspeed_crypto_trigger error \n");

	req->base.complete(&req->base, 0);

	return 0;
}

static void aspeed_aes_queue_task(unsigned long data)
{
	struct aspeed_crypto_dev *aspeed_crypto = (struct aspeed_crypto_dev *)data;
	CRYPTO_DBUG("\n");

	aspeed_crypto_handle_queue(aspeed_crypto, NULL);
}

int aspeed_hash_trigger(struct aspeed_crypto_dev *aspeed_crypto)
{
	struct ahash_request *req = aspeed_crypto->ahash_req;
	struct aspeed_sham_reqctx *ctx = ahash_request_ctx(req);

	CRYPTO_DBUG("ctx->bufcnt %d\n", ctx->bufcnt);

#ifdef ASPEED_CRYPTO_IRQ
	aspeed_crypto->cmd |= HASH_CMD_INT_ENABLE;
	aspeed_crypto->isr = 0;
	init_completion(&aspeed_crypto->cmd_complete);
//	CDBUG("hash cmd %x\n", aspeed_crypto->cmd);
#endif

	aspeed_crypto_write(aspeed_crypto, aspeed_crypto->hash_src_dma, ASPEED_HACE_HASH_SRC);
	aspeed_crypto_write(aspeed_crypto, aspeed_crypto->hash_digst_dma, ASPEED_HACE_HASH_DIGEST_BUFF);
	aspeed_crypto_write(aspeed_crypto, aspeed_crypto->hash_key_dma, ASPEED_HACE_HASH_KEY_BUFF);
	aspeed_crypto_write(aspeed_crypto, ctx->bufcnt, ASPEED_HACE_HASH_DATA_LEN);

	aspeed_crypto_write(aspeed_crypto, aspeed_crypto->cmd, ASPEED_HACE_HASH_CMD);


#ifdef ASPEED_CRYPTO_IRQ
	wait_for_completion_interruptible(&aspeed_crypto->cmd_complete);
#if 0
	if (!(aspeed_crypto->isr & HACE_HASH_ISR)) {
		printk("INTR ERROR aspeed_crypto->isr %x \n", aspeed_crypto->isr);
	}
	CDBUG("irq %x\n", aspeed_crypto->isr);
#endif
#else
	while (aspeed_crypto_read(aspeed_crypto, ASPEED_HACE_STS) & HACE_HASH_BUSY);
#endif

#if 0
	printk("digst dma : %x \n", aspeed_crypto->hash_digst_dma);
	for (i = 0; i < ctx->digcnt; i++)
		printk("%02x ", digst[i]);

	printk("\n");
#endif

	memcpy(aspeed_crypto->ahash_req->result, aspeed_crypto->hash_digst, ctx->digcnt);

//	printk("copy to sg done \n");
	return 0;
}

/*************************************************************************************/

int aspeed_hash_handle_queue(struct aspeed_crypto_dev *aspeed_crypto,
				    struct ahash_request *req)
{
	struct crypto_async_request *async_req, *backlog;
	struct aspeed_sham_reqctx *ctx;
	unsigned long flags;
	int ret = 0;

	CRYPTO_DBUG("nbytes : %d \n", req->nbytes);

	spin_lock_irqsave(&aspeed_crypto->lock, flags);
	if (req)
		ret = ahash_enqueue_request(&aspeed_crypto->queue, req);

	if (aspeed_crypto_read(aspeed_crypto, ASPEED_HACE_STS) & HACE_HASH_BUSY) {
		printk("hash Engine Busy \n");
		spin_unlock_irqrestore(&aspeed_crypto->lock, flags);
		return -1;
	}

	backlog = crypto_get_backlog(&aspeed_crypto->queue);
	async_req = crypto_dequeue_request(&aspeed_crypto->queue);
	spin_unlock_irqrestore(&aspeed_crypto->lock, flags);

	if (!async_req)
		return -1;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	req = ahash_request_cast(async_req);
	aspeed_crypto->ahash_req = req;
	ctx = ahash_request_ctx(req);

	aspeed_crypto->cmd = ctx->cmd;

	aspeed_hash_trigger(aspeed_crypto);

	req->base.complete(&req->base, 0);

	return ret;
}

/*************************************************************************************/
#ifdef ASPEED_CRYPTO_IRQ
static irqreturn_t aspeed_crypto_irq(int irq, void *dev)
{
	struct aspeed_crypto_dev *aspeed_crypto = (struct aspeed_crypto_dev *)dev;
	aspeed_crypto->isr = aspeed_crypto_read(aspeed_crypto, ASPEED_HACE_STS);

	CRYPTO_DBUG("sts %x \n", aspeed_crypto->isr);
//	CDBUG("irq %x\n", aspeed_crypto->isr);
	if (aspeed_crypto->isr == 0)
		printk("sts %x \n", aspeed_crypto->isr);
	aspeed_crypto_write(aspeed_crypto, aspeed_crypto->isr, ASPEED_HACE_STS);
	complete(&aspeed_crypto->cmd_complete);
	return IRQ_HANDLED;
}
#endif

static int aspeed_crypto_register(struct aspeed_crypto_dev *crypto_dev)
{
	unsigned int i, k;
	int err = 0;
	//todo ~~
	aspeed_register_crypto_algs(crypto_dev);
	aspeed_register_ahash_algs(crypto_dev);
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
	struct resource *res;
	int i, err, ret = -EINVAL;
	int irq;
	struct aspeed_crypto_dev *crypto_dev;

	crypto_dev = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_crypto_dev), GFP_KERNEL);
	if (!crypto_dev) {
		dev_err(dev, "unable to alloc data struct.\n");
		return -ENOMEM;
	}

	crypto_dev->dev = dev;

	platform_set_drvdata(pdev, crypto_dev);

	INIT_LIST_HEAD(&crypto_dev->list);
	spin_lock_init(&crypto_dev->lock);

	tasklet_init(&crypto_dev->queue_task, aspeed_aes_queue_task,
		     (unsigned long)crypto_dev);

	crypto_init_queue(&crypto_dev->queue, ASPEED_AES_QUEUE_LENGTH);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no MEM resource info\n");
		err = -ENODEV;
	}
	crypto_dev->regs = devm_ioremap_resource(&pdev->dev, res);
	if (!(crypto_dev->regs)) {
		dev_err(dev, "can't ioremap\n");
		return -ENOMEM;
	}
	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no memory/irq resource for crypto_dev\n");
		return -ENXIO;
	}

	crypto_dev->yclk = devm_clk_get(&pdev->dev, "yclk");
	if (IS_ERR(crypto_dev->yclk)) {
		dev_err(&pdev->dev, "no yclk clock defined\n");
		return -ENODEV;
	}

	clk_prepare_enable(crypto_dev->yclk);

#ifdef ASPEED_CRYPTO_IRQ
	if (devm_request_irq(&pdev->dev, irq, aspeed_crypto_irq, 0, dev_name(&pdev->dev), crypto_dev)) {
		dev_err(dev, "unable to request aes irq.\n");
		return -EBUSY;
	}
#endif

	spin_lock(&aspeed_drv.lock);
	list_add_tail(&crypto_dev->list, &aspeed_drv.dev_list);
	spin_unlock(&aspeed_drv.lock);

	// 8-byte aligned
	crypto_dev->ctx_buf = dma_alloc_coherent(&pdev->dev,
			      0x8000,
			      &crypto_dev->ctx_dma_addr, GFP_KERNEL);

	if (! crypto_dev->ctx_buf) {
		printk("error buff allocation\n");
		return -ENOMEM;
	}
	crypto_dev->buf_size = 0x1000;
	crypto_dev->buf_in = crypto_dev->ctx_buf + 0x1000;
	crypto_dev->dma_addr_in = crypto_dev->ctx_dma_addr + 0x1000;

	crypto_dev->buf_out = crypto_dev->buf_in + 0x1000;
	crypto_dev->dma_addr_out = crypto_dev->dma_addr_in + 0x1000;

	crypto_dev->hash_key = crypto_dev->buf_out + 0x1000;
	crypto_dev->hash_key_dma = crypto_dev->dma_addr_out + 0x1000;

	crypto_dev->hash_src = crypto_dev->hash_key + 0x1000;
	crypto_dev->hash_src_dma = crypto_dev->hash_key_dma + 0x1000;

	crypto_dev->hash_digst = crypto_dev->hash_src + ASPEED_HASH_BUFF_SIZE;
	crypto_dev->hash_digst_dma = crypto_dev->hash_src_dma + ASPEED_HASH_BUFF_SIZE;

//	printk("Crypto ctx %x , in : %x, out: %x\n", crypto_dev->ctx_dma_addr, crypto_dev->dma_addr_in, crypto_dev->dma_addr_out);

//	printk("Hash key %x , src : %x, digst: %x\n", crypto_dev->hash_key_dma, crypto_dev->hash_src_dma, crypto_dev->hash_digst_dma);

	///Ctrl init
	aspeed_crypto_write(crypto_dev, crypto_dev->ctx_dma_addr, ASPEED_HACE_CONTEXT);

	err = aspeed_crypto_register(crypto_dev);
	if (err) {
		dev_err(dev, "err in register alg");
		return err;
	}

	dev_info(dev, "ASPEED Crypto Accelerator successfully registered\n");

	return 0;
}

static int aspeed_crypto_remove(struct platform_device *pdev)
{
	struct aspeed_crypto_dev *crypto_dev = platform_get_drvdata(pdev);

	//aspeed_crypto_unregister();
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
