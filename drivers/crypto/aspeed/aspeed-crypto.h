#ifndef __ASPEED_CRYPTO_H__
#define __ASPEED_CRYPTO_H__

#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/algapi.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <crypto/internal/hash.h>

#include <crypto/md5.h>
#include <crypto/sha.h>

#define _SBF(v, f)			((v) << (f))

/* Crypto control registers*/
#define ASPEED_HACE_SRC			0x00
#define ASPEED_HACE_DEST		0x04
#define ASPEED_HACE_CONTEXT		0x08	/* 8 byte aligned*/
#define ASPEED_HACE_DATA_LEN		0x0C
#define ASPEED_HACE_CMD			0x10
#define  HACE_CMD_SINGLE_DES	0
#define  HACE_CMD_TRIPLE_DES	BIT(17)
#define  HACE_CMD_AES_SELECT	0
#define  HACE_CMD_DES_SELECT	BIT(16)
#define  HACE_CMD_ISR_EN		BIT(12)
#define  HACE_CMD_RI_WO_DATA_ENABLE		(0)
#define  HACE_CMD_RI_WO_DATA_DISABLE	BIT(11)
#define  HACE_CMD_CONTEXT_LOAD_ENABLE	(0)
#define  HACE_CMD_CONTEXT_LOAD_DISABLE	BIT(10)
#define  HACE_CMD_CONTEXT_SAVE_ENABLE	(0)
#define  HACE_CMD_CONTEXT_SAVE_DISABLE	BIT(9)
#define  HACE_CMD_AES					(0)
#define  HACE_CMD_DES					(0)
#define  HACE_CMD_RC4					BIT(8)
#define  HACE_CMD_DECRYPT				(0)
#define  HACE_CMD_ENCRYPT				BIT(7)
#define  HACE_CMD_ECB					(0)
#define  HACE_CMD_CBC					(0x1 << 4)
#define  HACE_CMD_CFB					(0x1 << 5)
#define  HACE_CMD_OFB					(0x3 << 4)
#define  HACE_CMD_CTR					(0x1 << 6)
#define  HACE_CMD_AES128				(0)
#define  HACE_CMD_AES192				(0x1 << 2)
#define  HACE_CMD_AES256				(0x1 << 3)
#define  HACE_CMD_OP_CASCADE			(0x3)
#define  HACE_CMD_OP_INDEPENDENT		(0x1)
#define ASPEED_HACE_TAG			0x18
#define ASPEED_HACE_STS			0x1C
#define  HACE_RSA_ISR			BIT(13)
#define  HACE_CRYPTO_ISR		BIT(12)
#define  HACE_HASH_ISR			BIT(9)

#define  HACE_RSA_BUSY			BIT(2)
#define  HACE_CRYPTO_BUSY		BIT(1)
#define  HACE_HASH_BUSY			BIT(0)
#define ASPEED_HACE_HASH_SRC		0x20
#define ASPEED_HACE_HASH_DIGEST_BUFF	0x24
#define ASPEED_HACE_HASH_KEY_BUFF	0x28	/* 64 byte aligned*/
#define ASPEED_HACE_HASH_DATA_LEN	0x2C
#define ASPEED_HACE_HASH_CMD		0x30
#define  HASH_CMD_INT_ENABLE		BIT(9)
#define  HASH_CMD_INT_DISABLE		(0)
#define  HASH_CMD_HMAC				BIT(7)
#define  HASH_CMD_MD5				(0)
#define  HASH_CMD_SHA1				(0x2 << 4)
#define  HASH_CMD_SHA224			(0x4 << 4)
#define  HASH_CMD_SHA256			(0x5 << 4)
#define  HASH_CMD_MD5_SWAP				(0x1 << 2)
#define  HASH_CMD_SHA_SWAP				(0x1 << 3)
#define  HASH_CMD_CASCADED_CRYPTO_FIRST	(2)
#define  HASH_CMD_CASCADED_HASH_FIRST	(3)
#define ASPEED_HACE_RSA_MD_EXP_BIT	0x40
#define ASPEED_HACE_RSA_CMD		0x4C
#define ASPEED_HACE_CMD_QUEUE		0x50
#define ASPEED_HACE_CMD_QUEUE_EP	0x54
#define ASPEED_HACE_CMD_QUEUE_WP	0x58
#define ASPEED_HACE_CMD_QUEUE_RP	0x5C
#define ASPEED_HACE_ENG_FEATURE		0x60

/*
 * Asynchronous crypto request structure.
 *
 * This structure defines a request that is either queued for processing or
 * being processed.
 */

struct aspeed_crypto_reqctx {
	unsigned long mode;
};

struct aspeed_crypto_dev {
	void __iomem			*regs;
	struct list_head		list;

	struct completion		cmd_complete;
	u32 					isr;
	struct clk 			*yclk;

	spinlock_t			lock;

	//crypto
	struct aspeed_aes_ctx	*aes_ctx;
	struct aspeed_des_ctx	*des_ctx;
	struct aspeed_rc4_ctx		*rc4_ctx;

	//hash
	struct aspeed_sham_reqctx *sham_reqctx;

	struct crypto_queue	queue;

	struct tasklet_struct	queue_task;

	unsigned long			flags;

	struct ablkcipher_request	*ablkcipher_req;
	struct ahash_request		*ahash_req;

	size_t	total;


	struct crypto_alg		*crypto_algs;
	struct ahash_alg		*ahash_algs;
//	struct akcipher_alg 	*akcipher_alg;	//for rsa 

	struct device			*dev;

	u32 		buf_size;
	void	*ctx_buf;
	dma_addr_t	ctx_dma_addr;

	void	*buf_in;
	dma_addr_t	dma_addr_in;

	void	*buf_out;
	dma_addr_t	dma_addr_out;

	/* hash */
	void	*hash_key;
	dma_addr_t	hash_key_dma;

	void	*hash_src;
	dma_addr_t	hash_src_dma;

	void	*hash_digst;
	dma_addr_t	hash_digst_dma;

	u32 		cmd;
};
/*************************************************************************************/

/* Algorithm type mask. */
#define SPACC_CRYPTO_ALG_MASK		0x7

/* Block cipher context. */
struct aspeed_aes_ctx {
	struct aspeed_crypto_dev		*crypto_dev;
	u8				*iv;
	u8				key[0xff];
	u8				key_len;
	u16				block_size;
	u8				done;
};

struct aspeed_des_ctx {
	struct aspeed_crypto_dev		*crypto_dev;
	u8				*iv;
	u8				key[0x24];
	u8				key_len;
	u8				done;
};

struct aspeed_rc4_ctx {
	struct aspeed_crypto_dev		*crypto_dev;
	u8				rc4_key[256];
	int				key_len;
	u8				done;
};

struct aspeed_cipher_context {
	struct aspeed_crypto_dev		*crypto_dev;
	int			key_len;
	int			enc_type;
	union {
		u8		aspeed_aes[AES_MAX_KEY_SIZE];
		u8		aspeed_des[DES_KEY_SIZE];
		u8		aspeed_des3[3 * DES_KEY_SIZE];
		u8		aspeed_arc4[258]; /* S-box, X, Y */
	} key;
	u8				done;
};

struct aspeed_sham_ctx {
	struct aspeed_crypto_dev	*crypto_dev;

	unsigned long		flags;	//hmac flag

	/* fallback stuff */
	struct crypto_shash	*fallback;
	struct crypto_shash 	*base_hash;		//for hmac
};

struct aspeed_sham_reqctx {
	struct aspeed_crypto_dev	*crypto_dev;
	unsigned long	flags;	//final update flag should no use
	u8			op; 	  	////0: init, 1 : upate , 2: final update

	u32			cmd;

	u8	digest[SHA256_DIGEST_SIZE] __aligned(sizeof(u32));

	size_t			digcnt;

	size_t			bufcnt;

	/* walk state */
	struct scatterlist	*sg;
	unsigned int		offset;	/* offset in current sg */
	unsigned int		total;	/* total request */

	size_t 		block_size;

	u8	buffer[0] __aligned(sizeof(u32));
};

struct aspeed_crypto_drv {
	struct list_head	dev_list;
	spinlock_t		lock;
};

extern struct aspeed_crypto_drv aspeed_drv;

extern int aspeed_hash_trigger(struct aspeed_crypto_dev *aspeed_crypto);
extern int aspeed_hash_handle_queue(struct aspeed_crypto_dev *aspeed_crypto, struct ahash_request *req);

extern int aspeed_register_crypto_algs(struct aspeed_crypto_dev *crypto_dev);
extern int aspeed_register_ahash_algs(struct aspeed_crypto_dev *crypto_dev);

extern int aspeed_crypto_handle_queue(struct aspeed_crypto_dev *aspeed_crypto, struct ablkcipher_request *req);

#endif
