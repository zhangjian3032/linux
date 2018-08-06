#ifndef __ASPEED_CRYPTO_H__
#define __ASPEED_CRYPTO_H__

#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/fips.h>
#include <linux/dma-mapping.h>
#include <crypto/scatterwalk.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/kpp.h>
#include <crypto/internal/rsa.h>
#include <crypto/internal/akcipher.h>
#include <crypto/kpp.h>
#include <crypto/dh.h>
#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/algapi.h>
#include <crypto/akcipher.h>
#include <crypto/md5.h>
#include <crypto/sha.h>
#include <crypto/ecdh.h>



/* Crypto control registers*/
#define ASPEED_HACE_SRC			0x00
#define ASPEED_HACE_DEST		0x04
#define ASPEED_HACE_CONTEXT		0x08	/* 8 byte aligned*/
#define ASPEED_HACE_DATA_LEN		0x0C
#define ASPEED_HACE_CMD			0x10
#define  HACE_CMD_SINGLE_DES		0
#define  HACE_CMD_TRIPLE_DES		BIT(17)
#define  HACE_CMD_AES_SELECT		0
#define  HACE_CMD_DES_SELECT		BIT(16)
#define  HACE_CMD_ISR_EN		BIT(12)
#define  HACE_CMD_RI_WO_DATA_ENABLE	(0)
#define  HACE_CMD_RI_WO_DATA_DISABLE	BIT(11)
#define  HACE_CMD_CONTEXT_LOAD_ENABLE	(0)
#define  HACE_CMD_CONTEXT_LOAD_DISABLE	BIT(10)
#define  HACE_CMD_CONTEXT_SAVE_ENABLE	(0)
#define  HACE_CMD_CONTEXT_SAVE_DISABLE	BIT(9)
#define  HACE_CMD_AES			(0)
#define  HACE_CMD_DES			(0)
#define  HACE_CMD_RC4			BIT(8)
#define  HACE_CMD_DECRYPT		(0)
#define  HACE_CMD_ENCRYPT		BIT(7)
#define  HACE_CMD_ECB			(0)
#define  HACE_CMD_CBC			BIT(4)
#define  HACE_CMD_CFB			BIT(5)
#define  HACE_CMD_OFB			(0x3 << 4)
#define  HACE_CMD_CTR			BIT(6)
#define  HACE_CMD_IV_REQUIRE		(HACE_CMD_CBC | HACE_CMD_CFB | \
					 HACE_CMD_OFB | HACE_CMD_CTR)
#define  HACE_CMD_AES128		(0)
#define  HACE_CMD_AES192		BIT(2)
#define  HACE_CMD_AES256		BIT(3)
#define  HACE_CMD_OP_CASCADE		(0x3)
#define  HACE_CMD_OP_INDEPENDENT	(0x1)
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
#define  HASH_CMD_HMAC			BIT(7)
#define  HASH_CMD_MD5			(0)
#define  HASH_CMD_SHA1			(0x2 << 4)
#define  HASH_CMD_SHA224		(0x4 << 4)
#define  HASH_CMD_SHA256		(0x5 << 4)
#define  HASH_CMD_MD5_SWAP		(0x1 << 2)
#define  HASH_CMD_SHA_SWAP		(0x1 << 3)
#define  HASH_CMD_CASCADED_CRYPTO_FIRST	(2)
#define  HASH_CMD_CASCADED_HASH_FIRST	(3)
#define ASPEED_HACE_RSA_MD_EXP_BIT	0x40
#define ASPEED_HACE_RSA_CMD		0x4C
#define  RSA_CMD_INT_ENABLE		BIT(13)
#define  RSA_CMD_SRAM_ENGINE_ACCESSABLE BIT(12)
#define  RSA_CMD_FIRE			BIT(11)
#define ASPEED_HACE_CMD_QUEUE		0x50
#define ASPEED_HACE_CMD_QUEUE_EP	0x54
#define ASPEED_HACE_CMD_QUEUE_WP	0x58
#define ASPEED_HACE_CMD_QUEUE_RP	0x5C
#define ASPEED_HACE_ENG_FEATURE		0x60

#define ASPEED_CRYPTO_G6		BIT(0)
#define ASPEED_CRYPTO_G6_RSA_BUFF_SIZE	508
#define ASPEED_CRYPTO_RSA_BUFF_SIZE	508

#define ASPEED_EUCLID_CTX_LEN		13312
#define ASPEED_EUCLID_LEN		1024
#define ASPEED_EUCLID_A			0
#define ASPEED_EUCLID_B			ASPEED_EUCLID_LEN * 1
#define ASPEED_EUCLID_Q			ASPEED_EUCLID_LEN * 2
#define ASPEED_EUCLID_R			ASPEED_EUCLID_LEN * 3
#define ASPEED_EUCLID_X			ASPEED_EUCLID_LEN * 4
#define ASPEED_EUCLID_Y			ASPEED_EUCLID_LEN * 5
#define ASPEED_EUCLID_LX		ASPEED_EUCLID_LEN * 6
#define ASPEED_EUCLID_LY		ASPEED_EUCLID_LEN * 7
#define ASPEED_EUCLID_T			ASPEED_EUCLID_LEN * 8
#define ASPEED_EUCLID_D1		ASPEED_EUCLID_LEN * 9
#define ASPEED_EUCLID_S			ASPEED_EUCLID_LEN * 10
#define ASPEED_EUCLID_N			ASPEED_EUCLID_LEN * 11
#define ASPEED_EUCLID_NP		ASPEED_EUCLID_LEN * 12

#define CRYPTO_FLAGS_BUSY 		BIT(1)

#define SHA_BUFFER_LEN		(PAGE_SIZE / 16)

#define OP_INIT				1
#define OP_UPDATE			2
#define OP_FINAL			3

#define SHA_FLAGS_SINGLE_UPDATE		BIT(1)
#define SHA_FLAGS_N_UPDATES		BIT(2)
#define SHA_FLAGS_SINGLE_SG		BIT(3)
#define SHA_FLAGS_N_SG			BIT(4)
#define SHA_FLAGS_CPU			BIT(5)

/*
 * Asynchronous crypto request structure.
 *
 * This structure defines a request that is either queued for processing or
 * being processed.
 */

struct aspeed_cipher_ctx {
	struct aspeed_crypto_dev	*crypto_dev;
	u8				*iv;
	bool				con;
	int 				key_len;
	int 				enc_cmd;
	void				*cipher_key;
	dma_addr_t			cipher_key_dma;
};

typedef int (*aspeed_crypto_fn_t)(struct aspeed_crypto_dev *);

struct aspeed_crypto_dev {
	void __iomem			*regs;
	void __iomem			*rsa_buff;
	struct device			*dev;
	int 				irq;
	struct clk			*yclk;
	struct clk			*rsaclk;
	spinlock_t			lock;

	bool				is_async;
	aspeed_crypto_fn_t		resume;
	//hash
	struct crypto_queue		queue;

	struct tasklet_struct		done_task;
	struct tasklet_struct		queue_task;

	unsigned long			flags;
	unsigned long			version;
	unsigned long			rsa_max_buf_len;

	size_t	total;

	struct ablkcipher_request	*ablk_req;
	struct ahash_request		*ahash_req;
	struct akcipher_request		*akcipher_req;

	/* ablkcipher */
	void				*cipher_addr;
	dma_addr_t			cipher_dma_addr;
};

struct aspeed_crypto_alg {
	struct aspeed_crypto_dev	*crypto_dev;
	union {
		struct crypto_alg	crypto;
		struct ahash_alg	ahash;
		struct kpp_alg 		kpp;
		struct akcipher_alg 	akcipher;
	} alg;
};

/*************************************************************************************/
/* the privete variable of hash for fallback */
struct aspeed_sha_hmac_ctx {
	struct crypto_shash *shash;
	u8 ipad[SHA512_BLOCK_SIZE] __attribute__((aligned(sizeof(u32))));
	u8 opad[SHA512_BLOCK_SIZE] __attribute__((aligned(sizeof(u32))));
};

struct aspeed_sham_ctx {
	struct aspeed_crypto_dev	*crypto_dev;
	unsigned long			flags; //hmac flag

	void				*hash_digst; //8byte align
	dma_addr_t			hash_digst_dma;

	void				*hmac_key; //64byte align
	dma_addr_t			hmac_key_dma;

	void				*hash_src;
	dma_addr_t			hash_src_dma;
	/* fallback stuff */
	struct crypto_shash		*fallback;
	struct aspeed_sha_hmac_ctx	base[0];		//for hmac
};


struct aspeed_sham_reqctx {
	unsigned long		flags;	//final update flag should no use
	unsigned long		op;	//final update flag should no use
	u32			cmd;	//trigger cmd

	u8	digest[SHA512_DIGEST_SIZE] __aligned(sizeof(u64));  //digest result
	u64	digcnt;  //total length
	size_t	digsize; //digest size
	size_t	bufcnt;  //buffer counter
	size_t	buflen;  //buffer length
	dma_addr_t	buffer_dma_addr;  //input src dma address
	dma_addr_t	digest_dma_addr;  //output digest result dma address

	/* walk state */
	struct scatterlist	*src_sg;
	unsigned int		offset;	/* offset in current sg */
	unsigned int		total;	/* per update length*/

	size_t 			block_size;

	// u8 buffer[SHA_BUFFER_LEN + SHA512_BLOCK_SIZE] __aligned(sizeof(u32));
};
/*************************************************************************************/

struct aspeed_ecdh_ctx {
	struct aspeed_crypto_dev	*crypto_dev;
	const u8 			*public_key;
	unsigned int 			curve_id;
	size_t				n_sz;
	u8				private_key[256];
};

/*************************************************************************************/
/**
 * aspeed_rsa_key - ASPEED RSA key structure. Keys are allocated in DMA zone.
 * @n           : RSA modulus raw byte stream
 * @e           : RSA public exponent raw byte stream
 * @d           : RSA private exponent raw byte stream
 * @np          : raw byte stream for Montgomery's method, length equal to n_sz
 * @n_sz        : length in bytes of RSA modulus n
 * @e_sz        : length in bytes of RSA public exponent
 * @d_sz        : length in bytes of RSA private exponent
 */
struct aspeed_rsa_key {
	u8 *n;
	u8 *e;
	u8 *d;
	size_t n_sz;
	size_t e_sz;
	size_t d_sz;
	int nm;
	int ne;
	int nd;
	int dwm;
	int mdwm;
	u8 *np;
};

struct aspeed_rsa_ctx {
	struct aspeed_crypto_dev *crypto_dev;
	struct aspeed_rsa_key key;
	u8 *euclid_ctx;
	int enc;
};

static inline void
aspeed_crypto_write(struct aspeed_crypto_dev *crypto, u32 val, u32 reg)
{
//	printk("write : val: %x , reg : %x \n",val,reg);
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

extern int aspeed_crypto_ahash_trigger(struct aspeed_crypto_dev *aspeed_crypto);
extern int aspeed_crypto_rsa_trigger(struct aspeed_crypto_dev *aspeed_crypto);

extern int aspeed_crypto_ablkcipher_trigger(struct aspeed_crypto_dev *aspeed_crypto);
extern int aspeed_hash_trigger(struct aspeed_crypto_dev *aspeed_crypto);
extern int aspeed_hash_handle_queue(struct aspeed_crypto_dev *aspeed_crypto, struct ahash_request *req);

extern int aspeed_register_crypto_algs(struct aspeed_crypto_dev *crypto_dev);
extern int aspeed_register_ahash_algs(struct aspeed_crypto_dev *crypto_dev);
extern int aspeed_register_akcipher_algs(struct aspeed_crypto_dev *crypto_dev);
extern int aspeed_register_kpp_algs(struct aspeed_crypto_dev *crypto_dev);
extern int aspeed_crypto_enqueue(struct aspeed_crypto_dev *aspeed_crypto, struct ablkcipher_request *req);
extern int aspeed_crypto_handle_queue(struct aspeed_crypto_dev *crypto_dev,
				      struct crypto_async_request *new_areq);
#endif
