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

#include <linux/module.h>
#include <crypto/internal/rsa.h>
#include <crypto/internal/akcipher.h>
#include <crypto/akcipher.h>
#include <crypto/kpp.h>
#include <crypto/internal/kpp.h>
#include <crypto/dh.h>
#include <linux/dma-mapping.h>
#include <linux/fips.h>
#include <crypto/scatterwalk.h>

#define ASPEED_RSA_DEBUG 

#ifdef ASPEED_RSA_DEBUG
//#define RSA_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#define RSA_DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define RSA_DBG(fmt, args...)
#endif

#define MAX_TABLE_DW 128

int div_cnt=0;
int shl_cnt=0;

void init_m1(unsigned long *Dm1)
{
	int j;
	for (j=0; j<256; j++)
		Dm1[j] = 0xffffffff;		
}


void init_0(unsigned long *D0)
{
	int j;
	for (j=0; j<256; j++)
		D0[j] = 0;	
}


void init_1(unsigned long *D1)
{
	int j;
	for (j=0; j<256; j++)
		if (j==0) D1[j] = 1;
		else      D1[j] = 0;	
}

void Mul2(unsigned long *T, int mdwm)
{
unsigned long msb, temp;
int j;

    temp = 0;
    for (j=0; j<mdwm; j++) {
        msb = (T[j]>>31)&1;
        T[j] = (T[j]<<1)|temp;
        temp = msb;
    }
}

void Sub2by32(unsigned long *Borrow, unsigned long *Sub, unsigned long C, unsigned long S, unsigned long M)
{
	unsigned long long Sub2;

	Sub2  = (unsigned long long)S - (unsigned long long)M;
	if (C) Sub2 -= (unsigned long long)1; 

	if ((Sub2 >>32)>0) 
		*Borrow = 1;
	else               
		*Borrow = 0;
	*Sub = (unsigned long)(Sub2 & 0xffffffff);
}

void MCompSub(unsigned long *S, unsigned long *M, int mdwm)
{ 
int flag;
int j;
unsigned long Borrow, Sub;
   
    flag = 0;    //0: S>=M, 1: S<M
    for (j=mdwm-1; j>=0; j--) {
        if      (S[j]>M[j])           break;
        else if (S[j]<M[j]) {flag =1; break;};
    }

    if (flag==0) {
    	Borrow = 0;
        for (j=0; j<mdwm; j++) {
            Sub2by32(&Borrow, &Sub, Borrow, S[j], M[j]);
            S[j] = Sub;
        }
    }
    
}

void MulRmodM(unsigned long *X, unsigned long *M, int nm, int mdwm)
{
	int k;
	RSA_DBG("\n");
    for (k=0; k<nm; k++) {
    	Mul2(X, mdwm);
    	MCompSub(X, M, mdwm);
    }
}

void Copy(unsigned long *D, unsigned long *S)
{
int j;
    for (j=0; j<256; j++)
        D[j] = S[j];
}

int Compare(unsigned long *X, unsigned long *Y)
{
int j;
int result;
    
    result = 0;
    for (j=256-1; j>=0; j--)
        if      (X[j]>Y[j]) {result =  1; break;}
        else if (X[j]<Y[j]) {result = -1; break;}
    return(result);
}

void Add(unsigned long *X, unsigned long *Y)
{
int j;
unsigned long long t1;
unsigned long long t2;
unsigned long long sum;
unsigned long long carry;

     carry =0;
     for (j=0; j<255; j++) {
         t1 = X[j];
         t2 = Y[j];
         sum = t1 + t2 + carry;
         X[j]  = sum & 0xffffffff;
         carry = (sum >>32) & 0xffffffff;
     }
     //if (carry>0) printf("!!!overflow!!!\n");
     X[255] = carry;
}

int nmsb(unsigned long *X)
{
int i, j;
int nmsb;
    nmsb = 256*32;
    for (j=256-1; j>=0; j--){ 
        if (X[j]==0) nmsb-=32;
        else {
            for (i=32-1; i>=0; i--)
                 if ((X[j]>>i)&1) {i=0; j=0; break;}
                 else  nmsb --;
        }
    }
    return(nmsb);

}

void ShiftLeftFast(unsigned long *R, unsigned long *X, int nx, int ny)
{
int j;
unsigned long cntb;
unsigned long shldw, shrbit;
unsigned long shloffset;
unsigned long bitbuf;

     cntb = nx/32;
     if ((nx%32)>0) cntb++;
    
     shldw  = cntb - ((nx-ny)/32);

     shrbit = (nx-ny) % 32;
     shloffset = (nx-ny) /32;
     bitbuf =0;
     //printf("nx=%d, ny=%d, cntb=%d, shldw=%d, shrbit=%d, shloffset=%d\n", nx, ny, cntb, shldw, shrbit, shloffset);
     for (j=shldw-1; j>=0; j--) {
          if (shrbit==0){
              R[j] = (X[shloffset+j]>>shrbit);
              bitbuf = X[shloffset+j]; 
          }
          else {
              R[j] = (X[shloffset+j]>>shrbit)|bitbuf;
              bitbuf = X[shloffset+j] <<(32-shrbit);
          }
          //printf("%x, ", bitbuf);
     }
     //printf("\n");    
}

unsigned char Getbit(unsigned long *X, int k)
{
	unsigned char bit = ((X[k/32]>>(k%32))&1) & 0xff;
     return bit;
}

void Substrate(unsigned long *X, unsigned long *Y)
{
int j;
unsigned long long t1;
unsigned long long t2;
unsigned long long sum;
unsigned long carry;

     carry =0;
     for (j=0; j<255; j++) {
         t1 = X[j];
         t2 = Y[j];
         if (carry) sum = t1 - t2 -1;
         else       sum = t1 - t2;
         X[j]  = sum & 0xffffffff;
         carry = (sum >>32) & 0xffffffff;
     }
     //X[255] = 0xffffffff;
     if (carry>0) X[255] = 0xffffffff;
     else         X[255] = 0x0;
}

void ShiftLeft(unsigned long *X, int i)
{
int j;
int msb;
int temp;

     msb = i;
     for (j=0; j<256; j++) {
          temp = X[j]>>31;
          X[j] = (X[j]<<1)|msb;
          msb = temp;
     }     
}

void ShiftRight(unsigned long *X)
{
int j;
int lsb;
int temp;

     lsb = 0;
     for (j=255; j>=0; j--) {
          temp = (X[j]&1);
          X[j] = (X[j]>>1)|(lsb<<31);
          lsb = temp;
     }     
}

void Divide(unsigned long *Q, unsigned long *R, unsigned long *X, unsigned long *Y)
{
int j;
int nx, ny;

unsigned long T [256];
    
    nx = nmsb(X);
    ny = nmsb(Y);
    init_0(Q);
    init_0(R);
    //printf("Div X = "); printX(X);
    //printf("Div Y = "); printX(Y);

    //for (j=nx-1; j>=0; j--) {
    //    ShiftLeft(R, Getbit(X, j));
    //    if ((nx-j)>=ny) {
    //        if(Compare(R, Y)>=0) {
    //            Substrate(R, Y);
    //            ShiftLeft(Q, 1);
    //        } else {
    //            ShiftLeft(Q, 0);
    //        }
    //    }
    //}
    ShiftLeftFast(R, X, nx, ny);
    //printf("Nor R = "); printX(R);
    for (j=nx-ny; j>=0; j--) {
         shl_cnt++;
         if(Compare(R, Y)>=0) {
             //printf("Div R = "); printA(R);
             //printf("Div Y = "); printA(Y);
             //printf("R>=Y\n");
             Substrate(R, Y); 
             ShiftLeft(Q, 1); 
         } else {             
             //printf("Div R = "); printA(R);
             //printf("Div Y = "); printA(Y);
             //printf("R<Y\n");
             ShiftLeft(Q, 0); 
         }
         if (j>0)
             ShiftLeft(R, Getbit(X, j-1));
    }

    //printf("Rst Q = "); printX(Q);
    //printf("Rst R = "); printX(R);                   
}

void Positive(unsigned long *X)
{
unsigned long D0[256];

     init_0(D0);
     Substrate(D0, X);
     Copy(X, D0);
}

void MultiplyLSB(unsigned long *X, unsigned long *Y)
{
int i, j;
unsigned long T [256];
unsigned long long t1;
unsigned long long t2;
unsigned long long product;
unsigned long carry;
unsigned long temp;

    init_0(T);
    for (i=0; i<128; i++) {
    	carry =0;
        for (j=0; j<130; j++) {
            if (i+j<130) {
            	t1=X[i];
            	t2=Y[j];
            	product = t1*t2 + carry + T[i+j];
            	temp = (product >>32)& 0xffffffff;
            	T[i+j] = product & 0xffffffff;
            	carry = temp;
            }
        }
    }
    Copy(X, T);       
	
}


//x = lastx - q * t;
void CalEucPar(unsigned long *x, unsigned long *lastx, unsigned long *q, unsigned long *t)
{
int j;
unsigned long temp [256];
    
    //printf("Start CalEucPar\n");
    Copy(temp, t);
    Copy(x, lastx);
    if (Getbit(temp, 4095)) {
        Positive(temp);
        //printf("t is negtive\n");
        //printf("q="); printX(q);
        //printf("t="); printX(temp);
        MultiplyLSB(temp, q);
        //printf("q*t="); printX(temp);
        Add(x, temp);
        //printf("lastx-q*t="); printX(x);
    } else {
        //printf("q="); printX(q);
        //printf("t="); printX(temp);
        MultiplyLSB(temp, q);
        //printf("q*t="); printX(temp);
        Substrate(x, temp);
        //printf("lastx-q*t="); printX(x);
    }
}

void Euclid(unsigned long *Mp, unsigned long *M, unsigned long *S, int nm)
{
int j;
int check;
unsigned long a[256];
unsigned long b[256];

unsigned long q [256];
unsigned long r [256];
unsigned long x [256];
unsigned long y [256];
unsigned long lastx [256];
unsigned long lasty [256];
unsigned long t  [256];
unsigned long D1 [256];
	RSA_DBG("\n");

    Copy(a, M);
    Copy(b, S);

    init_1(D1);
    init_1(x);
    init_0(lastx);

    init_m1(y);
    init_1(lasty);
    div_cnt = 0;
    shl_cnt = 0;
    //step 2
    while (Compare(b, D1)>0) {
        //q = a div b, r = a mod b
        //printf("a="); printX(a);
        //printf("b="); printX(b);
        Divide(q, r, a, b);
        div_cnt++;
        //printf("q="); printX(q);
        //printf("r="); printX(r);
        //a = b;
        Copy(a, b);
        //b = r;
        Copy(b, r);
        
        Copy(t, x);
        //x = lastx - q * x;
        CalEucPar(x, lastx, q, t);
        Copy(lastx, t);

        Copy(t, y);
        //y = lasty - q * y;
        CalEucPar(y, lasty, q, t);
        Copy(lasty, t);

        //printf("lastx="); printX(lastx);
        //printf("lasty="); printX(lasty);
        //printf("x="); printX(x);
        //printf("y="); printX(y);
    }

    init_1(r);
    for (j=0; j<nm; j++) ShiftLeft(r, 0);
    if (Getbit(x, 4095)) {
    	Add(x, M);
    	Substrate(y, r);
    }
    Positive(y);
    Copy(Mp, y);

}

int aspeed_crypto_rsa_trigger(struct aspeed_crypto_dev *crypto_dev)
{

	return 0;
}

static int aspeed_rsa_enc(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct aspeed_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	RSA_DBG("\n");

	return 0;
}

static int aspeed_rsa_dec(struct akcipher_request *req)
{
//	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
//	struct aspeed_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	RSA_DBG("---------\n");

}

static int aspeed_rsa_setpubkey(struct crypto_akcipher *tfm, const void *key,
				unsigned int keylen)
{
//	struct aspeed_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
//	struct rsa_key raw_key = {0};
//	struct aspeed_rsa_key *rsa_key = &ctx->key;
//	int ret;
	RSA_DBG("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
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
	struct aspeed_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct rsa_key raw_key;
	struct aspeed_crypto_dev	*crypto_dev = ctx->crypto_dev;
	struct aspeed_rsa_key *rsa_key = &ctx->key;
	int ret;
	u8 *key_buff = key;
	int i;
	int nm, dwm, mdwm;
	unsigned long S [256];	
	
	RSA_DBG("keylen %d \n", keylen);

	/* Free the old RSA key if any */
//	caam_rsa_free_key(rsa_key);
#if 1

	printk("key \n");

	for(i = 0; i < keylen; i++)
		printk("%x ", key_buff[i]);

	printk(" \n");
#endif

	ret = rsa_parse_priv_key(&raw_key, key, keylen);
	if (ret)
		return ret;

	printk("raw_key.n_sz %d, raw_key.e_sz %d, raw_key.d_sz %d, raw_key.p_sz %d, raw_key.q_sz %d, raw_key.dp_sz %d, raw_key.dq_sz %d, raw_key.qinv_sz %d \n", 
			raw_key.n_sz, raw_key.e_sz, raw_key.d_sz, 
			raw_key.p_sz, raw_key.q_sz, raw_key.dp_sz, 
			raw_key.dq_sz, raw_key.qinv_sz);

	printk("raw_key.n %x, raw_key.e %x, raw_key.d %x, raw_key.p %x, raw_key.q %x, raw_key.dp %x, raw_key.dq %x, raw_key.qinv %x \n", 
			raw_key.n, raw_key.e, raw_key.d, 
			raw_key.p, raw_key.q, raw_key.dp, 
			raw_key.dq, raw_key.qinv);
	
	/* Copy key in DMA zone */
	rsa_key->d = kzalloc(raw_key.d_sz, GFP_DMA | GFP_KERNEL);
	
	if (!rsa_key->d) {
		printk("!rsa_key->d ");
		goto err;
	}
	
	rsa_key->e = kzalloc(raw_key.e_sz, GFP_DMA | GFP_KERNEL);
	if (!rsa_key->e) {
		printk("!rsa_key->e ");
		goto err;
	}


	rsa_key->n = crypto_dev->rsa_buff + 0x800;
	if (!rsa_key->n) {
		printk("!rsa_key->n ");
		goto err;
	}

	rsa_key->d_sz = raw_key.d_sz;
	rsa_key->e_sz = raw_key.e_sz;
	rsa_key->n_sz = raw_key.n_sz;

	
	memcpy(rsa_key->d, raw_key.d, raw_key.d_sz);
	memcpy(rsa_key->e, raw_key.e, raw_key.e_sz);
	/*
	* @n		   : RSA modulus raw byte stream
	* @e		   : RSA public exponent raw byte stream
	* @d		   : RSA private exponent raw byte stream
	*/
	memcpy(rsa_key->n, raw_key.n, raw_key.n_sz);
//	nm = get_bit_number(rsa_key->n);
//	printk("modulus nm bits %d \n", nm);
#if 0
	nm = rsa_key->n_sz * 8;
	if ((nm%32) > 0)
		dwm = (nm/32) + 1;
	else
		dwm = (nm/32);

	mdwm = dwm;
	if ((nm%32)==0) mdwm++;

	

    init_1(S);
    MulRmodM(S, rsa_key->n, nm, mdwm);
	
    // calculate Mp, R*1/R - Mp*M = 1
    // Because R div M = 1 rem (R-M), S=R-M, so skip first divide.
    Euclid(crypto_dev->rsa_buff + 0x400, rsa_key->n, S, nm);
#endif
	return 0;

err:
	return -ENOMEM;

}


static int aspeed_rsa_max_size(struct crypto_akcipher *tfm)
{
	struct aspeed_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct aspeed_rsa_key *key = &ctx->key;
	RSA_DBG("key->n_sz %d %x \n", key->n_sz, key->n);
	return (key->n) ? key->n_sz : -EINVAL;
}

static int aspeed_rsa_init_tfm(struct crypto_akcipher *tfm)
{
	struct aspeed_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct akcipher_alg *alg = __crypto_akcipher_alg(tfm->base.__crt_alg);
	struct aspeed_crypto_alg *algt;
	RSA_DBG("\n");

	algt = container_of(alg, struct aspeed_crypto_alg, alg.akcipher);

	ctx->crypto_dev = algt->crypto_dev;

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
					.cra_flags		= CRYPTO_ALG_TYPE_AKCIPHER |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
				
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
		err = crypto_register_akcipher(&aspeed_akcipher_algs[i].alg.akcipher);
		if (err) {
			RSA_DBG("--------------------- err \n");
			return err;
		}
	}
	RSA_DBG("---------------------\n");
	return 0;
}
