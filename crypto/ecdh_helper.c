/*
 * Copyright (c) 2016, Intel Corporation
 * Authors: Salvatore Benedetto <salvatore.benedetto@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/random.h>
#include <crypto/ecdh.h>
#include <crypto/kpp.h>

#include "ecc_ecdh.h"

#define ECDH_KPP_SECRET_MIN_SIZE (sizeof(struct kpp_secret) + 2 * sizeof(short))

static inline u8 *ecdh_pack_data(void *dst, const void *src, size_t sz)
{
	memcpy(dst, src, sz);
	return dst + sz;
}

static inline const u8 *ecdh_unpack_data(void *dst, const void *src, size_t sz)
{
	memcpy(dst, src, sz);
	return src + sz;
}

int crypto_ecdh_shared_secret(unsigned int curve_id, unsigned int ndigits,
			      const u64 *private_key, const u64 *public_key,
			      u64 *secret)
{
	int ret = 0;
	struct ecc_point *product, *pk;
	u64 priv[ECC_MAX_DIGITS];
	u64 rand_z[ECC_MAX_DIGITS];
	unsigned int nbytes;
	const struct ecc_curve *curve = ecc_get_curve(curve_id);

	if (!private_key || !public_key || !curve ||
	    ndigits > ARRAY_SIZE(priv) || ndigits > ARRAY_SIZE(rand_z)) {
		ret = -EINVAL;
		goto out;
	}

	nbytes = ndigits << ECC_DIGITS_TO_BYTES_SHIFT;

	get_random_bytes(rand_z, nbytes);

	pk = ecc_alloc_point(ndigits);
	if (!pk) {
		ret = -ENOMEM;
		goto out;
	}

	ecc_swap_digits(public_key, pk->x, ndigits);
	ecc_swap_digits(&public_key[ndigits], pk->y, ndigits);
	ret = ecc_is_pubkey_valid_partial(curve, pk);
	if (ret)
		goto err_alloc_product;

	ecc_swap_digits(private_key, priv, ndigits);

	product = ecc_alloc_point(ndigits);
	if (!product) {
		ret = -ENOMEM;
		goto err_alloc_product;
	}

	ecc_point_mult(product, pk, priv, rand_z, curve, ndigits);

	ecc_swap_digits(product->x, secret, ndigits);

	if (ecc_point_is_zero(product))
		ret = -EFAULT;

	ecc_free_point(product);
err_alloc_product:
	ecc_free_point(pk);
out:
	return ret;
}

unsigned int crypto_ecdh_key_len(const struct ecdh *params)
{
	return ECDH_KPP_SECRET_MIN_SIZE + params->key_size;
}
EXPORT_SYMBOL_GPL(crypto_ecdh_key_len);

int crypto_ecdh_encode_key(char *buf, unsigned int len,
			   const struct ecdh *params)
{
	u8 *ptr = buf;
	struct kpp_secret secret = {
		.type = CRYPTO_KPP_SECRET_TYPE_ECDH,
		.len = len
	};

	if (unlikely(!buf))
		return -EINVAL;

	if (len != crypto_ecdh_key_len(params))
		return -EINVAL;

	ptr = ecdh_pack_data(ptr, &secret, sizeof(secret));
	ptr = ecdh_pack_data(ptr, &params->curve_id, sizeof(params->curve_id));
	ptr = ecdh_pack_data(ptr, &params->key_size, sizeof(params->key_size));
	ecdh_pack_data(ptr, params->key, params->key_size);

	return 0;
}
EXPORT_SYMBOL_GPL(crypto_ecdh_encode_key);

int crypto_ecdh_decode_key(const char *buf, unsigned int len,
			   struct ecdh *params)
{
	const u8 *ptr = buf;
	struct kpp_secret secret;

	if (unlikely(!buf || len < ECDH_KPP_SECRET_MIN_SIZE))
		return -EINVAL;

	ptr = ecdh_unpack_data(&secret, ptr, sizeof(secret));
	if (secret.type != CRYPTO_KPP_SECRET_TYPE_ECDH)
		return -EINVAL;

	ptr = ecdh_unpack_data(&params->curve_id, ptr, sizeof(params->curve_id));
	ptr = ecdh_unpack_data(&params->key_size, ptr, sizeof(params->key_size));
	if (secret.len != crypto_ecdh_key_len(params))
		return -EINVAL;

	/* Don't allocate memory. Set pointer to data
	 * within the given buffer
	 */
	params->key = (void *)ptr;

	return 0;
}
EXPORT_SYMBOL_GPL(crypto_ecdh_decode_key);
