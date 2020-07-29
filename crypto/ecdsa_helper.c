/*
 * ECDSA helper routines
 *
 * Copyright (c) 2017, NVIDIA Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/string.h>
#include <crypto/ecdsa.h>

#include "ecc.h"

#define ECDSA_KEY_MIN_SIZE	(1 + 1 + 24) /* ver + cid + n (P-192) */

unsigned int ecdsa_supported_curve(unsigned int curve_id)
{
	switch (curve_id) {
	case ECC_CURVE_NIST_P192: return 3;
	case ECC_CURVE_NIST_P256: return 4;
	default: return 0;
	}
}

static inline u8 *ecdsa_pack_data(void *dst, const void *src, size_t sz)
{
	memcpy(dst, src, sz);
	return dst + sz;
}

static inline const u8 *ecdsa_unpack_data(void *dst, const void *src, size_t sz)
{
	memcpy(dst, src, sz);
	return src + sz;
}

int crypto_ecdsa_parse_pub_key(const char *buf, unsigned int len,
			       struct ecdsa *params)
{
	unsigned char version;
	unsigned int ndigits;
	unsigned int nbytes;
	const u8 *ptr = buf;
	u8 *qx, *qy;

	if (unlikely(!buf) || len < ECDSA_KEY_MIN_SIZE)
		return -EINVAL;

	ptr = ecdsa_unpack_data(&version, ptr, sizeof(version));
	if (version != 1)
		return -EINVAL;

	ptr = ecdsa_unpack_data(&params->curve_id, ptr,
				sizeof(params->curve_id));

	ndigits = ecdsa_supported_curve(params->curve_id);
	if (!ndigits)
		return -EINVAL;

	nbytes = ndigits << ECC_DIGITS_TO_BYTES_SHIFT;

	/* skip private key */
	ptr = ecdsa_unpack_data(&params->key, ptr, nbytes);

	/* copy public key */
	qx = params->key;
	qy = qx + ECC_MAX_DIGIT_BYTES;

	ptr = ecdsa_unpack_data(qx, ptr, nbytes);
	ptr = ecdsa_unpack_data(qy, ptr, nbytes);

	params->key_size = 2 * nbytes;

	return 0;
}
EXPORT_SYMBOL_GPL(crypto_ecdsa_parse_pub_key);

int crypto_ecdsa_parse_priv_key(const char *buf, unsigned int len,
				struct ecdsa *params)
{
	unsigned char version;
	unsigned int ndigits;
	unsigned int nbytes;
	const u8 *ptr = buf;

	if (unlikely(!buf) || len < ECDSA_KEY_MIN_SIZE)
		return -EINVAL;

	ptr = ecdsa_unpack_data(&version, ptr, sizeof(version));
	if (version != 1)
		return -EINVAL;

	ptr = ecdsa_unpack_data(&params->curve_id, ptr,
				sizeof(params->curve_id));

	ndigits = ecdsa_supported_curve(params->curve_id);
	if (!ndigits)
		return -EINVAL;

	nbytes = ndigits << ECC_DIGITS_TO_BYTES_SHIFT;

	params->key_size = nbytes;

	/* copy private key */
	ptr = ecdsa_unpack_data(&params->key, ptr, nbytes);

	return 0;
}
EXPORT_SYMBOL_GPL(crypto_ecdsa_parse_priv_key);