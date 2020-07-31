/*
 * Copyright (c) 2013, Kenneth MacKay. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#ifndef _CRYPTO_ECC_ECDH_H
#define _CRYPTO_ECC_ECDH_H

#include "ecc.h"

/**
 * crypto_ecdh_shared_secret() - Compute a shared secret
 *
 * @curve_id:		id representing the curve to use
 * @ndigits:		curve's number of digits
 * @private_key:	private key of part A
 * @public_key:		public key of counterpart B
 * @secret:		buffer for storing the calculated shared secret
 *
 * Note: It is recommended that you hash the result of crypto_ecdh_shared_secret
 * before using it for symmetric encryption or HMAC.
 *
 * Returns 0 if the shared secret was generated successfully, a negative value
 * if an error occurred.
 */
int crypto_ecdh_shared_secret(unsigned int curve_id, unsigned int ndigits,
			      const u64 *private_key, const u64 *public_key,
			      u64 *secret);

#endif /* _CRYPTO_ECC_ECDH_H */