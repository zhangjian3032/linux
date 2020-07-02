/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _CRYTO_ECC_CURVE_DEFS_H
#define _CRYTO_ECC_CURVE_DEFS_H

struct ecc_point {
	u64 *x;
	u64 *y;
	u8 ndigits;
};

struct ecc_curve {
	char *name;
	struct ecc_point g;
	u64 *p;
	u64 *n;
	u64 *a;
	u64 *b;
};

#endif
