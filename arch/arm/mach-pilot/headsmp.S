// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018 Aspeed Technology Inc.
// Shivah Shankar S <shivahshankar.shankarnarayanrao@aspeedtech.com>
// Vishal C Nigade <vishal.nigade@aspeedtech.com>
//

#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/assembler.h>

/*
 * Invalidate the cache and jump to the secondary startup
 */
ENTRY(pilot_secondary_startup)
	bl      v7_invalidate_l1
	b	secondary_startup
ENDPROC(pilot_secondary_startup)


ENTRY(smc_storm)
.arch_extension sec
SMC #0
ENDPROC(smc_storm)
