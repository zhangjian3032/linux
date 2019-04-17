// SPDX-License-Identifier: GPL-2.0
// Copyright (c) Aspeed Technology Inc.
// Shivah Shankar S <shivahshankar.shankarnarayanrao@aspeedtech.com>
// Vishal C Nigade <vishal.nigade@aspeedtech.com>
//
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>

extern void smc_storm(unsigned int cypher);
static const char *const pilot_dt_compat[] __initconst = {
	"aspeed,pilot",
	NULL,
};
#define CYPHER 0xB00B00D0

void pilot4_restart(enum reboot_mode mode, const char *cmd)
{
	mdelay(50);
	smc_storm((unsigned int)CYPHER);

	while (1)
		wfe();
	//We will never reach here, hwoever if we reach if we reach here pls check wait_for_wfe in uboot
}


DT_MACHINE_START(ASPEED, "Aspeed Pilot")
//.atag_offset	= 0x100,
	.dt_compat      = pilot_dt_compat,
	.restart	= pilot4_restart,
MACHINE_END

