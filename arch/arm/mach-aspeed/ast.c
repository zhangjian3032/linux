/*
 *  linux/arch/arm/mach-aspeed/ast.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#include <asm/mach/arch.h>
#include <mach/ast_wdt.h>

static const char *const aspeed_dt_match[] __initconst = {
		"aspeed,ast2400",
		"aspeed,ast2500",
		"aspeed,ast2600",		
		NULL,
};

DT_MACHINE_START(aspeed_dt, "ASpeed BMC SoC")
	.dt_compat		= aspeed_dt_match,
#if defined(CONFIG_AST_WATCHDOG) || defined(CONFIG_AST_WATCHDOG_MODULE)
	.restart			= ast_soc_wdt_reset,
#endif
MACHINE_END
