/*
 * Copyright (C) 2003-2015 Emulex. All rights reserved. 
 * EMULEX is a trademark of Emulex. 
 * www.emulex.com 
 * This program is free software; you can redistribute it and/or modify it under the terms of version 2
 * of the GNU General Public License as published by the Free Software Foundation. 
 * This program is distributed in the hope that it will be useful. ALL EXPRESS OR IMPLIED CONDITIONS,
 * REPRESENTATIONS AND WARRANTIES, INCLUDING ANY IMPLIED WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, OR NON-INFRINGEMENT, ARE DISCLAIMED, EXCEPT TO THE EXTENT THAT SUCH DISCLAIMERS
 * ARE HELD TO BE LEGALLY INVALID. See the GNU General Public License for more details, a copy of which
 * can be found in the file COPYING included with this package.
 */

#ifndef __PILOT4SPI_FLASH__
#define __PILOT4SPI_FLASH__

extern unsigned int IO_ADDRESS(unsigned int);

#define SE_BOOT_SPI_VA_BASE 	IO_ADDRESS(0x40200000)
#define SE_BACKUP_SPI_VA_BASE	IO_ADDRESS(0x40280000)
#define SE_HOST_SPI_VA_BASE		IO_ADDRESS(0x40429000)

#define CONFIG_SYS_FLASH_BASE		0

#define CONFIG_SYS_MAX_BOOT_SPI_BANKS   3
#define CONFIG_SYS_MAX_BKUP_SPI_BANKS   3
#define CONFIG_SYS_MAX_HOST_SPI_BANKS   2
#define CONFIG_SYS_MAX_SPI_BANKS   (CONFIG_SYS_MAX_BOOT_SPI_BANKS + CONFIG_SYS_MAX_BKUP_SPI_BANKS + \
		                                        CONFIG_SYS_MAX_HOST_SPI_BANKS)

#define CONFIG_PROJ_MAX_BOOTSPI_SIZE 0xFC00000
#define CONFIG_PROJ_MAX_SPICS0_SIZE  0xFC00000


#endif // __PILOT4SPI_FLASH__
