/*******************************************************************************
 *
 * Copyright (C) 2005-2015 Emulex. All rights reserved.
 * EMULEX is a trademark of Emulex.
 * www.emulex.com
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of version 2 of the GNU General Public License as published by the
 * Free Software Foundation.
 * This program is distributed in the hope that it will be useful. ALL EXPRESS
 * OR IMPLIED CONDITIONS, REPRESENTATIONS AND WARRANTIES, INCLUDING ANY IMPLIED
 * WARRANTY OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
 * NON-INFRINGEMENT, ARE DISCLAIMED, EXCEPT TO THE EXTENT THAT SUCH DISCLAIMERS
 * ARE HELD TO BE LEGALLY INVALID. See the GNU General Public License for more
 * details, a copy of which can be found in the file COPYING included
 * with this package.
 *
 ********************************************************************************/

#ifndef __ESPI_MAFS_H__
#define __ESPI_MAFS_H__

#define ESPIMAFS_MAGIC   'e'

#define ESPI_CH3_CAP_1 (se_pilot_espi_va_base + 0x40)
#define ESPI_CH1_CAP        (se_pilot_espi_va_base + 0x20)
#define ESPI_FLASH_BASE (se_pilot_espi_va_base + 0xA00)

#define ESPI_FLASH_CTRL (ESPI_FLASH_BASE + 0x0)
#define ESPI_FLASH_STS  (ESPI_FLASH_BASE + 0x4)
#define ESPI_FLASH_SIZE (ESPI_FLASH_BASE + 0x8)
#define ESPI_FLASH_DATA (ESPI_FLASH_BASE + 0xC)
#define ESPI_FLASH_CYC  (ESPI_FLASH_BASE + 0x10)
#define ESPI_FLASH_TAG  (ESPI_FLASH_BASE + 0x14)
#define ESPI_FLASH_LEN  (ESPI_FLASH_BASE + 0x18)
#define ESPI_FLASH_ADD  (ESPI_FLASH_BASE + 0x1c)
#define ESPI_FLASH_PSR  (ESPI_FLASH_BASE + 0x20)

#define B2H_PKT_VLD		(1 << 0)
#define	B2H_INTR_EN		(1 << 1)
#define B2H_WPTR_CLR	(1 << 2)
#define B2H_RPTR_CLR	(1 << 3)
#define H2B_INTR_EN		(1 << 8)
#define H2B_WPTR_CLR	(1 << 9)
#define H2B_RPTR_CLR	(1 << 10)


#define MAFS_READ         0x0
#define MAFS_WRITE        0x1
#define MAFS_ERASE        0x2

#define H2B_INTR_STS	(1 << 5)
#define H2B_PKT_VALID	(1 << 7)
#define B2H_BUSY		(1 << 0)

#define TIMEOUT_MS	5000

typedef struct mafs_data
{
    unsigned int address;
    int length;
    unsigned char tag;
    unsigned char* data;
    unsigned char response;
} MAFS_DATA;

typedef enum  {
    IOCTL_ESPI_MAFS_ERASE= 1,
    IOCTL_ESPI_MAFS_READ,
    IOCTL_ESPI_MAFS_WRITE,
    IOCTL_ESPI_MAFS_CLEAR_RESET
}ESPI_MAFS_IOCTLS;

#define CMD_IOCTL_ESPI_MAFS_ERASE    	_IOWR(ESPIMAFS_MAGIC, IOCTL_ESPI_MAFS_ERASE, MAFS_DATA)
#define CMD_IOCTL_ESPI_MAFS_READ     	_IOWR(ESPIMAFS_MAGIC, IOCTL_ESPI_MAFS_READ, MAFS_DATA)
#define CMD_IOCTL_ESPI_MAFS_WRITE    	_IOWR(ESPIMAFS_MAGIC, IOCTL_ESPI_MAFS_WRITE, MAFS_DATA)
#define CMD_IOCTL_ESPI_MAFS_CLEAR_RESET _IOWR(ESPIMAFS_MAGIC, IOCTL_ESPI_MAFS_CLEAR_RESET, unsigned char)


#endif // __ESPI_MAFS_H__
