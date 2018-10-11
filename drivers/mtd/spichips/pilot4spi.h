/*******************************************************************************
 *
 * Copyright (C) 2004-2015 Emulex. All rights reserved.
 * EMULEX is a trademark of Emulex.
 * www.emulex.com
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of version 2 of the GNU General Public License as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful. ALL EXPRESS
 * OR IMPLIED CONDITIONS, REPRESENTATIONS AND WARRANTIES, INCLUDING ANY IMPLIED
 * WARRANTY OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
 * NON-INFRINGEMENT, ARE DISCLAIMED, EXCEPT TO THE EXTENT THAT SUCH DISCLAIMERS
 * ARE HELD TO BE LEGALLY INVALID. See the GNU General Public License for more
 * details, a copy of which can be found in the file COPYING included
 * with this package.
 *
 ********************************************************************************/


#ifndef _SPI_PILOT4_H
#define _SPI_PILOT4_H

#include "spi_sfdp.h"

/* Flash opcodes. */
#define	OPCODE_WREN				0x06	/* Write enable */
#define	OPCODE_WRDI				0x04	/* Write disable*/
#define	OPCODE_RDID				0x9F	/* Read JEDEC ID */
#define	OPCODE_RDSR				0x05	/* Read status register */
#define OPCODE_WRSR				0x01	/* Write status register */
#define	OPCODE_READ				0x03	/* Read data bytes */
#define	OPCODE_FAST_READ		0x0B	/* Read Fast read */
#define	OPCODE_PP				0x02	/* Page program */
#define	OPCODE_SE				0xD8	/* Sector erase */
#define OPCODE_DP				0xB9	/* Deep Power Down */
#define	OPCODE_RES				0xAB	/* Read Electronic Signature */
#define OPCODE_4BYTEEN			0xb7	/* Enable 4Byte addressing mode */
#define OPCODE_4BYEDIS			0xe9	/* Disable 4Byte addressing mode */
#define OPCODE_FLG_STS_RD		0x70	/* Read flag status register */

#define OPCODE_DUAL_READ       (0x3b)   /*Dual spi read data  */
#define OPCODE_DUAL_PP         (0xA2)    /*page pgrogramme(write data)*/

#define OPCODE_SE_4B    		0xDC    // Sector erase 4 byte
#define OPCODE_READ_4B  		0x13    // Read data bytes 4 bytes
#define OPCODE_FAST_READ_4B 	0x0c    // Read Fast read 4 bytes
#define OPCODE_PP_4B        	0x12    // Page program 4 bytes

#define OPCODE_DUAL_READ_4B		0x3C
#define OPCODE_QUAD_READ_4B		0x6C

#define OPCODE_RDSFDP   		0x5A    // Read SFDP

/* Status Register bits. */
#define	SR_WIP					0x01	/* Write in progress */
#define	SR_WEL					0x02	/* Write enable latch */
#define	SR_BP0					0x04	/* Block protect 0 */
#define	SR_BP1					0x08	/* Block protect 1 */
#define	SR_BP2					0x10	/* Block protect 2 */
#define	SR_SRWD					0x80	/* SR write protect */
#define	SR_FLG_STS_REG			0x80	/* Write/Erase cycle complete */

#define OPCODE_BRWR				0x17  /* Bank Register write for spansion*/

#define PROGRAM_PAGE_SIZE		256		/* Max Program Size */

/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_COUNT 	4000000

typedef enum spidir
{
	SPI_NONE = 0,
	SPI_READ = 1,
	SPI_WRITE = 2
} SPI_DIR;

#ifdef CONFIG_SFDP
#define SPI_3BYTE_ONLY          (1 << 0)    // for 3 byte address
#define SPI_4BYTE_ONLY          (1 << 1)    // for 4 byte address only
#define SPI_4BYTE_PART          (1 << 2)    // for 4 byte address support
#define SPI_4BYTE_ENABLE_NEEDED (1 << 3)    // for 3 byte upgraded to 4 byte
#define SPI_RD_FLAG_STATUS      (1 << 4)    // for Micron devices, flag status register
#define SPI_FAST_READ			(1 << 5)	// for spi devices for fast read
#define SPI_DUAL_READ			(1 << 6)	// for spi devices supporting dual read
#define SPI_QUAD_READ			(1 << 7)	// for spi devices supporting quad read

// quad and dual read support
#define READ_FAST   (1 << 0)
#define READ_DUAL   (1 << 1)
#define READ_QUAD   (1 << 2)

struct spi_info_cmd
{
	unsigned char cmd;
	int cmdlen;		// opcode + address length
	unsigned long addr;
	SPI_DIR dir;
	unsigned int dummycycles;
	// command length for creating command is cmdlen + dummycycles
	unsigned char fastread;
} __attribute__((packed));
#endif // CONFIG_SFDP

#endif		// _SPI_PILOT4_H
