/*
 * Copyright (C) 2007 American Megatrends Inc
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * Copyright (c) 2010-2015, Emulex Corporation.
 * Modifications made by Emulex Corporation under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#ifdef __UBOOT__	
#include <common.h>
#endif
#include "spiflash.h"
#ifdef	CFG_FLASH_SPI_DRIVER

#include <linux/version.h>

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,4,41)
#else
#define init_MUTEX(sem)     sema_init(sem, 1)
#endif


#ifndef CONFIG_SFDP
/* Name, ID1, ID2 , Size, Erase regions, { Offset, Erase Size, Erase Block Count } */
static struct spi_flash_info pilotavl_data [] = 
{
    /* Note -- we ignore the speed of the part because the Pilot-II always runs at 25MHz regardless */

	/* ST Micro 32K Sectors */
	{ "ST Micro m25p05A" , 0x20, 0x1020, 0x010000 , SPI_FLAG_NONE, 1, {{ 0, 32  * 1024, 2   },} },
	{ "ST Micro m25p10A" , 0x20, 0x1120, 0x020000 , SPI_FLAG_NONE, 1, {{ 0, 32  * 1024, 4   },} },

	/* ST Micro 64 K Sectors */
	{ "ST Micro m25p20"  , 0x20, 0x1220, 0x040000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 4   },} },
	{ "ST Micro m25p40"  , 0x20, 0x1320, 0x080000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 8   },} },
	{ "ST Micro m25p80"  , 0x20, 0x1420, 0x100000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 16  },} },
	{ "ST Micro m25p16"  , 0x20, 0x1520, 0x200000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 32  },} },
	{ "ST Micro m25p32"  , 0x20, 0x1620, 0x400000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 64  },} },
	{ "ST Micro m25p64"  , 0x20, 0x1720, 0x800000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 128 },} },
    { "ST Micro m25p64"  , 0x20, 0x1771, 0x800000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 128 },} },


	/* ST Micro 256K Sectors */
	{ "ST Micro m25p128" , 0x20, 0x1820, 0x1000000, SPI_FLAG_NONE, 1, {{ 0, 256 * 1024, 64  },} },

    /* ST Micro 64 K Sectors, 25MHz speed */
	{ "ST Micro m45p20"  , 0x20, 0x1240, 0x040000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 4   },} },
	{ "ST Micro m45p40"  , 0x20, 0x1340, 0x080000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 8   },} },
	{ "ST Micro m45p80"  , 0x20, 0x1440, 0x100000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 16  },} },
	{ "ST Micro m45p16"  , 0x20, 0x1540, 0x200000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 32  },} },
	{ "ST Micro m45p32"  , 0x20, 0x1640, 0x400000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 64  },} },
	{ "ST Micro m45p64"  , 0x20, 0x1740, 0x800000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 128 },} },
	//        { "Spansion Spi Flash"  , 0x01, 0x1820, 0x800000 , 1, {{ 0, 64  * 1024, 256 },} },


    /* Similar families that can use the same algorithm with uniform 64K sector erase, 256 byte page writes */

    /* ST Micro/Numonyx M25PX family */
	{ "ST Micro m25px32" , 0x20, 0x1671, 0x400000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 64  },} },
	{ "Numonyx m25px64" , 0x20, 0x1771, 0x800000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 128 },} },

	{ "Winbond W25X16"   , 0xef, 0x1530, 0x200000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 32  },} },
	{ "Winbond W25X32"   , 0xef, 0x1630, 0x400000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 64  },} },
	{ "Winbond W25X64"   , 0xef, 0x1730, 0x800000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 128 },} },
	{ "Winbond W25X128"   , 0xef, 0x1840, 0x1000000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 256 },} },
	{ "Winbond W25X256"   , 0xef, 0x1940, 0x2000000 , SPI_FBYTE_ADDR_MODE, 1, {{ 0, 64  * 1024, 512 },} },

	{ "Spans S25FL016A"  , 0x01, 0x1402, 0x200000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 32  },} },
	{ "Spans S25FL032A"  , 0x01, 0x1502, 0x400000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 64  },} },
	{ "Spans S25FL064A"  , 0x01, 0x1602, 0x800000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 128 },} },
	{ "Spans S25FL128P"  , 0x01, 0x1820, 0x1000000, SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 256 },} },
	{ "Spans S25FL064A"  , 0x01, 0x2002, 0x4000000, SPI_FBYTE_ADDR_MODE | SPI_FBYTE_SPANSION, 1, {{ 0, 256  * 1024, 256 },} },

	{ "Macronix MX25L1605D",0xc2,0x1520, 0x200000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 32  },} },
	{ "Macronix MX25L3205D",0xc2,0x1620, 0x400000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 64  },} },
	{ "Macronix MX25L6405D",0xc2,0x1720, 0x800000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 128 },} },
	{ "Macronix MX25L25635E",0xc2,0x1920,0x2000000 , SPI_FBYTE_ADDR_MODE, 1, {{ 0, 64  * 1024, 512},} },
	{ "Macronix MX66L51235F",0xc2,0x1a20,0x4000000 , SPI_FBYTE_ADDR_MODE, 1, {{ 0, 64  * 1024, 1024},} },
	{ "Eon EN25HQ128",0x1c,0x1870,0x1000000 , SPI_FLAG_NONE, 1, {{ 0, 64  * 1024, 256},} },
	{ "Eon EN25HQ256",0x1c,0x1970,0x2000000 , SPI_FBYTE_ADDR_MODE, 1, {{ 0, 64  * 1024, 512},} },
	{ "Micron N25Q256A13EXX40X",0x20,0x19ba,0x2000000 , SPI_FBYTE_ADDR_MODE | SPI_FLAG_STS_READ, 1, {{ 0, 64  * 1024, 512},} },
	{ "Micron N25Q512A13GXX40X",0x20,0x20ba,0x4000000 , SPI_FBYTE_ADDR_MODE | SPI_FLAG_STS_READ, 1, {{ 0, 64  * 1024, 1024},} },
	{ "Micron N25Q00AA13GXX40X",0x20,0x21ba,0x8000000 , SPI_FBYTE_ADDR_MODE | SPI_FLAG_STS_READ, 1, {{ 0, 64  * 1024, 2048},} },
	{ "Micron 256_1.8",0x20,0x19bb,0x2000000 , SPI_FBYTE_ADDR_MODE | SPI_FLAG_STS_READ, 1, {{ 0, 64  * 1024, 512},} },
	{ "Micron 512_1.8",0x20,0x20bb,0x4000000 , SPI_FBYTE_ADDR_MODE | SPI_FLAG_STS_READ, 1, {{ 0, 64  * 1024, 1024},} },
};
#endif

static
int 
pilotavl_probe(int bank,struct spi_ctrl_driver *ctrl_drv, struct spi_flash_info *chip_info)
{
	int retval;
	retval = spi_generic_probe(bank,ctrl_drv,chip_info,"pilotavl",
#ifndef CONFIG_SFDP
						pilotavl_data,ARRAY_SIZE(pilotavl_data));
#else
						NULL, 0);
#endif
	if (retval == -1)
		return retval;

	/* UnProctect all sectors */
 	/* SRWD=0 (Bit 7)  BP0,BP1,BP2 = 0 (Bit 2,3,4) */
	if (spi_generic_write_status(bank,ctrl_drv,0x0) < 0)
		printk("pilotavl: Unable to Unprotect all sectors\n");

	return retval;
}

struct spi_chip_driver pilotavl_driver =
{
	.name 		= "pilotavl",
	.module 	= THIS_MODULE,
	.probe	 	= pilotavl_probe,
	.erase_sector 	= spi_generic_erase,
	.read_bytes	= spi_generic_read,
	.write_bytes	= spi_generic_write,
};



int 
pilotavl_init(void)
{
	printk("Entered %s\n", __FUNCTION__);
	init_MUTEX(&pilotavl_driver.lock);
	register_spi_chip_driver(&pilotavl_driver);
	return 0;
}


void 
pilotavl_exit(void)
{
	init_MUTEX(&pilotavl_driver.lock);
	unregister_spi_chip_driver(&pilotavl_driver);
	return;
}


module_init(pilotavl_init);
module_exit(pilotavl_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("American Megatrends Inc");
MODULE_DESCRIPTION("MTD SPI driver for ST M25Pxx flash chips");

#endif
