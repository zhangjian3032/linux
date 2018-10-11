
/*
* Copyright (C) 2007 American Megatrends Inc.
*
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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/concat.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include "../pilot4spi_flash.h"

#ifdef __UBOOT__
#include "soc_hw.h"
#endif

#define PILOT_SPI_BMISC_REG 	(SE_BOOT_SPI_VA_BASE + 0x1C)
#define PILOT_SPI_BKUPMISC_REG	(SE_BACKUP_SPI_VA_BASE  + 0x1C)
#define PILOT_SPI_HMISC_REG 	(SE_HOST_SPI_VA_BASE + 0x1C)

extern unsigned int IO_ADDRESS(unsigned int);

extern int add_mtd_partitions(struct mtd_info *master,
		       const struct mtd_partition *parts,
		       int nbparts);
extern int del_mtd_partitions(struct mtd_info *master);

static struct mtd_partition pilot4_mtd_flash_partitions[] ={
// BootSPI-CS0
  {
    .name =         "BootLoader",
    .size =         0x00080000,     /* hopefully u-boot will stay 128k + 128*/
    .offset =       0,
  },
  {
    .name =         "SDRR Area",
    .size =         0x00080000,     /* BootSPI SDRR Aread */
    .offset =       MTDPART_OFS_APPEND,
  },
  {
    .name =         "Kernel 1",
    .size =         0x00300000,     /* BootSPI Kernel aread */
    .offset =       MTDPART_OFS_APPEND,
  }, 
  {
    .name =         "Ramdisk 1",
    .size =         0x00a00000,     /* BootSPI Ramdisk size */
    .offset =       MTDPART_OFS_APPEND,
  },
  {
    .name =         "NVCFG Data",
    .size =         0x001c0000,     /* BootSPI region to store Non-volatile data */
    .offset =       MTDPART_OFS_APPEND,
  },
  {
    .name =         "RawNVCFG",
    .size =         0x0040000,     /* BootSPI region to store Non-volatile data */
    .offset =       MTDPART_OFS_APPEND,
  },

  // BootSPI-CS1
  {
    .size =         MTDPART_SIZ_FULL,
    .offset =       MTDPART_OFS_APPEND,
  },

  // BootSPI-CS2
  {
    .size =         MTDPART_SIZ_FULL,
    .offset =       MTDPART_OFS_APPEND,
  }, 

  // BkupSPI-CS0
  {
    .size =         MTDPART_SIZ_FULL,
    .offset =       MTDPART_OFS_APPEND,
  },
  // BkupSPI-CS1
  {
    .size =         MTDPART_SIZ_FULL,
    .offset =       MTDPART_OFS_APPEND,
  },
  // BkupSPI-CS2
  {
    .size =         MTDPART_SIZ_FULL,
    .offset =       MTDPART_OFS_APPEND,
  },
  // HSPI-CS0
  {
    .size =         MTDPART_SIZ_FULL,
    .offset =       MTDPART_OFS_APPEND,
  }, 
  // HSPI-CS1
  {
    .size =         MTDPART_SIZ_FULL,
    .offset =       MTDPART_OFS_APPEND,
  },
};

unsigned long INITAL_PARITION_IN_BSPI_CS0 = 6;

char* bspi_cs[] = {"NULL", "BootSPI-CS1", "BootSPI-CS2"};
char* bkupspi_cs[] = {"BkupSPI-CS0", "BkupSPI-CS1", "BkupSPI-CS2"};
char* hspi_cs[] = {"HostSPI-CS0", "HostSPI-CS1"};

struct map_info pilot4_spi_flash_map[CONFIG_SYS_MAX_SPI_BANKS];
static struct mtd_info *pilot4_spi_mtd[CONFIG_SYS_MAX_SPI_BANKS];
static struct mtd_info *concat_mtd = NULL;
static unsigned long bankcount= 0;


// The SPI controller at Bank 0 is mapped to 3 chip selects. The BMISC register
// gets programmed with address ranges for the 3 chip selects during flash_init.
// Internally each bank represents one spi device. So to R/W to CS1, one has to
// use address at Bank 1. But get_bank_from_address will pass the 'offset' value to the
// flash's interface. So we have to add the start address of the current bootspi
// bank to get to the right address. Hence the use of this array.
// Eg: Consider CS0 16MB, CS1 16MB, CS2 16MB. If the user does 'md 1000100',
// the flash->read will be called with offset value 100. So the correction value of
// 1000000 will be taken from this array and pilot4_boot_spi_transfer will program
// the address register accordingly.
ulong bootspi_bank_addr_correction[CONFIG_SYS_MAX_BOOT_SPI_BANKS] = {0};
ulong hostspi_bank_addr_correction[CONFIG_SYS_MAX_HOST_SPI_BANKS] = {0};
ulong bkupspi_bank_addr_correction[CONFIG_SYS_MAX_BKUP_SPI_BANKS] = {0};

void
spi_map_flash_priv_init(int index, struct mtd_info* mtd, unsigned long totalSize)
{
	pilot4_spi_flash_map[index].phys = CONFIG_SYS_FLASH_BASE + totalSize;
	pilot4_spi_flash_map[index].size = mtd->size;

	simple_map_init(&pilot4_spi_flash_map[index]);	
	pilot4_spi_mtd[index]->owner = THIS_MODULE;
}

int __init 
init_pilot4_spi_map_flash(void)
{
	unsigned long totalSize = 0, tmp_size = 0;
	int bank = -1, i, temp;
	struct mtd_info *spi_mtd_temp[CONFIG_SYS_MAX_SPI_BANKS];
	int cs_size[CONFIG_SYS_MAX_BOOT_SPI_BANKS];
	int partition_count = INITAL_PARITION_IN_BSPI_CS0;

	cs_size[0] = cs_size[1] = cs_size[2] = 0;
	for (i = 0; i < CONFIG_MAX_BOOT_SPI_BANKS; i++)
	{
		bank++;

		// If all CSs size exceedsd the configured limit or,
		// if CS0 has a part > 252MB then do not proceed further.
		if (pilot4_spi_flash_map[0].size  == CONFIG_PROJ_MAX_SPICS0_SIZE ||
			totalSize >= CONFIG_PROJ_MAX_BOOTSPI_SIZE)
			continue;

		temp = *(unsigned long*)(PILOT_SPI_BMISC_REG);
		temp &= ~(0xC0000000);
		*(unsigned long*)(PILOT_SPI_BMISC_REG) = (temp | (bank << 30));

		printk("Probing for Flash at Bank # %d\n",bank);
		pilot4_spi_flash_map[bank].name = "Pilot4 SPI MAP";
		pilot4_spi_flash_map[bank].map_priv_1 = bank;
		pilot4_spi_flash_map[bank].bankwidth = 2;

		pilot4_spi_mtd[bank] = do_map_probe("spi_probe",&pilot4_spi_flash_map[bank]);

		if (!pilot4_spi_mtd[bank])
		{
			if (totalSize == 0)
			{
				printk("ERROR: %s: flash probe failed\n", __FUNCTION__);
				return -ENXIO;
			}
			continue;
		}

		spi_map_flash_priv_init(bank, pilot4_spi_mtd[bank], totalSize);

		if (bank > 0)
		{
			// See if the overall size exceeds the max limit. If so trim it to the max limit
			if (totalSize + pilot4_spi_mtd[bank]->size > CONFIG_PROJ_MAX_BOOTSPI_SIZE)
			{
				pilot4_spi_mtd[bank]->size = CONFIG_PROJ_MAX_BOOTSPI_SIZE - totalSize;

				// numeraseregions is always 1
				pilot4_spi_mtd[bank]->eraseregions[0].numblocks = 
					(uint32_t)pilot4_spi_mtd[bank]->size / pilot4_spi_mtd[bank]->eraseregions[0].erasesize;

				printk("Size exceeded max limit: CS:%d trimmed to:0x%llu num_blocks:%d\n", bank,
						pilot4_spi_mtd[bank]->size, pilot4_spi_mtd[bank]->eraseregions[0].numblocks);
			}
		}


		cs_size[bank] = (pilot4_spi_mtd[bank]->size/(1024*1024));


		if (bank == 0)
		{
			pilot4_mtd_flash_partitions[partition_count - 1].size =
				(pilot4_spi_mtd[bank]->size - (pilot4_mtd_flash_partitions[0].size +
											   pilot4_mtd_flash_partitions[3].size +
											   pilot4_mtd_flash_partitions[4].size +
											   pilot4_mtd_flash_partitions[1].size +
											   pilot4_mtd_flash_partitions[2].size));
		}
		else
		{
			partition_count++;
			pilot4_mtd_flash_partitions[partition_count-1].size = pilot4_spi_mtd[bank]->size;
			pilot4_mtd_flash_partitions[partition_count-1].name = bspi_cs[bank];
		}

		spi_mtd_temp[bankcount] = pilot4_spi_mtd[bank];
		bootspi_bank_addr_correction[bank] = totalSize;
		totalSize += pilot4_spi_mtd[bank]->size;
		bankcount++;
	}

	temp = *(unsigned long*)(PILOT_SPI_BMISC_REG);
	temp &= 0xFFC0C0C0;
	*(unsigned long*)(PILOT_SPI_BMISC_REG) = temp | ((cs_size[0] >> 2) |
			(((cs_size[0]  + cs_size[1]) >> 2) << 8) |
			(((cs_size[0] + cs_size[1] + cs_size[2]) >> 2) <<16));

	*(unsigned long*)(PILOT_SPI_BMISC_REG) &= ~(0xC0000000); // enable cs-0 by default
	*(unsigned long*)(PILOT_SPI_BMISC_REG) |= (0xC0000000);  // Make it no override

#if (CONFIG_MAX_BKUP_SPI_BANKS > 0)
	bank = CONFIG_SYS_MAX_BOOT_SPI_BANKS - 1;
	cs_size[0] = cs_size[1] = cs_size[2] = 0;
	for (i = 0; i < CONFIG_MAX_BKUP_SPI_BANKS; i++)
	{
		bank++;
		cs_size[i] = 0;

		temp = *(unsigned long*)(PILOT_SPI_BKUPMISC_REG);
		temp &= ~(0xC0000000);
		*(unsigned long*)(PILOT_SPI_BKUPMISC_REG) = (temp | (i << 30));

		printk("Probing for Flash at Bank # %d\n",bank);
		pilot4_spi_flash_map[bank].name = "Pilot4 SPI MAP";
		pilot4_spi_flash_map[bank].map_priv_1 = bank;
		pilot4_spi_flash_map[bank].bankwidth = 1;
		pilot4_spi_mtd[bank] = NULL;

		pilot4_spi_mtd[bank] = do_map_probe("spi_probe",&pilot4_spi_flash_map[bank]);

		if (!pilot4_spi_mtd[bank]) 
			continue;

		cs_size[i] = (pilot4_spi_mtd[bank]->size/(1024*1024));
		spi_map_flash_priv_init(bank, pilot4_spi_mtd[bank], totalSize);
		bkupspi_bank_addr_correction[i] = tmp_size;
		partition_count++;
		pilot4_mtd_flash_partitions[partition_count-1].size = pilot4_spi_mtd[bank]->size;
		pilot4_mtd_flash_partitions[partition_count-1].name = bkupspi_cs[i];

		spi_mtd_temp[bankcount] = pilot4_spi_mtd[bank];
		totalSize += pilot4_spi_mtd[bank]->size;
		tmp_size += pilot4_spi_mtd[bank]->size;

		bankcount++;
	}
	temp = *(unsigned long*)(PILOT_SPI_BKUPMISC_REG);
	temp &= 0xFFC0C0C0;
	*(unsigned long*)(PILOT_SPI_BKUPMISC_REG) = temp | ((cs_size[0] >> 2) |
			(((cs_size[0]  + cs_size[1]) >> 2) << 8) |
			(((cs_size[0] + cs_size[1] + cs_size[2]) >> 2) <<16));

	*(unsigned long*)(PILOT_SPI_BKUPMISC_REG) &= ~(0xC0000000); // enable cs-0 by default
	*(unsigned long*)(PILOT_SPI_BKUPMISC_REG) |= (0xC0000000);  // Make it no override
#endif

#if (CONFIG_MAX_HOST_SPI_BANKS > 0)
	tmp_size = 0;
	bank = (CONFIG_SYS_MAX_BOOT_SPI_BANKS + CONFIG_SYS_MAX_BKUP_SPI_BANKS) - 1;
	cs_size[0] = cs_size[1] = cs_size[2] = 0;
	for (i = 0; i < CONFIG_MAX_HOST_SPI_BANKS; i++)
	{
		bank++;
		cs_size[i] = 0;

		temp = *(unsigned long*)(PILOT_SPI_HMISC_REG);
		temp &= ~(0xC0000000);
		*(unsigned long*)(PILOT_SPI_HMISC_REG) = (temp | (i << 30));

		printk("Probing for Flash at Bank # %d\n",bank);
		pilot4_spi_flash_map[bank].name = "Pilot4 SPI MAP";
		pilot4_spi_flash_map[bank].map_priv_1 = bank;
		pilot4_spi_flash_map[bank].bankwidth = 1;
		pilot4_spi_mtd[bank] = NULL;

		pilot4_spi_mtd[bank] = do_map_probe("spi_probe",&pilot4_spi_flash_map[bank]);

		if (!pilot4_spi_mtd[bank]) 
			continue;

		cs_size[i] = (pilot4_spi_mtd[bank]->size/(1024*1024));
		spi_map_flash_priv_init(bank, pilot4_spi_mtd[bank], totalSize);
		hostspi_bank_addr_correction[i] = tmp_size;
		partition_count++;
		pilot4_mtd_flash_partitions[partition_count-1].size = pilot4_spi_mtd[bank]->size;
		pilot4_mtd_flash_partitions[partition_count-1].name = hspi_cs[i];

		spi_mtd_temp[bankcount] = pilot4_spi_mtd[bank];
		totalSize += pilot4_spi_mtd[bank]->size;
		tmp_size += pilot4_spi_mtd[bank]->size;
		bankcount++;
	}
	temp = *(unsigned long*)(PILOT_SPI_HMISC_REG);
	temp &= 0xFF000000;
	*(unsigned long*)(PILOT_SPI_HMISC_REG) = temp | ((cs_size[0] >> 2) |
			(((cs_size[0]  + cs_size[1]) >> 2) << 8) | (((cs_size[0]  + cs_size[1]) >> 2) << 16));
	*(unsigned long*)(PILOT_SPI_HMISC_REG) &= ~(0xC0000000); // enable cs-0 by default
	*(unsigned long*)(PILOT_SPI_HMISC_REG) |= (0xC0000000);  // Make it no override

#endif

	if (bank > 1)	
		concat_mtd = mtd_concat_create(spi_mtd_temp,bankcount,"Concat Ractrends");
	else	
		concat_mtd = pilot4_spi_mtd[0];

	if (!concat_mtd)
	{
		printk("ERROR: %s: flash concat failed\n", __FUNCTION__);
		return -ENXIO;
	}

	return add_mtd_partitions(concat_mtd,pilot4_mtd_flash_partitions,
			(partition_count)); //After adding SDRR area
}

static void __exit 
cleanup_pilot4_spi_map_flash(void)
{
	unsigned long bank;
	if (bankcount > 1)
	{
		del_mtd_partitions(concat_mtd);
		map_destroy(concat_mtd);
		concat_mtd = NULL;

	}

	for (bank = 0; bank < CONFIG_SYS_MAX_SPI_BANKS; bank++)
	{
		if (pilot4_spi_mtd[bank])
		{
			del_mtd_partitions(pilot4_spi_mtd[bank]);
			map_destroy(pilot4_spi_mtd[bank]);
			pilot4_spi_mtd[bank] = NULL;
			pilot4_spi_flash_map[bank].virt = 0;
		}
	}

}

module_init(init_pilot4_spi_map_flash);
module_exit(cleanup_pilot4_spi_map_flash);

MODULE_AUTHOR("Gnanasekar. Emulex.");
MODULE_DESCRIPTION("MTD map driver for the Pilot4 ASIC Board");
