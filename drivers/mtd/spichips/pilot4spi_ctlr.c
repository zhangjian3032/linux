/*
 * Copyright (C) 2007 American Megatrends Inc
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

#include <linux/dma-mapping.h>
#include "spiflash.h"

#include "pilot4spi.h"
#include "../pilot4spi_flash.h"

#define SPI_SECTOR_ERASE			0xd8
#define SPI_ENABLE_4BYTE_ADDR		0xb7
#define SPI_DISABLE_4BYTE_ADDR		0xe9
#define SPI_FASTREAD				0x0b
#define SPI_4BREAD					0x13    /* Read from 4-byte address */
#define SPI_4B_FASTREAD				0xC     /* Fast Read from 4-byte address */
#define SPI_4BWRITE					0x12    /* Write to 4byte address */
#define SPI_BRWR                    0x17

/* Define max times to check status register before we give up. */
#define MAX_READY_WAIT_COUNT		4000000

#define BOOT_SPI_CONTROLLER_INDEX	0
#define BKUP_SPI_CONTROLLER_INDEX	1
#define HOST_SPI_CONTROLLER_INDEX	2
#define MAX_SPI_CONTROLLERS			(HOST_SPI_CONTROLLER_INDEX + 1)


// Pilot supports multiple SPI busses.  Bank 0 is the Boot SPI, Bank 1 is the Backup/Data SPI
// Route any access to bank 1 to the backup SPI code
#ifdef CONFIG_SFDP
int pilot4_boot_bmc_spi_transfer(int bank, struct spi_info_cmd *spi_info,
		unsigned char *data, unsigned long datalen, int fbyteaddr);
int pilot4_host_spi_transfer(int bank, struct spi_info_cmd *spi_info,
		unsigned char *data, unsigned long datalen, int fbyteaddr);
int pilot4spi_transfer(int bank, struct spi_info_cmd *spi_info,
        unsigned char *data, unsigned long datalen, int fbyteaddr);
#else
int pilot4_boot_bmc_spi_transfer(int bank,unsigned char *cmd,int cmdlen, SPI_DIR dir,
		unsigned char *data, unsigned long datalen, int fbyteaddr);
int pilot4_host_spi_transfer(int bank,unsigned char *cmd,int cmdlen, SPI_DIR dir,
		unsigned char *data, unsigned long datalen, int fbyteaddr);
int pilot4spi_transfer(int bank,unsigned char *cmd,int cmdlen, SPI_DIR dir,
		unsigned char *data, unsigned long datalen, int fbyteaddr);
#endif

void pilot_change_spi_source_clk100mhz(unsigned int spi_no);
void pilot_change_spi_source_clk50mhz(unsigned int spi_no);

#ifdef CONFIG_SFDP
int  (*pilot_spi_transfer[CONFIG_SYS_MAX_SPI_BANKS])(int bank, struct spi_info_cmd *spi_info,
		unsigned char *data, unsigned long datalen, int fbyteaddr) =
#else
int  (*pilot_spi_transfer[CONFIG_SYS_MAX_SPI_BANKS])(int bank,unsigned char *cmd,int cmdlen,
		SPI_DIR dir, unsigned char *data, unsigned long datalen, int fb) = 
#endif
{
	pilot4_boot_bmc_spi_transfer,  // Bank 0
	pilot4_boot_bmc_spi_transfer,  // Bank 1
	pilot4_boot_bmc_spi_transfer,  // Bank 2
	pilot4_boot_bmc_spi_transfer,  // Bank 3
	pilot4_boot_bmc_spi_transfer,  // Bank 4
	pilot4_boot_bmc_spi_transfer,  // Bank 5
	pilot4_host_spi_transfer,  // Bank 6
	pilot4_host_spi_transfer   // Bank 7
};

extern ulong bootspi_bank_addr_correction[CONFIG_SYS_MAX_BOOT_SPI_BANKS];
extern ulong bkupspi_bank_addr_correction[CONFIG_SYS_MAX_BKUP_SPI_BANKS];
extern ulong hostspi_bank_addr_correction[CONFIG_SYS_MAX_HOST_SPI_BANKS];
#define BKUPSPI_BANK_START		(CONFIG_SYS_MAX_BOOT_SPI_BANKS)
#define HSPI_BANK_START			(BKUPSPI_BANK_START + CONFIG_SYS_MAX_BKUP_SPI_BANKS)

typedef struct 
{
	volatile union
	{
		volatile struct
		{
			volatile unsigned char Addr0;           /* LSB */
			volatile unsigned char Addr1;
			volatile unsigned char Addr2;           /* MSB */
			volatile unsigned char Addr3;			/* MSB (4 byte addressing mode) */
		};
		volatile unsigned long Addr;
	};
	volatile unsigned long pilot4_Dummy;
	volatile union {
		volatile struct {
			volatile unsigned char Opcode;
			volatile unsigned char CmdType;
			volatile unsigned char Dummy;
			volatile unsigned char CmdCtrl;
		};
		volatile unsigned long Command;
	};
	volatile unsigned char Trec;
	volatile unsigned char Tcs;
	volatile unsigned char Tcsl;
	volatile unsigned char Tcsh;
	volatile unsigned char SpiCtrl;
	volatile unsigned char Rsvd2[3];
	volatile unsigned char ClkDivL;
	volatile unsigned char ClkDivH;
	volatile unsigned char Rsvd3[2];
	volatile unsigned long SpiStatus;
	volatile unsigned long BMisc;
	volatile unsigned char Rsvd5[16];//Pilot needs so much of offset as the data register is at offset 0x30
	volatile union {
		volatile unsigned char Data8[4];
		volatile unsigned long Data32;
	} Data;
} SPI_REGS;

// 32 bit Addressing mode in BMISC
#define BMISC_32B_ADDR			(1 << 24)	// Enable 32 addressing mode

// Command register flag in BCMD
#define ENABLE_QUAD_READ		(1 << 30)	// Enable quad read
#define ENABLE_DUAL_READ		(1 << 27)	// Enable dual read
#define ENABLE_FAST_READ		(1 << 26)	// Enable fast read

SPI_REGS *BSPIRegs;
SPI_REGS *HSPIRegs;
SPI_REGS *BKUPSPIRegs;

static int retries = 0;
static void inline
WaitForSpiInternalStateReady(volatile SPI_REGS* Regs)
{
	retries = 10000000;
	while ((Regs->SpiStatus & 0x4) && --retries){
		schedule_timeout(10);
	}

	if (retries == 0)
	{
		printk("Unable to exit while!!! (%s %d)\n", __FUNCTION__, __LINE__);
#ifdef CONFIG_PANIC_AND_BAIL_OUT
		panic("");
#endif
	}
	return;
}

static void inline
WaitForSpiReady(volatile SPI_REGS* Regs)
{
	retries = 10000000;
	while ((Regs->SpiStatus & 0x01) && --retries){
		schedule_timeout(10);
	}
	if (retries == 0)
	{
		printk("Unable to exit while!!! (%s %d)\n", __FUNCTION__, __LINE__);
#ifdef CONFIG_PANIC_AND_BAIL_OUT
		panic("");
#endif
	}

	return;
}

static void inline
WaitForReadFifoNotEmpty(volatile SPI_REGS* Regs)
{
	retries = 10000000;
	/* Wait till Fifo Empty Bit is set */
	while ((Regs->SpiStatus & 0x20) && --retries) {
		schedule_timeout(10);
	}
	if (retries == 0)
	{
		printk("Unable to exit while!!! (%s %d)\n", __FUNCTION__, __LINE__);
#ifdef CONFIG_PANIC_AND_BAIL_OUT
		panic("");
#endif
	}

	return;
}

static void inline
WaitForWriteFifoEmpty(volatile SPI_REGS* Regs)
{
	retries = 10000000;
	/* Wait till Fifo Empty Bit is set */
	while (!(Regs->SpiStatus & 0x20) && --retries) {
		schedule_timeout(10);
	}
	if (retries == 0)
	{
		printk("Unable to exit while!!! (%s %d)\n", __FUNCTION__, __LINE__);
#ifdef CONFIG_PANIC_AND_BAIL_OUT
		panic("");
#endif
	}

	return;
}

static void
SpiEnable(volatile SPI_REGS* Regs, int en)
{
	if (en){
		Regs->SpiCtrl |= 0x01;
	}
	else {
		Regs->SpiCtrl &= 0xFE;
	}
	return;
}

static void noinline
write32(volatile unsigned int* srcAddr, volatile unsigned int* dstAddr)
{
    asm volatile ("push    {r3, r4, r5, r6, r7, r8, r9, r10, r11, lr}");
    asm volatile(
            "mov r0, %0\n\t"
            "mov r1, %1\n\t"
            :
            : "r" (srcAddr), "r" (dstAddr));
    asm volatile ("  LDMIA  r0!, {R2-R9} ");
    asm volatile ("  STMIA  r1!, {R2-R9} ");
    asm volatile ("pop {r3, r4, r5, r6, r7, r8, r9, r10, r11, pc}");
}

static void InitializeCtrl(void)
{
	BSPIRegs = (SPI_REGS *)(SE_BOOT_SPI_VA_BASE);
	HSPIRegs = (SPI_REGS *)(SE_HOST_SPI_VA_BASE);
	BKUPSPIRegs  = (SPI_REGS *)(SE_BACKUP_SPI_VA_BASE);
	printk("Fast clk is set\n");
	// Set fast read bit. This should never get unset because we
	// can run into problems while resetting the board at higher
	// clock, without this bit being set.
	BSPIRegs->CmdCtrl|=0x4;

	/* Disable the ctrl */
	SpiEnable(BSPIRegs, 0);
	// SpiEnable(HSPIRegs, 0);
	SpiEnable(BKUPSPIRegs, 0);

	pilot_change_spi_source_clk100mhz(BOOT_SPI_CONTROLLER_INDEX);

	/* Enable the ctrl */
	SpiEnable(BSPIRegs, 1);
	// SpiEnable(HSPIRegs, 1);
	SpiEnable(BKUPSPIRegs, 1);

	/* Wait for the SPI internal state to be ready
	 * Once the SPI is disabled and enabled bit 2 of
	 * the status register should become 0 which means
	 * the State Machine has completed all its transactions.
	 * It again becomes 1 once the SPI is accessed using
	 * direct/register mode */
	WaitForSpiInternalStateReady(BSPIRegs);
//	WaitForSpiInternalStateReady(HSPIRegs);
	WaitForSpiInternalStateReady(BKUPSPIRegs);

	return;
}

#ifdef CONFIG_SFDP
int pilot4_host_spi_transfer(int bank, struct spi_info_cmd *spi_info, unsigned char *data,
		unsigned long datalen, int fbyteaddr)
{
	unsigned char Type = 0x1;
	unsigned char Dummy = 0;
	unsigned char Ctrl = 0;
	unsigned long Command = 0;
	int i;

	Ctrl = (0x80|HSPIRegs->CmdCtrl);

	HSPIRegs->Addr = spi_info->addr;
	HSPIRegs->Addr3 = hostspi_bank_addr_correction[bank - HSPI_BANK_START] >> 24;

	if (fbyteaddr == 1)
	{
		// Enable 32 bit addressing bit in BMISC and addres correction for 4 byte address mode
		HSPIRegs->BMisc |= BMISC_32B_ADDR;
		HSPIRegs->Addr3 += (spi_info->addr >> 24);
	}

	if (spi_info->dir == SPI_READ) {
		Type = 0x0;
	}

	Command = (spi_info->cmd) |
				((spi_info->cmdlen + spi_info->dummycycles) << 8) |
				(Type << 12) |
				(Dummy << 16) |
				(Ctrl << 24);

	// Configuring controller for quad and dual read
	if ((spi_info->fastread & READ_QUAD) == READ_QUAD) {
		Command |= ENABLE_QUAD_READ;
	}
	else if ((spi_info->fastread & READ_DUAL) == READ_DUAL) {
		Command |= ENABLE_DUAL_READ;
	}
	else if ((spi_info->fastread & READ_FAST) == READ_FAST) {
		Command |= ENABLE_FAST_READ;
	}

	/* Issue Command */
	WaitForSpiReady(HSPIRegs);

	HSPIRegs->Command = Command;

	WaitForSpiReady(HSPIRegs);

	// Do FIFO write for write commands
	if (spi_info->dir == SPI_WRITE)
	{
		for (i = 0; i < datalen; i++)
		{
			WaitForWriteFifoEmpty(HSPIRegs);
			HSPIRegs->Data.Data8[0] = data[i];
		}
	}
	// Read the data from the data register
	else if (spi_info->dir == SPI_READ)
	{
		for (i = 0; i < datalen; i++)
		{
			WaitForReadFifoNotEmpty(HSPIRegs);
			data[i] = HSPIRegs->Data.Data8[0];
		}
	}

	// Reverting controller back to standard mode
	// and resetting all the IO modes, register access bits
	// by unsetting Command
	HSPIRegs->Command = 0;

	// Wait for SPI Controller become Idle
	WaitForSpiReady(HSPIRegs);

	// If a 4 byte read/write command was used then unset the BMISC[24].
	if (fbyteaddr == 1) {
		HSPIRegs->BMisc &= ~BMISC_32B_ADDR;
	}

	return 0;
}

int pilot4_boot_bmc_spi_transfer(int bank, struct spi_info_cmd *spi_info, unsigned char *data,
		unsigned long datalen, int fbyteaddr)
{
    unsigned char Type = 0x1;
    unsigned char Dummy = 0;
    unsigned char Ctrl = 0;
    unsigned long Command = 0;
    int i, aligned_addr = 0, unaligned_addr = 0;
    SPI_REGS* spiregs = 0;

    // See if the address is Dword aligned. If it is aligned then the reads/writes
    // can be optimized further
    if (((unsigned int)data & 0x3) == 0)
    {
        aligned_addr = datalen / 32;
        unaligned_addr = datalen % 32;
    }
    else
        unaligned_addr = datalen;

    // Select the SPI base registers based on the banks. Also populate the
    // address registers along with the correction that needs to be done
    // to account for the BMISC upper addresses programmed for CSs.
    if (bank < CONFIG_SYS_MAX_BOOT_SPI_BANKS)
    {
        spiregs = BSPIRegs;
        spiregs->Addr = spi_info->addr;

        // Populate the 4th byte of address also. If the device is in 3 byte mode
        // then this will not be used anyways. So it is not harmful.
        spiregs->Addr3 += (bootspi_bank_addr_correction[bank] >> 24);
    }
    else
    {
        spiregs = BKUPSPIRegs;
        spiregs->Addr = spi_info->addr;
        spiregs->Addr3 += (bkupspi_bank_addr_correction[bank - BKUPSPI_BANK_START] >> 24);
    }

    Ctrl = (0x80|spiregs->CmdCtrl);

    if (fbyteaddr == 1)
    {
        // Enable 32 bit addressing bit in BMISC and addres correction for 4 byte address mode
        spiregs->BMisc |= BMISC_32B_ADDR;
    }

    if (spi_info->dir == SPI_READ) {
        Type = 0x0;
    }

    Command = (spi_info->cmd) |
        ((spi_info->cmdlen + spi_info->dummycycles) << 8) |
        (Type << 12) |
        (Dummy << 16) |
        (Ctrl << 24);

    // Configuring controller for quad and dual read
    if ((spi_info->fastread & READ_QUAD) == READ_QUAD) {
        Command |= ENABLE_QUAD_READ;
    }
    else if ((spi_info->fastread & READ_DUAL) == READ_DUAL) {
        Command |= ENABLE_DUAL_READ;
    }
    else if ((spi_info->fastread & READ_FAST) == READ_FAST) {
        Command |= ENABLE_FAST_READ;
    }

    WaitForSpiReady(spiregs);

    spiregs->Command = Command;

    WaitForSpiReady(spiregs);

    // Do FIFO write for write commands
    if (spi_info->dir == SPI_WRITE)
    {
        // Do 32byte block copies if the source address was aligned and the data len
        // is more than 32 bytes length.
        while(aligned_addr--)
        {
            write32((unsigned int*)(data), (unsigned int*)&spiregs->Data.Data8[0]);
            data += 32;
        }
        // 1. If there is any unaligned length that is left over from the previous
        //    copy, do it in the following for loop
        // 2. If the destination address is unaligned then do the entire copy in the
        //    following for loop
        for (i = 0; i < unaligned_addr; i++)
            spiregs->Data.Data8[0] = data[i];
    }
    // Read the data from the data register
    else if (spi_info->dir == SPI_READ)
    {
        // Do 32byte block copies if the source address was aligned and the data len
        // is more than 32 bytes length.
        while(aligned_addr--)
        {
            write32((unsigned int*)spiregs->Data.Data8, (unsigned int*)(data));
            data += 32;
        }
        // 1. If there is any unaligned length that is left over from the previous
        //    copy, do it in the following for loop
        // 2. If the destination address is unaligned then do the entire copy in the
        //    following for loop
        for (i = 0; i < unaligned_addr; i++)
            data[i] = spiregs->Data.Data8[0];
    }

    // Wait for SPI Controller become Idle
    WaitForSpiReady(spiregs);

    // Reverting controller back to standard mode
    // and resetting all the IO modes, register access bits
    // by unsetting Command
    spiregs->Command = 0;

    // If a 4 byte read/write command was used then unset the BMISC[24]
    if (fbyteaddr == 1) {
        spiregs->BMisc &= ~BMISC_32B_ADDR;
    }

    return 0;
}

int pilot4spi_transfer(int bank, struct spi_info_cmd *spi_info, unsigned char *data,
		unsigned long datalen, int fbyteaddr)
{
  return (*pilot_spi_transfer[bank])(bank, spi_info, data, datalen, fbyteaddr);
}

#else //CONFIG_SFDP

static inline FBYTE_MODE determine_4bmode(unsigned long opcode, unsigned const char* data) {

	if ((opcode == SPI_ENABLE_4BYTE_ADDR) || (opcode == SPI_4B_FASTREAD) || (opcode == SPI_4BREAD)
			|| (opcode == SPI_4BWRITE))
		return ENTER_FBYTE;

	else if (opcode == SPI_DISABLE_4BYTE_ADDR)
		return EXIT_FBYTE;

    if (opcode ==  SPI_BRWR)
    {
        if (*data == 0x80)
            return ENTER_FBYTE;
        else
            return EXIT_FBYTE;
    }

	return NO_FBYTE;
}

int
pilot4_host_spi_transfer(int bank,unsigned char *cmd,int cmdlen, SPI_DIR dir, 
		unsigned char *data, unsigned long datalen, int fbyteaddr)
{
	unsigned char Opcode;
	unsigned char Type = 0x10;
	unsigned char Dummy;
	unsigned char Ctrl;
	unsigned long Command;
	int i;
	int address_offset = fbyteaddr ? 4 : 3;

	FBYTE_MODE fbyte_mode = determine_4bmode(cmd[0], data);

	Ctrl = (0x80|HSPIRegs->CmdCtrl);
	Dummy = 0;

	// Fill in Command And Address
	Opcode = cmd[0];
	if ((Opcode == SPI_FASTREAD) || (Opcode == SPI_4B_FASTREAD)) {
		Ctrl |= 0x04;
	}

	if (fbyte_mode == ENTER_FBYTE)
		HSPIRegs->BMisc |= BMISC_32B_ADDR;
	else if (fbyte_mode == EXIT_FBYTE)
		HSPIRegs->BMisc &= ~BMISC_32B_ADDR;

	HSPIRegs->Addr3 = hostspi_bank_addr_correction[bank - HSPI_BANK_START] >> 24;

	if (cmdlen >= 4)
	{
		HSPIRegs->Addr0 = cmd[address_offset--];
		HSPIRegs->Addr1 = cmd[address_offset--];	
		HSPIRegs->Addr2 = cmd[address_offset--];
		if (address_offset) {
			HSPIRegs->Addr3 += cmd[address_offset--];
		}
	}

	if (dir == SPI_READ) {
		Type = 0x0;
	}

	Type |= cmdlen;

	Command = (Opcode) | (Type << 8) | (Dummy << 16) | (Ctrl << 24);

	WaitForSpiReady(HSPIRegs);

	HSPIRegs->Command = Command;

	WaitForSpiReady(HSPIRegs);

	// Do FIFO write for write commands
	if (dir == SPI_WRITE)
	{
		for (i = 0; i < datalen; i++)
		{
			WaitForWriteFifoEmpty(HSPIRegs);
			HSPIRegs->Data.Data8[0] = data[i];
		}
	}
	// Read the data from the data register
	else if (dir == SPI_READ)
	{
		for (i = 0; i < datalen; i++)
		{
			WaitForReadFifoNotEmpty(HSPIRegs);
			data[i] = HSPIRegs->Data.Data8[0];
		}
	}

	// Switch back to non-register mode and disable Write Fifo mode
	HSPIRegs->CmdCtrl &= 0x73;

	// Wait for SPI Controller become Idle
	WaitForSpiReady(HSPIRegs);

	// If a 4 byte read/write command was used then unset the BMISC[24].
	if ((Opcode == SPI_4B_FASTREAD) || (Opcode == SPI_4BREAD) || (Opcode == SPI_4BWRITE))
		HSPIRegs->BMisc &= ~BMISC_32B_ADDR;

	return 0;
}

int
pilot4_boot_bmc_spi_transfer(int bank,unsigned char *cmd,int cmdlen, SPI_DIR dir, 
		unsigned char *data, unsigned long datalen, int fbyteaddr)
{
    unsigned char Opcode;
    unsigned char Type = 0x10;
    unsigned char Dummy;
    unsigned char Ctrl;
    unsigned long Command;
    int i, aligned_addr = 0, unaligned_addr = 0;
    int address_offset = fbyteaddr ? 4 : 3;
    SPI_REGS* spiregs = 0;
    FBYTE_MODE fbyte_mode = determine_4bmode(cmd[0], data);

    // See if the address is Dword aligned. If it is aligned then the reads/writes
    // can be optimized further
    if (((unsigned int)data & 0x3) == 0)
    {
        aligned_addr = datalen / 32;
        unaligned_addr = datalen % 32;
    }
    else
        unaligned_addr = datalen;

    if (bank < CONFIG_SYS_MAX_BOOT_SPI_BANKS)
    {
        spiregs = BSPIRegs;
        spiregs->Addr3 = bootspi_bank_addr_correction[bank] >> 24;
    }
    else
    {
        spiregs = BKUPSPIRegs;
        spiregs->Addr3 = bkupspi_bank_addr_correction[bank - BKUPSPI_BANK_START] >> 24;
    }

    Ctrl = (0x80|spiregs->CmdCtrl);
    Dummy = 0;

    /* Fill in Command And Address */
    Opcode = cmd[0];
    if ((Opcode == SPI_FASTREAD) || (Opcode == SPI_4B_FASTREAD)) {
        Ctrl |= 0x04;
    }

    if (fbyte_mode == ENTER_FBYTE)
        spiregs->BMisc |= BMISC_32B_ADDR;
    else if (fbyte_mode == EXIT_FBYTE)
        spiregs->BMisc &= ~BMISC_32B_ADDR;

    if (cmdlen >= 4)
    {
        spiregs->Addr0 = cmd[address_offset--];
        spiregs->Addr1 = cmd[address_offset--];	
        spiregs->Addr2 = cmd[address_offset--];
        if (address_offset) {
            spiregs->Addr3 += cmd[address_offset--];
        }
    }

    if (dir == SPI_READ) {
        Type = 0x0;
    }

    Type |= cmdlen;

    Command = (Opcode) | (Type << 8) | (Dummy << 16) | (Ctrl << 24);

    WaitForSpiReady(spiregs);

    spiregs->Command = Command;

    WaitForSpiReady(spiregs);

    // Do FIFO write for write commands
    if (dir == SPI_WRITE)
    {
        // Do 32byte block copies if the source address was aligned and the data len
        // is more than 32 bytes length.
        while(aligned_addr--)
        {
            write32((unsigned int*)(data), (unsigned int*)&spiregs->Data.Data8[0]);
            data += 32;
        }
        // 1. If there is any unaligned length that is left over from the previous
        //    copy, do it in the following for loop
        // 2. If the destination address is unaligned then do the entire copy in the
        //    following for loop
        for (i = 0; i < unaligned_addr; i++)
            spiregs->Data.Data8[0] = data[i];
    }
    // Read the data from the data register
    else if (dir == SPI_READ)
    {
        // Do 32byte block copies if the source address was aligned and the data len
        // is more than 32 bytes length.
        while(aligned_addr--)
        {
            write32((unsigned int*)&spiregs->Data.Data8[0], (unsigned int*)(data));
            data += 32;
        }
        // 1. If there is any unaligned length that is left over from the previous
        //    copy, do it in the following for loop
        // 2. If the destination address is unaligned then do the entire copy in the
        //    following for loop
        for (i = 0; i < unaligned_addr; i++)
            data[i] = spiregs->Data.Data8[0];
    }

    // Switch back to non-register mode and disable Write Fifo mode
    spiregs->CmdCtrl &= 0x73;

    WaitForSpiReady(spiregs);

    // If a 4 byte read/write command was used then unset the BMISC[24].
    if ((Opcode == SPI_4B_FASTREAD) || (Opcode == SPI_4BREAD) || (Opcode == SPI_4BWRITE))
        spiregs->BMisc &= ~BMISC_32B_ADDR;

    return 0;
}

int
pilot4spi_transfer(int bank,unsigned char *cmd,int cmdlen, SPI_DIR dir, 
		unsigned char *data, unsigned long datalen, int fbyteaddr)
{
	return (*pilot_spi_transfer[bank])(bank, cmd, cmdlen, dir, data, datalen, fbyteaddr);
}
#endif

extern void ll_serial_init(void);

struct spi_ctrl_driver pilot4spi_driver=
{
	.name 		= "pilot4spi",
	.module 	= THIS_MODULE,
	.fast_read	= 1,
	.spi_transfer	= pilot4spi_transfer,
	.spi_burst_read	= NULL,
};


static int 
pilot4spi_init(void)
{
	InitializeCtrl();	
	register_spi_ctrl_driver(&pilot4spi_driver);
	return 0;
}

static void
pilot4spi_exit(void)
{

	pilot_change_spi_source_clk50mhz(BOOT_SPI_CONTROLLER_INDEX);
	pilot_change_spi_source_clk50mhz(BKUP_SPI_CONTROLLER_INDEX);
	pilot_change_spi_source_clk50mhz(HOST_SPI_CONTROLLER_INDEX);

	unregister_spi_ctrl_driver(&pilot4spi_driver);
	return;
}

void pilot_change_spi_source_clk100mhz(unsigned int spi_no)
{
  unsigned int clk_ctrl;
  unsigned int temp;
  unsigned int bitmask[MAX_SPI_CONTROLLERS] = {0xfffffff0, 0xffffff0f, 0xfffff0ff};

  if (spi_no < 0 || spi_no > MAX_SPI_CONTROLLERS-1)
      return;

  clk_ctrl = (int)IO_ADDRESS(0x40100120);
  temp=*(volatile int *)(clk_ctrl);

  // spi_no: 0-BootSPI, 1-BkUpSPI 2-HostSPI
  temp = temp & bitmask[spi_no];
  temp = temp | (1 << (spi_no*4));
  *(volatile int *)(clk_ctrl)=temp;
}


void pilot_change_spi_source_clk50mhz(unsigned int spi_no)
{
  unsigned int clk_ctrl;
  unsigned int temp;
  unsigned int bitmask[MAX_SPI_CONTROLLERS] = {0xfffffff0, 0xffffff0f, 0xfffff0ff};

  if (spi_no < 0 || spi_no > MAX_SPI_CONTROLLERS-1)
      return;

  clk_ctrl = (int)IO_ADDRESS(0x40100120);
  temp=*(volatile int *)(clk_ctrl);

  // spi_no: 0-BootSPI, 1-BkUpSPI 2-HostSPI
  temp = temp & bitmask[spi_no];
  temp = temp | (2 << (spi_no*4));
  *(volatile int *)(clk_ctrl)=temp;
}

module_init(pilot4spi_init);
module_exit(pilot4spi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gnanasekar, Emulex");
MODULE_DESCRIPTION("PILOT-4 SOC SPI Controller driver");

