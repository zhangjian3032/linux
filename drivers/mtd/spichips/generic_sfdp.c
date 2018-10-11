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

#include "spiflash.h"

#ifdef	CFG_FLASH_SPI_DRIVER

#include <pilot4spi.h>
// Defining opcode2dummy look up table
static const unsigned int opcode2dummy[0xff + 1] = {
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0,	// 0x00 - 0x0f
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	// 0x10 - 0x1f
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	// 0x20 - 0x2f
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0,	// 0x30 - 0x3f
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	// 0x40 - 0x4f
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,	// 0x50 - 0x5f
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	// 0x60 - 0x6f
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	// 0x70 - 0x7f
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	// 0x80 - 0x8f
		 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	// 0x90 - 0x9f
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0,	// 0xa0 - 0xaf
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,	// 0xb0 - 0xbf
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	// 0xc0 - 0xcf
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2,	// 0xd0 - 0xdf
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 2,	// 0xe0 - 0xef
		 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0		// 0xf0 - 0xff
	};

static int wait_till_ready(int bank,struct spi_ctrl_driver *ctrl_drv,unsigned long timeout);
static int wait_till_flag_status_ready(int bank,struct spi_ctrl_driver *ctrl_drv);
static 
int inline 
spi_error(int retval)
{
	printk("SPI Chip %s (%d) : Error (%d)\n",__FILE__,__LINE__,retval);
	return retval;
}


int 
spi_generic_read_status(int bank, struct spi_ctrl_driver *ctrl_drv,unsigned char *status)
{
	int  retval;
	u8 code = OPCODE_RDSR;

	/* Issue Controller Transfer Routine */
	struct spi_info_cmd cmd_info;
    cmd_info.cmd = code;
    cmd_info.cmdlen = 1;
	cmd_info.addr = 0;
    cmd_info.dir = SPI_READ;
    cmd_info.dummycycles = opcode2dummy[(int)cmd_info.cmd];
    cmd_info.fastread = 0;
    retval = ctrl_drv->spi_transfer (bank, &cmd_info, status, 1, 0);

	if (retval < 0) 
		return spi_error(retval);

	return 0;
}

int 
spi_generic_read_flag_status(int bank, struct spi_ctrl_driver *ctrl_drv,unsigned char *status)
{
	int  retval;
	u8 code = OPCODE_FLG_STS_RD;

	/* Issue Read Flag Status register command */
    struct spi_info_cmd cmd_info;
    cmd_info.cmd = code;
    cmd_info.cmdlen = 1;
	cmd_info.addr = 0;
    cmd_info.dir = SPI_READ;
    cmd_info.dummycycles = opcode2dummy[(int)cmd_info.cmd];
    cmd_info.fastread = 0;
    retval = ctrl_drv->spi_transfer (bank, &cmd_info, status, 1, 0);

	if (retval < 0) 
		return spi_error(retval);

	return 0;
}

int 
spi_generic_write_status(int bank,struct spi_ctrl_driver *ctrl_drv, unsigned char status)
{
	int retval;
	u8 code = OPCODE_WRSR;
	struct spi_info_cmd cmd_info;

	/* Send write enable */
	spi_generic_write_enable(bank,ctrl_drv);

	/* Issue Controller Transfer Routine */
    cmd_info.cmd = code;
    cmd_info.cmdlen = 1;
	cmd_info.addr = 0;
    cmd_info.dir = SPI_WRITE;
    cmd_info.dummycycles = opcode2dummy[(int)cmd_info.cmd];
    cmd_info.fastread = 0;
    retval = ctrl_drv->spi_transfer (bank, &cmd_info, &status, 1, 0);
	if (retval < 0) 
		return spi_error(retval);

        if (wait_till_ready(bank,ctrl_drv,MAX_READY_WAIT_COUNT))
        {
                return -1;
        }
	return 0;
}

int 
spi_generic_enable_4byte_mode(int bank,struct spi_ctrl_driver *ctrl_drv)
{
	u8 code = OPCODE_4BYTEEN;
	int retval;
	struct spi_info_cmd cmd_info;

	/* Send write enable before sending enable 4-byte mode*/
	spi_generic_write_enable(bank,ctrl_drv);

	/* Issue Enable 4byte addressing mode */
    cmd_info.cmd = code;
    cmd_info.cmdlen = 1;
	cmd_info.addr = 0;
    cmd_info.dir = SPI_NONE;
    cmd_info.dummycycles = opcode2dummy[(int)cmd_info.cmd];
    cmd_info.fastread = 0;
    retval = ctrl_drv->spi_transfer (bank, &cmd_info, NULL, 0, 0);
	if (retval < 0) 
		return spi_error(retval);
	return 0;
}

int 
spi_generic_disable_4byte_mode(int bank, struct spi_ctrl_driver *ctrl_drv)
{
	u8 code = OPCODE_4BYEDIS;
	int retval;
	struct spi_info_cmd cmd_info;

	/* Send write enable before sending disable 4-byte mode*/
	spi_generic_write_enable(bank,ctrl_drv);

	/* Issue Disable 4byte addressing mode */
    cmd_info.cmd = code;
    cmd_info.cmdlen = 1;
	cmd_info.addr = 0;
    cmd_info.dir = SPI_NONE;
    cmd_info.dummycycles = opcode2dummy[(int)cmd_info.cmd];
    cmd_info.fastread = 0;
    retval = ctrl_drv->spi_transfer (bank, &cmd_info, NULL, 0, 0);
	if (retval < 0) 
		return spi_error(retval);
	return 0;
}

int 
spi_generic_write_enable(int bank,struct spi_ctrl_driver *ctrl_drv)
{
	u8 code = OPCODE_WREN;
	int retval;

	/* Issue Controller Transfer Routine */
    struct spi_info_cmd cmd_info;
    cmd_info.cmd = code;
    cmd_info.cmdlen = 1;
	cmd_info.addr = 0;
    cmd_info.dir = SPI_NONE;
    cmd_info.dummycycles = opcode2dummy[(int)cmd_info.cmd];
    cmd_info.fastread = 0;
    retval = ctrl_drv->spi_transfer (bank, &cmd_info, NULL, 0, 0);
	if (retval < 0) 
		return spi_error(retval);
	return 0;
}

int 
spi_generic_write_disable(int bank, struct spi_ctrl_driver *ctrl_drv)
{
	u8 code = OPCODE_WRDI;
	int retval;

	/* Issue Controller Transfer Routine */
    struct spi_info_cmd cmd_info;
    cmd_info.cmd = code;
    cmd_info.cmdlen = 1;
	cmd_info.addr = 0;
    cmd_info.dir = SPI_NONE;
    cmd_info.dummycycles = opcode2dummy[(int)cmd_info.cmd];
    cmd_info.fastread = 0;
    retval = ctrl_drv->spi_transfer (bank, &cmd_info, NULL, 0, 0);
	if (retval < 0) 
		return spi_error(retval);
	return 0;
}


static int 
wait_till_ready(int bank,struct spi_ctrl_driver *ctrl_drv, unsigned long timeout)
{
	unsigned long  count;
	unsigned char sr;

	for (count = 0; count < timeout; count++) 
	{
		if (spi_generic_read_status(bank,ctrl_drv,&sr) < 0)
		{
			printk("Error reading SPI Status Register\n");
			break;
		}
		else 
		{
			if (!(sr & SR_WIP))
				return 0;
		}
	}

	printk("spi_generic: Waiting for Ready Failed\n");
	return 1;
}

static int 
wait_till_flag_status_ready(int bank,struct spi_ctrl_driver *ctrl_drv)
{
	unsigned long  count;
	unsigned char sr;

	for (count = 0; count < MAX_READY_WAIT_COUNT; count++) 
	{
		if (spi_generic_read_flag_status(bank,ctrl_drv,&sr) < 0)
		{
			printk("Error reading SPI Flag Status Register\n");
			break;
		}
		else 
		{
			if ((sr & SR_FLG_STS_REG) == SR_FLG_STS_REG)
				return 0;
		}
	}

	printk("spi_generic: Waiting for Flag status ready Failed\n");
	return 1;
}

int spi_generic_erase(struct map_info *map, unsigned long sect_addr)
{
    struct spi_flash_private *priv=map->fldrv_priv;
    int bank = map->map_priv_1;
    struct spi_ctrl_driver *ctrl_drv = priv->ctrl_drv;
    int retval = 0;
    bool fbytemode = false;
    bool checkFlagStatus = false;
    struct spi_info_cmd spi_info;

    down(&priv->chip_drv->lock);
    /* Wait until finished previous command. */
    if (wait_till_ready(bank,ctrl_drv,MAX_READY_WAIT_COUNT))
    {
        up(&priv->chip_drv->lock);
        return -1;
    }

    checkFlagStatus = ((priv->flags & SPI_RD_FLAG_STATUS) == SPI_RD_FLAG_STATUS);
    fbytemode = ((priv->flags & SPI_4BYTE_ENABLE_NEEDED) == SPI_4BYTE_ENABLE_NEEDED);

    if (fbytemode)
        spi_generic_enable_4byte_mode(bank,ctrl_drv);

    /* Send write enable */
	spi_generic_write_enable(bank,ctrl_drv);

    /* Set up command buffer. */
    if ((priv->flags & SPI_4BYTE_ONLY) == SPI_4BYTE_ONLY) {
        spi_info.cmd = OPCODE_SE_4B;
    }
    else {
        spi_info.cmd = OPCODE_SE;
    }
    spi_info.cmdlen = ((priv->flags & SPI_4BYTE_PART) == SPI_4BYTE_PART)?5:4;
    spi_info.addr = sect_addr;
    spi_info.dir = SPI_NONE;
    spi_info.dummycycles = opcode2dummy[(int)spi_info.cmd];
    spi_info.fastread = 0;

    /* Issue Controller Transfer Routine */
    retval = ctrl_drv->spi_transfer(bank, &spi_info, NULL, 0, fbytemode);
    if (retval < 0)
    {
        printk("error in erase \n");
        up(&priv->chip_drv->lock);
        return spi_error(retval);
    }

    if (checkFlagStatus)
        wait_till_flag_status_ready(bank, ctrl_drv);

    /* Wait until finished previous command. */
    if (wait_till_ready(bank,ctrl_drv,MAX_READY_WAIT_COUNT))
    {
        up(&priv->chip_drv->lock);
        return -1;
    }

    if (fbytemode) {
        spi_generic_disable_4byte_mode(bank,ctrl_drv);

        if (wait_till_ready(bank,ctrl_drv,MAX_READY_WAIT_COUNT))
        {
            up(&priv->chip_drv->lock);
            return -1;
        }
    }

    up(&priv->chip_drv->lock);
    return retval;
}

int spi_generic_read(struct map_info *map, loff_t addr, size_t bytes, unsigned char *buff)
{
    struct spi_flash_private *priv=map->fldrv_priv;
    int bank = map->map_priv_1;
    struct spi_ctrl_driver *ctrl_drv = priv->ctrl_drv;
    int retval = 0;
    size_t transfer = 0;
    bool fbytemode = false;
    struct spi_info_cmd spi_info;

    int  (*readfn)(int bank, struct spi_info_cmd *, unsigned char *, unsigned long, int);

    fbytemode = ((priv->flags & SPI_4BYTE_ENABLE_NEEDED) == SPI_4BYTE_ENABLE_NEEDED);
	readfn = ctrl_drv->spi_transfer;

    /* Some time zero bytes length are sent */
    if (bytes==0)
        return 0;

    down(&priv->chip_drv->lock);

    if (fbytemode) {
        spi_generic_enable_4byte_mode(bank,ctrl_drv);
    }

    /* Wait until finished previous command. */
    if (wait_till_ready(bank,ctrl_drv,MAX_READY_WAIT_COUNT))
    {
        up(&priv->chip_drv->lock);
        return -1;
    }

	spi_info.cmd = priv->read_opcode;
	spi_info.cmdlen = ((priv->flags & SPI_4BYTE_ENABLE_NEEDED) == SPI_4BYTE_ENABLE_NEEDED)?5:4;
	spi_info.dir = SPI_READ;
	spi_info.dummycycles = opcode2dummy[(int)spi_info.cmd];
	if ((priv->flags & SPI_QUAD_READ) == SPI_QUAD_READ) {
		spi_info.dummycycles = (priv->sfdp_data.basic_param_table.dword3.dummy_fast_read_114
                        + priv->sfdp_data.basic_param_table.dword3.mode_clock_fast_read_114) / 8;
		spi_info.fastread = READ_QUAD;
	}
	else if ((priv->flags & SPI_DUAL_READ) == SPI_DUAL_READ) {
		spi_info.fastread = READ_DUAL;
		spi_info.dummycycles = (priv->sfdp_data.basic_param_table.dword4.dummy_fast_read_112
                        + priv->sfdp_data.basic_param_table.dword4.mode_clock_fast_read_112) / 8;
	}
	else if ((priv->flags & SPI_FAST_READ) == SPI_FAST_READ) {
		spi_info.fastread = READ_FAST;
	}
	

    transfer=bytes;
    while (bytes)
    {
        transfer=bytes;

        /* Set up command buffer. */    /* Normal Read */
        spi_info.addr = addr;

        /* Issue Controller Transfer Routine */
        retval = (*readfn)(bank, &spi_info, buff, (unsigned long)transfer, fbytemode);

        if (retval < 0)
        {
            up(&priv->chip_drv->lock);
            return spi_error(retval);
        }

        if (wait_till_ready(bank,ctrl_drv,MAX_READY_WAIT_COUNT))
        {
            up(&priv->chip_drv->lock);
            return -1;
        }

        bytes-=transfer;
        addr+=transfer;
        buff+=transfer;
    }

    if (fbytemode) {
        spi_generic_disable_4byte_mode(bank,ctrl_drv);

        if (wait_till_ready(bank,ctrl_drv,MAX_READY_WAIT_COUNT))
        {
            up(&priv->chip_drv->lock);
            return -1;
        }
    }

    up(&priv->chip_drv->lock);
    return 0;
}

int spi_generic_write(struct map_info *map, loff_t addr, size_t bytes, const unsigned char *buff)
{
    struct spi_flash_private *priv=map->fldrv_priv;
    int bank = map->map_priv_1;
    struct spi_ctrl_driver *ctrl_drv = priv->ctrl_drv;
    int retval = 0;
    bool fbytemode = false;
    bool checkFlagStatus = false;
    size_t transfer = 0;
    struct spi_info_cmd spi_info;

    /* Some time zero bytes length are sent */
    if (bytes==0)
        return 0;

    down(&priv->chip_drv->lock);

    checkFlagStatus = ((priv->flags & SPI_RD_FLAG_STATUS) == SPI_RD_FLAG_STATUS);
    fbytemode = ((priv->flags & SPI_4BYTE_ENABLE_NEEDED) == SPI_4BYTE_ENABLE_NEEDED);

    if (fbytemode) {
        spi_generic_enable_4byte_mode(bank,ctrl_drv);
    }

    if ((priv->flags & SPI_4BYTE_ONLY) == SPI_4BYTE_ONLY) {
        spi_info.cmd = OPCODE_PP_4B;
    }
    else {
        spi_info.cmd = OPCODE_PP;
    }

    spi_info.cmdlen = ((priv->flags & SPI_4BYTE_PART) == SPI_4BYTE_PART)?5:4;
    spi_info.dir = SPI_WRITE;
    spi_info.dummycycles = opcode2dummy[(int)spi_info.cmd];
    spi_info.fastread = 0;

    while (bytes)
    {
        /* Wait until finished previous command. */
        if (wait_till_ready(bank,ctrl_drv,MAX_READY_WAIT_COUNT))
        {
            up(&priv->chip_drv->lock);
            return -1;
        }

        /* Send write enable */
        spi_generic_write_enable(bank,ctrl_drv);

        transfer = PROGRAM_PAGE_SIZE;
        if (bytes <  transfer)
            transfer = bytes;

        spi_info.addr = addr;

        /* Issue Controller Transfer Routine */
        retval = ctrl_drv->spi_transfer (bank, &spi_info, (unsigned char *)buff, transfer, fbytemode);

        if (retval < 0)
        {
            up(&priv->chip_drv->lock);
            return spi_error(retval);
        }
        addr+=(transfer-retval);
        buff+=(transfer-retval);
        bytes-=(transfer-retval);

        if (checkFlagStatus) {
            wait_till_flag_status_ready(bank, ctrl_drv);
        }

        if (wait_till_ready(bank,ctrl_drv,MAX_READY_WAIT_COUNT))
        {
            up(&priv->chip_drv->lock);
            return -1;
        }
    }

    if (fbytemode) {
        spi_generic_disable_4byte_mode(bank,ctrl_drv);

        if (wait_till_ready(bank,ctrl_drv,MAX_READY_WAIT_COUNT))
        {
            up(&priv->chip_drv->lock);
            return -1;
        }
    }

    up(&priv->chip_drv->lock);
    return 0;
}

/***********************************************************************************/
extern int spi_verbose;
int spi_generic_probe(int bank,struct spi_ctrl_driver *ctrl_drv, struct spi_flash_info *chip_info,
			char *spi_name,struct spi_flash_info *spi_list, int spi_list_len)
{
	int  retval;
	unsigned char idcode[5];
	struct SFDP_DATA sfdp_data;
	struct spi_info_cmd cmd_info;
	unsigned long flags;
	unsigned int read_opcode;
	int numeraseregions;
	unsigned long size = sizeof (struct SFDPHeader) + sizeof (struct ParameterHeader);

	/* Send write enable */
	retval =spi_generic_write_enable(bank,ctrl_drv);
	if (retval < 0) 
	 	return -1;

	//The above write enable is not needed so we are making sure that the Chip is ready before issuing READ ID
	// We will send a smaller timout value during probe so that the control is not held for long when there is
	// no device present at a CS. Larger timeouts are needed only for erase/write operations
	wait_till_ready(bank,ctrl_drv,4000); 

	cmd_info.cmd = OPCODE_RDID;
	cmd_info.cmdlen = 1;
	cmd_info.addr = 0;
	cmd_info.dir = SPI_READ;
	cmd_info.dummycycles = opcode2dummy[(int)cmd_info.cmd];
	cmd_info.fastread = 0;
	retval = ctrl_drv->spi_transfer(bank, &cmd_info, idcode, 5, 0);
	if (retval < 0) 
	{
		spi_error(retval);
		return -1;
	}

	// Setting SFDP Header and Basic Parameter Header details
	memset (&cmd_info, 0, sizeof (cmd_info));
	cmd_info.cmd = OPCODE_RDSFDP;
	cmd_info.cmdlen = 4;
	cmd_info.dummycycles = opcode2dummy[(int)cmd_info.cmd];
	cmd_info.addr = 0;
	cmd_info.dir = SPI_READ;
	cmd_info.fastread = 0;
	retval = ctrl_drv->spi_transfer (bank, &cmd_info, (unsigned char*)&sfdp_data.head, size, 0);
	if (retval < 0)
	{
		printk (KERN_INFO "Error reading SFDP Header and standard Parameter Header\n");
		return -1;
	}

	// Verifying SFDP Signature should be SFDP (0x50444653)
	if (sfdp_data.head.signature != 0x50444653) {
		printk (KERN_INFO "Signature of device not verified, got: %08lx\n",
				sfdp_data.head.signature);
		return -1;
	}

	// Setting all bytes of Basic Parameter Table to its default value (0xFF)
	memset (&sfdp_data.basic_param_table.dword1, 0xff, sizeof (struct Basic_Flash_Parameter));

	// Setting all Basic Flash Parameter Header DWORD values
	cmd_info.cmd = OPCODE_RDSFDP;
	cmd_info.cmdlen = 4;
	cmd_info.addr = sfdp_data.param_head.ptable_pointer;
	cmd_info.dir = SPI_READ;
	cmd_info.dummycycles = opcode2dummy[(int)cmd_info.cmd];
	cmd_info.fastread = 0;
	retval = ctrl_drv->spi_transfer (bank, &cmd_info,
			(unsigned char*)&sfdp_data.basic_param_table.dword1,
			sfdp_data.param_head.plength * 4, 0);
	if (retval) {
		printk (KERN_INFO "Unable to read DWORDS of Basic Parameter Header\n");
		return -1;
	}

	size = (sfdp_data.basic_param_table.dword2.flash_mem_density + 1)/8;
	numeraseregions = size / (64 * 1024);

	flags = 0;
	if (sfdp_data.basic_param_table.dword1.address_bytes == 0x0) {
		flags |= SPI_3BYTE_ONLY;
	}
    else if (sfdp_data.basic_param_table.dword1.address_bytes == 0x2) {
		flags |= SPI_4BYTE_ONLY | SPI_4BYTE_PART;
	}
    else if (sfdp_data.basic_param_table.dword1.address_bytes == 0x1)
	{
		flags |= SPI_4BYTE_PART;
		if (size > 16 * 1024 * 1024) {
			flags |= SPI_4BYTE_ENABLE_NEEDED;
		}
	}

	read_opcode = 0;
	// Setting up of read_opcode value and flags for quad, dual, fast or standard read
	// read_opcode value will be used in "spi_common_read" while setting Command in command
	// register. Can be put in other modes by changing the "read_opcode" and flags
	// accordingly in "spi_flash_probe_sfdp"
	if (sfdp_data.basic_param_table.dword1.fast_read_114 == 1)
	{
		// Quad read (1-1-4)
		if ((flags & SPI_4BYTE_ONLY) == SPI_4BYTE_ONLY) {
			// JESD216B explicitly tells if the device supports
			// "4B quad read". Since we are compliant to JESD216 only,
			// we assume that if the device is 4B only and if it supports
			// quad mode then following will be the opcode.
            read_opcode = OPCODE_QUAD_READ_4B;
		}
		else {
			read_opcode = sfdp_data.basic_param_table.dword3.ins_fast_read_114;
		}
		flags |= SPI_QUAD_READ;
	}
	else if (sfdp_data.basic_param_table.dword1.fast_read_112 == 1)
	{
		// Dual read (1-1-2)
		if ((flags & SPI_4BYTE_ONLY) == SPI_4BYTE_ONLY) {
			// JESD216B explicitly tells if the device supports
			// "4B dual read". Since we are compliant to JESD216 only,
			// we assume that if the device is 4B only and if it supports
			// dual mode then following will be the opcode.
			read_opcode = OPCODE_DUAL_READ_4B;
		}
		else {
			read_opcode = sfdp_data.basic_param_table.dword4.ins_fast_read_112;
		}
		flags |= SPI_DUAL_READ;
	}
	else
	{
		// Fast read
		if ((flags & SPI_4BYTE_ONLY) == SPI_4BYTE_ONLY) {
			read_opcode = OPCODE_FAST_READ_4B;
		}
		else {
			read_opcode = OPCODE_FAST_READ;
		}
		flags |= SPI_FAST_READ;
	}

	if (idcode[0] == 0x20) {
		flags |= SPI_RD_FLAG_STATUS;
	}

	struct spi_flash_info chipinfo = { NULL, idcode[0], (idcode[1] << 8) | idcode[2],
			size, flags, 1, {{0, 64 * 1024, numeraseregions},}, sfdp_data, read_opcode};
	memcpy(chip_info, &chipinfo, sizeof(struct spi_flash_info));

	if (spi_verbose > 0) {
		printk(KERN_INFO "SFDP device with idcode: 0x%02x%02x%02x%02x%02x"
				" size: 0x%08lx flags: 0x%08lx\n", idcode[0], idcode[1], idcode[2],
				idcode[3], idcode[4], chip_info->size, chip_info->flags);
	}

	return 0;

}

EXPORT_SYMBOL(spi_generic_probe);
EXPORT_SYMBOL(spi_generic_erase);
EXPORT_SYMBOL(spi_generic_read);
EXPORT_SYMBOL(spi_generic_write);
EXPORT_SYMBOL(spi_generic_write_disable);
EXPORT_SYMBOL(spi_generic_write_enable);
EXPORT_SYMBOL(spi_generic_enable_4byte_mode);
EXPORT_SYMBOL(spi_generic_disable_4byte_mode);
EXPORT_SYMBOL(spi_generic_read_status);
EXPORT_SYMBOL(spi_generic_read_flag_status);
EXPORT_SYMBOL(spi_generic_write_status);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("American Megatrends Inc");
MODULE_DESCRIPTION("MTD SPI driver for Generic SPI flash chips");

#endif
