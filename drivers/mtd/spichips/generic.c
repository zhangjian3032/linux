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

#include "spiflash.h"

#ifdef	CFG_FLASH_SPI_DRIVER

#include "pilot4spi.h"

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
	retval = ctrl_drv->spi_transfer(bank,&code, 1,SPI_READ,status, 1, 0);

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
	retval = ctrl_drv->spi_transfer(bank,&code, 1,SPI_READ,status, 1, 0);

	if (retval < 0) 
		return spi_error(retval);

	return 0;
}

int 
spi_generic_write_status(int bank,struct spi_ctrl_driver *ctrl_drv, unsigned char status)
{
	int retval;
	u8 code = OPCODE_WRSR;

	/* Send write enable */
	spi_generic_write_enable(bank,ctrl_drv);

	/* Issue Controller Transfer Routine */
	retval = ctrl_drv->spi_transfer(bank,&code, 1,SPI_WRITE,&status, 1, 0);
	if (retval < 0) 
		return spi_error(retval);

        if (wait_till_ready(bank,ctrl_drv,MAX_READY_WAIT_COUNT))
        {
                return -1;
        }
	return 0;
}

int 
spi_generic_enable_4b_spansion(int bank,struct spi_ctrl_driver *ctrl_drv, unsigned char data)
{
    int retval;
    u8 code = OPCODE_BRWR;

    /* Issue Controller Transfer Routine */
    retval = ctrl_drv->spi_transfer(bank,&code, 1,SPI_WRITE, &data, 1, 0);
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

	/* Send write enable before sending enable 4-byte mode*/
	spi_generic_write_enable(bank,ctrl_drv);

	/* Issue Enable 4byte addressing mode */
	retval = ctrl_drv->spi_transfer(bank,&code, 1,SPI_NONE, NULL, 0, 1);
	if (retval < 0) 
		return spi_error(retval);
	return 0;
}

int 
spi_generic_disable_4byte_mode(int bank, struct spi_ctrl_driver *ctrl_drv)
{
	u8 code = OPCODE_4BYEDIS;
	int retval;

	/* Send write enable before sending disable 4-byte mode*/
	spi_generic_write_enable(bank,ctrl_drv);

	/* Issue Disable 4byte addressing mode */
	retval = ctrl_drv->spi_transfer(bank,&code, 1,SPI_NONE, NULL, 0, 1);
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
	retval = ctrl_drv->spi_transfer(bank,&code, 1,SPI_NONE, NULL, 0, 0);
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
	retval = ctrl_drv->spi_transfer(bank,&code, 1,SPI_NONE, NULL, 0, 0);
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

int
spi_generic_erase(struct map_info *map, unsigned long sect_addr)
{
	struct spi_flash_private *priv=map->fldrv_priv;
	int bank = map->map_priv_1;
	struct spi_ctrl_driver *ctrl_drv = priv->ctrl_drv;
	int retval;
	bool fbytemode;
	bool checkFlagStatus;
	int cmdlen = 0;
	unsigned char command[5];

	down(&priv->chip_drv->lock);
	/* Wait until finished previous command. */
	if (wait_till_ready(bank,ctrl_drv,MAX_READY_WAIT_COUNT))
	{
		up(&priv->chip_drv->lock);
		return -1;
	}

	checkFlagStatus = ((priv->flags & SPI_FLAG_STS_READ) == SPI_FLAG_STS_READ);
	fbytemode = ((priv->flags & SPI_FBYTE_ADDR_MODE) == SPI_FBYTE_ADDR_MODE);
	if (fbytemode)
	{
		if ((priv->flags & SPI_FBYTE_SPANSION) == SPI_FBYTE_SPANSION)
			spi_generic_enable_4b_spansion(bank,ctrl_drv,0x80);
		else
			spi_generic_enable_4byte_mode(bank,ctrl_drv);
	}

	/* Send write enable */
	spi_generic_write_enable(bank,ctrl_drv);

	/* Set up command buffer. */
	if (fbytemode)
	{
		command[0] = OPCODE_SE;
		command[1] = sect_addr >> 24;
		command[2] = sect_addr >> 16;
		command[3] = sect_addr >> 8;
		command[4] = sect_addr;
		cmdlen = 5;
	}
	else
	{
		command[0] = OPCODE_SE;
		command[1] = sect_addr >> 16;
		command[2] = sect_addr >> 8;
		command[3] = sect_addr;
		cmdlen = 4;
	}

	/* Issue Controller Transfer Routine */
	retval = ctrl_drv->spi_transfer(bank,command, cmdlen ,SPI_NONE, NULL, 0, fbytemode);
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
		if ((priv->flags & SPI_FBYTE_SPANSION) == SPI_FBYTE_SPANSION)
			spi_generic_enable_4b_spansion(bank,ctrl_drv,0x0);
		else
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

int  
spi_generic_read(struct map_info *map, loff_t addr, size_t bytes, unsigned char *buff)
{
	struct spi_flash_private *priv=map->fldrv_priv;


	int bank = map->map_priv_1;
	struct spi_ctrl_driver *ctrl_drv = priv->ctrl_drv;
	int retval;
	size_t transfer;
	unsigned char command[5];
	bool fbytemode = false;
	int cmdlen = 0;
	int  (*readfn)(int bank,unsigned char *,int , SPI_DIR, unsigned char *, unsigned long, int);

	fbytemode = ((priv->flags & SPI_FBYTE_ADDR_MODE) == SPI_FBYTE_ADDR_MODE);

	/* Some time zero bytes length are sent */
	if (bytes==0)
		return 0;	

	down(&priv->chip_drv->lock);

	if (fbytemode)
	{
		if ((priv->flags & SPI_FBYTE_SPANSION) == SPI_FBYTE_SPANSION)
			spi_generic_enable_4b_spansion(bank,ctrl_drv,0x80);
		else
			spi_generic_enable_4byte_mode(bank,ctrl_drv);
	}

	/* Wait until finished previous command. */
	if (wait_till_ready(bank,ctrl_drv,MAX_READY_WAIT_COUNT))
	{
		up(&priv->chip_drv->lock);
		return -1;
	}

	readfn = ctrl_drv->spi_transfer;

	transfer=bytes;
	while (bytes)
	{
	 	transfer=bytes;

		/* Set up command buffer. */	/* Normal Read */
		if (fbytemode)
		{
			command[0] = OPCODE_READ;
			command[1] = addr >> 24;
			command[2] = addr >> 16;
			command[3] = addr >> 8;
			command[4] = addr;
			cmdlen = 5;
		}
		else
		{
			command[0] = OPCODE_READ;
			command[1] = addr >> 16;
			command[2] = addr >> 8;
			command[3] = addr;
			cmdlen = 4;
		}
		
		/* Issue Controller Transfer Routine */
		retval = (*readfn)(bank,command, cmdlen ,SPI_READ, buff, (unsigned long)transfer, fbytemode);
	    
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
		if ((priv->flags & SPI_FBYTE_SPANSION) == SPI_FBYTE_SPANSION)
			spi_generic_enable_4b_spansion(bank,ctrl_drv,0x0);
		else
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

int  
spi_generic_write(struct map_info *map, loff_t addr, size_t bytes, const unsigned char *buff)
{
	struct spi_flash_private *priv=map->fldrv_priv;
	int bank = map->map_priv_1;
	struct spi_ctrl_driver *ctrl_drv = priv->ctrl_drv;
	int retval;
	bool fbytemode;
	bool checkFlagStatus;
	int cmdlen = 0;
	unsigned char command[5];
	size_t transfer;
	size_t pagesize = PROGRAM_PAGE_SIZE;

	/* Some time zero bytes length are sent */
	if (bytes==0)
		return 0;	

	down(&priv->chip_drv->lock);

	checkFlagStatus = ((priv->flags & SPI_FLAG_STS_READ) == SPI_FLAG_STS_READ);
	fbytemode = ((priv->flags & SPI_FBYTE_ADDR_MODE) == SPI_FBYTE_ADDR_MODE);
	if (fbytemode)
	{
		if ((priv->flags & SPI_FBYTE_SPANSION) == SPI_FBYTE_SPANSION)
		{
			spi_generic_enable_4b_spansion(bank,ctrl_drv,0x80);
			pagesize = 512;
		}
		else
			spi_generic_enable_4byte_mode(bank,ctrl_drv);
	}

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

		transfer = pagesize;
		if (bytes <  transfer)
			transfer = bytes;
	
		if (fbytemode)
		{
			command[0] = OPCODE_PP;
			command[1] = addr >> 24;
			command[2] = addr >> 16;
			command[3] = addr >> 8;
			command[4] = addr;
			cmdlen = 5;
		}
		else
		{		  
			/* Set up command buffer. */  
			command[0] = OPCODE_PP;
			command[1] = addr >> 16;
			command[2] = addr >> 8;
			command[3] = addr;
			cmdlen = 4;
		}
		    
		/* Issue Controller Transfer Routine */
		retval = ctrl_drv->spi_transfer(bank,command,cmdlen,SPI_WRITE, 
				    (unsigned char *)buff, transfer, fbytemode);

		    
		if (retval < 0) 
		{
			up(&priv->chip_drv->lock);
			return spi_error(retval);
		}
		addr+=(transfer-retval);
		buff+=(transfer-retval);
		bytes-=(transfer-retval);

		if (checkFlagStatus)
			wait_till_flag_status_ready(bank, ctrl_drv);

		if (wait_till_ready(bank,ctrl_drv,MAX_READY_WAIT_COUNT))
		{
			up(&priv->chip_drv->lock);
			return -1;
		}
	}

	if (fbytemode) {

		if ((priv->flags & SPI_FBYTE_SPANSION) == SPI_FBYTE_SPANSION)
			spi_generic_enable_4b_spansion(bank,ctrl_drv,0x0);
		else
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
int 
spi_generic_probe(int bank,struct spi_ctrl_driver *ctrl_drv, struct spi_flash_info *chip_info,
			char *spi_name,struct spi_flash_info *spi_list, int spi_list_len)
{
	int  retval;
	u32 val;
	int i;
	u8 code = OPCODE_RDID;

	/* Send write enable */
	retval =spi_generic_write_enable(bank,ctrl_drv);
	if (retval < 0) 
	 	return -1;

	//The above write enable is not needed so we are making sure that the Chip is ready before issuing READ ID
	// We will send a smaller timout value during probe so that the control is not held for long when there is
	// no device present at a CS. Larger timeouts are needed only for erase/write operations
	wait_till_ready(bank,ctrl_drv,4000); 

	/* Issue Controller Transfer Routine */
	val = 0;
	retval = ctrl_drv->spi_transfer(bank,&code, 1,SPI_READ,(unsigned char *)&val, 3, 0);
	val &= 0x00FFFFFF;

	if (retval < 0) 
	{
		spi_error(retval);
		return -1;
	}

	/* Match the ID against the table entries */
	for (i = 0; i < spi_list_len; i++) 
	{
		if ((spi_list[i].mfr_id == ((val)& 0xFF)) &&
		    (spi_list[i].dev_id == ((val >> 8)& 0xFFFF)))
			break;
	}

	if (i == spi_list_len) 
	{
		return -1;
	}
	memcpy(chip_info,&spi_list[i],sizeof(struct spi_flash_info));

	if (spi_verbose > 0)
		printk(KERN_INFO"Found SPI Chip %s \n",spi_list[i].name);

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
