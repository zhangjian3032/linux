/*
 * SPI controller driver for ASPEED's PILOT 
 *
 * Copyright (c) 2018, Aspeed Technology Inc.
 * Shivah Shankar S <shivahshankar.shankarnarayanrao@aspeedtech.com>
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/bug.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/mfd/syscon.h>

#define DEVICE_NAME	"pilot-spi"
#define PILOT_SPI_DEFAULT_FREQ 50000000


#define PILOT_SPI_NONE  0
#define PILOT_SPI_READ  1
#define PILOT_SPI_WRITE  2

struct pilot_spi_regs{
	volatile union {
		volatile struct {
			volatile unsigned char Addr0;           /* LSB */
			volatile unsigned char Addr1;
			volatile unsigned char Addr2;           /* MSB */
			volatile unsigned char Addr3;
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
	volatile union {
		volatile struct {
			volatile unsigned short ClkDiv_cs0;
			volatile unsigned short ClkDiv_cs1;
		};
		volatile unsigned long ClkDiv0;
	};

	volatile unsigned long SpiStatus;
	volatile unsigned long BMisc;
	volatile unsigned long BInten;
	volatile unsigned long ClkDiv1;
	volatile unsigned char Rsvd5[8];//Pilot needs so much of offset as the data register is at offset 0x30
	volatile union {
		volatile unsigned char Data8[4];
		volatile unsigned long Data32;
	} Data;
};

struct pilot_spi_info {
	u32	maxsize;
	u8	nce;
};

struct pilot_spi_controller;
struct pilot_spi_chip {
	int cs;
	struct pilot_spi_controller *controller;
	void __iomem *regs;			/* control register */
	void __iomem *map_base;			/* base of chip window */
	u32 map_size;				/* chip mapping window size */
#if 0
	u32 ctl_val[smc_max];			/* control settings */
	enum aspeed_smc_flash_type type;	/* what type of flash */
#endif
	struct spi_nor nor;
	u32 clk_rate;
};
struct pilot_spi_controller {
	struct device *dev;
	struct mutex mutex;
	struct pilot_spi_info *info;
	void __iomem *regs;
	void __iomem *mapped_base;
	struct regmap *strap_reg;		/* Strap status register */
	struct pilot_spi_regs * pilot_spi_regs;
	u32	mapped_size;
	u32	clk_frequency;
	struct pilot_spi_chip *chips[0];
	u32	SPI3B4B_strap;

};

static const struct pilot_spi_info pilot_bootspi_info = {
	.maxsize = 252 * 1024 * 1024,
	.nce = 3,
};



static const struct of_device_id pilot_spi_matches[] = {
	{ .compatible = "aspeed,pilot-bootspi", .data = &pilot_bootspi_info },
	{ }
};
MODULE_DEVICE_TABLE(of, pilot_spi_matches);
void pilot_write32(volatile unsigned int* srcAddr, volatile unsigned int* dstAddr)  {
        asm volatile ("  PUSH {R2-R9} ");
        asm volatile ("  LDMIA  R0!, {R2-R9} ");
        asm volatile ("  STMIA  R1!, {R2-R9} ");
        asm volatile ("  POP {R2-R9} ");
}

static void waitforspiready(volatile struct pilot_spi_regs* regs)
{
        int retries = 10000000;
        while ((regs->SpiStatus & 0x01) && --retries);
        if (retries == 0)
        {
                //printk("Unable to exit while!!! (%s %d)\n", __FUNCTION__, __LINE__);
#ifdef CONFIG_PANIC_AND_BAIL_OUT
                panic("");
#endif
        }
}

static int pilot4_boot_bmc_spi_transfer(struct pilot_spi_controller *priv, 
		int bus, int cs, int bank,unsigned char *cmd,int cmdlen, 
		int dir, unsigned char *data, unsigned long datalen)
{
    unsigned char Opcode;
    unsigned char Type;
    unsigned char Dummy;
    unsigned char Ctrl;
    unsigned long Command;
    int i, aligned_addr = 0, unaligned_addr = 0;
    //int address_offset = fbyteaddr ? 4 : 3;
    struct pilot_spi_regs* spiregs = priv->pilot_spi_regs;

    // See if the address is Dword aligned. If it is aligned then the reads/writes
    // can be optimized further
   #if 0
    if (((unsigned int)data & 0x3) == 0)
    {
        aligned_addr = datalen / 32;
        unaligned_addr = datalen % 32;
    }
    else
#endif
    unaligned_addr = datalen;
    spiregs->BMisc &= ~(0x3 << 30); //Override
    spiregs->BMisc |= (cs << 30); //Override
    Ctrl = (0x80|spiregs->CmdCtrl);
    Dummy = 0;

    /* Fill in Command And Address */
    Opcode = cmd[0];
    //printk("Opcode is %x\n", Opcode);

    if (cmdlen == 4)
    {
	// This is required with 4 byte strap enabled
	if(priv->SPI3B4B_strap == 0x40)
	{
		//printk("4B Strap enabled\n");
		spiregs->Addr0 = cmd[4];
		spiregs->Addr1 = cmd[3];
		spiregs->Addr2 = cmd[2];
		spiregs->Addr3 = cmd[1];
	}else{
		//printk("3B Strap \n");
		spiregs->Addr0 = cmd[3];
		spiregs->Addr1 = cmd[2];
		spiregs->Addr2 = cmd[1];
		spiregs->Addr3 = cmd[4];
	}
	cmdlen = 4;
	//printk("Address programmed is %x len is %x\n", spiregs->Addr, cmdlen);
    }else if(cmdlen > 4){
	//This is needed with 4 byte strap enabled
	if(priv->SPI3B4B_strap == 0x40)
	{
		//printk("4B Strap enabled\n");
		spiregs->Addr0 = cmd[4];
		spiregs->Addr1 = cmd[3];
		spiregs->Addr2 = cmd[2];
		spiregs->Addr3 = cmd[1];
	}else{
		//printk("3B Strap \n");
		spiregs->Addr0 = cmd[3];
		spiregs->Addr1 = cmd[2];
		spiregs->Addr2 = cmd[1];
		spiregs->Addr3 = cmd[4];
	}
	//printk("Address programmed is %x len is %x\n", spiregs->Addr, cmdlen);
    }

    /* Fill in Command Len and data len */
    Type = 0x10;
    if (dir == PILOT_SPI_READ) {
        Type = 0x0;
    }

    Type |= cmdlen;

    Command = (Opcode) | (Type << 8) | (Dummy << 16) | (Ctrl << 24);

    /* Issue Command */
    waitforspiready(spiregs);
    //printk("Command written is %x\n", Command);
    spiregs->Command = Command;

    waitforspiready(spiregs);

    /* Do FIFO write for write commands. Writes are always in page boundary*/
    if (dir == PILOT_SPI_WRITE)
    {
        //printk("Direction write \n");
        // Do 32byte block copies if the source address was aligned and the data len
        // is more than 32 bytes length. 
        while(aligned_addr--)
        {
            pilot_write32((volatile unsigned int*)(data), (volatile unsigned int*)&spiregs->Data.Data8[0]);
            data += 32;
        }
        // 1. If there is any unaligned length that is left over from the previous
        //    copy, do it in the following for loop
        // 2. If the destination address is unaligned then do the entire copy in the
        //    following for loop
        for (i = 0; i < unaligned_addr; i++){
            spiregs->Data.Data8[0] = data[i];
	    //printk("data is %x\n", data[i]);
	}
    }
    // Read the data from the data register
    else if (dir == PILOT_SPI_READ)
    {
        //printk("Direction read\n");
        // Do 32byte block copies if the source address was aligned and the data len
        // is more than 32 bytes length. 
        while(aligned_addr--)
        {
            pilot_write32((volatile unsigned int*)&spiregs->Data.Data8[0], (volatile unsigned int*)(data));
            data += 32;
        }
        // 1. If there is any unaligned length that is left over from the previous
        //    copy, do it in the following for loop
        // 2. If the destination address is unaligned then do the entire copy in the
        //    following for loop
        for (i = 0; i < unaligned_addr; i++){
            data[i] = spiregs->Data.Data8[0];
	    //printk("data is %x\n", data[i]);
	}
    }
    waitforspiready(spiregs);

    /* Switch back to non-register mode and disable Write Fifo mode */
    spiregs->CmdCtrl &= 0x73;
    spiregs->BMisc |= (0x3 << 30); //Undo Override

    /* Wait for SPI Controller become Idle */
    waitforspiready(spiregs);
    return 0;
}
static int pilot_spi_unregister(struct pilot_spi_controller *controller)
{
	struct pilot_spi_chip *chip;
	int n;

	//printk("Entered %s\n", __func__);
	for (n = 0; n < controller->info->nce; n++) {
		chip = controller->chips[n];
		if (chip)
			mtd_device_unregister(&chip->nor.mtd);
	}

	return 0;
}
static int pilot_spi_remove(struct platform_device *dev)
{
	//printk("Entered %s\n", __func__);
	/* Iterate through every chips and call mtd_unregister()*/
	return pilot_spi_unregister(platform_get_drvdata(dev));
}
static int pilot_spi_chip_setup_finish(struct pilot_spi_chip *chip)
{
	//Here if required we reprogram the BMISC register for size
	//printk("Entered %s\n", __func__);
	return 0;
}
static int pilot_spi_chip_setup_init(struct pilot_spi_chip *chip,
				      struct resource *res)
{
	struct pilot_spi_controller *controller = chip->controller;
	const struct pilot_spi_info *info = controller->info;
	u32 reg, base_reg;
	u32 cs0_sz, cs1_sz, cs2_sz;
	//printk("Entered %s\n", __func__);
	// Here we rely on uboot to populate the correct size from its flash init
	// into the BMisc register
	cs0_sz = (controller->pilot_spi_regs->BMisc & 0x3f) * (4*1024*1024);
	cs1_sz = (((controller->pilot_spi_regs->BMisc & 0x3f00) >> 8) * (4*1024*1024)) - cs0_sz;
	cs2_sz = (((controller->pilot_spi_regs->BMisc & 0x3f0000) >> 16) * (4*1024*1024)) - (cs0_sz + cs1_sz);
	if(chip->cs == 0){
		chip->map_base = controller->mapped_base;
		chip->map_size = cs0_sz;
	}else if(chip->cs == 1){
		chip->map_base = controller->mapped_base + cs0_sz;
		chip->map_size = cs1_sz;
	}else{
		chip->map_base = controller->mapped_base + cs0_sz + cs1_sz;
		chip->map_size = cs2_sz;
	}
	//printk("For cs%d Chip map base is %x and size is %x\n", chip->cs, chip->map_base, chip->map_size);
	return 0;
}
static void pilot_spi_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct pilot_spi_chip *chip = nor->priv;
	//printk("Entered %s\n", __func__);
	mutex_unlock(&chip->controller->mutex);
}
static int pilot_spi_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct pilot_spi_chip *chip = nor->priv;
	//printk("Entered %s\n", __func__);
	mutex_lock(&chip->controller->mutex);
	return 0;
}

static int pilot_spi_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf,
				int len)
{
	struct pilot_spi_chip *chip = nor->priv;
	struct pilot_spi_controller *controller = chip->controller;
	//printk("Entered %s opcode is %x len is %d\n", __func__, opcode, len);
	pilot4_boot_bmc_spi_transfer(controller, 0, chip->cs, 0, &opcode, 1, PILOT_SPI_WRITE, buf, len); 
	return 0;
}

static int pilot_spi_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct pilot_spi_chip *chip = nor->priv;
	struct pilot_spi_controller *controller = chip->controller;

	//printk("Entered %s opcode %x and len is %d\n", __func__, opcode, len);



	pilot4_boot_bmc_spi_transfer(controller, 0, chip->cs, 0, &opcode, 1, PILOT_SPI_READ, buf, len); 

	return 0;
}
static ssize_t pilot_spi_write_user(struct spi_nor *nor, loff_t to,
				     size_t len, const u_char *write_buf)
{
	struct pilot_spi_chip *chip = nor->priv;
	struct pilot_spi_controller *controller = chip->controller;
	u8 command[5];
	u32 address = ((u32) to);
	u8 * baddr = (u8*) &address;
	struct pilot_spi_regs* spiregs = controller->pilot_spi_regs;
	//printk("Entered %s to %x \n", __func__, (u32)to);
	//printk("Entered %s to %x \n", __func__, address);
	//printk("Entered %s opcode is %x \n", __func__, nor->program_opcode);
	//printk("Entered %s size is %d\n", __func__, len);
	//printk(" Address is %x: %x: %x: %x\n", baddr[0], baddr[1], baddr[2], baddr[3]);
	command[0] = nor->program_opcode;
	command[1] = ((u8*)&address)[3];
	command[2] = ((u8*)&address)[2];
	command[3] = ((u8*)&address)[1];
	command[4] = ((u8*)&address)[0];
	//printk(" Command is %x: %x: %x: %x: %x\n", command[0], command[1], command[2], command[3], command[4]);
	//memcpy(&command[1], &address, 4);
	pilot4_boot_bmc_spi_transfer(controller, 0, chip->cs, 0, command, 5, PILOT_SPI_WRITE, write_buf, len); 

	return len;
}
static int pilot_spi_erase(struct spi_nor *nor, loff_t offs)
{
	struct pilot_spi_chip *chip = nor->priv;
	struct pilot_spi_controller *controller = chip->controller;
	u8 command[5];
	u32 address = ((u32) offs);
	u8 * baddr = (u8*) &address;
	command[0] = nor->erase_opcode;
	command[1] = ((u8*)&address)[3];
	command[2] = ((u8*)&address)[2];
	command[3] = ((u8*)&address)[1];
	command[4] = ((u8*)&address)[0];
	pilot4_boot_bmc_spi_transfer(controller, 0, chip->cs, 0, command, 5, PILOT_SPI_WRITE, NULL, 0); 
	return 0;
}
static ssize_t pilot_spi_read(struct spi_nor *nor, loff_t from, size_t len,
			       u_char *read_buf)
{
	struct pilot_spi_chip *chip = nor->priv;
	struct pilot_spi_controller *controller = chip->controller;
	u8 command[5];
	u32 address = ((u32) from);
	u8 * baddr = (u8*) &address;

	/* For Read we can use Direct access*/	
	//printk("Entered %s from %x \n", __func__, (u32)from);
	//printk("Entered %s from %x \n", __func__, address);
	//printk("Entered %s opcode is %x \n", __func__, nor->read_opcode);
	//printk("Entered %s dummy is %x \n", __func__, nor->read_dummy);
	//printk("Entered %s size is %d\n", __func__, len);
	//printk(" Address is %x: %x: %x: %x\n", baddr[0], baddr[1], baddr[2], baddr[3]);
#if 0
	command[0] = nor->read_opcode;
	command[1] = ((u8*)&address)[3];
	command[2] = ((u8*)&address)[2];
	command[3] = ((u8*)&address)[1];
	command[4] = ((u8*)&address)[0];
	//printk(" Command is %x: %x: %x: %x: %x\n", command[0], command[1], command[2], command[3], command[4]);
	//memcpy(&command[1], &address, 4);
	//pilot4_boot_bmc_spi_transfer(controller, 0, chip->cs, 0, command, 4, PILOT_SPI_READ, read_buf, len); 
#endif
	memcpy(read_buf, controller->mapped_base + from, len);
	//printk(" Data is %x: %x: %x: %x: %x\n", read_buf[0], read_buf[1], read_buf[2], read_buf[3], read_buf[4]);

	return len;
}
static int pilot_spi_setup_flash(struct pilot_spi_controller *controller,
				  struct device_node *np, struct resource *r)
{
	const struct spi_nor_hwcaps hwcaps = {
		.mask = SNOR_HWCAPS_READ |
			SNOR_HWCAPS_READ_FAST |
			SNOR_HWCAPS_READ_1_1_2 |
			SNOR_HWCAPS_PP,
	};
	const struct pilot_spi_info *info = controller->info;
	struct device *dev = controller->dev;
	struct device_node *child;
	unsigned int cs;
	int ret = -ENODEV;

	printk("Entered %s\n", __func__);
	for_each_available_child_of_node(np, child) {
		struct pilot_spi_chip *chip;
		struct spi_nor *nor;
		struct mtd_info *mtd;

		/* This driver does not support NAND or NOR flash devices. */
		if (!of_device_is_compatible(child, "jedec,spi-nor"))
			continue;

		ret = of_property_read_u32(child, "reg", &cs);
		if (ret) {
			dev_err(dev, "Couldn't not read chip select.\n");
			break;
		}
		
		if (cs >= info->nce) {
			dev_err(dev, "Chip select %d out of range.\n",
				cs);
			ret = -ERANGE;
			break;
		}

		if (controller->chips[cs]) {
			dev_err(dev, "Chip select %d already in use by %s\n",
				cs, dev_name(controller->chips[cs]->nor.dev));
			ret = -EBUSY;
			break;
		}

		chip = devm_kzalloc(controller->dev, sizeof(*chip), GFP_KERNEL);
		if (!chip) {
			ret = -ENOMEM;
			break;
		}

		if (of_property_read_u32(child, "spi-max-frequency",
					 &chip->clk_rate)) {
			chip->clk_rate = PILOT_SPI_DEFAULT_FREQ;
		}
		dev_info(dev, "Using %d MHz SPI frequency\n",
			 chip->clk_rate / 1000000);

		chip->controller = controller;
		chip->cs = cs;

		nor = &chip->nor;
		mtd = &nor->mtd;

		nor->dev = dev;
		nor->priv = chip;
		spi_nor_set_flash_node(nor, child);
		nor->read = pilot_spi_read;
		nor->write = pilot_spi_write_user;
		nor->read_reg = pilot_spi_read_reg;
		nor->write_reg = pilot_spi_write_reg;
		nor->erase = pilot_spi_erase;
		nor->prepare = pilot_spi_prep;
		nor->unprepare = pilot_spi_unprep;
		
		ret = pilot_spi_chip_setup_init(chip, r);
		if (ret)
			break;
		
		ret = spi_nor_scan(nor, NULL, &hwcaps);
		if (ret)
			break;
		
		ret = pilot_spi_chip_setup_finish(chip);
		if (ret)
			break;
		
		ret = mtd_device_register(mtd, NULL, 0);
		if (ret)
			break;
		
		controller->chips[cs] = chip;
	}
	
	if (ret)
		pilot_spi_unregister(controller);

	return ret;
}
#define BOOTSPI_STRAP_REG_OFFSET  0xC
static int pilot_spi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct pilot_spi_controller *controller;
	const struct of_device_id *match;
	const struct pilot_spi_info *info;
	struct clk *clk; //right now no, may be later
	struct resource *res;
	int ret;
	u32 strap_reg;

	//printk("Entered %s\n", __func__);
	match = of_match_device(pilot_spi_matches, &pdev->dev);
	if (!match || !match->data)
		return -ENODEV;
	info = match->data;

	controller = devm_kzalloc(&pdev->dev, sizeof(*controller) +
		info->nce * sizeof(controller->chips[0]), GFP_KERNEL);
	if (!controller)
		return -ENOMEM;
	controller->info = info;
	controller->dev = dev;

	mutex_init(&controller->mutex);
	platform_set_drvdata(pdev, controller);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	controller->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(controller->regs))
		return PTR_ERR(controller->regs);
	controller->pilot_spi_regs = (struct pilot_spi_regs *) controller->regs;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	controller->mapped_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(controller->mapped_base))
		return PTR_ERR(controller->mapped_base);
	
	controller->mapped_size = resource_size(res);


#if 0 
	Not required for now, may be later
	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	controller->clk_frequency = clk_get_rate(clk);
	devm_clk_put(&pdev->dev, clk);
#endif
	ret = pilot_spi_setup_flash(controller, np, res);
	if (ret)
		dev_err(dev, "Pilot SPI probe failed %d\n", ret);
#if 0
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	controller->strap_reg = (u8*) devm_ioremap_resource(dev, res);
	if (IS_ERR(controller->strap_reg))
		return PTR_ERR(controller->strap_reg);
	controller->strap_reg = syscon_node_to_regmap(
			pdev->dev.parent->of_node);
#endif
	controller->strap_reg = syscon_regmap_lookup_by_phandle(np,
				"syscon");
	if (IS_ERR(controller->strap_reg)) {
		dev_err(dev, "Couldn't get regmap\n");
		return -ENODEV;
	}

	regmap_read(controller->strap_reg, BOOTSPI_STRAP_REG_OFFSET, &strap_reg);
	controller->SPI3B4B_strap = strap_reg;
	printk("The value of strap reg is %x address is %x\n", controller->SPI3B4B_strap, strap_reg);
	controller->SPI3B4B_strap &= 0x40;
	printk("The value of strap reg is %x \n", controller->SPI3B4B_strap);
	return ret;
}
static struct platform_driver pilot_spi_driver = {
	.probe = pilot_spi_probe,
	.remove = pilot_spi_remove,
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = pilot_spi_matches,
	}
};

module_platform_driver(pilot_spi_driver);

MODULE_DESCRIPTION("Pilot SPI Controller Driver");
MODULE_AUTHOR("<shivahshankar.shankarnarayanrao@aspeedtech.com>");
MODULE_LICENSE("GPL v2");
