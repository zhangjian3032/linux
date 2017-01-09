/********************************************************************************
* File Name     : ast_bitblt.c
* Author         : Ryan Chen
* Description   : AST RLE Controller
* 
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
*
*   Version      : 1.0
*   History      : 
*      1. 2015/04/30 Ryan Chen create this file 
*    
********************************************************************************/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/slab.h>

#include <linux/sched.h>  
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>

#include <mach/ast_lcd.h>

#include <plat/ast_bitblt.h>

/***********************************************************************/

//#define CONFIG_AST_RLE_DEBUG

#ifdef CONFIG_AST_RLE_DEBUG
	#define RLE_DBG(fmt, args...) printk(KERN_DEBUG "%s(): " fmt, __FUNCTION__, ## args)
#else
	#define RLE_DBG(fmt, args...)
#endif

/***********************************************************************/
void __iomem	*reg_base;			/* virtual */

static inline void
ast_rle_write(u32 val, u32 reg)
{
//	RLE_DBG("write offset: %x, val: %x \n",reg,val);
	writel(val, reg_base + reg);
}

static inline u32
ast_rle_read(u32 reg)
{
	u32 val = readl(reg_base + reg);
//	RLE_DBG("read offset: %x, val: %x \n",reg,val);

	return val;
}
/***************************************************************************/
#ifdef CONFIG_RLE_WITH_CMDQ

#else
void ast_rle_16_to_16()
{
#if 0
WriteMemoryLong(0x1e6eb300, 0x0c, RLE_BS[0]);
WriteMemoryLong(0x1e6eb300, 0x10, RLE_FB[0]); 
WriteMemoryLong(0x1e6eb300, 0x14, myRLE->Width);
WriteMemoryLong(0x1e6eb300, 0x18, myRLE->Height);
WriteMemoryLong(0x1e6eb300, 0x1c, myRLE->Color);
WriteMemoryLong(0x1e6eb300, 0x20, myRLE->Length);
WriteMemoryLong(0x1e6eb300, 0x24, myRLE->bFlip);
wmb();
WriteMemoryLong(0x1e6eb300, 0x28, 1);  //fire

do
{
	schedule();
	EngBusy = ReadMemoryLong(0x1e6eb300, 0x34);
}while(EngBusy);


#endif
}


#endif
/***************************************************************************/
#ifdef CONFIG_RLE_WITH_CMDQ

#else
static irqreturn_t ast_rle_err_irq(int this_irq, void *dev_id)
{
	printk("ast_rle_err_irq \n");
	return IRQ_HANDLED;
}


static irqreturn_t ast_rle_done_irq(int this_irq, void *dev_id)
{
	printk("ast_rle_done_irq \n");
	return IRQ_HANDLED;
}
#endif

static void ast_rle_init(void)
{

}

static int ast_rle_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = 0;
	int irq = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	if (!request_mem_region(res->start, resource_size(res), res->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		ret = -ENXIO;
		goto out;
	}

	reg_base = ioremap(res->start, resource_size(res));
	if (!reg_base) {
		ret = -EIO;
		goto out_region;
	}

#ifdef CONFIG_RLE_WITH_CMDQ
#else
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ret = request_irq(irq, ast_rle_done_irq, IRQF_DISABLED,
			  "ast-bitblt-done", NULL);

	if (ret) {
		printk(KERN_INFO "Bitblt: Failed request irq %d\n", irq);
		goto out_region;
	}

	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ret = request_irq(irq, ast_rle_err_irq, IRQF_DISABLED,
			  "ast-bitblt-error", NULL);

	if (ret) {
		printk(KERN_INFO "Bitblt: Failed request irq %d\n", irq);
		goto out_region;
	}
	
#endif

//	platform_set_drvdata(pdev, ast_bitblt);

	ast_rle_init();
	
	printk(KERN_INFO "ast_bitblt: driver successfully loaded.\n");

	return 0;

out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "driver init failed (ret=%d)!\n", ret);
	return ret;
}

static struct platform_driver ast_rle_driver = {
	.driver         = {
		.name   = "ast-bitblt",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver_probe(ast_rle_driver, ast_rle_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST Bitblt driver");
MODULE_LICENSE("GPL");
