/********************************************************************************
* File Name     : ast_rfx.c
* Author         : Ryan Chen
* Description   : AST Video Engine Controller
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
*      1. 2014/04/30 Ryan Chen create this file 
*    
********************************************************************************/

#include <linux/slab.h>
#include <linux/sched.h>

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>

#include <linux/hwmon-sysfs.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <plat/regs-rfx.h>
//#include <mach/ast_rfx.h>

//#define CONFIG_AST_RFX_DEBUG

#ifdef CONFIG_AST_RFX_DEBUG
	#define RFX_DBG(fmt, args...) printk("%s(): " fmt, __FUNCTION__, ## args)
#else
	#define RFX_DBG(fmt, args...)
#endif

/***********************************************************************/
//IOCTL ..
#define RFXIOC_BASE       'R'

#define AST_RFX_IOCRTIMING			_IOR(RFXIOC_BASE, 0, struct timing_negotiation*)
#define AST_RFX_IOCWTIMING		_IOW(RFXIOC_BASE, 1, struct timing_negotiation*)
#define AST_RFX_IOCXFER			_IOWR(RFXIOC_BASE, 2, struct xfer_msg*)

/***************************************************************************************************************************************/

#define AST_RFX_CMDQ_SIZE					(0x100000)
#define AST_RFX_BULK_STREAM_SIZE			(0x100000)
#define AST_RFX_BULK_HISTORY_SIZE			(0x300000)
#define AST_RFX_BULK_MAPPED_SIZE			(0x400000)

/***************************************************************************************************************************************/
static struct ast_rfx_data {
	struct device		*misc_dev;
	void __iomem			*reg_base;			/* virtual */
	int 					irq;				//Video IRQ number 
//	compress_header	
	phys_addr_t			*buff0_phy;             /* phy */
	u32					*buff0_virt;            /* virt */
        bool is_open;
        phys_addr_t             	*rfx_mem_phy;            /* phy */
        u32                             *rfx_mem_virt;           /* virt */
		
	u32		rfx_mem_size;			/* phy size*/			
	spinlock_t			rfx_state_lock;               /* Serializing lock */
	struct fasync_struct	*async_queue;		
};
/***************************************************************************************************************************************/

static inline void
ast_rfx_write(struct ast_rfx_data *ast_rfx, u32 val, u32 reg)
{
	writel(val, ast_rfx->reg_base + reg);
}

static inline u32
ast_rfx_read(struct ast_rfx_data *ast_rfx, u32 reg)
{
	u32 val = readl(ast_rfx->reg_base + reg);
//	RFX_DBG("read offset: %x, val: %x \n",reg,val);
	return val;
}

/***************************************************************************************************************************************/
static void ast_rfx_ctrl_init(struct ast_rfx_data *ast_rfx)
{
	RFX_DBG("\n");
	ast_rfx_write(ast_rfx, 0, AST_RFX_IER);
	//cmdq init
#if 0	
	ast_rfx_write(ast_rfx, 0, AST_RFX_CMDQ_CTRL);
	ast_rfx_write(ast_rfx, ast_rfx->rfx_mem_phy, AST_RFX_CMDQ_START);
	ast_rfx_write(ast_rfx, ast_rfx->rfx_mem_phy + AST_RFX_CMDQ_SIZE, AST_RFX_CMDQ_END);
	ast_rfx_write(ast_rfx, ast_rfx->rfx_mem_phy, AST_RFX_CMDQ_WRITE);
	ast_rfx_write(ast_rfx, ast_rfx->rfx_mem_phy, AST_RFX_CMDQ_READ);
	ast_rfx_write(ast_rfx, RFX_CMDQ_EN, AST_RFX_CMDQ_CTRL);
	//bulk init stream history
	ast_rfx_write(ast_rfx, ast_rfx->rfx_mem_phy + AST_RFX_CMDQ_SIZE, AST_RFX_BULK_STREAM_BUFF);
	ast_rfx_write(ast_rfx, ast_rfx->rfx_mem_phy + AST_RFX_CMDQ_SIZE + AST_RFX_BULK_STREAM_SIZE, AST_RFX_BULK_STREAM_END);
	ast_rfx_write(ast_rfx, ast_rfx->rfx_mem_phy + AST_RFX_CMDQ_SIZE + AST_RFX_BULK_STREAM_SIZE, AST_RFX_BULK_HISTORY_BUFF);
	ast_rfx_write(ast_rfx, ast_rfx->rfx_mem_phy + AST_RFX_CMDQ_SIZE + AST_RFX_BULK_STREAM_SIZE + AST_RFX_BULK_HISTORY_SIZE, AST_RFX_BULK_HISTORY_END);
	ast_rfx_write(ast_rfx, 1, AST_RFX_BULK_HISTORY_REST);
#endif
}

static irqreturn_t ast_rfx_irq(int this_irq, void *dev_id)
{
	struct ast_rfx_data *ast_rfx = dev_id;
//	u32 sts = ast_rfx_read(ast_rfx, AST_RFX_INT_STS) & ast_rfx_read(ast_rfx, AST_RFX_INT_EN);

	
//	RFX_DBG("ISR = %x : ",sts);
	

	
	return IRQ_HANDLED;
}

/***************************************************************************************************************************************/
static long ast_rfx_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	int ret = 1;
	u32 ctrl = 0;
	struct miscdevice *c = fp->private_data;
	struct ast_rfx_data *ast_rfx = dev_get_drvdata(c->this_device);
	void __user *argp = (void __user *)arg;

	switch (cmd) {
		default:
			ret = 3;
			break;
	}
	return ret;

}

static int ast_rfx_fasync(int fd, struct file *file, int mode)
{
	struct miscdevice *c = file->private_data;
	struct ast_rfx_data *ast_rfx = dev_get_drvdata(c->this_device);
	return fasync_helper(fd, file, mode, &ast_rfx->async_queue);
}

/** @note munmap handler is done by vma close handler */
static int ast_rfx_mmap(struct file * file, struct vm_area_struct * vma)
{
        struct miscdevice *c = file->private_data;
        struct ast_rfx_data *ast_rfx = dev_get_drvdata(c->this_device);
        size_t size = vma->vm_end - vma->vm_start;
        vma->vm_private_data = ast_rfx;

        if (PAGE_ALIGN(size) > ast_rfx->rfx_mem_size) {
                        printk(KERN_ERR "required length exceed the size "
                                   "of physical sram (%x)\n", ast_rfx->rfx_mem_size);
                        return -EAGAIN;
        }

        if ((ast_rfx->rfx_mem_phy + (vma->vm_pgoff << PAGE_SHIFT) + size)
                > (ast_rfx->rfx_mem_phy + ast_rfx->rfx_mem_size)) {
                        printk(KERN_ERR "required sram range exceed the size "
                                   "of phisical sram\n");
                        return -EAGAIN;
        }

        vma->vm_flags |= VM_IO;
        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

        if (io_remap_pfn_range(vma, vma->vm_start,
                        ((u32)ast_rfx->rfx_mem_phy >> PAGE_SHIFT),
                        size,
                        vma->vm_page_prot)) {
                printk(KERN_ERR "remap_pfn_range faile at %s()\n", __func__);
                return -EAGAIN;
        }

        return 0;
}

static int ast_rfx_open(struct inode *inode, struct file *file)
{
        struct miscdevice *c = file->private_data;
        struct ast_rfx_data *ast_rfx = dev_get_drvdata(c->this_device);

        RFX_DBG("\n");
        spin_lock(&ast_rfx->rfx_state_lock);

        if (ast_rfx->is_open) {
                spin_unlock(&ast_rfx->rfx_state_lock);
                return -EBUSY;
        }

        ast_rfx->is_open = true;
//        wake_up_process(ast_rfx->thread_task);
        spin_unlock(&ast_rfx->rfx_state_lock);

        return 0;

}

static int ast_rfx_release(struct inode *inode, struct file *file)
{
        struct miscdevice *c = file->private_data;
        struct ast_rfx_data *ast_rfx = dev_get_drvdata(c->this_device);

        RFX_DBG("\n");
        spin_lock(&ast_rfx->rfx_state_lock);

//        kthread_stop(ast_rfx->thread_task);

        ast_rfx->is_open = false;
        spin_unlock(&ast_rfx->rfx_state_lock);
        return 0;
}

static const struct file_operations ast_rfx_fops = {
	.owner 			= THIS_MODULE,
	.llseek 			= no_llseek,
	.unlocked_ioctl 	= ast_rfx_ioctl,
	.open 			= ast_rfx_open,
	.release 			= ast_rfx_release,
	.mmap 			= ast_rfx_mmap,
	.fasync			= ast_rfx_fasync,
};

struct miscdevice ast_rfx_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-rfx",
	.fops = &ast_rfx_fops,
};

/************************************************************************************************************/
static int ast_rfx_probe(struct platform_device *pdev)
{
	struct resource *res0, *res1;
	int ret=0;
	struct ast_rfx_data *ast_rfx;

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res0) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	if (!request_mem_region(res0->start, resource_size(res0), res0->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		ret = -ENXIO;
		goto out;
	}

	if(!(ast_rfx = kzalloc(sizeof(struct ast_rfx_data), GFP_KERNEL))) {
		return -ENOMEM;
		goto out;
        }

	ast_rfx->reg_base = ioremap(res0->start, resource_size(res0));
	if (!ast_rfx->reg_base) {
		ret = -EIO;
		goto out_region0;
	}
	
	res1 = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!res1)
		return -ENODEV;
	
	if (!request_mem_region(res1->start, resource_size(res1), res1->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		ret = -ENXIO;
		goto out_region0;
	}

	//Phy assign
	ast_rfx->rfx_mem_phy = res1->start;
	ast_rfx->rfx_mem_size = resource_size(res1);
	RFX_DBG("rfx_mem_size %d MB\n",ast_rfx->rfx_mem_size/1024/1024);

	//virt assign
	ast_rfx->rfx_mem_virt = ioremap(res1->start, resource_size(res1));
	if (!ast_rfx->rfx_mem_virt) {
	        ret = -EIO;
	        goto out_region1;
	}

	memset(ast_rfx->rfx_mem_virt, 0, resource_size(res1));	

	ast_rfx_ctrl_init(ast_rfx);
	
	ast_rfx->irq = platform_get_irq(pdev, 0);
	if (ast_rfx->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region1;
	}
//	ast_rfx->plat_data = pdev->dev.platform_data;	

	ret = misc_register(&ast_rfx_misc);
	if (ret){		
		printk(KERN_ERR "VIDEO : failed to request interrupt\n");
		goto out_irq;
	}

	platform_set_drvdata(pdev, ast_rfx);
	dev_set_drvdata(ast_rfx_misc.this_device, ast_rfx);

	ret = request_irq(ast_rfx->irq, ast_rfx_irq, IRQF_SHARED, "ast-rfx", ast_rfx);
	if (ret) {
		printk(KERN_INFO "VIDEO: Failed request irq %d\n", ast_rfx->irq);
		goto out_region1;
	}

	printk(KERN_INFO "ast_rfx: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(ast_rfx->irq, NULL);

out_region1:
	release_mem_region(res1->start, res1->end - res1->start + 1);	

out_region0:
	release_mem_region(res0->start, res0->end - res0->start + 1);
	
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;

}

static int ast_rfx_remove(struct platform_device *pdev)
{
	struct resource *res0, *res1;
	struct ast_rfx_data *ast_rfx = platform_get_drvdata(pdev);
	RFX_DBG("ast_rfx_remove\n");

	misc_deregister(&ast_rfx_misc);

	free_irq(ast_rfx->irq, ast_rfx);

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_rfx->reg_base);

	release_mem_region(res0->start, res0->end - res0->start + 1);

	res1 = platform_get_resource(pdev, IORESOURCE_DMA, 0);

	iounmap(ast_rfx->rfx_mem_virt);

	release_mem_region(res1->start, res1->end - res1->start + 1);

	return 0;	
}

#ifdef CONFIG_PM
static int 
ast_rfx_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("ast_rfx_suspend : TODO \n");
	return 0;
}

static int 
ast_rfx_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define ast_rfx_suspend        NULL
#define ast_rfx_resume         NULL
#endif

static struct platform_driver ast_rfx_driver = {
	.probe		= ast_rfx_probe,
	.remove 		= ast_rfx_remove,
	.suspend        = ast_rfx_suspend,
	.resume         = ast_rfx_resume,
	.driver  	       = {
	        .name   = "ast-rfx",
	        .owner  = THIS_MODULE,
	},
};

static int __init 
ast_rfx_init(void)
{
	return platform_driver_register(&ast_rfx_driver);
}

static void __exit 
ast_rfx_exit(void)
{
	platform_driver_unregister(&ast_rfx_driver);
}

module_init(ast_rfx_init);
module_exit(ast_rfx_exit);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("Video Engine driver");
MODULE_LICENSE("GPL");
