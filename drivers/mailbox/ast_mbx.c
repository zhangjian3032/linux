/*
 * ast-mbx.c - Mailbox driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/sched.h>
#include <linux/miscdevice.h>

#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <asm/uaccess.h>

#define CONFIG_AST_MBX_DEBUG

#ifdef CONFIG_AST_MBX_DEBUG
#define MBX_DBG(fmt, args...) printk("%s(): " fmt, __FUNCTION__, ## args)
#else
#define MBX_DBG(fmt, args...)
#endif

/***********************************************************************/
#define MBXIOC_BASE       'M'
#define AST_MBX_IOCTRIGGER			_IO(MBXIOC_BASE, 0x10)
#define AST_MBX_IOCSIZE			_IOR(MBXIOC_BASE, 0x11, unsigned long)
/***********************************************************************/
struct ast_coldfire_mbx_data {
	struct device		*misc_dev;
	int 				irq;
	void				*cvic_regs;
	u32				mbx_mem_size;
	phys_addr_t		*mbx_mem_phy;            /* phy */
	bool 			is_open;
	spinlock_t 		lock;
	struct fasync_struct	*fasync_queue;
};

/***************************************************************************/
static irqreturn_t ast_coldfire_mbx_handler(int this_irq, void *dev_id)
{
	struct ast_coldfire_mbx_data *ast_coldfire_mbx = dev_id;

	kill_fasync(&ast_coldfire_mbx->fasync_queue, SIGIO, POLL_IN);

	return IRQ_HANDLED;

}

/** @note munmap handler is done by vma close handler */
static int ast_coldfire_mbx_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct miscdevice *c = file->private_data;
	struct ast_coldfire_mbx_data *ast_coldfire_mbx = dev_get_drvdata(c->this_device);
	vma->vm_private_data = ast_coldfire_mbx;

	vma->vm_flags |= VM_IO;

	if (remap_pfn_range(vma, vma->vm_start, ((u32)ast_coldfire_mbx->mbx_mem_phy >> PAGE_SHIFT),
						ast_coldfire_mbx->mbx_mem_size, vma->vm_page_prot)) {
		printk(KERN_ERR "remap_pfn_range faile at %s()\n", __func__);
		return -EAGAIN ;
	}

	return 0;
}

static int ast_coldfire_mbx_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_coldfire_mbx_data *ast_coldfire_mbx = dev_get_drvdata(c->this_device);

	MBX_DBG("\n");
	spin_lock(&ast_coldfire_mbx->lock);

	if (ast_coldfire_mbx->is_open) {
		spin_unlock(&ast_coldfire_mbx->lock);
		return -1;
	}
	ast_coldfire_mbx->is_open = true;

	spin_unlock(&ast_coldfire_mbx->lock);

	return 0;
}

static int ast_coldfire_mbx_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_coldfire_mbx_data *ast_coldfire_mbx = dev_get_drvdata(c->this_device);

	MBX_DBG("\n");

	spin_lock(&ast_coldfire_mbx->lock);
	ast_coldfire_mbx->is_open = false;
	spin_unlock(&ast_coldfire_mbx->lock);

	return 0;
}

void ast_trigger_coldfire(struct ast_coldfire_mbx_data *ast_coldfire_mbx)
{
	writel(BIT(1), ast_coldfire_mbx->cvic_regs + 0x18);
}


static long  ast_coldfire_mbx_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	int ret = 1;
	struct miscdevice *c = fp->private_data;
	struct ast_coldfire_mbx_data *ast_coldfire_mbx = dev_get_drvdata(c->this_device);
//	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case AST_MBX_IOCSIZE:
		ret = __put_user(ast_coldfire_mbx->mbx_mem_size, (unsigned long __user *)arg);
		break;
	case AST_MBX_IOCTRIGGER:
		ast_trigger_coldfire(ast_coldfire_mbx);
		break;
	default:
		ret = 3;
		break;
	}
	return ret;

}

static int ast_coldfire_mbx_fasync(int fd, struct file *file, int mode)
{
	struct miscdevice *c = file->private_data;
	struct ast_coldfire_mbx_data *ast_coldfire_mbx = dev_get_drvdata(c->this_device);
	printk("%s enter \n", __func__);
	return fasync_helper(fd, file, mode, &ast_coldfire_mbx->fasync_queue);
}

static const struct file_operations ast_coldfire_mbx_fops = {
	.owner  			= THIS_MODULE,
	.llseek 			= no_llseek,
	.mmap			= ast_coldfire_mbx_mmap,
	.open 			= ast_coldfire_mbx_open,
	.release 			= ast_coldfire_mbx_release,
	.unlocked_ioctl 	= ast_coldfire_mbx_ioctl,
	.fasync			= ast_coldfire_mbx_fasync,
};

struct miscdevice ast_coldfire_mbx_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-mbx",
	.fops = &ast_coldfire_mbx_fops,
};

static int ast_coldfire_mbx_probe(struct platform_device *pdev)
{
	static struct ast_coldfire_mbx_data *ast_coldfire_mbx;
	struct resource *res;
	int ret = 0;

	MBX_DBG(" \n");

	ast_coldfire_mbx = devm_kzalloc(&pdev->dev, sizeof(struct ast_coldfire_mbx_data), GFP_KERNEL);
	if (ast_coldfire_mbx == NULL) {
		printk("memalloc error");
		goto out;
	}

	//CVIC register
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	ast_coldfire_mbx->cvic_regs = devm_ioremap_resource(&pdev->dev, res);
	if (!ast_coldfire_mbx->cvic_regs) {
		ret = -EIO;
		goto out_region;
	}

	//share memory
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	ast_coldfire_mbx->mbx_mem_phy = (phys_addr_t *)res->start;
	ast_coldfire_mbx->mbx_mem_size = resource_size(res);
	MBX_DBG("mbx_mem addr %x, size %d bytes\n", (u32)ast_coldfire_mbx->mbx_mem_phy, ast_coldfire_mbx->mbx_mem_size);

	ast_coldfire_mbx->irq = platform_get_irq(pdev, 0);
	if (ast_coldfire_mbx->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ret = request_irq(ast_coldfire_mbx->irq, ast_coldfire_mbx_handler, IRQF_SHARED,
					  "ast-mbx", ast_coldfire_mbx);

	if (ret) {
		printk(KERN_INFO "MBX: Failed request irq %d\n", ast_coldfire_mbx->irq);
		goto out_region;
	}

	ret = misc_register(&ast_coldfire_mbx_misc);
	if (ret) {
		printk(KERN_ERR "MBX : failed to request interrupt\n");
		goto out_irq;
	}

	spin_lock_init(&ast_coldfire_mbx->lock);
	platform_set_drvdata(pdev, ast_coldfire_mbx);

	dev_set_drvdata(ast_coldfire_mbx_misc.this_device, ast_coldfire_mbx);

	printk(KERN_INFO "ast_coldfire_mbx: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(ast_coldfire_mbx->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_coldfire_mbx_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_coldfire_mbx_data *ast_coldfire_mbx = platform_get_drvdata(pdev);

	MBX_DBG("ast_coldfire_mbx_remove\n");

	misc_deregister(&ast_coldfire_mbx_misc);

	free_irq(ast_coldfire_mbx->irq, &ast_coldfire_mbx);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	release_mem_region(res->start, res->end - res->start + 1);

	kfree(ast_coldfire_mbx);
	return 0;
}

#ifdef CONFIG_PM
static int
ast_coldfire_mbx_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("ast_coldfire_mbx_suspend : TODO \n");
	return 0;
}

static int
ast_coldfire_mbx_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define ast_coldfire_mbx_suspend        NULL
#define ast_coldfire_mbx_resume         NULL
#endif

static const struct of_device_id ast_coldfure_mbx_of_table[] = {
	{ .compatible = "aspeed,ast-coldfire-mbx", },
	{ },
};

MODULE_DEVICE_TABLE(of, ast_coldfure_mbx_of_table);

static struct platform_driver ast_coldfire_mbx_driver = {
	.probe		= ast_coldfire_mbx_probe,
	.remove 		= ast_coldfire_mbx_remove,
#ifdef CONFIG_PM	
	.suspend        = ast_coldfire_mbx_suspend,
	.resume         = ast_coldfire_mbx_resume,
#endif	
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = ast_coldfure_mbx_of_table,
	},
};

module_platform_driver(ast_coldfire_mbx_driver);


MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST Coldfire Mailbox driver");
MODULE_LICENSE("GPL");
