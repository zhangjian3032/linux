/*
 * ast-jtag.c - JTAG driver for the Aspeed SoC
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
#include <linux/sysfs.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <asm/io.h>
#include <asm/uaccess.h>
/*************************************************************************************/
//ast-g6 register add 
#define AST_JTAG_SHIFT0			0x20
#define AST_JTAG_SHIFT1			0x24
#define AST_JTAG_PADDING_CTRL0	0x28
#define AST_JTAG_PADDING_CTRL1	0x2C
#define AST_JTAG_SHIFT_CTRL		0x30
#define AST_JTAG_GBL_CTRL		0x34
#define AST_JTAG_IER			0x38
#define AST_JTAG_STS			0x3C

/* 	AST_JTAG_PADDING_CTRL - 0x28 : Padding control */
#define JTAG_PADDING_DATA			(0x1 << 24)
#define JTAG_POST_PADDING_NUM(x)	((x) << 12)
#define JTAG_PRE_PADDING_NUM(x)		(x)

/* 	AST_JTAG_SHIFT_CTRL - 0x30 : Shift control */
#define JTAG_TCK_FREE_RUN_EN		(0x1 << 31)
#define JTAG_STATIC_SHIFT_EN		(0x1 << 30)
#define JTAG_SHIFT_TMS(x)			((x) << 16)
#define JTAG_POST_TMS_SHIFT_NUM(x)	((x) << 13)
#define JTAG_PRE_TMS_SHIFT_NUM(x)	((x) << 10)
#define JTAG_PADDING_SELECT1		(0x1 << 9)
#define JTAG_END_OF_SHIFT			(0x1 << 8)
#define JTAG_START_OF_SHIFT			(0x1 << 7)
#define JTAG_DATA_SHIFT_NUM(x)		(x)

/*	AST_JTAG_GBL_CTRL - 0x34 : Global control */
#define JTAG_ENG_MODE_EN		(0x1 << 31)
#define JTAG_ENG_OUTPUT_EN		(0x1 << 30)
#define JTAG_ENG_FORCE_RESET	(0x1 << 29)

#define JTAG_STATIC_SHIFT_VAL	(0x1 << 16)
#define JTAG_CLK_DIV(x)			(x)			/*TCK period = Period of HCLK * (JTAG14[10:0] + 1)*/
#define JTAG_CLK_DIVISOR_MASK	(0x7ff)
#define JTAG_GET_CLK_DIVISOR(x)	(x & 0x7ff)

/* AST_JTAG_IER	- 0x38 : Interrupt Control */
#define JTAG_SHIFT_COMP_ISR_EN	(0x1 << 16)
#define JTAG_SHIFT_COMP_ISR		(0x1)
/* AST_JTAG_IER	- 0x3C : Status */
#define JTAG_ENG_BUSY

/*************************************************************************************/
struct jtag_xfer {
	int	dir;		//0: in 1:out
	u32 data0;
	u32 data1;
	u32 padding0;
	u32 padding1;
	u32 trigger_ctrl;
};

#define JTAGIOC_BASE       'T'

#define AST_JTAG_IOCXFER		_IOWR(JTAGIOC_BASE, 0, struct jtag_xfer)
#define AST_JTAG_SIOCFREQ		_IOW(JTAGIOC_BASE, 1, unsigned int)
#define AST_JTAG_GIOCFREQ		_IOR(JTAGIOC_BASE, 2, unsigned int)
/******************************************************************************/
//#define AST_JTAG_DEBUG

#ifdef AST_JTAG_DEBUG
#define JTAG_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define JTAG_DBUG(fmt, args...)
#endif

#define JTAG_MSG(fmt, args...) printk(fmt, ## args)

struct ast_jtag_info {
	void __iomem	*reg_base;
	int 			irq;			//JTAG IRQ number
	struct reset_control *reset;
	struct clk 		*clk;
	u32				hclk;
	u32 			flag;
	struct completion	xfer_complete;
	bool 			is_open;
};
/*************************************************************************************/
static DEFINE_SPINLOCK(jtag_state_lock);

/******************************************************************************/
static inline u32
ast_jtag_read(struct ast_jtag_info *ast_jtag, u32 reg)
{
#if 0
	u32 val;
	val = readl(ast_jtag->reg_base + reg);
	JTAG_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
#else
	return readl(ast_jtag->reg_base + reg);;
#endif
}

static inline void
ast_jtag_write(struct ast_jtag_info *ast_jtag, u32 val, u32 reg)
{
	JTAG_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, ast_jtag->reg_base + reg);
}

/******************************************************************************/
static void ast_jtag_set_freq(struct ast_jtag_info *ast_jtag, unsigned int freq)
{
	u16 i;
	for (i = 0; i < 0x7ff; i++) {
//		JTAG_DBUG("[%d] : freq : %d , target : %d \n", i, ast_get_pclk()/(i + 1), freq);
		if ((ast_jtag->hclk / (i + 1)) <= freq)
			break;
	}
//	printk("div = %x \n", i);
	ast_jtag_write(ast_jtag, ((ast_jtag_read(ast_jtag, AST_JTAG_GBL_CTRL) & ~JTAG_CLK_DIVISOR_MASK) | i),  AST_JTAG_GBL_CTRL);

}

static unsigned int ast_jtag_get_freq(struct ast_jtag_info *ast_jtag)
{
	return ast_jtag->hclk / (JTAG_GET_CLK_DIVISOR(ast_jtag_read(ast_jtag, AST_JTAG_GBL_CTRL)) + 1);
}
/******************************************************************************/
static int ast_jtag_xfer(struct ast_jtag_info *ast_jtag, struct jtag_xfer *xfer)
{
//	JTAG_DBUG("%s mode, ENDIR : %d, len : %d \n", sir->mode ? "SW" : "HW", sir->endir, sir->length);

	if(xfer->dir) {
		ast_jtag_write(ast_jtag, xfer->data0, AST_JTAG_SHIFT0);
		ast_jtag_write(ast_jtag, xfer->data1, AST_JTAG_SHIFT1);
	}

	ast_jtag_write(ast_jtag, xfer->padding0, AST_JTAG_PADDING_CTRL0);
	ast_jtag_write(ast_jtag, xfer->padding1, AST_JTAG_PADDING_CTRL1);

	ast_jtag_write(ast_jtag, xfer->trigger_ctrl, AST_JTAG_SHIFT_CTRL);

	wait_for_completion(&ast_jtag->xfer_complete);

	if(!xfer->dir) {
		xfer->data0 = ast_jtag_read(ast_jtag, AST_JTAG_SHIFT0);
		xfer->data1 = ast_jtag_read(ast_jtag, AST_JTAG_SHIFT1);
	}

	return 0;
}
/*************************************************************************************/
static irqreturn_t ast_jtag_interrupt(int this_irq, void *dev_id)
{
	u32 status;
	struct ast_jtag_info *ast_jtag = dev_id;

	status = ast_jtag_read(ast_jtag, AST_JTAG_IER);
	JTAG_DBUG("sts %x \n", status);

	if (status & JTAG_SHIFT_COMP_ISR) {
		ast_jtag_write(ast_jtag, JTAG_SHIFT_COMP_ISR_EN | JTAG_SHIFT_COMP_ISR, AST_JTAG_IER);
	}

	complete(&ast_jtag->xfer_complete);
	return IRQ_HANDLED;
}

/*************************************************************************************/
static struct ast_jtag_info *ast_jtag;

static long jtag_ioctl(struct file *file, unsigned int cmd,
					   unsigned long arg)
{
	int ret = 0;
	struct ast_jtag_info *ast_jtag = file->private_data;
	void __user *argp = (void __user *)arg;
	struct jtag_xfer xfer;

	switch (cmd) {
	case AST_JTAG_GIOCFREQ:
		ret = __put_user(ast_jtag_get_freq(ast_jtag), (unsigned int __user *)arg);
		break;
	case AST_JTAG_SIOCFREQ:
//		printk("set freq = %d , pck %d \n",config.freq, ast_get_pclk());
		if ((unsigned int)arg > ast_jtag->hclk)
			ret = -EFAULT;
		else
			ast_jtag_set_freq(ast_jtag, (unsigned int)arg);

		break;
	case AST_JTAG_IOCXFER:
		if (copy_from_user(&xfer, argp, sizeof(struct jtag_xfer)))
			ret = -EFAULT;
		else
			ast_jtag_xfer(ast_jtag, &xfer);

		if (copy_to_user(argp, &xfer, sizeof(struct jtag_xfer)))
			ret = -EFAULT;
		break;
	default:
		return -ENOTTY;
	}

	return ret;
}

static int jtag_open(struct inode *inode, struct file *file)
{
//	struct ast_jtag_info *drvdata;

	spin_lock(&jtag_state_lock);

//	drvdata = container_of(inode->i_cdev, struct ast_jtag_info, cdev);

	if (ast_jtag->is_open) {
		spin_unlock(&jtag_state_lock);
		return -EBUSY;
	}

	ast_jtag->is_open = true;
	file->private_data = ast_jtag;

	spin_unlock(&jtag_state_lock);

	return 0;
}

static int jtag_release(struct inode *inode, struct file *file)
{
	struct ast_jtag_info *drvdata = file->private_data;

	spin_lock(&jtag_state_lock);

	drvdata->is_open = false;

	spin_unlock(&jtag_state_lock);

	return 0;
}

static ssize_t show_frequency(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	struct ast_jtag_info *ast_jtag = dev_get_drvdata(dev);
//	printk("PCLK = %d \n", ast_get_pclk());
//	printk("DIV  = %d \n", JTAG_GET_CLK_DIVISOR(ast_jtag_read(ast_jtag, AST_JTAG_TCK)) + 1);
	return sprintf(buf, "Frequency : %d\n", ast_jtag->hclk / (JTAG_GET_CLK_DIVISOR(ast_jtag_read(ast_jtag, AST_JTAG_GBL_CTRL)) + 1));
}

static ssize_t store_frequency(struct device *dev,
							   struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_jtag_info *ast_jtag = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 20);
	ast_jtag_set_freq(ast_jtag, val);

	return count;
}

static DEVICE_ATTR(freq, S_IRUGO | S_IWUSR, show_frequency, store_frequency);

static struct attribute *jtag_sysfs_entries[] = {
	&dev_attr_freq.attr,
	NULL
};

static struct attribute_group jtag_attribute_group = {
	.attrs = jtag_sysfs_entries,
};

static const struct file_operations ast_jtag_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= jtag_ioctl,
	.open		= jtag_open,
	.release		= jtag_release,
};

static struct miscdevice ast_jtag_misc = {
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= "ast-jtag",
	.fops 	= &ast_jtag_fops,
};

static int ast_jtag_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = 0;

	JTAG_DBUG("ast_jtag_probe\n");

	if (!(ast_jtag = devm_kzalloc(&pdev->dev, sizeof(struct ast_jtag_info), GFP_KERNEL))) {
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	ast_jtag->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!ast_jtag->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	ast_jtag->irq = platform_get_irq(pdev, 0);
	if (ast_jtag->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ast_jtag->reset = devm_reset_control_get_exclusive(&pdev->dev, "jtag");
	if (IS_ERR(ast_jtag->reset)) {
		dev_err(&pdev->dev, "can't get jtag reset\n");
		return PTR_ERR(ast_jtag->reset);
	}

	ast_jtag->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ast_jtag->clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		return -ENODEV;
	}
	ast_jtag->hclk = clk_get_rate(ast_jtag->clk);

	//scu init
	reset_control_assert(ast_jtag->reset);
	udelay(3);
	reset_control_deassert(ast_jtag->reset);

	ast_jtag_write(ast_jtag, JTAG_ENG_MODE_EN | JTAG_ENG_OUTPUT_EN | JTAG_CLK_DIV(7), AST_JTAG_GBL_CTRL);

	ret = devm_request_irq(&pdev->dev, ast_jtag->irq, ast_jtag_interrupt,
						   0, dev_name(&pdev->dev), ast_jtag);
	if (ret) {
		printk("JTAG Unable to get IRQ");
		goto out_region;
	}

	ast_jtag_write(ast_jtag, JTAG_SHIFT_COMP_ISR_EN, AST_JTAG_IER);

	init_completion(&ast_jtag->xfer_complete);

	ret = misc_register(&ast_jtag_misc);
	if (ret) {
		printk(KERN_ERR "JTAG : failed to request interrupt\n");
		goto out_irq;
	}

	platform_set_drvdata(pdev, ast_jtag);
	dev_set_drvdata(ast_jtag_misc.this_device, ast_jtag);

	ret = sysfs_create_group(&pdev->dev.kobj, &jtag_attribute_group);
	if (ret) {
		printk(KERN_ERR "ast_jtag: failed to create sysfs device attributes.\n");
		return -1;
	}

	printk(KERN_INFO "ast_jtag: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(ast_jtag->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "ast_jtag: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_jtag_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_jtag_info *ast_jtag = platform_get_drvdata(pdev);

	JTAG_DBUG("ast_jtag_remove\n");

	sysfs_remove_group(&pdev->dev.kobj, &jtag_attribute_group);

	misc_deregister(&ast_jtag_misc);

	free_irq(ast_jtag->irq, ast_jtag);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_jtag->reg_base);

	platform_set_drvdata(pdev, NULL);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

#ifdef CONFIG_PM
static int
ast_jtag_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int
ast_jtag_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

static const struct of_device_id ast_jtag_of_matches[] = {
	{ .compatible = "aspeed,ast-jtag", },
	{},
};
MODULE_DEVICE_TABLE(of, ast_jtag_of_matches);

static struct platform_driver ast_jtag_driver = {
	.probe 		= ast_jtag_probe,
	.remove 		= ast_jtag_remove,
#ifdef CONFIG_PM
	.suspend        = ast_jtag_suspend,
	.resume         = ast_jtag_resume,
#endif
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = ast_jtag_of_matches,
	},
};

module_platform_driver(ast_jtag_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST JTAG LIB Driver");
MODULE_LICENSE("GPL");
