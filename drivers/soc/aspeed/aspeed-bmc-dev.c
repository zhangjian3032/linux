/*
 * BMC device driver for the Aspeed SoC
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>       /* printk()             */
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/ide.h>
#include <linux/pci.h>
#include <asm/delay.h>

#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include <linux/io.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/module.h>

#define DEVICE_NAME     "bmc-device"

struct aspeed_bmc_device {
	unsigned char *host2bmc_base_virt;
	struct miscdevice	miscdev;
	void __iomem	*reg_base;	
	void __iomem	*bmc_mem_virt;
	dma_addr_t bmc_mem_phy;
	struct bin_attribute	bin0;
	struct bin_attribute	bin1;



//	phys_addr_t		mem_base;
//	resource_size_t		mem_size;

	struct kernfs_node	*kn0;
	struct kernfs_node	*kn1;
	
	unsigned int irq;	
};

/* ================================================================================== */
#define ASPEED_BMC_CONFIG_SPACE		0xF00
#define ASPEED_BMC_MEM_BAR			0xF10
#define ASPEED_BMC_MSG_BAR_REMAP	0xF14
#define ASPEED_BMC_MEM_BAR_REMAP	0xF18

#define ASPEED_BMC_SHADOW_CTRL		0xF50
#define  READ_ONLY_MASK					BIT(31)
#define  MASK_BAR1						BIT(2)
#define  MASK_BAR0						BIT(1)
#define  SHADOW_CFG						BIT(0)

#define ASPEED_BMC_HOST2BMC_Q1		0xA000
#define ASPEED_BMC_HOST2BMC_Q2		0xA010
#define ASPEED_BMC_BMC2HOST_Q1		0xA020
#define ASPEED_BMC_BMC2HOST_Q2		0xA030
#define ASPEED_BMC_BMC2HOST_STS		0xA040
#define	 BMC2HOST_INT_STS_DOORBELL		BIT(31)
#define	 BMC2HOST_ENABLE_INTB			BIT(30)
/* */
#define	 BMC2HOST_Q1_FULL				BIT(27)
#define	 BMC2HOST_Q1_EMPTY				BIT(26)
#define	 BMC2HOST_Q2_FULL				BIT(25)
#define	 BMC2HOST_Q2_EMPTY				BIT(24)
#define	 BMC2HOST_Q1_FULL_UNMASK		BIT(23)
#define	 BMC2HOST_Q1_EMPTY_UNMASK		BIT(22)
#define	 BMC2HOST_Q2_FULL_UNMASK		BIT(21)
#define	 BMC2HOST_Q2_EMPTY_UNMASK		BIT(20)

#define ASPEED_BMC_HOST2BMC_STS		0xA044
#define	 HOST2BMC_INT_STS_DOORBELL		BIT(31)
#define	 HOST2BMC_ENABLE_INTB			BIT(30)
/* */
#define	 HOST2BMC_Q1_FULL				BIT(27)
#define	 HOST2BMC_Q1_EMPTY				BIT(26)
#define	 HOST2BMC_Q2_FULL				BIT(25)
#define	 HOST2BMC_Q2_EMPTY				BIT(24)
#define	 HOST2BMC_Q1_FULL_UNMASK		BIT(23)
#define	 HOST2BMC_Q1_EMPTY_UNMASK		BIT(22)
#define	 HOST2BMC_Q2_FULL_UNMASK		BIT(21)
#define	 HOST2BMC_Q2_EMPTY_UNMASK		BIT(20)

static struct aspeed_bmc_device *file_aspeed_bmc_device(struct file *file)
{
	return container_of(file->private_data, struct aspeed_bmc_device,
			miscdev);
}

static int aspeed_bmc_device_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct aspeed_bmc_device *bmc_device = file_aspeed_bmc_device(file);
	unsigned long vsize = vma->vm_end - vma->vm_start;
	pgprot_t prot = vma->vm_page_prot;

	if (vma->vm_pgoff + vsize > bmc_device->bmc_mem_phy + 0x100000)
		return -EINVAL;

	prot = pgprot_noncached(prot);

	if (remap_pfn_range(vma, vma->vm_start,
		(bmc_device->bmc_mem_phy >> PAGE_SHIFT) + vma->vm_pgoff,
		vsize, prot))
		return -EAGAIN;

	return 0;
}

static const struct file_operations aspeed_bmc_device_fops = {
	.owner		= THIS_MODULE,
	.mmap		= aspeed_bmc_device_mmap,
};

static ssize_t aspeed_host2bmc_queue1_rx(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct aspeed_bmc_device *bmc_device = dev_get_drvdata(container_of(kobj, struct device, kobj));

	//read from bmc fifo ~~~
	printk("aspeed_host2bmc_queue1_rx \n");
	if(!(readl(bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS) & HOST2BMC_Q1_EMPTY)) {
		printk("! HOST2BMC_Q1_EMPTY \n");
		buf = readl(bmc_device->reg_base + ASPEED_BMC_HOST2BMC_Q1);
		return 4;
	} else 
		return 0;

}

static ssize_t aspeed_host2bmc_queue2_rx(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	u32 write_pt;
	struct aspeed_bmc_device *bmc_device = dev_get_drvdata(container_of(kobj, struct device, kobj));

	//read from bmc fifo ~~~
	printk("aspeed_host2bmc_queue2_rx \n");
	if(!(readl(bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS) & HOST2BMC_Q2_EMPTY)) {
		printk("! HOST2BMC_Q1_EMPTY \n");
		buf = readl(bmc_device->reg_base + ASPEED_BMC_HOST2BMC_Q2);
		return 4;
	} else 
		return 0;
	
	return count;
}

static ssize_t aspeed_bmc2host_queue1_tx(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	u32 tx_buff;
	struct aspeed_bmc_device *bmc_device = dev_get_drvdata(container_of(kobj, struct device, kobj));	

	if(count != 4)
		return -1;
	
	if(readl(bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS) & BMC2HOST_Q1_FULL)
		return -1;
	else {
		memcpy(&tx_buff, buf, 4);
		printk("tx_buff %x \n", tx_buff);
		writel(tx_buff, bmc_device->reg_base + ASPEED_BMC_BMC2HOST_Q1);
		return 4;
	}
}

static ssize_t aspeed_bmc2host_queue2_tx(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	u32 tx_buff = 0;
	struct aspeed_bmc_device *bmc_device = dev_get_drvdata(container_of(kobj, struct device, kobj));	

	if(count != 4)
		return -1;
	
	if(readl(bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS) & BMC2HOST_Q2_FULL)
		return -1;
	else {
		memcpy(&tx_buff, buf, 4);
		printk("tx_buff %x \n", tx_buff);
		writel(tx_buff, bmc_device->reg_base + ASPEED_BMC_BMC2HOST_Q2);
		return 4;
	}
}

static irqreturn_t aspeed_bmc_dev_isr(int irq, void *dev_id)
{
	struct aspeed_bmc_device *bmc_device = dev_id;

	u32 bmc2host_q_sts = readl(bmc_device->reg_base + ASPEED_BMC_BMC2HOST_STS);
	u32 host2bmc_q_sts = readl(bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS);

	printk("%s bmc2host_q_sts is %x host2bmc_q_sts is %x\n", __FUNCTION__, bmc2host_q_sts, host2bmc_q_sts);

	if(host2bmc_q_sts & HOST2BMC_INT_STS_DOORBELL) {
		writel(HOST2BMC_INT_STS_DOORBELL, bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS);
	}

	if(host2bmc_q_sts & HOST2BMC_ENABLE_INTB) {
		writel(HOST2BMC_ENABLE_INTB, bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS);
	}
	if(host2bmc_q_sts & HOST2BMC_Q1_FULL) {
	}	

	if(host2bmc_q_sts & HOST2BMC_Q2_FULL) {
	}	

	if(bmc2host_q_sts & BMC2HOST_INT_STS_DOORBELL) {
	}

	if(bmc2host_q_sts & BMC2HOST_ENABLE_INTB) {
	}

	return IRQ_HANDLED;
}


static void aspeed_bmc_device_init(struct aspeed_bmc_device *bmc_device)
{
	printk("aspeed_bmc_device_init \n");

	writel(0x080019A2, bmc_device->reg_base);
//	writel(0x00000001, bmc_device->reg_base + 0x04);
	writel(0x0B400000, bmc_device->reg_base + 0x08);
	writel(0x00000000, bmc_device->reg_base + 0x0C);	// It was 0x01
//
//Bit 31:2 – This is a programmable range where the amount of memory needed is exposed.
//Bit 1 – Enable Memory BAR on PCIe2PCI Bridge.
//Bit 0 – Enable Memory BAR on Host2BMC Device.
//When the Host System boots, it scans the MEM_BAR register. The bits [31:4] will indicate the range of memory exposed. 
//Note:  Supports only 4byte (32 bits) aligned memory read and writes
//            Remap address has to be programmed as 64KB boundary only
	writel(0xfff00003, bmc_device->reg_base + 0x10);	
/*
The MSG_BAR has need not be initialized from the BMC side. 
The default values will request 256 KB of memory resources from the host. 
The host can access the  MSG_BAR register at offset MSG_BAR +0x30000, see section 8.2 for the address map.
*/
	writel(bmc_device->bmc_mem_phy, bmc_device->reg_base + 0x18);
	writel(0x00000000, bmc_device->reg_base + 0x1C);
	writel(0x00000000, bmc_device->reg_base + 0x20);
	writel(0x00000000, bmc_device->reg_base + 0x28);
	writel(0x080019A2, bmc_device->reg_base + 0x2C);
	
//	writel(0xFFFFFFFF, bmc_device->reg_base + 0x30);
	writel(0x00000000, bmc_device->reg_base + 0x34);
	writel(0x00000000, bmc_device->reg_base + 0x38);
	writel(0x000001FF, bmc_device->reg_base + 0x3C);

	//Setting BMC to Host Q register
	writel(BMC2HOST_Q2_FULL_UNMASK| BMC2HOST_Q1_FULL_UNMASK | BMC2HOST_ENABLE_INTB, bmc_device->reg_base + ASPEED_BMC_BMC2HOST_STS);
	writel(HOST2BMC_Q2_FULL_UNMASK| HOST2BMC_Q1_FULL_UNMASK | HOST2BMC_ENABLE_INTB, bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS);
}

static const struct of_device_id aspeed_bmc_device_of_matches[] = {
	{ .compatible = "aspeed,ast2600-bmc-device", },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_bmc_device_of_matches);

#define BMC_QUEUE_SIZE (16 * 4)
static int aspeed_bmc_device_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aspeed_bmc_device *bmc_device;
	int ret;
	int rc;	

	bmc_device = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_bmc_device), GFP_KERNEL);
	if (!bmc_device)
		return -ENOMEM;

	bmc_device->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bmc_device->reg_base))
		return PTR_ERR(bmc_device->reg_base);

	bmc_device->bmc_mem_virt = dma_alloc_coherent(&pdev->dev, 0x100000, &bmc_device->bmc_mem_phy, GFP_KERNEL);
	memset(bmc_device->bmc_mem_virt, 0, 0x100000);
	
	printk("virt=%x phy %x\n", bmc_device->bmc_mem_virt, bmc_device->bmc_mem_phy);

	sysfs_bin_attr_init(&bmc_device->bin0);
	sysfs_bin_attr_init(&bmc_device->bin1);

	bmc_device->bin0.attr.name = "bmc-dev-queue1";
	bmc_device->bin0.attr.mode = S_IRUSR | S_IWUSR;
	bmc_device->bin0.read = aspeed_host2bmc_queue1_rx;
	bmc_device->bin0.write = aspeed_bmc2host_queue1_tx;
	bmc_device->bin0.size = BMC_QUEUE_SIZE;

	rc = sysfs_create_bin_file(&pdev->dev.kobj, &bmc_device->bin0);
	if (rc) {
		printk("error for bin file ");
		return rc;
	}

	bmc_device->kn0 = kernfs_find_and_get(dev->kobj.sd, bmc_device->bin0.attr.name);
	if (!bmc_device->kn0) {
		sysfs_remove_bin_file(&dev->kobj, &bmc_device->bin0);
		return -EFAULT;
	}

	bmc_device->bin1.attr.name = "bmc-dev-queue2";
	bmc_device->bin1.attr.mode = S_IRUSR | S_IWUSR;
	bmc_device->bin1.read = aspeed_host2bmc_queue2_rx;
	bmc_device->bin1.write = aspeed_bmc2host_queue2_tx;
	bmc_device->bin1.size = BMC_QUEUE_SIZE;

	rc = sysfs_create_bin_file(&pdev->dev.kobj, &bmc_device->bin1);
	if (rc) {
		printk("error for bin file ");
		return rc;
	}

	bmc_device->kn1 = kernfs_find_and_get(dev->kobj.sd, bmc_device->bin1.attr.name);
	if (!bmc_device->kn1) {
		sysfs_remove_bin_file(&dev->kobj, &bmc_device->bin1);
		return -EFAULT;
	}

printk("-------------------------------------------------------------------------------------------------------------------3\n");	
	dev_set_drvdata(dev, bmc_device);

	aspeed_bmc_device_init(bmc_device);

	bmc_device->irq =  platform_get_irq(pdev,0);
	if(bmc_device->irq < 0) {
		dev_err(&pdev->dev,"platform get of irq[=%d] failed!\n", bmc_device->irq);
		return -ENODEV;
	}
	printk("-------------------------------------------------------------------------------------------------------------------4\n");

	ret = devm_request_irq(&pdev->dev, bmc_device->irq, aspeed_bmc_dev_isr,
							0, dev_name(&pdev->dev), bmc_device);
	if (ret) {
		printk("aspeed bmc device Unable to get IRQ");
		goto out_region;
	}

	bmc_device->miscdev.minor = MISC_DYNAMIC_MINOR;
	bmc_device->miscdev.name = DEVICE_NAME;
	bmc_device->miscdev.fops = &aspeed_bmc_device_fops;
	bmc_device->miscdev.parent = dev;
	rc = misc_register(&bmc_device->miscdev);
	if (rc) {
		dev_err(dev, "Unable to register device\n");
		return rc;
	}

	printk(KERN_INFO "aspeed bmc device: driver successfully loaded.\n");

	return 0;

out_irq:
	devm_free_irq(&pdev->dev, bmc_device->irq, bmc_device);

out_region:
	kfree(bmc_device);

out:
	printk(KERN_WARNING "aspeed bmc device: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int  aspeed_bmc_device_remove( struct platform_device *pdev)
{
	struct aspeed_bmc_device *bmc_device = platform_get_drvdata(pdev);
	unsigned int ret;
#if 0
	misc_deregister(bmc_device->misc_dev);

	kfree(bmc_device->misc_dev);

	devm_free_irq(&pdev->dev, bmc_device->irq, bmc_device);

	iounmap(bmc_device->reg_base);
	release_mem_region(res->start, res->end - res->start + 1);

	if (bmc_device->ddr_virt_addr) {
		dma_free_coherent(&pdev->dev,
				priv->ddr_len,
				priv->ddr_virt_addr,
				priv->ddr_phy_addr);
	}

	iounmap(priv->reg_base);
#endif
	
    return 0;
}


static struct platform_driver aspeed_bmc_device_driver = {
	.probe		= aspeed_bmc_device_probe,
	.remove		= aspeed_bmc_device_remove,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = aspeed_bmc_device_of_matches,
	},
};

module_platform_driver(aspeed_bmc_device_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED BMC DEVICE Driver");
MODULE_LICENSE("GPL");
