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
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>

#include <linux/pci.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/interrupt.h>

#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>

#include "../../char/ipmi/ipmi_si.h"

#define ASPEED_PCI_BMC_HOST2BMC_Q1		0x30000
#define ASPEED_PCI_BMC_HOST2BMC_Q2		0x30010
#define ASPEED_PCI_BMC_BMC2HOST_Q1		0x30020
#define ASPEED_PCI_BMC_BMC2HOST_Q2		0x30030
#define ASPEED_PCI_BMC_BMC2HOST_STS		0x30040
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

#define ASPEED_PCI_BMC_HOST2BMC_STS		0x30044
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

struct aspeed_pci_bmc_dev {
	struct miscdevice miscdev;

	unsigned long mem_bar_base;
	unsigned long mem_bar_size;
	void __iomem *mem_bar_reg;

	unsigned long message_bar_base;
	unsigned long message_bar_size;
	void __iomem *msg_bar_reg;

	struct bin_attribute	bin0;
	struct bin_attribute	bin1;

	struct kernfs_node	*kn0;
	struct kernfs_node	*kn1;

	void *serial_priv;

	u8 IntLine;
};

#define HOST_BMC_QUEUE_SIZE			(16 * 4)

#define BMC_MSI_INT 1

#define DRIVER_NAME "ASPEED BMC DEVICE"

static uint16_t ioport_kcs = 0x3a2;
static uint16_t ioport_vuart = 0x3f8;

module_param(ioport_kcs, ushort, 0644);
MODULE_PARM_DESC(ioport_kcs, "The I/O port base address of IPMI-KCS over PCIe");

module_param(ioport_vuart, ushort, 0644);
MODULE_PARM_DESC(ioport_vuart, "The I/O port base address of VUART over PCIe");

static struct aspeed_pci_bmc_dev *file_aspeed_bmc_device(struct file *file)
{
	return container_of(file->private_data, struct aspeed_pci_bmc_dev,
			miscdev);
}

static int aspeed_pci_bmc_dev_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = file_aspeed_bmc_device(file);
	unsigned long vsize = vma->vm_end - vma->vm_start;
	pgprot_t prot = vma->vm_page_prot;

	if (vma->vm_pgoff + vsize > pci_bmc_dev->mem_bar_base + 0x100000)
		return -EINVAL;

	prot = pgprot_noncached(prot);

	if (remap_pfn_range(vma, vma->vm_start,
		(pci_bmc_dev->mem_bar_base >> PAGE_SHIFT) + vma->vm_pgoff,
		vsize, prot))
		return -EAGAIN;

	return 0;
}

static const struct file_operations aspeed_pci_bmc_dev_fops = {
	.owner		= THIS_MODULE,
	.mmap		= aspeed_pci_bmc_dev_mmap,
};

static ssize_t aspeed_pci_bmc_dev_queue1_rx(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct aspeed_pci_bmc_dev *pci_bmc_device = dev_get_drvdata(container_of(kobj, struct device, kobj));
	u32 *data = (u32 *) buf;

//	printk("aspeed_pci_bmc_dev_queue1_rx \n");

	if (readl(pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_BMC2HOST_STS) & BMC2HOST_Q1_EMPTY)
		return 0;
	else {
		data[0] = readl(pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_BMC2HOST_Q1);
//		printk("Got BMC2HOST_Q1 [%x] \n", data[0]);
		return 4;
	}
}

static ssize_t aspeed_pci_bmc_dev_queue2_rx(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct aspeed_pci_bmc_dev *pci_bmc_device = dev_get_drvdata(container_of(kobj, struct device, kobj));
	u32 *data = (u32 *) buf;

//	printk("aspeed_pci_bmc_dev_queue2_rx \n");

	if (!(readl(pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_BMC2HOST_STS) & BMC2HOST_Q2_EMPTY)) {
		data[0] = readl(pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_BMC2HOST_Q2);
//		printk("Got BMC2HOST_Q2 [%x] \n", data[0]);
		return 4;
	} else
		return 0;

	return count;
}

static ssize_t aspeed_pci_bmc_dev_queue1_tx(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	u32 tx_buff;
	struct aspeed_pci_bmc_dev *pci_bmc_device = dev_get_drvdata(container_of(kobj, struct device, kobj));

	if(count != 4)
		return -1;

	if(readl(pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_HOST2BMC_STS) & HOST2BMC_Q1_FULL)
		return -1;
	else {
		memcpy(&tx_buff, buf, 4);
//		printk("tx_buff %x \n", tx_buff);
		writel(tx_buff, pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_HOST2BMC_Q1);
		//trigger to host 
		writel(HOST2BMC_INT_STS_DOORBELL | HOST2BMC_ENABLE_INTB, pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_HOST2BMC_STS);
		return 4;
	}

}

static ssize_t aspeed_pci_bmc_dev_queue2_tx(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	u32 tx_buff = 0;
	struct aspeed_pci_bmc_dev *pci_bmc_device = dev_get_drvdata(container_of(kobj, struct device, kobj));

	if(count != 4)
		return -1;

	if(readl(pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_HOST2BMC_STS) & HOST2BMC_Q2_FULL)
		return -1;
	else {
		memcpy(&tx_buff, buf, 4);
//		printk("tx_buff %x \n", tx_buff);
		writel(tx_buff, pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_HOST2BMC_Q2);
		//trigger to host
		writel(HOST2BMC_INT_STS_DOORBELL | HOST2BMC_ENABLE_INTB, pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_HOST2BMC_STS);
		return 4;
	}
}

irqreturn_t aspeed_pci_host_bmc_device_interrupt(int irq, void* dev_id)
{
	struct aspeed_pci_bmc_dev *pci_bmc_device = dev_id;

	u32 bmc2host_q_sts = readl(pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_BMC2HOST_STS);

//	printk("%s bmc2host_q_sts is %x \n", __FUNCTION__, bmc2host_q_sts);

	if(bmc2host_q_sts & BMC2HOST_INT_STS_DOORBELL) {
		writel(BMC2HOST_INT_STS_DOORBELL, pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_BMC2HOST_STS);
	}

	if(bmc2host_q_sts & BMC2HOST_ENABLE_INTB) {
		writel(BMC2HOST_ENABLE_INTB, pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_BMC2HOST_STS);
	}
	if(bmc2host_q_sts & BMC2HOST_Q1_FULL) {
	}

	if(bmc2host_q_sts & BMC2HOST_Q2_FULL) {
	}

	return IRQ_HANDLED;

}


static int aspeed_pci_host_bmc_device_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
#ifdef CONFIG_IPMI_SI
	struct si_sm_io kcs_io;
#endif
	struct uart_8250_port uart;
	struct aspeed_pci_bmc_dev *pci_bmc_dev;
	struct device *dev = &pdev->dev;
	u16 config_cmd_val;
	int rc = 0;

	dev_dbg(&pdev->dev, "ASPEED BMC PCI ID %04x:%04x, IRQ=%u\n", pdev->vendor, pdev->device, pdev->irq);

	rc = pci_enable_device(pdev);
	if (rc != 0) {
		dev_err(&pdev->dev, "pci_enable_device() returned error %d\n", rc);
		goto out_err;
	}

	/* set PCI host mastering  */
	pci_set_master(pdev);

#if BMC_MSI_INT
	rc = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI);
	if (rc < 0)
		printk("cannot allocate PCI MSI, rc=%d\n", rc);

	pci_read_config_word(pdev, PCI_COMMAND, &config_cmd_val);
	printk("config_cmd_val %x \n", config_cmd_val);
	config_cmd_val |= PCI_COMMAND_INTX_DISABLE;
	printk("config_cmd_val %x \n", config_cmd_val);
	pci_write_config_word((struct pci_dev *)pdev, PCI_COMMAND, config_cmd_val);
#else
	rc = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_LEGACY);
	if (rc < 0)
		printk("cannot allocate PCI INTx, rc=%d\n", rc);

	pci_read_config_word(pdev, PCI_COMMAND, &config_cmd_val);
	printk("config_cmd_val %x \n", config_cmd_val);
	config_cmd_val &= ~PCI_COMMAND_INTX_DISABLE;
	printk("config_cmd_val %x \n", config_cmd_val);
	pci_write_config_word((struct pci_dev *)pdev, PCI_COMMAND, config_cmd_val);
#endif

	pci_bmc_dev = kzalloc(sizeof(*pci_bmc_dev), GFP_KERNEL);
	if (pci_bmc_dev == NULL) {
			rc = -ENOMEM;
			dev_err(&pdev->dev, "kmalloc() returned NULL memory.\n");
			goto out_err;
	}

	//Get MEM bar
	pci_bmc_dev->mem_bar_base = pci_resource_start(pdev, 0);
	pci_bmc_dev->mem_bar_size = pci_resource_len(pdev, 0);

	printk("BAR0 I/O Mapped Base Address is: %08lx End %08lx\n", pci_bmc_dev->mem_bar_base, pci_bmc_dev->mem_bar_size);

	rc = pci_request_region(pdev, 0, dev_name(&pdev->dev));
	if (rc != 0) {
		dev_err(&pdev->dev, "pci_request_region returned error %d\n", rc);
	}

	pci_bmc_dev->mem_bar_reg = ioremap_nocache(pci_bmc_dev->mem_bar_base, pci_bmc_dev->mem_bar_size);
	if (!pci_bmc_dev->mem_bar_reg) {
		rc = -ENOMEM;
		goto out_free0;
	}

    //Get MSG BAR info
    pci_bmc_dev->message_bar_base = pci_resource_start(pdev, 1);
	pci_bmc_dev->message_bar_size = pci_resource_len(pdev, 1);

    printk("MSG BAR1 Memory Mapped Base Address is: %08lx End %08lx\n", pci_bmc_dev->message_bar_base, pci_bmc_dev->message_bar_size);

    pci_bmc_dev->msg_bar_reg = ioremap_nocache(pci_bmc_dev->message_bar_base, pci_bmc_dev->message_bar_size);
	if (!pci_bmc_dev->msg_bar_reg) {
		rc = -ENOMEM;
		goto out_free1;
	}

	/* ERRTA40: dummy read */
	rc = *((volatile uint32_t *)pci_bmc_dev->msg_bar_reg);

	sysfs_bin_attr_init(&pci_bmc_dev->bin0);
	sysfs_bin_attr_init(&pci_bmc_dev->bin1);

	pci_bmc_dev->bin0.attr.name = "pci-bmc-dev-queue1";
	pci_bmc_dev->bin0.attr.mode = S_IRUSR | S_IWUSR;
	pci_bmc_dev->bin0.read = aspeed_pci_bmc_dev_queue1_rx;
	pci_bmc_dev->bin0.write = aspeed_pci_bmc_dev_queue1_tx;
	pci_bmc_dev->bin0.size = 4;

	rc = sysfs_create_bin_file(&pdev->dev.kobj, &pci_bmc_dev->bin0);
	if (rc) {
		printk("error for bin file ");
		goto out_free1;
	}

	pci_bmc_dev->kn0 = kernfs_find_and_get(dev->kobj.sd, pci_bmc_dev->bin0.attr.name);
	if (!pci_bmc_dev->kn0) {
		sysfs_remove_bin_file(&dev->kobj, &pci_bmc_dev->bin0);
		goto out_free1;
	}

	pci_bmc_dev->bin1.attr.name = "pci-bmc-dev-queue2";
	pci_bmc_dev->bin1.attr.mode = S_IRUSR | S_IWUSR;
	pci_bmc_dev->bin1.read = aspeed_pci_bmc_dev_queue2_rx;
	pci_bmc_dev->bin1.write = aspeed_pci_bmc_dev_queue2_tx;
	pci_bmc_dev->bin1.size = 4;

	rc = sysfs_create_bin_file(&pdev->dev.kobj, &pci_bmc_dev->bin1);
	if (rc) {
		sysfs_remove_bin_file(&dev->kobj, &pci_bmc_dev->bin1);
		goto out_free1;
	}

	pci_bmc_dev->kn1 = kernfs_find_and_get(dev->kobj.sd, pci_bmc_dev->bin1.attr.name);
	if (!pci_bmc_dev->kn1) {
		sysfs_remove_bin_file(&dev->kobj, &pci_bmc_dev->bin1);
		goto out_free1;
	}

	pci_bmc_dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	pci_bmc_dev->miscdev.name = DRIVER_NAME;
	pci_bmc_dev->miscdev.fops = &aspeed_pci_bmc_dev_fops;
	pci_bmc_dev->miscdev.parent = dev;

	rc = misc_register(&pci_bmc_dev->miscdev);
	if (rc) {
		pr_err("host bmc register fail %d\n",rc);
		goto out_free;
	}

	pci_set_drvdata(pdev, pci_bmc_dev);

	rc = request_irq(pdev->irq, aspeed_pci_host_bmc_device_interrupt, IRQF_SHARED, "ASPEED BMC DEVICE", pci_bmc_dev);
	if (rc) {
		pr_err("host bmc device Unable to get IRQ %d\n",rc);
		goto out_unreg;
	}

#ifdef CONFIG_IPMI_SI
	/* setup IPMI-KCS over PCIe */
	memset(&kcs_io, 0, sizeof(kcs_io));
	kcs_io.addr_source = SI_PCI;
	kcs_io.addr_source_data = pdev;
	kcs_io.si_type = SI_KCS;
	kcs_io.addr_space = IPMI_MEM_ADDR_SPACE;
	kcs_io.io_setup = ipmi_si_mem_setup;
	kcs_io.addr_data = pci_bmc_dev->message_bar_base + (ioport_kcs << 2);
	kcs_io.dev = &pdev->dev;
	kcs_io.regspacing = 4;
	kcs_io.regsize = 1;
	kcs_io.regshift = 0;
	kcs_io.irq = pci_irq_vector(pdev, 0);
	if (kcs_io.irq)
		kcs_io.irq_setup = ipmi_std_irq_setup;

	rc = ipmi_si_add_smi(&kcs_io);
	if (rc)
		dev_err(dev, "cannot setup IPMI-KCS@%xh over PCIe, rc=%d\n", ioport_kcs, rc);
#endif

	/* setup VUART */
	memset(&uart, 0, sizeof(uart));

	uart.port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ;
	uart.port.uartclk = 115200 * 16;

	uart.port.irq = pci_irq_vector(pdev, 0);
	uart.port.dev = &pdev->dev;

	uart.port.iotype = UPIO_MEM32;
	uart.port.iobase = 0;
	uart.port.mapbase = pci_bmc_dev->message_bar_base + (ioport_vuart << 2);
	uart.port.membase = 0;
	uart.port.type = PORT_16550A;
	uart.port.flags |= (UPF_IOREMAP | UPF_FIXED_PORT | UPF_FIXED_TYPE);
	uart.port.regshift = 2;

	rc = serial8250_register_8250_port(&uart);
	if (rc < 0)
		dev_err(dev, "cannot setup VUART@%xh over PCIe, rc=%d\n", ioport_vuart, rc);
	return 0;

out_unreg:
	misc_deregister(&pci_bmc_dev->miscdev);
out_free1:
	pci_release_region(pdev, 1);
out_free0:
	pci_release_region(pdev, 0);
out_free:
	kfree(pci_bmc_dev);
out_err:
	pci_disable_device(pdev);

	return rc;

}

static void aspeed_pci_host_bmc_device_remove(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);

	free_irq(pdev->irq, pdev);
	misc_deregister(&pci_bmc_dev->miscdev);
	pci_release_regions(pdev);
	kfree(pci_bmc_dev);
	pci_disable_device(pdev);
}

/**
 * This table holds the list of (VendorID,DeviceID) supported by this driver
 *
 */
static struct pci_device_id aspeed_host_bmc_dev_pci_ids[] = {
	{ PCI_DEVICE(0x1A03, 0x2402), },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, aspeed_host_bmc_dev_pci_ids);

static struct pci_driver aspeed_host_bmc_dev_driver = {
	.name		= DRIVER_NAME,
	.id_table	= aspeed_host_bmc_dev_pci_ids,
	.probe		= aspeed_pci_host_bmc_device_probe,
	.remove		= aspeed_pci_host_bmc_device_remove,
};

static int __init aspeed_host_bmc_device_init(void)
{
	int ret;

	/* register pci driver */
	ret = pci_register_driver(&aspeed_host_bmc_dev_driver);
	if (ret < 0) {
		printk(KERN_ERR "pci-driver: can't register pci driver\n");
		return ret;
	}

	return 0;

}

static void aspeed_host_bmc_device_exit(void)
{
	/* unregister pci driver */
	pci_unregister_driver(&aspeed_host_bmc_dev_driver);
}

late_initcall(aspeed_host_bmc_device_init);
module_exit(aspeed_host_bmc_device_exit);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED Host BMC DEVICE Driver");
MODULE_LICENSE("GPL");
