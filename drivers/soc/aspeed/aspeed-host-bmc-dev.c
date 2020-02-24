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
#include <linux/kernel.h>       /* printk()             */
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/ide.h>
#include <asm/delay.h>
#include <linux/cdev.h>
#include <linux/timer.h>

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <linux/miscdevice.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/module.h>


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/cdev.h>		/* for cdev_ */
#include <asm/uaccess.h>        /* for put_user */

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


#define ASPEED_PCI_BMC_HOST2BMC_Q1		0x30000
#define ASPEED_PCI_BMC_HOST2BMC_Q2		0x30010
#define ASPEED_PCI_BMC_BMC2HOST_Q1		0x30020
#define ASPEED_PCI_BMC_BMC2HOST_Q2		0x30030
#define ASPEED_PCI_BMC_BMC2HOST_STS		0x30040
#define ASPEED_PCI_BMC_HOST2BMC_STS		0x30044


struct aspeed_pci_host_bmc_dev {
	struct miscdevice miscdev;	

	unsigned long mem_bar_base; 
	unsigned long mem_bar_size;
	void __iomem *virt_shared_mem;

	unsigned long message_bar_base; 
	unsigned long message_bar_size;
	void __iomem *virt_msg_bar;

	u8 IntLine;		
};

#define DRIVER_NAME "ASPEED BMC DEVICE"

irqreturn_t aspeed_pci_host_bmc_device_interrupt(int irq, void* dev_id)
{
	unsigned long status2 = 0;
	unsigned long a = 0;
        
	printk("Entered %s\n", __FUNCTION__);
#if 0
	status2 = *(volatile unsigned int *)pilotfunc1_priv.virt_b2h_sts;
	// status1 = *(volatile unsigned long *)pilotfunc1_priv.virt_h2b_sts;

	# if 0
	if((status1 & H2B_QUEUE1_EMPTY)  == H2B_QUEUE1_EMPTY)
	{
		PILOT_FUNC1_PRINT("%s H2B Queue Empty\n",__FUNCTION__);
		//up(&host1_empty_sem);
	}

	if((status1 & H2B_QUEUE2_EMPTY)  == H2B_QUEUE2_EMPTY)
	{
		PILOT_FUNC1_PRINT("%s H2B Queue Empty\n",__FUNCTION__);
		//up(&host2_empty_sem);
	}

	if((status2 & B2H_QUEUE2_FULL) == B2H_QUEUE2_FULL)
	{
		PILOT_FUNC1_PRINT("%s B2H Queue 2 FULL\n",__FUNCTION__);
		//memcpy(ioctl_data,(void *)pilotfunc1_priv.virt_b2h_q2, sizeof(ioctl_data));
		//up(&bmc2_empty_sem);
	}
	# endif

	if((status2 & B2H_QUEUE1_FULL) == B2H_QUEUE1_FULL)
	{
		PILOT_FUNC1_PRINT("%s B2H Queue 1 FULL\n",__FUNCTION__);
		a = readl(pilotfunc1_priv.virt_b2h_q1);
		up(&bmc1_empty_sem);
	}

	if((status2 & B2H_QUEUE2_FULL) == B2H_QUEUE2_FULL)
	{
		PILOT_FUNC1_PRINT("%s B2H Queue 2 FULL\n",__FUNCTION__);
		ioctl_data[0]= readl(pilotfunc1_priv.virt_b2h_q2);
		up(&bmc2_empty_sem);
	}

	if((status2 & 0x80000000))
	{
		*(volatile unsigned int *)pilotfunc1_priv.virt_b2h_sts &= ~(0x80000000);
	}
#endif
	return IRQ_HANDLED;
}

static const struct file_operations aspeed_host_bmc_char_driver_ops = {
        .owner          = THIS_MODULE,
};

static int aspeed_pci_host_bmc_device_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	unsigned int a;
	int retval = -EINVAL;
	int pci_bar = 1;
	unsigned char config_val,config_rd_val;
	
	static struct aspeed_pci_host_bmc_dev *pci_bmc_drv_data;

	dev_dbg(&pdev->dev, "%s %s(%d): ASPEED BMC PCI ID %04x:%04x\n", __FILE__,
					__func__, __LINE__, pdev->vendor, pdev->device);

	retval = pci_enable_device(pdev);
	if (retval != 0) {
			dev_err(&pdev->dev,
					"%s: pci_enable_device() returned error %d\n",
					__func__, retval);
			goto err_unreg_misc;
	}

	pci_bmc_drv_data = kzalloc(sizeof(*pci_bmc_drv_data), GFP_KERNEL);
	if (pci_bmc_drv_data == NULL) {
			retval = -ENOMEM;
			dev_err(&pdev->dev,
					"%s(%d): kmalloc() returned NULL memory.\n",
					__func__, __LINE__);
			goto err_disable_pci;
	}

	pci_bmc_drv_data->miscdev.minor = MISC_DYNAMIC_MINOR;
	pci_bmc_drv_data->miscdev.name = "host-bmc-device";
	pci_bmc_drv_data->miscdev.fops = &aspeed_host_bmc_char_driver_ops;
	pci_bmc_drv_data->miscdev.parent = dev;

	retval = misc_register(&pci_bmc_drv_data->miscdev);
	if (retval) {
			pr_err("%s(%d): CHAR registration failed of pti driver\n",
					__func__, __LINE__);
			pr_err("%s(%d): Error value returned: %d\n",
					__func__, __LINE__, retval);
			goto err;
	}

	//Get MEM bar
	pci_bmc_drv_data->mem_bar_base = pci_resource_start(pdev, pci_bar);
	pci_bmc_drv_data->mem_bar_size = pci_resource_len(pdev, 0);

	printk("BAR0 I/O Mapped Base Address is: %08x End %08x\n", pci_bmc_drv_data->mem_bar_base, pci_bmc_drv_data->mem_bar_size);

	retval = pci_request_region(pdev, pci_bar, dev_name(&pdev->dev));
	if (retval != 0) {
			dev_err(&pdev->dev,
					"%s(%d): pci_request_region() returned error %d\n",
					__func__, __LINE__, retval);
			goto err_free_dd;
	}

	pci_bmc_drv_data->virt_shared_mem = ioremap_nocache(pci_bmc_drv_data->mem_bar_base, pci_bmc_drv_data->mem_bar_size);
	if (!pci_bmc_drv_data->virt_shared_mem) {
			retval = -ENOMEM;
			goto err_rel_reg;
	}

    //Get MSG BAR info
    pci_bmc_drv_data->message_bar_base = pci_resource_start(pdev, 1);
	pci_bmc_drv_data->message_bar_size = pci_resource_len(pdev, 1);

    printk("MSG BAR1 Memory Mapped Base Address is: %08x End %08x\n", pci_bmc_drv_data->message_bar_base, pci_bmc_drv_data->message_bar_size);
			
    pci_bmc_drv_data->virt_msg_bar = ioremap_nocache(pci_bmc_drv_data->message_bar_base, pci_bmc_drv_data->message_bar_size);
	if (!pci_bmc_drv_data->virt_msg_bar) {
			retval = -ENOMEM;
			goto err_rel_reg;
	}
	pci_set_drvdata(pdev, pci_bmc_drv_data);

	printk("aspeed_pci_host_bmc_device_probe 4\n");

	//if bios does not doing mem enable , enable this code
	pci_read_config_byte(pdev, PCI_COMMAND, &config_val);
	printk("command reg= %x\n",config_val);
	config_val = config_val | PCI_COMMAND_MEMORY | PCI_COMMAND_IO;
	config_val = config_val & ~PCI_COMMAND_INTX_DISABLE;
	pci_write_config_byte( (struct pci_dev *)pdev,PCI_COMMAND,config_val);
	pci_read_config_byte( (struct pci_dev *)pdev, PCI_COMMAND,&config_rd_val);
	printk("command reg= %x\n",config_rd_val);

	pci_read_config_byte( pdev,PCI_INTERRUPT_PIN,&config_val);
	printk("INT pin reg= %x\n",config_val);
	config_val = 0x2;
	//	pci_write_config_byte( pdev,PCI_INTERRUPT_PIN,&config_val);
	//		pci_read_config_byte( pdev,PCI_INTERRUPT_PIN,&config_val);
	//	printk("INT pin reg= %x\n",config_val);
	pci_read_config_byte( (struct pci_dev *)pdev, PCI_INTERRUPT_LINE, (u8*)&pci_bmc_drv_data->IntLine);
	printk("pilotfunc1_priv.IntLine=%d\n", pci_bmc_drv_data->IntLine);

	pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &pci_bmc_drv_data->IntLine);

#if 0
	printk ("H2B Status reg = %x\n", *(volatile unsigned int *)(pilotfunc1_priv.virt_h2b_sts));
	*(volatile unsigned int *)(pilotfunc1_priv.virt_h2b_sts) = 0x40a00000;

	printk ("H2B Status reg = %x\n", *(volatile unsigned int *)(pilotfunc1_priv.virt_h2b_sts));
	printk ("B2H Status reg = %x\n", *(volatile unsigned int *)(pilotfunc1_priv.virt_b2h_sts));

	printk("IRQ Used is: 0x%x \n", pilotfunc1_priv.IntLine);
	printk("See %x\n", *(volatile unsigned int *)pilotfunc1_priv.virt_shared_mem);
#endif

	printk ("Before pdev->irq:%d\n", pdev->irq);
	retval = pci_enable_msi(pdev);
	if (retval)
	{
		printk("MSI failed. et:%d\n", retval);
	}

	printk ("After pdev->irq:%d\n", pdev->irq);
//	pilotfunc1_priv.IntLine = pdev->irq;

	retval = request_irq(pdev->irq, aspeed_pci_host_bmc_device_interrupt, IRQF_SHARED, "ASPEED BMC DEVICE", pci_bmc_drv_data);

//	pci_intx(pcr->pci, !pcr->msi_en);

	return 0;

err_rel_reg:
	pci_release_region(pdev, pci_bar);
err_free_dd:
	kfree(pci_bmc_drv_data);
err_disable_pci:
	pci_disable_device(pdev);
err_unreg_misc:
	misc_deregister(&pci_bmc_drv_data->miscdev);
err:
	return retval;
	
}

static void aspeed_pci_host_bmc_device_remove(struct pci_dev *pdev)
{
#if 0
	iounmap(pilotfunc1_priv.virt_msg_bar);
	iounmap(pilotfunc1_priv.virt_shared_mem);
	free_irq(pilotfunc1_priv.IntLine, pdev);
#endif	
	/* release the IO region */
	pci_release_regions(pdev);
	
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
	.name 		= DRIVER_NAME,
	.id_table 	= aspeed_host_bmc_dev_pci_ids,
	.probe 		= aspeed_pci_host_bmc_device_probe,
	.remove 	= aspeed_pci_host_bmc_device_remove,
};

static int __init aspeed_host_bmc_device_init(void)
{
	int ret;

	printk(KERN_DEBUG "Module pci init\n");

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
	int i;

	/* unregister pci driver */
	pci_unregister_driver(&aspeed_host_bmc_dev_driver);


	printk(KERN_DEBUG "Module pci exit\n");	
}

module_init(aspeed_host_bmc_device_init);
module_exit(aspeed_host_bmc_device_exit);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED Host BMC DEVICE Driver");
MODULE_LICENSE("GPL");
