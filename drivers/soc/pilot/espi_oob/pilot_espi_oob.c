/*******************************************************************************
 *
 * Copyright (C) 2005-2015 Emulex. All rights reserved.
 * EMULEX is a trademark of Emulex.
 * www.emulex.com
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of version 2 of the GNU General Public License as published by the
 * Free Software Foundation.
 * This program is distributed in the hope that it will be useful. ALL EXPRESS
 * OR IMPLIED CONDITIONS, REPRESENTATIONS AND WARRANTIES, INCLUDING ANY IMPLIED
 * WARRANTY OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
 * NON-INFRINGEMENT, ARE DISCLAIMED, EXCEPT TO THE EXTENT THAT SUCH DISCLAIMERS
 * ARE HELD TO BE LEGALLY INVALID. See the GNU General Public License for more
 * details, a copy of which can be found in the file COPYING included
 * with this package.
 *
 ********************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/ide.h>
#include <linux/spinlock.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include "espi_oob.h"

#define RETURN_IF_ESPI_RESET_HAPPENED(); \
    do { \
        if (espi_oob_reset)   \
        return -ECONNRESET; \
    } while(0)

static int ESPI_MAJOR = 0;
static const char ESPI_OOB_DEV[] = "eSPI_OOB";
volatile unsigned char *se_pilot_espi_va_base;
volatile unsigned char *se_sys_clk_va_base;
unsigned int irq_espi_oob;
unsigned int irq_sio_psr;

static int pilot_espi_oob_probe(struct platform_device *pdev);
static int pilot_espi_oob_remove(struct platform_device *pdev);

static int espi_oob_open(struct inode * inode, struct file * filp);
static int espi_oob_close(struct inode * inode, struct file * filp);

static int espi_registered;
static int espi_oob_majnum;
static int espi_oob_irq_regd;
static spinlock_t espioob_lock;
static unsigned int espi_oob_intrsts= 0;
wait_queue_head_t espioob_intr_wq;

long espi_oob_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static int espi_oob_reset = 0;

static struct file_operations espi_fops = {
    owner:		THIS_MODULE,
    open:		espi_oob_open,
    release:    espi_oob_close,
    unlocked_ioctl: espi_oob_ioctl,
};

int espi_oob_reset_handler(void)
{
    unsigned long flags;

    printk(KERN_DEBUG "ESPIOOB: got eSPI RESET\n");
    spin_lock_irqsave(&espioob_lock, flags);
    espi_oob_reset = 1;
    espi_oob_intrsts = 0;

    // clear the RD/WR PTRs
    *(volatile unsigned int*)(ESPI_OOB_CTRL) |= (B2H_WPTR_CLR | B2H_RPTR_CLR | H2B_WPTR_CLR | H2B_RPTR_CLR);
    spin_unlock_irqrestore(&espioob_lock, flags);

    wake_up_interruptible(&espioob_intr_wq);
    return 0;
}

static irqreturn_t oobdummy_irqhandler(int irq, void *dev_id)
{
    printk(KERN_DEBUG "ESPIOOB: dummy handler\n");
    return IRQ_HANDLED;
}

static irqreturn_t espi_oob_handler(int irq, void *dev_id)
{
    unsigned long flags;
    unsigned char status;

    status = *(volatile unsigned char*) (ESPI_OOB_STS);

    if ((status & H2B_INTR_STS) == H2B_INTR_STS)
    {
        printk(KERN_DEBUG "ESPIOOB: irq_sts:0x%x\n", status);
        *(volatile unsigned char*)(ESPI_OOB_STS) = (H2B_PKT_VALID | H2B_INTR_STS);
        spin_lock_irqsave(&espioob_lock, flags);
        espi_oob_intrsts = 1;
        spin_unlock_irqrestore(&espioob_lock, flags);
    }

    wake_up_interruptible(&espioob_intr_wq);

    return IRQ_HANDLED;
}

void espi_put_oob(unsigned char* data, int length)
{
    int i = 0;

    *(volatile unsigned int*)(ESPI_OOB_CTRL) |= (B2H_WPTR_CLR | B2H_RPTR_CLR | H2B_WPTR_CLR | H2B_RPTR_CLR);

    *(volatile unsigned char*)(ESPI_OOB_SIZE) = length;

    for (i = 0; i < length; i++)
        *(volatile unsigned char*)(ESPI_OOB_DATA) = data[i];

    // Generate alert to host
    *(volatile unsigned char*)(ESPI_OOB_CTRL) |= 0x1;
}

long espi_oob_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    OOB_DATA oob_data;
    unsigned long flags;
    int i = 0;

    if (copy_from_user(&oob_data, (void*)arg, sizeof(OOB_DATA)))
        return -EAGAIN;

    printk(KERN_DEBUG "ESPIOOB: cmd:0x%x\n", cmd);
    switch(cmd)
    {
        case CMD_IOCTL_ESPI_PUT_OOB:
            RETURN_IF_ESPI_RESET_HAPPENED();

            if ((*(volatile unsigned char*) (ESPI_OOB_STS) & B2H_BUSY) == B2H_BUSY)
            {
                printk(KERN_DEBUG "B2H Busy\n");
                return -EAGAIN;
            }

            if (oob_data.length > 64)
            {
                printk(KERN_DEBUG "Invalid OOB len\n");
                return -EINVAL;
            }

            spin_lock_irqsave(&espioob_lock, flags);
            espi_oob_intrsts = 0;
            spin_unlock_irqrestore(&espioob_lock, flags);

            espi_put_oob(&oob_data.data[0], oob_data.length);
            break;

        case CMD_IOCTL_ESPI_GET_OOB:
            RETURN_IF_ESPI_RESET_HAPPENED();

            if ((*(volatile unsigned char*) (ESPI_OOB_STS) & B2H_BUSY) == B2H_BUSY)
            {
                printk(KERN_DEBUG "B2H Busy\n");
                return -EAGAIN;
            }

            spin_lock_irqsave(&espioob_lock, flags);
            espi_oob_intrsts = 0;
            spin_unlock_irqrestore(&espioob_lock, flags);

            oob_data.response = *(volatile unsigned char*) (ESPI_OOB_CYC);
            oob_data.length = *(volatile unsigned char*)(ESPI_OOB_SIZE);

            i = 0;
            while ((*(volatile unsigned char*) (ESPI_OOB_STS) & 0x40) == 0x00)
                oob_data.data[i++] = *(volatile unsigned char*)(ESPI_OOB_DATA);

            if (copy_to_user((void*)arg, &oob_data, sizeof(OOB_DATA)))
                return -EAGAIN;

            break;

        case CMD_IOCTL_ESPI_OOB_CLEAR_RESET:
            printk(KERN_DEBUG "ESPIOOB: Clearing eSPI RESET\n");
            spin_lock_irqsave(&espioob_lock, flags);
            espi_oob_reset = 0;
            spin_unlock_irqrestore(&espioob_lock, flags);
            break;

        case CMD_IOCTL_ESPI_OOB_WFI:
            if (oob_data.length == 0)
                wait_event_interruptible(espioob_intr_wq, ((espi_oob_intrsts != 0) || (espi_oob_reset != 0)));
            else if (wait_event_interruptible_timeout(espioob_intr_wq,
                        ((espi_oob_intrsts != 0) || (espi_oob_reset != 0)), msecs_to_jiffies(oob_data.length)) < 1)
                return -ERESTARTSYS;

            break;

        default:
            printk(KERN_ERR "ESPIOOB: Unknown IOCTL\n");
            return -EINVAL;
    };

    return 0;
}
static int espi_oob_open(struct inode * inode, struct file * filp)
{
    return 0;
}


static int espi_oob_close(struct inode * inode, struct file * filp)
{
    return 0;
}


static int pilot_espi_oob_probe(struct platform_device *pdev)
{
	int err = 0, ret = 0;
	struct device_node *node;

	irq_espi_oob = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!irq_espi_oob) {
		pr_err("%s: unable to get the IRQ resource 0\n", __func__);
		return -EINVAL;
	}
	printk("irq_espi_oob=%d\n",irq_espi_oob);
	irq_sio_psr = irq_of_parse_and_map(pdev->dev.of_node, 1);
	if (!irq_sio_psr) {
		pr_err("%s: unable to get the IRQ resource 0\n", __func__);
		return -EINVAL;
	}
	printk("irq_sio_psr=%d\n",irq_sio_psr);

	node = of_find_compatible_node(NULL, NULL, "aspeed,pilot-espi-reg");
	se_sys_clk_va_base = of_iomap(node, 1);
	se_pilot_espi_va_base = of_iomap(node, 3);

	// Check if eSPI strap is set. Otherwise do not insmod the driver
	if ((*(volatile unsigned long*)(se_sys_clk_va_base + 0x0C) & (1 << 14)) != (1 << 14))
	{
		printk("eSPI strap not set\n");
		return -EPFNOSUPPORT;
	}

	printk("eSPI_OOB(BMC) Driver\n");
	printk("Copyright (c) 2015 Avago Technologies\n");

	spin_lock_init(&espioob_lock);
	init_waitqueue_head(&espioob_intr_wq);

	/* ----  Register the character device ------------------- */
	if ((espi_oob_majnum = register_chrdev(ESPI_MAJOR, ESPI_OOB_DEV, &espi_fops)) < 0)
	{
		printk ("failed to register chrdev (err %d)\n", err);
		ret = -ENODEV;
		goto fail;
	}
	espi_registered = 1;

	// Needed only for pre A2 chips
	if ((*(volatile unsigned char*)(se_sys_clk_va_base + 0x50) & 0xF) < 2)
	{
		if (request_irq(irq_sio_psr, oobdummy_irqhandler, IRQF_SHARED, "OOB_Dummy", &oobdummy_irqhandler) < 0)
		{
			printk("ERROR: Failed to request irq %d", irq_sio_psr);
			unregister_chrdev(espi_oob_majnum, ESPI_OOB_DEV);
			ret = -EBUSY;
			goto fail;
		}
	}

	if (request_irq(irq_espi_oob, espi_oob_handler, 0, "espi_oob", &espi_oob_handler) < 0)
	{
		printk("failed to request irq %d", irq_espi_oob);
		ret = -EBUSY;
		unregister_chrdev(espi_oob_majnum, ESPI_OOB_DEV);
		goto fail;
	}
	espi_oob_irq_regd = 1;

	// Enable interrupts
	*(volatile unsigned short*)(ESPI_OOB_CTRL) = H2B_INTR_EN;

	return ret;

fail:
	pilot_espi_oob_remove(pdev);
	return ret;
}
static int pilot_espi_oob_remove(struct platform_device *pdev)
{
    printk("Unregister eSPI_OOB driver\n");
    /* unregister the device */
    if (espi_registered)
    {
        unregister_chrdev(espi_oob_majnum, ESPI_OOB_DEV);
        espi_registered = 0;
    }

    if (espi_oob_irq_regd)
    {
        free_irq(irq_espi_oob, &espi_oob_handler);
        if ((*(volatile unsigned char*)(se_sys_clk_va_base + 0x50) & 0xF) < 2)
            free_irq(irq_sio_psr, &oobdummy_irqhandler);
        espi_oob_irq_regd = 0;
    }

return 0;
}
static const struct of_device_id pilot_espi_oob_of_match[] = {
	        { .compatible = "aspeed,pilot-espi-oob", },
		{ }
};

static struct platform_driver pilot_espi_oob_driver = {
	.driver = {
		.name		= "pilot_espi_oob",
		.of_match_table = pilot_espi_oob_of_match,
	},
	.probe		= pilot_espi_oob_probe,
	.remove		= pilot_espi_oob_remove,
};
module_platform_driver(pilot_espi_oob_driver);
MODULE_DESCRIPTION("eSPI OOB driver module.");
MODULE_LICENSE("GPL");
