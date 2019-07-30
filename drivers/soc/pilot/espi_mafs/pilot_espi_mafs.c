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
#include <linux/kernel.h>	/* printk() 		*/
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

#include "espi_mafs.h"

#define RETURN_IF_ESPI_RESET_HAPPENED(); \
    do { \
        if (espi_mafs_reset)   \
        return -ECONNRESET; \
    } while(0)

static int ESPI_MAJOR = 0;
static const char ESPI_DEVNAME[] = "eSPI_MAFS";
static volatile unsigned char * se_sys_clk_va_base;
static volatile unsigned char * se_pilot_espi_va_base;
static unsigned int irq_espi_fac;

static int pilot_espi_mafs_probe(struct platform_device *pdev);
static int pilot_espi_mafs_remove(struct platform_device *pdev);

static int espi_mafs_open(struct inode * inode, struct file * filp);
static int espi_mafs_close(struct inode * inode, struct file * filp);

static int espi_registered;
static int espi_major_num;
static int espi_mafs_irq_regd;
static spinlock_t espimafs_lock;
static unsigned int espi_mafs_intrsts= 0;
wait_queue_head_t espimafs_intr_wq;

long espi_mafs_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static int espi_mafs_reset = 0;

static struct file_operations espi_fops = {
    owner:		THIS_MODULE,
    open:		espi_mafs_open,
    release:    espi_mafs_close,
    unlocked_ioctl: espi_mafs_ioctl,
};

static irqreturn_t espi_mafs_handler(int irq, void *dev_id)
{
    unsigned long flags;
    unsigned char status;

    status = *(volatile unsigned char*) (ESPI_FLASH_STS);
    printk(KERN_DEBUG "ESPIMAFS: irq_sts:0x%x\n", status);
    if ((status & H2B_INTR_STS) == H2B_INTR_STS)
    {
        *(volatile unsigned char*)(ESPI_FLASH_STS) = (H2B_PKT_VALID | H2B_INTR_STS);
        spin_lock_irqsave(&espimafs_lock, flags);
        espi_mafs_intrsts = 1;
        spin_unlock_irqrestore(&espimafs_lock, flags);
    }

    wake_up_interruptible(&espimafs_intr_wq);

    return IRQ_HANDLED;
}

void espi_write_cmd(unsigned char tag, unsigned char length, unsigned int addr, unsigned char cycle_type, unsigned char* data)
{
    int i = 0;

    *(volatile unsigned int*)(ESPI_FLASH_CTRL) |= (B2H_WPTR_CLR | B2H_RPTR_CLR | H2B_WPTR_CLR | H2B_RPTR_CLR);

    *(volatile unsigned char*)(ESPI_FLASH_SIZE) = 7 + length; // Flash erase or read request packet is 7.
    *(volatile unsigned char*)(ESPI_FLASH_DATA) = cycle_type;

    // Byte 1(Tag + Length[11:8])
    *(volatile unsigned char*)(ESPI_FLASH_DATA) = (unsigned char)(((length >> 8) & 0xf) | ((tag & 0xf) << 4));

    // Byte 2 Length[7:0]
    *(volatile unsigned char*)(ESPI_FLASH_DATA) = (unsigned char)(length & 0xff);

    *(volatile unsigned char*)(ESPI_FLASH_DATA) = ((unsigned char) (addr >> 24) & 0xFF) ;
    *(volatile unsigned char*)(ESPI_FLASH_DATA) = ((unsigned char) (addr >> 16) & 0xFF) ;
    *(volatile unsigned char*)(ESPI_FLASH_DATA) = ((unsigned char) (addr >> 8) & 0xFF) ;
    *(volatile unsigned char*)(ESPI_FLASH_DATA) = ((unsigned char) (addr & 0xFF) ) ;

    for (i = 0; i < length; i++)
        *(volatile unsigned char*)(ESPI_FLASH_DATA) = data[i];

    // Generate alert to host
    *(volatile unsigned char*)(ESPI_FLASH_CTRL) |= 0x1;
}

void espi_generic_cmd(unsigned char tag, unsigned char length, unsigned int addr, unsigned char cycle_type)
{
    *(volatile unsigned int*)(ESPI_FLASH_CTRL) |= (B2H_WPTR_CLR | B2H_RPTR_CLR | H2B_WPTR_CLR | H2B_RPTR_CLR);

    *(volatile unsigned char*)(ESPI_FLASH_SIZE) = 7; // Flash erase or read request packet is 7.
    *(volatile unsigned char*)(ESPI_FLASH_DATA) = cycle_type;

    // Byte 1(Tag + Length[11:8])
    *(volatile unsigned char*)(ESPI_FLASH_DATA) = (unsigned char)(((length >> 8) & 0xf) | ((tag & 0xf) << 4));

    // Byte 2 Length[7:0]
    *(volatile unsigned char*)(ESPI_FLASH_DATA) = (unsigned char)(length & 0xff);

    *(volatile unsigned char*)(ESPI_FLASH_DATA) = ((unsigned char) (addr >> 24) & 0xFF) ;
    *(volatile unsigned char*)(ESPI_FLASH_DATA) = ((unsigned char) (addr >> 16) & 0xFF) ;
    *(volatile unsigned char*)(ESPI_FLASH_DATA) = ((unsigned char) (addr >> 8) & 0xFF) ;
    *(volatile unsigned char*)(ESPI_FLASH_DATA) = ((unsigned char) (addr & 0xFF) ) ;

    // Generate alert to host
    *(volatile unsigned char*)(ESPI_FLASH_CTRL) |= 0x1;
}


long espi_mafs_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    MAFS_DATA mafs_data;
    unsigned long flags;
    unsigned char* data;
    int i = 0;


    if (copy_from_user(&mafs_data, (void*)arg, sizeof(MAFS_DATA)))
        return -EAGAIN;

    printk(KERN_DEBUG "ESPIMAFS: cmd:0x%x len:%d addr:0x%x\n", cmd, mafs_data.length, mafs_data.address);
    switch(cmd)
    {
        case CMD_IOCTL_ESPI_MAFS_ERASE:
            RETURN_IF_ESPI_RESET_HAPPENED();

            if ((*(volatile unsigned char*) (ESPI_FLASH_STS) & B2H_BUSY) == B2H_BUSY)
            {
                printk(KERN_DEBUG "B2H Busy\n");
                return -EAGAIN;
            }

            espi_mafs_intrsts = 0;
            espi_generic_cmd(mafs_data.tag, mafs_data.length, mafs_data.address, MAFS_ERASE);

            if (wait_event_interruptible_timeout(espimafs_intr_wq,
                        ((espi_mafs_intrsts != 0) || (espi_mafs_reset != 0)), msecs_to_jiffies(TIMEOUT_MS)) < 1)
                return -ERESTARTSYS;

            RETURN_IF_ESPI_RESET_HAPPENED();

            mafs_data.response = *(volatile unsigned char*) (ESPI_FLASH_CYC);

            if (copy_to_user((void*)arg, &mafs_data, sizeof(MAFS_DATA)))
                return -EAGAIN;

            break;

        case CMD_IOCTL_ESPI_MAFS_READ:
            RETURN_IF_ESPI_RESET_HAPPENED();

            if ((*(volatile unsigned char*) (ESPI_FLASH_STS) & B2H_BUSY) == B2H_BUSY)
            {
                printk(KERN_DEBUG "B2H Busy\n");
                return -EAGAIN;
            }
            // Supporting only 64 chunks for now
            if (mafs_data.length > 64)
            {
                printk(KERN_DEBUG "Invalid read length\n");
                return -EINVAL;
            }

            espi_mafs_intrsts = 0;
            espi_generic_cmd(mafs_data.tag, mafs_data.length, mafs_data.address, MAFS_READ);

            if (wait_event_interruptible_timeout(espimafs_intr_wq,
                        ((espi_mafs_intrsts != 0) || (espi_mafs_reset != 0)), msecs_to_jiffies(TIMEOUT_MS)) < 1)
                return -ERESTARTSYS;

            RETURN_IF_ESPI_RESET_HAPPENED();

            data = kmalloc(mafs_data.length * sizeof(unsigned char), GFP_KERNEL);
            if (!data)
                return -ENOMEM;

            while ((*(volatile unsigned char*) (ESPI_FLASH_STS) & 0x40) == 0x00)
                data[i++] = *(volatile unsigned char*)(ESPI_FLASH_DATA);

            mafs_data.length = i;
            mafs_data.response = *(volatile unsigned char*) (ESPI_FLASH_CYC);
            printk(KERN_DEBUG "Read 0x%x data\n", i);

            if (copy_to_user((void*)arg, &mafs_data, sizeof(MAFS_DATA)))
            {
                kfree(data);
                return -EAGAIN;
            }

            if (copy_to_user(mafs_data.data, data, mafs_data.length * sizeof(unsigned char)))
            {
                kfree(data);
                return -EFAULT;
            }

            kfree(data);
            break;

        case CMD_IOCTL_ESPI_MAFS_WRITE:
            RETURN_IF_ESPI_RESET_HAPPENED();

            if ((*(volatile unsigned char*) (ESPI_FLASH_STS) & B2H_BUSY) == B2H_BUSY)
            {
                printk(KERN_DEBUG "B2H Busy\n");
                return -EAGAIN;
            }

            // Supporting only 64 chunks for now
            if (mafs_data.length > 64)
            {
                printk(KERN_DEBUG "Invalid write length\n");
                return -EINVAL;
            }

            data = kmalloc(mafs_data.length * sizeof(unsigned char), GFP_KERNEL);
            if (!data)
                return -ENOMEM;

            if (copy_from_user(data, mafs_data.data, mafs_data.length * sizeof(unsigned char)))
            {
                kfree(data);
                return -EFAULT;
            }

            espi_mafs_intrsts = 0;
            espi_write_cmd(mafs_data.tag, mafs_data.length, mafs_data.address, MAFS_WRITE, data);

            if (wait_event_interruptible_timeout(espimafs_intr_wq,
                        ((espi_mafs_intrsts != 0) || (espi_mafs_reset != 0)), msecs_to_jiffies(TIMEOUT_MS)) < 1)
            {
                kfree(data);
                return -ERESTARTSYS;
            }

            RETURN_IF_ESPI_RESET_HAPPENED();

            mafs_data.response = *(volatile unsigned char*) (ESPI_FLASH_CYC);
            if (copy_to_user((void*)arg, &mafs_data, sizeof(MAFS_DATA)))
            {
                kfree(data);
                return -EAGAIN;
            }
            kfree(data);

            break;

        case CMD_IOCTL_ESPI_MAFS_CLEAR_RESET:
            printk(KERN_DEBUG "ESPIMAFS: Clearing eSPI RESET\n");
            spin_lock_irqsave(&espimafs_lock, flags);
            espi_mafs_reset = 0;
            spin_unlock_irqrestore(&espimafs_lock, flags);
            break;

        default:
            printk(KERN_ERR "ESPIMAFS: Unknown IOCTL\n");
            return -EINVAL;
    };

    return 0;
}

static int espi_mafs_open(struct inode * inode, struct file * filp)
{
    return 0;
}

static int espi_mafs_close(struct inode * inode, struct file * filp)
{
    return 0;
}

static int pilot_espi_mafs_probe(struct platform_device *pdev)
{
	int err = 0, ret = 0;
	struct device_node *node;

	irq_espi_fac = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!irq_espi_fac) {
		pr_err("%s: unable to get the IRQ resource 0\n", __func__);
		return -EINVAL;
	}


	node = of_find_compatible_node(NULL, NULL, "aspeed,pilot-espi-reg");
	se_sys_clk_va_base = of_iomap(node, 1);
	se_pilot_espi_va_base = of_iomap(node, 3);

	// Check if eSPI strap is set. Otherwise do not insmod the driver
	if ((*(volatile unsigned long*)(se_sys_clk_va_base + 0x0C) & (1 << 14)) != (1 << 14))
	{
		printk("eSPI strap not set\n");
		return -EPFNOSUPPORT;
	}

	if ((*(volatile unsigned long*)(ESPI_CH3_CAP_1) & (1 << 11)) == (1 << 11))
	{
		printk("eSPI not in MAFS mode\n");
		return -EPFNOSUPPORT;
	}

	printk("eSPI MAFS Driver\n");
	printk("Copyright (c) 2015 Avago Technologies\n");

	spin_lock_init(&espimafs_lock);
	init_waitqueue_head(&espimafs_intr_wq);

	/* ----  Register the character device ------------------- */
	if ((espi_major_num = register_chrdev(ESPI_MAJOR, ESPI_DEVNAME, &espi_fops)) < 0)
	{
		printk ("failed to register chrdev (err %d)\n", err);
		ret = -ENODEV;
		goto fail;
	}
	espi_registered = 1;

	if (request_irq(irq_espi_fac, espi_mafs_handler, 0, "espi_mafs", &espi_mafs_handler) < 0)
	{
		printk("failed to request irq %d", irq_espi_fac);
		ret = -EBUSY;
		goto fail;
	}
	espi_mafs_irq_regd = 1;

	// Enable interrupts
	*(volatile unsigned int*)(ESPI_FLASH_CTRL) = H2B_INTR_EN;

	return ret;
fail:
	pilot_espi_mafs_remove(pdev);
	return ret;
}
static int pilot_espi_mafs_remove(struct platform_device *pdev)
{
	printk("Unregister eSPI_MAFS driver\n");
	/* unregister the device */
	if (espi_registered)
	{
		unregister_chrdev(espi_major_num, ESPI_DEVNAME);
		espi_registered = 0;
	}

	if (espi_mafs_irq_regd)
	{
		free_irq(irq_espi_fac, &espi_mafs_handler);
		espi_mafs_irq_regd = 0;
	}
	return 0;
}
static const struct of_device_id pilot_espi_mafs_of_match[] = {
	        { .compatible = "aspeed,pilot-espi-mafs", },
		{ }
};

static struct platform_driver pilot_espi_mafs_driver = {
	.driver = {
		.name		= "pilot_espi_mafs",
		.of_match_table = pilot_espi_mafs_of_match,
	},
	.probe		= pilot_espi_mafs_probe,
	.remove		= pilot_espi_mafs_remove,
};
module_platform_driver(pilot_espi_mafs_driver);
MODULE_DESCRIPTION("eSPI driver module.");
MODULE_LICENSE("GPL");
