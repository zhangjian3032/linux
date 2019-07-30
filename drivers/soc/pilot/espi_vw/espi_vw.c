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

#include "espi.h"

static struct Host2bmc_vw_ixs
{
    const unsigned char index;
    unsigned char prev_value;
    unsigned char curr_value;
    int espi_reset_type;
} host2bmc_vw_ixs[] = {
    {0x02, 0, 0, ESPI_NONE},
    {0x03, 0, 0, ESPI_RESET},
    {0x07, 0, 0, ESPI_PLTRST | ESPI_RESET},
    {0x41, 0, 0, ESPI_RESET},
    {0x42, 0, 0, ESPI_NONE},
    {0x43, 0, 0, ESPI_RESET},
    {0x44, 0, 0, ESPI_RESET},
    {0x47, 0, 0, ESPI_PLTRST | ESPI_RESET}
};

// Supported VW interrupt indexes from BMC to Host
static unsigned char bmc2host_vw_ixs[] = {0x4, 0x5, 0x6, 0x40, 0x45, 0x46};

static spinlock_t espivw_lock;
static unsigned int espi_vw_intr_sts = 0;
wait_queue_head_t espivw_intr_waitq;
static int espi_vw_irq_requested = 0;
static int espi_reset_irq_requested = 0;
static unsigned char first_vw_triggered = 0;
static int espi_reset = 0;
static int espi_vw_file_added = 0;

static irqreturn_t espi_vw_handler(int irq, void *dev_id);

#define MAX_RESET_LISTENERS 1
int  (*espiResetListeners[MAX_RESET_LISTENERS])(int);
void handle_espi_resets(int reset_type);

#define ESPI_INBAND_EN  (1 << 7)
#define ESPI_INBAND_STS (1 << 0)
#define ESPI_RST_INTR_EN (1 << 2)

#define H2B_ARRAY_SIZE  (sizeof(host2bmc_vw_ixs)/(sizeof(struct Host2bmc_vw_ixs)))
#define B2H_ARRAY_SIZE  (sizeof(bmc2host_vw_ixs)/(1*sizeof(unsigned char)))
#define SET_VWIRE(bit, value)    ((value << bit) | (1 << (bit + 4)))

#define RETURN_IF_ESPI_RESET_HAPPENED(); \
    do { \
        if (espi_reset)   \
        return -ECONNRESET; \
    } while(0)


static ssize_t sysfs_show_vwires(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
    char *start_buf = buf;
    volatile int i = 0, j = 0;
    const char* header = "Index:prev_val:cur_val\n\r";

    while (header[i] != '\0')
        *buf++ = header[i++];

    for (i = 0; i < H2B_ARRAY_SIZE; i++)
    {
        buf += sprintf(buf, "[%02x]:", host2bmc_vw_ixs[i].index);

        for (j = 7; j >= 0; j--)
        {
            *buf++ = ((host2bmc_vw_ixs[i].prev_value & (1 << j)) == (1 << j)) ? '1' : '0';
            if (j == 4)
                *buf++ = '_';
        }
        *buf++ = ':';

        for (j = 7; j >= 0; j--)
        {
            *buf++ = ((host2bmc_vw_ixs[i].curr_value & (1 << j)) == (1 << j)) ? '1' : '0';
            if (j == 4)
                *buf++ = '_';
        }
        *buf++ = '\n';
        *buf++ = '\r';
    }
    *buf++ = '\0';
    return (buf - start_buf);

}
static DEVICE_ATTR(vwires, 0444, sysfs_show_vwires, NULL);

static int notifyResetListeners(int reset_type)
{
    int err, i;

    for(i=0; i < MAX_RESET_LISTENERS; i++)
    {
        if(NULL != espiResetListeners[i])
            err = (*espiResetListeners[i])(reset_type);
    }
    return 0;
}

int registerESPIReset(int (*CBFunc)(int))
{
    int i ;
    for(i = 0; i < MAX_RESET_LISTENERS; i++)
    {
        if (espiResetListeners[i] == NULL)
        {
            espiResetListeners[i] = CBFunc;
            return 0;
        }
    }

    printk("ERROR: Cannot Install eSPI Reset Handler\n");
    return 1;
}

int deregisterESPIReset(int (*CBFunc)(int))
{
    int i;
    for(i = 0; i < MAX_RESET_LISTENERS; i++)
    {
        if(CBFunc == espiResetListeners[i])
        {
            espiResetListeners[i]=NULL;
            return 0;
        }
    }

    printk("ERROR: Cannot uninstall eSPI Reset Handler\n");
    return 1;
}



static int auto_ack_enabled(unsigned char index, unsigned char value)
{
    if (index == INDEX_05h)
    {
        if (((value & (1 << (BIT_SBLS + 4))) == (1 << (BIT_SBLS + 4))) &&
            ((*(volatile unsigned char*)(ESPI_VW_HW_ACK_CTL) & ESPI_HW_SLV_BLS) == 0))
        return 1;

        if (((value & (1 << (BIT_SBLD + 4))) == (1 << (BIT_SBLD + 4))) &&
            ((*(volatile unsigned char*)(ESPI_VW_HW_ACK_CTL) & ESPI_HW_SLV_BLD) == 0))
        return 1;
    }

    if ((index == INDEX_40h) &&
        ((value & (1 << (BIT_SUSACK + 4))) == (1 << (BIT_SUSACK + 4))) &&
        ((*(volatile unsigned char*)(ESPI_VW_HW_ACK_CTL) & ESPI_HW_SUSACK) == 0))
        return 1;

    if ((index == INDEX_04h) &&
        ((value & (1 << (BIT_OOBACK + 4))) == (1 << (BIT_OOBACK + 4))) &&
        ((*(volatile unsigned char*)(ESPI_VW_HW_ACK_CTL) & ESPI_HW_OOBACK) == 0))
        return 1;

    if ((index == INDEX_06h) &&
        ((value & (1 << (BIT_HST_ACK + 4))) == (1 << (BIT_HST_ACK + 4))) &&
        ((*(volatile unsigned char*)(ESPI_VW_HW_ACK_CTL) & ESPI_HW_HSTACK) == 0))
        return 1;

    return 0;
}

static int send_vwire(unsigned char index, unsigned char value)
{
    int i = 0;

    if ((*(volatile unsigned long*)(ESPI_CH1_CAP) & ESPI_VW_CH_EN) != ESPI_VW_CH_EN)
    {
        printk(KERN_DEBUG "ESPIVW: cannot send vwire(0x%x,0x%x), channel disabled\n", index, value);
        return -EAGAIN;
    }

    // Make sure that the chip revision is A2
    if (((*(volatile unsigned char*)(SE_SYS_CLK_VA_BASE + 0x50) & 0xF) >= 2) &&
            auto_ack_enabled(index, value))
    {
        printk(KERN_DEBUG "ESPIVW: HW ACK enabled, returning...\n");
        return 0;
    }

    if (!first_vw_triggered)
    {
        // Exception to this condition is ESPI_VW_SLAVE_BOOT_LOAD_DONE and
        // ESPI_VW_SLAVE_BOOT_LOAD_STATUS vwires
        if (!((index == INDEX_05h) &&
             (((value & (ESPI_VW_SLAVE_BOOT_LOAD_DONE << 4)) == (ESPI_VW_SLAVE_BOOT_LOAD_DONE << 4)) ||
             ((value & (ESPI_VW_SLAVE_BOOT_LOAD_STATUS << 4)) == (ESPI_VW_SLAVE_BOOT_LOAD_STATUS << 4)))))
        {
            printk(KERN_DEBUG "ESPIVW: cannot send vwire(0x%x,0x%x), host not up\n", index, value);
            return -EAGAIN;
        }
    }

    for (i = 0; i < B2H_ARRAY_SIZE; i++)
    {
        if (bmc2host_vw_ixs[i] == index)
        {
            *(volatile unsigned char*)(ESPI_VW_BASE + index) = value;
            printk(KERN_DEBUG "ESPIVW: put_vwire index:0x%x value:0x%x\n", index, value);
            break;
        }
    }

    if (i == B2H_ARRAY_SIZE)
    {
        printk(KERN_ERR "ESPIVW: Unsupported B2H vwire index :%x\n",index);
        return -EINVAL;
    }

    return 0;
}

static void send_acks_if_needed(void)
{
    int i = 0;
    unsigned char value;

    for (i = 0; i < H2B_ARRAY_SIZE; i++)
    {
        // If a SUS_WARN# is triggered from the host then generate
        // a SUS_ACK# from BMC. The value should be 0 or 1 based on how the host
        // triggered the interrupt. So take the value of SUS_WARN# and set it as a
        // value to SUS_ACK
        if ((host2bmc_vw_ixs[i].index == INDEX_41h) && ((host2bmc_vw_ixs[i].curr_value & 0x10) == 0x10))
        {
            value = host2bmc_vw_ixs[i].curr_value & 0x1;
            if (send_vwire(INDEX_40h, SET_VWIRE(BIT_SUSACK, value)) == 0)
            {
                printk(KERN_DEBUG "ESPIVW: SUS_ACK(%x)\n", host2bmc_vw_ixs[i].curr_value & 0x1);

                // clear the valid bits for this interrupt and make it as previous
                host2bmc_vw_ixs[i].prev_value = host2bmc_vw_ixs[i].curr_value;
                host2bmc_vw_ixs[i].curr_value &= 0xEF;
            }
        }

        // If a OOB_RST_WARN is triggered from the host then generate
        // a OOB_RST_ACK from BMC. The value should be 0 or 1 based on how the host
        // triggered the interrupt. So take the value of OOB_RST_WARN and set it as a
        // value to OOB_RST_ACK
        if ((host2bmc_vw_ixs[i].index == INDEX_03h) && ((host2bmc_vw_ixs[i].curr_value & 0x40) == 0x40))
        {
            value = (host2bmc_vw_ixs[i].curr_value >> 2) & 0x1;
            if (send_vwire(INDEX_04h, SET_VWIRE(BIT_OOBACK, value)) == 0)
            {
                printk(KERN_DEBUG "ESPIVW: OOB_ACK(%x)\n", value);

                // clear the valid bit for this interrupt and make it as previous
                host2bmc_vw_ixs[i].prev_value = host2bmc_vw_ixs[i].curr_value;
                host2bmc_vw_ixs[i].curr_value &= 0xBF;
            }
        }

        // If a HST_RST_WARN is triggered from the host then generate
        // a HST_RST_ACK from BMC. The value should be 0 or 1 based on how the host
        // triggered the interrupt. So take the value of HST_RST_WARN and set it as a
        // value to HST_RST_ACK
        if ((host2bmc_vw_ixs[i].index == INDEX_07h) && ((host2bmc_vw_ixs[i].curr_value & 0x10) == 0x10))
        {
            value = host2bmc_vw_ixs[i].curr_value & 0x1;
            if (send_vwire(INDEX_06h, SET_VWIRE(BIT_HST_ACK, value)) == 0)
            {
                printk(KERN_DEBUG "ESPIVW: HST_RST_ACK(%x)\n", value);

                // clear the valid bit for this interrupt and make it as previous
                host2bmc_vw_ixs[i].prev_value = host2bmc_vw_ixs[i].curr_value;
                host2bmc_vw_ixs[i].curr_value &= 0xEF;
            }
        }
    }
}

static void check_vwires(void)
{
    int i = 0, j = 0;
    volatile unsigned char vw_status = 0;
    unsigned char temp = 0;

    for (i = 0; i < H2B_ARRAY_SIZE; i++)
    {
        vw_status = *(volatile unsigned char*)(ESPI_VW_BASE + host2bmc_vw_ixs[i].index);

        // Check if valid is set for any of the bits
        if ((vw_status & VW_VALID) == VW_VALID)
        {
            first_vw_triggered = 1;
            host2bmc_vw_ixs[i].prev_value = host2bmc_vw_ixs[i].curr_value;
            for (j = 0; j < 4; j++)
            {
                if ((host2bmc_vw_ixs[i].prev_value & (1 << j)) != (vw_status & (1 << j)))
                {
                    temp |= (1 << j);
                    espi_vw_intr_sts |= (1 << i);

                    // Check if the current vwire was triggered due to PLTRST.
                    if ((host2bmc_vw_ixs[i].index == INDEX_03h) && (j == 1))
                    {
                        // When PLTRST is asserted some of the vwires are cleared
                        if ((vw_status & 0x2) == 0x0)
                        {
                            printk("ESPI PLTRST# 0\n");
                            handle_espi_resets(ESPI_PLTRST);
                        }
                        // When PLTRST is deasserted notify all the listeners to re-initialize their
                        // settings
                        if ((vw_status & 0x2) == 0x2)
                        {
                            printk("ESPI PLTRST# 1\n");
                            notifyResetListeners(0);
                        }
                    }
                    wake_up_interruptible(&espivw_intr_waitq);
                }
            }

            host2bmc_vw_ixs[i].curr_value &= ~temp;
            host2bmc_vw_ixs[i].curr_value |= (((temp & 0xF) << 4) | (vw_status & 0xF));
            printk(KERN_DEBUG "ESPIVW: vwire_valid Index(%x) hwsts:0x%x swsts:0x%x\n", 
                    host2bmc_vw_ixs[i].index, vw_status, host2bmc_vw_ixs[i].curr_value);
        }
        temp = 0;
    }
    
    send_acks_if_needed();
}

static irqreturn_t espi_reset_handler(int irq, void *dev_id)
{
    unsigned char sio_intr;
    unsigned long flags = 0;

    sio_intr = *(unsigned char*)(SE_PILOT_SPEC_VA_BASE + 0x23);
    printk(KERN_DEBUG "espi_reset_handler:%d sio_intr=%x\n",irq,sio_intr);
    /* If interrupte not generated by RESET edge, return */
    if (sio_intr & 0x02)
    {
        printk(KERN_DEBUG "eSPI Reset:0x%x\n", sio_intr);
        /* write back  to clear reset interrupt */
        sio_intr &= 0xf7; //do not touch lock bit
        *(unsigned char *)(SE_PILOT_SPEC_VA_BASE + 0x23) = sio_intr;

        spin_lock_irqsave(&espivw_lock, flags);
        handle_espi_resets(ESPI_RESET);
        spin_unlock_irqrestore(&espivw_lock, flags);
    }

    if ((*(unsigned char *)(SE_PILOT_SPEC_VA_BASE + 0x24) & ESPI_INBAND_STS) == ESPI_INBAND_STS)
    {
        printk(KERN_INFO "eSPI In-band reset\n");
        *(unsigned char *)(SE_PILOT_SPEC_VA_BASE + 0x24) |= ESPI_INBAND_STS;
    }

    return (IRQ_HANDLED);
}


static irqreturn_t espi_vw_handler(int irq, void *dev_id)
{
    unsigned long flags;

    printk(KERN_DEBUG "espi_vw_handler: irq%d\n",irq);
    spin_lock_irqsave(&espivw_lock, flags);

    // If this is the first time vwire is triggered then generate ESPI_VW_SLAVE_BOOT_LOAD_DONE and
    // ESPI_VW_SLAVE_BOOT_LOAD_STATUS. This is to handle the case where the channel was not
    // enabled at the time of driver init or after espi reset
    if (!first_vw_triggered)
    {
        if (send_vwire(INDEX_05h, (SET_VWIRE(BIT_SBLD, 1) | SET_VWIRE(BIT_SBLS, 1))) == 0)
            printk(KERN_DEBUG "ESPIVW: SLBLD/SBLS\n");
    }

    check_vwires();

    spin_unlock_irqrestore(&espivw_lock, flags);

    return IRQ_HANDLED;
}

long espi_vw_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    VW_DATA vw_data;
    unsigned long flags;
    int i = 0;

    printk(KERN_DEBUG "ESPIVW: IOCTL:0x%x\n", cmd & 0xFF);
    switch(cmd)
    {
        case CMD_IOCTL_ESPIVW_GET_HOST2BMC_STATUS:
            RETURN_IF_ESPI_RESET_HAPPENED();

            if (copy_from_user(&vw_data, (void*)arg, sizeof(VW_DATA)))
                return -EAGAIN;

            spin_lock_irqsave(&espivw_lock, flags);

            for (i = 0; i < H2B_ARRAY_SIZE; i++)
            {
                if (host2bmc_vw_ixs[i].index == vw_data.index)
                {
                    vw_data.value = host2bmc_vw_ixs[i].curr_value;

                    // Do not clear the valid bits for the vwires for which the the driver
                    // has to automatically generate ACKs. These valids will be cleared
                    // automatically once the ACK is generated.
                    if (vw_data.index == INDEX_41h)
                        host2bmc_vw_ixs[i].curr_value &= 0x1F;
                    else if (vw_data.index == INDEX_07h)
                        host2bmc_vw_ixs[i].curr_value &= 0x1F;
                    else if (vw_data.index == INDEX_03h)
                        host2bmc_vw_ixs[i].curr_value &= 0x4F;
                    else
                        host2bmc_vw_ixs[i].curr_value &= 0x0F;
                    espi_vw_intr_sts &= ~(1 << i);
                    break;
                }
            }
            spin_unlock_irqrestore(&espivw_lock, flags);

            if (i == H2B_ARRAY_SIZE)
            {
                printk(KERN_INFO "ESPIVW: Unsupported H2B vwire index\n");
                return -EINVAL;
            }

            if (copy_to_user((void*)arg, &vw_data, sizeof(VW_DATA)))
                return -EAGAIN;

            break;

        case CMD_IOCTL_ESPIVW_SET_BMC2HOST_VWIRE:
            RETURN_IF_ESPI_RESET_HAPPENED();

            if (copy_from_user(&vw_data, (void*)arg, sizeof(VW_DATA)))
                return -EAGAIN;

            return send_vwire(vw_data.index, vw_data.value);

        case CMD_IOCTL_ESPIVW_ENABLE_INTERRUPT:
            RETURN_IF_ESPI_RESET_HAPPENED();

            *(volatile unsigned char*)(ESPI_VW_CTL) |= EN_VW_INTR;
            break;

        case CMD_IOCTL_ESPIVW_DISABLE_INTERRUPT:
            RETURN_IF_ESPI_RESET_HAPPENED();

            *(volatile unsigned long*)(ESPI_VW_CTL) &= ~EN_VW_INTR;
            break;

        case CMD_IOCTL_ESPIVW_WAIT_FOR_INTERRUPT:
            RETURN_IF_ESPI_RESET_HAPPENED();

            if (copy_from_user(&vw_data, (void*)arg, sizeof(VW_DATA)))
                return -EAGAIN;

            if (vw_data.value == 0)
                wait_event_interruptible(espivw_intr_waitq, ((espi_vw_intr_sts != 0) || (espi_reset != 0)));
            else
            {
                if (wait_event_interruptible_timeout(espivw_intr_waitq,
                            ((espi_vw_intr_sts != 0) || (espi_reset != 0)), msecs_to_jiffies(vw_data.value)) < 1)
                    return -ERESTARTSYS;
            }

            if(espi_reset)
                return -ECONNRESET;

            break;

        case CMD_IOCTL_ESPIVW_CLEAR_RESET:
            printk(KERN_DEBUG "Clearing eSPI RESET\n");
            spin_lock_irqsave(&espivw_lock, flags);
            espi_reset = 0;
            spin_unlock_irqrestore(&espivw_lock, flags);
            break;

        default:
            printk(KERN_ERR "ESPIVW: Unknown IOCTL\n");
            return -EINVAL;
    };

    return 0;
}

int espivw_init(struct device* espi_device)
{
    int i = 0;
    unsigned char intr_s;

    init_waitqueue_head(&espivw_intr_waitq);
    spin_lock_init(&espivw_lock);

    espi_reset = 0;

    device_create_file(espi_device, &dev_attr_vwires);
    espi_vw_file_added = 1;

    if (send_vwire(INDEX_05h, (SET_VWIRE(BIT_SBLD, 1) | SET_VWIRE(BIT_SBLS, 1))) == 0)
        printk(KERN_DEBUG "ESPIVW: SLBLD/SBLS\n");

    // If BMC is coming out of soft reset
    if (*(volatile unsigned char*)(SE_SCRATCH_128_VA_BASE + 0x7C) == 1)
    {
        printk(KERN_DEBUG "ESPIVW: Soft reset\n");

        // When the system is coming up for the first time the driver only report the VW values
        // and will not report about the valid bits since it does not know which VWIRE changed.
        // This value will be used to compare when subsequent interrupts happen and report
        // changes in a particular vwire.
        // If a SUSACK has to be sent it will be done by just looking at the valid and values.
        // i.e., see if bit 8, bit 0 & bit 1 are set in a VW Index_41(best assumption).
        // Not taking spin lock since IRQ is not enabled at this point anyways.
        for (i = 0;i < H2B_ARRAY_SIZE; i++)
        {
            host2bmc_vw_ixs[i].prev_value = 0;
            host2bmc_vw_ixs[i].curr_value = *(volatile unsigned char*)(ESPI_VW_BASE + host2bmc_vw_ixs[i].index);

            // If valid is set then clear the valid bits and just populate the current values
            if ((host2bmc_vw_ixs[i].curr_value & 0xF0) != 0)
            {
                if (host2bmc_vw_ixs[i].index == INDEX_41h)
                {
                    // See if Index_41 valid bit is set and BIT0 is set. If so set valids for
                    // SUS_WARN# (best assumption) to aid SUSACK generation.
                    // And clear other valid bits
                    if ((host2bmc_vw_ixs[i].curr_value & ESPI_VW_SUS_WARN) == ESPI_VW_SUS_WARN)
                        host2bmc_vw_ixs[i].curr_value |= (ESPI_VW_SUS_WARN << 4);

                    host2bmc_vw_ixs[i].curr_value &= 0x1F;
                }
                else if (host2bmc_vw_ixs[i].index == INDEX_03h)
                {
                    // See if Index_03 valid bit is set and BIT2 is set. If so set valids for
                    // OOB_RST_ACK(best assumption) to aid OOB_RST_ACK generation.
                    // And clear other valid bits
                    if ((host2bmc_vw_ixs[i].curr_value & ESPI_VW_OOB_RST_WARN) == ESPI_VW_OOB_RST_WARN)
                        host2bmc_vw_ixs[i].curr_value |= (ESPI_VW_OOB_RST_WARN << 4);

                    host2bmc_vw_ixs[i].curr_value &= 0x4F;
                }
                else if (host2bmc_vw_ixs[i].index == INDEX_07h)
                {
                    // See if Index_07 valid bit is set and BIT2 is set. If so set valids for
                    // OOB_RST_ACK(best assumption) to aid OOB_RST_ACK generation.
                    // And clear other valid bits
                    if ((host2bmc_vw_ixs[i].curr_value & ESPI_VW_HST_RST_WARN) == ESPI_VW_HST_RST_WARN)
                        host2bmc_vw_ixs[i].curr_value |= (ESPI_VW_HST_RST_WARN<< 4);

                    host2bmc_vw_ixs[i].curr_value &= 0x1F;
                }
                else
                    host2bmc_vw_ixs[i].curr_value = 0x0F;
            }

            // If the value is 1 for any of the vwire then it means that the host has triggered
            // atleast once vwire
            if (host2bmc_vw_ixs[i].index == 0x02 && ((host2bmc_vw_ixs[i].curr_value & 0x4) == 0x4))
            {
                printk(KERN_DEBUG "Host is up\n");
                first_vw_triggered = 1;
            }
        }

        send_acks_if_needed();
    }
    else
        check_vwires();

    if (request_irq(IRQ_ESPI_VWIRE, espi_vw_handler, IRQF_SHARED, "espi_vw", &espi_vw_handler) < 0)
    {
        printk("failed to request irq %d", IRQ_ESPI_VWIRE);
        return -EBUSY;
    }
    espi_vw_irq_requested = 1;

    if (request_irq(IRQ_SIO_PSR, espi_reset_handler, IRQF_SHARED, "espi_reset", &espi_reset_handler) < 0)
    {
        printk("failed to request irq %d", IRQ_SIO_PSR);
        return -EBUSY;
    }
    espi_reset_irq_requested = 1;

    /* Enable eSPI Reset Interrupt */
    intr_s = *(unsigned char *)(SE_PILOT_SPEC_VA_BASE + 0x23);
    intr_s = intr_s | ESPI_RST_INTR_EN;
    intr_s = intr_s & 0xf7; //do not touch lock bit
    *(unsigned char *)(SE_PILOT_SPEC_VA_BASE + 0x23) = intr_s;

    /* Enable In-band reset . Do RMW so as to not overwrite other bits in this register */
    *(unsigned char *)(SE_PILOT_SPEC_VA_BASE + 0x25) |= (ESPI_INBAND_EN);

    *(volatile unsigned char*)(ESPI_VW_CTL) |= EN_VW_INTR;

    return 0;
}

void cleanup_espivw_module(struct device* espi_device)
{
    /* free irq */
    if (espi_vw_irq_requested)
    {
        free_irq(IRQ_ESPI_VWIRE, &espi_vw_handler);
        espi_vw_irq_requested = 0;
    }

    if (espi_reset_irq_requested)
    {
        free_irq(IRQ_SIO_PSR, &espi_reset_handler);
        espi_reset_irq_requested = 0;
    }

    if (espi_vw_file_added)
    {
        device_remove_file(espi_device, &dev_attr_vwires);
        espi_vw_file_added = 0;
    }
}

void handle_espi_resets(int reset_type)
{
    int i = 0;

    for (i = 0;i < H2B_ARRAY_SIZE; i++)
    {
        if ((reset_type & host2bmc_vw_ixs[i].espi_reset_type) == reset_type)
        {
            host2bmc_vw_ixs[i].prev_value = 0;
            host2bmc_vw_ixs[i].curr_value = 0;
            espi_vw_intr_sts &= ~(1 << i);
        }
    }

    if (reset_type == ESPI_RESET)
    {
        notifyResetListeners(1);
        first_vw_triggered = 0;
        espi_reset = 1;
        wake_up_interruptible(&espivw_intr_waitq);
    }
}

EXPORT_SYMBOL(registerESPIReset);
EXPORT_SYMBOL(deregisterESPIReset);
