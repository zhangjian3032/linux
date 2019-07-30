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

#ifndef __ESPI_H__
#define __ESPI_H__

extern volatile unsigned char * SE_SYS_CLK_VA_BASE;
extern volatile unsigned char * SE_PILOT_SPEC_VA_BASE;
extern volatile unsigned char * SE_SCRATCH_128_VA_BASE;
extern volatile unsigned char * SE_PILOT_ESPI_VA_BASE;

extern unsigned int IRQ_SIO_PSR;
extern unsigned int IRQ_ESPI_VWIRE;

#define ESPIVW_MAGIC   'e'

//VW defines
#define ESPI_VW_HW_ACK_CTL  (SE_PILOT_SPEC_VA_BASE + 0x25)
#define ESPI_HW_SLV_BLS     (1 << 0)
#define ESPI_HW_SLV_BLD     (1 << 1)
#define ESPI_HW_SUSACK      (1 << 2)
#define ESPI_HW_OOBACK      (1 << 3)
#define ESPI_HW_HSTACK      (1 << 4)

#define ESPI_CH1_CAP        (SE_PILOT_ESPI_VA_BASE + 0x20)


#define ESPI_VW_BASE        (SE_PILOT_ESPI_VA_BASE + 0xC00)
#define ESPI_VW_CTL         (ESPI_VW_BASE + 0xC0)

#define ESPI_VW_CH_EN       (1 << 0)

#define BIT0        (0)
#define BIT1        (1)
#define BIT2        (2)
#define BIT3        (3)

// To be used for SET_VWIRE macros only as bit positions
#define BIT_SBLD    BIT0
#define BIT_SBLS    BIT3
#define BIT_SUSACK  BIT0
#define BIT_OOBACK  BIT0
#define BIT_HST_ACK BIT3

#define ESPI_VW_SUS_WARN        (1 << 0)
#define ESPI_VW_OOB_RST_WARN    (1 << 2)
#define ESPI_VW_HST_RST_WARN    (1 << 0)

#define ESPI_VW_SUS_PWRDN_ACK   (1 << 1)
#define ESPI_VW_SLAVE_BOOT_LOAD_DONE    (1 << 0)
#define ESPI_VW_SLAVE_BOOT_LOAD_STATUS  (1 << 3)

#define INDEX_03h   0x3
#define INDEX_04h   0x4
#define INDEX_05h   0x5
#define INDEX_06h   0x6
#define INDEX_07h   0x7
#define INDEX_40h   0x40
#define INDEX_41h   0x41

#define VW_VALID    (1 << 7)
#define EN_VW_INTR  (1 << 0)

typedef struct virtual_wires
{
    unsigned char index;
    unsigned int value;
} VW_DATA;

#define ESPI_NONE       (1 << 0)
#define ESPI_RESET      (1 << 1)
#define ESPI_PLTRST     (1 << 2)
#define ESPI_INBAND_RST (1 << 3)

typedef enum  {
    IOCTL_ESPIVW_GET_HOST2BMC_STATUS= 1,
    IOCTL_ESPIVW_SET_BMC2HOST_VWIRE,
    IOCTL_ESPIVW_ENABLE_INTERRUPT,
    IOCTL_ESPIVW_DISABLE_INTERRUPT,
    IOCTL_ESPIVW_WAIT_FOR_INTERRUPT,
    IOCTL_ESPIVW_CLEAR_RESET
}ESPIVW_IOCTLS;

#define CMD_IOCTL_ESPIVW_GET_HOST2BMC_STATUS    _IOWR(ESPIVW_MAGIC, IOCTL_ESPIVW_GET_HOST2BMC_STATUS, VW_DATA)
#define CMD_IOCTL_ESPIVW_SET_BMC2HOST_VWIRE     _IOWR(ESPIVW_MAGIC, IOCTL_ESPIVW_SET_BMC2HOST_VWIRE, VW_DATA)
#define CMD_IOCTL_ESPIVW_ENABLE_INTERRUPT       _IOWR(ESPIVW_MAGIC, IOCTL_ESPIVW_ENABLE_INTERRUPT, unsigned char)
#define CMD_IOCTL_ESPIVW_DISABLE_INTERRUPT      _IOWR(ESPIVW_MAGIC, IOCTL_ESPIVW_DISABLE_INTERRUPT, unsigned char)
#define CMD_IOCTL_ESPIVW_WAIT_FOR_INTERRUPT     _IOWR(ESPIVW_MAGIC, IOCTL_ESPIVW_WAIT_FOR_INTERRUPT, VW_DATA)
#define CMD_IOCTL_ESPIVW_CLEAR_RESET            _IOWR(ESPIVW_MAGIC, IOCTL_ESPIVW_CLEAR_RESET, unsigned char)


#endif // __ESPI_H__
