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

#ifndef __ESPI_OOB_H__
#define __ESPI_OOB_H__

volatile unsigned char * se_sys_clk_va_base; //SE_SYS_CLK_VA_BASE;
volatile unsigned char * se_pilot_espi_va_base; //SE_PILOT_ESPI_VA_BASE;
unsigned int irq_sio_psr; //IRQ_SIO_PSR;
unsigned int irq_espi_oob; //IRQ_ESPI_OOB;
#define ESPI_CH1_CAP        (se_pilot_espi_va_base + 0x20)
#define ESPI_OOB_BASE	(se_pilot_espi_va_base + 0x800)

#define ESPIOOB_MAGIC   'e'

#define ESPI_OOB_CTRL (ESPI_OOB_BASE + 0x0)
#define ESPI_OOB_STS  (ESPI_OOB_BASE + 0x4)
#define ESPI_OOB_SIZE (ESPI_OOB_BASE + 0x8)
#define ESPI_OOB_DATA (ESPI_OOB_BASE + 0xC)
#define ESPI_OOB_CYC  (ESPI_OOB_BASE + 0x10)
#define ESPI_OOB_TAG  (ESPI_OOB_BASE + 0x14)
#define ESPI_OOB_LEN  (ESPI_OOB_BASE + 0x18)
#define ESPI_OOB_ADD  (ESPI_OOB_BASE + 0x1c)
#define ESPI_OOB_PSR  (ESPI_OOB_BASE + 0x20)

#define B2H_PKT_VLD		(1 << 0)
#define	B2H_INTR_EN		(1 << 1)
#define B2H_WPTR_CLR	(1 << 2)
#define B2H_RPTR_CLR	(1 << 3)
#define H2B_INTR_EN		(1 << 8)
#define H2B_WPTR_CLR	(1 << 9)
#define H2B_RPTR_CLR	(1 << 10)

#define H2B_INTR_STS	(1 << 5)
#define H2B_PKT_VALID	(1 << 7)
#define B2H_BUSY		(1 << 0)

#define TIMEOUT_MS	5000

typedef struct oob_data
{
    unsigned char data[64];
	int length;
	unsigned char response;
} OOB_DATA;

typedef enum  {
    IOCTL_ESPI_PUT_OOB= 1,
    IOCTL_ESPI_GET_OOB,
	IOCTL_ESPI_OOB_CLEAR_RESET,
	IOCTL_ESPI_OOB_WFI,
}ESPI_OOB_IOCTLS;

#define CMD_IOCTL_ESPI_PUT_OOB    	_IOWR(ESPIOOB_MAGIC, IOCTL_ESPI_PUT_OOB, OOB_DATA)
#define CMD_IOCTL_ESPI_GET_OOB     	_IOWR(ESPIOOB_MAGIC, IOCTL_ESPI_GET_OOB, OOB_DATA)
#define CMD_IOCTL_ESPI_OOB_CLEAR_RESET _IOWR(ESPIOOB_MAGIC, IOCTL_ESPI_OOB_CLEAR_RESET, unsigned char)
#define CMD_IOCTL_ESPI_OOB_WFI      _IOWR(ESPIOOB_MAGIC, IOCTL_ESPI_OOB_WFI, OOB_DATA)


#endif // __ESPI_OOB_H__
