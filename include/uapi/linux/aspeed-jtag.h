/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021 ASPEED Technology Inc.
 */

#ifndef _UAPI_LINUX_ASPEED_JTAG_H
#define _UAPI_LINUX_ASPEED_JTAG_H

#include <linux/ioctl.h>
#include <linux/types.h>

enum jtag_xfer_mode {
	HW_MODE = 0,
	SW_MODE,
};

enum jtag_xfer_type {
	JTAG_SIR_XFER = 0,
	JTAG_SDR_XFER = 1,
};

enum jtag_endstate {
	JTAG_TLRESET,
	JTAG_IDLE,
	JTAG_PAUSEDR,
	JTAG_PAUSEIR,
	JTAG_SHIFTDR,
	JTAG_SHIFTIR
};

struct jtag_runtest_idle {
	enum jtag_xfer_mode mode;
	enum jtag_endstate end;
	unsigned int tck;
};

struct jtag_xfer {
	enum jtag_xfer_mode mode;
	enum jtag_xfer_type type;
	unsigned short length;
	unsigned int *tdi;
	unsigned int *tdo;
	enum jtag_endstate end_sts;
};

struct sir_xfer {
	enum jtag_xfer_mode mode;
	unsigned short length;
	unsigned int *tdi;
	unsigned int *tdo;
	unsigned char endir;
};

struct sdr_xfer {
	enum jtag_xfer_mode mode;
	unsigned char direct;
	unsigned short length;
	unsigned int *tdio;
	unsigned char enddr;
};


struct io_xfer {
	enum jtag_xfer_mode mode;
	unsigned long Address;
	unsigned long Data;
};

struct trst_reset {
	unsigned long operation;
	unsigned long Data;
};

#define JTAGIOC_BASE 'T'

#define ASPEED_JTAG_IOCRUNTEST _IOW(JTAGIOC_BASE, 0, struct jtag_runtest_idle)
#define ASPEED_JTAG_IOCSIR _IOWR(JTAGIOC_BASE, 1, struct sir_xfer)
#define ASPEED_JTAG_IOCSDR _IOWR(JTAGIOC_BASE, 2, struct sdr_xfer)
#define ASPEED_JTAG_SIOCFREQ _IOW(JTAGIOC_BASE, 3, unsigned int)
#define ASPEED_JTAG_GIOCFREQ _IOR(JTAGIOC_BASE, 4, unsigned int)
#define ASPEED_JTAG_IOWRITE _IOW(JTAGIOC_BASE, 5, struct io_xfer)
#define ASPEED_JTAG_IOREAD _IOR(JTAGIOC_BASE, 6, struct io_xfer)
#define ASPEED_JTAG_RESET _IOW(JTAGIOC_BASE, 7, struct io_xfer)
#define ASPEED_JTAG_TRST_RESET _IOW(JTAGIOC_BASE, 8, struct trst_reset)
#define ASPEED_JTAG_IOCXFER _IOWR(JTAGIOC_BASE, 9, struct jtag_xfer)
#define ASPEED_JTAG_RUNTCK _IOW(JTAGIOC_BASE, 12, struct io_xfer)

#endif /* _UAPI_LINUX_ASPEED_JTAG_H */
