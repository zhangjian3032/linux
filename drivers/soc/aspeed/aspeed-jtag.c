// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * JTAG driver for the Aspeed SoC
 *
 * Copyright (C) 2021 ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 */
#include <linux/poll.h>
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
#include <linux/of.h>
#include <linux/of_device.h>
#include <asm/uaccess.h>
#include <uapi/linux/aspeed-jtag.h>
/******************************************************************************/
#define ASPEED_JTAG_DATA		0x00
#define ASPEED_JTAG_INST		0x04
#define ASPEED_JTAG_CTRL		0x08
#define ASPEED_JTAG_ISR			0x0C
#define ASPEED_JTAG_SW			0x10
#define ASPEED_JTAG_TCK			0x14
#define ASPEED_JTAG_IDLE		0x18

/* ASPEED_JTAG_CTRL - 0x08 : Engine Control */
#define JTAG_ENG_EN			BIT(31)
#define JTAG_ENG_OUT_EN			BIT(30)
#define JTAG_FORCE_TMS			BIT(29)

#define JTAG_IR_UPDATE			BIT(26)		//AST2500 only

#define JTAG_G6_RESET_FIFO		BIT(21)		//AST2600 only
#define JTAG_G6_CTRL_MODE		BIT(20)		//AST2600 only
#define JTAG_G6_XFER_LEN_MASK		(0x3ff << 8)	//AST2600 only
#define JTAG_G6_SET_XFER_LEN(x)		(x << 8)
#define JTAG_G6_MSB_FIRST		BIT(6)		//AST2600 only
#define JTAG_G6_TERMINATE_XFER		BIT(5)		//AST2600 only
#define JTAG_G6_LAST_XFER		BIT(4)		//AST2600 only
#define JTAG_G6_INST_EN			BIT(1)

#define JTAG_INST_LEN_MASK		(0x3f << 20)
#define JTAG_SET_INST_LEN(x)		(x << 20)
#define JTAG_SET_INST_MSB		BIT(19)
#define JTAG_TERMINATE_INST		BIT(18)
#define JTAG_LAST_INST			BIT(17)
#define JTAG_INST_EN			BIT(16)
#define JTAG_DATA_LEN_MASK		(0x3f << 4)

#define JTAG_DR_UPDATE			BIT(10)		//AST2500 only
#define JTAG_DATA_LEN(x)		(x << 4)
#define JTAG_MSB_FIRST			BIT(3)
#define JTAG_TERMINATE_DATA		BIT(2)
#define JTAG_LAST_DATA			BIT(1)
#define JTAG_DATA_EN			BIT(0)

/* ASPEED_JTAG_ISR	- 0x0C : INterrupt status and enable */
#define JTAG_INST_PAUSE			BIT(19)
#define JTAG_INST_COMPLETE		BIT(18)
#define JTAG_DATA_PAUSE			BIT(17)
#define JTAG_DATA_COMPLETE		BIT(16)

#define JTAG_INST_PAUSE_EN		BIT(3)
#define JTAG_INST_COMPLETE_EN		BIT(2)
#define JTAG_DATA_PAUSE_EN		BIT(1)
#define JTAG_DATA_COMPLETE_EN		BIT(0)

/* ASPEED_JTAG_SW	- 0x10 : Software Mode and Status */
#define JTAG_SW_MODE_EN			BIT(19)
#define JTAG_SW_MODE_TCK		BIT(18)
#define JTAG_SW_MODE_TMS		BIT(17)
#define JTAG_SW_MODE_TDIO		BIT(16)
//
#define JTAG_STS_INST_PAUSE		BIT(2)
#define JTAG_STS_DATA_PAUSE		BIT(1)
#define JTAG_STS_ENG_IDLE		(0x1)

/* ASPEED_JTAG_TCK	- 0x14 : TCK Control */
#define JTAG_TCK_INVERSE		BIT(31)
#define JTAG_TCK_DIVISOR_MASK		(0x7ff)
#define JTAG_GET_TCK_DIVISOR(x)		(x & 0x7ff)

/*  ASPEED_JTAG_IDLE - 0x18 : Ctroller set for go to IDLE */
#define JTAG_CTRL_TRSTn_HIGH		BIT(31)
#define JTAG_GO_IDLE			BIT(0)

#define BUFFER_LEN			1024
#define TCK_FREQ			8000000
/******************************************************************************/
#define ASPEED_JTAG_DEBUG

#ifdef ASPEED_JTAG_DEBUG
#define JTAG_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#else
#define JTAG_DBUG(fmt, args...)
#endif

struct aspeed_jtag_config {
	u8	jtag_version;
	u32	jtag_buff_len;
};

struct aspeed_jtag_info {
	void __iomem			*reg_base;
	struct aspeed_jtag_config	*config;
	u32				*tdi;
	u32				*tdo;
	enum jtag_endstate		sts;
	int				irq;	// JTAG IRQ number
	struct reset_control		*reset;
	struct clk			*clk;
	u32				clkin;	// ast2600 use hclk, old use pclk
	u32				sw_delay; /* unit is ns */
	u32				flag;
	wait_queue_head_t		jtag_wq;
	bool				is_open;
	struct miscdevice		misc_dev;
};

/*
 * This structure represents a TMS cycle, as expressed in a set of bits and a
 * count of bits (note: there are no start->end state transitions that require
 * more than 1 byte of TMS cycles)
 */
struct tms_cycle {
	unsigned char		tmsbits;
	unsigned char		count;
};

/*
 * This is the complete set TMS cycles for going from any TAP state to any
 * other TAP state, following a "shortest path" rule.
 */
static const struct tms_cycle _tms_cycle_lookup[][6] = {
/*	    TLR        RTI        PDR      PIR      SDR      SIR*/
/* TLR  */{{0x1f, 5}, {0x00, 1}, {0x0a, 5}, {0x16, 6}, {0x02, 4}, {0x06, 5}},

/*	    TLR        RTI        PDR      PIR      SDR      SIR*/
/* RTI  */{{0x1f, 5}, {0x00, 0}, {0x05, 4}, {0x0b, 5}, {0x01, 3}, {0x03, 4}},

/*	    TLR        RTI        PDR      PIR      SDR      SIR*/
/* PDR  */{{0x1f, 5}, {0x03, 3}, {0x00, 0}, {0x2f, 7}, {0x01, 2}, {0x0f, 6}},

/*	    TLR        RTI        PDR      PIR      SDR      SIR*/
/* PIR  */{{0x1f, 5}, {0x03, 3}, {0x17, 6}, {0x00, 0}, {0x07, 5}, {0x01, 2}},

/*	    TLR        RTI        PDR      PIR      SDR      SIR*/
/* SDR  */{{0x1f, 5}, {0x03, 3}, {0x01, 2}, {0x2f, 7}, {0x00, 0}, {0x0f, 6}},

/*	    TLR        RTI        PDR      PIR      SDR      SIR*/
/* SIR  */{{0x1f, 5}, {0x03, 3}, {0x17, 6}, {0x01, 2}, {0x07, 5}, {0x00, 0}}

};

/******************************************************************************/
static DEFINE_SPINLOCK(jtag_state_lock);

/******************************************************************************/
static inline u32
aspeed_jtag_read(struct aspeed_jtag_info *aspeed_jtag, u32 reg)
{
	int val;
	val = readl(aspeed_jtag->reg_base + reg);
	JTAG_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
}

static inline void
aspeed_jtag_write(struct aspeed_jtag_info *aspeed_jtag, u32 val, u32 reg)
{
	JTAG_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, aspeed_jtag->reg_base + reg);
}

/******************************************************************************/
static void aspeed_jtag_set_freq(struct aspeed_jtag_info *aspeed_jtag, unsigned int freq)
{
	u32 div;

	/* SW mode frequency setting */
	aspeed_jtag->sw_delay = DIV_ROUND_UP(NSEC_PER_SEC, freq);
	JTAG_DBUG("sw mode delay = %d \n", aspeed_jtag->sw_delay);
	/*
	 * HW mode frequency setting
	 * AST2600: TCK period = Period of PCLK * (JTAG14[10:0] + 1)
	 * AST2500: TCK period = Period of PCLK * (JTAG14[10:0] + 1) * 2
	 */
	if (aspeed_jtag->config->jtag_version == 6)
		div = DIV_ROUND_CLOSEST(aspeed_jtag->clkin, freq);
	else
		div = DIV_ROUND_CLOSEST(aspeed_jtag->clkin, freq * 2);
	if (div >= 1)
		div = div - 1;
	if (div > JTAG_TCK_DIVISOR_MASK)
		div = JTAG_TCK_DIVISOR_MASK;
	JTAG_DBUG("%d target freq = %d div = %d", aspeed_jtag->clkin, freq,
		  div);
	/*
	 * HW constraint:
	 * AST2600 minimal TCK divisor = 7 
	 * AST2500 minimal TCK divisor = 1
	 */
	if (aspeed_jtag->config->jtag_version == 6) {
		if (div < 7)
			div = 7;
	} else if (aspeed_jtag->config->jtag_version == 0) {
		if (div < 1)
			div = 1;
	}
	JTAG_DBUG("set div = %x \n", div);

	aspeed_jtag_write(aspeed_jtag, ((aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_TCK) & ~JTAG_TCK_DIVISOR_MASK) | div),  ASPEED_JTAG_TCK);
}

static unsigned int aspeed_jtag_get_freq(struct aspeed_jtag_info *aspeed_jtag)
{
	unsigned int freq;
	if (aspeed_jtag->config->jtag_version == 6) {
		/* TCK period = Period of PCLK * (JTAG14[10:0] + 1) */
		freq = aspeed_jtag->clkin /
		       (JTAG_GET_TCK_DIVISOR(aspeed_jtag_read(
				aspeed_jtag, ASPEED_JTAG_TCK)) + 1);
	} else if (aspeed_jtag->config->jtag_version == 0) {
		/* TCK period = Period of PCLK * (JTAG14[10:0] + 1) * 2 */
		freq = (aspeed_jtag->clkin /
			(JTAG_GET_TCK_DIVISOR(aspeed_jtag_read(
				 aspeed_jtag, ASPEED_JTAG_TCK)) +1)) >> 1;
	} else {
		/* unknown jtag version */
		freq = 0;
	}
	return freq;
}
/******************************************************************************/
static u8 TCK_Cycle(struct aspeed_jtag_info *aspeed_jtag, u8 TMS, u8 TDI)
{
	u8 tdo;

	/* IEEE 1149.1
	 * TMS & TDI shall be sampled by the test logic on the rising edge
	 * test logic shall change TDO on the falling edge
	 */
	// TCK = 0
	aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | (TMS * JTAG_SW_MODE_TMS) | (TDI * JTAG_SW_MODE_TDIO), ASPEED_JTAG_SW);

	/* Target device have their operating frequency*/
	ndelay(aspeed_jtag->sw_delay);

	// TCK = 1
	aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TCK | (TMS * JTAG_SW_MODE_TMS) | (TDI * JTAG_SW_MODE_TDIO), ASPEED_JTAG_SW);

	ndelay(aspeed_jtag->sw_delay);
	/* Sampled TDI(slave, master's TDO) on the rising edge */
	if (aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_SW) & JTAG_SW_MODE_TDIO)
		tdo = 1;
	else
		tdo = 0;

	JTAG_DBUG("tms: %d tdi: %d tdo: %d", TMS, TDI, tdo);

	return tdo;
}

static int aspeed_jtag_sw_set_tap_state(struct aspeed_jtag_info *aspeed_jtag,
				      enum jtag_endstate endstate)
{
	int i = 0;
	enum jtag_endstate from, to;

	if (endstate > JTAG_SHIFTIR)
		return -EFAULT;

	from = aspeed_jtag->sts;
	to = endstate;
	for (i = 0; i < _tms_cycle_lookup[from][to].count; i++)
		TCK_Cycle(aspeed_jtag,
			((_tms_cycle_lookup[from][to].tmsbits >> i) & 0x1), 0);
	aspeed_jtag->sts = endstate;
	JTAG_DBUG("go to %d", endstate);
	return 0;
}

/******************************************************************************/
static void aspeed_jtag_wait_instruction_pause_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq,
				 (aspeed_jtag->flag & JTAG_INST_PAUSE));
	JTAG_DBUG("\n");
	aspeed_jtag->flag &= ~JTAG_INST_PAUSE;
}

static void aspeed_jtag_wait_instruction_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq,
				 (aspeed_jtag->flag & JTAG_INST_COMPLETE));
	JTAG_DBUG("\n");
	aspeed_jtag->flag &= ~JTAG_INST_COMPLETE;
}

static void aspeed_jtag_wait_data_pause_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq,
				 (aspeed_jtag->flag & JTAG_DATA_PAUSE));
	JTAG_DBUG("\n");
	aspeed_jtag->flag &= ~JTAG_DATA_PAUSE;
}

static void aspeed_jtag_wait_data_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq,
				 (aspeed_jtag->flag & JTAG_DATA_COMPLETE));
	JTAG_DBUG("\n");
	aspeed_jtag->flag &= ~JTAG_DATA_COMPLETE;
}

static int aspeed_jtag_run_to_tlr(struct aspeed_jtag_info *aspeed_jtag)
{
	if (aspeed_jtag->sts == JTAG_PAUSEIR)
		aspeed_jtag_write(aspeed_jtag, JTAG_INST_COMPLETE_EN,
				ASPEED_JTAG_ISR);
	else if (aspeed_jtag->sts == JTAG_PAUSEDR)
		aspeed_jtag_write(aspeed_jtag, JTAG_DATA_COMPLETE_EN,
				ASPEED_JTAG_ISR);
	aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN | JTAG_FORCE_TMS, ASPEED_JTAG_CTRL);	// x TMS high + 1 TMS low
	if (aspeed_jtag->sts == JTAG_PAUSEIR)
		aspeed_jtag_wait_instruction_complete(aspeed_jtag);
	else if (aspeed_jtag->sts == JTAG_PAUSEDR)
		aspeed_jtag_wait_data_complete(aspeed_jtag);
	/* After that the fsm will go to idle state: hw constraint */
	aspeed_jtag->sts = JTAG_IDLE;
	return 0;
}

static int aspeed_jtag_run_to_idle(struct aspeed_jtag_info *aspeed_jtag)
{
	if (aspeed_jtag->sts == JTAG_TLRESET) {
		TCK_Cycle(aspeed_jtag, 0, 0);
	} else if (aspeed_jtag->sts == JTAG_IDLE) {
		/* nothing to do */
	} else if (aspeed_jtag->sts == JTAG_PAUSEDR) {
		aspeed_jtag_write(aspeed_jtag, JTAG_DATA_COMPLETE_EN,
					  ASPEED_JTAG_ISR);
		if (aspeed_jtag->config->jtag_version == 6) {
			aspeed_jtag_write(aspeed_jtag,
					JTAG_ENG_EN | JTAG_ENG_OUT_EN |
						JTAG_G6_TERMINATE_XFER |
						JTAG_DATA_EN,
					ASPEED_JTAG_CTRL);
		} else {
			aspeed_jtag_write(aspeed_jtag,
					  JTAG_ENG_EN | JTAG_ENG_OUT_EN |
						  JTAG_TERMINATE_DATA |
						  JTAG_DATA_EN,
					  ASPEED_JTAG_CTRL);
		}
		aspeed_jtag_wait_data_complete(aspeed_jtag);
	} else if (aspeed_jtag->sts == JTAG_PAUSEIR) {
		aspeed_jtag_write(aspeed_jtag, JTAG_INST_COMPLETE_EN,
					  ASPEED_JTAG_ISR);
		if (aspeed_jtag->config->jtag_version == 6) {
			aspeed_jtag_write(aspeed_jtag,
					JTAG_ENG_EN | JTAG_ENG_OUT_EN |
						JTAG_G6_TERMINATE_XFER |
						JTAG_G6_INST_EN,
					ASPEED_JTAG_CTRL);
		} else {
			aspeed_jtag_write(aspeed_jtag,
					JTAG_ENG_EN | JTAG_ENG_OUT_EN |
						JTAG_TERMINATE_INST |
						JTAG_INST_EN,
					ASPEED_JTAG_CTRL);
		}
		aspeed_jtag_wait_instruction_complete(aspeed_jtag);
	} else {
		pr_err("Should not get here unless aspeed_jtag->sts error!");
		return -EFAULT;
	}
	aspeed_jtag->sts = JTAG_IDLE;
	return 0;
}

static int aspeed_jtag_hw_set_tap_state(struct aspeed_jtag_info *aspeed_jtag,
				      enum jtag_endstate endstate)
{
	int ret;

	aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_SW); //dis sw mode
	mdelay(2);
	if (endstate == JTAG_TLRESET) {
		ret = aspeed_jtag_run_to_tlr(aspeed_jtag);
	} else if (endstate == JTAG_IDLE) {
		ret = aspeed_jtag_run_to_idle(aspeed_jtag);
	} else {
		pr_warn("HW mode not support state %d", endstate);
		return -EFAULT;
	}
	return ret;
}

/******************************************************************************/
/* JTAG_reset() is to generate at leaspeed 9 TMS high and
 * 1 TMS low to force devices into Run-Test/Idle State
 */
static int aspeed_jtag_run_test_idle(struct aspeed_jtag_info *aspeed_jtag, struct jtag_runtest_idle *runtest)
{
	int i = 0;
	int ret;

	JTAG_DBUG(":%s mode\n", runtest->mode ? "SW" : "HW");

	if (runtest->mode) {
		// SW mode
		ret = aspeed_jtag_sw_set_tap_state(aspeed_jtag, runtest->end);
	} else {
		ret = aspeed_jtag_hw_set_tap_state(aspeed_jtag, runtest->end);
	}
	if (ret)
		return ret;
	for (i = 0; i < runtest->tck; i++)
		TCK_Cycle(aspeed_jtag, 0, 0);
	return 0;
}
static void aspeed_sw_jtag_xfer(struct aspeed_jtag_info *aspeed_jtag, struct jtag_xfer *xfer)
{
	unsigned int index = 0;
	u32 shift_bits = 0;
	u32 tdi = 0, tdo = 0;
	u32 remain_xfer = xfer->length;

	if (xfer->type == JTAG_SIR_XFER)
		aspeed_jtag_sw_set_tap_state(aspeed_jtag, JTAG_SHIFTIR);
	else
		aspeed_jtag_sw_set_tap_state(aspeed_jtag, JTAG_SHIFTDR);

	aspeed_jtag->tdo[index] = 0;
	while (remain_xfer) {
		tdi = (aspeed_jtag->tdi[index]) >> (shift_bits % 32) & (0x1);
		if (remain_xfer == 1)
			tdo = TCK_Cycle(aspeed_jtag, 1, tdi); // go to Exit1-IR
		else
			tdo = TCK_Cycle(aspeed_jtag, 0, tdi); // go to IRShift
		aspeed_jtag->tdo[index] |= (tdo << (shift_bits % 32));
		shift_bits++;
		remain_xfer--;
		if ((shift_bits % 32) == 0) {
			index++;
			aspeed_jtag->tdo[index] = 0;
		}
	}
	TCK_Cycle(aspeed_jtag, 0, 0);
	if (xfer->type == JTAG_SIR_XFER)
		aspeed_jtag->sts = JTAG_PAUSEIR;
	else
		aspeed_jtag->sts = JTAG_PAUSEDR;
	aspeed_jtag_sw_set_tap_state(aspeed_jtag, xfer->end_sts);
}

static int aspeed_hw_ir_scan(struct aspeed_jtag_info *aspeed_jtag, enum jtag_endstate end_sts, u32 shift_bits)
{
	if (end_sts == JTAG_PAUSEIR) {
		aspeed_jtag_write(aspeed_jtag, JTAG_INST_PAUSE_EN,
					  ASPEED_JTAG_ISR);
		if (aspeed_jtag->config->jtag_version == 6) {
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_G6_SET_XFER_LEN(shift_bits),
					ASPEED_JTAG_CTRL);
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_G6_SET_XFER_LEN(shift_bits) |
					JTAG_G6_INST_EN, ASPEED_JTAG_CTRL);
		} else {
			if (aspeed_jtag->sts == JTAG_PAUSEDR)
				aspeed_jtag_write(aspeed_jtag,
						  JTAG_INST_PAUSE_EN |
							  JTAG_DATA_COMPLETE_EN,
						  ASPEED_JTAG_ISR);
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_SET_INST_LEN(shift_bits),
					ASPEED_JTAG_CTRL);
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_SET_INST_LEN(shift_bits) |
					JTAG_INST_EN, ASPEED_JTAG_CTRL);
			if (aspeed_jtag->sts == JTAG_PAUSEDR)
				aspeed_jtag_wait_data_complete(aspeed_jtag);
		}
		aspeed_jtag_wait_instruction_pause_complete(aspeed_jtag);
		aspeed_jtag->sts = JTAG_PAUSEIR;
	} else if (end_sts == JTAG_IDLE) {
		aspeed_jtag_write(aspeed_jtag, JTAG_INST_COMPLETE_EN,
					  ASPEED_JTAG_ISR);
		if (aspeed_jtag->config->jtag_version == 6) {
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_G6_LAST_XFER | JTAG_G6_SET_XFER_LEN(shift_bits),
					ASPEED_JTAG_CTRL);
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_G6_LAST_XFER | JTAG_G6_SET_XFER_LEN(shift_bits) |
					JTAG_G6_INST_EN, ASPEED_JTAG_CTRL);
		} else {
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_LAST_INST | JTAG_SET_INST_LEN(shift_bits),
					ASPEED_JTAG_CTRL);
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_LAST_INST | JTAG_SET_INST_LEN(shift_bits) |
					JTAG_INST_EN, ASPEED_JTAG_CTRL);
		}
		aspeed_jtag_wait_instruction_complete(aspeed_jtag);
		aspeed_jtag->sts = JTAG_IDLE;
	} else {
		pr_err("End state %d not support", end_sts);
		return -EFAULT;
	}
	return 0;
}

static int aspeed_hw_dr_scan(struct aspeed_jtag_info *aspeed_jtag, enum jtag_endstate end_sts, u32 shift_bits)
{
	if (end_sts == JTAG_PAUSEDR) {
		aspeed_jtag_write(aspeed_jtag, JTAG_DATA_PAUSE_EN,
					  ASPEED_JTAG_ISR);
		if (aspeed_jtag->config->jtag_version == 6) {
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_G6_SET_XFER_LEN(shift_bits),
					ASPEED_JTAG_CTRL);
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_G6_SET_XFER_LEN(shift_bits) |
					JTAG_DATA_EN, ASPEED_JTAG_CTRL);
		} else {
			if (aspeed_jtag->sts == JTAG_PAUSEIR)
				aspeed_jtag_write(aspeed_jtag,
						  JTAG_DATA_PAUSE_EN |
							  JTAG_INST_COMPLETE_EN,
						  ASPEED_JTAG_ISR);
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_DATA_LEN(shift_bits),
					ASPEED_JTAG_CTRL);
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_DATA_LEN(shift_bits) |
					JTAG_DATA_EN, ASPEED_JTAG_CTRL);
			if (aspeed_jtag->sts == JTAG_PAUSEIR)
				aspeed_jtag_wait_instruction_complete(
					aspeed_jtag);
		}
		aspeed_jtag_wait_data_pause_complete(aspeed_jtag);
		aspeed_jtag->sts = JTAG_PAUSEDR;
	} else if (end_sts == JTAG_IDLE) {
		aspeed_jtag_write(aspeed_jtag, JTAG_DATA_COMPLETE_EN,
					  ASPEED_JTAG_ISR);
		if (aspeed_jtag->config->jtag_version == 6) {
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_G6_LAST_XFER | JTAG_G6_SET_XFER_LEN(shift_bits),
					ASPEED_JTAG_CTRL);
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_G6_LAST_XFER | JTAG_G6_SET_XFER_LEN(shift_bits) |
					JTAG_DATA_EN, ASPEED_JTAG_CTRL);
		} else {
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_LAST_INST | JTAG_SET_INST_LEN(shift_bits),
					ASPEED_JTAG_CTRL);
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					JTAG_LAST_INST | JTAG_SET_INST_LEN(shift_bits) |
					JTAG_DATA_EN, ASPEED_JTAG_CTRL);
		}
		aspeed_jtag_wait_data_complete(aspeed_jtag);
		aspeed_jtag->sts = JTAG_IDLE;
	} else {
		pr_err("End state %d not support", end_sts);
		return -EFAULT;
	}
	return 0;
}

static void aspeed_hw_jtag_xfer(struct aspeed_jtag_info *aspeed_jtag, struct jtag_xfer *xfer)
{
	unsigned int index = 0;
	u32 shift_bits = 0;
	u32 remain_xfer = xfer->length;
	int i, tmp_idx = 0;
	u32 fifo_reg = xfer->type ? ASPEED_JTAG_DATA : ASPEED_JTAG_INST;

	aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_SW); //dis sw mode

	while (remain_xfer) {
		if (remain_xfer > aspeed_jtag->config->jtag_buff_len) {
			shift_bits = aspeed_jtag->config->jtag_buff_len;
			tmp_idx = shift_bits / 32;
			for (i = 0; i < tmp_idx; i++)
				aspeed_jtag_write(aspeed_jtag, aspeed_jtag->tdi[index + i], fifo_reg);
			if (xfer->type == JTAG_SIR_XFER)
				aspeed_hw_ir_scan(aspeed_jtag, JTAG_PAUSEIR, shift_bits);
			else
				aspeed_hw_dr_scan(aspeed_jtag, JTAG_PAUSEDR, shift_bits);
		} else {
			shift_bits = remain_xfer;
			tmp_idx = shift_bits / 32;
			if (shift_bits % 32)
				tmp_idx += 1;
			for (i = 0; i < tmp_idx; i++)
				aspeed_jtag_write(aspeed_jtag, aspeed_jtag->tdi[index + i], fifo_reg);
			if (xfer->type == JTAG_SIR_XFER)
				aspeed_hw_ir_scan(aspeed_jtag, xfer->end_sts, shift_bits);
			else
				aspeed_hw_dr_scan(aspeed_jtag, xfer->end_sts, shift_bits);
		}

		remain_xfer = remain_xfer - shift_bits;

		//handle tdo data
		tmp_idx = shift_bits / 32;
		if (shift_bits % 32)
			tmp_idx += 1;
		for (i = 0; i < tmp_idx; i++) {
			if (shift_bits < 32)
				aspeed_jtag->tdo[index + i] = aspeed_jtag_read(aspeed_jtag, fifo_reg) >> (32 - shift_bits);
			else
				aspeed_jtag->tdo[index + i] = aspeed_jtag_read(aspeed_jtag, fifo_reg);
			JTAG_DBUG("TDO[%d]: %x\n", index + i, aspeed_jtag->tdo[index + i]);
			shift_bits -= 32;
		}
		index += tmp_idx;
	}
}

static int aspeed_jtag_xfer(struct aspeed_jtag_info *aspeed_jtag, struct jtag_xfer *xfer)
{
	JTAG_DBUG("%s mode, END : %d, len : %d\n", xfer->mode ? "SW" : "HW", xfer->end_sts, xfer->length);
	memset(aspeed_jtag->tdi, 0, BUFFER_LEN * 2);

	if (copy_from_user(aspeed_jtag->tdi, xfer->tdi, (xfer->length + 7) / 8))
		return -EFAULT;

	if (xfer->mode)
		aspeed_sw_jtag_xfer(aspeed_jtag, xfer);
	else
		aspeed_hw_jtag_xfer(aspeed_jtag, xfer);

	if (copy_to_user(xfer->tdo, aspeed_jtag->tdo, (xfer->length + 7) / 8))
		return -EFAULT;

	return 0;
}
/*************************************************************************************/
static irqreturn_t aspeed_jtag_isr(int this_irq, void *dev_id)
{
	u32 status;
	struct aspeed_jtag_info *aspeed_jtag = dev_id;

	status = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_ISR);
	JTAG_DBUG("sts %x \n", status);

	if (status & JTAG_INST_PAUSE) {
		aspeed_jtag_write(aspeed_jtag, JTAG_INST_PAUSE | (status & 0xf), ASPEED_JTAG_ISR);
		aspeed_jtag->flag |= JTAG_INST_PAUSE;
	}

	if (status & JTAG_INST_COMPLETE) {
		aspeed_jtag_write(aspeed_jtag, JTAG_INST_COMPLETE | (status & 0xf), ASPEED_JTAG_ISR);
		aspeed_jtag->flag |= JTAG_INST_COMPLETE;
	}

	if (status & JTAG_DATA_PAUSE) {
		aspeed_jtag_write(aspeed_jtag, JTAG_DATA_PAUSE | (status & 0xf), ASPEED_JTAG_ISR);
		aspeed_jtag->flag |= JTAG_DATA_PAUSE;
	}

	if (status & JTAG_DATA_COMPLETE) {
		aspeed_jtag_write(aspeed_jtag, JTAG_DATA_COMPLETE | (status & 0xf), ASPEED_JTAG_ISR);
		aspeed_jtag->flag |= JTAG_DATA_COMPLETE;
	}

	if (aspeed_jtag->flag) {
		wake_up_interruptible(&aspeed_jtag->jtag_wq);
		return IRQ_HANDLED;
	} else {
		pr_err("TODO Check JTAG's interrupt %x\n", aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_ISR));
		return IRQ_NONE;
	}

}

static void JTAG_reset(struct aspeed_jtag_info *aspeed_jtag)
{
	aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_SW);
	aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN, ASPEED_JTAG_CTRL);
	aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN | JTAG_FORCE_TMS, ASPEED_JTAG_CTRL);
	mdelay(5);
	while (1) {
		if (aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_CTRL) & JTAG_FORCE_TMS)
			break;
	}
	aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO, ASPEED_JTAG_SW);
}

/*************************************************************************************/
static long jtag_ioctl(struct file *file, unsigned int cmd,
		       unsigned long arg)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_jtag_info *aspeed_jtag = container_of(c, struct aspeed_jtag_info, misc_dev);
	void __user *argp = (void __user *)arg;
	struct io_xfer io;
	struct trst_reset trst_pin;
	struct jtag_runtest_idle run_idle;
	struct jtag_xfer xfer;
	int ret = 0;

	switch (cmd) {
	case ASPEED_JTAG_GIOCFREQ:
		ret = __put_user(aspeed_jtag_get_freq(aspeed_jtag), (unsigned int __user *)arg);
		break;
	case ASPEED_JTAG_SIOCFREQ:
//			printk("set freq = %d , pck %d \n",config.freq, aspeed_get_pclk());
		if ((unsigned int)arg > aspeed_jtag->clkin)
			ret = -EFAULT;
		else
			aspeed_jtag_set_freq(aspeed_jtag, (unsigned int)arg);
		break;
	case ASPEED_JTAG_IOCRUNTEST:
		if (copy_from_user(&run_idle, argp, sizeof(struct jtag_runtest_idle)))
			ret = -EFAULT;
		else
			aspeed_jtag_run_test_idle(aspeed_jtag, &run_idle);
		break;
	case ASPEED_JTAG_IOCXFER:
		if (copy_from_user(&xfer, argp, sizeof(struct jtag_xfer)))
			return -EFAULT;
		if (xfer.length > 1024)
			return -EINVAL;
		ret = aspeed_jtag_xfer(aspeed_jtag, &xfer);
		if (ret)
			return ret;
		if (copy_to_user(argp, &xfer, sizeof(struct jtag_xfer)))
			return -EFAULT;
		break;
	case ASPEED_JTAG_IOWRITE:
		if (copy_from_user(&io, argp, sizeof(struct io_xfer))) {
			ret = -EFAULT;
		} else {
			void __iomem	*reg_add;

			reg_add = ioremap(io.Address, 4);
			writel(io.Data, reg_add);
			iounmap(reg_add);
		}

		break;
	case ASPEED_JTAG_IOREAD:
		if (copy_from_user(&io, argp, sizeof(struct io_xfer))) {
			ret = -EFAULT;
		} else {
			void __iomem	*reg_add;

			reg_add = ioremap(io.Address, 4);
			io.Data = readl(reg_add);
			iounmap(reg_add);
		}
		if (copy_to_user(argp, &io, sizeof(struct io_xfer)))
			ret = -EFAULT;
		break;
	case ASPEED_JTAG_RESET:
		JTAG_reset(aspeed_jtag);
		break;
	case ASPEED_JTAG_RUNTCK:
		if (copy_from_user(&io, argp, sizeof(struct io_xfer))) {
			ret = -EFAULT;
		} else {
			int i;

			for (i = 0; i < io.Address; i++)
				TCK_Cycle(aspeed_jtag, io.mode, io.Data);
		}
		break;
	case ASPEED_JTAG_TRST_RESET:
		if (copy_from_user(&trst_pin, argp, sizeof(struct trst_reset))) {
			ret = -EFAULT;
		} else {
			unsigned int regs = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_IDLE);

			if (trst_pin.operation == 1) {
				if (trst_pin.Data == 1)
					aspeed_jtag_write(aspeed_jtag, regs | (1 << 31), ASPEED_JTAG_IDLE);
				else
					aspeed_jtag_write(aspeed_jtag, regs & (~(1 << 31)), ASPEED_JTAG_IDLE);
			} else
				trst_pin.Data = (regs >> 31);

		}
		if (copy_to_user(argp, &trst_pin, sizeof(struct trst_reset)))
			ret = -EFAULT;
		break;
	default:
		return -ENOTTY;
	}

	return ret;
}

static int jtag_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_jtag_info *aspeed_jtag = container_of(c, struct aspeed_jtag_info, misc_dev);

	spin_lock(&jtag_state_lock);

	if (aspeed_jtag->is_open) {
		spin_unlock(&jtag_state_lock);
		return -EBUSY;
	}

	aspeed_jtag->is_open = true;

	spin_unlock(&jtag_state_lock);

	return 0;
}

static int jtag_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_jtag_info *aspeed_jtag = container_of(c, struct aspeed_jtag_info, misc_dev);


	spin_lock(&jtag_state_lock);

	aspeed_jtag->is_open = false;

	spin_unlock(&jtag_state_lock);

	return 0;
}

static ssize_t show_tdo(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_SW) & JTAG_SW_MODE_TDIO ? "1" : "0");
}

static DEVICE_ATTR(tdo, S_IRUGO, show_tdo, NULL);

static ssize_t store_tdi(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 tdi;
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);
	tdi = simple_strtoul(buf, NULL, 1);

	aspeed_jtag_write(aspeed_jtag, aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_SW) | JTAG_SW_MODE_EN | (tdi * JTAG_SW_MODE_TDIO), ASPEED_JTAG_SW);

	return count;
}

static DEVICE_ATTR(tdi, S_IWUSR, NULL, store_tdi);

static ssize_t store_tms(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 tms;
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);
	tms = simple_strtoul(buf, NULL, 1);

	aspeed_jtag_write(aspeed_jtag, aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_SW) | JTAG_SW_MODE_EN | (tms * JTAG_SW_MODE_TMS), ASPEED_JTAG_SW);

	return count;
}

static DEVICE_ATTR(tms, S_IWUSR, NULL, store_tms);

static ssize_t store_tck(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 tck;
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);
	tck = simple_strtoul(buf, NULL, 1);

	aspeed_jtag_write(aspeed_jtag, aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_SW) | JTAG_SW_MODE_EN | (tck * JTAG_SW_MODE_TDIO), ASPEED_JTAG_SW);

	return count;
}

static DEVICE_ATTR(tck, S_IWUSR, NULL, store_tck);

static ssize_t show_sts(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", aspeed_jtag->sts ? "Pause" : "Idle");
}

static DEVICE_ATTR(sts, S_IRUGO, show_sts, NULL);

static ssize_t show_frequency(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);
//	printk("PCLK = %d \n", aspeed_get_pclk());
//	printk("DIV  = %d \n", JTAG_GET_TCK_DIVISOR(aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_TCK)) + 1);
	return sprintf(buf, "Frequency : %d\n", aspeed_jtag->clkin / (JTAG_GET_TCK_DIVISOR(aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_TCK)) + 1));
}

static ssize_t store_frequency(struct device *dev,
			       struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);
	val = simple_strtoul(buf, NULL, 20);
	aspeed_jtag_set_freq(aspeed_jtag, val);

	return count;
}

static DEVICE_ATTR(freq, S_IRUGO | S_IWUSR, show_frequency, store_frequency);

static struct attribute *jtag_sysfs_entries[] = {
	&dev_attr_freq.attr,
	&dev_attr_sts.attr,
	&dev_attr_tck.attr,
	&dev_attr_tms.attr,
	&dev_attr_tdi.attr,
	&dev_attr_tdo.attr,
	NULL
};

static struct attribute_group jtag_attribute_group = {
	.attrs = jtag_sysfs_entries,
};

static const struct file_operations aspeed_jtag_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= jtag_ioctl,
	.open		= jtag_open,
	.release		= jtag_release,
};

static struct aspeed_jtag_config jtag_config = {
	.jtag_version = 0,
	.jtag_buff_len = 32,
};

static struct aspeed_jtag_config jtag_g6_config = {
	.jtag_version = 6,
	.jtag_buff_len = 512,
};

static const struct of_device_id aspeed_jtag_of_matches[] = {
	{ .compatible = "aspeed,ast2400-jtag", .data = &jtag_config, 	},
	{ .compatible = "aspeed,ast2500-jtag", .data = &jtag_config, 	},
	{ .compatible = "aspeed,ast2600-jtag", .data = &jtag_g6_config, },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_jtag_of_matches);

static int reserved_idx = -1;

static int aspeed_jtag_probe(struct platform_device *pdev)
{
	struct aspeed_jtag_info *aspeed_jtag;
	const struct of_device_id *jtag_dev_id;
	struct resource *res;
	int max_reserved_idx;
	int idx;
	int ret = 0;

	JTAG_DBUG("aspeed_jtag_probe\n");

	aspeed_jtag = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_jtag_info), GFP_KERNEL);
	if (!aspeed_jtag)
		return -ENOMEM;

	jtag_dev_id = of_match_device(aspeed_jtag_of_matches, &pdev->dev);
	if (!jtag_dev_id)
		return -EINVAL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	aspeed_jtag->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!aspeed_jtag->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	aspeed_jtag->irq = platform_get_irq(pdev, 0);
	if (aspeed_jtag->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	aspeed_jtag->reset = devm_reset_control_get_exclusive(&pdev->dev, "jtag");
	if (IS_ERR(aspeed_jtag->reset)) {
		dev_err(&pdev->dev, "can't get jtag reset\n");
		return PTR_ERR(aspeed_jtag->reset);
	}

	aspeed_jtag->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(aspeed_jtag->clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		return -ENODEV;
	}

	aspeed_jtag->clkin = clk_get_rate(aspeed_jtag->clk);
	JTAG_DBUG("aspeed_jtag->clkin %d \n", aspeed_jtag->clkin);

	aspeed_jtag->config = (struct aspeed_jtag_config *)jtag_dev_id->data;

	aspeed_jtag->tdi = kmalloc(BUFFER_LEN * 2, GFP_KERNEL);
	aspeed_jtag->tdo = aspeed_jtag->tdi + (BUFFER_LEN / sizeof(u32));

	JTAG_DBUG("buffer addr : tdi %x tdo %x \n", (u32)aspeed_jtag->tdi, (u32)aspeed_jtag->tdo);

	// SCU init
	reset_control_assert(aspeed_jtag->reset);
	udelay(3);
	reset_control_deassert(aspeed_jtag->reset);

	// enable clock
	aspeed_jtag_write(aspeed_jtag,
			  JTAG_ENG_EN | JTAG_ENG_OUT_EN,
			  ASPEED_JTAG_CTRL);

	// enable sw mode for disable clk
	aspeed_jtag_write(aspeed_jtag,
			  JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO,
			  ASPEED_JTAG_SW);

	ret = devm_request_irq(&pdev->dev, aspeed_jtag->irq, aspeed_jtag_isr,
			       0, dev_name(&pdev->dev), aspeed_jtag);
	if (ret) {
		printk("JTAG Unable to get IRQ");
		goto out_region;
	}

	// clear interrupt
	aspeed_jtag_write(aspeed_jtag,
			  JTAG_INST_PAUSE | JTAG_INST_COMPLETE |
			  JTAG_DATA_PAUSE | JTAG_DATA_COMPLETE,
			  ASPEED_JTAG_ISR);

	aspeed_jtag->flag = 0;
	init_waitqueue_head(&aspeed_jtag->jtag_wq);

	if (reserved_idx == -1) {
		max_reserved_idx = of_alias_get_highest_id("jtag");
		if (max_reserved_idx >= 0)
			reserved_idx = max_reserved_idx;
	}

	idx = of_alias_get_id(pdev->dev.of_node, "jtag");;
	if (idx < 0) {
		idx = ++reserved_idx;
	}

	aspeed_jtag->misc_dev.minor = MISC_DYNAMIC_MINOR;
	aspeed_jtag->misc_dev.name = kasprintf(GFP_KERNEL, "aspeed-jtag%d", idx);
	aspeed_jtag->misc_dev.fops = &aspeed_jtag_fops;

	ret = misc_register(&aspeed_jtag->misc_dev);
	if (ret) {
		printk(KERN_ERR "JTAG : failed to register misc device\n");
		goto out_irq;
	}

	platform_set_drvdata(pdev, aspeed_jtag);

	ret = sysfs_create_group(&pdev->dev.kobj, &jtag_attribute_group);
	if (ret) {
		printk(KERN_ERR "aspeed_jtag: failed to create sysfs device attributes.\n");
		return -1;
	}

	aspeed_jtag_set_freq(aspeed_jtag, TCK_FREQ);

	printk(KERN_INFO "aspeed_jtag: driver successfully loaded.\n");

	return 0;

out_irq:
	devm_free_irq(&pdev->dev, aspeed_jtag->irq, aspeed_jtag);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
	kfree(aspeed_jtag->tdi);
out:
	printk(KERN_WARNING "aspeed_jtag: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int aspeed_jtag_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct aspeed_jtag_info *aspeed_jtag = platform_get_drvdata(pdev);

	JTAG_DBUG("aspeed_jtag_remove\n");

	sysfs_remove_group(&pdev->dev.kobj, &jtag_attribute_group);

	misc_deregister(&aspeed_jtag->misc_dev);

	kfree_const(aspeed_jtag->misc_dev.name);

	devm_free_irq(&pdev->dev, aspeed_jtag->irq, aspeed_jtag);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(aspeed_jtag->reg_base);

	platform_set_drvdata(pdev, NULL);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

#ifdef CONFIG_PM
static int
aspeed_jtag_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int
aspeed_jtag_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

static struct platform_driver aspeed_jtag_driver = {
	.probe		= aspeed_jtag_probe,
	.remove		= aspeed_jtag_remove,
#ifdef CONFIG_PM
	.suspend	= aspeed_jtag_suspend,
	.resume		= aspeed_jtag_resume,
#endif
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = aspeed_jtag_of_matches,
	},
};

module_platform_driver(aspeed_jtag_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST JTAG LIB Driver");
MODULE_LICENSE("GPL");
