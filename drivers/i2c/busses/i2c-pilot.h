/*
 *  Aspeed PilotXX I2C Controller.
 *  (C) Copyright 2019 Vishal C Nigade (vishal.nigade@aspeedtech.com)
 *  Copyright (c) 2019, Aspeed Technologies Inc.
 *  SPDX-License-Identifier:     GPL-2.0+
 */
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/nmi.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#define DEBUG
#undef DEBUG
#define BUS_COUNT					10
#define DEFAULT_I2C_SPEED				400
#define DEFAULT_H_I2C_SPEED				3400
#define DEFAULT_SLAVE_ADDR				0x10
#define DEFAULT_TX_THRESHOLD				16
#define DEFAULT_RX_THRESHOLD				8
#if 0

int fifo_depth[ BUS_COUNT ] =
{
#if (BUS_COUNT>=1)
	36,
#endif
#if (BUS_COUNT>=2)
	36,
#endif
#if (BUS_COUNT>=3)
	36,
#endif
#if (BUS_COUNT>=4)
	36,
#endif
#if (BUS_COUNT>=5)
	36,
#endif
#if (BUS_COUNT>=6)
	128,
#endif
#if (BUS_COUNT>=7)
	128,
#endif
#if (BUS_COUNT>=8)
	128,
#endif
#if (BUS_COUNT>=9)
	128,
#endif
#if (BUS_COUNT>=10)
	36
#endif

};

#endif
/* Sensible Defaults */
#define DEFAULT_TIMEOUT					( 100 )         /* Timeout/delay for bus activities */
#define DEFAULT_RETRIES					( 3 )           /* Retries on send/receive */
#define DEFAULT_BB_RETRIES				( 10 )          /* Retries to get a free bus */
#define PILOT_II_SIGNAL_RECEIVED			(0xffff)

/* BUS recovery vals */
#define DEFAULT_NUM_PULSES				16
#define DEFAULT_PULSE_PERIOD				5
#define DEFAULT_FREQ					100000
#define ENABLE_SMBUS_RESET				1
#define DISABLE_SMBUS_RESET				0
#define ENABLE_CLOCK_PULSE				1
#define DISABLE_CLOCK_PULSE				0
#define ENABLE_FORCE_STOP				1
#define DISABLE_FORCE_STOP				0



/* I2C Register */

#define PILOT_I2C_CTRL_2				0x910
#define PILOT_I2C_COUNTER_INTR_EN_REG  			0x938
#define PILOT_I2C_COUNTER_EN				0x93C
#define PILOT_I2C_COUNTER_STS_REG			0x940
#define PILOT_I2C_FEATURE_EN_0				0x948
#define PILOT_I2C_FEATURE_EN_1				0x94C
#define PILOT_I2C_CTRL_3				0x960

#define PILOT_I2C_COUNTER_CTRL8				0x964
#define PILOT_I2C_COUNTER_CTRL9				0x968
#define PILOT_I2C_COUNTER_INTR_EN_REG_1			0x96C
#define PILOT_I2C_COUNTER_EN_1				0x970
#define PILOT_I2C_COUNTER_STS_REG_1				0x974
#define PILOT_I2C_DEBUG_STS_1				0x978
#define PILOT_I2C_DEBUG_STS				0x944
#define PILOT_I2C_FEATURE_EN_2				0x97C


#define PILOT_I2CPCT0					0x24
#define PILOT_I2CPCT1					0x28


#define SMB_TFIFO_TO					BIT(3)
#define SMB_GTO						BIT(2)
#define SMB_STO						BIT(1)
#define SMB_MTO						BIT(0)
#define PILOT_I2C_TO_BITS				((SMB_TFIFO_TO)|(SMB_MTO)|(SMB_STO)|(SMB_GTO))

//P3_Features
#define  EN_STOP_DET					BIT(0)
#define  EN_BUS_HANG_FIX				BIT(1)
#define  EN_CLK_STREACHING				BIT(2)
#define  EN_TXFIFO_TO					BIT(3)

#define PILOT_I2C_FEATURES				((EN_TXFIFO_TO)|(EN_BUS_HANG_FIX))


#define PILOT_I2C_ENABLE				BIT(0)

#define PILOT_I2C_FUN_CTRL_REG				0x00
#define PILOT_I2C_M_TAR					0x04
#define PILOT_I2C_M_SAR					0x08
#define PILOT_I2C_CMD_REG				0x10
#define PILOT_I2C_SS_SCL_HCNT_REG			0x14
#define PILOT_I2C_SS_SCL_LCNT_REG			0x18
#define PILOT_I2C_FS_SCL_HCNT_REG			0x1C
#define PILOT_I2C_FS_SCL_LCNT_REG			0x20
#define PILOT_I2C_HS_SCL_HCNT_REG			0x24
#define PILOT_I2C_HS_SCL_LCNT_REG			0x28
#define PILOT_I2C_INTR_STS_REG				0x2C
#define PILOT_I2C_INTR_CTRL_REG				0x30
#define PILOT_I2C_RX_TL_REG				0x38
#define PILOT_I2C_TX_TL_REG				0x3C
#define PILOT_I2C_INTR_CLR_REG				0x40

#define PILOT_I2C_CLR_RX_OVER_REG			0x48
#define PILOT_I2C_CLR_ACTIVITY_REG			0X5C

#define PILOT_I2C_ENABLE_REG				0x6C
#define PILOT_I2C_STATUS_REG				0x70
#define PILOT_I2C_TXFLR_REG				0x74
#define PILOT_I2C_RXFLR_REG				0x78
#define PILOT_I2C_ABRT_SOURCE_REG			0x80
#define PILOT_I2C_SLV_DATA_NACK_ONLY			0x84

#define PILOT_TFNF					BIT(1)
#define PILOT_I2C_STATUS_ACTIVITY			BIT(0)

/* FUNCTION CONTROL REGISTER */
#define PILOT_I2CF_MASTER_EN				BIT(0)
#define PILOT_I2CF_FAST_MODE_H				BIT(2)
#define PILOT_I2CF_FAST_MODE_L				BIT(1)
#define PILOT_I2CF_10BITADDR_SLAVE			BIT(3)
#define PILOT_I2CF_10BITADDR_MASTER			BIT(4)
#define PILOT_I2CF_RESTART_EN				BIT(5)
#define PILOT_I2CF_SLAVE_EN				BIT(6)
#define PILOT_STOP_DET_IFADDRESSED			BIT(7)
#define PILOT_I2C_RXFIFO_CLK_STRETCH			BIT(9)
/* Command And Data Regesiter */
#define PILOT_I2CD_RW_CMD				BIT(8)
#define PILOT_I2CD_M_STOP_CMD				BIT(9)

/* TAR Functionallity */
#define PILOT_I2CT_10BITADDR_MASTER			BIT(12)

#define PILOT_I2CD_INTR_MUST_ON_HOLD			BIT(13)
#define PILOT_I2CD_INTR_RESTART_DET			BIT(12)
#define PILOT_I2CD_INTR_GEN_CALL			BIT(11)
#define PILOT_I2CD_INTR_START_DET			BIT(10)
#define PILOT_I2CD_INTR_STOP_DET			BIT(9)
#define PILOT_I2CD_INTR_ACTIVITY			BIT(8)
#define PILOT_I2CD_INTR_RX_DONE				BIT(7)
#define PILOT_I2CD_INTR_TX_ABRT				BIT(6)
#define PILOT_I2CD_INTR_RD_REQ				BIT(5)
#define PILOT_I2CD_INTR_TX_EMPTY			BIT(4)
#define PILOT_I2CD_INTR_TX_OVER				BIT(3)
#define PILOT_I2CD_INTR_RX_FULL				BIT(2)
#define PILOT_I2CD_INTR_RX_OVER				BIT(1)
#define PILOT_I2CD_INTR_RX_UNDER			BIT(0)
#define PILOT_I2CD_INTR_ALL				\
	 (PILOT_I2CD_INTR_RESTART_DET		|	\
	 PILOT_I2CD_INTR_RX_OVER		|	\
	 PILOT_I2CD_INTR_RX_FULL		|	\
	 PILOT_I2CD_INTR_STOP_DET		|	\
	 PILOT_I2CD_INTR_RD_REQ			|	\
	 PILOT_I2CD_INTR_TX_ABRT			\
	 )

#define PILOT_I2C_CLR_RX_UNDER_REG			0x44
#define PILOT_I2C_CLR_RX_OVER_REG			0x48
#define PILOT_I2C_CLR_RESTART_DET_REG			0xA8
#define PILOT_I2C_CLR_STOP_DET_REG			0X60
#define PILOT_I2C_CLR_RD_REQ_REG			0X50
#define PILOT_I2C_CLR_TX_ABRT_REG			0X54


#define PILOT_ABRT_7B_ADDR_NOACK			BIT(0)
#define PILOT_ABRT_10ADDR1_NOACK			BIT(1)
#define PILOT_ABRT_10ADDR2_NOACK			BIT(2)
#define PILOT_ABRT_TXDATA_NOACK				BIT(3)
#define PILOT_ABRT_GCALL_NOACK				BIT(4)
#define PILOT_ABRT_GCALL_READ				BIT(5)
#define PILOT_ABRT_HS_ACKDET				BIT(6)
#define PILOT_ABRT_SBYTE_ACKDET				BIT(7)
#define PILOT_ABRT_HS_NORSTRT				BIT(8)
#define PILOT_ABRT_SBYTE_NORSTRT			BIT(9)
#define PILOT_ABRT_10B_RD_NORSTRT			BIT(10)
#define PILOT_ARB_MASTER_DIS				BIT(11)
#define PILOT_ARB_LOST					BIT(12)
#define PILOT_ABRT_SLVFLUSH_TXFIFO			BIT(13)
#define PILOT_ABRT_SLV_ARBLOST				BIT(14)
#define PILOT_ABRT_SLVRD_INTX				BIT(15)
#define PILOT_ABRT_USER_ABRT				BIT(16)


enum pilot_i2c_master_state {
	PILOT_I2C_MASTER_START,
	PILOT_I2C_MASTER_TX_FIRST,
	PILOT_I2C_MASTER_TX,
	PILOT_I2C_MASTER_RX_FIRST,
	PILOT_I2C_MASTER_RX,
	PILOT_I2C_MASTER_STOP,
	PILOT_I2C_MASTER_INACTIVE,
	PILOT_I2C_MASTER_RESTART,
};

enum pilot_i2c_slave_state {
	PILOT_I2C_SLAVE_START,
	PILOT_I2C_SLAVE_READ_REQUESTED,
	PILOT_I2C_SLAVE_READ_PROCESSED,
	PILOT_I2C_SLAVE_WRITE_REQUESTED,
	PILOT_I2C_SLAVE_WRITE_RECEIVED,
	PILOT_I2C_SLAVE_STOP,
};
static u32 m_bus_recovery_info [ BUS_COUNT ] =
{ DISABLE_SMBUS_RESET, ENABLE_CLOCK_PULSE, DISABLE_FORCE_STOP, DEFAULT_NUM_PULSES,DEFAULT_FREQ };



#define TRANSFERSIZE					1024
#define MAX_FIFO_LEN					16

#define MASTER						0x10
#define SLAVE						0x00
#define MASTER_RECV					0x10
#define MASTER_XMIT					0x11
#define SLAVE_XMIT					0x12
#define RESTART_MRMW					0x13
#define RESTART_MWMR					0x14


#define TMEXT						1
#define TSEXT						2
#define TTimeout					4


struct pilot_i2c_bus{
	struct i2c_adapter		adap;
	struct device			*dev;
	void __iomem			*base;
	void __iomem			*sys_base;
	struct reset_control		*rst;
	bool				slave_disable;
	/* Synchronizes I/O mem access to base. */
	spinlock_t			lock;
	struct completion		cmd_complete;
	u32				(*get_clk_reg_val)(u32 divisor);
	unsigned long			parent_clk_frequency;
	u32				bus_frequency;

	unsigned char			TX_data[TRANSFERSIZE];
	int				TX_len;
	int				TX_index;

	unsigned char			Slave_TX_data[TRANSFERSIZE];
	int				Slave_TX_len;
	int				Slave_TX_index;


	unsigned char			MasterRX_data[TRANSFERSIZE];
	int				MasterRX_len;
	int				MasterRX_index;
	int				Master_rd_cmd_index;

	unsigned char			Linear_SlaveRX_data[TRANSFERSIZE];
	int				Linear_SlaveRX_len;
	int				Linear_SlaveRX_index;

	unsigned char			SlaveRX_data[MAX_FIFO_LEN][TRANSFERSIZE];
	int				SlaveRX_len[MAX_FIFO_LEN];
	int				SlaveRX_index[MAX_FIFO_LEN];
	int				SlaveRX_Writer;
	int				SlaveRX_Reader;
	int				SlaveRX_Entries;

	spinlock_t			data_lock;
	spinlock_t			i2c_irq_lock;
	spinlock_t			to_irq_lock;

	volatile u32			op_status;
	volatile u32			abort_status;

	int 				master_xmit_recv_mode_flag;

	wait_queue_head_t		pilot_i2c_wait;

	int				start_detected;
	int				Time_Out;
	int				RD_REQ_Pending;
	int				bus_speed;
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	struct i2c_client		*slave;
	enum pilot_i2c_slave_state	slave_state;
#endif /* CONFIG_I2C_SLAVE */
	/* Transaction state. */
	enum pilot_i2c_master_state	master_state;
	struct i2c_msg			*msgs;
	size_t				buf_index;
	size_t				msgs_index;
	size_t				msgs_count;
	bool				send_stop;
	int				cmd_err;
	/* Protected only by i2c_lock_bus */
	int				master_xfer_result;
};


#if 0
struct pilot_i2c_bus {
	struct i2c_adapter		adap;
	struct device			*dev;
	void __iomem			*base;
	struct reset_control		*rst;
	/* Synchronizes I/O mem access to base. */
	spinlock_t			lock;
	struct completion		cmd_complete;
	u32				(*get_clk_reg_val)(u32 divisor);
	unsigned long			parent_clk_frequency;
	u32				bus_frequency;
	/* Transaction state. */
	enum pilot_i2c_master_state	master_state;
	struct i2c_msg			*msgs;
	size_t				buf_index;
	size_t				msgs_index;
	size_t				msgs_count;
	bool				send_stop;
	int				cmd_err;
	/* Protected only by i2c_lock_bus */
	int				master_xfer_result;

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	struct i2c_client		*slave;
	enum pilot_i2c_slave_state	slave_state;
#endif /* CONFIG_I2C_SLAVE */
};

#endif
