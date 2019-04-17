/*
 *  (C) Copyright 2019 Vishal C Nigade (vishal.nigade@aspeedtech.com)
 *  Copyright (c) 2019, Aspeed Technologies Inc.
 *  SPDX-License-Identifier:     GPL-2.0+
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>



#define SOURCE_UPDATE_ALLOW
//#undef SOURCE_UPDATE_ALLOW

#ifdef SOURCE_UPDATE_ALLOW
	#include <linux/regmap.h>
	#include <linux/mfd/syscon.h>
#endif

#define FANTACH_OFFSET_WRAP			0x8
#define PWM_OFFSET_WRAP 			0x4
#define NUMPWM					0x8
#define NUMFT					16
#define SELECT_200KHZ_CLK			BIT(7)
#define CLK_SELECT_8KHZ				0x00
#define CLK_SELECT_4KHZ				0x40
#define CLK_SELECT_2KHZ				0x80
#define CLK_SELECT_500HZ			0xC0
#define INIT_FAN_CTRL				0xFF
#define MAX_FANS				16

#define CLK_FREQ_4KHZ           (4*1000)
#define CLK_FREQ_8KHZ           (8*1000)
#define CLK_FREQ_16KHZ          (16*1000)
#define CLK_FREQ_200KHZ         (200*1000)
#define CLK_FREQ_8_3_MHZ        (8333*1000)


#define TACH_CLK_FREQ   CLK_FREQ_4KHZ
#define PWM_CLK_FREQ    CLK_FREQ_200KHZ

#define PULSES_PER_REVOLUTION 2

#define FAN_0   (0)
#define FAN_1   (1)
#define FAN_2   (2)
#define FAN_3   (3)
#define FAN_4   (4)
#define FAN_5   (5)
#define FAN_6   (6)
#define FAN_7   (7)
#define FAN_8   (8)
#define FAN_9   (9)
#define FAN_10  (10)
#define FAN_11  (11)
#define FAN_12  (12)
#define FAN_13  (13)
#define FAN_14  (14)
#define FAN_15  (15)

#define PWM_0   (0)
#define PWM_1   (1)
#define PWM_2   (2)
#define PWM_3   (3)
#define PWM_4   (4)
#define PWM_5   (5)
#define PWM_6   (6)
#define PWM_7   (7)

#define TACH_0  (0)
#define TACH_1  (1)
#define TACH_2  (2)
#define TACH_3  (3)
#define TACH_4  (4)
#define TACH_5  (5)
#define TACH_6  (6)
#define TACH_7  (7)
#define TACH_8  (8)
#define TACH_9  (9)
#define TACH_10 (10)
#define TACH_11 (11)
#define TACH_12 (12)
#define TACH_13 (13)
#define TACH_14 (14)
#define TACH_15 (15)






/* ASPEED PILOT PWM & FAN Tach Register Definition */
#define PILOT_PWM_CNTRL_CONF_REG03		0x00
#define PILOT_PWM_CNTRL_CONF_REG47		0x100
#define PILOT_FAN_TACH_MON_CONF_REG07		0x04
#define PILOT_FAN_TACH_MON_CONF_REG815		0x104
#define PILOT_PWM_OTS_CONF_REG03		0x08
#define PILOT_PWM_OTS_CONF_REG47		0x108
#define PILOT_PWM_CNTR_CONF_REG03		0x10
#define PILOT_PWM_CNTR_CONF_REG47		0x110
#define PILOT_PWM_CNTR_VAL_REG0			0x14
#define PILOT_PWM_CNTR_VAL_REG1			0x18
#define PILOT_PWM_CNTR_VAL_REG2			0x1C
#define PILOT_PWM_CNTR_VAL_REG3			0x20
#define PILOT_PWM_CNTR_VAL_REG4			0x114
#define PILOT_PWM_CNTR_VAL_REG5			0x118
#define PILOT_PWM_CNTR_VAL_REG6			0x11C
#define PILOT_PWM_CNTR_VAL_REG7			0x120

#define PILOT_PWM_PS_REG0			0x40
#define PILOT_PWM_PS_REG1			0x48
#define PILOT_PWM_PS_REG2			0x50
#define PILOT_PWM_PS_REG3			0x58
#define PILOT_PWM_PS_REG4			0x140
#define PILOT_PWM_PS_REG5			0x148
#define PILOT_PWM_PS_REG6			0x150
#define PILOT_PWM_PS_REG7			0x158

#define PILOT_PWM_DUTY_REG0			0x44
#define PILOT_PWM_DUTY_REG1			0x4C
#define PILOT_PWM_DUTY_REG2			0x54
#define PILOT_PWM_DUTY_REG3			0x5C
#define PILOT_PWM_DUTY_REG4			0x144
#define PILOT_PWM_DUTY_REG5			0x14C
#define PILOT_PWM_DUTY_REG6			0x154
#define PILOT_PWM_DUTY_REG7			0x15C

#define PILOT_FAN_MON_THR_REG0			0x60
#define PILOT_FAN_MON_THR_REG1			0x6C
#define PILOT_FAN_MON_THR_REG2			0x78
#define PILOT_FAN_MON_THR_REG3			0x84
#define PILOT_FAN_MON_THR_REG4			0x90
#define PILOT_FAN_MON_THR_REG5			0x9C
#define PILOT_FAN_MON_THR_REG6			0xA8
#define PILOT_FAN_MON_THR_REG7			0xB4
#define PILOT_FAN_MON_THR_REG8			0x160
#define PILOT_FAN_MON_THR_REG9			0x16C
#define PILOT_FAN_MON_THR_REG10			0x178
#define PILOT_FAN_MON_THR_REG11			0x184
#define PILOT_FAN_MON_THR_REG12			0x190
#define PILOT_FAN_MON_THR_REG13			0x19C
#define PILOT_FAN_MON_THR_REG14			0x1A8
#define PILOT_FAN_MON_THR_REG15			0x1B4

#define PILOT_FNA_MON_SPEED_REG0		0x64
#define PILOT_FNA_MON_SPEED_REG1		0x70
#define PILOT_FNA_MON_SPEED_REG2		0x7C
#define PILOT_FNA_MON_SPEED_REG3		0x88
#define PILOT_FNA_MON_SPEED_REG4		0x94
#define PILOT_FNA_MON_SPEED_REG5		0xA0
#define PILOT_FNA_MON_SPEED_REG6		0xAC
#define PILOT_FNA_MON_SPEED_REG7		0xB8
#define PILOT_FNA_MON_SPEED_REG8		0x164
#define PILOT_FNA_MON_SPEED_REG9		0x170
#define PILOT_FNA_MON_SPEED_REG10		0x17C
#define PILOT_FNA_MON_SPEED_REG11		0x188
#define PILOT_FNA_MON_SPEED_REG12		0x194
#define PILOT_FNA_MON_SPEED_REG13		0x1A0
#define PILOT_FNA_MON_SPEED_REG14		0x1AC
#define PILOT_FNA_MON_SPEED_REG15		0x1B8

#define PILOT_FAN_MON_CTRL_STS_REG0		0x68
#define PILOT_FAN_MON_CTRL_STS_REG1		0x74
#define PILOT_FAN_MON_CTRL_STS_REG2		0x80
#define PILOT_FAN_MON_CTRL_STS_REG3		0x8C
#define PILOT_FAN_MON_CTRL_STS_REG4		0x98
#define PILOT_FAN_MON_CTRL_STS_REG5		0xA4
#define PILOT_FAN_MON_CTRL_STS_REG6		0xB0
#define PILOT_FAN_MON_CTRL_STS_REG7		0xBC
#define PILOT_FAN_MON_CTRL_STS_REG8		0x168
#define PILOT_FAN_MON_CTRL_STS_REG9		0x174
#define PILOT_FAN_MON_CTRL_STS_REG10		0x180
#define PILOT_FAN_MON_CTRL_STS_REG11		0x18C
#define PILOT_FAN_MON_CTRL_STS_REG12		0x198
#define PILOT_FAN_MON_CTRL_STS_REG13		0x1A4
#define PILOT_FAN_MON_CTRL_STS_REG14		0x1B0
#define PILOT_FAN_MON_CTRL_STS_REG15		0x1BC

#define PILOT_FAN_FILTER_SEL_REG0		0xC0
#define PILOT_FAN_FILTER_SEL_REG1		0xC4
#define PILOT_FAN_FILTER_SEL_REG2		0x1C0
#define PILOT_FAN_FILTER_SEL_REG3		0x1C4

/*PWM Control Configuration Register(PILOT_PWM_CNTRL_CONF_REG)*/
#define PILOT_PWM_CONTRL_04_EN			BIT(0)
#define PILOT_PWM_INVERT_04_EN			BIT(1)
#define PILOT_PWM_CONTRL_15_EN			BIT(2)
#define PILOT_PWM_INVERT_15_EN			BIT(3)
#define PILOT_PWM_CONTRL_26_EN			BIT(4)
#define PILOT_PWM_INVERT_26_EN			BIT(5)
#define PILOT_PWM_CONTRL_37_EN			BIT(6)
#define PILOT_PWM_INVERT_37_EN			BIT(7)

/*PWM Control OTS Configuration Register(PILOT_PWM_OTS_CONF_REG)*/
#define PILOT_FAN0_OTS_EN			BIT(0)
#define PILOT_FAN1_OTS_EN			BIT(1)
#define PILOT_FAN2_OTS_EN			BIT(2)
#define PILOT_FAN3_OTS_EN			BIT(3)
#define PILOT_PWM0_DBY_128_64_EN		BIT(4)
#define PILOT_PWM1_DBY_128_64_EN		BIT(5)
#define PILOT_PWM2_DBY_128_64_EN		BIT(6)
#define PILOT_PWM3_DBY_128_64_EN		BIT(7)

/*Fan Tach Monitor Configuration Register(PILOT_FAN_TACH_MON_CONF_REG)*/
#define PILOT_FAN_MON_08_EN			BIT(0)
#define PILOT_FAN_MON_19_EN			BIT(1)
#define PILOT_FAN_MON_210_EN			BIT(2)
#define PILOT_FAN_MON_311_EN			BIT(3)
#define PILOT_FAN_MON_412_EN			BIT(4)
#define PILOT_FAN_MON_513_EN			BIT(5)
#define PILOT_FAN_MON_614_EN			BIT(6)
#define PILOT_FAN_MON_715_EN			BIT(7)

/*PWM Counter Configuration Register(PILOT_PWM_COUNTER_CONF_REG)*/
#define PILOT_PWM_CNTR_RES_04_EN		BIT(0)
#define PILOT_PWM_CNTR_RES_04_ER		BIT(1)
#define PILOT_PWM_CNTR_RES_15_EN		BIT(2)
#define PILOT_PWM_CNTR_RES_15_ER		BIT(3)
#define PILOT_PWM_CNTR_RES_26_EN		BIT(4)
#define PILOT_PWM_CNTR_RES_26_ER		BIT(5)
#define PILOT_PWM_CNTR_RES_37_EN		BIT(6)
#define PILOT_PWM_CNTR_RES_37_ER		BIT(7)

/*Fan Tach Monitor Control and Status Register(PILOT_FAN_MON_CTRL_STS_REG)*/
#define PILOT_FAN_SPEED_READY			BIT(0)
#define PILOT_FAN_OVER_THRESHOLD		BIT(1)
#define PILOT_FAN_OVERFLOW			BIT(2)
#define PILOT_FAN_ERROR				BIT(3)
#define PILOT_FAN_INTR_EN			BIT(4)
#define PILOT_FAN_FILTER_DIS			BIT(5)
#define PILOT_FAN_CLK_L				BIT(6)
#define PILOT_FAN_CLK_U				BIT(7)

#define PWM_MAX 255

#define MAX_CDEV_NAME_LEN 16

struct aspeed_cooling_device {
	char name[16];
	struct aspeed_pwm_tacho_data *priv;
	struct thermal_cooling_device *tcdev;
	int pwm_port;
	u8 *cooling_levels;
	u8 max_state;
	u8 cur_state;
};

struct aspeed_pwm_tacho_data {
	struct regmap *regmap;
	struct reset_control *rst;
	unsigned long clk_freq;
	bool pwm_present[8];
	bool fan_tach_present[16];
	u8 pwm_port_type[8];
	u8 pwm_port_fan_ctrl[8];
	u8 fan_tach_ch_source[16];
	unsigned int pwm_src_freq;
	struct aspeed_cooling_device *cdev[8];
	const struct attribute_group *groups[3];
};
enum pwm_port { PWM0, PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7 };

struct pwm_port_params {
        u32 pwm_en;
        u32 ctrl_reg;
        u32 type_part1;
        u32 type_part2;
        u32 type_mask;
        u32 duty_ctrl_rise_point;
        u32 duty_ctrl_fall_point;
        u32 duty_ctrl_reg;
        u32 duty_ctrl_rise_fall_mask;
};


/*
struct pwm_port_params {
	u32 pwm_en;
	u32 ctrl_reg;
	u32 duty_ctrl_reg;
};
*/

static const struct pwm_port_params pwm_port_params[] = {
	[PWM0] = {
		.pwm_en = PILOT_PWM_CONTRL_04_EN,
		.ctrl_reg = PILOT_PWM_CNTRL_CONF_REG03,
	},
	[PWM1] = {
		.pwm_en = PILOT_PWM_CONTRL_15_EN,
		.ctrl_reg = PILOT_PWM_CNTRL_CONF_REG03,
	},
	[PWM2] = {
		.pwm_en = PILOT_PWM_CONTRL_26_EN,
		.ctrl_reg = PILOT_PWM_CNTRL_CONF_REG03,
	},
	[PWM3] = {
		.pwm_en = PILOT_PWM_CONTRL_37_EN,
		.ctrl_reg = PILOT_PWM_CNTRL_CONF_REG03,
	},
	[PWM4] = {
		.pwm_en = PILOT_PWM_CONTRL_04_EN,
		.ctrl_reg = PILOT_PWM_CNTRL_CONF_REG47,
	},
	[PWM5] = {
		.pwm_en = PILOT_PWM_CONTRL_15_EN,
		.ctrl_reg = PILOT_PWM_CNTRL_CONF_REG47,
	},
	[PWM6] = {
		.pwm_en = PILOT_PWM_CONTRL_26_EN,
		.ctrl_reg = PILOT_PWM_CNTRL_CONF_REG47,
	},
	[PWM7] = {
		.pwm_en = PILOT_PWM_CONTRL_37_EN,
		.ctrl_reg = PILOT_PWM_CNTRL_CONF_REG47,
	}
};
