/*
 *  (C) Copyright 2019 Vishal C Nigade (vishal.nigade@aspeedtech.com)
 *  Copyright (c) 2019, Aspeed Technologies Inc.
 *  SPDX-License-Identifier:     GPL-2.0+
 */

#include "aspeed-pilot-pwm-tacho.h"
//#define DEBUG

int pwm_to_pwm_mapping[MAX_FANS][2] = {
	{FAN_0,PWM_0},
	{FAN_1,PWM_1},
	{FAN_2,PWM_2},
	{FAN_3,PWM_3},
	{FAN_4,PWM_4},
	{FAN_5,PWM_5},
	{FAN_6,PWM_6},
	{FAN_7,PWM_7},
	{FAN_8,PWM_0},
	{FAN_9,PWM_1},
	{FAN_10,PWM_2},
	{FAN_11,PWM_3},
	{FAN_12,PWM_4},
	{FAN_13,PWM_5},
	{FAN_14,PWM_6},
	{FAN_15,PWM_7}
};

/* FAN number, connected TACHOMETER */
int pwm_to_tach_mapping[MAX_FANS][2] = {
	{FAN_0,TACH_0},
	{FAN_1,TACH_1},
	{FAN_2,TACH_2},
	{FAN_3,TACH_3},
	{FAN_4,TACH_4},
	{FAN_5,TACH_5},
	{FAN_6,TACH_6},
	{FAN_7,TACH_7},
	{FAN_8,TACH_8},
	{FAN_9,TACH_9},
	{FAN_10,TACH_10},
	{FAN_11,TACH_11},
	{FAN_12,TACH_12},
	{FAN_13,TACH_13},
	{FAN_14,TACH_14},
	{FAN_15,TACH_15}
};

int get_pwm_number(unsigned char fan_number)
{
	unsigned char i = 0;

	for(i = 0; i < MAX_FANS; i++)
	{
		if(pwm_to_pwm_mapping[i][0] == fan_number)
			break;
	}
	if(i == MAX_FANS)
	{
		printk("No PWM association found for fan number %d\n",fan_number);
		return -1;
	}
#ifdef DEBUG
	printk("FAN NUMBER : %d PWM NUMNBER: %d\n",pwm_to_pwm_mapping[i][0],pwm_to_pwm_mapping[i][1]);
#endif
	return pwm_to_pwm_mapping[i][1];
}

char get_tach_number(unsigned char fan_number)
{
	unsigned char i = 0;

	for(i = 0; i < MAX_FANS; i++)
	{
		if(pwm_to_tach_mapping[i][0] == fan_number)
			break;
	}
	if(i == MAX_FANS)
	{
		printk("No TACH association found for fan number %d\n",fan_number);
		return -1;
	}
	return pwm_to_tach_mapping[i][1];
}


/* returns previous state of the pwm control - Enabled (1) OR Disabled (0) */
unsigned int enable_disable_pwm_control(unsigned char pwm_num, 
		struct aspeed_pwm_tacho_data *priv, unsigned char enable_disable)
{
	uint32_t ui_offset;
	uint32_t CfgVal = 0;
	uint8_t CfgMask = 0;
	uint8_t PrevVal = 0;

	if (pwm_num >= PWM_OFFSET_WRAP) {
		ui_offset = PILOT_PWM_CNTRL_CONF_REG47;
	}
	else {
		ui_offset = PILOT_PWM_CNTRL_CONF_REG03;
	}

	/* Read pwm cfg regsiter */
	regmap_read(priv->regmap, ui_offset,&CfgVal);
#ifdef DEBUG
	printk("I HAVE ADDED:enable_disable_pwm_control: CFGval %x\n", CfgVal);
#endif

	switch(pwm_num)
	{
		case 0:
			CfgMask = PILOT_PWM_CONTRL_04_EN;
			break;
		case 1:
			CfgMask = PILOT_PWM_CONTRL_15_EN;
			break;
		case 2:
			CfgMask = PILOT_PWM_CONTRL_26_EN;
			break;
		case 3:
			CfgMask = PILOT_PWM_CONTRL_37_EN;
			break;
		case 4:
			CfgMask = PILOT_PWM_CONTRL_04_EN;
			break;
		case 5:
			CfgMask = PILOT_PWM_CONTRL_15_EN;
			break;
		case 6:
			CfgMask = PILOT_PWM_CONTRL_26_EN;
			break;
		case 7:
			CfgMask = PILOT_PWM_CONTRL_37_EN;
			break;
		default:
			printk("Invalid pwm_num\n");
	}

	PrevVal = (CfgVal & CfgMask) ? 1 : 0;

	if(enable_disable)
		CfgVal |= CfgMask;
	else
		CfgVal &= ~CfgMask;

	regmap_write(priv->regmap, ui_offset, CfgVal);

	return PrevVal;
}

/* full register */
void update_prescale_reg(unsigned char pwm_num,unsigned char value, struct aspeed_pwm_tacho_data *priv)
{
	uint32_t ui_offset;
	uint8_t prev_pwm_ctrl_state = 0;

	if (pwm_num >= PWM_OFFSET_WRAP) 
	{
		ui_offset = PILOT_PWM_PS_REG4 + ((pwm_num - PWM_OFFSET_WRAP) * 8);
	}
	else 
	{
		ui_offset = PILOT_PWM_PS_REG0 + (pwm_num * 8);
	}

	/* Disable pwm control first */
	prev_pwm_ctrl_state = enable_disable_pwm_control(pwm_num,priv,0);

	regmap_write(priv->regmap, ui_offset, value);

	/* If previous state was enables, then enable pwm control again */
	if(prev_pwm_ctrl_state)
		enable_disable_pwm_control(pwm_num,priv,1);
}

void update_duty_cycle(unsigned char pwm_num,unsigned char value, struct aspeed_pwm_tacho_data *priv)
{
	uint32_t ui_offset;
	//uint8_t prev_pwm_ctrl_state = 0;

	if (pwm_num >= PWM_OFFSET_WRAP)
	{
		ui_offset = PILOT_PWM_DUTY_REG4 + ((pwm_num - PWM_OFFSET_WRAP) * 8);
	}
	else
	{
		ui_offset = PILOT_PWM_DUTY_REG0 + (pwm_num * 8);
	}
	regmap_write(priv->regmap, ui_offset, value);
}



/* returns previous state of the ft control - Enabled (1) OR Disabled (0) */
unsigned int enable_disable_ft_control(unsigned char ft_num,
		struct aspeed_pwm_tacho_data *priv, unsigned char enable_disable)
{
	uint32_t ui_offset;
	uint32_t CfgVal = 0;
	uint8_t CfgMask = 0;
	uint8_t PrevVal = 0;

	if (ft_num >= FANTACH_OFFSET_WRAP) {
		ui_offset = PILOT_FAN_TACH_MON_CONF_REG815;
	}
	else {
		ui_offset = PILOT_FAN_TACH_MON_CONF_REG07;
	}

	/* Read FT cfg regsiter */
	regmap_read(priv->regmap, ui_offset, &CfgVal);

	switch(ft_num)
	{
		case 0:
			CfgMask = PILOT_FAN_MON_08_EN;
			break;
		case 1:
			CfgMask = PILOT_FAN_MON_19_EN;
			break;
		case 2:
			CfgMask = PILOT_FAN_MON_210_EN;
			break;
		case 3:
			CfgMask = PILOT_FAN_MON_311_EN;
			break;
		case 4:
			CfgMask = PILOT_FAN_MON_412_EN;
			break;
		case 5:
			CfgMask = PILOT_FAN_MON_513_EN;
			break;
		case 6:
			CfgMask = PILOT_FAN_MON_614_EN;
			break;
		case 7:
			CfgMask = PILOT_FAN_MON_715_EN;
			break;
		case 8:
			CfgMask = PILOT_FAN_MON_08_EN;
			break;
		case 9:
			CfgMask = PILOT_FAN_MON_19_EN;
			break;
		case 10:
			CfgMask = PILOT_FAN_MON_210_EN;
			break;
		case 11:
			CfgMask = PILOT_FAN_MON_311_EN;
			break;
		case 12:
			CfgMask = PILOT_FAN_MON_412_EN;
			break;
		case 13:
			CfgMask = PILOT_FAN_MON_513_EN;
			break;
		case 14:
			CfgMask = PILOT_FAN_MON_614_EN;
			break;
		case 15:
			CfgMask = PILOT_FAN_MON_715_EN;
			break;
		default:
			printk("Invalid ft_num\n");
	}

	if (ft_num >= FANTACH_OFFSET_WRAP) {
		PrevVal = (CfgVal & CfgMask) >> (ft_num - FANTACH_OFFSET_WRAP);
	}
	else {
		PrevVal = (CfgVal & CfgMask) >> ft_num;
	}

	if(enable_disable)
		CfgVal |= CfgMask;
	else
		CfgVal &= ~CfgMask;

	regmap_write(priv->regmap, ui_offset, CfgVal);

	return PrevVal;
}

void update_ft_threshold_reg(unsigned char ft_num,unsigned char value, struct aspeed_pwm_tacho_data *priv)
{
	uint32_t ui_offset;
	uint8_t prev_ft_ctrl_state = 0;

	if (ft_num >= FANTACH_OFFSET_WRAP) {
		ui_offset = PILOT_FAN_MON_THR_REG8 + ((ft_num - FANTACH_OFFSET_WRAP) * 0x0C);
	}
	else {
		ui_offset = PILOT_FAN_MON_THR_REG0 + (ft_num * 0x0C);
	}

	/* Disable FT control first */
	prev_ft_ctrl_state = enable_disable_ft_control(ft_num,priv,0);

	regmap_write(priv->regmap, ui_offset, value);

	/* Enable FT control again */
	if(prev_ft_ctrl_state)
		enable_disable_ft_control(ft_num,priv,1);
}

void update_ft_ctrl_status_reg(unsigned char ft_num,unsigned char value, struct aspeed_pwm_tacho_data *priv)
{
	uint32_t ui_offset;
	uint8_t prev_ft_ctrl_state = 0;

	if (ft_num >= FANTACH_OFFSET_WRAP) {
		ui_offset = PILOT_FAN_MON_CTRL_STS_REG8 + ((ft_num - FANTACH_OFFSET_WRAP) * 0x0C);
	}
	else {
		ui_offset = PILOT_FAN_MON_CTRL_STS_REG0 + (ft_num * 0x0C);
	}

	/* Disable FT control first */
	prev_ft_ctrl_state = enable_disable_ft_control(ft_num,priv,0);

	regmap_write(priv->regmap, ui_offset, value);

	/* Enable FT control again */
	if(prev_ft_ctrl_state)
		enable_disable_ft_control(ft_num,priv,1);
}
/*
 * pilot_ii_enableallpwm
 */
int pilot_ii_enableallpwm (struct aspeed_pwm_tacho_data *priv, void* in_data)
{
	unsigned int pwm_num = 0;
	enable_disable_pwm_control(pwm_num++, priv,1);
	enable_disable_pwm_control(pwm_num++, priv,1);
	enable_disable_pwm_control(pwm_num++, priv,1);
	enable_disable_pwm_control(pwm_num++, priv,1);
	enable_disable_pwm_control(pwm_num++, priv,1);
	enable_disable_pwm_control(pwm_num++, priv,1);
	enable_disable_pwm_control(pwm_num++, priv,1);
	enable_disable_pwm_control(pwm_num, priv,1);
	return 0;
}
/*
 * pilot_ii_enablealltach
 */
int pilot_ii_enablealltach (struct aspeed_pwm_tacho_data *priv, void* in_data)
{
	uint8_t i = 0;
	unsigned int temp;

	for(i = TACH_0; i < NUMFT; i++){
		temp = i;
		enable_disable_ft_control((uint8_t)i, priv, 1);
		i = temp;
#ifdef DEBUG
		printk("I HAVE ADDED:pilot_ii_enablealltach:i=%d\n",i);
#endif
	}

	return 0;
}

/*
 * pilot_ii_gettachvalue
 */
int pilot_ii_gettachvalue (struct aspeed_pwm_tacho_data *priv , unsigned int ft_num)
{
	uint32_t ui_FMCSR0OFF_offset;
	uint32_t ui_FMSPR0OFF_offset;
	int tachnumber = ft_num;
	uint32_t CurrentSpeed = 0;
	uint32_t CurrentCtrlStatVal = 0;
	unsigned int retries = 10;
	unsigned int rpmvalue;
	if (tachnumber >= FANTACH_OFFSET_WRAP) {
		ui_FMCSR0OFF_offset = PILOT_FAN_MON_CTRL_STS_REG8 + ((tachnumber - FANTACH_OFFSET_WRAP) * 0x0C);
		ui_FMSPR0OFF_offset = PILOT_FNA_MON_SPEED_REG8 + ((tachnumber - FANTACH_OFFSET_WRAP) * 0x0C);
	}
	else {
		ui_FMCSR0OFF_offset = PILOT_FAN_MON_CTRL_STS_REG0 + (tachnumber * 0x0C);
		ui_FMSPR0OFF_offset = PILOT_FNA_MON_SPEED_REG0 + (tachnumber * 0x0C);
	}

	/* Wait for Speed bit to be ready */
	printk("Waiting for speed ready...");
	while(retries)
	{
		/* Read the current Control/Status register value */
		regmap_read(priv->regmap, ui_FMCSR0OFF_offset, &CurrentCtrlStatVal);
		if(CurrentCtrlStatVal & PILOT_FAN_SPEED_READY)
			break;
		else if(CurrentCtrlStatVal & PILOT_FAN_ERROR)
		{
			regmap_write(priv->regmap, ui_FMCSR0OFF_offset, CurrentCtrlStatVal);
			goto out;
		}
		else if(CurrentCtrlStatVal & PILOT_FAN_OVERFLOW)
		{
			regmap_write(priv->regmap, ui_FMCSR0OFF_offset, CurrentCtrlStatVal);
			goto out;
		}
		else if(CurrentCtrlStatVal & PILOT_FAN_OVER_THRESHOLD)
		{
			regmap_write(priv->regmap, ui_FMCSR0OFF_offset, CurrentCtrlStatVal);
			goto out;
		}
		mdelay(100);
		retries--;
	}
	if(retries == 0)
	{
		printk("ran out of retries in gettachvalue...returning -1\n");
		return -1;
	}

	printk("got it. FMCSR = 0x%02X\n",CurrentCtrlStatVal);
	regmap_read(priv->regmap, ui_FMSPR0OFF_offset, &CurrentSpeed);
	rpmvalue = (60 * TACH_CLK_FREQ)/CurrentSpeed;
	printk("FMSPR = %d, RPM = %d\n",CurrentSpeed, rpmvalue);
	return rpmvalue;

out:	
	printk("Failed to read speed reg:0X%x\n",CurrentCtrlStatVal);
	return -2;
}
