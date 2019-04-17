/*
 *  (C) Copyright 2019 Vishal C Nigade (vishal.nigade@aspeedtech.com)
 *  Copyright (c) 2019, Aspeed Technologies Inc.
 *  SPDX-License-Identifier:     GPL-2.0+
 */
#include "aspeed-pilot-pwm-tacho.h"
//#define DEBUG
extern void update_prescale_reg(unsigned char ,unsigned char, struct aspeed_pwm_tacho_data *);
extern void update_duty_cycle(unsigned char ,unsigned char, struct aspeed_pwm_tacho_data *);
extern void update_ft_threshold_reg(unsigned char ,unsigned char , struct aspeed_pwm_tacho_data *);
extern void update_ft_ctrl_status_reg(unsigned char ,unsigned char , struct aspeed_pwm_tacho_data *);
extern int pilot_ii_enableallpwm (struct aspeed_pwm_tacho_data *, void* );
extern int pilot_ii_enablealltach (struct aspeed_pwm_tacho_data *, void* );
extern int get_pwm_number(unsigned char);
extern char get_tach_number(unsigned char);
extern unsigned int enable_disable_pwm_control(unsigned char pwm_num,
					struct aspeed_pwm_tacho_data *priv,
					unsigned char enable_disable);

extern unsigned int enable_disable_ft_control(unsigned char ft_num,
					struct aspeed_pwm_tacho_data *priv,
					unsigned char enable_disable);

extern int pilot_ii_gettachvalue (struct aspeed_pwm_tacho_data *priv , unsigned int ft_num);
#ifdef SOURCE_UPDATE_ALLOW
struct regmap * syscon_pwm_regmap;

static u32 pilot_read_reg(struct regmap *regm, u32 reg)
{
	unsigned int val = 0;
	int rc = regmap_read(regm, reg, &val);

	if (rc)
		printk("pilot_read_reg Error\n");

	return val;
}

static void pilot_write_reg(struct regmap *regm, u32 data, u32 reg)
{
	int rc = regmap_write(regm, reg, data);

	if (rc)
		printk("pilot_write_reg Error\n");
}

static void update_pwm_source_freq(u32 data){
	u32 value =  pilot_read_reg(syscon_pwm_regmap, 0x94) & ~(0xF);
	pilot_write_reg(syscon_pwm_regmap, value | data, 0x94);
}
#endif
static int regmap_pilot_pwm_tacho_reg_write(void *context, unsigned int reg,
		unsigned int val)
{
	void __iomem *regs = (void __iomem *)context;

	writel(val, regs + reg);
	return 0;
}

static int regmap_pilot_pwm_tacho_reg_read(void *context, unsigned int reg,
					    unsigned int *val)
{
	void __iomem *regs = (void __iomem *)context;

	*val = readl(regs + reg);
	return 0;
}

static const struct regmap_config aspeed_pilot_pwm_tacho_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.reg_write = regmap_pilot_pwm_tacho_reg_write,
	.reg_read = regmap_pilot_pwm_tacho_reg_read,
	.fast_io = true,
};


static void aspeed_set_pwm_port_enable(struct regmap *regmap, u8 pwm_port,
				       bool enable)
{
	regmap_update_bits(regmap, pwm_port_params[pwm_port].ctrl_reg,
			   pwm_port_params[pwm_port].pwm_en,
			   enable ? pwm_port_params[pwm_port].pwm_en : 0);
}

static void aspeed_set_pwm_port_fan_ctrl(struct aspeed_pwm_tacho_data *priv,
					 u8 index, u8 fan_ctrl)
{
	u16 period, dc_time_on;
	/*
	//	period = priv->type_pwm_clock_unit[priv->pwm_port_type[index]];
	period = 0x80;
	period += 1;
	dc_time_on = (fan_ctrl * period) / PWM_MAX;
	 */
	dc_time_on = fan_ctrl;
	update_duty_cycle(index, dc_time_on, priv);
	priv->pwm_port_fan_ctrl[index] = dc_time_on;
	enable_disable_pwm_control(index, priv, true);
}


static int aspeed_get_fan_tach_ch_rpm(struct aspeed_pwm_tacho_data *priv,
		u8 fan_tach_ch)
{
	u32 raw_data, tach_div, clk_source, msec, usec, val;
	u8 fan_tach_ch_source, type, mode, both;
	int ret;

	return pilot_ii_gettachvalue (priv ,fan_tach_ch);
}

static ssize_t set_pwm(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int index = sensor_attr->index;
	int ret;
	struct aspeed_pwm_tacho_data *priv = dev_get_drvdata(dev);
	long fan_ctrl;

	ret = kstrtol(buf, 10, &fan_ctrl);
	if (ret != 0)
		return ret;

	if (fan_ctrl < 0 || fan_ctrl > PWM_MAX)
		return -EINVAL;

	if (priv->pwm_port_fan_ctrl[index] == fan_ctrl)
		return count;

	priv->pwm_port_fan_ctrl[index] = fan_ctrl;
	aspeed_set_pwm_port_fan_ctrl(priv, index, fan_ctrl);

	return count;
}

static ssize_t show_pwm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int index = sensor_attr->index;
	struct aspeed_pwm_tacho_data *priv = dev_get_drvdata(dev);
	
	return sprintf(buf, "%u\n", priv->pwm_port_fan_ctrl[index]);
}

static ssize_t show_rpm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int index = sensor_attr->index;
	int rpm;
	struct aspeed_pwm_tacho_data *priv = dev_get_drvdata(dev);
	if (index < 0 || index > MAX_FANS)
		return -EINVAL;
	rpm = aspeed_get_fan_tach_ch_rpm(priv, index);
	if (rpm < 0)
		return rpm;

	return sprintf(buf, "%d\n", rpm);
}

static umode_t pwm_is_visible(struct kobject *kobj,
                              struct attribute *a, int index)
{
        struct device *dev = container_of(kobj, struct device, kobj);
        struct aspeed_pwm_tacho_data *priv = dev_get_drvdata(dev);
#ifdef DEBUG
	printk("I HAVE ADDED:pwm_is_visible index = %x\n",index);
#endif

        if (!priv->pwm_present[index])
                return 0;
#ifdef DEBUG
	printk("I HAVE ADDED:pwm_is_visible:%x\n",index);
#endif
        return a->mode;
}

static umode_t fan_dev_is_visible(struct kobject *kobj,
                                  struct attribute *a, int index)
{
        struct device *dev = container_of(kobj, struct device, kobj);
        struct aspeed_pwm_tacho_data *priv = dev_get_drvdata(dev);
#ifdef DEBUG
	printk("I HAVE ADDED:fan_dev_is_visible\n");
#endif

        if (!priv->fan_tach_present[index])
                return 0;
#ifdef DEBUG
	printk("I HAVE ADDED:fan_dev_is_visible:%x\n",index);
#endif
        return a->mode;
}


static SENSOR_DEVICE_ATTR(pwm0, 0644,
			show_pwm, set_pwm, 0);
static SENSOR_DEVICE_ATTR(pwm1, 0644,
			show_pwm, set_pwm, 1);
static SENSOR_DEVICE_ATTR(pwm2, 0644,
			show_pwm, set_pwm, 2);
static SENSOR_DEVICE_ATTR(pwm3, 0644,
			show_pwm, set_pwm, 3);
static SENSOR_DEVICE_ATTR(pwm4, 0644,
			show_pwm, set_pwm, 4);
static SENSOR_DEVICE_ATTR(pwm5, 0644,
			show_pwm, set_pwm, 5);
static SENSOR_DEVICE_ATTR(pwm6, 0644,
			show_pwm, set_pwm, 6);
static SENSOR_DEVICE_ATTR(pwm7, 0644,
			show_pwm, set_pwm, 7);
static struct attribute *pwm_dev_attrs[] = {
	&sensor_dev_attr_pwm0.dev_attr.attr,
	&sensor_dev_attr_pwm1.dev_attr.attr,
	&sensor_dev_attr_pwm2.dev_attr.attr,
	&sensor_dev_attr_pwm3.dev_attr.attr,
	&sensor_dev_attr_pwm4.dev_attr.attr,
	&sensor_dev_attr_pwm5.dev_attr.attr,
	&sensor_dev_attr_pwm6.dev_attr.attr,
	&sensor_dev_attr_pwm7.dev_attr.attr,
	NULL,
};

static const struct attribute_group pwm_dev_group = {
	.attrs = pwm_dev_attrs,
	.is_visible = pwm_is_visible,
};

static SENSOR_DEVICE_ATTR(fan0_input, 0444,
		show_rpm, NULL, 0);
static SENSOR_DEVICE_ATTR(fan1_input, 0444,
		show_rpm, NULL, 1);
static SENSOR_DEVICE_ATTR(fan2_input, 0444,
		show_rpm, NULL, 2);
static SENSOR_DEVICE_ATTR(fan3_input, 0444,
		show_rpm, NULL, 3);
static SENSOR_DEVICE_ATTR(fan4_input, 0444,
		show_rpm, NULL, 4);
static SENSOR_DEVICE_ATTR(fan5_input, 0444,
		show_rpm, NULL, 5);
static SENSOR_DEVICE_ATTR(fan6_input, 0444,
		show_rpm, NULL, 6);
static SENSOR_DEVICE_ATTR(fan7_input, 0444,
		show_rpm, NULL, 7);
static SENSOR_DEVICE_ATTR(fan8_input, 0444,
		show_rpm, NULL, 8);
static SENSOR_DEVICE_ATTR(fan9_input, 0444,
		show_rpm, NULL, 9);
static SENSOR_DEVICE_ATTR(fan10_input, 0444,
		show_rpm, NULL, 10);
static SENSOR_DEVICE_ATTR(fan11_input, 0444,
		show_rpm, NULL, 11);
static SENSOR_DEVICE_ATTR(fan12_input, 0444,
		show_rpm, NULL, 12);
static SENSOR_DEVICE_ATTR(fan13_input, 0444,
		show_rpm, NULL, 13);
static SENSOR_DEVICE_ATTR(fan14_input, 0444,
		show_rpm, NULL, 14);
static SENSOR_DEVICE_ATTR(fan15_input, 0444,
		show_rpm, NULL, 15);
static struct attribute *fan_dev_attrs[] = {
	&sensor_dev_attr_fan0_input.dev_attr.attr,
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_fan3_input.dev_attr.attr,
	&sensor_dev_attr_fan4_input.dev_attr.attr,
	&sensor_dev_attr_fan5_input.dev_attr.attr,
	&sensor_dev_attr_fan6_input.dev_attr.attr,
	&sensor_dev_attr_fan7_input.dev_attr.attr,
	&sensor_dev_attr_fan8_input.dev_attr.attr,
	&sensor_dev_attr_fan9_input.dev_attr.attr,
	&sensor_dev_attr_fan10_input.dev_attr.attr,
	&sensor_dev_attr_fan11_input.dev_attr.attr,
	&sensor_dev_attr_fan12_input.dev_attr.attr,
	&sensor_dev_attr_fan13_input.dev_attr.attr,
	&sensor_dev_attr_fan14_input.dev_attr.attr,
	&sensor_dev_attr_fan15_input.dev_attr.attr,
	NULL
};

static const struct attribute_group fan_dev_group = {
	.attrs = fan_dev_attrs,
	.is_visible = fan_dev_is_visible,
};

/*
 * The clock type is type M :
 * The PWM frequency = 24MHz / (type M clock division L bit *
 * type M clock division H bit * (type M PWM period bit + 1))
 */

static void aspeed_init_vars(struct aspeed_pwm_tacho_data *priv){
	u32 index = 0;
	for (index = 0;index < MAX_FANS; index++){
		priv->fan_tach_present[index] = false;
		priv->fan_tach_ch_source[index] = 0;
	}
	for (index = 0;index < NUMPWM; index++){
		priv->pwm_present[index] = false;
		priv->pwm_port_fan_ctrl[index] = INIT_FAN_CTRL;
	}

}

static void aspeed_create_fan_tach_channel(struct aspeed_pwm_tacho_data *priv,
					   u8 *fan_tach_ch,
					   int count,
					   u8 pwm_source)
{
	u8 val, index;

	for (val = 0; val < count; val++) {
		index = fan_tach_ch[val];
		enable_disable_ft_control(index, priv, true);
		priv->fan_tach_present[index] = true;
		priv->fan_tach_ch_source[index] = pwm_source;
		priv->pwm_present[pwm_source] = true;
		priv->pwm_port_fan_ctrl[pwm_source] = INIT_FAN_CTRL;
		update_duty_cycle(pwm_source, INIT_FAN_CTRL, priv);
		enable_disable_pwm_control(pwm_source, priv, true);
	}
}

static int
aspeed_pwm_cz_get_max_state(struct thermal_cooling_device *tcdev,
			    unsigned long *state)
{
	struct aspeed_cooling_device *cdev = tcdev->devdata;

	*state = cdev->max_state;

	return 0;
}

static int
aspeed_pwm_cz_get_cur_state(struct thermal_cooling_device *tcdev,
			    unsigned long *state)
{
	struct aspeed_cooling_device *cdev = tcdev->devdata;

	*state = cdev->cur_state;

	return 0;
}

static int
aspeed_pwm_cz_set_cur_state(struct thermal_cooling_device *tcdev,
			    unsigned long state)
{
	struct aspeed_cooling_device *cdev = tcdev->devdata;

	if (state > cdev->max_state)
		return -EINVAL;

	cdev->cur_state = state;
	cdev->priv->pwm_port_fan_ctrl[cdev->pwm_port] =
					cdev->cooling_levels[cdev->cur_state];
	aspeed_set_pwm_port_fan_ctrl(cdev->priv, cdev->pwm_port,
				     cdev->cooling_levels[cdev->cur_state]);

	return 0;
}

static const struct thermal_cooling_device_ops aspeed_pwm_cool_ops = {
	.get_max_state = aspeed_pwm_cz_get_max_state,
	.get_cur_state = aspeed_pwm_cz_get_cur_state,
	.set_cur_state = aspeed_pwm_cz_set_cur_state,
};

static int aspeed_create_pwm_cooling(struct device *dev,
				     struct device_node *child,
				     struct aspeed_pwm_tacho_data *priv,
				     u32 pwm_port, u8 num_levels)
{
	int ret;
	struct aspeed_cooling_device *cdev;

	cdev = devm_kzalloc(dev, sizeof(*cdev), GFP_KERNEL);

	if (!cdev)
		return -ENOMEM;

	cdev->cooling_levels = devm_kzalloc(dev, num_levels, GFP_KERNEL);
	if (!cdev->cooling_levels)
		return -ENOMEM;

	cdev->max_state = num_levels - 1;
	ret = of_property_read_u8_array(child, "cooling-levels",
					cdev->cooling_levels,
					num_levels);
	if (ret) {
		dev_err(dev, "Property 'cooling-levels' cannot be read.\n");
		return ret;
	}
	snprintf(cdev->name, MAX_CDEV_NAME_LEN, "%s%d", child->name, pwm_port);

	cdev->tcdev = thermal_of_cooling_device_register(child,
							 cdev->name,
							 cdev,
							 &aspeed_pwm_cool_ops);
	if (IS_ERR(cdev->tcdev))
		return PTR_ERR(cdev->tcdev);

	cdev->priv = priv;
	cdev->pwm_port = pwm_port;

	priv->cdev[pwm_port] = cdev;

	return 0;
}
//#define DEBUG
static int aspeed_create_fan(struct device *dev,
			     struct device_node *child,
			     struct aspeed_pwm_tacho_data *priv)
{
	u8 *fan_tach_ch;
	volatile u32 pwm_port,val,prescale_val;
	int ret, count;

	ret = of_property_read_u32(child, "reg", &pwm_port);
#ifdef DEBUG
	printk("aspeed creat fan\n");
	printk("I HAVE ADDED:aspeed_create_fan: reg = %d\n",pwm_port);
#endif
	if (ret)
		return ret;
	if (!of_property_read_u32(child, "aspeed,pwm-clk-divider", &val)) {
		if (of_property_read_u32(child, "aspeed,pwm-prescaler", &prescale_val)) {
				prescale_val = 0;
#ifdef DEBUG
	printk("I HAVE ADDED:aspeed_create_fan: aspeed,pwm-prescaler not found = %x \n",prescale_val);
#endif
		}
		if (prescale_val > 126 || prescale_val < 0){
			prescale_val = 126;
		}
#ifdef DEBUG
	printk("I HAVE ADDED:aspeed_create_fan: aspeed,pwm-prescaler = %x %d\n",prescale_val, val);
#endif
		switch(val){
			case 8330000:
				val = prescale_val;
				update_prescale_reg(pwm_port, (char)val, priv);
				break;
			case 200000:
				val = SELECT_200KHZ_CLK | prescale_val;
				update_prescale_reg(pwm_port, (char)val, priv);
				break;
			default:
				val = prescale_val;
				update_prescale_reg(pwm_port, (char)val, priv);
				break;
		}
	}

#ifdef DEBUG
	printk("I HAVE ADDED:aspeed_create_fan: aspeed,pwm-prescaler_val = %d\n",val);
#endif

	if (!of_device_is_compatible(child, "fan-tach-ch")){
		if (of_device_is_compatible(child, "pwm-stand-alone"))
			goto set_pwm_alone;
		return -EINVAL;
	}


	count = of_property_count_u8_elems(child, "aspeed,fan-tach-ch");
#ifdef DEBUG
	printk("I HAVE ADDED:aspeed_create_fan: count  = %d\n", count);
#endif


	if (count < 1)
		return -EINVAL;
	fan_tach_ch = devm_kzalloc(dev, sizeof(*fan_tach_ch) * count,
				   GFP_KERNEL);
	if (!fan_tach_ch)
		return -ENOMEM;
	ret = of_property_read_u8_array(child, "aspeed,fan-tach-ch",
					fan_tach_ch, count);
	if (ret)
		return ret;
#ifdef DEBUG
	printk("I HAVE ADDED:aspeed_create_fan: fantachch  = %d\n", fan_tach_ch[0]);
#endif

	ret = of_property_count_u8_elems(child, "cooling-levels");
	

#ifdef DEBUG
	printk("I HAVE ADDED:aspeed_create_fan: cooling levels = %d\n", ret);
#endif
//	pwm_port = get_pwm_number(fan_tach_ch[0]);
		
	if (ret > 0) {
		ret = aspeed_create_pwm_cooling(dev, child, priv, pwm_port,
						ret);
		if (ret)
			return ret;
	}

	aspeed_create_fan_tach_channel(priv, fan_tach_ch, count, pwm_port);

	printk("Fan@%d attahced to Pwm@%d\n",fan_tach_ch[0],pwm_port);
	
	return 0;

set_pwm_alone:
	priv->pwm_present[pwm_port] = true;
	priv->pwm_port_fan_ctrl[pwm_port] = INIT_FAN_CTRL;
	update_duty_cycle(pwm_port, INIT_FAN_CTRL, priv);
	enable_disable_pwm_control(pwm_port, priv, true);
	printk("Pwm@%d is in Stand Alone Mode\n",pwm_port,priv->pwm_present[pwm_port]);
	return 0;

}
//#undef DEBUG
static void init_pwmtach(struct aspeed_pwm_tacho_data *priv)
{
        uint32_t ui_offset;
        volatile uint32_t  Addr;
        volatile uint8_t  Value;
        uint8_t i = 0;
        void *dummy_indata = NULL;


        /*********** Initialize PWM registers ************/

        /* Initialize the PILOT_PWM_OTS_CONF_REG03 resigter to have the fixed 256 divisor set for all PWMs */
        Value = (uint8_t) ~PILOT_FAN0_OTS_EN &~PILOT_FAN1_OTS_EN &~PILOT_FAN2_OTS_EN &~PILOT_FAN3_OTS_EN
			&~PILOT_PWM0_DBY_128_64_EN &~PILOT_PWM1_DBY_128_64_EN &~PILOT_PWM2_DBY_128_64_EN
			&~PILOT_PWM3_DBY_128_64_EN;

	regmap_write(priv->regmap, PILOT_PWM_OTS_CONF_REG03, Value);
#ifdef DEBUG
        printk("init_pwmtach\n");
	printk("I HAVE ADDED:init_pwmtach:1\n");
#endif
        /* Initialize the PILOT_PWM_OTS_CONF_REG47 resigter to have the fixed 256 divisor set for all PWMs */
        Value = (uint8_t) ~PILOT_FAN0_OTS_EN &~PILOT_FAN1_OTS_EN &~PILOT_FAN2_OTS_EN &~PILOT_FAN3_OTS_EN
			&~PILOT_PWM0_DBY_128_64_EN &~PILOT_PWM1_DBY_128_64_EN &~PILOT_PWM2_DBY_128_64_EN
			&~PILOT_PWM3_DBY_128_64_EN;
	
	regmap_write(priv->regmap, PILOT_PWM_OTS_CONF_REG47, Value);
#ifdef DEBUG
	printk("I HAVE ADDED:init_pwmtach:2\n");
#endif

        /* Set Duty Cycle register to 0x80 to reflect 50% duty cycle for all PWMs */
        for (i = 0; i < NUMPWM; i++)
        {
                if (i >= PWM_OFFSET_WRAP) {
                        ui_offset = PILOT_PWM_DUTY_REG4 + ((i - PWM_OFFSET_WRAP) * 8);
                }
                else {
                        ui_offset = PILOT_PWM_DUTY_REG0 + (i * 8);
                }

                Value = 0x80;
		regmap_write(priv->regmap, ui_offset, Value);
        }

#ifdef DEBUG
	printk("I HAVE ADDED:init_pwmtach:3\n");
#endif
        /* Set all PILOT_PWM_CNTR_VAL_REG registers to value 0 as we don't want to use counter resolution value */
        for (i = 0; i < NUMPWM; i++)
        {
                if (i >= PWM_OFFSET_WRAP) {
                        ui_offset = PILOT_PWM_CNTR_VAL_REG4 + ((i - PWM_OFFSET_WRAP) * 4);
                }
                else {
                        ui_offset = PILOT_PWM_CNTR_VAL_REG0 + (i * 4);
                }

                Value = 0x00;
		regmap_write(priv->regmap, ui_offset, Value);
        }
#ifdef DEBUG
	printk("I HAVE ADDED:init_pwmtach:4\n");
#endif

        /* Set bits for all PWMs in the PILOT_PWM_CNTR_CONF_REG03 to value 0 to not use programmable counter resolution */
        Value = (uint8_t) ~PILOT_PWM_CNTR_RES_04_EN & ~PILOT_PWM_CNTR_RES_04_ER & ~PILOT_PWM_CNTR_RES_15_EN 
		& ~PILOT_PWM_CNTR_RES_15_ER & ~PILOT_PWM_CNTR_RES_26_EN & ~PILOT_PWM_CNTR_RES_26_ER 
		& ~PILOT_PWM_CNTR_RES_37_EN & ~PILOT_PWM_CNTR_RES_37_ER;
		
	regmap_write(priv->regmap, PILOT_PWM_CNTR_CONF_REG03, Value);

#ifdef DEBUG
	printk("I HAVE ADDED:init_pwmtach:5\n");
#endif
        /* Set bits for all PWMs in the PILOT_PWM_CNTR_CONF_REG47 to value 0 to not use programmable counter resolution */
        Value = (uint8_t) ~PILOT_PWM_CNTR_RES_04_EN & ~PILOT_PWM_CNTR_RES_04_ER & ~PILOT_PWM_CNTR_RES_15_EN 
		& ~PILOT_PWM_CNTR_RES_15_ER & ~PILOT_PWM_CNTR_RES_26_EN & ~PILOT_PWM_CNTR_RES_26_ER 
		& ~PILOT_PWM_CNTR_RES_37_EN & ~PILOT_PWM_CNTR_RES_37_ER;
		
	regmap_write(priv->regmap, PILOT_PWM_CNTR_CONF_REG47, Value);

#ifdef DEBUG
	printk("I HAVE ADDED:init_pwmtach:6\n");
#endif

        /* Set bits for all PWMs in the PILOT_PWM_CNTRL_CONF_REG03 to value 0 to not use PWM invert */
        Value = (uint8_t) ~PILOT_PWM_CONTRL_04_EN &~PILOT_PWM_INVERT_04_EN &~PILOT_PWM_CONTRL_15_EN
		&~PILOT_PWM_INVERT_15_EN &~PILOT_PWM_CONTRL_26_EN &~PILOT_PWM_INVERT_26_EN
		&~PILOT_PWM_CONTRL_37_EN &~PILOT_PWM_INVERT_37_EN;
	
	regmap_write(priv->regmap, PILOT_PWM_CNTRL_CONF_REG03, Value);

#ifdef DEBUG
	printk("I HAVE ADDED:init_pwmtach:7\n");
#endif
        /* Set bits for all PWMs in the PILOT_PWM_CNTRL_CONF_REG47 to value 0 to not use PWM invert */
        Value = (uint8_t) ~PILOT_PWM_CONTRL_04_EN &~PILOT_PWM_INVERT_04_EN &~PILOT_PWM_CONTRL_15_EN
		&~PILOT_PWM_INVERT_15_EN &~PILOT_PWM_CONTRL_26_EN &~PILOT_PWM_INVERT_26_EN
		&~PILOT_PWM_CONTRL_37_EN &~PILOT_PWM_INVERT_37_EN;
	
	regmap_write(priv->regmap, PILOT_PWM_CNTRL_CONF_REG47, Value);

#ifdef DEBUG
	printk("I HAVE ADDED:init_pwmtach:8\n");
#endif
        /* Write PreScale register for all PWMs to set value as 0x07 */
        /* Check the required condition as done in sample code */
        for(i = PWM_0; i < NUMPWM; i++)
        {
                Value = SELECT_200KHZ_CLK & 0x07;
                update_prescale_reg(i,Value,priv);
        }


#ifdef DEBUG
	printk("I HAVE ADDED:init_pwmtach:9\n");
#endif
        /*********** Initialize FAN TACH registers ************/

        /* Set the threshold as following */
        /* Threshold_Value = (60 * TACH_Clk_Frequency)/Mininum_desired_RPM */
        for(i = TACH_0; i < NUMFT; i++)
        {
                Value = 0xF0; //Clk Freq = 4KHz, Min RPM = 1000
                update_ft_threshold_reg(i,Value,priv);
        }

#ifdef DEBUG
	printk("I HAVE ADDED:init_pwmtach:10\n");
#endif
        /* Set all PILOT_FAN_MON_CTRL_STS_REG registers to chose 4KHz clock, filter disable, interrupt enable */
        for(i = TACH_0; i < NUMFT; i++)
        {
                Value = CLK_SELECT_4KHZ|PILOT_FAN_FILTER_DIS;
                update_ft_ctrl_status_reg(i,Value,priv);
        }

#ifdef DEBUG
	printk("I HAVE ADDED:init_pwmtach:11\n");
#endif
        /* Set all filter select registers to value 0 */
        Value = 0x00;
        regmap_write(priv->regmap, PILOT_FAN_FILTER_SEL_REG0, Value);
        regmap_write(priv->regmap, PILOT_FAN_FILTER_SEL_REG1, Value);

        /* Set all filter select registers _2 to value 0 */
        Value = 0x00;
        regmap_write(priv->regmap, PILOT_FAN_FILTER_SEL_REG2, Value);
        regmap_write(priv->regmap, PILOT_FAN_FILTER_SEL_REG3, Value);

#if 0
#ifdef DEBUG
	printk("I HAVE ADDED:init_pwmtach:12\n");
#endif
        /* Let us just enable all fans control and TACH read by default */
        pilot_ii_enableallpwm(priv,dummy_indata);

#ifdef DEBUG
	printk("I HAVE ADDED:init_pwmtach:1333\n");
#endif
        pilot_ii_enablealltach(priv,dummy_indata);
#endif
        return;
}


static void aspeed_pwm_tacho_remove(void *data)
{
	struct aspeed_pwm_tacho_data *priv = data;

	reset_control_assert(priv->rst);
}
//#define DEBUG
static int aspeed_pilot_pwm_tacho_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np, *child;
	struct aspeed_pwm_tacho_data *priv;
	struct device_node *np1 = pdev->dev.of_node;
	void __iomem *regs;
	struct resource *res;
	struct device *hwmon;
	struct clk *clk;
	int ret;

	np = dev->of_node;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;
#ifdef DEBUG
	printk("I HAVE ADDED:aspeed_pilot_pwm_tacho_probe:platform_get_resource\n");
#endif
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
#ifdef DEBUG
	printk("I HAVE ADDED:aspeed_pilot_pwm_tacho_probe:devm_ioremap_resource\n");
#endif
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
#ifdef DEBUG
	printk("I HAVE ADDED:aspeed_pilot_pwm_tacho_probe:devm_kzalloc\n");
#endif
	priv->regmap = devm_regmap_init(dev, NULL, (__force void *)regs,
			&aspeed_pilot_pwm_tacho_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);
#ifdef DEBUG
	printk("I HAVE ADDED:aspeed_pilot_pwm_tacho_probe:devm_regmap_init\n");
#endif

	ret = of_property_read_u32(pdev->dev.of_node,
                        "pwm-source-freq", &priv->pwm_src_freq);	
	 if (ret < 0) {
                dev_err(&pdev->dev,
                                "Could not read pwm-source-frequency property\n");
                priv->pwm_src_freq = 50000000;
        }

#ifdef SOURCE_UPDATE_ALLOW
	syscon_pwm_regmap = syscon_regmap_lookup_by_phandle(np1,
					 "syscon");
	if (IS_ERR(syscon_pwm_regmap)) {
                dev_err(&pdev->dev, "Couldn't get regmap\n");
                return -ENODEV;
        }
	switch(priv->pwm_src_freq){
		case 50000000:
			update_pwm_source_freq(0x2);
			break;
		case 25000000:
			update_pwm_source_freq(0x4);
			break;
		default:
			update_pwm_source_freq(0x2);
			break;
	}
	udelay(10);
	printk("pwm-source-freq = %d\n",priv->pwm_src_freq);
#endif

#if 0
	priv->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(priv->rst)) {
		dev_err(dev,
				"missing or invalid reset controller device tree entry");
		return PTR_ERR(priv->rst);
	}
	reset_control_deassert(priv->rst);

	ret = devm_add_action_or_reset(dev, aspeed_pwm_tacho_remove, priv);
	if (ret)
		return ret;

	regmap_write(priv->regmap, ASPEED_PTCR_TACH_SOURCE, 0);
	regmap_write(priv->regmap, ASPEED_PTCR_TACH_SOURCE_EXT, 0);
#endif
#if 0
	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk))
		return -ENODEV;
	priv->clk_freq = clk_get_rate(clk);
#ifdef DEBUG
	printk("I HAVE ADDED:aspeed_pilot_pwm_tacho_probe:devm_clk_get\n");
#endif
#endif
#if 0
	aspeed_set_clock_enable(priv->regmap, true);
	aspeed_set_clock_source(priv->regmap, 0);

	aspeed_create_type(priv);
#endif
	aspeed_init_vars(priv);
	init_pwmtach(priv);
#ifdef DEBUG
	printk("I HAVE ADDED:aspeed_pilot_pwm_tacho_probe:init_pwmtach\n");
#endif

	for_each_child_of_node(np, child) {
		ret = aspeed_create_fan(dev, child, priv);
		if (ret) {
			of_node_put(child);
			return ret;
		}
	}
#ifdef DEBUG
	printk("I HAVE ADDED:aspeed_pilot_pwm_tacho_probe:for_each_child_of_node\n");
#endif
	printk("Aspeed Pilot Pwm-Fantach Driver initialized\n");
	priv->groups[0] = &pwm_dev_group;
	priv->groups[1] = &fan_dev_group;
	priv->groups[2] = NULL;
	hwmon = devm_hwmon_device_register_with_groups(dev,
						       "aspeed_pilot_pwm_tacho",
						       priv, priv->groups);
	
	return PTR_ERR_OR_ZERO(hwmon);
}

static const struct of_device_id of_pwm_tacho_match_table[] = {
	{ .compatible = "aspeed,pilot-pwm-tacho", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_tacho_match_table);

static struct platform_driver aspeed_pilot_pwm_tacho_driver = {
	.probe		= aspeed_pilot_pwm_tacho_probe,
	.driver		= {
		.name	= "aspeed_pilot_pwm_tacho",
		.of_match_table = of_pwm_tacho_match_table,
	},
};

module_platform_driver(aspeed_pilot_pwm_tacho_driver);

MODULE_AUTHOR("Vishal C Nigade <vishal.nigade@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED PWM and Fan Tacho device driver");
MODULE_LICENSE("GPL");
