/*
 * ast_pwm.c - PWM driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
/*********************************************************************************************/
/* PWM Register */
// 0x1E786000
#define PWM_GENERAL_0		0x00	// [7:0]- enable PWM clock
#define PWM_GENERAL_1		0x04	// [7:0]- PWM_GENERAL_0 selected, [15:8]- duty cycle selected
#define PWM_GENERAL_2		0x08
#define PWM_GENERAL_3		0x0C	// [0]- fire PWM_GENERAL_0 & PWM_GENERAL_1, [1]- fire duty cycle

#define PWM1_STATUS			0x10
#define PWM1_DIVISION		0x14	// [19:16]- division_H, 1, 2, 4,..., 32768, [15:0] division_L, 1, 2, 3, 4,..,65536(0xffff)
#define PWM1_DUTY			0x18	// [31:24]- loop, [23:16]-period, [15:8]-falling, [7:0]-rising
#define PWM1_CONTROL		0x1C	// [9]- interrupt status and clear, [8]- enable interrpt, [4:]- enable loop end, [3]- output PWM Level, [2]- inverse, [1]- enable open-drain, [0]- enable pwm pin

#define PWM2_STATUS			0x20
#define PWM2_DIVISION		0x24
#define PWM2_DUTY			0x28
#define PWM2_CONTROL		0x2C

#define PWM3_STATUS			0x30
#define PWM3_DIVISION		0x34
#define PWM3_DUTY			0x38
#define PWM3_CONTROL		0x3C

#define PWM4_STATUS			0x40
#define PWM4_DIVISION		0x44
#define PWM4_DUTY			0x48
#define PWM4_CONTROL		0x4C

#define PWM5_STATUS			0x50
#define PWM5_DIVISION		0x54
#define PWM5_DUTY			0x58
#define PWM5_CONTROL		0x5C

#define PWM6_STATUS			0x60
#define PWM6_DIVISION		0x64
#define PWM6_DUTY			0x68
#define PWM6_CONTROL		0x6C

#define PWM7_STATUS			0x70
#define PWM7_DIVISION		0x74
#define PWM7_DUTY			0x78
#define PWM7_CONTROL		0x7C

#define PWM8_STATUS			0x80
#define PWM8_DIVISION		0x84
#define PWM8_DUTY			0x88
#define PWM8_CONTROL		0x8C

#define PWM_DIVISION		(0x0)	// [19:16]- division_H, 1, 2, 4,..., 32768, [15:0] division_L, 1, 2, 3, 4,..,65536(0xffff)
#define PWM_DUTY			(0x4)	// [31:16]- falling point, [15:0]- rising point, period >= falling, period >= rising
#define PWM_PERIOD			(0x8)	// [31:16]- loop, [15:0]- period
#define PWM_CONTROL			(0xC)	// [9]- interrupt status and clear, [8]- enable interrpt, [4]- enable loop end, [3]- output PWM Level, [2]- inverse, [1]- enable open-drain, [0]- enable pwm pin
#define PWM_CHANNEL_SIZE	0x10

/**********************************************************************************************************************/
struct ast_pwm_data {
	struct device			*hwmon_dev;
	void __iomem			*reg_base;			/* virtual */
	u32 					pwm_unit_frequency[8];	
	u8						pwm_duty[8];
	u8						pwm_loop[8];
	int 					irq;				
};

struct ast_pwm_data *ast_pwm;
/**********************************************************************************************************************/
static inline void
ast_pwm_write(struct ast_pwm_data *ast_pwm, u32 val, u32 reg)
{
//	printk("write offset: %x, val: %x \n",reg,val);
	writel(val, ast_pwm->reg_base+ reg);
}

static inline u32
ast_pwm_read(struct ast_pwm_data *ast_pwm, u32 reg)
{
	u32 val = readl(ast_pwm->reg_base + reg);
//	printk("read offset: %x, val: %x \n",reg,val);
	return val;
}
/**********************************************************************************************************************/
void ast_set_pwm_en(struct ast_pwm_data *ast_pwm, u8 pwm_ch, u8 enable)
{
	u32 pwm_sts = ast_pwm_read(ast_pwm, PWM1_STATUS + (pwm_ch * PWM_CHANNEL_SIZE));
	if(enable) {	
		ast_pwm_write(ast_pwm, pwm_sts | 0x1, PWM1_STATUS + (pwm_ch * PWM_CHANNEL_SIZE)); 
		ast_pwm_write(ast_pwm, ast_pwm_read(ast_pwm, PWM_GENERAL_0) | (1 << pwm_ch), PWM_GENERAL_0); 
	} else {
		ast_pwm_write(ast_pwm, pwm_sts & ~ 0x1, PWM1_STATUS + (pwm_ch * PWM_CHANNEL_SIZE)); 
		ast_pwm_write(ast_pwm, ast_pwm_read(ast_pwm, PWM_GENERAL_0) & ~(1 << pwm_ch), PWM_GENERAL_0); 
		//TODO fire
		ast_pwm_write(ast_pwm, 1, PWM_GENERAL_3); 
	}
}

static u8 
ast_get_pwm_en(struct ast_pwm_data *ast_pwm, u8 pwm_ch)
{
	if(ast_pwm_read(ast_pwm, PWM_GENERAL_0) & (1 << pwm_ch))
		return 1;
	else
		return 0;
}

static void
ast_set_pwm_frequency(struct ast_pwm_data *ast_pwm, u8 pwm_ch, u32 freq)
{
	u32 pwm_addr = PWM1_STATUS + pwm_ch * PWM_CHANNEL_SIZE + PWM_DIVISION;
	u32 base_clk_divider;
	u32 h_parameter;
	u32 l_parameter;
	u32 found = 0;
	u8 i = 0;
	
	u32 base_clk = 50000000;	// temp for FPGA// 24000000
	
	base_clk_divider = base_clk / (100*freq);
	
	for (h_parameter = 1; h_parameter < 32768; h_parameter *=2)
	{
		if (found == 1)
			break;
		
		i++;
		
		for (l_parameter = 1; l_parameter < 65536; l_parameter ++)
		{
			if (h_parameter * l_parameter >= base_clk_divider)
			{
				found = 1;
				break;
			}
		}
	}
	
	ast_pwm->pwm_unit_frequency[pwm_ch] = base_clk / h_parameter / l_parameter;
	printk ("base: %dHz, h_param: %d, l_param: %d, unit: %dHz\n", base_clk, h_parameter, l_parameter, ast_pwm->pwm_unit_frequency[pwm_ch]);
	l_parameter -= 1;
	h_parameter = i - 1;
 
	ast_pwm_write(ast_pwm, (ast_pwm_read(ast_pwm, pwm_addr) & 0xfff00000) | (h_parameter << 16) | l_parameter, pwm_addr); 
}

void ast_set_pwm_duty(struct ast_pwm_data *ast_pwm, u8 pwm_ch, u8 duty)
{
	u32 pwm_base = PWM1_STATUS + pwm_ch * PWM_CHANNEL_SIZE;
	u8 rising = 0, falling = duty, period = 99;

	ast_pwm->pwm_duty[pwm_ch] = duty;
	// period >= rising, period >= falling
	if (rising > period || falling > period)
		printk("[PWM] wrong setting: ch=%d, r=%d, f=%d, p=%d\n", pwm_ch, rising, falling, period);
	
	ast_pwm_write(ast_pwm, ast_pwm_read(ast_pwm, pwm_base + PWM_DUTY) | (falling << 16) | rising, pwm_base + PWM_DUTY); 

	//OUT32(pwm_base + PWM_PERIOD, (IN32(pwm_base + PWM_PERIOD) & 0xffff0000) | period);
	ast_pwm_write(ast_pwm, (ast_pwm_read(ast_pwm, pwm_base + PWM_PERIOD) & 0xffff0000) | period, pwm_base + PWM_PERIOD); 

	//OUT32(PWM_GENERAL_1, IN32(PWM_GENERAL_1) | (pwm_ch << 8));
	ast_pwm_write(ast_pwm, ast_pwm_read(ast_pwm, PWM_GENERAL_1) | (pwm_ch << 8), PWM_GENERAL_1); 

	//OUT32(PWM_GENERAL_3, IN32(PWM_GENERAL_3) | 0x2);
	ast_pwm_write(ast_pwm, ast_pwm_read(ast_pwm, PWM_GENERAL_3) | 0x2, PWM_GENERAL_3); 
}

void ast_set_pwm_loop_count(struct ast_pwm_data *ast_pwm, u8 pwm_ch, u8 loop_cnt)
{
	u32 pwm_addr1 = PWM1_STATUS + pwm_ch * PWM_CHANNEL_SIZE + PWM_PERIOD;
	u32 pwm_addr2 = PWM1_STATUS + pwm_ch * PWM_CHANNEL_SIZE + PWM_CONTROL;

	ast_pwm->pwm_loop[pwm_ch] = loop_cnt;
	if (loop_cnt > 0)
	{
//		OUT32(pwm_addr1, (IN32(pwm_addr1) & 0xffff) | ((loop_cnt-1) << 16));
		ast_pwm_write(ast_pwm, (ast_pwm_read(ast_pwm, pwm_addr1) & 0xffff)| ((loop_cnt-1) << 16), pwm_addr1); 
		//OUT32(pwm_addr2, IN32(pwm_addr2) | (1 << 4));
		ast_pwm_write(ast_pwm, ast_pwm_read(ast_pwm, pwm_addr2) | (1 << 4), pwm_addr2); 
	} else {
		//OUT32(pwm_addr1, IN32(pwm_addr1) & 0xffff);
		ast_pwm_write(ast_pwm, (ast_pwm_read(ast_pwm, pwm_addr1) & 0xffff), pwm_addr1); 
		//OUT32(pwm_addr2, IN32(pwm_addr2) & ~(1 << 4));
		ast_pwm_write(ast_pwm, ast_pwm_read(ast_pwm, pwm_addr2) & ~(1 << 4), pwm_addr2); 
	}
}

void ast_set_pwm_update(struct ast_pwm_data *ast_pwm, u8 pwm_ch, u8 update)
{
	//OUT32(PWM_GENERAL_1, IN32(PWM_GENERAL_1) | pwm_fire_channels);
	ast_pwm_write(ast_pwm, ast_pwm_read(ast_pwm, PWM_GENERAL_1) | (pwm_ch << 0x1), PWM_GENERAL_1); 

	//OUT32(PWM_GENERAL_3, 1);	// fire
	ast_pwm_write(ast_pwm, ast_pwm_read(ast_pwm, PWM_GENERAL_3) | 0x1, PWM_GENERAL_3); 
}

/* attr
 *  0 - show/store enable
 *  1 - show/store frequency 
 *	2 - show/store duty
 *	3 - show/store loop_count
 *	4 - show/store update
 */
static ssize_t 
ast_cam_show_pwm(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

	//sensor_attr->index : pwm_ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //enable, disable
			return sprintf(sysfsbuf, "%d : %s\n", ast_get_pwm_en(ast_pwm,sensor_attr->index),ast_get_pwm_en(ast_pwm,sensor_attr->index) ? "Enable":"Disable");
			break;
		case 1: //frequecny
			return sprintf(sysfsbuf, "%d \n",ast_pwm->pwm_unit_frequency[sensor_attr->index]);
			break;
		case 2: //duty
			return sprintf(sysfsbuf, "%d \n",ast_pwm->pwm_duty[sensor_attr->index]);
			break;
		case 3: //loop count
			return sprintf(sysfsbuf, "%d \n",ast_pwm->pwm_loop[sensor_attr->index]);
			break;
		default:
			return -EINVAL;
			break;
	}
}

static ssize_t 
ast_cam_store_pwm(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct sensor_device_attribute_2 *sensor_attr =
						to_sensor_dev_attr_2(attr);

	input_val = simple_strtoul(sysfsbuf, NULL, 10);

	//sensor_attr->index : pwm_ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //enable, disable
			ast_set_pwm_en(ast_pwm, sensor_attr->index, input_val);
			break;
		case 1: //pwm freq
			ast_set_pwm_frequency(ast_pwm, sensor_attr->index, input_val);
			break;
		case 2: //duty
			ast_set_pwm_duty(ast_pwm, sensor_attr->index, input_val);
			break;
		case 3: //loop count
			ast_set_pwm_loop_count(ast_pwm, sensor_attr->index, input_val);
			break;		
		case 4: //update
			ast_set_pwm_update(ast_pwm, sensor_attr->index, input_val);
			break;		
		default:
			return -EINVAL;
			break;
	}

	return count;
}

/* PWM sysfs
 * Macro defining SENSOR_DEVICE_ATTR for a pwm sysfs entries.
 *  0 - show/store enable
 *  1 - show/store freq
 *  2 - show/store duty
 *  3 - show/store loop count
 *  4 - show/store update
 */

#define sysfs_pwm_num(index) \
static SENSOR_DEVICE_ATTR_2(pwm##index##_en, S_IRUGO | S_IWUSR, \
	ast_cam_show_pwm, ast_cam_store_pwm, 0, index); \
\
static SENSOR_DEVICE_ATTR_2(pwm##index##_freq, S_IRUGO | S_IWUSR, \
	ast_cam_show_pwm, ast_cam_store_pwm, 1, index); \
\
static SENSOR_DEVICE_ATTR_2(pwm##index##_duty, S_IRUGO | S_IWUSR, \
	ast_cam_show_pwm, ast_cam_store_pwm, 2, index); \
\
static SENSOR_DEVICE_ATTR_2(pwm##index##_loop, S_IWUSR | S_IWUSR, \
	ast_cam_show_pwm, ast_cam_store_pwm, 3, index); \
\
static SENSOR_DEVICE_ATTR_2(pwm##index##_update, S_IWUSR, \
	ast_cam_show_pwm, ast_cam_store_pwm, 4, index); \
\
static struct attribute *pwm##index##_attributes[] = { \
	&sensor_dev_attr_pwm##index##_en.dev_attr.attr, \
	&sensor_dev_attr_pwm##index##_freq.dev_attr.attr, \
	&sensor_dev_attr_pwm##index##_duty.dev_attr.attr, \
	&sensor_dev_attr_pwm##index##_loop.dev_attr.attr, \
	&sensor_dev_attr_pwm##index##_update.dev_attr.attr, \
	NULL \
};

/*
 * Create the needed functions for each pwm using the macro defined above
 * (4 pwms are supported)
 */
sysfs_pwm_num(0);
sysfs_pwm_num(1);
sysfs_pwm_num(2);
sysfs_pwm_num(3);
sysfs_pwm_num(4);
sysfs_pwm_num(5);
sysfs_pwm_num(6);
sysfs_pwm_num(7);

static const struct attribute_group pwm_attribute_groups[] = {
	{ .attrs = pwm0_attributes },
	{ .attrs = pwm1_attributes },
	{ .attrs = pwm2_attributes },
	{ .attrs = pwm3_attributes },
	{ .attrs = pwm4_attributes },
	{ .attrs = pwm5_attributes },
	{ .attrs = pwm6_attributes },
	{ .attrs = pwm7_attributes },
};

#define PWM_CH_NUM 8
static int 
ast_pwm_probe(struct platform_device *pdev)
{
	struct resource *res;
	int err;
	int ret=0;
	int i;

	dev_dbg(&pdev->dev, "ast_pwm_probe \n");

	ast_pwm = devm_kzalloc(&pdev->dev, sizeof(struct ast_pwm_data), GFP_KERNEL);
	if (!ast_pwm) {
		ret = -ENOMEM;
		goto out;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out_mem;
	}

	ast_pwm->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!ast_pwm->reg_base) {
		ret = -EIO;
		goto out_region;
	}
	/* Register sysfs hooks */
	ast_pwm->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(ast_pwm->hwmon_dev)) {
		ret = PTR_ERR(ast_pwm->hwmon_dev);
		goto out_region;
	}

	for(i=0; i< PWM_CH_NUM; i++) {
		err = sysfs_create_group(&pdev->dev.kobj, &pwm_attribute_groups[i]);
		if (err)
			goto out_region;
	}

	printk(KERN_INFO "ast_pwm: driver successfully loaded.\n");

	return 0;

out_region:
	iounmap(ast_pwm->reg_base);
out_mem:
	kfree(ast_pwm);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int 
ast_pwm_remove(struct platform_device *pdev)
{
	int i=0;
	struct ast_pwm_data *ast_pwm = platform_get_drvdata(pdev);
	struct resource *res;
	printk(KERN_INFO "ast_pwm: driver unloaded.\n");

    hwmon_device_unregister(ast_pwm->hwmon_dev);

	for(i=0; i<8; i++)
		sysfs_remove_group(&pdev->dev.kobj, &pwm_attribute_groups[i]);

	platform_set_drvdata(pdev, NULL);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(ast_pwm->reg_base);
	release_mem_region(res->start, res->end - res->start + 1);
	kfree(ast_pwm);
	return 0;
}

#ifdef CONFIG_PM
static int 
ast_pwm_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("ast_pwm_suspend : TODO \n");
	return 0;
}

static int 
ast_pwm_resume(struct platform_device *pdev)
{	
	printk("ast_pwm_resume : TODO \n");
	return 0;
}

#else
#define ast_pwm_suspend        NULL
#define ast_pwm_resume         NULL
#endif

static const struct of_device_id of_ast_pwm_match_table[] = {
	{ .compatible = "aspeed,ast-pwm", },
	{},
};
MODULE_DEVICE_TABLE(of, of_ast_pwm_match_table);

static struct platform_driver ast_pwm_driver = {
	.probe 		= ast_pwm_probe,
	.remove 		= ast_pwm_remove,
#ifdef CONFIG_PM	
	.suspend        = ast_pwm_suspend,
	.resume         = ast_pwm_resume,
#endif	
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = of_ast_pwm_match_table,
	},
};

module_platform_driver(ast_pwm_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED PWM driver");
MODULE_LICENSE("GPL");

