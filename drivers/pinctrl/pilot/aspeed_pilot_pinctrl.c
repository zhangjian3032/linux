/*
 * Copyright (C) 2019 ASPEED Technology Inc
 * sudheer.veliseti@aspeedtech.com
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


#include <linux/bitops.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include "aspeed_pilot_pinctrl.h"



int aspeed_pilot_pinctrl_get_groups_count (struct pinctrl_dev *pctldev)
{

	struct aspeed_pilot_pinmux *pinmux = pinctrl_dev_get_drvdata(pctldev);
	return 	pinmux->soc->ngroups;
}

int aspeed_pilot_pinctrl_get_group_pins (struct pinctrl_dev *pctldev,
                               unsigned selector,
                               const unsigned **pins,
                               unsigned *num_pins)
{

	struct aspeed_pilot_pinmux *pinmux = pinctrl_dev_get_drvdata(pctldev);
	const struct aspeed_pilot_pin_group *g = &(pinmux->soc->groups[selector]);
	*pins = g->pins;
	*num_pins = 1;
	return 0;

}


const char *aspeed_pilot_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
                                       unsigned selector)
{

	struct aspeed_pilot_pinmux *pinmux = pinctrl_dev_get_drvdata(pctldev);
	const struct aspeed_pilot_pin_group *g = &(pinmux->soc->groups[selector]);
	
	return g->gname;

}

int aspeed_pilot_pinctrl_dt_node_to_map (struct pinctrl_dev *pctldev,
                               struct device_node *np_config,
                               struct pinctrl_map **map, unsigned *num_maps)
{
	pinconf_generic_dt_node_to_map_group(pctldev,np_config,map,num_maps);
	PDEBUG("map.data.mux.function=%s  map.data.mux.group=%s\n",(*map)->data.mux.function,((*map)->data.mux).group);
	return 0;

}	



void aspeed_pilot_pinctrl_dt_free_map (struct pinctrl_dev *pctldev,
                             struct pinctrl_map *map, unsigned num_maps)
{

	pinconf_generic_dt_free_map(pctldev, map,num_maps);
}

static const struct pinctrl_ops aspeed_pilot_pinctrl_ops = {
        .get_groups_count = aspeed_pilot_pinctrl_get_groups_count,
        .get_group_name = aspeed_pilot_pinctrl_get_group_name,
        .get_group_pins = aspeed_pilot_pinctrl_get_group_pins,
//        .pin_dbg_show = aspeed_pilot_pinctrl_pin_dbg_show,
        .dt_node_to_map = aspeed_pilot_pinctrl_dt_node_to_map,
        .dt_free_map = aspeed_pilot_pinctrl_dt_free_map,

};







int aspeed_pilot_pinmux_get_fn_count(struct pinctrl_dev *pctldev)
{
	struct aspeed_pilot_pinmux *pinmux = pinctrl_dev_get_drvdata(pctldev);
	return  pinmux->soc->nfunctions;
}


const char *aspeed_pilot_pinmux_get_fn_name(struct pinctrl_dev *pctldev,
                                      unsigned int function)
{

        struct aspeed_pilot_pinmux *pinmux = pinctrl_dev_get_drvdata(pctldev);
	return  pinmux->soc->functions[function].name;

}



/*
 *  populates array of  groupnames connected to a function selector
 *
 *
 *
 */

int aspeed_pilot_pinmux_get_fn_groups(struct pinctrl_dev *pctldev,
                                unsigned int function,
                                const char * const **groups,
                                unsigned int * const num_groups)
{
        struct aspeed_pilot_pinmux *pinmux = pinctrl_dev_get_drvdata(pctldev);

	*groups =  (pinmux->soc->functions[function].groups);
	*num_groups = (pinmux->soc->functions[function].ngroups);

	return 0;
}

int aspeed_pilot_pinmux_set_mux(struct pinctrl_dev *pctldev, unsigned int func_selector,
                          unsigned int group_selector)
{

        unsigned int reg_addr;
        unsigned int regval_mask;
        unsigned int regval;
	bool change;
        struct aspeed_pilot_pinmux *pinmux = pinctrl_dev_get_drvdata(pctldev);

	struct aspeed_pilot_function *f;
	f = &pinmux->soc->functions[func_selector];
        PDEBUG("%s: %d %s\n",__func__,func_selector,f->name);
	PDEBUG("f->strap_type=%d f->ioengccfg_bit_num=%d f->pinmux_ctrl_bit_num=%d f->gpioen_offset=%d\n",
			f->strap_type,f->ioengccfg_bit_num,f->pinmux_ctrl_bit_num,f->gpioen_offset );

	if(f->strap_type > NO_STRAP ) 
	{

		if( f->strap_type == UART4_STRAP )
		{
#if 0
			reg_addr = pinmux->systemlevel_and_clockcontrol + OSCTL;
			regval = ioread32(reg_addr);
#else
			reg_addr = SYSTEM_CLK_CTL + OSCTL;
			regmap_read(pinmux->map, reg_addr, &regval);
#endif
			PDEBUG("reg_addr=%x regval=%x \n",(unsigned int)reg_addr,regval);
			if(regval & (1 << OEM_UART4_STRAP) )
			{
				PDEBUG("strap Enabled -%d: Fn is=%s \n",f->strap_type,f->name);
			}
			else
			{
				PDEBUG("OEM_UART4_STRAP: not Enabled-%d: Fn is=%s\n",f->strap_type,f->name);
				return -ENODEV;
			}
		}

		else
		{
#if 0
			reg_addr = pinmux->systemlevel_and_clockcontrol + STRPSTS;
			PDEBUG("reg_addr=%d f->strap_type=%d\n",(unsigned int)reg_addr,f->strap_type);
			regval = ioread32(reg_addr);
#endif
			reg_addr = SYSTEM_CLK_CTL + STRPSTS;
			regmap_read(pinmux->map, reg_addr, &regval);
			PDEBUG("strap-status:  reg_addr=%x regval=%x\n",reg_addr,regval);

			if ( regval & 1 << OEM_STRAP )
			{
				PDEBUG("OEM strap Can not be Set: reg_addr=%x regval=%x\n",(unsigned int)reg_addr,regval);
				return -EINVAL; 
		
			}
			else if( (regval & (1 << f->strap_type)) )
			{
				PDEBUG("strap Enabled -%d: Fn is=%s \n",f->strap_type,f->name);

			}
			else
			{
				PDEBUG("strap not Enabled\n");
				return -ENODEV;
			}
		}

	}

	if(f->ioengccfg_bit_num > 0)
	{
#if 0
		reg_addr = pinmux->systemlevel_and_clockcontrol + IOENGCCFG;
		PDEBUG("reg_addr=%x f->=ioengccfg_bit_num=%d f->ioengccfg_bit_val=%d\n",(unsigned int )reg_addr,f->ioengccfg_bit_num,f->ioengccfg_bit_val); 

		regval = ioread32(reg_addr);
#endif
		reg_addr = SYSTEM_CLK_CTL + IOENGCCFG;
		regmap_read(pinmux->map, reg_addr, &regval);
		PDEBUG("reg_addr=%x,regval=%x",(unsigned int)reg_addr, regval);
		if(f->ioengccfg_bit_val)
		{
			regval_mask = 1 << f->ioengccfg_bit_num;
			regval = regval | regval_mask;
		}
		else
		{
			regval_mask = 1 << f->ioengccfg_bit_num;
			regval = regval & (~ regval_mask);

		}
		PDEBUG("setting ioengccfg_bit_num=%d\n",f->ioengccfg_bit_num);
		PDEBUG("reg_addr=%x,regval=%x",(unsigned int)reg_addr, regval);
#if 0
		iowrite32(regval,reg_addr);
		regval = ioread32(reg_addr);
#endif
		regmap_update_bits_base(pinmux->map,reg_addr,regval_mask,regval,&change,false,true);
		regmap_read(pinmux->map, reg_addr, &regval);
		PDEBUG("reg_addr=%x,regval=%x",(unsigned int)reg_addr, regval);

	}

        if(f->pinmux_ctrl_bit_num > -1)
        {
#if 0
		reg_addr = pinmux->top_level_pin_control + PINMUX_CTRL;

		PDEBUG("reg_addr=%x f->pinmux_ctrl_bit_num=%d f->pinmux_ctrl_bit_val=%d\n",(unsigned int )reg_addr,f->pinmux_ctrl_bit_num,f->pinmux_ctrl_bit_val); 
		regval = ioread32(reg_addr);
#endif
		reg_addr = TOP_LVL_PINCTL + PINMUX_CTRL;
		regmap_read(pinmux->map, reg_addr, &regval);
		PDEBUG("reg_addr=%x,regval=%x",(unsigned int)reg_addr, regval);
		if(f->pinmux_ctrl_bit_val)
		{
			regval_mask =  (1 << f->pinmux_ctrl_bit_num);
			regval = regval | regval_mask;
		}
		else 
		{
			regval_mask =  (1 << f->pinmux_ctrl_bit_num);
			regval = regval & (~ regval_mask);
		}
		PDEBUG("setting pinmux_ctrl_bit_num=%d\n", f->pinmux_ctrl_bit_num);
		PDEBUG("reg_addr=%x,regval=%x",(unsigned int)reg_addr, regval);
#if 0
		iowrite32(regval,reg_addr);
		regval = ioread32(reg_addr);
#endif
		regmap_update_bits_base(pinmux->map,reg_addr,regval_mask,regval,&change,false,true);
		regmap_read(pinmux->map, reg_addr, &regval);
		PDEBUG("reg_addr=%x,regval=%x",(unsigned int)reg_addr, regval);

        }

        if ( f->gpioen_offset > -1)
        {

		PDEBUG("gpioen_offset=%x gpioen_bitnum=%d gpioen_bitval=%d\n",f->gpioen_offset,f->gpioen_bitnum,f->gpioen_bitval);	
#if 0
		reg_addr = pinmux->top_level_pin_control + (f->gpioen_offset+ (f->gpioen_bitnum/8)  );
		regval = ioread32(reg_addr);
#endif
		reg_addr = TOP_LVL_PINCTL + (f->gpioen_offset);
		regmap_read(pinmux->map, reg_addr, &regval);
		PDEBUG("reg_addr=%x,regval=%x",(unsigned int)reg_addr, regval);
		if(f->gpioen_bitval)
		{
		regval_mask =  (1 << (f->gpioen_bitnum));
		regval = regval | regval_mask;
		}
		else
		{
			regval_mask = (1 << (f->gpioen_bitnum));
			regval = regval & (~ regval_mask);
		}
		PDEBUG("setting gpioen \n");
		PDEBUG("reg_addr=%x,regval=%x",(unsigned int)reg_addr, regval);
#if 0
		iowrite32(regval,reg_addr);
		regval = ioread32(reg_addr);
#endif
		regmap_update_bits_base(pinmux->map,reg_addr,regval_mask,regval,&change,false,true);
		regmap_read(pinmux->map, reg_addr, &regval);
		PDEBUG("reg_addr=%x,regval=%x",(unsigned int)reg_addr, regval);
        }
	return 0;
}

int aspeed_pilot_gpio_request_enable(struct pinctrl_dev *pctldev,struct pinctrl_gpio_range *range,unsigned offset)
{
	struct aspeed_pilot_pinmux *pinmux = pinctrl_dev_get_drvdata(pctldev);
	const struct aspeed_pilot_pin_group *g = NULL;
	const struct aspeed_pilot_function  *f = NULL;
	unsigned int grp_selector;
	unsigned int func_selector;
	unsigned int pin = offset;

        PDEBUG("%s\n",__func__);
	for (grp_selector=0; grp_selector < pinmux->soc->ngroups; grp_selector++)
	{
		g = &(pinmux->soc->groups[grp_selector]);
		if(pin == (*(g->pins)) )
		{
			PDEBUG("pin=%d g->gname=%s found\n",pin,g->gname);
			PDEBUG("funcs- %d %d %d\n",g->funcs[0],g->funcs[1],g->funcs[2]);
			break;

		}
	}

	if( grp_selector < pinmux->soc->ngroups )
	{
		int i,ret;
		for( i=0; i<= 2; i++)
		{
			if(g->funcs[i] == -1)
				continue;
			f = &(pinmux->soc->functions[ g->funcs[i] ] );
			PDEBUG("f->name=%s strncmp-val=%d\n",f->name,strncmp(f->name,"GPIO",4) );
			if(strncmp(f->name,"GPIO",4) == 0)
			{
				func_selector = g->funcs[i];
				PDEBUG("func_selector =%d func->name=%s\n",func_selector,f->name);
				ret = aspeed_pilot_pinmux_set_mux(pctldev, func_selector,grp_selector);
				if (ret < 0)
				{
					PDEBUG("pinmux setting  for pin-%s Failed\n",f->name);
					return ret;
				}
			}
		}
	}
#if 0
	unsigned int func_selector;
	if( offset >= 0 && offset <= 31)
		func_selector = offset + 319;

	if(offset >= 32 && offset <= 141)
		func_selector = offset + 320;

	if(offset >= 142 && offset <= 506)
		func_selector = offset + 320;

	aspeed_pilot_pinmux_set_mux(pctldev, func_selector, 0);
#endif

 return 0;
}

static const struct pinmux_ops aspeed_pilot_pinmux_ops = {
	.strict = 1,
	.get_functions_count = aspeed_pilot_pinmux_get_fn_count,
        .get_function_name = aspeed_pilot_pinmux_get_fn_name,
        .get_function_groups = aspeed_pilot_pinmux_get_fn_groups,
        .set_mux = aspeed_pilot_pinmux_set_mux,
        .gpio_request_enable = aspeed_pilot_gpio_request_enable,

};



int aspeed_pilot_pin_config_get(struct pinctrl_dev *pctldev, unsigned int offset,
		                unsigned long *config)
{
        PDEBUG("%s\n",__func__);
	return -1;
}


int aspeed_pilot_pin_config_set(struct pinctrl_dev *pctldev, unsigned int pin,
                unsigned long *configs, unsigned int num_configs)
{
	struct aspeed_pilot_pinmux *pinmux = pinctrl_dev_get_drvdata(pctldev);
	const struct aspeed_pilot_pin_group *g = NULL;
	unsigned int selector;
        PDEBUG("%s\n",__func__);
	for (selector=0; selector < pinmux->soc->ngroups; selector++)
	{
		g = &(pinmux->soc->groups[selector]);
		PDEBUG("g->gname=%s,g->pins=%d\n",g->gname,*(g->pins));
		if(pin == (*(g->pins)) )
		{
			printk("pin=%d g->gname=%s found\n",pin,g->gname);
			aspeed_pilot_pin_config_group_set(pctldev,selector,configs,num_configs);
			return 0;
		}
	}
	PDEBUG("pin %d not found\n",pin);
	return -1;
}


int aspeed_pilot_pin_config_group_get (struct pinctrl_dev *pctldev,
                                     unsigned selector,
                                     unsigned long *config)
{
	PDEBUG("%s\n",__func__);

	return 0;

}

int print_pin_group( const struct aspeed_pilot_pin_group *g)
{
	PDEBUG("g->gname=%s\n",g->gname);
	PDEBUG("g->pinctrlreg_offset=%x \n",g->pinctrlreg_offset);
	PDEBUG("g->padctrl_type=%d\n",g->padctrl_type);
return 0;
}

static void pilot_regmap_read8(struct regmap * map, 

					u32 offset, u8 * val)
{
	u32 temp_val;
	u8 b_pos = offset & 3;
	regmap_read(map, (offset & ~3), &temp_val);
	PDEBUG("Read val is %x\n", temp_val);
	*val = ((temp_val) >> (b_pos << 3)) & 0xFF;
}

/*
static void pilot_regmap_write8(struct regmap * map,
					u32 offset, u8 val)
{
	u32 temp_val;
	u8 b_pos = offset & 3;
	regmap_read(map, offset, &temp_val);
	temp_val &= ~(((0xFF) << (b_pos << 3)));
	temp_val |= (((val) << (b_pos << 3)));
	regmap_update_bits(map, (offset&3), temp_val, temp_val);

}
*/
static void pilot_regmap_write8(struct regmap * map,u32 offset, u8 mask, u8 val)
{
	u32 temp_val;
	bool change;
	u32 msk=0;
	u8 b_pos = offset & 3;

	regmap_read(map, offset&~(3), &temp_val);
	temp_val |= (((val) << (b_pos << 3)));
	PDEBUG("The value written is %x to be written is %x\n", temp_val, val);

	temp_val =0;
	temp_val |= (((val) << (b_pos << 3)));
        msk = 0;
	msk |= (((mask) << (b_pos << 3)));
	PDEBUG("mask is %x tval is %x val is %x\n", msk, temp_val, val);

	regmap_update_bits_base(map, (offset&~(3)), msk, temp_val,&change, false, true);
}

int aspeed_pilot_pin_config_group_set (struct pinctrl_dev *pctldev,
                                     unsigned selector,
                                     unsigned long *configs,
                                     unsigned num_configs)
{
	int i;
	unsigned char reg_addr;
	unsigned char regval_mask;
	unsigned char regval;

	struct aspeed_pilot_pinmux *pinmux = pinctrl_dev_get_drvdata(pctldev);

	const struct aspeed_pilot_pin_group *g=NULL;

	PDEBUG("%s:selector=%d  grp-name=%s\n",__func__,selector, aspeed_pilot_pinctrl_get_group_name(pctldev,selector) );
	g = &(pinmux->soc->groups[selector]);
	print_pin_group(g);
#if 0
	reg_addr = pinmux->top_level_pad_control + g->pinctrlreg_offset;
#endif
	reg_addr = TOP_LVL_PADCTL + g->pinctrlreg_offset;
	for (i = 0; i < num_configs; i++) {
	
		enum pin_config_param param;
		u32 arg;
		regval_mask = 0;

		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);
		PDEBUG("param=%d arg=%d\n",param,arg);
#if 0
		regval = ioread8(reg_addr);
#endif
		pilot_regmap_read8(pinmux->map, (u32)reg_addr, &regval);
		PDEBUG("reg_addr=%x,regval=%x",(unsigned int)reg_addr, regval);
		switch (param) {
			case PIN_CONFIG_DRIVE_STRENGTH:
				if (arg > 7)
					return -EINVAL;
				if (arg >=0 && arg <4)
				{
					if (g->padctrl_type != 1)
						return -EINVAL;
					regval_mask |= arg << 2;

				}


				if (arg >= 4 && arg < 8)
				{
					if (g->padctrl_type != 2)
						return -EINVAL;
					regval_mask |= (arg-4) << 2;

				}
				PDEBUG("setting Drive strength\n");
				regval = regval & DRIVESTRENGTH_MASK;
				regval = regval | regval_mask;
#if 0
				iowrite8(regval|regval_mask , reg_addr);
#endif
				break;

			case PIN_CONFIG_BIAS_PULL_UP:

				regval_mask |=  0x01;
				PDEBUG("setting bias pull up\n");
				regval = regval & BIASPULL_MASK;
				regval = regval | regval_mask;
#if 0
				iowrite8(regval|regval_mask , reg_addr);
#endif
				break;


			case PIN_CONFIG_BIAS_PULL_DOWN:
				regval_mask |=  0x03;
				PDEBUG("setting bias pull down\n");
				regval = regval & BIASPULL_MASK;
				regval = regval | regval_mask;
#if 0
				iowrite8(regval|regval_mask , reg_addr);
#endif
				break;

			case PIN_CONFIG_BIAS_PULL_PIN_DEFAULT:
				regval_mask |= 0x00;
				PDEBUG("setting bias pull default\n");
				regval = regval & BIASPULL_MASK;
				regval = regval | regval_mask;
#if 0
				iowrite8(regval|regval_mask , reg_addr);
#endif
				break;


			case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
			
				if (g->padctrl_type != 2)
					return -EINVAL;
				regval_mask |= 1<<4;
				PDEBUG("setting i/p schmit\n");
				regval = regval | regval_mask;
#if 0
				iowrite8(regval|regval_mask , reg_addr);
#endif
				break;
			}
	PDEBUG("regval_mask=%x regval=%x\n",regval_mask,regval);
	pilot_regmap_write8(pinmux->map,(u32)reg_addr,0xFF,regval);
#if 0
	regval = ioread8(reg_addr);
#endif
	pilot_regmap_read8(pinmux->map, (u32)reg_addr, &regval);
	PDEBUG("reg_addr=%x,regval=%x",(unsigned int)reg_addr, regval);
	}
	return 0;

}

static const struct pinconf_ops aspeed_pilot_conf_ops = {
	.is_generic = true,
        .pin_config_get = aspeed_pilot_pin_config_get,
        .pin_config_set = aspeed_pilot_pin_config_set,
        .pin_config_group_get = aspeed_pilot_pin_config_group_get,
        .pin_config_group_set = aspeed_pilot_pin_config_group_set,

};

static struct pinctrl_desc aspeed_pilot_pinctrl_desc = {

	.name = "aspeed-pilot-pinctrl",
        .pctlops = &aspeed_pilot_pinctrl_ops,
        .pmxops = &aspeed_pilot_pinmux_ops,
        .confops = &aspeed_pilot_conf_ops,
	.owner = THIS_MODULE,
};


static struct pinctrl_gpio_range aspeed_pilot_pinctrl_gpio_range = {
        .name = "ASPEED PILOT GPIOs",
        .id = 0,
        .base = 0,
};


 int aspeed_pilot_pinctrl_probe(struct platform_device *pdev,const struct aspeed_pilot_pinctrl_soc_data *aspeed_pilot_soc_data ){

	struct aspeed_pilot_pinmux *pinmux;
	int fn,gn,gfn;
	struct device_node *np;
	//struct resource *res;
	const char **func_groups;

	np = pdev->dev.of_node;

        pinmux = (struct aspeed_pilot_pinmux *)devm_kzalloc(&pdev->dev, sizeof(*pinmux), GFP_KERNEL);
        if (!pinmux)
                return -ENOMEM;

        pinmux->dev = &pdev->dev;
	pinmux->map = syscon_regmap_lookup_by_phandle(np,
                "syscon");
         if (IS_ERR(pinmux->map)) {
                dev_err(&pdev->dev, "Couldn't get regmap\n");
                return -ENODEV;
	 }

/*
        res = platform_get_resource(pdev, IORESOURCE_MEM,0);
	PDEBUG("res->start = %x  res->end=%x \n", res->start,res->end);	
        pinmux->systemlevel_and_clockcontrol = devm_ioremap_resource(&pdev->dev, res);
                if (IS_ERR(pinmux->systemlevel_and_clockcontrol))
                        return PTR_ERR(pinmux->systemlevel_and_clockcontrol);
	PDEBUG("pinmux->systemlevel_and_clockcontrol=%x\n",(unsigned int)pinmux->systemlevel_and_clockcontrol);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
        pinmux->top_level_pin_control = devm_ioremap_resource(&pdev->dev, res);
                if (IS_ERR(pinmux->top_level_pin_control))
                        return PTR_ERR(pinmux->top_level_pin_control);
	PDEBUG("res->start = %x  res->end=%x \n", res->start,res->end);	
	PDEBUG("pinmux->top_level_pin_control=%x\n",(unsigned int)pinmux->top_level_pin_control);

        res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
        pinmux->top_level_pad_control = devm_ioremap_resource(&pdev->dev, res);
                if (IS_ERR(pinmux->top_level_pad_control))
                        return PTR_ERR(pinmux->top_level_pad_control);

	PDEBUG("res->start = %x  res->end=%x \n", res->start,res->end);	
	PDEBUG("pinmux->top_level_pad_control=%x\n",(unsigned int)pinmux->top_level_pad_control);
	*/
	pinmux->soc = aspeed_pilot_soc_data;
	pinmux->func_groups = devm_kzalloc(&pdev->dev,aspeed_pilot_soc_data->ngroups * 4 * sizeof(*pinmux->func_groups),GFP_KERNEL);
	func_groups = pinmux->func_groups;

	for (fn = 0; fn < aspeed_pilot_soc_data->nfunctions; fn++) {
		struct aspeed_pilot_function *func = &(pinmux->soc->functions[fn]);
		func->groups = func_groups;
		for (gn = 0; gn < aspeed_pilot_soc_data->ngroups; gn++) {

			const struct  aspeed_pilot_pin_group *pin_group = &(aspeed_pilot_soc_data->groups[gn]);
			for (gfn = 0; gfn < 3; gfn++) {
				if (pin_group->funcs[gfn] == fn) {
					break;
				}
			}
			if(gfn==3)
			  continue;
			*func_groups++ = pin_group->gname;
			func->ngroups++;
	           }
	}
	aspeed_pilot_pinctrl_desc.name = dev_name(&pdev->dev);
	aspeed_pilot_pinctrl_desc.pins = pinmux->soc->pins;
	aspeed_pilot_pinctrl_desc.npins = pinmux->soc->npins; 
	pinmux->pctrldev = pinctrl_register(&aspeed_pilot_pinctrl_desc, &pdev->dev, pinmux );
        if (IS_ERR(pinmux->pctrldev)) {
                dev_err(&pdev->dev, "Failed to register pinctrl\n");
                return PTR_ERR(pinmux->pctrldev);
        }

return 0;
}	
EXPORT_SYMBOL_GPL(aspeed_pilot_pinctrl_probe);
