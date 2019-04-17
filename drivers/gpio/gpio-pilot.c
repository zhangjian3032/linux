/*
 * Copyright (C) 2019 ASPEED Technology Inc
 * sudheer.veliseti@aspeedtech.com
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/gpio/driver.h>
#include <linux/gpio.h>

#define MAX_NUM_GPIOS 186

// offset addresses of  per port registers
#define GPCFG		 0x00
#define GPDO		 0x08
#define GPDI		 0x09
#define GPEVEN		 0x0A
#define GPEVST		 0x0B
#define GPDBC0		 0x0C
#define GPDBC1		 0x0D
#define GPHOSTGRP	 0x0E

#define OUTPUT_ENABLE 		0x01
#define GPIO_BIT_SET(x) 	(1 << x)
#define GPIO_BIT_CLR(x)  	~(1 << x)
#define EDGE_TRIGGER		0xF7
#define FALLING_EDGE		0xEF
#define RISING_EDGE  		0x10
#define LEVEL_TRIGGER 		0x04
#define LOW_LEVEL  		0xEF
#define HIGH_LEVEL 		0x10
#define DEBOUNCE_ENABLE 	0x20
#define GPIO_EVENT_TO_IRQ_ENABLE 0x40
#define CLR_GPCFG_BIAS_CTRL_BITS  0xF9
#define INTERNAL_PULL_DOWN_ENABLE 0x00
#define INTERNAL_PULL_UP_ENABLE   0x04
#define NO_PULLUP_NO_PULLDOWN     0x06

// represents gpio-block of soc
struct pilot_gpio {
        struct gpio_chip chip;
        spinlock_t lock;
        void __iomem *base;
        int irq;
        struct irq_chip irq_chip;
        struct irq_domain *irq_domain;
	int gpio_bank1_usage;
};

// gpio-port-block
struct pilot_gpio_bank {
        uint16_t        bank_base;
        uint16_t        irq_regs;
};

struct pilot_gpio_pin {
	// gpionumber is offset in chip; also gpionumber = hwirq of  gpio
	unsigned int hwirq;
	unsigned char bank_number;
	unsigned char port_number;
	unsigned char pin_number;
	// bank specific details
	unsigned int first_gpio_in_bank;
	unsigned int first_port_in_bank;
        unsigned int ports_index_in_bank;
	unsigned int bank_offset_addr;
	unsigned int port_offset_addr;

	char  *gpio_base;
};


int  gpio_bank_base_addroffsets[]={ 0x00, 0x100, 0x300 };

unsigned int  comined_irq_status_reg_offset[6] = { 0xf1, 0xf2, 0xf5, 0xf6, 0x2a1, 0x2a2};

char GPISR_port_number_lookup[6][8] ={ {0,1,2,3,4,5,6,7},{8,9,10,11,12,13,14,-1}, {16,17,18,19,20,21,22,23},
		{24,25,26,27,28,29,-1,-1}, {30,31,32,33,34,35,36,37},{38,39,40,41,42,43,44,45} };

//#define GPIO_DEBUG_PRINTS 1

#undef PDEBUG
#ifdef GPIO_DEBUG_PRINTS
        #define PDEBUG(fmt, args...) printk("pilot-gpio:"  fmt , ##args)
	#define DEBUG_PRINT(reg)  debug_print(reg)
#else
        #define PDEBUG(fmt, args...)
	#define DEBUG_PRINT(reg)
#endif


static void  debug_print(char *reg_addr)
{
	char reg_data;
	reg_data = ioread8(reg_addr);
	PDEBUG("reg_addr=%x reg_data=%x\n",reg_addr,reg_data);
	
}

/*
static inline struct pilot_gpio *to_pilot_gpio(struct gpio_chip *gc)
{
        return  container_of(gc, struct pilot_gpio, chip) ;
}
*/

/*
Bank1: port0 to port3 ==> gpio0 to gpio31

Bank2: port16 to port29(29_0 to 29_5)==> gpio32 to gpio141

Bank3: port40 to port45(45_0 to 45_3) ==> gpio142 to gpio185
*/

#define BANK0_FIRST_GPIO 0
#define BANK0_LAST_GPIO  31
#define BANK1_FIRST_GPIO 32
#define BANK1_LAST_GPIO  141
#define BANK2_FIRST_GPIO 142
#define BANK2_LAST_GPIO  185

#define BANK0_FIRST_PORT 0
#define BANK0_LAST_PORT  3
#define BANK1_FIRST_PORT 16
#define BANK1_LAST_PORT  29
#define BANK2_FIRST_PORT 40
#define BANK2_LAST_PORT  45


static void  get_pin_details(struct pilot_gpio_pin *gpio_pin, unsigned int offset)
{
	if( (offset >= BANK0_FIRST_GPIO) && (offset <= BANK0_LAST_GPIO) )
	{
		gpio_pin->bank_number = 0;
		gpio_pin->first_gpio_in_bank = BANK0_FIRST_GPIO;
		gpio_pin->first_port_in_bank = BANK0_FIRST_PORT;
	}
	else if( (offset >= BANK1_FIRST_GPIO) && (offset <= BANK1_LAST_GPIO) )
	{
		gpio_pin->bank_number = 1;
		gpio_pin->first_gpio_in_bank = BANK1_FIRST_GPIO;
		gpio_pin->first_port_in_bank = BANK1_FIRST_PORT;
	}
	else if( (offset >= BANK2_FIRST_GPIO) && (offset <= BANK2_LAST_GPIO) )
	{
		gpio_pin->bank_number = 2;
		gpio_pin->first_gpio_in_bank = BANK2_FIRST_GPIO;
		gpio_pin->first_port_in_bank = BANK2_FIRST_PORT;
	}

	gpio_pin->ports_index_in_bank = (offset - gpio_pin->first_gpio_in_bank)/8;
	gpio_pin->port_number = gpio_pin->first_port_in_bank + gpio_pin->ports_index_in_bank;
	gpio_pin->pin_number = (offset - gpio_pin->first_gpio_in_bank)%8;

	gpio_pin->bank_offset_addr = gpio_bank_base_addroffsets[gpio_pin->bank_number];
	gpio_pin->port_offset_addr = (gpio_pin->bank_offset_addr + (gpio_pin->ports_index_in_bank*16));
	gpio_pin->hwirq = offset;

	PDEBUG("%s:bank=%d port=%d pin=%d\n",__func__,gpio_pin->bank_number,gpio_pin->port_number,gpio_pin->pin_number);
	PDEBUG("ports_index_in_bank=%d bank_offset_addr=%x port_offset_addr=%x\n",gpio_pin->ports_index_in_bank,gpio_pin->bank_offset_addr,gpio_pin->port_offset_addr);
}


static void *pilot_gpio_port_reg(const struct pilot_gpio_pin *gpio_pin,
                unsigned int reg_offset)
{
	if(reg_offset == GPCFG)
		reg_offset = GPCFG + gpio_pin->pin_number;
	return(void *)(gpio_pin->gpio_base + gpio_pin->port_offset_addr + reg_offset);
}


static int pilot_gpio_request(struct gpio_chip *chip, unsigned int offset)
{
	int ret=0;
	ret =  gpiochip_generic_request( chip,offset);
	if(ret)
	{
		PDEBUG("GPIO request Failed for gpio%d \n",chip->base + offset);
	}
	else
	{
		PDEBUG("GPIO request success for gpio%d \n",chip->base + offset);
	}
	return ret;
}


static void pilot_gpio_free(struct gpio_chip *chip, unsigned int offset)
{
	gpiochip_generic_free(chip,offset);
}


static int enable_debounce(struct pilot_gpio_pin *gpio_pin,
				    unsigned long debounce)
{
	char *reg_addr,reg_data;
	char timer_debounce_sel=0;
	switch(debounce)
	{
	 case 16000000:
		timer_debounce_sel=1;
		break;

	 case 8000000:
		timer_debounce_sel=0;
		break;

	 case 120:
		timer_debounce_sel=3;
		break;

	  case 60:
		timer_debounce_sel=2;
		break;

	   default:
		PDEBUG("%s Invalid debounce period\n",__func__);
		return -EINVAL;

	}

	timer_debounce_sel = timer_debounce_sel<<((gpio_pin->pin_number%4)*2);
	reg_addr = pilot_gpio_port_reg(gpio_pin,GPCFG);
	reg_data = ioread8(reg_addr);
	DEBUG_PRINT(reg_addr);
	reg_data = reg_data | DEBOUNCE_ENABLE;
	iowrite8( reg_data ,reg_addr );
	DEBUG_PRINT(reg_addr);
	if(gpio_pin->pin_number <= 3)
		reg_addr = pilot_gpio_port_reg(gpio_pin,GPDBC0);
	else
		reg_addr = pilot_gpio_port_reg(gpio_pin,GPDBC1);
	reg_data = ioread8(reg_addr);
	DEBUG_PRINT(reg_addr);
	reg_data = reg_data & ( ~ (3 << ((gpio_pin->pin_number%4)*2) ));
	reg_data = reg_data | timer_debounce_sel;
	iowrite8( reg_data,reg_addr);
	DEBUG_PRINT(reg_addr);

	return 0;
}


static int disable_debounce( struct pilot_gpio_pin *gpio_pin)
{
	char *reg_addr,reg_data;
	reg_addr = pilot_gpio_port_reg(gpio_pin,GPCFG);
	reg_data = ioread8(reg_addr);
	DEBUG_PRINT(reg_addr);
	reg_data = reg_data & ~DEBOUNCE_ENABLE;
	iowrite8(reg_data, reg_addr);
	DEBUG_PRINT(reg_addr);
	return 0;

}

static int pilot_set_debounce(struct gpio_chip *gc, unsigned int offset,
				    unsigned int debounce )
{

	char *reg_addr;
	struct pilot_gpio *gpio = (struct pilot_gpio *)gpiochip_get_data(gc);
	struct pilot_gpio_pin gpio_pin;
	get_pin_details(&gpio_pin,offset);
	gpio_pin.gpio_base = (char *) gpio->base;
	reg_addr = pilot_gpio_port_reg(&gpio_pin,GPCFG);
	DEBUG_PRINT(reg_addr);
	if (debounce)
		return enable_debounce(&gpio_pin , debounce);
	else
		return disable_debounce(&gpio_pin);

return 0;
}

static int pilot_gpio_pin_bias_settings(struct gpio_chip *gc, unsigned int offset,
				        unsigned int arg  )
{

	char *reg_addr;
	char reg_data;
	struct pilot_gpio *gpio = (struct pilot_gpio *)gpiochip_get_data(gc);
	struct pilot_gpio_pin gpio_pin;
	get_pin_details(&gpio_pin,offset);
	gpio_pin.gpio_base = (char *) gpio->base;
	reg_addr = pilot_gpio_port_reg(&gpio_pin,GPCFG);
	DEBUG_PRINT(reg_addr);
	reg_data = ioread8(reg_addr);
	reg_data = reg_data & CLR_GPCFG_BIAS_CTRL_BITS;
	reg_data = reg_data | arg;
	iowrite8(reg_data, reg_addr);
        DEBUG_PRINT(reg_addr);
return 0;
}

int pilot_gpio_of_xlate(struct gpio_chip *gc,const struct of_phandle_args *gpiospec,u32 *flags)
{

	struct device_node *np = gpiospec->np;
	unsigned int gpio = gpiospec->args[0];
	unsigned int drive_strength;
	unsigned int debounce_interval;
	int rc;
	unsigned int i;

        if (gc->of_gpio_n_cells < 2) {
                WARN_ON(1);
                return -EINVAL;
        }

        if (WARN_ON(gpiospec->args_count < gc->of_gpio_n_cells))
                return -EINVAL;

        if (gpiospec->args[0] >= gc->ngpio)
                return -EINVAL;

        if (flags)
                *flags = gpiospec->args[1];


	for(i=2;i < gpiospec->args_count;i++)
	{
		unsigned long packed;
		unsigned int param,arg;
		packed = gpiospec->args[i];
		param = packed & 0xff;
		arg = (packed >> 8) & 0x00ffffff;
		packed = pinconf_to_config_packed(param,arg);
		rc = gc->set_config(gc, gpio, packed);
		if (rc == -ENOTSUPP) {
			PDEBUG("param=%d not supported for GPIO%d\n",param,gpio);
			return 0;
		}
		PDEBUG("gpios-cell%d-handled,gpiospec->args[%d]=%d set\n",i,i,gpiospec->args[i] );
	}
return gpio;
}


/* set_config: optional hook for all kinds of settings. Uses the same
 *	packed config format as generic pinconf */

static int pilot_gpio_set_config(struct gpio_chip *chip, unsigned int offset,
				  unsigned long config)
{

	unsigned long param = pinconf_to_config_param(config);
	u32 arg = pinconf_to_config_argument(config);
	PDEBUG("offset=%d param =%ld  arg=%d\n",offset,param,arg);
	if (param == PIN_CONFIG_INPUT_DEBOUNCE)
		return pilot_set_debounce(chip, offset, arg);
	else if (param == PIN_CONFIG_BIAS_DISABLE ||
			param == PIN_CONFIG_BIAS_PULL_DOWN ||
			param == PIN_CONFIG_BIAS_PULL_UP ||
			param == PIN_CONFIG_BIAS_PULL_PIN_DEFAULT)
		return pilot_gpio_pin_bias_settings(chip,offset,arg);
	else if (param == PIN_CONFIG_DRIVE_OPEN_DRAIN ||
			param == PIN_CONFIG_DRIVE_OPEN_SOURCE)
		return -ENOTSUPP;
	else if (param == PIN_CONFIG_DRIVE_STRENGTH)
		return pinctrl_gpio_set_config(offset, config);
	else if (param == PIN_CONFIG_PERSIST_STATE)
		//TO DO : need to handle this config setting in case of power management
		return -ENOTSUPP;
return 0;
}




static int pilot_gpio_dir_in(struct gpio_chip *gc, unsigned int offset)
{
	char *reg_addr,reg_data;
	struct pilot_gpio *gpio=NULL;
	struct pilot_gpio_pin gpio_pin;

	gpio = (struct pilot_gpio *)gpiochip_get_data(gc);
	get_pin_details(&gpio_pin,offset);
	gpio_pin.gpio_base = (char *) gpio->base;
	reg_addr = pilot_gpio_port_reg(&gpio_pin,GPCFG);
	reg_data = ioread8(reg_addr);
	reg_data = (reg_data & ~OUTPUT_ENABLE );
	DEBUG_PRINT(reg_addr);
	iowrite8( reg_data ,reg_addr );
	DEBUG_PRINT(reg_addr);
	return 0;
}

static int pilot_gpio_dir_out(struct gpio_chip *gc, unsigned int offset,int value)
{
	char *reg_addr,reg_data;
	struct pilot_gpio *gpio=NULL;
	struct pilot_gpio_pin gpio_pin;

	gpio = (struct pilot_gpio *)gpiochip_get_data(gc);
	gpio_pin.gpio_base = (char *) gpio->base;
	get_pin_details(&gpio_pin,offset);
	reg_addr =  pilot_gpio_port_reg(&gpio_pin,GPCFG);
	reg_data = ioread8(reg_addr);
	reg_data = (reg_data | OUTPUT_ENABLE);
	DEBUG_PRINT(reg_addr);
	iowrite8(reg_data,reg_addr);
	DEBUG_PRINT(reg_addr);

	reg_addr =  pilot_gpio_port_reg(&gpio_pin,GPDO);
	reg_data = ioread8(reg_addr);
	reg_data = reg_data & ~(1 << gpio_pin.pin_number);
	reg_data = reg_data | (value << gpio_pin.pin_number);
	DEBUG_PRINT(reg_addr);
	iowrite8( reg_data , reg_addr );
	DEBUG_PRINT(reg_addr);

	return 0;
}


static void __pilot_gpio_set(struct gpio_chip *gc, unsigned int offset, int val)
{

	char *reg_addr,reg_data;
	struct pilot_gpio *gpio=NULL;
	struct pilot_gpio_pin gpio_pin;

	gpio = (struct pilot_gpio *)gpiochip_get_data(gc);
	gpio_pin.gpio_base = (char *) gpio->base;
	get_pin_details(&gpio_pin,offset);
	reg_addr = (char *)pilot_gpio_port_reg(&gpio_pin,GPDO);
	reg_data = ioread8(reg_addr);

	DEBUG_PRINT(reg_addr);
        if (val)
	{
		reg_data = (reg_data|GPIO_BIT_SET(gpio_pin.pin_number));
	}
	else
	{
		reg_data = (reg_data & GPIO_BIT_CLR(gpio_pin.pin_number));
	}
	DEBUG_PRINT(reg_addr);
	iowrite8(reg_data , reg_addr);
	DEBUG_PRINT(reg_addr);

}

static void pilot_gpio_set(struct gpio_chip *gc, unsigned int offset,int val)
{
	char *reg_addr,reg_data;
	struct pilot_gpio *gpio=NULL;
	struct pilot_gpio_pin gpio_pin;

	gpio = (struct pilot_gpio *)gpiochip_get_data(gc);
	gpio_pin.gpio_base = (char *) gpio->base;
	get_pin_details(&gpio_pin,offset);

        reg_addr = pilot_gpio_port_reg(&gpio_pin,GPCFG);
        reg_data = ioread8(reg_addr);
	DEBUG_PRINT(reg_addr);

        if( reg_data & 0x01)
        {
		__pilot_gpio_set(gc, offset, val);
	}
	else
	{
                PDEBUG("Error:%s-gpio%d is input pin\n",__func__,offset);
	}

}

static int  pilot_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	unsigned char *reg_addr = NULL;
	unsigned char reg_data;
	struct pilot_gpio *gpio = NULL;
	struct pilot_gpio_pin gpio_pin;

	gpio = (struct pilot_gpio *)gpiochip_get_data(gc);

	gpio_pin.gpio_base = ( char * ) gpio->base;
	get_pin_details(&gpio_pin,offset);

	reg_addr = pilot_gpio_port_reg(&gpio_pin,GPCFG);
	reg_data = ioread8(reg_addr);
	DEBUG_PRINT(reg_addr);
	if( reg_data & 0x01)
	{
		reg_addr = pilot_gpio_port_reg(&gpio_pin,GPDO);
		reg_data = ioread8(reg_addr);
		DEBUG_PRINT(reg_addr);
		return (reg_data & (1 << gpio_pin.pin_number));
	}
	else
	{
                reg_addr = pilot_gpio_port_reg(&gpio_pin,GPDI);
                reg_data = ioread8(reg_addr);
		DEBUG_PRINT(reg_addr);
                return (reg_data & (1 << gpio_pin.pin_number));
	}

}

static int pilot_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	int linux_irq;
	struct pilot_gpio *gpio = NULL;
        gpio = (struct pilot_gpio *)gpiochip_get_data(chip);
	if(!gpio)
		return -ENXIO;
	linux_irq = irq_find_mapping(gpio->irq_domain, offset);
	PDEBUG("%s:hw_irq=%d linux_irq=%d\n",__func__,offset,linux_irq);
	return linux_irq;
}




static void pilot_gpio_irq_ack(struct irq_data *d)
{
#if 0
	char *reg_addr;
	char reg_data;
	unsigned int offset;
	struct pilot_gpio *gpio;
	struct pilot_gpio_pin gpio_pin;
	//get the hw_irq number from irq_data
	offset = irqd_to_hwirq(d);
	gpio=irq_data_get_irq_chip_data(d);
	get_pin_details(&gpio_pin,offset);
	gpio_pin.gpio_base = (char *) gpio->base;
	//clear the the status bit in GPEVST reg , for Edge trigger case
	reg_addr = pilot_gpio_port_reg(&gpio_pin,GPEVST);
	reg_data = ioread8(reg_addr);
	DEBUG_PRINT(reg_addr);
	reg_data =  reg_data | GPIO_BIT_SET(gpio_pin.pin_number);
	iowrite8( reg_data , reg_addr );
	DEBUG_PRINT(reg_addr);
	//For Level triggered INTR: the actual source should be cleared to clear the status.
#endif
	return ;
}


static int pilot_gpio_set_type(struct irq_data *d, unsigned int type)
{
	char *reg_addr,reg_data;
	unsigned int offset;
	struct pilot_gpio *gpio;
	struct pilot_gpio_pin gpio_pin;
	irq_flow_handler_t handler;
	//bool active_low;
	//struct gpio_desc *gpiodesc;
	gpio=irq_data_get_irq_chip_data(d);
	offset = irqd_to_hwirq(d);
	get_pin_details(&gpio_pin,offset);
	gpio_pin.gpio_base = (char *) gpio->base;

	reg_addr = pilot_gpio_port_reg(&gpio_pin,GPCFG);
	DEBUG_PRINT(reg_addr);
	reg_data = ioread8(reg_addr);
	PDEBUG("%s: type=%d\n",__func__,type);
	switch (type & IRQ_TYPE_SENSE_MASK) {

	case IRQ_TYPE_EDGE_BOTH:
#if 0
		gpiodesc = gpio_to_desc(offset);
		active_low = gpiod_is_active_low(gpiodesc);
		if(active_low)
		{
			PDEBUG("Active low set\n");
			reg_data = reg_data & EDGE_TRIGGER;
			reg_data = reg_data & FALLING_EDGE;
			//PDEBUG("reg_data=%x\n",reg_data);
		        handler = handle_edge_irq;
		}
		else
		{

			PDEBUG("Active low not set\n");

			reg_data = reg_data &  EDGE_TRIGGER;
			//PDEBUG("reg_data=%x\n",reg_data);
			reg_data = reg_data | RISING_EDGE;
			//PDEBUG("reg_data=%x\n",reg_data);
		        handler = handle_edge_irq;
		}
		break;
#endif

                printk("pilot-gpio: irq%d: unsupported type %d\n",
                        d->irq, type);
                return -EINVAL;

	case IRQ_TYPE_EDGE_RISING :
		reg_data = reg_data &  EDGE_TRIGGER;
		reg_data = reg_data | RISING_EDGE;
	        handler = handle_edge_irq;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		reg_data = reg_data & EDGE_TRIGGER;
		reg_data = reg_data & FALLING_EDGE;
	        handler = handle_edge_irq;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		reg_data = reg_data | (LEVEL_TRIGGER | HIGH_LEVEL);
		handler = handle_level_irq;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		reg_data = reg_data | (LEVEL_TRIGGER);
		reg_data = reg_data & (LOW_LEVEL);
		handler = handle_level_irq;
		break;

	default:
		PDEBUG("%s: INVALID type=%d\n",__func__,type);
		return -EINVAL;
	}

	irq_set_handler_locked(d, handler);
	reg_data = reg_data |  GPIO_EVENT_TO_IRQ_ENABLE;
	DEBUG_PRINT(reg_addr);
	iowrite8(reg_data, reg_addr);
	DEBUG_PRINT(reg_addr);
return 0;
}


static void __pilot_gpio_irq_set_mask(struct irq_data *d, bool set)
{
	char *reg_addr,reg_data;
	struct pilot_gpio *gpio;
	struct pilot_gpio_pin gpio_pin;
	unsigned int offset;

	gpio=irq_data_get_irq_chip_data(d);
	offset = irqd_to_hwirq(d);
	get_pin_details(&gpio_pin,offset);
	gpio_pin.gpio_base = (char *) gpio->base;

	reg_addr = pilot_gpio_port_reg(&gpio_pin,GPEVEN);
	reg_data = ioread8(reg_addr) ;
	DEBUG_PRINT(reg_addr);
        if (set)
                reg_data |= GPIO_BIT_SET( gpio_pin.pin_number);
        else
                reg_data &= GPIO_BIT_CLR(gpio_pin.pin_number);
        iowrite8(reg_data,reg_addr);
	DEBUG_PRINT(reg_addr);
}


static void pilot_gpio_irq_mask(struct irq_data *d)
{
	 __pilot_gpio_irq_set_mask(d, false);
	return ;

}
static void pilot_gpio_irq_unmask(struct irq_data *d)
{
	__pilot_gpio_irq_set_mask(d, true);
	return ;

}


static void pilot_gpio_irq_handler(struct irq_desc *desc)
{
	struct pilot_gpio *gpio = (struct pilot_gpio *)irq_desc_get_handler_data(desc);
	struct irq_chip *gpio_chip = irq_desc_get_chip(desc);
	int i,j;
	unsigned char *gpisr_address,gpisr_status;
	unsigned char *gpevst_addr,gpevst_status;
	unsigned char *gpeven_addr,gpeven_ctrlwd;
	unsigned int bitnum;
	unsigned int hwirq,linux_irq;
	struct pilot_gpio_pin gpio_pin;

	memset(&gpio_pin,0,sizeof(gpio_pin));
	chained_irq_enter(gpio_chip, desc);
	gpio_pin.gpio_base = (char *) gpio->base;
	for(i=0;i<6;i++)
	{
		//for each gpio port, read combined irq status register of the port
		gpisr_address = gpio->base + comined_irq_status_reg_offset[i];
		gpisr_status = ioread8(gpisr_address);
		DEBUG_PRINT(gpisr_address);
		if(!gpisr_status)
			continue;
		else
		{
			// for each bit set in  gpisr_status find port number
			for(j=0;j<=7;j++)
			{
				if( gpisr_status & BIT(j) )
				{
					gpio_pin.port_number = GPISR_port_number_lookup[i][j];
					if( (gpio_pin.port_number >= BANK0_FIRST_PORT) && (gpio_pin.port_number <= BANK0_LAST_PORT) )
					{
						gpio_pin.bank_number = 0;
						gpio_pin.ports_index_in_bank = gpio_pin.port_number;
						gpio_pin.first_gpio_in_bank = BANK0_FIRST_GPIO;
					}
					else if ( (gpio_pin.port_number >= BANK1_FIRST_PORT) && (gpio_pin.port_number <= BANK1_LAST_PORT) )
					{
						gpio_pin.bank_number = 1;
						gpio_pin.ports_index_in_bank = gpio_pin.port_number - BANK1_FIRST_PORT;
						gpio_pin.first_gpio_in_bank = BANK1_FIRST_GPIO;
					}
					else if ( (gpio_pin.port_number >= BANK2_FIRST_PORT) && (gpio_pin.port_number <= BANK2_LAST_PORT) )
					{
						gpio_pin.bank_number = 2;
						gpio_pin.ports_index_in_bank = gpio_pin.port_number - BANK2_FIRST_PORT;
						gpio_pin.first_gpio_in_bank = BANK2_FIRST_GPIO;
					}

					gpio_pin.bank_offset_addr= gpio_bank_base_addroffsets[gpio_pin.bank_number];
					gpio_pin.port_offset_addr= gpio_pin.bank_offset_addr + (gpio_pin.ports_index_in_bank*16);
					// find pin number from EVST reg of that port
					gpevst_addr = gpio->base + gpio_pin.port_offset_addr + GPEVST;
					gpevst_status = ioread8(gpevst_addr );
					DEBUG_PRINT(gpevst_addr);
					gpeven_addr = gpio->base + gpio_pin.port_offset_addr + GPEVEN;
					gpeven_ctrlwd = ioread8(gpeven_addr);
					DEBUG_PRINT(gpeven_addr);

					for(bitnum = 0 ; bitnum <=7; bitnum++ )
					{
						if( (gpeven_ctrlwd & BIT(bitnum)) && (gpevst_status & BIT(bitnum)) )
						{
							gpio_pin.pin_number = bitnum;
							hwirq = gpio_pin.first_gpio_in_bank + (gpio_pin.ports_index_in_bank*8) + gpio_pin.pin_number;
							if( (gpeven_ctrlwd & BIT(bitnum)) && (hwirq >=  0 &&  hwirq <= 185))
							{
								unsigned char status_clr_byte = 0;	
								linux_irq = irq_linear_revmap(gpio->irq_domain,hwirq);
								PDEBUG("Event on bank_number=%d port_number=%d pin=%d\n",gpio_pin.bank_number,gpio_pin.port_number,gpio_pin.pin_number);
								PDEBUG("hwirq=%d linux_irq=%d\n",hwirq,linux_irq);
								status_clr_byte =  GPIO_BIT_SET(gpio_pin.pin_number);
								iowrite8( status_clr_byte, gpevst_addr );
								generic_handle_irq(linux_irq);

							}
							else
							{
								if(!(hwirq >=  0 &&  hwirq <= 185))
									PDEBUG("Wrong hwirq read from status reg\n");
							}
	
						}
					}
				}
			}
		}
	}
	chained_irq_exit(gpio_chip, desc);
	return ;
}


static struct irq_chip pilot_gpio_irqchip = {
        .name           = "pilot-gpio-irq",
        .irq_ack        = pilot_gpio_irq_ack,
        .irq_mask       = pilot_gpio_irq_mask,
        .irq_unmask     = pilot_gpio_irq_unmask,
        .irq_set_type   = pilot_gpio_set_type,
};


static int  pilot_gpio_setup_irqs(struct pilot_gpio *gpio,
			                struct platform_device *pdev)
{

	int hw_irq;
	int linux_irq;
	char *reg_addr,reg_data;
        /* get  upstream IRQ */
        gpio->irq = platform_get_irq(pdev, 0);
        if (gpio->irq < 0)
	{
		PDEBUG("Error:unable to get IRQ\n");
		return -1;
	}
	gpio->irq_domain = irq_domain_add_linear(pdev->dev.of_node,
		gpio->chip.ngpio, &irq_domain_simple_ops, NULL);
        if (!gpio->irq_domain)
	{
		PDEBUG("Error: Unable to get IRQ-domain");
		return -1;
	}


	//for each port
	for(hw_irq = 0; hw_irq < gpio->chip.ngpio; hw_irq++)
	{
		struct pilot_gpio_pin gpio_pin;
		get_pin_details(&gpio_pin,hw_irq);
		gpio_pin.gpio_base = (char *) gpio->base;
		//clear EVST register of port
		if(gpio_pin.pin_number == 0)
		{
			reg_addr = pilot_gpio_port_reg(&gpio_pin,GPEVST);
			reg_data = ioread8(reg_addr);
			DEBUG_PRINT(reg_addr);
			iowrite8( 0xFF,reg_addr );
			DEBUG_PRINT(reg_addr);
		}
		//clear EVEN reg of port(if needed)
	}

	for (hw_irq = 0; hw_irq < gpio->chip.ngpio; hw_irq++)
	{
		//map hwirq to linux space irq ,in otherwords map irq number of a pvt(irq chip)domain space to linux space
		linux_irq = irq_create_mapping(gpio->irq_domain,hw_irq );
		if(!linux_irq)
		{
			PDEBUG("Error: irq mapping for gpio%d Failed\n",hw_irq);
			return -1;
		}
		irq_set_chip_data(linux_irq, gpio);
		irq_set_chip_and_handler(linux_irq,&pilot_gpio_irqchip,handle_simple_irq);
		// set Interrupt autoprobing
		irq_set_probe(linux_irq);
	}
	irq_set_chained_handler_and_data(gpio->irq,pilot_gpio_irq_handler, gpio);
	return 0;
}


static int pilot_gpio_probe(struct platform_device *pdev)
{
        struct pilot_gpio *gpio=NULL;
        struct resource *res;
        int rc;
	int ret;

        gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
        if (!gpio)
                return -ENOMEM;
        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!res)
                return -ENXIO;
	PDEBUG("res->start=%x res->end=%x\n",(unsigned int)res->start,(unsigned int)res->end );
        gpio->base = devm_ioremap_resource(&pdev->dev, res);
        if (!gpio->base)
                return -ENOMEM;
	PDEBUG("gpio->base =%x\n",(unsigned int)gpio->base);

	gpio->chip.ngpio = MAX_NUM_GPIOS;
        gpio->chip.base = -1;
        gpio->chip.label = dev_name(&pdev->dev);
	gpio->chip.parent = &pdev->dev;

	gpio->chip.request = pilot_gpio_request;
	gpio->chip.free = pilot_gpio_free;
        gpio->chip.direction_input = pilot_gpio_dir_in;
        gpio->chip.direction_output = pilot_gpio_dir_out;
        gpio->chip.get = pilot_gpio_get;
        gpio->chip.set = pilot_gpio_set;
	gpio->chip.set_config = pilot_gpio_set_config;
	gpio->chip.of_xlate = pilot_gpio_of_xlate;
        gpio->chip.to_irq = pilot_gpio_to_irq;

	gpio->chip.of_gpio_n_cells = 5;
	gpio->chip.need_valid_mask = false;

        platform_set_drvdata(pdev, gpio);
	rc = gpiochip_add_data(&gpio->chip,gpio);
        if (rc < 0)
                return rc;
        ret = pilot_gpio_setup_irqs(gpio, pdev);
	if(ret < 0)
	{
		PDEBUG("pilot-gpio: pilot_gpio_setup_irqs Failed\n");
		return ret;
	}
        return 0;
}

static const struct of_device_id pilot_gpio_of_table[] = {
        { .compatible = "aspeed,pilot-gpio" },
        {},
};
MODULE_DEVICE_TABLE(of, pilot_gpio_of_table);


static struct platform_driver pilot_gpio_driver = {
        .driver = {
                .name = KBUILD_MODNAME,
                .of_match_table = pilot_gpio_of_table,
        },
	.probe = pilot_gpio_probe,
};
module_platform_driver(pilot_gpio_driver);

MODULE_DESCRIPTION("Aspeed pilot4 GPIO Driver");
MODULE_LICENSE("GPL");
