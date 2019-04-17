/*
 * Copyright (C) 2019 ASPEED Technology Inc
 * sudheer.veliseti@aspeedtech.com
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/**
 * struct aspeed_pilot_function - pinctrl mux function
 * @name: The name of the function, exported to pinctrl core.
 * @groups: An array of pin groups that may select this function.
 * @ngroups: The number of entries in @groups.
 */
/*
struct aspeed_pilot_function {
        const char *name;
        const char **groups;
        unsigned int ngroups;
	short int gpioen_offset;
	short int gpioen_bitnum;
	short int gpioen_bitval;
	short int strap_type ;
	short int pinmux_ctrl_bit_num;
	short int pinmux_ctrl_bit_val;
	short int ioengccfg_bit_num;
	short int ioengccfg_bit_val;
};
*/
struct aspeed_pilot_function {
        const char *name;
        const char **groups;
        char ngroups;
	signed char  gpioen_offset;
	signed char gpioen_bitnum;
	signed char gpioen_bitval;
	signed char strap_type ;
	signed char pinmux_ctrl_bit_num;
	signed char pinmux_ctrl_bit_val;
	signed char ioengccfg_bit_num;
	signed char ioengccfg_bit_val;
};


/*
struct  aspeed_pilot_pin_group {
	const char *gname;
	const unsigned *pins;
        u8 npins;
	int  funcs[4];
	short int pinctrlreg_offset;
	short int padctrl_type;

};
*/


struct  aspeed_pilot_pin_group {
	const char *gname;
	const unsigned *pins;
        u8 npins;
	short  funcs[3];
	signed char pinctrlreg_offset;
	signed char padctrl_type;

};


/**
 * struct aspeed_pilot_pinctrl_soc_data - Tegra pin controller driver configuration
 * @ngpios:     The number of GPIO pins the pin controller HW affects.
 * @pins:       An array describing all pins the pin controller affects.
 *              All pins which are also GPIOs must be listed first within the
 *              array, and be numbered identically to the GPIO controller's
 *              numbering.
 * @npins:      The numbmer of entries in @pins.
 * @functions:  An array describing all mux functions the SoC supports.
 * @nfunctions: The numbmer of entries in @functions.
 * @groups:     An array describing all pin groups the pin SoC supports.
 * @ngroups:    The numbmer of entries in @groups.
 */
struct aspeed_pilot_pinctrl_soc_data {
        unsigned ngpios;
        const struct pinctrl_pin_desc *pins;
        unsigned npins;
        struct aspeed_pilot_function *functions;
        unsigned nfunctions;
        const struct aspeed_pilot_pin_group *groups;
        unsigned ngroups;
        bool hsm_in_mux;
        bool schmitt_in_mux;
        bool drvtype_in_mux;
};




struct aspeed_pilot_pinmux {
        struct device *dev;
        struct pinctrl_dev *pctrldev;
	const struct aspeed_pilot_pinctrl_soc_data *soc;

	const char **func_groups;

	struct regmap 	*map;	
	void __iomem    *membase;
#if 0
	__iomem    *systemlevel_and_clockcontrol;
	void __iomem    *top_level_pin_control;
	void __iomem    *top_level_pad_control;
#endif
	void __iomem    *gpio_base;

};

#define SYSTEM_CLK_CTL	0x000
#define TOP_LVL_PINCTL	0x800
#define TOP_LVL_PADCTL	0xA00

enum  {
	NO_STRAP  = -1,
	OEM_STRAP = 0,
	DDR3_MODE_STRAP = 1,
	ROM_BOOT_STRAP = 2,
	MAC1_MODE_STRAP = 3,
	MAC0_MODE_STRAP = 4,
	EXT_BUS_STRAP = 5,
	EEPROM_STRAP = 8,
	eSPI_STRAP = 14,
	SSPJTAG_STRAP = 15,
	WATCHDOG_RESET_STRAP = 17,
	HOST_SPI_DISABLE_STRAP = 19,
	UART4_STRAP = 20,
};

enum {
	OEM_MAC0_MODE_STRAP = 7,
	OEM_MAC1_MODE_STRAP = 8,
	OEM_NAND_STRAP = 6,
	OEM_SD2_STRAP = 5,
	OEM_UART4_STRAP = 4,
	OEM_EPROM_STRAP = 3,
	OEM_SSP_JTAG_STRAP = 2,
};

enum
{
  PAD_TYPE1 = 1,
  PAD_TYPE2,
  PAD_TYPE3,
};

#define DRIVESTRENGTH_MASK 0xF3
#define BIASPULL_MASK 0xFC

// System level & clock control register map
#define STRPSTS 0X0C
#define OSCTL  0x88
#define IOENGCCFG 0x30


//Top level pad control register map
// in group table

// TOP level pin control register map
#define  GPIOEN0 0x00
#define  GPIOEN1 0x01
#define  GPIOEN2 0x08
#define  GPIOEN3 0x0C
#define  GPIOEN4 0x10
#define  SWSTRAPCTL 0x18
#define  LPCPINCTL 0x20
#define  STRENGTH_CTRL2 0x28
#define  PINMUX_CTRL 0x30
#define  OSCTL       0x88

int aspeed_pilot_pinctrl_probe(struct platform_device *pdev,const struct aspeed_pilot_pinctrl_soc_data *aspeed_pilot_soc_data );

int aspeed_pilot_pin_config_group_set (struct pinctrl_dev *pctldev,unsigned selector,unsigned long *configs,unsigned num_configs);

int aspeed_pilot_pin_config_set(struct pinctrl_dev *pctldev, unsigned int pin,unsigned long *configs, unsigned int num_configs);

int aspeed_pilot_pinmux_set_mux(struct pinctrl_dev *pctldev, unsigned int func_selector,unsigned int group_selector);


int aspeed_pilot_pinmux_get_fn_groups(struct pinctrl_dev *pctldev,unsigned int function,const char * const **groups,unsigned int * const num_groups);

const char *aspeed_pilot_pinmux_get_fn_name(struct pinctrl_dev *pctldev,unsigned int function);
int aspeed_pilot_pinmux_get_fn_count(struct pinctrl_dev *pctldev);

//#define PINCTRL_DEBUG_PRINTS 1

#undef PDEBUG
#ifdef PINCTRL_DEBUG_PRINTS
	#define PDEBUG(fmt, args...) printk("pilot-pinctrl:"  fmt , ##args)
#else
	#define PDEBUG(fmt, args...)
#endif



