/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <asm/div64.h>
#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/aspeed.h>
#include <linux/hashtable.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/string.h>

/*
 * These two headers aren't meant to be used by GPIO drivers. We need
 * them in order to access gpio_chip_hwgpio() which we need to implement
 * the aspeed specific API which allows the coprocessor to request
 * access to some GPIOs and to arbitrate between coprocessor and ARM.
 */
#include <linux/gpio/consumer.h>
#include "gpiolib.h"

struct aspeed_bank_props {
	unsigned int bank;
	u32 input;
	u32 output;
};

struct aspeed_sgpio_config {
	unsigned int nr_gpios;
	const struct aspeed_bank_props *props;
};

struct aspeed_gpio {
	struct gpio_chip chip;
	struct irq_chip aspeed_sgpio_irqchip;
	spinlock_t lock;
	void __iomem *base;
	int irq;
	const struct aspeed_sgpio_config *config;

	unsigned int timer_users[4];
	struct clk *clk;

	u32 *dcache;
};

struct aspeed_sgpio_bank {
	uint16_t	val_regs;	/* +0: Rd: read input value, Wr: set write latch
					 * +4: Rd/Wr: Direction (0=in, 1=out)
					 */
	uint16_t	rdata_reg;	/*     Rd: read write latch, Wr: <none>  */
	uint16_t	irq_regs;
	uint16_t	tolerance_regs;
	const char	names[4][3];
};

/*
 * Note: The "value" register returns the input value sampled on the
 *       line even when the GPIO is configured as an output. Since
 *       that input goes through synchronizers, writing, then reading
 *       back may not return the written value right away.
 *
 *       The "rdata" register returns the content of the write latch
 *       and thus can be used to read back what was last written
 *       reliably.
 */

static const struct aspeed_sgpio_bank aspeed_sgpio_banks[] = {
	{
		.val_regs = 0x0000,
		.rdata_reg = 0x0070,
		.irq_regs = 0x0004,
		.tolerance_regs = 0x0018,
		.names = { "A", "B", "C", "D" },
	},
	{
		.val_regs = 0x001C,
		.rdata_reg = 0x0074,
		.irq_regs = 0x0020,
		.tolerance_regs = 0x0034,
		.names = { "E", "F", "G", "H" },
	},
	{
		.val_regs = 0x0038,
		.rdata_reg = 0x0078,
		.irq_regs = 0x003c,
		.tolerance_regs = 0x0050,
		.names = { "I", "J", "", "" },
	},
};

enum aspeed_sgpio_reg {
	reg_val,
	reg_rdata,
	reg_dir,
	reg_irq_enable,
	reg_irq_type0,
	reg_irq_type1,
	reg_irq_type2,
	reg_irq_status,
	reg_tolerance,
};

#define GPIO_VAL_VALUE	0x00
#define GPIO_VAL_DIR	0x04

#define GPIO_IRQ_ENABLE	0x00
#define GPIO_IRQ_TYPE0	0x04
#define GPIO_IRQ_TYPE1	0x08
#define GPIO_IRQ_TYPE2	0x0c
#define GPIO_IRQ_STATUS	0x10

/* This will be resolved at compile time */
static inline void __iomem *bank_reg(struct aspeed_gpio *gpio,
				     const struct aspeed_sgpio_bank *bank,
				     const enum aspeed_sgpio_reg reg)
{
	switch (reg) {
	case reg_val:
		return gpio->base + bank->val_regs + GPIO_VAL_VALUE;
	case reg_rdata:
		return gpio->base + bank->rdata_reg;
	case reg_dir:
		return gpio->base + bank->val_regs + GPIO_VAL_DIR;
	case reg_irq_enable:
		return gpio->base + bank->irq_regs + GPIO_IRQ_ENABLE;
	case reg_irq_type0:
		return gpio->base + bank->irq_regs + GPIO_IRQ_TYPE0;
	case reg_irq_type1:
		return gpio->base + bank->irq_regs + GPIO_IRQ_TYPE1;
	case reg_irq_type2:
		return gpio->base + bank->irq_regs + GPIO_IRQ_TYPE2;
	case reg_irq_status:
		return gpio->base + bank->irq_regs + GPIO_IRQ_STATUS;
	case reg_tolerance:
		return gpio->base + bank->tolerance_regs;
	}
	BUG();
}

#define GPIO_BANK(x)	((x) >> 5)
#define GPIO_OFFSET(x)	((x) & 0x1f)
#define GPIO_BIT(x)	BIT(GPIO_OFFSET(x))

static const struct aspeed_sgpio_bank *to_bank(unsigned int offset)
{
	unsigned int bank = GPIO_BANK(offset);

	WARN_ON(bank >= ARRAY_SIZE(aspeed_sgpio_banks));
	return &aspeed_sgpio_banks[bank];
}

static inline bool is_bank_props_sentinel(const struct aspeed_bank_props *props)
{
	return !(props->input || props->output);
}

static inline const struct aspeed_bank_props *find_bank_props(
		struct aspeed_gpio *gpio, unsigned int offset)
{
	const struct aspeed_bank_props *props = gpio->config->props;

	while (!is_bank_props_sentinel(props)) {
		if (props->bank == GPIO_BANK(offset))
			return props;
		props++;
	}

	return NULL;
}

static inline bool have_gpio(struct aspeed_gpio *gpio, unsigned int offset)
{
	const struct aspeed_bank_props *props = find_bank_props(gpio, offset);
	const struct aspeed_sgpio_bank *bank = to_bank(offset);
	unsigned int group = GPIO_OFFSET(offset) / 8;

	return bank->names[group][0] != '\0' &&
		(!props || ((props->input | props->output) & GPIO_BIT(offset)));
}

static inline bool have_input(struct aspeed_gpio *gpio, unsigned int offset)
{
	const struct aspeed_bank_props *props = find_bank_props(gpio, offset);

	return !props || (props->input & GPIO_BIT(offset));
}

#define have_irq(g, o) have_input((g), (o))
#define have_debounce(g, o) have_input((g), (o))

static inline bool have_output(struct aspeed_gpio *gpio, unsigned int offset)
{
	const struct aspeed_bank_props *props = find_bank_props(gpio, offset);

	return !props || (props->output & GPIO_BIT(offset));
}

static int aspeed_sgpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct aspeed_gpio *gpio = gpiochip_get_data(gc);
	const struct aspeed_sgpio_bank *bank = to_bank(offset);

	return !!(ioread32(bank_reg(gpio, bank, reg_val)) & GPIO_BIT(offset));
}

static void __aspeed_sgpio_set(struct gpio_chip *gc, unsigned int offset,
			      int val)
{
	struct aspeed_gpio *gpio = gpiochip_get_data(gc);
	const struct aspeed_sgpio_bank *bank = to_bank(offset);
	void __iomem *addr;
	u32 reg;

	addr = bank_reg(gpio, bank, reg_val);
	reg = gpio->dcache[GPIO_BANK(offset)];

	if (val)
		reg |= GPIO_BIT(offset);
	else
		reg &= ~GPIO_BIT(offset);
	gpio->dcache[GPIO_BANK(offset)] = reg;

	iowrite32(reg, addr);
}

static void aspeed_sgpio_set(struct gpio_chip *gc, unsigned int offset,
			    int val)
{
	struct aspeed_gpio *gpio = gpiochip_get_data(gc);
	unsigned long flags;

	spin_lock_irqsave(&gpio->lock, flags);
	__aspeed_sgpio_set(gc, offset, val);
	spin_unlock_irqrestore(&gpio->lock, flags);
}

static int aspeed_sgpio_dir_in(struct gpio_chip *gc, unsigned int offset)
{
	struct aspeed_gpio *gpio = gpiochip_get_data(gc);
	const struct aspeed_sgpio_bank *bank = to_bank(offset);
	void __iomem *addr = bank_reg(gpio, bank, reg_dir);
	unsigned long flags;
	u32 reg;

	if (!have_input(gpio, offset))
		return -ENOTSUPP;

	spin_lock_irqsave(&gpio->lock, flags);

	reg = ioread32(addr);
	reg &= ~GPIO_BIT(offset);

	spin_unlock_irqrestore(&gpio->lock, flags);

	return 0;
}

static int aspeed_sgpio_dir_out(struct gpio_chip *gc,
			       unsigned int offset, int val)
{
	struct aspeed_gpio *gpio = gpiochip_get_data(gc);
	const struct aspeed_sgpio_bank *bank = to_bank(offset);
	void __iomem *addr = bank_reg(gpio, bank, reg_dir);
	unsigned long flags;
	u32 reg;

	if (!have_output(gpio, offset))
		return -ENOTSUPP;

	spin_lock_irqsave(&gpio->lock, flags);

	reg = ioread32(addr);
	reg |= GPIO_BIT(offset);

	__aspeed_sgpio_set(gc, offset, val);
	iowrite32(reg, addr);

	spin_unlock_irqrestore(&gpio->lock, flags);

	return 0;
}

static int aspeed_sgpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct aspeed_gpio *gpio = gpiochip_get_data(gc);
	const struct aspeed_sgpio_bank *bank = to_bank(offset);
	unsigned long flags;
	u32 val;

	if (!have_input(gpio, offset))
		return 0;

	if (!have_output(gpio, offset))
		return 1;

	spin_lock_irqsave(&gpio->lock, flags);

	val = ioread32(bank_reg(gpio, bank, reg_dir)) & GPIO_BIT(offset);

	spin_unlock_irqrestore(&gpio->lock, flags);

	return !val;

}

static inline int irqd_to_aspeed_sgpio_data(struct irq_data *d,
					   struct aspeed_gpio **gpio,
					   const struct aspeed_sgpio_bank **bank,
					   u32 *bit, int *offset)
{
	struct aspeed_gpio *internal;

	*offset = irqd_to_hwirq(d);

	internal = irq_data_get_irq_chip_data(d);

	/* This might be a bit of a questionable place to check */
	if (!have_irq(internal, *offset))
		return -ENOTSUPP;

	*gpio = internal;
	*bank = to_bank(*offset);
	*bit = GPIO_BIT(*offset);

	return 0;
}

static void aspeed_sgpio_irq_ack(struct irq_data *d)
{
	const struct aspeed_sgpio_bank *bank;
	struct aspeed_gpio *gpio;
	unsigned long flags;
	void __iomem *status_addr;
	int rc, offset;
	u32 bit;

	rc = irqd_to_aspeed_sgpio_data(d, &gpio, &bank, &bit, &offset);
	if (rc)
		return;

	status_addr = bank_reg(gpio, bank, reg_irq_status);

	spin_lock_irqsave(&gpio->lock, flags);

	iowrite32(bit, status_addr);

	spin_unlock_irqrestore(&gpio->lock, flags);
}

static void aspeed_sgpio_irq_set_mask(struct irq_data *d, bool set)
{
	const struct aspeed_sgpio_bank *bank;
	struct aspeed_gpio *gpio;
	unsigned long flags;
	u32 reg, bit;
	void __iomem *addr;
	int rc, offset;

	rc = irqd_to_aspeed_sgpio_data(d, &gpio, &bank, &bit, &offset);
	if (rc)
		return;

	addr = bank_reg(gpio, bank, reg_irq_enable);

	spin_lock_irqsave(&gpio->lock, flags);

	reg = ioread32(addr);
	if (set)
		reg |= bit;
	else
		reg &= ~bit;
	iowrite32(reg, addr);

	spin_unlock_irqrestore(&gpio->lock, flags);
}

static void aspeed_sgpio_irq_mask(struct irq_data *d)
{
	aspeed_sgpio_irq_set_mask(d, false);
}

static void aspeed_sgpio_irq_unmask(struct irq_data *d)
{
	aspeed_sgpio_irq_set_mask(d, true);
}

static int aspeed_sgpio_set_type(struct irq_data *d, unsigned int type)
{
	u32 type0 = 0;
	u32 type1 = 0;
	u32 type2 = 0;
	u32 bit, reg;
	const struct aspeed_sgpio_bank *bank;
	irq_flow_handler_t handler;
	struct aspeed_gpio *gpio;
	unsigned long flags;
	void __iomem *addr;
	int rc, offset;

	rc = irqd_to_aspeed_sgpio_data(d, &gpio, &bank, &bit, &offset);
	if (rc)
		return -EINVAL;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_BOTH:
		type2 |= bit;
		/* fall through */
	case IRQ_TYPE_EDGE_RISING:
		type0 |= bit;
		/* fall through */
	case IRQ_TYPE_EDGE_FALLING:
		handler = handle_edge_irq;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		type0 |= bit;
		/* fall through */
	case IRQ_TYPE_LEVEL_LOW:
		type1 |= bit;
		handler = handle_level_irq;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&gpio->lock, flags);

	addr = bank_reg(gpio, bank, reg_irq_type0);
	reg = ioread32(addr);
	reg = (reg & ~bit) | type0;
	iowrite32(reg, addr);

	addr = bank_reg(gpio, bank, reg_irq_type1);
	reg = ioread32(addr);
	reg = (reg & ~bit) | type1;
	iowrite32(reg, addr);

	addr = bank_reg(gpio, bank, reg_irq_type2);
	reg = ioread32(addr);
	reg = (reg & ~bit) | type2;
	iowrite32(reg, addr);

	spin_unlock_irqrestore(&gpio->lock, flags);

	irq_set_handler_locked(d, handler);

	return 0;
}

static void aspeed_sgpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct irq_chip *ic = irq_desc_get_chip(desc);
	struct aspeed_gpio *data = gpiochip_get_data(gc);
	unsigned int i, p, girq;
	unsigned long reg;

	chained_irq_enter(ic, desc);

	for (i = 0; i < ARRAY_SIZE(aspeed_sgpio_banks); i++) {
		const struct aspeed_sgpio_bank *bank = &aspeed_sgpio_banks[i];

		reg = ioread32(bank_reg(data, bank, reg_irq_status));

		for_each_set_bit(p, &reg, 32) {
			girq = irq_find_mapping(gc->irq.domain, i * 32 + p);
			generic_handle_irq(girq);
		}

	}

	chained_irq_exit(ic, desc);
}

static void set_irq_valid_mask(struct aspeed_gpio *gpio)
{
	const struct aspeed_bank_props *props = gpio->config->props;

	while (!is_bank_props_sentinel(props)) {
		unsigned int offset;
		const unsigned long int input = props->input;

		/* Pretty crummy approach, but similar to GPIO core */
		for_each_clear_bit(offset, &input, 32) {
			unsigned int i = props->bank * 32 + offset;

			if (i >= gpio->config->nr_gpios)
				break;

			clear_bit(i, gpio->chip.irq.valid_mask);
		}

		props++;
	}
}

static int aspeed_sgpio_setup_irqs(struct aspeed_gpio *gpio,
		struct platform_device *pdev)
{
	int rc;

	rc = platform_get_irq(pdev, 0);
	if (rc < 0)
		return rc;

	gpio->irq = rc;

	set_irq_valid_mask(gpio);

	rc = gpiochip_irqchip_add(&gpio->chip, &gpio->aspeed_sgpio_irqchip,
			0, handle_bad_irq, IRQ_TYPE_NONE);
	if (rc) {
		dev_info(&pdev->dev, "Could not add irqchip\n");
		return rc;
	}

	gpiochip_set_chained_irqchip(&gpio->chip, &gpio->aspeed_sgpio_irqchip,
				     gpio->irq, aspeed_sgpio_irq_handler);

	return 0;
}

static int aspeed_sgpio_reset_tolerance(struct gpio_chip *chip,
					unsigned int offset, bool enable)
{
	struct aspeed_gpio *gpio = gpiochip_get_data(chip);
	unsigned long flags;
	void __iomem *treg;
	u32 val;

	treg = bank_reg(gpio, to_bank(offset), reg_tolerance);

	spin_lock_irqsave(&gpio->lock, flags);

	val = readl(treg);

	if (enable)
		val |= GPIO_BIT(offset);
	else
		val &= ~GPIO_BIT(offset);

	writel(val, treg);

	spin_unlock_irqrestore(&gpio->lock, flags);

	return 0;
}

static int aspeed_sgpio_request(struct gpio_chip *chip, unsigned int offset)
{
	if (!have_gpio(gpiochip_get_data(chip), offset))
		return -ENODEV;

	return pinctrl_gpio_request(chip->base + offset);
}

static void aspeed_sgpio_free(struct gpio_chip *chip, unsigned int offset)
{
	pinctrl_gpio_free(chip->base + offset);
}

static int aspeed_sgpio_set_config(struct gpio_chip *chip, unsigned int offset,
				  unsigned long config)
{
	unsigned long param = pinconf_to_config_param(config);
	u32 arg = pinconf_to_config_argument(config);

	if (param == PIN_CONFIG_BIAS_DISABLE ||
			param == PIN_CONFIG_BIAS_PULL_DOWN ||
			param == PIN_CONFIG_DRIVE_STRENGTH)
		return pinctrl_gpio_set_config(offset, config);
	else if (param == PIN_CONFIG_DRIVE_OPEN_DRAIN ||
			param == PIN_CONFIG_DRIVE_OPEN_SOURCE)
		/* Return -ENOTSUPP to trigger emulation, as per datasheet */
		return -ENOTSUPP;
	else if (param == PIN_CONFIG_PERSIST_STATE)
		return aspeed_sgpio_reset_tolerance(chip, offset, arg);

	return -ENOTSUPP;
}

/*
 * Any banks not specified in a struct aspeed_bank_props array are assumed to
 * have the properties:
 *
 *     { .input = 0xffffffff, .output = 0xffffffff }
 */

static const struct aspeed_bank_props ast2400_bank_props[] = {
	/*     input	  output   */
	{ 5, 0xffffffff, 0x0000ffff }, /* U/V/W/X */
	{ 6, 0x0000000f, 0x0fffff0f }, /* Y/Z/AA/AB, two 4-GPIO holes */
	{ },
};

static const struct aspeed_sgpio_config ast2400_config =
	/* 220 for simplicity, really 216 with two 4-GPIO holes, four at end */
	{ .nr_gpios = 80, .props = ast2400_bank_props, };

static const struct aspeed_bank_props ast2500_bank_props[] = {
	/*     input	  output   */
	{ 5, 0xffffffff, 0x0000ffff }, /* U/V/W/X */
	{ 6, 0x0fffffff, 0x0fffffff }, /* Y/Z/AA/AB, 4-GPIO hole */
	{ },
};

static const struct aspeed_sgpio_config ast2500_config =
	/* 232 for simplicity, actual number is 228 (4-GPIO hole in GPIOAB) */
	{ .nr_gpios = 80, .props = ast2500_bank_props, };

static const struct aspeed_bank_props ast2600_bank_props[] = {
	/*     input	  output   */
	{ 5, 0xffffffff, 0x0000ffff }, /* U/V/W/X */
	{ 6, 0x0000ffff, 0x0000ffff }, /* Y/Z */
	{ },
};

static const struct aspeed_sgpio_config ast2600_config =
	/* 182 for simplicity, actual number is xx (4-GPIO hole in GPIOAB) */
	{ .nr_gpios = 80, .props = ast2600_bank_props, };


static const struct of_device_id aspeed_sgpio_of_table[] = {
	{ .compatible = "aspeed,ast2400-sgpio", .data = &ast2400_config, },
	{ .compatible = "aspeed,ast2500-sgpio", .data = &ast2500_config, },
	{ .compatible = "aspeed,ast2600-sgpio", .data = &ast2600_config, },
	{}
};
MODULE_DEVICE_TABLE(of, aspeed_sgpio_of_table);

static int __init aspeed_sgpio_probe(struct platform_device *pdev)
{
	const struct of_device_id *gpio_id;
	struct aspeed_gpio *gpio;
	struct resource *res;
	int rc, i, banks;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gpio->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gpio->base))
		return PTR_ERR(gpio->base);

	spin_lock_init(&gpio->lock);

	gpio_id = of_match_node(aspeed_sgpio_of_table, pdev->dev.of_node);
	if (!gpio_id)
		return -EINVAL;

	gpio->clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(gpio->clk)) {
		dev_warn(&pdev->dev,
				"Failed to get clock from devicetree, debouncing disabled\n");
		gpio->clk = NULL;
	}

	gpio->config = gpio_id->data;

	gpio->chip.parent = &pdev->dev;
	gpio->chip.ngpio = gpio->config->nr_gpios;
	gpio->chip.direction_input = aspeed_sgpio_dir_in;
	gpio->chip.direction_output = aspeed_sgpio_dir_out;
	gpio->chip.get_direction = aspeed_sgpio_get_direction;
	gpio->chip.request = aspeed_sgpio_request;
	gpio->chip.free = aspeed_sgpio_free;
	gpio->chip.get = aspeed_sgpio_get;
	gpio->chip.set = aspeed_sgpio_set;
	gpio->chip.set_config = aspeed_sgpio_set_config;
	gpio->chip.label = dev_name(&pdev->dev);
	gpio->chip.base = -1;
	gpio->chip.irq.need_valid_mask = true;

	gpio->aspeed_sgpio_irqchip.name = pdev->name;
	gpio->aspeed_sgpio_irqchip.irq_ack = aspeed_sgpio_irq_ack;
	gpio->aspeed_sgpio_irqchip.irq_mask	= aspeed_sgpio_irq_mask;
	gpio->aspeed_sgpio_irqchip.irq_unmask = aspeed_sgpio_irq_unmask,
	gpio->aspeed_sgpio_irqchip.irq_set_type	= aspeed_sgpio_set_type;

	/* Allocate a cache of the output registers */
	banks = gpio->config->nr_gpios >> 5;
	gpio->dcache = devm_kcalloc(&pdev->dev,
				    banks, sizeof(u32), GFP_KERNEL);
	if (!gpio->dcache)
		return -ENOMEM;

	/*
	 * Populate it with initial values read from the HW and switch
	 * all command sources to the ARM by default
	 */
	for (i = 0; i < banks; i++) {
		const struct aspeed_sgpio_bank *bank = &aspeed_sgpio_banks[i];
		void __iomem *addr = bank_reg(gpio, bank, reg_rdata);
		gpio->dcache[i] = ioread32(addr);
	}

	rc = devm_gpiochip_add_data(&pdev->dev, &gpio->chip, gpio);
	if (rc < 0)
		return rc;

	return aspeed_sgpio_setup_irqs(gpio, pdev);
}

static struct platform_driver aspeed_sgpio_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_sgpio_of_table,
	},
};

module_platform_driver_probe(aspeed_sgpio_driver, aspeed_sgpio_probe);

MODULE_DESCRIPTION("Aspeed GPIO Driver");
MODULE_LICENSE("GPL");
