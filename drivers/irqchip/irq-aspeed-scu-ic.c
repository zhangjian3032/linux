/*
 * irq-aspeed-scu.c - SCU IRQCHIP driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
/*******************************************************************/
/*  AST_SCU_INTR_CTRL : 0x18 - Interrupt control and status register   */
#define INTR_LPC_H_L_RESET				BIT(21)
#define INTR_LPC_L_H_RESET				BIT(20)
#define INTR_PCIE_H_L_RESET				BIT(19)
#define INTR_PCIE_L_H_RESET				BIT(18)
#define INTR_VGA_SCRATCH_CHANGE			BIT(17)
#define INTR_VGA_CURSOR_CHANGE			BIT(16)
#define INTR_ISSUE_MSI					BIT(6)
#define INTR_LPC_H_L_RESET_EN			BIT(5)
#define INTR_LPC_L_H_RESET_EN			BIT(4)
#define INTR_PCIE_H_L_RESET_EN			BIT(3)
#define INTR_PCIE_L_H_RESET_EN			BIT(2)
#define INTR_VGA_SCRATCH_CHANGE_EN		BIT(1)
#define INTR_VGA_CURSOR_CHANGE_EN		BIT(0)
/*******************************************************************/
//#define AST_SCU_IRQ_DEBUG

#ifdef AST_SCU_IRQ_DEBUG
#define SCU_IRQ_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#else
#define SCU_IRQ_DBUG(fmt, args...)
#endif
/*******************************************************************/
struct ast_scu_irq {
	void __iomem	*regs;
	int			parent_irq;
	int			irq_num;
	struct irq_domain	*irq_domain;
};
/*******************************************************************/
static void ast_scu_irq_handler(struct irq_desc *desc)
{
	struct ast_scu_irq *scu_irq = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long bit, status, irq_sts;
	unsigned int bus_irq;

	chained_irq_enter(chip, desc);
	status = readl(scu_irq->regs);
	irq_sts = (status >> 16) & status;

	for_each_set_bit(bit, &irq_sts, scu_irq->irq_num) {
		bus_irq = irq_find_mapping(scu_irq->irq_domain, bit);
		generic_handle_irq(bus_irq);
		writel((status & 0x7f) | (1 << (bit + 16)), scu_irq->regs);
	}
	chained_irq_exit(chip, desc);
}

static void scu_mask_irq(struct irq_data *data)
{
	struct ast_scu_irq *scu_irq = irq_data_get_irq_chip_data(data);
	unsigned int sbit = 1 << data->hwirq;

	writel(readl(scu_irq->regs) & ~sbit, scu_irq->regs);
}

static void scu_unmask_irq(struct irq_data *data)
{
	struct ast_scu_irq *scu_irq = irq_data_get_irq_chip_data(data);
	unsigned int sbit = 1 << (data->hwirq);

	writel((readl(scu_irq->regs) | sbit) & 0x7f, scu_irq->regs);
}

struct irq_chip scu_irq_chip = {
	.name		= "scu-irq",
	.irq_mask	= scu_mask_irq,
	.irq_unmask	= scu_unmask_irq,
};

static int ast_scu_map_irq_domain(struct irq_domain *domain,
				  unsigned int irq, irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &scu_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops ast_scu_irq_domain_ops = {
	.map = ast_scu_map_irq_domain,
};

static const struct of_device_id aspeed_scu_ic_of_match[] = {
	{ .compatible = "aspeed,ast2400-scu-ic", .data = (void *) 22},
	{ .compatible = "aspeed,ast2500-scu-ic", .data = (void *) 22},
	{ .compatible = "aspeed,ast2600-scu-ic", .data = (void *) 22},
	{},
};

static int aspeed_scu_ic_probe(struct platform_device *pdev)
{
	struct ast_scu_irq *scu_irq;
	struct device_node *node = pdev->dev.of_node;
	const struct of_device_id *match;
	int ret = 0;

	scu_irq = kzalloc(sizeof(*scu_irq), GFP_KERNEL);
	if (!scu_irq)
		return -ENOMEM;

	scu_irq->regs = of_iomap(node, 0);
	if (IS_ERR(scu_irq->regs))
		return PTR_ERR(scu_irq->regs);

	scu_irq->parent_irq = irq_of_parse_and_map(node, 0);
	if (scu_irq->parent_irq < 0) {
		printk("get scu irq parent error \n");
		return scu_irq->parent_irq;
	}

	match = of_match_node(aspeed_scu_ic_of_match, node);
	if (!match)
		goto err_iounmap;

	scu_irq->irq_num = (int) match->data;

	scu_irq->irq_domain = irq_domain_add_linear(
				      node, scu_irq->irq_num,
				      &ast_scu_irq_domain_ops, scu_irq);
	if (!scu_irq->irq_domain) {
		ret = (int) scu_irq->irq_domain;
		printk("no irq domain \n");

	}

	scu_irq->irq_domain->name = "aspeed-scu-domain";

	irq_set_chained_handler_and_data(scu_irq->parent_irq,
					 ast_scu_irq_handler, scu_irq);

	pr_info("scu-irq controller registered, irq %d\n", scu_irq->parent_irq);
	return 0;

err_iounmap:
	iounmap(scu_irq->regs);
err_free_ic:
	kfree(scu_irq);
	return ret;
}

static struct platform_driver aspeed_scu_ic_driver = {
	.probe	= aspeed_scu_ic_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_scu_ic_of_match,
	},
};

static int __init aspeed_scu_ic_init(void)
{
	return platform_driver_register(&aspeed_scu_ic_driver);
}
postcore_initcall(aspeed_scu_ic_init);

MODULE_AUTHOR("Ryan Chen");
MODULE_DESCRIPTION("ASPEED SCU INTC Driver");
MODULE_LICENSE("GPL v2");
