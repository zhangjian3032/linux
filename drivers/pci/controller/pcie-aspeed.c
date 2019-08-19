// SPDX-License-Identifier: GPL-2.0
/*
 * PCIE driver for the Aspeed SoC
 *
 * Author: Ryan Chen <ryan_chen@aspeedtech.com>
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include <linux/delay.h>

#include <linux/pci.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/module.h>

#include <linux/slab.h>
#include <linux/msi.h>
#include <linux/pci.h>

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/irqchip/chained_irq.h>

#include "../pci.h"

//#include "pcie-aspeed.h"
//#include <linux/aspeed-sdmc.h>

#define MAX_LEGACY_IRQS			4
#define MAX_MSI_HOST_IRQS		64


/* H2X Controller registers */

/* reg 0x24 */
#define PCIE_TX_IDLE			BIT(31)

#define PCIE_STATUS_OF_TX		GENMASK(25, 24)
#define	PCIE_RC_TX_COMPLETE		0
#define	PCIE_RC_L_TX_COMPLETE	BIT(24)		
#define	PCIE_RC_H_TX_COMPLETE	BIT(25)

#define PCIE_TRIGGER_TX			BIT(0)

/* reg 0x80, 0xC0 */
#define PCIE_RX_TAG_MASK		GENMASK(23, 16)
#define PCIE_RX_LINEAR			BIT(8)
#define PCIE_RX_MSI_SEL			BIT(7)
#define PCIE_RX_MSI_EN			BIT(6)
#define PCIE_1M_ADDRESS_EN		BIT(5)
#define PCIE_UNLOCK_RX_BUFF		BIT(4)
#define PCIE_RX_TLP_TAG_MATCH	BIT(3)
#define PCIE_Wait_RX_TLP_CLR	BIT(2)
#define PCIE_RC_RX_ENABLE		BIT(1)
#define PCIE_RC_ENABLE			BIT(0)

/* reg 0x88, 0xC8 : RC ISR */

#define PCIE_RC_CPLCA_ISR		BIT(6)
#define PCIE_RC_CPLUR_ISR		BIT(5)
#define PCIE_RC_RX_DONE_ISR		BIT(4)

#define PCIE_RC_INTD_ISR		BIT(3)
#define PCIE_RC_INTC_ISR		BIT(2)
#define PCIE_RC_INTB_ISR		BIT(1)
#define PCIE_RC_INTA_ISR		BIT(0)


/* PCI Host Controller registers */

#define ASPEED_PCIE_CLASS_CODE		0x04	
#define ASPEED_PCIE_GLOBAL			0x30
#define ASPEED_PCIE_CFG_DIN			0x50
#define ASPEED_PCIE_CFG3			0x58
#define ASPEED_PCIE_LOCK			0x7C
	
#define ASPEED_PCIE_LINK			0xC0
#define ASPEED_PCIE_INT				0xC4

/* 	AST_PCIE_CFG2			0x04		*/
#define PCIE_CFG_CLASS_CODE(x)	(x << 8)
#define PCIE_CFG_REV_ID(x)		(x)

/* 	AST_PCIE_GLOBAL			0x30 	*/
#define ROOT_COMPLEX_ID(x)		(x << 4)

/* 	AST_PCIE_LOCK			0x7C	*/
#define PCIE_UNLOCK				0xa8

/*	AST_PCIE_LINK			0xC0	*/
#define PCIE_LINK_STS			BIT(5)



#define AST_PCIE_WIN_BASE				0x70000000
#define AST_PCIE_WIN_SIZE				0x10000000

struct aspeed_msi {			/* MSI information */
	struct irq_domain *msi_domain;
	unsigned long *bitmap;
	struct irq_domain *dev_domain;
	struct mutex lock;		/* protect bitmap variable */
	int irq_msi0;
	int irq_msi1;
};

struct aspeed_pcie {
	struct device *dev;

	void __iomem *pciereg_base;
	void __iomem *h2xreg_base;
	phys_addr_t config_addr;

	u8 txTag;


	u8 last_busno;
	u8 root_busno;


	struct clk *free_ck;

	struct resource mem;
	const struct mtk_pcie_soc *soc;
	unsigned int busnr;

	u32 irq;
	unsigned long msi_pages;
	struct irq_domain *leg_domain;
	struct aspeed_msi msi;
	struct list_head resources;
	raw_spinlock_t leg_mask_lock;	
	
};

static int
aspeed_pcie_rd_conf(struct pci_bus *bus, unsigned int devfn, 
				int where, int size, u32 *val)
{
	struct aspeed_pcie *pcie = bus->sysdata;
	u32 timeout = 0;
	u32 bdf_offset;
	u32 type = 0;

	//H2X80[4] (unlock) is write-only.
	//Driver may set H2X80[4]=1 before triggering next TX config.
	writel(BIT(4) | readl(pcie->h2xreg_base + 0x80), pcie->h2xreg_base + 0x80);

	if(bus->number)
		type = 1;
	else
		type = 0;

	bdf_offset = (bus->number << 24) |
					(PCI_SLOT(devfn) << 19) |
					(PCI_FUNC(devfn) << 16) |
					(where & ~3);

	pcie->txTag %= 0x7;

	writel(0x04000001 | (type << 24), pcie->h2xreg_base + 0x10);
	writel(0x0000200f | (pcie->txTag << 8), pcie->h2xreg_base + 0x14);
	writel(bdf_offset, pcie->h2xreg_base + 0x18);
	writel(0x00000000, pcie->h2xreg_base + 0x1c);

	//trigger tx
	writel(PCIE_TRIGGER_TX, pcie->h2xreg_base + 0x24);

	//wait tx idle
	while(!(readl(pcie->h2xreg_base + 0x24) & PCIE_TX_IDLE)) {
		timeout++;
		if(timeout > 10000) {
			printk("time out b : %d, d : %d, f: %d \n", bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn));
			*val = 0xffffffff;
			goto out;
		}
	};

	//write clr tx idle
	writel(1, pcie->h2xreg_base + 0x08);

	//check tx status 
	switch(readl(pcie->h2xreg_base + 0x24) & PCIE_STATUS_OF_TX) {
		case PCIE_RC_L_TX_COMPLETE:
			while(!(readl(pcie->h2xreg_base + 0x88) & PCIE_RC_RX_DONE_ISR));
			if(readl(pcie->h2xreg_base + 0x88) & (PCIE_RC_CPLCA_ISR | PCIE_RC_CPLUR_ISR)) {
				printk("return ffffffff \n");
				*val = 0xffffffff;
			} else
				*val = readl(pcie->h2xreg_base + 0x8C);
			writel(readl(pcie->h2xreg_base + 0x88), pcie->h2xreg_base + 0x88);
			break;
		case PCIE_RC_H_TX_COMPLETE:
			while(!(readl(pcie->h2xreg_base + 0xC8) & PCIE_RC_RX_DONE_ISR));
			if(readl(pcie->h2xreg_base + 0xC8) & (PCIE_RC_CPLCA_ISR | PCIE_RC_CPLUR_ISR))
				*val = 0xffffffff;
			else
				*val = readl(pcie->h2xreg_base + 0xCC);
			writel(readl(pcie->h2xreg_base + 0xC8), pcie->h2xreg_base + 0xC8);
			break;
		default:	//read rc data
			*val = readl(pcie->h2xreg_base + 0x0C);
			break;
	}

	switch (size) {
		case 1:
			*val = (*val >> ((where & 3) * 8)) & 0xff;
			break;
		case 2:
			*val = (*val >> ((where & 2) * 8)) & 0xffff;
			break;
	}

out:
	pcie->txTag++;

	return PCIBIOS_SUCCESSFUL;
}


static int
aspeed_pcie_wr_conf(struct pci_bus *bus, unsigned int devfn, 
				int where, int size, u32 val)
{
	u32 timeout = 0;
	u32 type = 0;
	u32 shift = 8 * (where & 3);
	u32 bdf_offset;
	u8 byte_en = 0;

	struct aspeed_pcie *pcie = bus->sysdata;


//	printk("%x:%x:%x => size = %d, where = %xh, val = %xh\n",
//		bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn), size, where, val);
#if 1

	writel(BIT(4) | readl(pcie->h2xreg_base + 0x80), pcie->h2xreg_base + 0x80);

#if 0
	switch (size) {
	case 1:
		data32 = (value & 0xff) << shift;
		byte_en = 1 << (where & 3);
		break;
	case 2:
		data32 = (value & 0xffff) << shift;
		byte_en = 3 << (where & 3);
		break;
	default:
		data32 = value;
		byte_en = 0xf;
		break;
	}
#endif	
	switch (size) {
	case 1:
		switch(where % 4) {
			case 0:
				byte_en = 0x1;		
				break;
			case 1:
				byte_en = 0x2;
				break;
			case 2:
				byte_en = 0x4;
				break;	
			case 3:
				byte_en = 0x8;
				break;			
		}
		val = (val & 0xff) << shift;
		break;
	case 2:
		switch((where >> 1) % 2 ) {
			case 0:
				byte_en = 0x3;
				break;
			case 1:
				byte_en = 0xc;
				break;
		}
		val = (val & 0xffff) << shift;		
		break;
	default:
		byte_en = 0xf;
		break;
	}
	
	if(bus->number)
		type = 1;
	else
		type = 0;
		
	bdf_offset = (bus->number << 24) | (PCI_SLOT(devfn) << 19) |
					(PCI_FUNC(devfn) << 16) | (where & ~3);
	
	pcie->txTag %= 0x7;
	
	writel(0x44000001 | (type << 24), pcie->h2xreg_base + 0x10);
	writel(0x00002000 | (pcie->txTag << 8) | byte_en, pcie->h2xreg_base + 0x14);
	writel(bdf_offset, pcie->h2xreg_base + 0x18);
	writel(0x00000000, pcie->h2xreg_base + 0x1C);
	
	writel(val, pcie->h2xreg_base + 0x20);
	
	//trigger tx
	writel(1, pcie->h2xreg_base + 0x24); 
	
//wait tx idle
	while(!(readl(pcie->h2xreg_base + 0x24) & BIT(31))) {
		timeout++;
		if(timeout > 10000) {
			printk("time out \n");
			goto out;
		}
	};

	//write clr tx idle
	writel(1, pcie->h2xreg_base + 0x08);

	//check tx status and clr rx done int
	switch(readl(pcie->h2xreg_base + 0x24) & PCIE_STATUS_OF_TX) {
		case PCIE_RC_L_TX_COMPLETE:
			while(!(readl(pcie->h2xreg_base + 0x88) & PCIE_RC_RX_DONE_ISR));
			writel(PCIE_RC_RX_DONE_ISR, pcie->h2xreg_base + 0x88);

			break;
		case PCIE_RC_H_TX_COMPLETE:
			while(!(readl(pcie->h2xreg_base + 0xC8) & PCIE_RC_RX_DONE_ISR));
			writel(PCIE_RC_RX_DONE_ISR, pcie->h2xreg_base + 0xC8);

			break;
	}

out:
	pcie->txTag++;
#endif
	return PCIBIOS_SUCCESSFUL;
}


static struct pci_ops aspeed_pcie_ops = {
	.read	= aspeed_pcie_rd_conf,
	.write	= aspeed_pcie_wr_conf,
};

static irqreturn_t aspeed_pcie_misc_handler(int irq, void *data)
{
	struct aspeed_pcie *pcie = data;
	struct device *dev = pcie->dev;
	u32 misc_stat;
	printk("aspeed_pcie_misc_handler\n");

	return IRQ_HANDLED;
}

static void aspeed_pcie_leg_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct aspeed_pcie *pcie;
	struct device *dev = pcie->dev;
	unsigned long status;
	u32 reg;
	u32 bit;
	u32 virq;

	printk("aspeed_pcie_leg_handler \n");
	chained_irq_enter(chip, desc);
	pcie = irq_desc_get_handler_data(desc);

	while ((status = readl(pcie->h2xreg_base + 0x88) &
				0xf ) != 0) {
		for_each_set_bit(bit, &status, PCI_NUM_INTX) {
			printk("intx bit %d \n", bit);
			virq = irq_find_mapping(pcie->leg_domain, bit);
			if (virq)
				generic_handle_irq(virq);
		}
	}

	chained_irq_exit(chip, desc);
}

static void aspeed_pcie_handle_msi_irq(struct aspeed_pcie *pcie, u32 status_reg)
{
	struct aspeed_msi *msi;
	unsigned long status;
	u32 bit;
	u32 virq;

	msi = &pcie->msi;

	while ((status = readl(pcie->h2xreg_base + status_reg)) != 0) {
		for_each_set_bit(bit, &status, 32) {
			writel(1 << bit, pcie->h2xreg_base + status_reg);
			virq = irq_find_mapping(msi->dev_domain, bit);
			if (virq)
				generic_handle_irq(virq);
		}
	}
}

static void aspeed_pcie_msi_handler_high(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct aspeed_pcie *pcie = irq_desc_get_handler_data(desc);

	if(readl(pcie->h2xreg_base + 0x88) & BIT(10)) {
		chained_irq_enter(chip, desc);
		aspeed_pcie_handle_msi_irq(pcie, 0xAC);
		chained_irq_exit(chip, desc);
	}
}

static void aspeed_pcie_msi_handler_low(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct aspeed_pcie *pcie = irq_desc_get_handler_data(desc);

	if(readl(pcie->h2xreg_base + 0x88) & BIT(9)) {
		chained_irq_enter(chip, desc);
		aspeed_pcie_handle_msi_irq(pcie, 0xA8);
		chained_irq_exit(chip, desc);
	}
}

static void aspeed_mask_leg_irq(struct irq_data *data)
{
	struct irq_desc *desc = irq_to_desc(data->irq);
	struct aspeed_pcie *pcie;
	unsigned long flags;
	u32 mask;
	u32 val;

	printk("aspeed_mask_leg_irq : data->hwirq %d \n", data->hwirq);
	
	pcie = irq_desc_get_chip_data(desc);
	mask = 1 << (data->hwirq - 1);
	raw_spin_lock_irqsave(&pcie->leg_mask_lock, flags);
	writel((readl(pcie->h2xreg_base + 0x84) & ~mask), pcie->h2xreg_base + 0x84);
	raw_spin_unlock_irqrestore(&pcie->leg_mask_lock, flags);
}

static void aspeed_unmask_leg_irq(struct irq_data *data)
{
	struct irq_desc *desc = irq_to_desc(data->irq);
	struct aspeed_pcie *pcie;
	unsigned long flags;
	u32 mask;
	u32 val;

	printk("aspeed_unmask_leg_irq : data->hwirq %d \n", data->hwirq);
	
	pcie = irq_desc_get_chip_data(desc);
	mask = 1 << (data->hwirq);
	raw_spin_lock_irqsave(&pcie->leg_mask_lock, flags);
	writel((readl(pcie->h2xreg_base + 0x84) | mask), pcie->h2xreg_base + 0x84);
	raw_spin_unlock_irqrestore(&pcie->leg_mask_lock, flags);
}

static struct irq_chip aspeed_leg_irq_chip = {
	.name = "pcie:intx",
	.irq_enable = aspeed_unmask_leg_irq,
	.irq_disable = aspeed_mask_leg_irq,
	.irq_mask = aspeed_mask_leg_irq,
	.irq_unmask = aspeed_unmask_leg_irq,
};

static int aspeed_legacy_map(struct irq_domain *domain, unsigned int irq,
                                  irq_hw_number_t hwirq)
{
    irq_set_chip_and_handler(irq, &aspeed_leg_irq_chip, handle_level_irq);
    irq_set_chip_data(irq, domain->host_data);
	irq_set_status_flags(irq, IRQ_LEVEL);

    return 0;
}

static const struct irq_domain_ops intx_domain_ops = {
    .map = aspeed_legacy_map,
	.xlate = pci_irqd_intx_xlate,
};

#ifdef CONFIG_PCI_MSI
static struct irq_chip aspeed_msi_irq_chip = {
	.name = "aspeed_pcie:msi",
	.irq_enable = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
	.irq_mask = mask_msi_irq,
	.irq_unmask = unmask_msi_irq,

};

static struct msi_domain_info aspeed_msi_domain_info = {
	.flags = (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		  MSI_FLAG_MULTI_PCI_MSI),
	.chip = &aspeed_msi_irq_chip,
};
#endif

static void aspeed_compose_msi_msg(struct irq_data *data, struct msi_msg *msg)
{
	struct aspeed_pcie *pcie = irq_data_get_irq_chip_data(data);
//	phys_addr_t msi_addr = pcie->phys_pcie_reg_base;

	msg->address_lo = 0x1e770058;//lower_32_bits(msi_addr);
	msg->address_hi = 0;//upper_32_bits(msi_addr);
	msg->data = data->hwirq;
}

static int aspeed_msi_set_affinity(struct irq_data *irq_data,
				const struct cpumask *mask, bool force)
{
	return -EINVAL;
}

static struct irq_chip aspeed_irq_chip = {
	.name = "ASPEED MSI",
	.irq_compose_msi_msg = aspeed_compose_msi_msg,
	.irq_set_affinity = aspeed_msi_set_affinity,
};

static int aspeed_irq_domain_alloc(struct irq_domain *domain, unsigned int virq,
				unsigned int nr_irqs, void *args)
{
	struct aspeed_pcie *pcie = domain->host_data;
	struct aspeed_msi *msi = &pcie->msi;
	int bit;
	int i;

	mutex_lock(&msi->lock);
	bit = bitmap_find_next_zero_area(msi->bitmap, MAX_MSI_HOST_IRQS, 0,
					 nr_irqs, 0);
	if (bit >= MAX_MSI_HOST_IRQS) {
		mutex_unlock(&msi->lock);
		return -ENOSPC;
	}

	bitmap_set(msi->bitmap, bit, nr_irqs);

	for (i = 0; i < nr_irqs; i++) {
		irq_domain_set_info(domain, virq + i, bit + i, &aspeed_irq_chip,
				domain->host_data, handle_simple_irq,
				NULL, NULL);
	}
	mutex_unlock(&msi->lock);
	return 0;
}

static void aspeed_irq_domain_free(struct irq_domain *domain, unsigned int virq,
					unsigned int nr_irqs)
{
	struct irq_data *data = irq_domain_get_irq_data(domain, virq);
	struct aspeed_pcie *pcie = irq_data_get_irq_chip_data(data);
	struct aspeed_msi *msi = &pcie->msi;
printk("aspeed_irq_domain_free \n");
	mutex_lock(&msi->lock);
	bitmap_clear(msi->bitmap, data->hwirq, nr_irqs);
	mutex_unlock(&msi->lock);
}

static const struct irq_domain_ops dev_msi_domain_ops = {
	.alloc  = aspeed_irq_domain_alloc,
	.free   = aspeed_irq_domain_free,
};

static int aspeed_pcie_init_msi_irq_domain(struct aspeed_pcie *pcie)
{
#ifdef CONFIG_PCI_MSI
	struct device *dev = pcie->dev;
	struct fwnode_handle *fwnode = of_node_to_fwnode(dev->of_node);
	struct aspeed_msi *msi = &pcie->msi;

	msi->dev_domain = irq_domain_add_linear(NULL, MAX_MSI_HOST_IRQS,
						&dev_msi_domain_ops, pcie);
	if (!msi->dev_domain) {
		dev_err(dev, "failed to create dev IRQ domain\n");
		return -ENOMEM;
	}
	msi->msi_domain = pci_msi_create_irq_domain(fwnode,
						    &aspeed_msi_domain_info,
						    msi->dev_domain);
	if (!msi->msi_domain) {
		dev_err(dev, "failed to create msi IRQ domain\n");
		irq_domain_remove(msi->dev_domain);
		return -ENOMEM;
	}
#endif
	return 0;
}

static int aspeed_pcie_init_irq_domain(struct aspeed_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct device_node *node = dev->of_node;
	struct device_node *legacy_intc_node;

	legacy_intc_node = of_get_next_child(node, NULL);
	if (!legacy_intc_node) {
		dev_err(dev, "No legacy intc node found\n");
		return -EINVAL;
	}
	
	pcie->leg_domain = irq_domain_add_linear(legacy_intc_node, MAX_LEGACY_IRQS,
											 &intx_domain_ops,
											 pcie);
	of_node_put(legacy_intc_node);
	if (!pcie->leg_domain) {
			dev_err(dev, "Failed to get a INTx IRQ domain\n");
			return -ENODEV;
	}

	raw_spin_lock_init(&pcie->leg_mask_lock);
	aspeed_pcie_init_msi_irq_domain(pcie);
	return 0;

}

static int aspeed_pcie_enable_msi(struct aspeed_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct aspeed_msi *msi = &pcie->msi;
	unsigned long base;
	int ret;
	int size = BITS_TO_LONGS(MAX_MSI_HOST_IRQS) * sizeof(long);

	mutex_init(&msi->lock);

	msi->bitmap = kzalloc(size, GFP_KERNEL);
	if (!msi->bitmap)
		return -ENOMEM;

	/* Get msi_1 IRQ number */
	irq_set_chained_handler_and_data(pcie->irq,
					 aspeed_pcie_msi_handler_high, pcie);

	/* Get msi_0 IRQ number */
	irq_set_chained_handler_and_data(pcie->irq,
					 aspeed_pcie_msi_handler_low, pcie);


	/*
	 * For high range MSI interrupts: disable, clear any pending,
	 * and enable
	 */
	writel(readl(pcie->h2xreg_base + 0x84) | BIT(10), pcie->h2xreg_base + 0x84);	

	/*
	 * For low range MSI interrupts: disable, clear any pending,
	 * and enable
	 */
	writel(readl(pcie->h2xreg_base + 0x84) | BIT(9), pcie->h2xreg_base + 0x84);	
	

	writel(readl(pcie->h2xreg_base + 0x84) | BIT(8), pcie->h2xreg_base + 0x84);

	return 0;
err:
	kfree(msi->bitmap);
	msi->bitmap = NULL;
	return ret;
}

static void aspeed_pcie_bridge_init(struct aspeed_pcie *pcie)
{
	struct device *dev = pcie->dev;
//	struct platform_device *pdev = to_platform_device(dev);
//	int err;

	//TODO : ahbc remap enable
	//aspeed_ahbc_remap_enable(devfdt_get_addr_ptr(ahbc_dev));

	//h2x init
	//reset_assert(&reset_ctl);
	//reset_deassert(&reset_ctl);

	pcie->txTag = 0;
	writel(0x1, pcie->h2xreg_base + 0x00);

	//ahb to pcie rc 
	writel(0xe0006000, pcie->h2xreg_base + 0x60);
	writel(0x00000000, pcie->h2xreg_base + 0x64);
	writel(0xFFFFFFFF, pcie->h2xreg_base + 0x68);
	
//	writel( PCIE_RX_LINEAR | PCIE_RX_MSI_SEL | PCIE_RX_MSI_EN |
//			PCIE_Wait_RX_TLP_CLR | PCIE_RC_RX_ENABLE | PCIE_RC_L,
#if 1
	//rc_l
	writel( PCIE_RX_LINEAR | PCIE_RX_MSI_EN |
			PCIE_Wait_RX_TLP_CLR | PCIE_RC_RX_ENABLE | PCIE_RC_ENABLE,
	pcie->h2xreg_base + 0x80);
#else
	//rc_h
	writel( PCIE_RX_LINEAR | PCIE_RX_MSI_EN |
			PCIE_Wait_RX_TLP_CLR | PCIE_RC_RX_ENABLE | PCIE_RC_ENABLE,
	pcie->h2xreg_base + 0x80);
#endif
	//assign debug tx tag
	writel(0x28, pcie->h2xreg_base + 0xBC);
		
	//plda init
	writel(PCIE_UNLOCK, pcie->pciereg_base + ASPEED_PCIE_LOCK);
//	writel(PCIE_CFG_CLASS_CODE(0x60000) | PCIE_CFG_REV_ID(4), pcie->pciereg_base + ASPEED_PCIE_CLASS_CODE);
	writel(ROOT_COMPLEX_ID(0x3), pcie->pciereg_base + ASPEED_PCIE_GLOBAL);

	/* Don't register host if link is down */
	if (readl(pcie->pciereg_base + ASPEED_PCIE_LINK) & PCIE_LINK_STS) {
		printk("PCIE- Link up\n");
	} else {
		printk("PCIE- Link down\n");
	}

	return 0;
}

static int aspeed_pcie_parse_dt(struct aspeed_pcie *pcie,
			     struct platform_device *pdev)
{
	struct device *dev = pcie->dev;
	struct device_node *node = dev->of_node;	
	struct resource *res;
	int err;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pciebreg");
	if(res) {
		printk("res->start %x \n", res->start);
		printk("res->end %x \n", res->end);
		pcie->pciereg_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(pcie->pciereg_base))
			return PTR_ERR(pcie->pciereg_base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "h2xreg");
	if(res) {
		printk("res->start %x \n", res->start);
		printk("res->end %x \n", res->end);
		pcie->h2xreg_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(pcie->h2xreg_base))
			return PTR_ERR(pcie->h2xreg_base);
	}

	pcie->irq = irq_of_parse_and_map(node, 0);
	if (pcie->irq < 0) {
		dev_err(dev, "failed to get IRQ %d\n", pcie->irq);
		return pcie->irq;
	}

	err = devm_request_irq(dev, pcie->irq, aspeed_pcie_misc_handler,
						   IRQF_SHARED,
						   "aspeed-pcie:misc", pcie);
	if (err) {
		dev_err(dev, "unable to request irq %d\n", pcie->irq);
		return err;
	}

	irq_set_chained_handler_and_data(pcie->irq,
					 aspeed_pcie_leg_handler, pcie);

	printk("==================================== ***\n");
	return 0;
}

static const struct of_device_id aspeed_pcie_of_match[] = {
	{ .compatible = "aspeed,ast2600-pcie", },
	{},
};

static int aspeed_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aspeed_pcie *pcie;
	struct pci_bus *bus;
	struct pci_bus *child;
	struct pci_host_bridge *bridge;
	int err;
	resource_size_t iobase = 0;
	LIST_HEAD(res);

	printk("aspeed_pcie_probe 0 =============================================\n");
	
	if (!dev->of_node)
			 return -ENODEV;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if (!bridge)
		return -ENOMEM;

	pcie = pci_host_bridge_priv(bridge);

	pcie->dev = dev;

	err = aspeed_pcie_parse_dt(pcie, pdev);
	if (err) {
		dev_err(dev, "Parsing DT failed\n");
		return err;
	}

	aspeed_pcie_bridge_init(pcie);


	err = devm_of_pci_get_host_bridge_resources(dev, 0, 0xff, &res,
												&iobase);
	if (err) {
			dev_err(dev, "Getting bridge resources failed\n");
			return err;
	}

	err = devm_request_pci_bus_resources(dev, &res);
	if (err)
			goto error;
	err = aspeed_pcie_init_irq_domain(pcie);
	if (err) {
	   dev_err(dev, "Failed creating IRQ Domain\n");
	   goto error;
	}
	
	list_splice_init(&res, &bridge->windows);

	bridge->dev.parent = dev;
	bridge->sysdata = pcie;
	bridge->busnr = 0;
	bridge->ops = &aspeed_pcie_ops;
	bridge->map_irq = of_irq_parse_and_map_pci;
	bridge->swizzle_irq = pci_common_swizzle;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		err = aspeed_pcie_enable_msi(pcie);
		if (err < 0) {
			dev_err(dev, "failed to enable MSI support: %d\n", err);
			goto error;
		}
	}

	err = pci_scan_root_bus_bridge(bridge);
	if (err)
		goto error;

	bus = bridge->bus;

	pci_assign_unassigned_bus_resources(bus);
	list_for_each_entry(child, &bus->children, node)
		pcie_bus_configure_settings(child);
	pci_bus_add_devices(bus);
	printk("aspeed_pcie_probe end =============================================\n");	
	return 0;

error:
	pci_free_resource_list(&res);
	return err;
}
static struct platform_driver aspeed_pcie_driver = {
	.probe = aspeed_pcie_probe,
	.driver = {
		.name	= "aspeed-pcie",
		.of_match_table = aspeed_pcie_of_match,
	},
};

builtin_platform_driver(aspeed_pcie_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED PCIe Host driver");
MODULE_LICENSE("GPL");
