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

#include <linux/aspeed_pcie_io.h>

#include "../pci.h"

//#include "pcie-aspeed.h"
//#include <linux/aspeed-sdmc.h>

#define MAX_LEGACY_IRQS			4
#define MAX_MSI_HOST_IRQS		64

//#define H2X_RC_L
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

struct aspeed_pcie {
	struct device *dev;

	void __iomem *pciereg_base;
	void __iomem *h2xreg_base;
	phys_addr_t config_addr;

	u8 txTag;

	struct resource mem;
	unsigned int busnr;

	u32 irq;
	struct irq_domain *leg_domain;
	struct irq_domain *msi_domain;
	struct list_head resources;
	
};

static DECLARE_BITMAP(msi_irq_in_use, MAX_MSI_HOST_IRQS);

void __iomem *h2xreg_base;

void aspeed_pcie_workaround(void)
{
	u32 timeout = 0;

#ifdef H2X_RC_L	
	writel(BIT(4) | readl(h2xreg_base + 0x80), h2xreg_base + 0x80);
#else
	writel(BIT(4) | readl(h2xreg_base + 0xc0), h2xreg_base + 0xc0);
#endif

	writel(0x74000001, h2xreg_base + 0x10);
	writel(0x00400050, h2xreg_base + 0x14);
	writel(0x00000000, h2xreg_base + 0x18);
	writel(0x00000000, h2xreg_base + 0x1c);

	writel(0x1a, h2xreg_base + 0x20);

	//trigger tx
	writel(PCIE_TRIGGER_TX, h2xreg_base + 0x24);

	//wait tx idle
	while(!(readl(h2xreg_base + 0x24) & BIT(31))) {
		timeout++;
		if(timeout > 1000) {
			return;
		}
	};

	//write clr tx idle
	writel(1, h2xreg_base + 0x08);
	timeout = 0;

	//check tx status and clr rx done int
#ifdef H2X_RC_L	
	while(!(readl(h2xreg_base + 0x88) & PCIE_RC_RX_DONE_ISR)) {
		timeout++;
		if(timeout > 10) {
			break;
		}
		mdelay(1);
	}
	writel(PCIE_RC_RX_DONE_ISR, h2xreg_base + 0x88);
#else	
	while(!(readl(h2xreg_base + 0xc8) & PCIE_RC_RX_DONE_ISR)) {
		timeout++;
		if(timeout > 10) {
			break;
		}
		mdelay(1);
	}
	writel(PCIE_RC_RX_DONE_ISR, h2xreg_base + 0xc8);
#endif	
}

EXPORT_SYMBOL(aspeed_pcie_workaround);

static int
aspeed_pcie_rd_conf(struct pci_bus *bus, unsigned int devfn, 
				int where, int size, u32 *val)
{
	struct aspeed_pcie *pcie = bus->sysdata;
	u32 timeout = 0;
	u32 bdf_offset;
	int rx_done_fail = 0;	
	u32 type = 0;

	//H2X80[4] (unlock) is write-only.
	//Driver may set H2X80[4]=1 before triggering next TX config.
#ifdef H2X_RC_L	
	writel(BIT(4) | readl(pcie->h2xreg_base + 0x80), pcie->h2xreg_base + 0x80);
#else
	writel(BIT(4) | readl(pcie->h2xreg_base + 0xC0), pcie->h2xreg_base + 0xC0);
#endif

//	printk("cfg rd : %x:%x:%x => size = %d, where = %xh \n",
//		bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn), size, where);

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
	writel((readl(pcie->h2xreg_base + 0x24) & 0xf) | PCIE_TRIGGER_TX, pcie->h2xreg_base + 0x24);

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

	timeout = 0;
	//check tx status 
	switch(readl(pcie->h2xreg_base + 0x24) & PCIE_STATUS_OF_TX) {
		case PCIE_RC_L_TX_COMPLETE:
			while(!(readl(pcie->h2xreg_base + 0x88) & PCIE_RC_RX_DONE_ISR)) {
				timeout++;
				if(timeout > 10) {
					rx_done_fail = 1;
					*val = 0xffffffff;
					break;
				}
				mdelay(1);
			}
			if(!rx_done_fail) {
				if(readl(pcie->h2xreg_base + 0x94) & BIT(13)) {
					*val = 0xffffffff;
				} else
					*val = readl(pcie->h2xreg_base + 0x8C);
			}
//			writel(BIT(4) | readl(pcie->h2xreg_base + 0x80), pcie->h2xreg_base + 0x80);
			writel(readl(pcie->h2xreg_base + 0x88), pcie->h2xreg_base + 0x88);
			break;
		case PCIE_RC_H_TX_COMPLETE:
			while(!(readl(pcie->h2xreg_base + 0xC8) & PCIE_RC_RX_DONE_ISR)) {
				timeout++;
				if(timeout > 10) {
					rx_done_fail = 1;
					*val = 0xffffffff;
					break;
				}
				mdelay(1);
			}
			if(!rx_done_fail) {
				if(readl(pcie->h2xreg_base + 0x94) & BIT(13)) {
					*val = 0xffffffff;
				} else
					*val = readl(pcie->h2xreg_base + 0xCC);
			}
//			writel(BIT(4) | readl(pcie->h2xreg_base + 0xC0), pcie->h2xreg_base + 0xC0);
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

#ifdef H2X_RC_L	
	writel(BIT(4) | readl(pcie->h2xreg_base + 0x80), pcie->h2xreg_base + 0x80);
#else
	writel(BIT(4) | readl(pcie->h2xreg_base + 0xC0), pcie->h2xreg_base + 0xC0);
#endif

//	printk("cfg w : %x:%x:%x => size = %d, where = %xh, val = %xh\n",
//		bus->number, PCI_SLOT(devfn), PCI_FUNC(devfn), size, where, val);
	
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
	writel((readl(pcie->h2xreg_base + 0x24) & 0xf) | PCIE_TRIGGER_TX, pcie->h2xreg_base + 0x24); 
	
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

	timeout = 0;
	//check tx status and clr rx done int
	switch(readl(pcie->h2xreg_base + 0x24) & PCIE_STATUS_OF_TX) {
		case PCIE_RC_L_TX_COMPLETE:
			while(!(readl(pcie->h2xreg_base + 0x88) & PCIE_RC_RX_DONE_ISR)) {
				timeout++;
				if(timeout > 10) {
					break;
				}
				mdelay(1);
			}
			writel(PCIE_RC_RX_DONE_ISR, pcie->h2xreg_base + 0x88);

			break;
		case PCIE_RC_H_TX_COMPLETE:
			while(!(readl(pcie->h2xreg_base + 0xC8) & PCIE_RC_RX_DONE_ISR)) {
				timeout++;
				if(timeout > 10) {
					break;
				}
				mdelay(1);
			}
			writel(PCIE_RC_RX_DONE_ISR, pcie->h2xreg_base + 0xC8);
			break;
	}

out:
	pcie->txTag++;

	return PCIBIOS_SUCCESSFUL;
}

u8 aspeed_pcie_inb(u32 addr)
{
	int timeout = 0;

#ifdef H2X_RC_L	
	writel(BIT(4) | readl(h2xreg_base + 0x80), h2xreg_base + 0x80);
#else
	writel(BIT(4) | readl(h2xreg_base + 0xC0), h2xreg_base + 0xC0);
#endif

	writel(0x02000001, h2xreg_base + 0x10);
	writel(0x00002000 | (0x1 << (addr & 0x3)), h2xreg_base + 0x14);
	writel(addr & (~0x3), h2xreg_base + 0x18);
	writel(0x00000000, h2xreg_base + 0x1c);

	//trigger
	writel((readl(h2xreg_base + 0x24) & 0xf) | PCIE_TRIGGER_TX, h2xreg_base + 0x24); 	

	//wait tx idle
	while(!(readl(h2xreg_base + 0x24) & PCIE_TX_IDLE)) {
		timeout++;
		if(timeout > 10000) {
			printk("aspeed_pcie_inb timeout\n");
			return 0xff;
		}
	};

	//write clr tx idle
	writel(1, h2xreg_base + 0x08);

	timeout = 0;
#ifdef H2X_RC_L	
	while(!(readl(h2xreg_base + 0x88) & PCIE_RC_RX_DONE_ISR)) {
		timeout++;
		if(timeout > 10) {
			break;
		}
		mdelay(1);
	}
	writel(readl(h2xreg_base + 0x88), h2xreg_base + 0x88);
//	writel(BIT(4) | readl(h2xreg_base + 0x80), h2xreg_base + 0x80);
	return ((readl(h2xreg_base + 0x8C) >> ((addr & 0x3) * 8)) & 0xff);
#else
	while(!(readl(h2xreg_base + 0xc8) & PCIE_RC_RX_DONE_ISR)) {
		timeout++;
		if(timeout > 10) {
			break;
		}
		mdelay(1);
	}

	writel(readl(h2xreg_base + 0xC8), h2xreg_base + 0xC8);
//	writel(BIT(4) | readl(h2xreg_base + 0xC0), h2xreg_base + 0xC0);
	return ((readl(h2xreg_base + 0xCC) >> ((addr & 0x3) * 8)) & 0xff);
#endif

}

EXPORT_SYMBOL_GPL(aspeed_pcie_inb);

void aspeed_pcie_outb(u8 value, u32 addr)
{
	int timeout = 0;
	u32 wvalue = value;

#ifdef H2X_RC_L	
	writel(BIT(4) | readl(h2xreg_base + 0x80), h2xreg_base + 0x80);
#else
	writel(BIT(4) | readl(h2xreg_base + 0xC0), h2xreg_base + 0xC0);
#endif

	writel(0x42000001, h2xreg_base + 0x10);
	writel(0x00002000 | (0x1 << (addr & 0x3)), h2xreg_base + 0x14);
	writel(addr & (~0x3), h2xreg_base + 0x18);
	writel(0x00000000, h2xreg_base + 0x1c);

	writel((wvalue << (8 * (addr & 0x3))), h2xreg_base + 0x20);

	//trigger
	writel((readl(h2xreg_base + 0x24) & 0xf) | PCIE_TRIGGER_TX, h2xreg_base + 0x24); 		

	//wait tx idle
	while(!(readl(h2xreg_base + 0x24) & PCIE_TX_IDLE)) {
		timeout++;
		if(timeout > 10000) {
			printk("aspeed_pcie_outb timeout\n");
			return;
		}
	};

	//write clr tx idle
	writel(1, h2xreg_base + 0x08);

#ifdef H2X_RC_L	
	while(!(readl(h2xreg_base + 0x88) & PCIE_RC_RX_DONE_ISR));
	writel(readl(h2xreg_base + 0x88), h2xreg_base + 0x88);
	writel(BIT(4) | readl(h2xreg_base + 0x80), h2xreg_base + 0x80);
#else
	while(!(readl(h2xreg_base + 0xC8) & PCIE_RC_RX_DONE_ISR));
	writel(readl(h2xreg_base + 0xC8), h2xreg_base + 0xC8);
	writel(BIT(4) | readl(h2xreg_base + 0xC0), h2xreg_base + 0xC0);
#endif
}

EXPORT_SYMBOL_GPL(aspeed_pcie_outb);


/* PCIe operations */
static struct pci_ops aspeed_pcie_ops = {
	.read	= aspeed_pcie_rd_conf,
	.write	= aspeed_pcie_wr_conf,
};

/* MSI functions */

/**
 * aspeed_pcie_destroy_msi - Free MSI number
 * @irq: IRQ to be freed
 */
static void aspeed_pcie_destroy_msi(unsigned int irq)
{
	struct msi_desc *msi;
	struct aspeed_pcie *pcie;
	struct irq_data *d = irq_get_irq_data(irq);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	if (!test_bit(hwirq, msi_irq_in_use)) {
		msi = irq_get_msi_desc(irq);
		pcie = msi_desc_to_pci_sysdata(msi);
		dev_err(pcie->dev, "Trying to free unused MSI#%d\n", irq);
	} else {
		clear_bit(hwirq, msi_irq_in_use);
	}
}

/**
 * aspeed_pcie_assign_msi - Allocate MSI number
 *
 * Return: A valid IRQ on success and error value on failure.
 */
static int aspeed_pcie_assign_msi(void)
{
	int pos;

	pos = find_first_zero_bit(msi_irq_in_use, MAX_MSI_HOST_IRQS);
	if (pos < MAX_MSI_HOST_IRQS)
		set_bit(pos, msi_irq_in_use);
	else
		return -ENOSPC;

	return pos;
}

/**
 * aspeed_msi_teardown_irq - Destroy the MSI
 * @chip: MSI Chip descriptor
 * @irq: MSI IRQ to destroy
 */
static void aspeed_msi_teardown_irq(struct msi_controller *chip,
				    unsigned int irq)
{
	aspeed_pcie_destroy_msi(irq);
	irq_dispose_mapping(irq);
}

/**
 * aspeed_pcie_msi_setup_irq - Setup MSI request
 * @chip: MSI chip pointer
 * @pdev: PCIe device pointer
 * @desc: MSI descriptor pointer
 *
 * Return: '0' on success and error value on failure
 */
static int aspeed_pcie_msi_setup_irq(struct msi_controller *chip,
				     struct pci_dev *pdev,
				     struct msi_desc *desc)
{
	struct aspeed_pcie *pcie = pdev->bus->sysdata;
	unsigned int irq;
	int hwirq;
	struct msi_msg msg;

	hwirq = aspeed_pcie_assign_msi();
	if (hwirq < 0)
		return hwirq;

	irq = irq_create_mapping(pcie->msi_domain, hwirq);
	if (!irq)
		return -EINVAL;

	irq_set_msi_desc(irq, desc);

	msg.address_hi = 0;
#ifdef H2X_RC_L	
	msg.address_lo = 0x1e770058;
#else
	msg.address_lo = 0x1e77005C;
#endif
//	msg.data = irq;
	msg.data = hwirq;

	pci_write_msi_msg(irq, &msg);

	return 0;
}


/* MSI Chip Descriptor */
static struct msi_controller aspeed_pcie_msi_chip = {
	.setup_irq = aspeed_pcie_msi_setup_irq,
	.teardown_irq = aspeed_msi_teardown_irq,
};

/* HW Interrupt Chip Descriptor */
static struct irq_chip aspeed_msi_irq_chip = {
	.name = "Aspeed PCIe MSI",
	.irq_enable = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

/**
 * aspeed_pcie_msi_map - Set the handler for the MSI and mark IRQ as valid
 * @domain: IRQ domain
 * @irq: Virtual IRQ number
 * @hwirq: HW interrupt number
 *
 * Return: Always returns 0.
 */
static int aspeed_pcie_msi_map(struct irq_domain *domain, unsigned int irq,
			       irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &aspeed_msi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

/* IRQ Domain operations */
static const struct irq_domain_ops msi_domain_ops = {
	.map = aspeed_pcie_msi_map,
};

static void aspeed_h2x_intx_ack_irq(struct irq_data *d)
{
	struct irq_desc *desc = irq_to_desc(d->irq);
	struct aspeed_pcie *pcie = irq_desc_get_chip_data(desc);

#ifdef H2X_RC_L
	writel(readl(pcie->h2xreg_base + 0x84) | BIT(d->hwirq), pcie->h2xreg_base + 0x84);
#else
	writel(readl(pcie->h2xreg_base + 0xC4) | BIT(d->hwirq), pcie->h2xreg_base + 0xC4);
#endif
}

static void aspeed_h2x_intx_mask_irq(struct irq_data *d)
{
	struct irq_desc *desc = irq_to_desc(d->irq);
	struct aspeed_pcie *pcie = irq_desc_get_chip_data(desc);

	//disable irq
#ifdef H2X_RC_L
	writel(readl(pcie->h2xreg_base + 0x84) & ~BIT(d->hwirq), pcie->h2xreg_base + 0x84);
#else	
	writel(readl(pcie->h2xreg_base + 0xC4) & ~BIT(d->hwirq), pcie->h2xreg_base + 0xC4);
#endif
}

static void aspeed_h2x_intx_unmask_irq(struct irq_data *d)
{
	struct irq_desc *desc = irq_to_desc(d->irq);
	struct aspeed_pcie *pcie = irq_desc_get_chip_data(desc);

	//Enable IRQ ..
#ifdef H2X_RC_L
	writel(readl(pcie->h2xreg_base + 0x84) | BIT(d->hwirq), pcie->h2xreg_base + 0x84);
#else	
	writel(readl(pcie->h2xreg_base + 0xC4) | BIT(d->hwirq), pcie->h2xreg_base + 0xC4);
#endif
}

static struct irq_chip aspeed_h2x_intx_chip = {
	.name		= "Aspeed IntX",
	.irq_ack	= aspeed_h2x_intx_ack_irq,
	.irq_mask	= aspeed_h2x_intx_mask_irq,
	.irq_unmask	= aspeed_h2x_intx_unmask_irq,
};

/* INTx Functions */

/**
 * aspeed_pcie_intx_map - Set the handler for the INTx and mark IRQ as valid
 * @domain: IRQ domain
 * @irq: Virtual IRQ number
 * @hwirq: HW interrupt number
 *
 * Return: Always returns 0.
 */
static int aspeed_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
                                  irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &aspeed_h2x_intx_chip, handle_level_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

/* INTx IRQ Domain operations */
static const struct irq_domain_ops intx_domain_ops = {
    .map = aspeed_pcie_intx_map,
//	.xlate = pci_irqd_intx_xlate,
};

/* PCIe HW Functions */

/**
 * aspeed_pcie_intr_handler - Interrupt Service Handler
 * @irq: IRQ number
 * @data: PCIe port information
 *
 * Return: IRQ_HANDLED on success and IRQ_NONE on failure
 */
 
static irqreturn_t aspeed_pcie_intr_handler(int irq, void *data)
{
	struct aspeed_pcie *pcie = (struct aspeed_pcie *)data;
	u32 bit;	
	u32 virq;
	unsigned long status;
#ifdef H2X_RC_L
	unsigned long intx = readl(pcie->h2xreg_base + 0x88) & 0xf;
#else
	unsigned long intx = readl(pcie->h2xreg_base + 0xc8) & 0xf;
#endif
	int i;

#ifdef H2X_RC_L
	sts0 = readl(pcie->h2xreg_base + 0xc8);
	sts1 = readl(pcie->h2xreg_base + 0xcc);

	if(sts0) {
		writel(sts0, pcie->h2xreg_base + 0xc8);
	}

	if(sts1) {
		writel(sts1, pcie->h2xreg_base + 0xcc);
	}
#else
	//intx isr
	if(intx) {
		for_each_set_bit(bit, &intx, 32) {
			virq = irq_find_mapping(pcie->leg_domain, bit);
			if (virq)
				generic_handle_irq(virq);
			else
				dev_err(pcie->dev, "unexpected Int - X\n");
		}
	}
	//msi isr
	for (i = 0; i < 2; i++) {
			status = readl(pcie->h2xreg_base + 0xe8 + (i * 4));
//			printk("aspeed_pcie_intr_handler  status %lx \n", status);
			writel(status, pcie->h2xreg_base + 0xe8 + (i * 4));
//			printk("read  status %x \n", readl(pcie->h2xreg_base + 0xe8 + (i * 4)));
			if (!status)
					continue;

			for_each_set_bit(bit, &status, 32) {
				if(i) {
					bit += 32;
				}
				virq = irq_find_mapping(pcie->msi_domain, bit);
//				printk("[%d] : find bit %d mapping irq no %d \n", i, bit, virq);
				if (virq)
					generic_handle_irq(virq);
				else
					dev_err(pcie->dev, "unexpected MSI\n");
			}
	}

#endif

	return IRQ_HANDLED;
}

/**
 * aspeed_pcie_init_irq_domain - Initialize IRQ domain
 * @port: PCIe port information
 *
 * Return: '0' on success and error value on failure
 */
static int aspeed_pcie_init_irq_domain(struct aspeed_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct device_node *node = dev->of_node;
	struct device_node *pcie_intc_node;

	/* Setup INTx */
	pcie_intc_node = of_get_next_child(node, NULL);
	if (!pcie_intc_node) {
		dev_err(dev, "No PCIe Intc node found\n");
		return -ENODEV;
	}
	
	pcie->leg_domain = irq_domain_add_linear(pcie_intc_node, MAX_LEGACY_IRQS,
						 &intx_domain_ops,
						 pcie);
	of_node_put(pcie_intc_node);
	if (!pcie->leg_domain) {
			dev_err(dev, "Failed to get a INTx IRQ domain\n");
			return -ENODEV;
	}

	/* Setup MSI */
	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pcie->msi_domain = irq_domain_add_linear(node,
								MAX_MSI_HOST_IRQS,
								&msi_domain_ops,
								&aspeed_pcie_msi_chip);
		if (!pcie->msi_domain) {
		dev_err(dev, "Failed to get a MSI IRQ domain\n");
		return -ENODEV;
		}

		//enable all msi interrupt
#ifdef H2X_RC_L
		writel(0xffffffff, pcie->h2xreg_base + 0xa0);
		writel(0xffffffff, pcie->h2xreg_base + 0xa4);
#else
		writel(0xffffffff, pcie->h2xreg_base + 0xe0);
		writel(0xffffffff, pcie->h2xreg_base + 0xe4);
#endif
	}	

	return 0;
}

/**
 * aspeed_pcie_init_port - Initialize hardware
 * @port: PCIe port information
 */

static void aspeed_pcie_init_port(struct aspeed_pcie *pcie)
{
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

	//todo clr intx isr 
#ifdef H2X_RC_L
	writel(0x0, pcie->h2xreg_base + 0x84);

	//clr msi isr 
	writel(0xFFFFFFFF, pcie->h2xreg_base + 0xa8);
	writel(0xFFFFFFFF, pcie->h2xreg_base + 0xac);

	//rc_l
	writel( PCIE_RX_LINEAR | PCIE_RX_MSI_EN |
			PCIE_Wait_RX_TLP_CLR | PCIE_RC_RX_ENABLE | PCIE_RC_ENABLE,
	pcie->h2xreg_base + 0x80);
	//assign debug tx tag
	writel(0x28, pcie->h2xreg_base + 0xBC);

#else
	writel(0x0, pcie->h2xreg_base + 0xC4);

	//clr msi isr 
	writel(0xFFFFFFFF, pcie->h2xreg_base + 0xe8);
	writel(0xFFFFFFFF, pcie->h2xreg_base + 0xec);

	//rc_h
	writel( PCIE_RX_LINEAR | PCIE_RX_MSI_SEL | PCIE_RX_MSI_EN |
			PCIE_Wait_RX_TLP_CLR | PCIE_RC_RX_ENABLE | PCIE_RC_ENABLE,
	pcie->h2xreg_base + 0xC0);

	//assign debug tx tag
	writel(0x28, pcie->h2xreg_base + 0xFC);
#endif
		
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

}

/**
 * aspeed_pcie_parse_dt - Parse Device tree
 * @port: PCIe port information
 *
 * Return: '0' on success and error value on failure
 */
static int aspeed_pcie_parse_dt(struct aspeed_pcie *pcie, struct platform_device *pdev)
{
	struct device *dev = pcie->dev;
	struct device_node *node = dev->of_node;	
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pciebreg");
	if(res) {
		pcie->pciereg_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(pcie->pciereg_base))
			return PTR_ERR(pcie->pciereg_base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "h2xreg");
	if(res) {
		pcie->h2xreg_base = devm_ioremap_resource(dev, res);
		h2xreg_base = pcie->h2xreg_base;
		if (IS_ERR(pcie->h2xreg_base))
			return PTR_ERR(pcie->h2xreg_base);
	}

	
	pcie->irq = irq_of_parse_and_map(node, 0);
#if 0
	printk("request irq pcie->irq %d ---------------------- \n", pcie->irq);
	err = devm_request_irq(dev, pcie->irq, aspeed_pcie_intr_handler,
						   0,
						   "aspeed-pcie", pcie);
	if (err) {
		dev_err(dev, "unable to request irq %d\n", pcie->irq);
		return err;
	}
#endif
	return 0;
}

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

	aspeed_pcie_init_port(pcie);

	err = aspeed_pcie_init_irq_domain(pcie);
	if (err) {
		dev_err(dev, "Failed creating IRQ Domain\n");
	   goto error;
	}

	err = devm_of_pci_get_host_bridge_resources(dev, 0, 0xff, &res,
						    &iobase);
	if (err) {
		dev_err(dev, "Getting bridge resources failed\n");
		return err;
	}

	err = devm_request_pci_bus_resources(dev, &res);
	if (err)
		goto error;

	list_splice_init(&res, &bridge->windows);
	bridge->dev.parent = dev;
	bridge->sysdata = pcie;
	bridge->busnr = 0;
	bridge->ops = &aspeed_pcie_ops;
	bridge->map_irq = of_irq_parse_and_map_pci;
//	bridge->swizzle_irq = pci_common_swizzle;

#ifdef CONFIG_PCI_MSI
	aspeed_pcie_msi_chip.dev = dev;
	bridge->msi = &aspeed_pcie_msi_chip;
#endif
	err = pci_scan_root_bus_bridge(bridge);
	if (err)
		goto error;

	bus = bridge->bus;

	pci_assign_unassigned_bus_resources(bus);
	list_for_each_entry(child, &bus->children, node)
		pcie_bus_configure_settings(child);
	pci_bus_add_devices(bus);

	err = devm_request_irq(dev, pcie->irq, aspeed_pcie_intr_handler,
						   0,
						   "aspeed-pcie", pcie);
	if (err) {
		dev_err(dev, "unable to request irq %d\n", pcie->irq);
		return err;
	}

	return 0;

error:
	pci_free_resource_list(&res);
	return err;
}

static const struct of_device_id aspeed_pcie_of_match[] = {
	{ .compatible = "aspeed,ast2600-pcie", },
	{},
};

static struct platform_driver aspeed_pcie_driver = {
	.driver = {
		.name	= "aspeed-pcie",
		.of_match_table = aspeed_pcie_of_match,
	},
	.probe = aspeed_pcie_probe,
};

builtin_platform_driver(aspeed_pcie_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED PCIe Host driver");
MODULE_LICENSE("GPL");
