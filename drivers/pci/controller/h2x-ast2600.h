#ifndef __H2X_ASPEED_H_INCLUDED
#define __H2X_ASPEED_H_INCLUDED

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include <linux/delay.h>

#include <linux/pci.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <linux/msi.h>

#include <linux/of_pci.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/irqdomain.h>
#include <linux/aspeed_pcie_io.h>

#if 1
struct aspeed_pcie {
	struct device *dev;

	void __iomem *pciereg_base;
	void __iomem *h2x_rc_base;
	u32 irq;	
	u32 rc_offset;
	u32 msi_address;	

	/* PCIe operations */
	struct pci_ops aspeed_pcie_ops;

	struct resource mem;
	unsigned int busnr;

	/* INTx IRQ Domain operations */	
	struct irq_domain *leg_domain;
	struct irq_chip aspeed_h2x_intx_chip;	
	struct irq_domain_ops intx_domain_ops;

	// msi
	struct irq_domain *msi_domain;
	struct msi_controller aspeed_pcie_msi_chip;
	struct irq_chip aspeed_msi_irq_chip;
	struct irq_domain_ops msi_domain_ops;
	
	struct list_head resources;
	
};

#else
struct aspeed_pcie {
	struct device *dev;

	void __iomem *pciereg_base;
	void __iomem *h2x_rc_base;
	u32 rc_offset;
	u32 msi_address;	

	struct resource mem;
	unsigned int busnr;

	u32 irq;
	struct irq_domain *msi_domain;

	
	struct list_head resources;
	
};







#endif
extern void aspeed_h2x_intx_ack_irq(struct irq_data *d);
extern void aspeed_h2x_intx_mask_irq(struct irq_data *d);
extern void aspeed_h2x_intx_unmask_irq(struct irq_data *d);
extern void aspeed_h2x_msi_enable(struct aspeed_pcie *pcie);
extern void aspeed_h2x_rc_intr_handler(struct aspeed_pcie *pcie);

extern int aspeed_h2x_rd_conf(struct pci_bus *bus, unsigned int devfn, 
				int where, int size, u32 *val);

extern int aspeed_h2x_wr_conf(struct pci_bus *bus, unsigned int devfn, 
				int where, int size, u32 val);

extern void aspeed_h2x_workaround(struct aspeed_pcie *pcie);

extern void aspeed_h2x_rc_init(struct aspeed_pcie *pcie);

#endif
