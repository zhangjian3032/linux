// SPDX-License-Identifier: GPL-2.0
/*******************************************************************************
 *
 * Copyright 2019 Aspeed Technology Inc
 * Ashok Reddy Soma <ashok.soma@aspeedtech.com>
 * This file is based on drivers/net/ethernet/dec/tulip/media.c
 * and drivers/net/ethernet/dec/tulip/timer.c
 * which are written by Donald Becker.
 *
 ********************************************************************************/
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/init.h>
#include "pilot_net.h"

/* CSR9_MII_MANAGEMENT_REG */
#define MII_MGMT_DATA_IN            (1 << 19) /* 0x00080000L */
#define MII_MGMT_RD_WRBAR           (1 << 18) /* 0x00040000L */
#define MII_MGMT_DATA_OUT           (1 << 17) /* 0x00020000L */
#define MII_MGMT_CLK                (1 << 16) /* 0x00010000L */

#define MDI     MII_MGMT_DATA_IN    /* Input */
#define MDRE    MII_MGMT_RD_WRBAR   /* Input if set, Output if not set */
#define MDO     MII_MGMT_DATA_OUT   /* Output */
#define MDC     MII_MGMT_CLK        /* Clock */
#if 0
#define MII_PHY_CTRL_REGNO          0
#define MII_PHY_STATUS_REGNO        1
#define MII_PHY_PHYID_HIGH          2
#define MII_PHY_PHYID_LOW           3

#define LAN_BASE_ADDR		    0x40500000
#define MDIO_MASK                   ~(MDI | MDRE | MDO | MDC)
#define MDIO_SET_BITS(mMdioBits)    { CSR9_MII_MANAGEMENT_REG = ((CSR9_MII_MANAGEMENT_REG & MDIO_MASK) | (mMdioBits)); }
#define LAN_REG32(ofs)          *(volatile unsigned int *)(LAN_BASE_ADDR + ofs)
#define CSR9_MII_MANAGEMENT_REG     LAN_REG32(0x48)
#endif
#define MDIO_RD 0                   /* BIT19 Clear */
#define MDIO_WR MDRE                /* BIT19 Set   */

/* The maximum data clock rate is 2.5 Mhz.  The minimum timing is usually
   met by back-to-back PCI I/O cycles, but we insert a delay to avoid
   "overclocking" issues or future 66Mhz PCI. */
#define pilot_mdio_delay() udelay(1)

/* The maximum data clock rate is 2.5 Mhz.  The minimum timing is usually
   met by back-to-back PCI I/O cycles, but we insert a delay to avoid
   "overclocking" issues or future 66Mhz PCI. */
#define mdio_delay() ioread32(mdio_addr)

/* Read and write the MII registers using software-generated serial
   MDIO protocol.  It is just different enough from the EEPROM protocol
   to not share code.  The maxium data clock rate is 2.5 Mhz. */
#define MDIO_SHIFT_CLK		0x10000
#define MDIO_DATA_WRITE0	0x00000
#define MDIO_DATA_WRITE1	0x20000
#define MDIO_ENB		0x00000 /* Ignore the 0x02000 databook setting. */
#define MDIO_ENB_IN		0x40000
#define MDIO_DATA_READ		0x80000

static const unsigned char comet_miireg2offset[32] = {
	0xB4, 0xB8, 0xBC, 0xC0,  0xC4, 0xC8, 0xCC, 0,  0,0,0,0,  0,0,0,0,
	0,0xD0,0,0,  0,0,0,0,  0,0,0,0, 0, 0xD4, 0xD8, 0xDC, };


/* MII transceiver control section.
   Read and write the MII registers using software-generated serial
   MDIO protocol.  See the MII specifications or DP83840A data sheet
   for details. */

int tulip_mdio_read(struct net_device *dev, int phy_id, int location)
{
	struct tulip_private *tp = netdev_priv(dev);
	int i;
	int read_cmd = (0xf6 << 10) | ((phy_id & 0x1f) << 5) | location;
	int retval = 0;

	void __iomem *ioaddr = tp->base_addr;
	void __iomem *mdio_addr = ioaddr + CSR9;
	unsigned long flags;


	if (tp->chip_id == PILOT_TULIP)
	{
		ioaddr = (void *)IOADDR_INTERFACE_ETH_A; 
	}

	if (location & ~0x1f)
		return 0xffff;

	spin_lock_irqsave(&tp->mii_lock, flags);
	if (tp->chip_id == PILOT_TULIP){
		retval = Lan_MII_MDIO_ReadReg16(dev, phy_id & 0x1f, location);
		spin_unlock_irqrestore(&tp->mii_lock, flags);
		return retval & 0xffff;
	}

	/* Establish sync by sending at least 32 logic ones. */
	for (i = 32; i >= 0; i--) {
		iowrite32(MDIO_ENB | MDIO_DATA_WRITE1, mdio_addr);
		mdio_delay();
		iowrite32(MDIO_ENB | MDIO_DATA_WRITE1 | MDIO_SHIFT_CLK, mdio_addr);
		mdio_delay();
	}
	/* Shift the read command bits out. */
	for (i = 15; i >= 0; i--) {
		int dataval = (read_cmd & (1 << i)) ? MDIO_DATA_WRITE1 : 0;

		iowrite32(MDIO_ENB | dataval, mdio_addr);
		mdio_delay();
		iowrite32(MDIO_ENB | dataval | MDIO_SHIFT_CLK, mdio_addr);
		mdio_delay();
	}
	/* Read the two transition, 16 data, and wire-idle bits. */
	for (i = 19; i > 0; i--) {
		iowrite32(MDIO_ENB_IN, mdio_addr);
		mdio_delay();
		retval = (retval << 1) | ((ioread32(mdio_addr) & MDIO_DATA_READ) ? 1 : 0);
		iowrite32(MDIO_ENB_IN | MDIO_SHIFT_CLK, mdio_addr);
		mdio_delay();
	}

	spin_unlock_irqrestore(&tp->mii_lock, flags);
	return (retval>>1) & 0xffff;
}

void tulip_mdio_write(struct net_device *dev, int phy_id, int location, int val)
{
	struct tulip_private *tp = netdev_priv(dev);
	int i;
	int cmd = (0x5002 << 16) | ((phy_id & 0x1f) << 23) | (location<<18) | (val & 0xffff);
	void __iomem *ioaddr =  (void *)IOADDR_INTERFACE_ETH_A;  //tp->base_addr;
	void __iomem *mdio_addr = ioaddr + CSR9;
	unsigned long flags;

	if (location & ~0x1f)
		return;

	spin_lock_irqsave(&tp->mii_lock, flags);
	if (tp->chip_id == PILOT_TULIP){
		Lan_MII_MDIO_WriteReg16(dev, location, phy_id & 0x1f, val & 0xffff);
		spin_unlock_irqrestore(&tp->mii_lock, flags);
		return;
	}
	/* Establish sync by sending 32 logic ones. */
	for (i = 32; i >= 0; i--) {
		iowrite32(MDIO_ENB | MDIO_DATA_WRITE1, mdio_addr);
		mdio_delay();
		iowrite32(MDIO_ENB | MDIO_DATA_WRITE1 | MDIO_SHIFT_CLK, mdio_addr);
		mdio_delay();
	}
	/* Shift the command bits out. */
	for (i = 31; i >= 0; i--) {
		int dataval = (cmd & (1 << i)) ? MDIO_DATA_WRITE1 : 0;
		iowrite32(MDIO_ENB | dataval, mdio_addr);
		mdio_delay();
		iowrite32(MDIO_ENB | dataval | MDIO_SHIFT_CLK, mdio_addr);
		mdio_delay();
	}
	/* Clear out extra bits. */
	for (i = 2; i > 0; i--) {
		iowrite32(MDIO_ENB_IN, mdio_addr);
		mdio_delay();
		iowrite32(MDIO_ENB_IN | MDIO_SHIFT_CLK, mdio_addr);
		mdio_delay();
	}

	spin_unlock_irqrestore(&tp->mii_lock, flags);
}

/* Set up the transceiver control registers for the selected media type. */
void tulip_select_media(struct net_device *dev, int startup)
{
	struct tulip_private *tp = netdev_priv(dev);
	void __iomem *ioaddr = tp->base_addr;
	struct mediatable *mtable = tp->mtable;
	u32 new_csr6;
	int i;

	if (mtable) {
		struct medialeaf *mleaf = &mtable->mleaf[tp->cur_index];
		unsigned char *p = mleaf->leafdata;
		switch (mleaf->type) {
		case 0:					/* 21140 non-MII xcvr. */
			if (tulip_debug > 1)
				printk(KERN_DEBUG "%s: Using a 21140 non-MII transceiver"
					   " with control setting %2.2x.\n",
					   dev->name, p[1]);
			dev->if_port = p[0];
			if (startup)
				iowrite32(mtable->csr12dir | 0x100, ioaddr + CSR12);
			iowrite32(p[1], ioaddr + CSR12);
			new_csr6 = 0x02000000 | ((p[2] & 0x71) << 18);
			break;
		case 2: case 4: {
			u16 setup[5];
			u32 csr13val, csr14val, csr15dir, csr15val;
			for (i = 0; i < 5; i++)
				setup[i] = get_u16(&p[i*2 + 1]);

			dev->if_port = p[0] & MEDIA_MASK;
			if (tulip_media_cap[dev->if_port] & MediaAlwaysFD)
				tp->full_duplex = 1;

			if (startup && mtable->has_reset) {
				struct medialeaf *rleaf = &mtable->mleaf[mtable->has_reset];
				unsigned char *rst = rleaf->leafdata;
				if (tulip_debug > 1)
					printk(KERN_DEBUG "%s: Resetting the transceiver.\n",
						   dev->name);
				for (i = 0; i < rst[0]; i++)
					iowrite32(get_u16(rst + 1 + (i<<1)) << 16, ioaddr + CSR15);
			}
			if (tulip_debug > 1)
				printk(KERN_DEBUG "%s: 21143 non-MII %s transceiver control "
					   "%4.4x/%4.4x.\n",
					   dev->name, medianame[dev->if_port], setup[0], setup[1]);
			if (p[0] & 0x40) {	/* SIA (CSR13-15) setup values are provided. */
				csr13val = setup[0];
				csr14val = setup[1];
				csr15dir = (setup[3]<<16) | setup[2];
				csr15val = (setup[4]<<16) | setup[2];
				iowrite32(0, ioaddr + CSR13);
				iowrite32(csr14val, ioaddr + CSR14);
				iowrite32(csr15dir, ioaddr + CSR15);	/* Direction */
				iowrite32(csr15val, ioaddr + CSR15);	/* Data */
				iowrite32(csr13val, ioaddr + CSR13);
			} else {
				csr13val = 1;
				csr14val = 0;
				csr15dir = (setup[0]<<16) | 0x0008;
				csr15val = (setup[1]<<16) | 0x0008;
				if (startup) {
					iowrite32(0, ioaddr + CSR13);
					iowrite32(csr14val, ioaddr + CSR14);
				}
				iowrite32(csr15dir, ioaddr + CSR15);	/* Direction */
				iowrite32(csr15val, ioaddr + CSR15);	/* Data */
				if (startup) iowrite32(csr13val, ioaddr + CSR13);
			}
			if (tulip_debug > 1)
				printk(KERN_DEBUG "%s:  Setting CSR15 to %8.8x/%8.8x.\n",
					   dev->name, csr15dir, csr15val);
			if (mleaf->type == 4)
				new_csr6 = 0x82020000 | ((setup[2] & 0x71) << 18);
			else
				new_csr6 = 0x82420000;
			break;
		}
		case 1: case 3: {
			int phy_num = p[0];
			int init_length = p[1];
			u16 *misc_info, tmp_info;

			dev->if_port = 11;
			new_csr6 = 0x020E0000;
			if (mleaf->type == 3) {	/* 21142 */
				u16 *init_sequence = (u16*)(p+2);
				u16 *reset_sequence = &((u16*)(p+3))[init_length];
				int reset_length = p[2 + init_length*2];
				misc_info = reset_sequence + reset_length;
				if (startup)
					for (i = 0; i < reset_length; i++)
						iowrite32(get_u16(&reset_sequence[i]) << 16, ioaddr + CSR15);
				for (i = 0; i < init_length; i++)
					iowrite32(get_u16(&init_sequence[i]) << 16, ioaddr + CSR15);
			} else {
				u8 *init_sequence = p + 2;
				u8 *reset_sequence = p + 3 + init_length;
				int reset_length = p[2 + init_length];
				misc_info = (u16*)(reset_sequence + reset_length);
				if (startup) {
					iowrite32(mtable->csr12dir | 0x100, ioaddr + CSR12);
					for (i = 0; i < reset_length; i++)
						iowrite32(reset_sequence[i], ioaddr + CSR12);
				}
				for (i = 0; i < init_length; i++)
					iowrite32(init_sequence[i], ioaddr + CSR12);
			}
			tmp_info = get_u16(&misc_info[1]);
			if (tmp_info)
				tp->advertising[phy_num] = tmp_info | 1;
			if (tmp_info && startup < 2) {
				if (tp->mii_advertise == 0)
					tp->mii_advertise = tp->advertising[phy_num];
				if (tulip_debug > 1)
					printk(KERN_DEBUG "%s:  Advertising %4.4x on MII %d.\n",
					       dev->name, tp->mii_advertise, tp->phys[phy_num]);
				tulip_mdio_write(dev, tp->phys[phy_num], 4, tp->mii_advertise);
			}
			break;
		}
		case 5: case 6: {
			u16 setup[5];

			new_csr6 = 0; /* FIXME */

			for (i = 0; i < 5; i++)
				setup[i] = get_u16(&p[i*2 + 1]);

			if (startup && mtable->has_reset) {
				struct medialeaf *rleaf = &mtable->mleaf[mtable->has_reset];
				unsigned char *rst = rleaf->leafdata;
				if (tulip_debug > 1)
					printk(KERN_DEBUG "%s: Resetting the transceiver.\n",
						   dev->name);
				for (i = 0; i < rst[0]; i++)
					iowrite32(get_u16(rst + 1 + (i<<1)) << 16, ioaddr + CSR15);
			}

			break;
		}
		default:
			printk(KERN_DEBUG "%s:  Invalid media table selection %d.\n",
					   dev->name, mleaf->type);
			new_csr6 = 0x020E0000;
		}
		if (tulip_debug > 1)
			printk(KERN_DEBUG "%s: Using media type %s, CSR12 is %2.2x.\n",
				   dev->name, medianame[dev->if_port],
				   ioread32(ioaddr + CSR12) & 0xff);
	} else {					/* Unknown chip type with no media table. */
		if (tp->default_port == 0)
			dev->if_port = tp->mii_cnt ? 11 : 3;
		if (tulip_media_cap[dev->if_port] & MediaIsMII) {
			new_csr6 = 0x020E0000;
		} else if (tulip_media_cap[dev->if_port] & MediaIsFx) {
			new_csr6 = 0x02860000;
		} else
			new_csr6 = 0x03860000;
		if (tulip_debug > 1)
			printk(KERN_DEBUG "%s: No media description table, assuming "
				   "%s transceiver, CSR12 %2.2x.\n",
				   dev->name, medianame[dev->if_port],
				   ioread32(ioaddr + CSR12));
	}

	tp->csr6 = new_csr6 | (tp->csr6 & 0xfdff) | (tp->full_duplex ? 0x0200 : 0);

	mdelay(1);

	return;
}
/*
   Check the MII negotiated duplex and change the CSR6 setting if
   required.
   Return 0 if everything is OK.
   Return < 0 if the transceiver is missing or has no link beat.
   */
int tulip_check_duplex(struct net_device *dev)
{
	struct tulip_private *tp = netdev_priv(dev);
	unsigned int bmsr, lpa, negotiated, new_csr6;

	bmsr = tulip_mdio_read(dev, tp->phys[0], MII_BMSR);
	lpa = tulip_mdio_read(dev, tp->phys[0], MII_LPA);
	if (tulip_debug > 1)
		printk(KERN_INFO "%s: MII status %4.4x, Link partner report "
				"%4.4x.\n", dev->name, bmsr, lpa);
	if (bmsr == 0xffff)
		return -2;
	if ((bmsr & BMSR_LSTATUS) == 0) {
		int new_bmsr = tulip_mdio_read(dev, tp->phys[0], MII_BMSR);
		if ((new_bmsr & BMSR_LSTATUS) == 0) {
			if (tulip_debug  > 1)
				printk(KERN_INFO "%s: No link beat on the MII interface,"
						" status %4.4x.\n", dev->name, new_bmsr);
			return -1;
		}
	}
	negotiated = lpa & tp->advertising[0];
	tp->full_duplex = mii_duplex(tp->full_duplex_lock, negotiated);

	new_csr6 = tp->csr6;

	if (negotiated & LPA_100) new_csr6 &= ~TxThreshold;
	else			  new_csr6 |= TxThreshold;
	if (tp->full_duplex) new_csr6 |= FullDuplex;
	else		     new_csr6 &= ~FullDuplex;

	if (new_csr6 != tp->csr6) {
		tp->csr6 = new_csr6;
		tulip_restart_rxtx(tp);

		if (tulip_debug > 0)
			printk(KERN_INFO "%s: Setting %s-duplex based on MII"
					"#%d link partner capability of %4.4x.\n",
					dev->name, tp->full_duplex ? "full" : "half",
					tp->phys[0], lpa);
		return 1;
	}

	return 0;
}

void tulip_media_task(struct work_struct *work)
{
     	struct tulip_private *tp =
                container_of(work, struct tulip_private, media_work);
     	struct net_device *dev = tp->dev;
	void __iomem *ioaddr = tp->base_addr;
	u32 csr12 = ioread32(ioaddr + CSR12);
	int next_tick = 2*HZ;
	unsigned long flags;
#ifndef final_version
	if (tulip_debug > 2) {
		printk(KERN_DEBUG "%s: Media selection tick, %s, status %8.8x mode"
			   " %8.8x SIA %8.8x %8.8x %8.8x %8.8x.\n",
			   dev->name, medianame[dev->if_port], ioread32(ioaddr + CSR5),
			   ioread32(ioaddr + CSR6), csr12, ioread32(ioaddr + CSR13),
			   ioread32(ioaddr + CSR14), ioread32(ioaddr + CSR15));
	}
#endif
	switch (tp->chip_id) {
	case DC21140:
	case DC21142:
	case MX98713:
	case COMPEX9881:
	case DM910X:
	default: {
		struct medialeaf *mleaf;
		unsigned char *p;
		if (tp->mtable == NULL) {	/* No EEPROM info, use generic code. */
			/* Not much that can be done.
			   Assume this a generic MII or SYM transceiver. */
			next_tick = 60*HZ;
#ifndef final_version
			if (tulip_debug > 2)
				printk(KERN_DEBUG "%s: network media monitor CSR6 %8.8x "
					   "CSR12 0x%2.2x.\n",
					   dev->name, ioread32(ioaddr + CSR6), csr12 & 0xff);
#endif
			break;
		}
		mleaf = &tp->mtable->mleaf[tp->cur_index];
		p = mleaf->leafdata;
		switch (mleaf->type) {
		case 0: case 4: {
			/* Type 0 serial or 4 SYM transceiver.  Check the link beat bit. */
			int offset = mleaf->type == 4 ? 5 : 2;
			s8 bitnum = p[offset];
			if (p[offset+1] & 0x80) {
#ifndef final_version
				if (tulip_debug > 1)
					printk(KERN_DEBUG"%s: Transceiver monitor tick "
						   "CSR12=%#2.2x, no media sense.\n",
						   dev->name, csr12);
#endif
				if (mleaf->type == 4) {
					if (mleaf->media == 3 && (csr12 & 0x02))
						goto select_next_media;
				}
				break;
			}
#ifndef final_version
			if (tulip_debug > 2)
				printk(KERN_DEBUG "%s: Transceiver monitor tick: CSR12=%#2.2x"
					   " bit %d is %d, expecting %d.\n",
					   dev->name, csr12, (bitnum >> 1) & 7,
					   (csr12 & (1 << ((bitnum >> 1) & 7))) != 0,
					   (bitnum >= 0));
#endif
			/* Check that the specified bit has the proper value. */
			if ((bitnum < 0) !=
				((csr12 & (1 << ((bitnum >> 1) & 7))) != 0)) {
#ifndef final_version
				if (tulip_debug > 2)
					printk(KERN_DEBUG "%s: Link beat detected for %s.\n", dev->name,
					       medianame[mleaf->media & MEDIA_MASK]);
#endif
				if ((p[2] & 0x61) == 0x01)	/* Bogus Znyx board. */
					goto actually_mii;
				netif_carrier_on(dev);
				break;
			}
			netif_carrier_off(dev);
			if (tp->medialock)
				break;
	  select_next_media:
			if (--tp->cur_index < 0) {
				/* We start again, but should instead look for default. */
				tp->cur_index = tp->mtable->leafcount - 1;
			}
			dev->if_port = tp->mtable->mleaf[tp->cur_index].media;
			if (tulip_media_cap[dev->if_port] & MediaIsFD)
				goto select_next_media; /* Skip FD entries. */
#ifndef final_version
			if (tulip_debug > 1)
				printk(KERN_DEBUG "%s: No link beat on media %s,"
				       " trying transceiver type %s.\n",
				       dev->name, medianame[mleaf->media & MEDIA_MASK],
				       medianame[tp->mtable->mleaf[tp->cur_index].media]);
#endif
			tulip_select_media(dev, 0);
			/* Restart the transmit process. */
			tulip_restart_rxtx(tp);
			next_tick = (24*HZ)/10;
			break;
		}
		case 1:  case 3:		/* 21140, 21142 MII */
		actually_mii:
			if (tulip_check_duplex(dev) < 0) {
				netif_carrier_off(dev);
				next_tick = 3*HZ;
			} else {
				netif_carrier_on(dev);
				next_tick = 60*HZ;
			}
			break;
		case 2:					/* 21142 serial block has no link beat. */
		default:
			break;
		}
	}
	break;
	}

        spin_lock_irqsave(&tp->lock, flags);
        if (tp->timeout_recovery) {
                tulip_tx_timeout_complete(tp, ioaddr);
                tp->timeout_recovery = 0;
        }
        spin_unlock_irqrestore(&tp->lock, flags);

	/* mod_timer synchronizes us with potential add_timer calls
	 * from interrupts.
	 */
	mod_timer(&tp->timer, RUN_AT(next_tick));
}

unsigned int Lan_MII_MDIO_Read(
        struct net_device *dev,             /* Lan port */
        unsigned char DataBitLen        /* Number of bits to read in (maximum 32 bits) */
)
{
    unsigned int Mask;
    unsigned int Data = 0;
//Note in case of our pilot the both the PHYs are accessed form MAC-0 itself
    void __iomem *ioaddr = (void __iomem *)IOADDR_INTERFACE_ETH_A;

    void __iomem *mdio_addr = ioaddr + CSR9;
    unsigned int csr9;
    /* Bit bang in the data */
    for (Mask=1<<(DataBitLen-1);Mask!=0;Mask>>=1)
    {
        /* Bit bang in one bit's worth of data */
                   /* MDRE    | MDO | MDC */
		pilot_mdio_delay();
        iowrite32(MDIO_RD | 0 | 0  , mdio_addr);
		pilot_mdio_delay();
        iowrite32(MDIO_RD | 0 | MDC, mdio_addr);
		pilot_mdio_delay();
        csr9 = ioread32(mdio_addr);
		pilot_mdio_delay();
        if (csr9 & MDI)
        {
            Data |= Mask;
        }
        else
        {
            Data &= ~Mask;
        }
        iowrite32(MDIO_RD | 0 | 0  , mdio_addr);
		pilot_mdio_delay();
    }

    return Data;
}

void Lan_MII_MDIO_Write(
        struct net_device *dev,        /* Lan port */
        unsigned int Data,        /* Data to be written */
        unsigned char  DataBitLen   /* Number of relevant bits in Data */
)
{
//Note in case of our pilot the both the PHYs are accessed form MAC-0 itself
    void __iomem *ioaddr = (void __iomem *)IOADDR_INTERFACE_ETH_A;
    void __iomem *mdio_addr = ioaddr + CSR9;
    unsigned int Mask;

    for (Mask=1<<(DataBitLen-1);Mask!=0;Mask>>=1)
    {
        /* Bit bang out a 1 or a 0 */
        if (Data & Mask)
        {
                       /* MDRE    | MDO | MDC */
		    pilot_mdio_delay();
            iowrite32(MDIO_WR | MDO | 0, mdio_addr);
		    pilot_mdio_delay();
            iowrite32(MDIO_WR | MDO | MDC, mdio_addr);
		    pilot_mdio_delay();
            iowrite32(MDIO_WR | MDO | 0, mdio_addr);
		    pilot_mdio_delay();
        }
        else
        {
                       /* MDRE    | MDO | MDC */
		    pilot_mdio_delay();
            iowrite32(MDIO_WR | 0   | 0, mdio_addr);
		    pilot_mdio_delay();
            iowrite32(MDIO_WR | 0   | MDC, mdio_addr);
		    pilot_mdio_delay();
            iowrite32(MDIO_WR | 0   | 0, mdio_addr);
		    pilot_mdio_delay();
        }
    }
}

void Lan_MII_MDIO_WriteZ(struct net_device *dev)
{
//Note in case of our pilot the both the PHYs are accessed form MAC-0 itself
    void __iomem *ioaddr = (void __iomem *)IOADDR_INTERFACE_ETH_A;
    void __iomem *mdio_addr = ioaddr + CSR9;
               /* MDRE    | MDO | MDC */
    pilot_mdio_delay();
    iowrite32(MDIO_RD | 0 | 0  , mdio_addr);
    pilot_mdio_delay();
    iowrite32(MDIO_RD | 0 | MDC  , mdio_addr);
    pilot_mdio_delay();
    iowrite32(MDIO_RD | 0 | 0  , mdio_addr);
    pilot_mdio_delay();
}

void Lan_MII_MDIO_Release(struct net_device *dev)
{
//Note in case of our pilot the both the PHYs are accessed form MAC-0 itself
    void __iomem *ioaddr = (void __iomem *)IOADDR_INTERFACE_ETH_A;
    void __iomem *mdio_addr = ioaddr + CSR9;

               /* MDRE    | MDO | MDC */
    iowrite32(MDIO_RD | 0 | 0  , mdio_addr);
    pilot_mdio_delay();
}

void Lan_MII_MDIO_WriteReg16(
        struct net_device *dev,        /* Lan port */
        unsigned char  RegNum,      /* MII PHY register number */
        unsigned char phy_id,
        unsigned int Data         /* Data to be written */
)
{
/*    printk("%s: phy_id = 0x%x\n", __FUNCTION__, phy_id); */
    Lan_MII_MDIO_Write(dev, 0xFFFFFFFF, 32);  // Idle = <1111111111111111111111111111111111>
    Lan_MII_MDIO_Write(dev, 0x00000001,  2);  // Start = <01>
    Lan_MII_MDIO_Write(dev, 0x00000001,  2);  // Command = <Write=01>
    Lan_MII_MDIO_Write(dev, phy_id,  5);  // Phy Address = <InternalPHY=00000>
    Lan_MII_MDIO_Write(dev, RegNum,      5);  // PHY reg to write = <XXXXX>
    Lan_MII_MDIO_Write(dev, 0x00000002,  2);  // TurnAround = <01>
    Lan_MII_MDIO_Write(dev, Data,       16);  // Data = <XXXXXXXXXXXXXXXX>
    Lan_MII_MDIO_Release(dev);

}

unsigned int Lan_MII_MDIO_ReadReg16(
        struct net_device *dev,
        unsigned char phy_id, 
        unsigned char RegNum        /* MII PHY register number */
)
{
    unsigned int RegData16;


    Lan_MII_MDIO_Write(dev, 0xFFFFFFFF, 32);    // Idle = <1111111111111111111111111111111111>
    Lan_MII_MDIO_Write(dev, 0x00000001,  2);    // Start = <01>
    Lan_MII_MDIO_Write(dev, 0x00000002,  2);    // Command = <Read=10>
    Lan_MII_MDIO_Write(dev, phy_id,  5);    // Phy Address = <InternalPHY=00000>
    Lan_MII_MDIO_Write(dev, RegNum,      5);    // PHY reg to write = <XXXXX>
    Lan_MII_MDIO_WriteZ(dev);                   // TurnAround = <Z1>
    RegData16 = Lan_MII_MDIO_Read(dev, 16);     // Data = <XXXXXXXXXXXXXXXX>
    Lan_MII_MDIO_WriteZ(dev);                   // TurnAround
    Lan_MII_MDIO_Release(dev);


    return RegData16;
}
