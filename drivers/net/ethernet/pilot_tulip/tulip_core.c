/* tulip_core.c: A DEC 21x4x-family ethernet driver for Linux. */

/*
	Maintained by Jeff Garzik <jgarzik@pobox.com>
	Copyright 2000,2001  The Linux Kernel Team
	Written/copyright 1994-2001 by Donald Becker.

	This software may be used and distributed according to the terms
	of the GNU General Public License, incorporated herein by reference.

	Please refer to Documentation/DocBook/tulip-user.{pdf,ps,html}
	for more information on this driver, or visit the project
	Web page at http://sourceforge.net/projects/tulip/

    Modified for ServerEngines Pilot2 by Amrut Joshi <amrutj@serverengines.com>
*/

/******************************************************************************
 *
 * Copyright (c) 2010-2014, Emulex Corporation.
 *
 * Modifications made by Emulex Corporation under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 *****************************************************************************/

#include <linux/version.h>
#if (LINUX_VERSION_CODE <  KERNEL_VERSION(2,6,26))	
#include <linux/config.h>
#endif
#define DRV_NAME	"tulip"
#ifdef CONFIG_TULIP_NAPI
#define DRV_VERSION    "1.1.13-NAPI" /* Keep at least for test */
#else
#define DRV_VERSION	"1.1.13"
#endif
#define DRV_RELDATE	"May 11, 2002"

#include <linux/module.h>
#include <linux/pci.h>
#include "tulip.h"
#include <linux/init.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/ethtool.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <uapi/linux/ethtool.h>
#include <linux/crc32.h>
#include <asm/unaligned.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/proc_fs.h>
#ifdef __sparc__
#include <asm/pbm.h>
#endif
#include "pilot_realtek_phy.h"

#define PILOT_PAUSE_FR_ENABLE	1

#define TXPBL           0x0
#define RXPBL           0x0
#define I2C_SMB_CTRL_2  0x40100A08

extern unsigned int IO_ADDRESS(unsigned int);
/* By default enable DDMA for both Tx and Rx */
static unsigned int ddma = 3;	//ddma=1->tx, 2->rx, 3->both

static char version[] =
	"Linux Pilot4 Tulip driver version " DRV_VERSION " (" DRV_RELDATE ")\n";

static const char filename0[] = "pilotmac0stats";
static const char filename1[] = "pilotmac1stats";
static struct proc_dir_entry *proc_mac0stats = NULL;
static struct proc_dir_entry *proc_mac1stats = NULL;
volatile static long *dev_iobaseaddr0 = NULL;
volatile static long *dev_iobaseaddr1 = NULL;


/* A few user-configurable values. */

/* Maximum events (Rx packets, etc.) to handle at each interrupt. */
static unsigned int max_interrupt_work = 25;

struct net_device *pilot2_devices[2];
#define MAX_UNITS 8
/* Used to pass the full-duplex flag, etc. */
static int full_duplex[MAX_UNITS];
static int options[MAX_UNITS];
static int mtu[MAX_UNITS];			/* Jumbo MTU for interfaces. */

/*  The possible media types that can be set in options[] are: */
const char * const medianame[32] = {
	"10baseT", "10base2", "AUI", "100baseTx",
	"10baseT-FDX", "100baseTx-FDX", "100baseT4", "100baseFx",
	"100baseFx-FDX", "MII 10baseT", "MII 10baseT-FDX", "MII",
	"10baseT(forced)", "MII 100baseTx", "MII 100baseTx-FDX", "MII 100baseT4",
	"MII 100baseFx-HDX", "MII 100baseFx-FDX", "Home-PNA 1Mbps", "Invalid-19",
	"","","","", "","","","",  "","","","Transceiver reset",
};

/* Set the copy breakpoint for the copy-only-tiny-buffer Rx structure. */
#if defined(__alpha__) || defined(__arm__) || defined(__hppa__) \
	|| defined(__sparc_) || defined(__ia64__) \
	|| defined(__sh__) || defined(__mips__)
//static int rx_copybreak = 1518;
static int rx_copybreak = 0;
//static int rx_copybreak = 100;
#else
static int rx_copybreak = 100;
#endif

/*
  Set the bus performance register.
	Typical: Set 16 longword cache alignment, no burst limit.
	Cache alignment bits 15:14	     Burst length 13:8
		0000	No alignment  0x00000000 unlimited		0800 8 longwords
		4000	8  longwords		0100 1 longword		1000 16 longwords
		8000	16 longwords		0200 2 longwords	2000 32 longwords
		C000	32  longwords		0400 4 longwords
	Warning: many older 486 systems are broken and require setting 0x00A04800
	   8 longword cache alignment, 8 longword burst.
	ToDo: Non-Intel setting could be better.
*/

#if defined(__alpha__) || defined(__ia64__)
static int csr0 = 0x01A00000 | 0xE000;
#elif defined(__i386__) || defined(__powerpc__) || defined(__x86_64__)
static int csr0 = 0x01A00000 | 0x8000;
#elif defined(__sparc__) || defined(__hppa__)
/* The UltraSparc PCI controllers will disconnect at every 64-byte
 * crossing anyways so it makes no sense to tell Tulip to burst
 * any more than that.
 */
static int csr0 = 0x01A00000 | 0x9000;
#elif defined(__arm__) || defined(__sh__)
static int csr0 = 0x01A00000 | 0x4800;
#elif defined(__mips__)
static int csr0 = 0x00200000 | 0x4000;
#else
#warning Processor architecture undefined!
//static int csr0 = 0x00A00000 | 0x4800;
static int csr0 = 0x00A00000 | 0x2000;
#endif

/* Operational parameters that usually are not changed. */
/* Time in jiffies before concluding the transmitter is hung. */
#define TX_TIMEOUT  (4*HZ)


MODULE_AUTHOR("The Linux Kernel Team");
MODULE_DESCRIPTION("Digital 21*4* Tulip ethernet driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
module_param(tulip_debug, int, 0);
module_param(max_interrupt_work, int, 0);
module_param(rx_copybreak, int, 0);
module_param(csr0, int, 0);
module_param_array(options, int, NULL, 0);
module_param_array(full_duplex, int, NULL, 0);
module_param(ddma, int, 0); 

#define PFX DRV_NAME ": "
#ifdef TULIP_DEBUG
int tulip_debug = TULIP_DEBUG;
#else
int tulip_debug = 1;
#endif

int tulip_rx(struct net_device *dev);


/*
 * This table use during operation for capabilities and media timer.
 *
 * It is indexed via the values in 'enum chips'
 */

struct tulip_chip_table tulip_tbl[] = {
  { }, /* placeholder for array, slot unused currently */
  { }, /* placeholder for array, slot unused currently */

  /* DC21140 */
  { "Digital DS21140 Tulip", 128, 0x0001ebef,
	HAS_MII | HAS_MEDIA_TABLE | CSR12_IN_SROM | HAS_PCI_MWI, tulip_timer },

  /* DC21142, DC21143 */
  { "Digital DS21143 Tulip", 128, 0x0801fbff,
	HAS_MII | HAS_MEDIA_TABLE | ALWAYS_CHECK_MII | HAS_ACPI | HAS_NWAY
	| HAS_INTR_MITIGATION | HAS_PCI_MWI, t21142_timer },

  /* LC82C168 */
  { "Lite-On 82c168 PNIC", 256, 0x0001fbef,
	HAS_MII | HAS_PNICNWAY, pnic_timer },

  /* MX98713 */
  { "Macronix 98713 PMAC", 128, 0x0001ebef,
	HAS_MII | HAS_MEDIA_TABLE | CSR12_IN_SROM, mxic_timer },

  /* MX98715 */
  { "Macronix 98715 PMAC", 256, 0x0001ebef,
	HAS_MEDIA_TABLE, mxic_timer },

  /* MX98725 */
  { "Macronix 98725 PMAC", 256, 0x0001ebef,
	HAS_MEDIA_TABLE, mxic_timer },

  /* AX88140 */
  { "ASIX AX88140", 128, 0x0001fbff,
	HAS_MII | HAS_MEDIA_TABLE | CSR12_IN_SROM | MC_HASH_ONLY
	| IS_ASIX, tulip_timer },

  /* PNIC2 */
  { "Lite-On PNIC-II", 256, 0x0801fbff,
	HAS_MII | HAS_NWAY | HAS_8023X | HAS_PCI_MWI, pnic2_timer },

  /* COMET */
  { "ADMtek Comet", 256, 0x0001abef,
	HAS_MII | MC_HASH_ONLY | COMET_MAC_ADDR, comet_timer },

  /* COMPEX9881 */
  { "Compex 9881 PMAC", 128, 0x0001ebef,
	HAS_MII | HAS_MEDIA_TABLE | CSR12_IN_SROM, mxic_timer },

  /* I21145 */
  { "Intel DS21145 Tulip", 128, 0x0801fbff,
	HAS_MII | HAS_MEDIA_TABLE | ALWAYS_CHECK_MII | HAS_ACPI
	| HAS_NWAY | HAS_PCI_MWI, t21142_timer },

  /* DM910X */
  { "Davicom DM9102/DM9102A", 128, 0x0001ebef,
	HAS_MII | HAS_MEDIA_TABLE | CSR12_IN_SROM | HAS_ACPI,
	tulip_timer },

  /* RS7112 */
  { "Conexant LANfinity", 256, 0x0001ebef,
	HAS_MII | HAS_ACPI, tulip_timer },

  /* Pilot2 */
  { "Pilot4 Tulip", 128, 0x00018942, HAS_MII | HAS_PHY_IRQ, tulip_timer }, 
 
};

/* A full-duplex map for media types. */
const char tulip_media_cap[32] =
{0,0,0,16,  3,19,16,24,  27,4,7,5, 0,20,23,20,  28,31,0,0, };

static void tulip_tx_timeout(struct net_device *dev);
static void tulip_init_ring(struct net_device *dev);
static int tulip_start_xmit(struct sk_buff *skb, struct net_device *dev);
static int tulip_open(struct net_device *dev);
static int tulip_close(struct net_device *dev);
void tulip_up(struct net_device *dev);
static void tulip_down(struct net_device *dev);
static struct net_device_stats *tulip_get_stats(struct net_device *dev);
static int private_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);
static void set_rx_mode(struct net_device *dev);
#ifdef CONFIG_NET_POLL_CONTROLLER
static void poll_tulip(struct net_device *dev);
#endif

static void tulip_set_power_state (struct tulip_private *tp, int sleep, int snooze)
{}

void pilot_up(struct net_device *dev)
{
  struct tulip_private *tp = netdev_priv(dev);
  void __iomem *ioaddr = tp->base_addr;
  unsigned short status;
  unsigned short lnk_status;
  unsigned short cntrl = 0;

  if(tp->phys[0]==0xff)
    {//100Mbps Full
      tp->full_duplex = 1;
      printk("There is no phy detected on this interface assuming NC-SI\n");
      printk("Setting 100 BASE  Full Duplex mode.\n");
      tp->csr6 &= ~((0x1 << 16) | (0x1 << 17)); //Telling MAC we are 100Mbps
      tp->csr6 |= (0x1 << 9);
      tp->full_duplex = 1;
      tp->speed_neg = SPEED_NEG_100;
    }
    else if( (PHY_SMSC == tp->phy_manuf_model_num)||(PHY_SMSC_REVB == tp->phy_manuf_model_num))
      {
	status = tulip_mdio_read(dev, tp->phys[0], 31);
	printk("MAC-1 SMSC PHY STS %x\n",status);
	tp->csr6 = ((0x1<<1) | (0x1<<30));
	
	status=status>>2;
	if((status & 0x7) == 0x1)
	  { //10Mbps Half
	    //tulip_mdio_write(dev, tp->phys[0], PILOT_RT_BMCR, cntrl & ~(BMCR_SET_DUPLX));
	    printk("Setting 10 BASE  Half  Duplex Mode.\n");
	    tp->full_duplex = 0;
	    tp->csr6 |= (0x1 << 17); // Telling MAC we are 10Mbps
	    tp->csr6 &= ~(0x1 << 16);
	    tp->csr6 &= (~(0x1 << 9));
	    tp->speed_neg = SPEED_NEG_10;
	  }
	else if ((status & 0x7) == 0x5)
	  { //10Mbps Full
	    tp->full_duplex = 1;
	    printk("Setting 10 BASE  Full Duplex Mode.\n");
	    tp->csr6 |= (0x1 << 17); // Telling MAC we are 10Mbps
	    tp->csr6 &= ~(0x1 << 16);
	    tp->csr6 |= (0x1 << 9);
	    tp->speed_neg = SPEED_NEG_10;

	  }
	else if((status & 0x7) == 0x2)
	  {//100Mbps Half
	    printk("Setting 100 BASE Half  duplex mode.\n");
	    tp->full_duplex = 0;
	    tp->csr6 &= ~((0x1 << 16) | (0x1 << 17)); //Telling MAC we are 100Mbps
	    tp->csr6 &= (~(0x1 << 9));
	    tp->speed_neg = SPEED_NEG_100;
	  }
	else if ((status & 0x7) == 0x6)
	  {//100Mbps Full
	    tp->full_duplex = 1;
	    printk("Setting 100 BASE  Full Duplex mode.\n");
	    tp->csr6 &= ~((0x1 << 16) | (0x1 << 17)); //Telling MAC we are 100Mbps
	    tp->csr6 |= (0x1 << 9);
	    tp->full_duplex = 1;
	    tp->speed_neg = SPEED_NEG_100;
	  }
	else{
	  lnk_status = tulip_mdio_read(dev, tp->phys[0], 1);
	  if(lnk_status&4)
	    {
	      tp->speed_neg = SPEED_NEG_10;
	      printk("Unknown Speed Value(4:2) %x\n", status);
	    }
	  else
	    {
	      tp->speed_neg = SPEED_NEG_10;
	      printk("Link is down lnk sts %x spd sts(4:2) %x\n",lnk_status,status);
	    }

	} 
      }
    else if (PHY_REALTEK == tp->phy_manuf_model_num)
      {
	status = tulip_mdio_read(dev, tp->phys[0], PILOT_RT_PHYSR);
	printk("This  interface using  REALTEK PHY Status is %x\n",status);
    	tp->csr6 = ((0x1<<1) | (0x1<<30));
	
    	if(status & REG0x11_FULL_DUPLX_MODE)
	  {
	    printk("Setting full duplex mode.\n");
	    tp->csr6 |= (0x1 << 9);
	    tp->full_duplex = 1;
	  }
	else 
	  {
	    printk("Setting half duplex mode\n");
	    tp->full_duplex = 0;
	    tp->csr6 &= (~(0x1 << 9));
	  }
	
    	if((status & (REG0x11_1000Mbps | REG0x11_100Mbps | REG0x11_10Mbps)) == REG0x11_100Mbps)
	  {
	    printk("Setting 100 BASE-X\n");
	    tp->csr6 &= ~((0x1 << 16) | (0x1 << 17));
	    tp->speed_neg = SPEED_NEG_100;
	  } 
	else if ((status & (REG0x11_1000Mbps | REG0x11_100Mbps | REG0x11_10Mbps)) == REG0x11_1000Mbps)
	  {
	    printk("Setting 1000 BASE-X\n");
	    tp->csr6 |= ((0x1 << 16) | (0x1 << 17));
	    tp->speed_neg = SPEED_NEG_1000;
	  } 
	else 
	  {
	    printk("Setting 10 BASE\n");
	    tp->csr6 |= (0x1 << 17);
	    tp->csr6 &= ~(0x1 << 16);
	    tp->speed_neg = SPEED_NEG_10;
	  }
      }
    else if (PHY_MICREL == tp->phy_manuf_model_num)
      {
	tp->csr6 = ((0x1<<1) | (0x1<<30));

	//set PILOT4 MACRGMIICTL register to 0x200, Otherwise we will see
	//Tx issues on MAC0 in 1000Mbps speed
//	*((volatile unsigned long *)(SE_SYS_CLK_VA_BASE + 0x88)) = 0x200;
	*((volatile unsigned long *)IO_ADDRESS(0x40100100 + 0x88)) = 0x200;
	
	status = tulip_mdio_read(dev, tp->phys[0],Mircel_Phy_Control);
	status=( (status>>3)&(0xF) );
	printk(KERN_DEBUG "MICREL Micrel_Phy_Control Status is %x\n",status);
	switch(status)
	  {
	  case 2:
	    {
	      printk("Setting 10Base-T Half Duplex\n");
	      tp->full_duplex = 0;
	      tp->csr6 |= (0x1 << 17);
	      tp->csr6 &= ~(0x1 << 16);
	      tp->csr6 &= (~(0x1 << 9));
	      tp->speed_neg = SPEED_NEG_10;
	      break;
	    }
	    
	  case 3:
	    {
	      printk("Setting 10Base-T Full Duplex\n");
	      tp->full_duplex = 1;
	      tp->csr6 |= (0x1 << 17);
	      tp->csr6 &= ~(0x1 << 16);
	      tp->csr6 |= ((0x1 << 9));
	      tp->speed_neg = SPEED_NEG_10;
	      break;
	    }
	  case 4:
	    {
	      printk("Setting 100Base-TX Half Duplex\n");
	      tp->full_duplex = 0;
	      tp->csr6 &= ~((0x1 << 16) | (0x1 << 17));
	      tp->csr6 &= (~(0x1 << 9));
	      tp->speed_neg = SPEED_NEG_100;
	      break;
	    }
	  case 5:
	    {
	      printk("Setting 100Base-TX Full Duplex\n");
	      tp->full_duplex = 1;
	      tp->csr6 &= ~((0x1 << 16) | (0x1 << 17));
	      tp->csr6 |= ((0x1 << 9));
	      tp->speed_neg = SPEED_NEG_100;
	      break;
	    }
	  case 8:
	    {
	      printk("Setting 1000Base Half Duplex\n");
	      tp->full_duplex = 0;
	      tp->csr6 |= ((0x1 << 16) | (0x1 << 17));
	      tp->csr6 &= (~(0x1 << 9));
	      tp->speed_neg = SPEED_NEG_1000;
	      break;
	    }
	  case 9:
	    {
	      printk("Setting 1000Base Full  Duplex\n");
	      tp->full_duplex = 1;
	      tp->csr6 |= ((0x1 << 16) | (0x1 << 17));
	      tp->csr6 |= ((0x1 << 9));
	      tp->speed_neg = SPEED_NEG_1000;
	      break;
	    }
	  default:
	    {
	    //  printk("Auto-negotiation failed\n");
	    tp->speed_neg = SPEED_NEG_10;
	    break;
	    }
	  }
      }
    else if (PHY_MARVELL == tp->phy_manuf_model_num)
      {
	tp->csr6 = ((0x1<<1) | (0x1<<30));
	
	status = tulip_mdio_read(dev, tp->phys[0],17);
	if(status & 0x800) {
	  status = ((status >> 13) & 0x7);
	  printk(KERN_DEBUG "MARVELL Marvell_Phy_Control Status is %x\n",status);
	  switch(status)
	    {
	   case 0:
	      {
	        printk("Setting 10Base-T Half Duplex\n");
	        tp->full_duplex = 0;
	        tp->csr6 |= (0x1 << 17);
	        tp->csr6 &= ~(0x1 << 16);
	        tp->csr6 &= (~(0x1 << 9));
	        tp->speed_neg = SPEED_NEG_10;
	        break;
	      }
	    
	   case 1:
	     {
	        printk("Setting 10Base-T Full Duplex\n");
	        tp->full_duplex = 1;
	        tp->csr6 |= (0x1 << 17);
	        tp->csr6 &= ~(0x1 << 16);
	        tp->csr6 |= ((0x1 << 9));
	        tp->speed_neg = SPEED_NEG_10;
	        break;
             }  
	   case 2:
             {
	        printk("Setting 100Base-TX Half Duplex\n");
	        tp->full_duplex = 0;
	        tp->csr6 &= ~((0x1 << 16) | (0x1 << 17));
	        tp->csr6 &= (~(0x1 << 9));
	        tp->speed_neg = SPEED_NEG_100;
	        break;
	     }
	   case 3:
	     {
	        printk("Setting 100Base-TX Full Duplex\n");
	        tp->full_duplex = 1;
	        tp->csr6 &= ~((0x1 << 16) | (0x1 << 17));
	        tp->csr6 |= ((0x1 << 9));
	        tp->speed_neg = SPEED_NEG_100;
	        break;
	     }
	   case 4:
	     {
	        printk("Setting 1000Base Half Duplex\n");
	        tp->full_duplex = 0;
	        tp->csr6 |= ((0x1 << 16) | (0x1 << 17));
	        tp->csr6 &= (~(0x1 << 9));
	        tp->speed_neg = SPEED_NEG_1000;
	        break;
	     }
	   case 5:
	     {
	        printk("Setting 1000Base Full  Duplex\n");
	        tp->full_duplex = 1;
	        tp->csr6 |= ((0x1 << 16) | (0x1 << 17));
	        tp->csr6 |= ((0x1 << 9));
	        tp->speed_neg = SPEED_NEG_1000;
	        break;
	     }
	   default:
	     {
	        //  printk("Auto-negotiation failed\n");
	        tp->speed_neg = SPEED_NEG_10;
	        break;
	     }
           }
	}
      }
    else if (PHY_BROADCOM == tp->phy_manuf_model_num)
      {
	status = tulip_mdio_read(dev, tp->phys[0], 0x18);
    	tp->csr6 = ((0x1<<1) | (0x1<<30));
    	cntrl = tulip_mdio_read(dev, tp->phys[0], 0);
#ifndef final_version
    	if (tulip_debug > 4)
	  printk("cntrl = 0x%x\n", cntrl);
#endif
    	if(status & 0x00000001)
	  {
	    printk("Setting full duplex mode.\n");
	    tp->csr6 |= (0x1 << 9);
	    tulip_mdio_write(dev, tp->phys[0], 0, cntrl | 0x100);
	    tp->full_duplex = 1;
	  } 
	else 
	  {
	    printk("Setting half duplex mode\n");
	    tulip_mdio_write(dev, tp->phys[0], 0, cntrl & ~0x100);
	    tp->full_duplex = 0;
	  }
    	if(status & 0x00000002)
	  {
	    printk("Setting 100 BASE-X\n");
	    tp->speed_neg = SPEED_NEG_100;
	  } 
	else 
	  {
	    printk("Setting 10 BASE-T\n");
	    tp->csr6 |= (0x1 << 17);
	    tp->speed_neg = SPEED_NEG_10;
	  }
    	cntrl = tulip_mdio_read(dev, tp->phys[0], 0x1A);
    	cntrl = (cntrl | (1 << 14)) & ~(0xf << 8);
    	tulip_mdio_write(dev, tp->phys[0], 0x1A, cntrl);
	
      } 
    else
      {
	printk("\nERROR ERROR Unknown PHY MANUF received\n");
	tp->speed_neg = SPEED_NEG_10;
      }

    
    if(tp->speed_neg==SPEED_NEG_10)
      {
	iowrite32( (CSR11_TIM_10MB), (ioaddr + CSR11)); 
      }	
    else if(tp->speed_neg==SPEED_NEG_100)
      {		
	iowrite32( (CSR11_TIM_100MB), (ioaddr + CSR11)); 
      }	
    else	
      {	
	iowrite32( (CSR11_TIM), (ioaddr + CSR11)); 
      }	
        

    return;
}

void tulip_up(struct net_device *dev)
{
	struct tulip_private *tp = netdev_priv(dev);
	void __iomem *ioaddr = tp->base_addr;
	int next_tick = 3*HZ;
	int i;
#ifdef  PILOT_PAUSE_FR_ENABLE
	int phy_id = tp->phys[0] & 0x1f;
	u16 *mac_addrs = (u16 *)dev->dev_addr;
	unsigned int MACL;
	unsigned int MACH;
#endif
	unsigned int ddma_en=0;
	u32 rxpbl=0;
	u32 txpbl=0;
	volatile int timeout = 1000000;

#ifdef CONFIG_TULIP_NAPI
	napi_enable(&tp->napi);
#endif

	/* Wake the chip from sleep/snooze mode. */
	tulip_set_power_state (tp, 0, 0);
	iowrite32(0x00000000, ioaddr + CSR6);

	/* On some chip revs we must set the MII/SYM port before the reset!? */
	if (tp->mii_cnt  ||  (tp->mtable  &&  tp->mtable->has_mii))
		iowrite32(0x00040000, ioaddr + CSR6);

	/* Reset the chip, holding bit 0 set at least 50 PCI cycles. */

	iowrite32((ioread32(ioaddr + CSR0)|0x1), ioaddr + CSR0);
/*	printk("waiting for mac s/w reset to be  done\n");	*/
	 while(ioread32(ioaddr + CSR0)&0x1 )
	 {
		 timeout--;
		 if(timeout <= 0) 
		 {
			 printk("%s %d Condition NOT met even after wait!!! Breaking while loop!!!\n", __FUNCTION__, __LINE__);
#ifdef CONFIG_PANIC_AND_BAIL_OUT
			 panic("");
#endif
			 break;
		 }
		 udelay(1);
	 }
/*	printk("mac s/w reset done\n"); */
	 iowrite32(tp->csr0, ioaddr + CSR0);

/*	start with smaller value,if not when connected to 10MB and switched to 1GIG
	 it will take 1Hr to get timer interpt. */
	 
	 iowrite32( (CSR11_TIM_10MB), (ioaddr + CSR11)); 
	 
#ifdef PILOT_PAUSE_FR_ENABLE
	 if(PHY_MICREL != tp->phy_manuf_model_num)
	 {
		 MACL=mac_addrs[0];
		 MACL|=(mac_addrs[1]<<16);
		 MACH=mac_addrs[2];

		 iowrite32(MACL, (ioaddr + CSR16));
		 iowrite16(MACH, (ioaddr + CSR17));
		 iowrite32(0x00000fff, (ioaddr + CSR18) );
		 iowrite32(0x00800040, (ioaddr + CSR19) );
		 iowrite32( (ioread32(ioaddr + CSR20)|0xF0000000), (ioaddr+ CSR20) );
		 tulip_mdio_write (dev, phy_id & 0x1f, 4, (tulip_mdio_read(dev, phy_id & 0x1f, 4) | 0xC00));
	 } else {
		printk("Pause frames will not be supported for MICREL phy.\nPlease read Errata of Micrel Phy. phy id = %x\n", tp->phy_manuf_model_num);
	 }
#endif

	if(ddma)
	{
	/*	To check if TX/RX process is running */
		txpbl = ioread32(ioaddr+CSR0);
		txpbl = (txpbl & 0xFFFFC0FF) | (TXPBL << 8);
		iowrite32(txpbl, ioaddr + CSR0); //Tx pbl 

		rxpbl = ioread32((volatile u32 *)IO_ADDRESS(I2C_SMB_CTRL_2));
		if((u32)tp->base_addr == (u32)IOADDR_INTERFACE_ETH_A)
			rxpbl = (rxpbl & 0xFFC0FFFF) | (RXPBL << 16);
		if((u32)tp->base_addr == (u32)IOADDR_INTERFACE_ETH_B)
			rxpbl = (rxpbl & 0xC0FFFFFF) | (RXPBL << 24);
		iowrite32(rxpbl, (volatile u32 *)IO_ADDRESS(I2C_SMB_CTRL_2)); //Rx pbl

		ddma_en = ddma << 30;                       //ddma=1->tx, 2->rx, 3->both
		iowrite32( 0xC0000000, (ioaddr + CSR21) ); /* Enabling DDMA */
	}

	/*	Enable Statistical counters Accumulation	*/
	iowrite32( (ioread32(ioaddr+CSR22) | (1<<31)), (ioaddr + CSR22) ); 

#ifndef final_version
	if (tulip_debug > 1)
	  printk(KERN_DEBUG "%s: tulip_up(), irq==%d.\n", dev->name, dev->irq);
#endif
	
	iowrite32(tp->rx_ring_dma, ioaddr + CSR3);
	iowrite32(tp->tx_ring_dma, ioaddr + CSR4);
	tp->cur_rx = tp->cur_tx = 0;
	tp->dirty_rx = tp->dirty_tx = 0;
	
	if (tp->flags & MC_HASH_ONLY) 
	  {
	    u32 addr_low = le32_to_cpu(get_unaligned((u32 *)dev->dev_addr));
	    u32 addr_high = le16_to_cpu(get_unaligned((u16 *)(dev->dev_addr+4)));
	    if (tp->chip_id == AX88140) 
	      {
		iowrite32(0, ioaddr + CSR13);
		iowrite32(addr_low,  ioaddr + CSR14);
		iowrite32(1, ioaddr + CSR13);
		iowrite32(addr_high, ioaddr + CSR14);
	      }
	    else if (tp->flags & COMET_MAC_ADDR) 
	      {
		iowrite32(addr_low,  ioaddr + 0xA4);
		iowrite32(addr_high, ioaddr + 0xA8);
		iowrite32(0, ioaddr + 0xAC);
		iowrite32(0, ioaddr + 0xB0);
	      }
	  }
	else 
	  {
	    /* This is set_rx_mode(), but without starting the transmitter. */
	    u16 *eaddrs = (u16 *)dev->dev_addr;
	    u16 *setup_frm = &tp->setup_frame[15*6];
	    dma_addr_t mapping;
	    
	    /* 21140 bug: you must add the broadcast address. */
	    memset(tp->setup_frame, 0xff, sizeof(tp->setup_frame));
	    /* Fill the final entry of the table with our physical address. */
	    *setup_frm++ = eaddrs[0]; *setup_frm++ = eaddrs[0];
	    *setup_frm++ = eaddrs[1]; *setup_frm++ = eaddrs[1];
	    *setup_frm++ = eaddrs[2]; *setup_frm++ = eaddrs[2];
	    
	    mapping = pci_map_single(tp->pdev, tp->setup_frame,
				     sizeof(tp->setup_frame),
				     PCI_DMA_TODEVICE);
	    tp->tx_buffers[tp->cur_tx].skb = NULL;
	    tp->tx_buffers[tp->cur_tx].mapping = mapping;
	    
	    /* Put the setup frame on the Tx list. */
	    tp->tx_ring[tp->cur_tx].length = cpu_to_le32(0x08000000 | 192);
	    tp->tx_ring[tp->cur_tx].buffer1 = cpu_to_le32(mapping);
	    tp->tx_ring[tp->cur_tx].status = cpu_to_le32(DescOwned);
	    
	    tp->cur_tx++;
	  }
	
	tp->saved_if_port = dev->if_port;
	if (dev->if_port == 0)
		dev->if_port = tp->default_port;

	/* Allow selecting a default media. */
	i = 0;
	if (tp->mtable == NULL)
		goto media_picked;
	if (dev->if_port) {
		int looking_for = tulip_media_cap[dev->if_port] & MediaIsMII ? 11 :
			(dev->if_port == 12 ? 0 : dev->if_port);
		for (i = 0; i < tp->mtable->leafcount; i++)
			if (tp->mtable->mleaf[i].media == looking_for) {
				printk(KERN_INFO "%s: Using user-specified media %s.\n",
					   dev->name, medianame[dev->if_port]);
				goto media_picked;
			}
	}
	if ((tp->mtable->defaultmedia & 0x0800) == 0) {
		int looking_for = tp->mtable->defaultmedia & MEDIA_MASK;
		for (i = 0; i < tp->mtable->leafcount; i++)
			if (tp->mtable->mleaf[i].media == looking_for) {
				printk(KERN_INFO "%s: Using EEPROM-set media %s.\n",
					   dev->name, medianame[looking_for]);
				goto media_picked;
			}
	}
	/* Start sensing first non-full-duplex media. */
	for (i = tp->mtable->leafcount - 1;
		 (tulip_media_cap[tp->mtable->mleaf[i].media] & MediaAlwaysFD) && i > 0; i--)
		;
media_picked:

	tp->csr6 = 0;
	tp->cur_index = i;
	tp->nwayset = 0;

	if (dev->if_port) {
		if (tp->chip_id == DC21143  &&
		    (tulip_media_cap[dev->if_port] & MediaIsMII)) {
			/* We must reset the media CSRs when we force-select MII mode. */
			iowrite32(0x0000, ioaddr + CSR13);
			iowrite32(0x0000, ioaddr + CSR14);
			iowrite32(0x0008, ioaddr + CSR15);
		}
		tulip_select_media(dev, 1);
	} else if (tp->chip_id == DC21142) {
		if (tp->mii_cnt) {
			tulip_select_media(dev, 1);
#ifndef final_version
			if (tulip_debug > 1)
				printk(KERN_INFO "%s: Using MII transceiver %d, status "
					   "%4.4x.\n",
					   dev->name, tp->phys[0], tulip_mdio_read(dev, tp->phys[0], 1));
#endif
			iowrite32(csr6_mask_defstate, ioaddr + CSR6);
			tp->csr6 = csr6_mask_hdcap;
			dev->if_port = 11;
			iowrite32(0x0000, ioaddr + CSR13);
			iowrite32(0x0000, ioaddr + CSR14);
		} else
			t21142_start_nway(dev);
	} else if (tp->chip_id == PNIC2) {
	        /* for initial startup advertise 10/100 Full and Half */
	        tp->sym_advertise = 0x01E0;
                /* enable autonegotiate end interrupt */
	        iowrite32(ioread32(ioaddr+CSR5)| 0x00008010, ioaddr + CSR5);
	        iowrite32(ioread32(ioaddr+CSR7)| 0x00008010, ioaddr + CSR7);
		pnic2_start_nway(dev);
	} else if (tp->chip_id == LC82C168  &&  ! tp->medialock) {
		if (tp->mii_cnt) {
			dev->if_port = 11;
			tp->csr6 = 0x814C0000 | (tp->full_duplex ? 0x0200 : 0);
			iowrite32(0x0001, ioaddr + CSR15);
		} else if (ioread32(ioaddr + CSR5) & TPLnkPass)
			pnic_do_nway(dev);
		else {
			/* Start with 10mbps to do autonegotiation. */
			iowrite32(0x32, ioaddr + CSR12);
			tp->csr6 = 0x00420000;
			iowrite32(0x0001B078, ioaddr + 0xB8);
			iowrite32(0x0201B078, ioaddr + 0xB8);
			next_tick = 1*HZ;
		}
	} else if ((tp->chip_id == MX98713 || tp->chip_id == COMPEX9881)
			   && ! tp->medialock) {
		dev->if_port = 0;
		tp->csr6 = 0x01880000 | (tp->full_duplex ? 0x0200 : 0);
		iowrite32(0x0f370000 | ioread16(ioaddr + 0x80), ioaddr + 0x80);
	} else if (tp->chip_id == MX98715 || tp->chip_id == MX98725) {
		/* Provided by BOLO, Macronix - 12/10/1998. */
		dev->if_port = 0;
		tp->csr6 = 0x01a80200;
		iowrite32(0x0f370000 | ioread16(ioaddr + 0x80), ioaddr + 0x80);
		iowrite32(0x11000 | ioread16(ioaddr + 0xa0), ioaddr + 0xa0);
	} else if (tp->chip_id == COMET || tp->chip_id == CONEXANT) {
		/* Enable automatic Tx underrun recovery. */
		iowrite32(ioread32(ioaddr + 0x88) | 1, ioaddr + 0x88);
		dev->if_port = tp->mii_cnt ? 11 : 0;
		tp->csr6 = 0x00040000;
	} else if (tp->chip_id == AX88140) {
		tp->csr6 = tp->mii_cnt ? 0x00040100 : 0x00000100;
	} else if (tp->chip_id == PILOT2_TULIP) {
        pilot_up(dev);
		tp->csr6 |= 1<<21;
	} else
		tulip_select_media(dev, 1);
	/* Start the chip's Tx to process setup frame. */
	tulip_stop_rxtx(tp);
	barrier();
	udelay(5);
	iowrite32(tp->csr6 | TxOn , ioaddr + CSR6);

	/* Enable interrupts by setting the interrupt mask. */
	iowrite32(tulip_tbl[tp->chip_id].valid_intrs, ioaddr + CSR5);
	iowrite32(tulip_tbl[tp->chip_id].valid_intrs, ioaddr + CSR7);
	tulip_start_rxtx(tp);
	iowrite32(0, ioaddr + CSR2);		/* Rx poll demand */
#ifndef final_version
	if (tulip_debug > 2) {
		printk(KERN_DEBUG "%s: Done tulip_up(), CSR0 %8.8x, CSR5 %8.8x CSR6 %8.8x.\n",
			   dev->name, ioread32(ioaddr + CSR0), ioread32(ioaddr + CSR5),
			   ioread32(ioaddr + CSR6));
	}
#endif

	/* Set the timer to switch to check for link beat and perhaps switch
	   to an alternate media type. */
	tp->timer.expires = RUN_AT(next_tick);
	add_timer(&tp->timer);
#ifdef CONFIG_TULIP_NAPI
	init_timer(&tp->oom_timer);
        tp->oom_timer.data = (unsigned long)dev;
        tp->oom_timer.function = oom_timer;
#endif
}

static int tulip_open(struct net_device *dev)
{
	int retval;

	if ((retval = request_irq(dev->irq, &tulip_interrupt, IRQF_SHARED, dev->name, dev)))
	{
		printk("request_irq failed\n");
		return retval;
	}

	tulip_init_ring (dev);

	tulip_up (dev);

	netif_start_queue (dev);
	return 0;
}


static void tulip_tx_timeout(struct net_device *dev)
{
	struct tulip_private *tp = netdev_priv(dev);
	void __iomem *ioaddr = tp->base_addr;
	unsigned long flags;

	spin_lock_irqsave (&tp->lock, flags);

	if (tulip_media_cap[dev->if_port] & MediaIsMII) {
		/* Do nothing -- the media monitor should handle this. */
#ifndef final_version
		if (tulip_debug > 1)
			printk(KERN_WARNING "%s: Transmit timeout using MII device.\n",
				   dev->name);
#endif
	} else if (tp->chip_id == DC21140 || tp->chip_id == DC21142
			   || tp->chip_id == MX98713 || tp->chip_id == COMPEX9881
			   || tp->chip_id == DM910X) {
		printk(KERN_WARNING "%s: 21140 transmit timed out, status %8.8x, "
			   "SIA %8.8x %8.8x %8.8x %8.8x, resetting...\n",
			   dev->name, ioread32(ioaddr + CSR5), ioread32(ioaddr + CSR12),
			   ioread32(ioaddr + CSR13), ioread32(ioaddr + CSR14), ioread32(ioaddr + CSR15));
		if ( ! tp->medialock  &&  tp->mtable) {
			do
				--tp->cur_index;
			while (tp->cur_index >= 0
						&& (tulip_media_cap[tp->mtable->mleaf[tp->cur_index].media]
								& MediaIsFD));

			if (--tp->cur_index < 0) {
				/* We start again, but should instead look for default. */
				tp->cur_index = tp->mtable->leafcount - 1;
			}
			tulip_select_media(dev, 0);
			printk(KERN_WARNING "%s: transmit timed out, switching to %s "
				   "media.\n", dev->name, medianame[dev->if_port]);
		}
	} else if (tp->chip_id == PNIC2) {
		printk(KERN_WARNING "%s: PNIC2 transmit timed out, status %8.8x, "
		       "CSR6/7 %8.8x / %8.8x CSR12 %8.8x, resetting...\n",
		       dev->name, (int)ioread32(ioaddr + CSR5), (int)ioread32(ioaddr + CSR6),
		       (int)ioread32(ioaddr + CSR7), (int)ioread32(ioaddr + CSR12));
	} else {
		printk(KERN_WARNING "%s: Transmit timed out, status %8.8x, CSR12 "
			   "%8.8x, resetting...\n",
			   dev->name, ioread32(ioaddr + CSR5), ioread32(ioaddr + CSR12));
		dev->if_port = 0;
	}

#if defined(way_too_many_messages)
	if (tulip_debug > 3) {
		int i;
		for (i = 0; i < RX_RING_SIZE; i++) {
			u8 *buf = (u8 *)(tp->rx_ring[i].buffer1);
			int j;
			printk(KERN_DEBUG "%2d: %8.8x %8.8x %8.8x %8.8x  "
				   "%2.2x %2.2x %2.2x.\n",
				   i, (unsigned int)tp->rx_ring[i].status,
				   (unsigned int)tp->rx_ring[i].length,
				   (unsigned int)tp->rx_ring[i].buffer1,
				   (unsigned int)tp->rx_ring[i].buffer2,
				   buf[0], buf[1], buf[2]);
			for (j = 0; buf[j] != 0xee && j < 1600; j++)
				if (j < 100) printk(" %2.2x", buf[j]);
			printk(" j=%d.\n", j);
		}
		printk(KERN_DEBUG "  Rx ring %8.8x: ", (int)tp->rx_ring);
		for (i = 0; i < RX_RING_SIZE; i++)
			printk(" %8.8x", (unsigned int)tp->rx_ring[i].status);
		printk("\n" KERN_DEBUG "  Tx ring %8.8x: ", (int)tp->tx_ring);
		for (i = 0; i < TX_RING_SIZE; i++)
			printk(" %8.8x", (unsigned int)tp->tx_ring[i].status);
		printk("\n");
	}
#endif

	/* Stop and restart the chip's Tx processes . */

	tulip_restart_rxtx(tp);
	/* Trigger an immediate transmit demand. */
	iowrite32(0, ioaddr + CSR1);

	tp->stats.tx_errors++;

	spin_unlock_irqrestore (&tp->lock, flags);
	dev->trans_start = jiffies;
	netif_wake_queue (dev);
}


/* Initialize the Rx and Tx rings, along with various 'dev' bits. */
static void tulip_init_ring(struct net_device *dev)
{
	struct tulip_private *tp = netdev_priv(dev);
	int i;

	tp->susp_rx = 0;
	tp->ttimer = 0;
	tp->nir = 0;

	for (i = 0; i < RX_RING_SIZE; i++) {
		tp->rx_ring[i].status = 0x00000000;
		tp->rx_ring[i].length = cpu_to_le32(PKT_BUF_SZ);
		tp->rx_ring[i].buffer2 = cpu_to_le32(tp->rx_ring_dma + sizeof(struct tulip_rx_desc) * (i + 1));
		tp->rx_buffers[i].skb = NULL;
		tp->rx_buffers[i].mapping = 0;
	}
	/* Mark the last entry as wrapping the ring. */
	tp->rx_ring[i-1].length = cpu_to_le32(PKT_BUF_SZ | DESC_RING_WRAP);
	tp->rx_ring[i-1].buffer2 = cpu_to_le32(tp->rx_ring_dma);

	for (i = 0; i < RX_RING_SIZE; i++) {
		dma_addr_t mapping;

		/* Note the receive buffer must be longword aligned.
		   dev_alloc_skb() provides 16 byte alignment.  But do *not*
		   use skb_reserve() to align the IP header! */
		struct sk_buff *skb = dev_alloc_skb(PKT_BUF_SZ);
		tp->rx_buffers[i].skb = skb;
		if (skb == NULL)
			break;
		mapping = pci_map_single(tp->pdev, skb->data,
					 PKT_BUF_SZ, PCI_DMA_FROMDEVICE);
		tp->rx_buffers[i].mapping = mapping;
		skb->dev = dev;			/* Mark as being used by this device. */
		tp->rx_ring[i].status = cpu_to_le32(DescOwned);	/* Owned by Tulip chip */
		tp->rx_ring[i].buffer1 = cpu_to_le32(mapping);
	}
	tp->dirty_rx = (unsigned int)(i - RX_RING_SIZE);

	/* The Tx buffer descriptor is filled in as needed, but we
	   do need to clear the ownership bit. */
	for (i = 0; i < TX_RING_SIZE; i++) {
		tp->tx_buffers[i].skb = NULL;
		tp->tx_buffers[i].mapping = 0;
		tp->tx_ring[i].status = 0x00000000;
		tp->tx_ring[i].buffer2 = cpu_to_le32(tp->tx_ring_dma + sizeof(struct tulip_tx_desc) * (i + 1));
	}
	tp->tx_ring[i-1].buffer2 = cpu_to_le32(tp->tx_ring_dma);
}


static int tulip_txbuffer_replinish(void *dev_instance)
{
  struct net_device *dev = (struct net_device *)dev_instance;
  struct tulip_private *tp = netdev_priv(dev);
  void __iomem *ioaddr = tp->base_addr;
  int csr5;
  int tx = 0;
  unsigned int dirty_tx;

  csr5 = ioread32(ioaddr + CSR5);

 if (csr5 & (TxNoBuf | TxDied | TxIntr)) 
      {

	iowrite32( (csr5 & (TxNoBuf | TxDied | TxIntr)), ioaddr + CSR5);

	for (dirty_tx = tp->dirty_tx; tp->cur_tx - dirty_tx > 0; dirty_tx++) 
	  {
	    int entry = dirty_tx % TX_RING_SIZE;
	    int status = le32_to_cpu(tp->tx_ring[entry].status);
#ifdef CONFIG_PILOT_SG
		u32 tdes1 = le32_to_cpu(tp->tx_ring[entry].length);
#endif

	    
	    if (status < 0)
	      break;			/* It still has not been Txed */
	    
	    /* Check for Rx filter setup frames. */
	    if (tp->tx_buffers[entry].skb == NULL) 
	      {
		/* test because dummy frames not mapped */
		if (tp->tx_buffers[entry].mapping)
		  pci_unmap_single(tp->pdev,
				   tp->tx_buffers[entry].mapping,
				   sizeof(tp->setup_frame),
				   PCI_DMA_TODEVICE);
	      continue;
	      }
	  
	    /* Changed for Pilot2 to ignore carrier errors in full duplex mode - Amrut! */
	    if (!(status & 0x8000) || ( ( (status & 0x8400)==0x8400) && (tp->full_duplex == 1) )||( ((status & 0x8800)==0x8800) && (tp->full_duplex == 1) )) //DJ022310
	      {
#ifdef CONFIG_PILOT_SG
			tp->stats.tx_bytes += tp->tx_buffers[entry].maplen;
#else
		tp->stats.tx_bytes += tp->tx_buffers[entry].skb->len;
#endif
		tp->stats.collisions += (status >> 3) & 15;
#ifdef CONFIG_PILOT_SG
		if(tdes1 & (1 << 30))
			tp->stats.tx_packets++;
#else
		tp->stats.tx_packets++;
#endif
#ifndef final_version
		if(tulip_debug > 1)
		  {
		    printk(KERN_DEBUG "%s: Successfully sent\n", dev->name);
		  }
#endif
	      } 
	    else 
	      {
		/* There was an major error, log it. */
#ifndef final_version
		if (tulip_debug > 1)
		  printk(KERN_DEBUG "%s: Transmit error, Tx status %8.8x fd = %d.\n",
			 dev->name, status, tp->full_duplex);
#endif
		tp->stats.tx_errors++;
		if (status & 0x4104) tp->stats.tx_aborted_errors++;
		
		
		/* Changed for Pilot2 to ignore carrier errors in full duplex mode - Amrut! */
		if (status & 0x0C00 && tp->full_duplex == 0)
		  {
		    tp->stats.tx_carrier_errors++;
		  } 
		if (status & 0x0200) tp->stats.tx_window_errors++;
		if (status & 0x0002) tp->stats.tx_fifo_errors++;
		if ((status & 0x0080) && tp->full_duplex == 0)
		  tp->stats.tx_heartbeat_errors++;
	      } 
#ifdef CONFIG_PILOT_SG
		pci_unmap_single(tp->pdev, tp->tx_buffers[entry].mapping,tp->tx_buffers[entry].maplen, PCI_DMA_TODEVICE);
#else
	    pci_unmap_single(tp->pdev, tp->tx_buffers[entry].mapping,tp->tx_buffers[entry].skb->len, PCI_DMA_TODEVICE);
#endif
	    
	    /* Free the original skb. */
	    dev_kfree_skb_irq(tp->tx_buffers[entry].skb);
	    tp->tx_buffers[entry].skb = NULL;
	    tp->tx_buffers[entry].mapping = 0;
	    tx++;
	  }
	
#ifndef final_version
	if (tp->cur_tx - dirty_tx > TX_RING_SIZE) 
	  {
	    printk(KERN_ERR "%s: Out-of-sync dirty pointer, %d vs. %d.\n",dev->name, dirty_tx, tp->cur_tx);
	    dirty_tx += TX_RING_SIZE;
	  }
#endif
	
	if (tp->cur_tx - dirty_tx < TX_RING_SIZE - 2)
	  netif_wake_queue(dev);
	
	tp->dirty_tx = dirty_tx;
	if (csr5 & TxDied) 
	  {
#ifndef final_version
	    if (tulip_debug > 2)
	      printk(KERN_WARNING "%s: The transmitter stopped."
		     "  CSR5 is %x, CSR6 %x, new CSR6 %x.\n",
		     dev->name, csr5, ioread32(ioaddr + CSR6), tp->csr6);
#endif
	    tulip_restart_rxtx(tp);
	  }
      }
 return 0;
}


static int tulip_process_rx(void *dev_instance)
{
  struct net_device *dev = (struct net_device *)dev_instance;
  struct tulip_private *tp = netdev_priv(dev);
  void __iomem *ioaddr = tp->base_addr;
  int csr5;
  int rx = 0;
  int rd=0;

  /* Let's see whether the interrupt really is for us */

  csr5 = ioread32(ioaddr + CSR5);

  
  if(csr5 & (RxIntr | RxNoBuf)) 
    {
      iowrite32( (csr5 & (RxIntr | RxNoBuf) ), ioaddr + CSR5);
      
      //      for(rd=0;rd<(RX_RING_SIZE);rd++)
      for(rd=0;rd<(RX_RING_SIZE/SE_RD_DIV);rd++)
	{
	  rx += tulip_rx(dev);
	  tulip_refill_rx(dev);
	  }
    }

  iowrite32(0x01, tp->base_addr + CSR2); 

  return 0;
}

static int tulip_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
  struct tulip_private *tp = netdev_priv(dev);
  int entry;
  u32 flag;
//  dma_addr_t mapping;
  void __iomem *ioaddr = tp->base_addr;

#ifndef CONFIG_PILOT_SG
	spin_lock_irq(&tp->lock);
#endif
  tulip_txbuffer_replinish(dev);  //DJ
  
  /* Calculate the next Tx descriptor entry. */
  entry = tp->cur_tx % TX_RING_SIZE;
  
  tp->tx_buffers[entry].skb = skb;
#if 0
  mapping = pci_map_single(tp->pdev, skb->data,
			   skb->len, PCI_DMA_TODEVICE);
  tp->tx_buffers[entry].mapping = mapping;

  tp->tx_ring[entry].buffer1 = cpu_to_le32(mapping);
#endif

  tp->tx_buffers[entry].mapping = pci_map_single(tp->pdev, skb->data,
                           skb->len, PCI_DMA_TODEVICE);
  tp->tx_ring[entry].buffer1 = cpu_to_le32(tp->tx_buffers[entry].mapping);

  if (tp->cur_tx - tp->dirty_tx < TX_RING_SIZE/2) 
    {/* Typical path */
      flag = 0x60000000; /* No interrupt */
    } 
  else if (tp->cur_tx - tp->dirty_tx == TX_RING_SIZE/2) 
    {
      flag = 0xe0000000; /* Tx-done intr. */
    } 
  else if (tp->cur_tx - tp->dirty_tx < TX_RING_SIZE - 2) 
    {
      flag = 0x60000000; /* No Tx-done intr. */
    } 
  else 
    {		/* Leave room for set_rx_mode() to fill entries. */
      flag = 0xe0000000; /* Tx-done intr. */
      //enable tx interrupt,when the ring is full
      tp->p_tx_coalising_en=1;
      iowrite32( (ioread32(ioaddr + CSR7)|TxIntr),(ioaddr + CSR7) ); 
      netif_stop_queue(dev);
    }
  if (entry == TX_RING_SIZE-1)
    flag = 0xe0000000 | DESC_RING_WRAP;
  
  tp->tx_ring[entry].length = cpu_to_le32(skb->len | flag);

  /* if we were using Transmit Automatic Polling, we would need a
   * wmb() here. */
  tp->tx_ring[entry].status = cpu_to_le32(DescOwned);
  //wmb();
  
  tp->cur_tx++;
  
  /* Trigger an immediate transmit demand. */
  iowrite32(0, tp->base_addr + CSR1);

#ifndef CONFIG_TULIP_NAPI
	tulip_process_rx(dev);
#endif  

#ifndef CONFIG_PILOT_SG
  spin_unlock_irq(&tp->lock);
#endif
  dev->trans_start = jiffies;
  
 
  return 0;
}

#ifdef CONFIG_PILOT_SG
static int tulip_handle_fast_sg(struct sk_buff *skb, struct net_device *dev)
{
        struct tulip_private *tp = netdev_priv(dev);
        unsigned int nr_frags;
        unsigned int f, len, offset = 0;
        int entry, last_entry, first_entry;
        tulip_txbuffer_replinish(dev);
        len = skb_headlen(skb);
        first_entry = last_entry = entry = tp->cur_tx & (TX_RING_SIZE - 1);
        tp->tx_buffers[entry].skb = skb;

        tp->tx_buffers[entry].mapping = pci_map_single(tp->pdev, skb->data,
                               len, PCI_DMA_TODEVICE);;
        tp->tx_ring[entry].buffer1 = cpu_to_le32(tp->tx_buffers[entry].mapping);
        tp->tx_ring[entry].length = ((1 << 24) | (1 << 29) | len);//Chain
	tp->tx_buffers[entry].maplen = len;
        nr_frags = skb_shinfo(skb)->nr_frags;
        tp->cur_tx++;
        for(f = 0; f < nr_frags;f++) {
                const struct skb_frag_struct *frag;
                int frag_len = 0;
                frag = &skb_shinfo(skb)->frags[f];
                frag_len = skb_frag_size(frag);
                (void)skb_get(skb);
                entry = tp->cur_tx & (TX_RING_SIZE - 1);
                tp->tx_buffers[entry].skb = skb;
                tp->tx_buffers[entry].mapping = pci_map_single(tp->pdev, skb_frag_address(frag),
                                       frag_len, PCI_DMA_TODEVICE);
		tp->tx_buffers[entry].maplen = frag_len;
                tp->tx_ring[entry].buffer1 = cpu_to_le32(tp->tx_buffers[entry].mapping);
                offset = (u8*)(tp->tx_ring + entry) - (u8 *)tp->tx_ring;
                tp->tx_ring[last_entry].buffer2 = tp->tx_ring_dma + offset;
                tp->tx_ring[entry].length = ((1 << 24) | frag_len);//Chain
                tp->tx_ring[entry].status = cpu_to_le32(DescOwned);
                last_entry = entry;
                tp->cur_tx++;
        }
        tp->tx_ring[entry].length &= ~(1 << 24);
        if(entry == TX_RING_SIZE-1) {
                tp->tx_ring[entry].length |= (DESC_RING_WRAP | (1 << 30)) ;
        } else {
                tp->tx_ring[entry].length |= (1 << 30);
        }
        tp->tx_ring[first_entry].status = cpu_to_le32(DescOwned);
        iowrite32(0, tp->base_addr + CSR1);
#ifndef CONFIG_TULIP_NAPI  //Ashok 
	tulip_process_rx(dev);  //DJ
#endif  
        dev->trans_start = jiffies;
//      printk("2:tp->cur_tx  %d tp->dirty_tx %d nr_frags %d\n", tp->cur_tx, tp->dirty_tx, nr_frags);
        spin_unlock_irq(&tp->lock);
        return NETDEV_TX_OK;
}

static int tulip_sg_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
        unsigned int nr_frags;
        unsigned int f, len, offset = 0;
        int tot_len;
        struct tulip_private *tp = netdev_priv(dev);
        //  dma_addr_t mapping;
        void __iomem *ioaddr = tp->base_addr;
        int entry, last_entry, first_entry, enable_intr = 0;
#ifdef MEASURE_TIME
        unsigned long time1, time2, diff, time3;
#endif

        spin_lock_irq(&tp->lock);
        if(!skb_is_nonlinear(skb)) {
                tot_len = tulip_start_xmit(skb, dev);
                spin_unlock_irq(&tp->lock);
                return tot_len;
        }
#if 0
        if(skb_shinfo(skb)->nr_frags > 2) {
                printk("skblen %d nr_frags %d\n",
                        skb->len, skb_shinfo(skb)->nr_frags);
        }
#endif
        if ((tp->cur_tx - tp->dirty_tx) < (TX_RING_SIZE >> 2))  {
                return tulip_handle_fast_sg(skb, dev);
        }
        tulip_txbuffer_replinish(dev);  //DJ

        len = skb_headlen(skb);
        first_entry = last_entry = entry = tp->cur_tx & (TX_RING_SIZE - 1);
        tp->tx_buffers[first_entry].skb = skb;
        tp->tx_buffers[first_entry].mapping = pci_map_single(tp->pdev, skb->data,
                               len, PCI_DMA_TODEVICE);
        tp->tx_ring[first_entry].buffer1 = cpu_to_le32(tp->tx_buffers[first_entry].mapping);
	tp->tx_buffers[entry].maplen = len;
        if((tp->cur_tx - tp->dirty_tx) > (TX_RING_SIZE - 10)) {
                tp->tx_ring[last_entry].length = ((1 << 24) | len | (1 << 29) | (1 << 31));//Chain first desc, no EOR TX done intr
                tp->p_tx_coalising_en=1;
                iowrite32( (ioread32(ioaddr + CSR7)|TxIntr),(ioaddr + CSR7) );
                netif_stop_queue(dev);
                enable_intr = 1;
        } else {
                tp->tx_ring[last_entry].length = ((1 << 24) | len | (1 << 29));//Chain first desc, no EOR
        }
        tp->cur_tx++;
        nr_frags = skb_shinfo(skb)->nr_frags;

        for(f = 0; f < nr_frags;f++) {
                const struct skb_frag_struct *frag;
                int frag_len = 0;
                frag = &skb_shinfo(skb)->frags[f];
                frag_len = skb_frag_size(frag);
                (void)skb_get(skb);
                entry = tp->cur_tx & (TX_RING_SIZE - 1);
                tp->tx_buffers[entry].skb = skb;
                tp->tx_buffers[entry].mapping = pci_map_single(tp->pdev, skb_frag_address(frag),
                                       frag_len, PCI_DMA_TODEVICE);
		tp->tx_buffers[entry].maplen = frag_len;
                tp->tx_ring[entry].buffer1 = cpu_to_le32(tp->tx_buffers[entry].mapping);
                offset = (u8*)(tp->tx_ring + entry) - (u8 *)tp->tx_ring;
                tp->tx_ring[last_entry].buffer2 = tp->tx_ring_dma + offset;
                tp->tx_ring[entry].length = ((1 << 24) | frag_len);//Chain no EOR
                tp->tx_ring[entry].status = cpu_to_le32(DescOwned);
                last_entry = entry;
                tp->cur_tx++;
        }
        if(enable_intr)
                tp->tx_ring[entry].length |= (1 << 31);
        if(entry == TX_RING_SIZE-1) {
                tp->tx_ring[entry].length |= (DESC_RING_WRAP | (1 << 30));//last
                tp->tx_ring[entry].length &= ~(1 << 24);
        } else {
                tp->tx_ring[entry].length &= ~(1 << 24);
                tp->tx_ring[entry].length |= (1 << 30);//last desc
        }
        tp->tx_ring[first_entry].status = cpu_to_le32(DescOwned);
        iowrite32(0, tp->base_addr + CSR1);
#ifndef CONFIG_TULIP_NAPI  //Ashok 
	tulip_process_rx(dev);  //DJ
#endif  
        dev->trans_start = jiffies;
//      printk("2:tp->cur_tx  %d tp->dirty_tx %d nr_frags %d\n", tp->cur_tx, tp->dirty_tx, nr_frags);
        spin_unlock_irq(&tp->lock);
        return NETDEV_TX_OK;
}
#endif

static void tulip_clean_tx_ring(struct tulip_private *tp)
{
  unsigned int dirty_tx;
  
  for (dirty_tx = tp->dirty_tx ; tp->cur_tx - dirty_tx > 0;
       dirty_tx++) {
    int entry = dirty_tx % TX_RING_SIZE;
    int status = le32_to_cpu(tp->tx_ring[entry].status);
    
    if (status < 0) {
      tp->stats.tx_errors++;	/* It wasn't Txed */
      tp->tx_ring[entry].status = 0;
    }
    
    /* Check for Tx filter setup frames. */
    if (tp->tx_buffers[entry].skb == NULL) {
      /* test because dummy frames not mapped */
      if (tp->tx_buffers[entry].mapping)
	pci_unmap_single(tp->pdev,
			 tp->tx_buffers[entry].mapping,
			 sizeof(tp->setup_frame),
			 PCI_DMA_TODEVICE);
      continue;
    }
    
    pci_unmap_single(tp->pdev, tp->tx_buffers[entry].mapping,
		     tp->tx_buffers[entry].skb->len,
		     PCI_DMA_TODEVICE);
    
    /* Free the original skb. */
    dev_kfree_skb_irq(tp->tx_buffers[entry].skb);
    tp->tx_buffers[entry].skb = NULL;
    tp->tx_buffers[entry].mapping = 0;
  }
}

static void tulip_down (struct net_device *dev)
{
  struct tulip_private *tp = netdev_priv(dev);
  void __iomem *ioaddr = tp->base_addr;
  unsigned long flags;

#ifdef CONFIG_TULIP_NAPI
   napi_disable(&tp->napi);
#endif
  del_timer_sync (&tp->timer);
#ifdef CONFIG_TULIP_NAPI
  del_timer_sync (&tp->oom_timer);
#endif
  spin_lock_irqsave (&tp->lock, flags);
  
  /* Disable interrupts by clearing the interrupt mask. */
  iowrite32 (0x00000000, ioaddr + CSR7);
  
  /* Stop the Tx and Rx processes. */
  tulip_stop_rxtx(tp);
  
  /* prepare receive buffers */
  tulip_refill_rx(dev);
  
  /* release any unconsumed transmit buffers */
  tulip_clean_tx_ring(tp);
  
  if (ioread32 (ioaddr + CSR6) != 0xffffffff)
    tp->stats.rx_missed_errors += ioread32 (ioaddr + CSR8) & 0xffff;
  
  spin_unlock_irqrestore (&tp->lock, flags);
  
  init_timer(&tp->timer);
  tp->timer.data = (unsigned long)dev;
  tp->timer.function = tulip_tbl[tp->chip_id].media_timer;
  
  dev->if_port = tp->saved_if_port;
  
  /* Leave the driver in snooze, not sleep, mode. */
  tulip_set_power_state (tp, 0, 1);
}


static int tulip_close (struct net_device *dev)
{
  struct tulip_private *tp = netdev_priv(dev);
#ifndef final_version
  void __iomem *ioaddr = tp->base_addr;
#endif
  int i;
  
  netif_stop_queue (dev);

	tulip_down (dev);
#ifndef final_version
	if (tulip_debug > 1)
		printk (KERN_DEBUG "%s: Shutting down ethercard, status was %2.2x.\n",
			dev->name, ioread32 (ioaddr + CSR5));
#endif
	free_irq (dev->irq, dev);

	/* Free all the skbuffs in the Rx queue. */
	for (i = 0; i < RX_RING_SIZE; i++) {
		struct sk_buff *skb = tp->rx_buffers[i].skb;
		dma_addr_t mapping = tp->rx_buffers[i].mapping;

		tp->rx_buffers[i].skb = NULL;
		tp->rx_buffers[i].mapping = 0;

		tp->rx_ring[i].status = 0;	/* Not owned by Tulip chip. */
		tp->rx_ring[i].length = 0;
		tp->rx_ring[i].buffer1 = 0xBADF00D0;	/* An invalid address. */
		if (skb) {
			pci_unmap_single(tp->pdev, mapping, PKT_BUF_SZ,
					 PCI_DMA_FROMDEVICE);
			dev_kfree_skb (skb);
		}
	}
	for (i = 0; i < TX_RING_SIZE; i++) {
		struct sk_buff *skb = tp->tx_buffers[i].skb;

		if (skb != NULL) {
			pci_unmap_single(tp->pdev, tp->tx_buffers[i].mapping,
					 skb->len, PCI_DMA_TODEVICE);
			dev_kfree_skb (skb);
		}
		tp->tx_buffers[i].skb = NULL;
		tp->tx_buffers[i].mapping = 0;
	}

	return 0;
}

static struct net_device_stats *tulip_get_stats(struct net_device *dev)
{
	struct tulip_private *tp = netdev_priv(dev);
	void __iomem *ioaddr = tp->base_addr;
	
	if (netif_running(dev)) 
	  {
	    unsigned long flags;
	    
	    spin_lock_irqsave (&tp->lock, flags);
	    
	    tp->stats.rx_missed_errors += ioread32(ioaddr + CSR8) & 0xffff;
	    
	    spin_unlock_irqrestore(&tp->lock, flags);
	  }
	
	return &tp->stats;
}

static int tp_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct tulip_private *tp = netdev_priv(dev);
	unsigned int phy_ctrl = 0;
	unsigned int speed = SPEED_UNKNOWN;
	unsigned int x = SPEED_UNKNOWN;
	const unsigned int phy_idx = 0;
	unsigned int phy = tp->phys[phy_idx] & 0x1f;

	phy_ctrl = tulip_mdio_read (dev, phy, 0);
//	printk("gs addr %x, phyctrl = %x\n", phy, phy_ctrl);

	cmd->supported = (SUPPORTED_1000baseT_Full | SUPPORTED_1000baseT_Half | 
			SUPPORTED_100baseT_Full | SUPPORTED_100baseT_Half | 
			SUPPORTED_10baseT_Full | SUPPORTED_10baseT_Half | 
			SUPPORTED_Autoneg | SUPPORTED_TP | SUPPORTED_Pause | SUPPORTED_Asym_Pause); 

	cmd->advertising = (ADVERTISED_1000baseT_Full | ADVERTISED_1000baseT_Half | 
			ADVERTISED_100baseT_Full | ADVERTISED_100baseT_Half | 
			ADVERTISED_10baseT_Full | ADVERTISED_10baseT_Half |
			ADVERTISED_Autoneg | ADVERTISED_TP | ADVERTISED_Pause | ADVERTISED_Asym_Pause);

//	cmd->lp_advertising = phydev->lp_advertising;
#if 1
	/* Getting from stored value of tulip_private */
    x = tp->speed_neg-1; 
	cmd->duplex = tp->full_duplex; 
#else 
	/* Reading from phy register */
	x = ((phy_ctrl & (1<<13)) >> 13) |  ((phy_ctrl & (1<<6)) >> 5); 
	cmd->duplex = ((phy_ctrl & (1<<8)) >> 8 ) ? DUPLEX_FULL : DUPLEX_HALF;
#endif	
	if(x==0)
		speed = SPEED_10;
	else if (x==1)
		speed = SPEED_100;
	else if (x==2)
		speed = SPEED_1000;
	else
		speed = SPEED_UNKNOWN;

	ethtool_cmd_speed_set(cmd, speed);

	cmd->port = PORT_TP;
	cmd->phy_address = phy;
	cmd->transceiver = XCVR_EXTERNAL;
	cmd->autoneg = ((phy_ctrl & (1<<12)) >> 12 ) ? AUTONEG_ENABLE : AUTONEG_DISABLE;
//	cmd->eth_tp_mdix_ctrl = phydev->mdix;

	return 0;
}

static int tp_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct tulip_private *tp = netdev_priv(dev);
	unsigned int phy_ctrl = 0;
	unsigned int speed = ethtool_cmd_speed(cmd);
	const unsigned int phy_idx = 0;
	unsigned int phy = tp->phys[phy_idx] & 0x1f;

	if (cmd->phy_address != phy)
		return -EINVAL;

	/* We make sure that we don't pass unsupported values in to the PHY */
//	cmd->advertising &= phydev->supported;

	/* Verify the settings we care about. */
	if (cmd->autoneg != AUTONEG_ENABLE && cmd->autoneg != AUTONEG_DISABLE)
		return -EINVAL;

	if (cmd->autoneg == AUTONEG_ENABLE && cmd->advertising == 0)
		return -EINVAL;

	if (cmd->autoneg == AUTONEG_DISABLE &&
			((speed != SPEED_1000 &&
			  speed != SPEED_100 &&
			  speed != SPEED_10) ||
			 (cmd->duplex != DUPLEX_HALF &&
			  cmd->duplex != DUPLEX_FULL)))
		return -EINVAL;

	phy_ctrl = tulip_mdio_read (dev, phy, 0);
	phy_ctrl &= (~(1<<12));						//clear autoneg bit
	phy_ctrl &= ( (~(1<<13)) &  (~(1<<6)) ); 	//clear speed bits
	phy_ctrl &= (~(1<<8));						//clear duplex bit

	/* set autoneg */
	phy_ctrl |= (cmd->autoneg == AUTONEG_ENABLE) ? (1<<12) : 0; 

	/* clear speed and duplex bits of csr6 */
	tp->csr6 &= ~((0x1 << 16) | (0x1 << 17));
	tp->csr6 &= ~(0x1 << 9);

	/* set speed */
	if(speed==SPEED_1000)
	{
		phy_ctrl |= (1<<6); 
		tp->csr6 |= ((0x1 << 16) | (0x1 << 17));
		tp->speed_neg = SPEED_NEG_1000;
	}
	else if (speed==SPEED_100) 
	{
		phy_ctrl |= (1<<13); 
		tp->csr6 &= ~((0x1 << 16) | (0x1 << 17));
		tp->speed_neg = SPEED_NEG_100;
	}
	else
	{
		phy_ctrl = phy_ctrl; 		//do nothing
		tp->csr6 |= (0x1 << 17);
		tp->speed_neg = SPEED_NEG_10;
	}

	/* set duplex */
	tp->full_duplex = cmd->duplex; 
	phy_ctrl |= (cmd->duplex << 8);
	tp->csr6 |= (cmd->duplex << 9);

//	phydev->advertising = cmd->advertising;
#if 0
	if (AUTONEG_ENABLE == cmd->autoneg)
		phydev->advertising |= ADVERTISED_Autoneg;
	else
		phydev->advertising &= ~ADVERTISED_Autoneg;
#endif

	/* Restart autoneg */
	if(cmd->autoneg == AUTONEG_ENABLE)
		phy_ctrl |= (1 << 9);					

	/* Dont Write to phy in case of NC-SI */
	if(phy != 0xFF)
	{
//		printk("ss phy_ctrl = %x\n", phy_ctrl);
		tulip_mdio_write (dev, phy, 0, phy_ctrl);
	}

//	phydev->mdix = cmd->eth_tp_mdix_ctrl;

	/* Restart the PHY */
//	phy_start_aneg(phydev);

	return 0;
}

static int tulip_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct tulip_private *tp = netdev_priv(dev);
	const unsigned int phy_idx = 0;
	if (tp->phys[phy_idx])
		return tp_get_settings(dev, cmd);
//		return phy_ethtool_gset(tp->phys[phy_idx], cmd);

	return -EINVAL;
}


static int tulip_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct tulip_private *tp = netdev_priv(dev);
	const unsigned int phy_idx = 0;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (tp->phys[phy_idx])
		return tp_set_settings(dev, cmd);
//		return phy_ethtool_sset(dev, cmd);

	return -EINVAL;
}

static void tulip_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	//struct tulip_private *np = netdev_priv(dev);
	strcpy(info->driver, DRV_NAME);
	strcpy(info->version, DRV_VERSION);
//	strcpy(info->bus_info, pci_name(np->pdev));
	strcpy(info->bus_info, DRV_NAME);
	info->regdump_len = 0;
}

static struct ethtool_ops ops = {
	.get_drvinfo = tulip_get_drvinfo,
	.get_settings = tulip_get_settings,
	.set_settings = tulip_set_settings,
	.get_link = ethtool_op_get_link,
};

/* Provide ioctl() calls to examine the MII xcvr state. */
static int private_ioctl (struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct tulip_private *tp = netdev_priv(dev);
	void __iomem *ioaddr = tp->base_addr;
	struct mii_ioctl_data *data = if_mii(rq);
	const unsigned int phy_idx = 0;
	int phy = tp->phys[phy_idx] & 0x1f;
	unsigned int regnum = data->reg_num;

//	printk("regnum = %x\n", regnum);

	switch (cmd) {
	case SIOCGMIIPHY:		/* Get address of MII PHY in use. */
#if 0
		if (tp->mii_cnt)
			data->phy_id = phy;
		else if (tp->flags & HAS_NWAY)
			data->phy_id = 32;
		else if (tp->chip_id == COMET)
			data->phy_id = 1;
		else
			return -ENODEV;

#else
		data->phy_id = phy;
//		printk("phy_id = %x\n", phy);
		return 0;
#endif
	case SIOCGMIIREG:		/* Read MII PHY register. */
		if (0)   {
//		if (data->phy_id == 32 && (tp->flags & HAS_NWAY)) {
			int csr12 = ioread32 (ioaddr + CSR12);
			int csr14 = ioread32 (ioaddr + CSR14);
			switch (regnum) {
			case 0:
                                if (((csr14<<5) & 0x1000) ||
                                        (dev->if_port == 5 && tp->nwayset))
                                        data->val_out = 0x1000;
                                else
                                        data->val_out = (tulip_media_cap[dev->if_port]&MediaIs100 ? 0x2000 : 0)
                                                | (tulip_media_cap[dev->if_port]&MediaIsFD ? 0x0100 : 0);
				break;
			case 1:
                                data->val_out =
					0x1848 +
					((csr12&0x7000) == 0x5000 ? 0x20 : 0) +
					((csr12&0x06) == 6 ? 0 : 4);
                                data->val_out |= 0x6048;
				break;
			case 4:
                                /* Advertised value, bogus 10baseTx-FD value from CSR6. */
                                data->val_out =
					((ioread32(ioaddr + CSR6) >> 3) & 0x0040) +
					((csr14 >> 1) & 0x20) + 1;
                                data->val_out |= ((csr14 >> 9) & 0x03C0);
				break;
			case 5: data->val_out = tp->lpar; break;
			default: data->val_out = 0; break;
			}
		} else {
			data->val_out = tulip_mdio_read (dev, data->phy_id & 0x1f, regnum);
//			printk("Read valout = %x\n", data->val_out);
		}
		return 0;

	case SIOCSMIIREG:		/* Write MII PHY register. */
#if 0
		if (!capable (CAP_NET_ADMIN))
			return -EPERM;
		if (regnum & ~0x1f)
			return -EINVAL;
		if (data->phy_id == phy) {
			u16 value = data->val_in;
			switch (regnum) {
			case 0:	/* Check for autonegotiation on or reset. */
				tp->full_duplex_lock = (value & 0x9000) ? 0 : 1;
				if (tp->full_duplex_lock)
					tp->full_duplex = (value & 0x0100) ? 1 : 0;
				break;
			case 4:
				tp->advertising[phy_idx] =
				tp->mii_advertise = data->val_in;
				break;
			}
		}
		if (data->phy_id == 32 && (tp->flags & HAS_NWAY)) {
			u16 value = data->val_in;
			if (regnum == 0) {
			  if ((value & 0x1200) == 0x1200) {
			    if (tp->chip_id == PNIC2) {
                                   pnic2_start_nway (dev);
                            } else {
				   t21142_start_nway (dev);
                            }
			  }
			} else if (regnum == 4)
				tp->sym_advertise = value;
		} else {
			tulip_mdio_write (dev, data->phy_id & 0x1f, regnum, data->val_in);
		}
		return 0;
#else
		tulip_mdio_write (dev, data->phy_id & 0x1f, regnum, data->val_in);
//		printk("Write valin  = %x\n", data->val_in);
       return 0;
#endif
	default:
		return -EOPNOTSUPP;
	}

	return -EOPNOTSUPP;
}


/* Set or clear the multicast filter for this adaptor.
   Note that we only use exclusion around actually queueing the
   new frame, not around filling tp->setup_frame.  This is non-deterministic
   when re-entered but still correct. */

#undef set_bit_le
#define set_bit_le(i,p) do { ((char *)(p))[(i)/8] |= (1<<((i)%8)); } while(0)

static void build_setup_frame_hash(u16 *setup_frm, struct net_device *dev)
{
	struct tulip_private *tp = netdev_priv(dev);
	u16 hash_table[32];
//	struct dev_mc_list *mclist;   
	struct netdev_hw_addr *ha;
	int i;
	u16 *eaddrs;

	memset(hash_table, 0, sizeof(hash_table));
	set_bit_le(255, hash_table); 			/* Broadcast entry */
	/* This should work on big-endian machines as well. */
	//for (i = 0, mclist = dev->mc_list; mclist && i < dev->mc_count;
	//     i++, mclist = mclist->next) {
	i=0;
	netdev_for_each_mc_addr(ha, dev) {
		//int index = ether_crc_le(ETH_ALEN, mclist->dmi_addr) & 0x1ff;
		int index = ether_crc_le(ETH_ALEN, ha->addr) & 0x1ff;

		set_bit_le(index, hash_table);
		i++;

	}
	for (i = 0; i < 32; i++) {
		*setup_frm++ = hash_table[i];
		*setup_frm++ = hash_table[i];
	}
	setup_frm = &tp->setup_frame[13*6];

	/* Fill the final entry with our physical address. */
	eaddrs = (u16 *)dev->dev_addr;
	*setup_frm++ = eaddrs[0]; *setup_frm++ = eaddrs[0];
	*setup_frm++ = eaddrs[1]; *setup_frm++ = eaddrs[1];
	*setup_frm++ = eaddrs[2]; *setup_frm++ = eaddrs[2];
}

static void build_setup_frame_perfect(u16 *setup_frm, struct net_device *dev)
{
	struct tulip_private *tp = netdev_priv(dev);
//	struct dev_mc_list *mclist;
	struct netdev_hw_addr *ha;
	int i;
	u16 *eaddrs;

	/* We have <= 14 addresses so we can use the wonderful
	   16 address perfect filtering of the Tulip. */
	//for (i = 0, mclist = dev->mc_list; i < dev->mc_count;
//	for (i = 0, mclist = dev->mc_list; i < netdev_mc_count(dev);
//	     i++, mclist = mclist->next) {
	i=0;
	netdev_for_each_mc_addr(ha, dev) {
		eaddrs = (u16 *)ha->addr;
		*setup_frm++ = *eaddrs; *setup_frm++ = *eaddrs++;
		*setup_frm++ = *eaddrs; *setup_frm++ = *eaddrs++;
		*setup_frm++ = *eaddrs; *setup_frm++ = *eaddrs++;
		i++;
	}
	/* Fill the unused entries with the broadcast address. */
	memset(setup_frm, 0xff, (15-i)*12);
	setup_frm = &tp->setup_frame[15*6];

	/* Fill the final entry with our physical address. */
	eaddrs = (u16 *)dev->dev_addr;
	*setup_frm++ = eaddrs[0]; *setup_frm++ = eaddrs[0];
	*setup_frm++ = eaddrs[1]; *setup_frm++ = eaddrs[1];
	*setup_frm++ = eaddrs[2]; *setup_frm++ = eaddrs[2];
}


static void set_rx_mode(struct net_device *dev)
{
	struct tulip_private *tp = netdev_priv(dev);
	void __iomem *ioaddr = tp->base_addr;
	int csr6;

	csr6 = ioread32(ioaddr + CSR6) & ~0x00D5;

	tp->csr6 &= ~0x00D5;
	if (dev->flags & IFF_PROMISC) {			/* Set promiscuous. */
		tp->csr6 |= AcceptAllMulticast | AcceptAllPhys;
		csr6 |= AcceptAllMulticast | AcceptAllPhys;
		/* Unconditionally log net taps. */
		printk(KERN_INFO "%s: Promiscuous mode enabled.\n", dev->name);
	} else if ((netdev_mc_count(dev) > 1000)  ||  (dev->flags & IFF_ALLMULTI)) {
		/* Too many to filter well -- accept all multicasts. */
		tp->csr6 |= AcceptAllMulticast;
		csr6 |= AcceptAllMulticast;
	} else	if (tp->flags & MC_HASH_ONLY) {
		/* Some work-alikes have only a 64-entry hash filter table. */
		/* Should verify correctness on big-endian/__powerpc__ */
//		struct dev_mc_list *mclist;
		struct netdev_hw_addr *ha;
		int i;
		if (netdev_mc_count(dev) > 64) {		/* Arbitrary non-effective limit. */
			tp->csr6 |= AcceptAllMulticast;
			csr6 |= AcceptAllMulticast;
		} else {
			u32 mc_filter[2] = {0, 0};		 /* Multicast hash filter */
			int filterbit;
			//for (i = 0, mclist = dev->mc_list; mclist && i < netdev_mc_count(dev);
			//	 i++, mclist = mclist->next) {
			i=0;
			netdev_for_each_mc_addr(ha, dev) {
				if (tp->flags & COMET_MAC_ADDR)
					filterbit = ether_crc_le(ETH_ALEN, ha->addr);
				else
					filterbit = ether_crc(ETH_ALEN, ha->addr) >> 26;
				filterbit &= 0x3f;
				mc_filter[filterbit >> 5] |= 1 << (filterbit & 31);
#ifndef final_version
				if (tulip_debug > 2) {
					printk(KERN_INFO "%s: Added filter for %2.2x:%2.2x:%2.2x:"
						   "%2.2x:%2.2x:%2.2x  %8.8x bit %d.\n", dev->name,
						   mclist->addr[0], ha->addr[1],
						   mclist->addr[2], ha->addr[3],
						   mclist->addr[4], ha->addr[5],
						   ether_crc(ETH_ALEN, ha->addr), filterbit);
				}
#endif
				i++;
			}
			if (mc_filter[0] == tp->mc_filter[0]  &&
				mc_filter[1] == tp->mc_filter[1])
				;				/* No change. */
			else if (tp->flags & IS_ASIX) {
				iowrite32(2, ioaddr + CSR13);
				iowrite32(mc_filter[0], ioaddr + CSR14);
				iowrite32(3, ioaddr + CSR13);
				iowrite32(mc_filter[1], ioaddr + CSR14);
			} else if (tp->flags & COMET_MAC_ADDR) {
				iowrite32(mc_filter[0], ioaddr + 0xAC);
				iowrite32(mc_filter[1], ioaddr + 0xB0);
			}
			tp->mc_filter[0] = mc_filter[0];
			tp->mc_filter[1] = mc_filter[1];
		}
	} else {
		unsigned long flags;
		u32 tx_flags = 0x08000000 | 192;

		/* Note that only the low-address shortword of setup_frame is valid!
		   The values are doubled for big-endian architectures. */
		if (netdev_mc_count(dev) > 14) { /* Must use a multicast hash table. */
			build_setup_frame_hash(tp->setup_frame, dev);
			tx_flags = 0x08400000 | 192;
		} else {
//		        printk("setting perfect hw filtering mode\n");
			build_setup_frame_perfect(tp->setup_frame, dev);
		}

		spin_lock_irqsave(&tp->lock, flags);

		if (tp->cur_tx - tp->dirty_tx > TX_RING_SIZE - 2) {
			/* Same setup recently queued, we need not add it. */
		} else {
			unsigned int entry;
			int dummy = -1;

			/* Now add this frame to the Tx list. */

			entry = tp->cur_tx++ % TX_RING_SIZE;

			if (entry != 0) {
				/* Avoid a chip errata by prefixing a dummy entry. */
				tp->tx_buffers[entry].skb = NULL;
				tp->tx_buffers[entry].mapping = 0;
				tp->tx_ring[entry].length =
					(entry == TX_RING_SIZE-1) ? cpu_to_le32(DESC_RING_WRAP) : 0;
				tp->tx_ring[entry].buffer1 = 0;
				/* Must set DescOwned later to avoid race with chip */
				dummy = entry;
				entry = tp->cur_tx++ % TX_RING_SIZE;

			}

			tp->tx_buffers[entry].skb = NULL;
			tp->tx_buffers[entry].mapping =
				pci_map_single(tp->pdev, tp->setup_frame,
					       sizeof(tp->setup_frame),
					       PCI_DMA_TODEVICE);
			/* Put the setup frame on the Tx list. */
			if (entry == TX_RING_SIZE-1)
				tx_flags |= DESC_RING_WRAP;		/* Wrap ring. */
			tp->tx_ring[entry].length = cpu_to_le32(tx_flags);
			tp->tx_ring[entry].buffer1 =
				cpu_to_le32(tp->tx_buffers[entry].mapping);
			tp->tx_ring[entry].status = cpu_to_le32(DescOwned);
			if (dummy >= 0)
				tp->tx_ring[dummy].status = cpu_to_le32(DescOwned);
			if (tp->cur_tx - tp->dirty_tx >= TX_RING_SIZE - 2)
				netif_stop_queue(dev);

			/* Trigger an immediate transmit demand. */
			iowrite32(0, ioaddr + CSR1);
		}

		spin_unlock_irqrestore(&tp->lock, flags);
	}

	iowrite32(csr6, ioaddr + CSR6);
}

#ifdef CONFIG_TULIP_MWI
static void tulip_mwi_config (struct pci_dev *pdev,
					struct net_device *dev)
{
	struct tulip_private *tp = netdev_priv(dev);
	u8 cache;
	u16 pci_command;
	u32 csr0;

	if (tulip_debug > 3)
		printk(KERN_DEBUG "%s: tulip_mwi_config()\n", pci_name(pdev));

	tp->csr0 = csr0 = 0;

	/* if we have any cache line size at all, we can do MRM */
	csr0 |= MRM;

	/* ...and barring hardware bugs, MWI */
	if (!(tp->chip_id == DC21143 && tp->revision == 65))
		csr0 |= MWI;

	/* set or disable MWI in the standard PCI command bit.
	 * Check for the case where  mwi is desired but not available
	 */
	if (csr0 & MWI)	pci_set_mwi(pdev);
	else		pci_clear_mwi(pdev);

	/* read result from hardware (in case bit refused to enable) */
	pci_read_config_word(pdev, PCI_COMMAND, &pci_command);
	if ((csr0 & MWI) && (!(pci_command & PCI_COMMAND_INVALIDATE)))
		csr0 &= ~MWI;

	/* if cache line size hardwired to zero, no MWI */
	pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &cache);
	if ((csr0 & MWI) && (cache == 0)) {
		csr0 &= ~MWI;
		pci_clear_mwi(pdev);
	}

	/* assign per-cacheline-size cache alignment and
	 * burst length values
	 */
	switch (cache) {
	case 8:
		csr0 |= MRL | (1 << CALShift) | (16 << BurstLenShift);
		break;
	case 16:
		csr0 |= MRL | (2 << CALShift) | (16 << BurstLenShift);
		break;
	case 32:
		csr0 |= MRL | (3 << CALShift) | (32 << BurstLenShift);
		break;
	default:
		cache = 0;
		break;
	}

	/* if we have a good cache line size, we by now have a good
	 * csr0, so save it and exit
	 */
	if (cache)
		goto out;

	/* we don't have a good csr0 or cache line size, disable MWI */
	if (csr0 & MWI) {
		pci_clear_mwi(pdev);
		csr0 &= ~MWI;
	}

	/* sane defaults for burst length and cache alignment
	 * originally from de4x5 driver
	 */
	csr0 |= (8 << BurstLenShift) | (1 << CALShift);

out:
	tp->csr0 = csr0;
	if (tulip_debug > 2)
		printk(KERN_DEBUG "%s: MWI config cacheline=%d, csr0=%08x\n",
		       pci_name(pdev), cache, csr0);
}
#endif

/*
 *	Chips that have the MRM/reserved bit quirk and the burst quirk. That
 *	is the DM910X and the on chip ULi devices
 */
 
//static int tulip_uli_dm_quirk(struct pci_dev *pdev)
//{
//	if (pdev->vendor == 0x1282 && pdev->device == 0x9102)
//		return 1;
//	return 0;
//}

static const struct net_device_ops tulip_netdev_ops = {
        .ndo_open               = tulip_open,
#ifdef CONFIG_PILOT_SG
        .ndo_start_xmit         = tulip_sg_start_xmit,
#else
        .ndo_start_xmit         = tulip_start_xmit,
#endif
        .ndo_tx_timeout         = tulip_tx_timeout,
        .ndo_stop               = tulip_close,
        .ndo_get_stats          = tulip_get_stats,
        .ndo_do_ioctl           = private_ioctl,
        .ndo_set_rx_mode        = set_rx_mode,
        .ndo_change_mtu         = eth_change_mtu,
        .ndo_set_mac_address    = eth_mac_addr,
        .ndo_validate_addr      = eth_validate_addr,
#ifdef CONFIG_NET_POLL_CONTROLLER
        .ndo_poll_controller     = poll_tulip,
#endif
};


//#define SE_PILOT2_READ_MAC_ADDR_SPI
#ifdef SE_PILOT2_READ_MAC_ADDR_SPI
#define SPI_ADDR_TO_MAC_BASE_0 0x7C0000
#define SPI_ADDR_TO_MAC_BASE_1 0x780000
#define MAC_ADDR_SIZE 6

static void read_spi_mac_addr(unsigned char * mac_addr, unsigned int interface){
  unsigned char * spi_addr = NULL;
  //Map the memory 0x7fff00 - SPI memory map, map 6 bytes
  if(interface == 0){
    spi_addr = (unsigned char *) ioremap_nocache(SPI_ADDR_TO_MAC_BASE_0, MAC_ADDR_SIZE);
  }else{
    spi_addr = (unsigned char *) ioremap_nocache(SPI_ADDR_TO_MAC_BASE_1, MAC_ADDR_SIZE);
  }
  if(spi_addr == NULL){
    //put our hardcoded mac address which is 00:0f:1f:a8:15:78
  }else if((spi_addr[5] & (spi_addr[4] & (spi_addr[3] & (spi_addr[2] & (spi_addr[0] & spi_addr[1]))))) == 0xff){ //All are FFs
    //put our hardcoded mac address which is 00:0f:1f:a8:15:78
    mac_addr[0] = 0x0;
    mac_addr[1] = 0xf;
    mac_addr[2] = 0x1f;
    mac_addr[3] = 0xa8;
    mac_addr[4] = 0x15;
    mac_addr[5] = 0x78;
  }else if((spi_addr[5] | (spi_addr[4] | (spi_addr[3] | (spi_addr[2] | (spi_addr[0] | spi_addr[1]))))) == 0x00){ //All are 0s
    //put our hardcoded mac address which is 00:0f:1f:a8:15:78
    mac_addr[0] = 0x0;
    mac_addr[1] = 0xf;
    mac_addr[2] = 0x1f;
    mac_addr[3] = 0xa8;
    mac_addr[4] = 0x15;
    mac_addr[5] = 0x78;
  }else{
    mac_addr[0] = spi_addr[0];
    mac_addr[1] = spi_addr[1];
    mac_addr[2] = spi_addr[2];
    mac_addr[3] = spi_addr[3];
    mac_addr[4] = spi_addr[4];
    mac_addr[5] = spi_addr[5];
  }
  iounmap(spi_addr);
}
#endif

#if 1
/*------------PHY SWIZZLE BREAKER----------------------*
NOTE NOTE: Added by shivah to solve the PUZZLE of different PHY's (SMSC, REALTEK, BroadCom) connected to MAC-0 and / or MAC-1
Here is how we do it?
	1. MAC-0 is the interface that gets enumerated first by SW design we read PHY_ID giving PHY_ADDR as 1 through MAC-0
	   if we get a valid id we proceed further using that PHY, if we dont get a valid PHY-ID then nothing is connected from PHY_ADDR=1
	   So we read PHY_ID giving PHY_ADDR as 2 through MAC-0, if we get a valid id we proceed further using that PHY, if we dont get a valid 
	   PHY-ID then nothing is connected from PHY_ADDR=2 or PHY_ADDR=1 so the conclusion, MAC-0 not used.
	2. MAC-1 is the interface that gets enumerated second by SW design we read PHY_ID giving PHY_ADDR as 1 through MAC-1
           if we get a valid id we proceed further using that PHY, if we dont get a valid PHY-ID then nothing is connected from PHY_ADDR=1
           So we read PHY_ID giving PHY_ADDR as 2 through MAC-1, if we get a valid id we proceed further using that PHY, if we dont get a valid
           PHY-ID then nothing is connected from PHY_ADDR=2 or PHY_ADDR=1 so the conclusion, MAC-1 not used.
	3. If we dont get valid ID's from steps 1 &2 no PHY's are connected.
*/
#define MANUF_MODEL_NUMBER_POS		4
#define MANUF_MODEL_NUMBER_BITS		0x3F
#define MANUF_MODEL_NUMBER		(MANUF_MODEL_NUMBER_BITS << MANUF_MODEL_NUMBER_POS)

#ifdef CONFIG_MACH_PILOT4_ASIC
	#define PHY_ADDR0	2
#else
	#define PHY_ADDR0	3 
#endif

static int pilot_tulip_find_phy(struct net_device *dev)
{
  struct tulip_private *tp = netdev_priv(dev);
  int phyn, phy_idx = 0;
  int start_no;
  int data=0;
  volatile int timeout = 1000000;

  /*The lower number PHY is always connected to MAC0 and the other to MAC1 on the board.*/
  if((u32)tp->base_addr == (u32)IOADDR_INTERFACE_ETH_A){
    start_no = PHY_ADDR0; 
  //  printk("Reading PHY Mnfc Id for MAC0 \n");
  } else if((u32)tp->base_addr == (u32)IOADDR_INTERFACE_ETH_B){
    start_no = 1;
 //   printk("Reading PHY Mnfc Id for MAC1 \n");
  }else{
    printk("ERROR WRONG MAC-BASE\n");
    return -1;
  }

  tp->phys[0] = 0xff;  //used to detect there is no phy connected(NC-SI case)
  
    for (phyn = start_no; ( (phyn <= PHY_ADDR0) && (phy_idx < sizeof (tp->phys)) ); phyn++) {
    int phy = phyn & 0x1f;
    int mii_status = tulip_mdio_read (dev, phy, 3);
//    printk("addr %x,reg %x mii_rdata %x\n",phy,3,mii_status); 
    
    switch((mii_status & MANUF_MODEL_NUMBER)>> MANUF_MODEL_NUMBER_POS){
    case PHY_SMSC:
      printk("SMSC PHY found at phy address %d!!\n", phy);
      tp->phy_manuf_model_num = PHY_SMSC; 
      tp->phys[0] = phy;

      //Issue a soft reset and wait for the PHY to come out of the soft reset.
      data= 0x8000;
      tulip_mdio_write(dev, phy, 0x0, data);
	  while((data&0x8000))
	  {
		  data=tulip_mdio_read(dev, phy, 0x0);
		  timeout--;
		  if(timeout <= 0) 
		  {
			  printk("%s %d Condition NOT met even after wait!!! Breaking while loop!!!\n", __FUNCTION__, __LINE__);
#ifdef CONFIG_PANIC_AND_BAIL_OUT
			  panic("");
#endif
			  break;
		  }
		  udelay(1);
	  }

      //Enable Auto Negotiation
      data = 0x1200;
      tulip_mdio_write(dev, phy, 0x0, data);


      return 0;
      break;

    case PHY_SMSC_REVB:
      printk("SMSC (REVB and above) PHY %x Detected at phy address %d!!\n",mii_status, phy);
      tp->phy_manuf_model_num = PHY_SMSC_REVB; 
      tp->phys[0] = phy;

      //Issue a soft reset and wait for the PHY to come out of the soft reset.
      data= 0x8000;
      tulip_mdio_write(dev, phy, 0x0, data);
	  while((data&0x8000))
	  {
		  data=tulip_mdio_read(dev, phy, 0x0);
		  timeout--;
		  if(timeout <= 0) 
		  {
			  printk("%s %d Condition NOT met even after wait!!! Breaking while loop!!!\n", __FUNCTION__, __LINE__);
#ifdef CONFIG_PANIC_AND_BAIL_OUT
			  panic("");
#endif
			  break;
		  }
		  udelay(1);
	  }

      //Enable Auto Negotiation
      data = 0x1200;
      tulip_mdio_write(dev, phy, 0x0, data);
      
      return 0;
      break;
      
      
    case PHY_REALTEK:
      printk("REALTEK: PHY found at phy address %d!!\n", phy);
      tp->phy_manuf_model_num = PHY_REALTEK; 
      tp->phys[0] = phy;
      
      data = 0x5;
      tulip_mdio_write(dev, phy, 0x1F, data);
      data = 0x0;
      tulip_mdio_write(dev, phy, 0xc, data);
      data=tulip_mdio_read(dev, phy, 1);
      data |= 0x80;
      tulip_mdio_write(dev, phy, 0x1, data);
      data = 0x0;
      tulip_mdio_write(dev, phy, 0x1F, data);
      return 0;
      break;
    case PHY_MICREL:
      printk("MICREL: PHY found at phy address %d!!\n", phy);
      tp->phy_manuf_model_num = PHY_MICREL; 
      tp->phys[0] = phy;

#if 0 
      //Micrel Tx clk skew programming
      data=0x104;
      tulip_mdio_write(dev, phy, 0xb, data);
      data=tulip_mdio_read(dev, phy, 0xd);
      printk("RGMII CLK and Control Pad Skew Default values %x\n",data);

      if( (data&0x00f0)!=0x00f0)
	{
	  data=( (data)&(0xff0f) );
	  data=( (data)|(0xf<<4));
	  
	  tulip_mdio_write(dev, phy, 0xc, data);
	  data=0x8104;
	  tulip_mdio_write(dev, phy, 0xb, data);
	  
	  //read back and make sure that our bits are programmed correctly
	  data=0x104;
	  tulip_mdio_write(dev, phy, 0xb, data);
	  data=tulip_mdio_read(dev, phy, 0xd);
	  printk("RGMII CLK and Control Pad Skew Modified  values %x\n",data);
	}

      //enable micrel clk jitter work around
      data=tulip_mdio_read(dev, phy, 0x9);
      if((data&0x1800)!=0x1800)
	{
#endif	  
	  /* Issue soft reset and wait for the PHY to come out of the soft reset */
	  data = 0x8000;
      tulip_mdio_write(dev, phy, 0x0, data);
	  while( (data & 0x8000) == 0x8000)
	  {
		  data=tulip_mdio_read(dev, phy, 0x0);
		  timeout--;
		  if(timeout <= 0) 
		  {
			  printk("%s %d Condition NOT met even after wait!!! Breaking while loop!!!\n", __FUNCTION__, __LINE__);
#ifdef CONFIG_PANIC_AND_BAIL_OUT
			  panic("");
#endif
			  break;
		  }
		  udelay(1);
	  }

	  //Enable all phy interrupts
	  tulip_mdio_write(dev, phy, 0x1B, 0xFF00);

	  //restart auto negotiation
	  data=tulip_mdio_read(dev, phy, 0x0);
	  data |= (0x1<<9) | (0x1<<12);
	  tulip_mdio_write(dev, phy, 0x0, data);
	  udelay(10);
//	}
      return 0;
      break;

    case PHY_MARVELL:
      printk("MARVELL: PHY found at phy address %d!!\n", phy);
      tp->phy_manuf_model_num = PHY_MARVELL;
      tp->phys[0] = phy;

     //Issue a soft reset and wait for the PHY to come out of the soft reset.
      data= 0x8000;
      tulip_mdio_write(dev, phy, 0x0, data);
	  while((data&0x8000))
	  {
		  data=tulip_mdio_read(dev, phy, 0x0);
		  timeout--;
		  if(timeout <= 0) 
		  {
			  printk("%s %d Condition NOT met even after wait!!! Breaking while loop!!!\n", __FUNCTION__, __LINE__);
#ifdef CONFIG_PANIC_AND_BAIL_OUT
			  panic("");
#endif
			  break;
		  }
		  udelay(1);
	  }

	  //Enable all phy interrupts
	  tulip_mdio_write(dev, phy, 0x18, 0xFFFF);

      //Enable Auto Negotiation
	  data=tulip_mdio_read(dev, phy, 0x0);
	  data |= ((1<<9) | (1<<12) );
      tulip_mdio_write(dev, phy, 0x0, data);
	  udelay(10);

	  /* Programming the LEDs of Marvell phy */
      tulip_mdio_write(dev, phy, 0x16, 3);	//select page 3
      tulip_mdio_write(dev, phy, 0x10, 0x1064);
      tulip_mdio_write(dev, phy, 0x12, 0x4A05);
      tulip_mdio_write(dev, phy, 0x16, 0);	//bring back to page 0

      return 0;
      break;

    case PHY_BROADCOM:
      printk("BROADCOM: PHY found at phy address %d!!\n", phy);
      tp->phy_manuf_model_num = PHY_BROADCOM; 
      tp->phys[0] = phy;
      return 0;
      break;
    default:
      printk(" ERROR ERROR Unknown PHY Manufacturer model number is %x\n", mii_status);
      break;
    }
  }
  return 0;
  
}

#endif

static int tulip_init_one (struct platform_device *pdev)
{
	struct tulip_private *tp;
	const void *mac_address;
	static int multiport_cnt;	/* For four-port boards w/one EEPROM */
	u8 chip_rev = 0;
	int i, irq;
	unsigned short sum;
	unsigned char *ee_data;
	struct net_device *dev;
	void __iomem *ioaddr;
	static int board_idx = -1;
	int chip_idx = PILOT2_TULIP;
	const char *chip_name = tulip_tbl[chip_idx].chip_name;
	unsigned int eeprom_missing = 0;
	//unsigned int force_csr0 = 0;
	struct resource *iomem;
	struct device_node *np = pdev->dev.of_node;

#ifndef MODULE
	static int did_version;		/* Already printed version info. */
	if (tulip_debug > 0  &&  did_version++ == 0)
		printk (KERN_INFO "%s", version);
#endif
//	printk("name is %s\n", chip_name);
	board_idx++;

	/* alloc_etherdev ensures aligned and zeroed private structures */
	dev = alloc_etherdev (sizeof (*tp));
	if (!dev) {
	  printk (KERN_ERR PFX "ether device alloc failed, aborting\n");
	  return -ENOMEM;
	}
#if (LINUX_VERSION_CODE <  KERNEL_VERSION(2,6,26))	
	SET_MODULE_OWNER(dev);
#endif

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ioaddr  = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(ioaddr)) {
	  	goto err_out_free_netdev;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get IRQ number\n");
		return irq;
	}

	/*
	 * initialize private data structure 'tp'
	 * it is zeroed and aligned in alloc_etherdev
	 */
	tp = netdev_priv(dev);
	
	tp->rx_ring = dma_alloc_coherent(NULL,
					 sizeof(struct tulip_rx_desc) * RX_RING_SIZE +
					 sizeof(struct tulip_tx_desc) * TX_RING_SIZE,
					 &tp->rx_ring_dma, GFP_ATOMIC);
	
	if (!tp->rx_ring)
	  goto err_out_mtable;
	tp->tx_ring = (struct tulip_tx_desc *)(tp->rx_ring + RX_RING_SIZE);
	tp->tx_ring_dma = tp->rx_ring_dma + sizeof(struct tulip_rx_desc) * RX_RING_SIZE;
	
	tp->chip_id = chip_idx;
	tp->flags = tulip_tbl[chip_idx].flags;
	tp->pdev = NULL;
	tp->base_addr = ioaddr;
	//	tp->revision = chip_rev;
	tp->csr0 = (0x10<<8);
	spin_lock_init(&tp->lock);
	spin_lock_init(&tp->mii_lock);
	init_timer(&tp->timer);
	tp->timer.data = (unsigned long)dev;
	tp->timer.function = tulip_tbl[tp->chip_id].media_timer;
	
	dev->base_addr = (unsigned long)ioaddr;

	if(of_device_is_compatible(np, "pilot,eth-0"))
		dev_iobaseaddr0 = (unsigned long *)ioaddr;
	if(of_device_is_compatible(np, "pilot,eth-1"))
		dev_iobaseaddr1 = (unsigned long *)ioaddr;
	
	if(pilot_tulip_find_phy(dev)){
	  goto err_out_free_res;
	}

	/* Stop the chip's Tx and Rx processes. */
	tulip_stop_rxtx(tp);
	
	/* Clear the missed-packet counter. */
	ioread32(ioaddr + CSR8);
	
	/* The station address ROM is read byte serially.  The register must
	   be polled, waiting for the value to be read bit serially from the
	   EEPROM.
	*/
	ee_data = tp->eeprom;
	sum = 0;
	{
	  /* A serial EEPROM interface, we read now and sort it out later. */
	  int sa_offset = 0;
	  int ee_addr_size = tulip_read_eeprom(dev, 0xff, 8) & 0x40000 ? 8 : 6;
	  
	  for (i = 0; i < sizeof(tp->eeprom); i+=2) {
	    u16 data = tulip_read_eeprom(dev, i/2, ee_addr_size);
	    ee_data[i] = data & 0xff;
	    ee_data[i + 1] = data >> 8;
	  }
	  
	  /* DEC now has a specification (see Notes) but early board makers
	     just put the address in the first EEPROM locations. */
	  /* This does  memcmp(ee_data, ee_data+16, 8) */
	  for (i = 0; i < 8; i ++)
	    if (ee_data[i] != ee_data[16+i])
	      sa_offset = 20;
	  if (chip_idx == CONEXANT) {
	    /* Check that the tuple type and length is correct. */
	    if (ee_data[0x198] == 0x04  &&  ee_data[0x199] == 6)
	      sa_offset = 0x19A;
	  } else if (ee_data[0] == 0xff  &&  ee_data[1] == 0xff &&
		     ee_data[2] == 0) {
	    sa_offset = 2;		/* Grrr, damn Matrox boards. */
	    multiport_cnt = 4;
	  }
	  
	  
#ifdef CONFIG_GSC
	  /* Check to see if we have a broken srom */
		if (ee_data[0] == 0x61 && ee_data[1] == 0x10) {
		  /* pci_vendor_id and subsystem_id are swapped */
		  ee_data[0] = ee_data[2];
		  ee_data[1] = ee_data[3];
		  ee_data[2] = 0x61;
		  ee_data[3] = 0x10;
		  
		  /* HSC-PCI boards need to be byte-swaped and shifted
		   * up 1 word.  This shift needs to happen at the end
		   * of the MAC first because of the 2 byte overlap.
		   */
		  for (i = 4; i >= 0; i -= 2) {
		    ee_data[17 + i + 3] = ee_data[17 + i];
		    ee_data[16 + i + 5] = ee_data[16 + i];
		  }
		}
#endif
		
		for (i = 0; i < 6; i ++) {
		  dev->dev_addr[i] = ee_data[i + sa_offset];
		  sum += ee_data[i + sa_offset];
		}
	}
	/* Lite-On boards have the address byte-swapped. */
	if ((dev->dev_addr[0] == 0xA0  ||  dev->dev_addr[0] == 0xC0 || dev->dev_addr[0] == 0x02)
	    &&  dev->dev_addr[1] == 0x00)
	  for (i = 0; i < 6; i+=2) {
	    char tmp = dev->dev_addr[i];
	    dev->dev_addr[i] = dev->dev_addr[i+1];
	    dev->dev_addr[i+1] = tmp;
	  }

//Till we fix U-boot to have mac-address for MAC1	
#ifdef SE_PILOT2_READ_MAC_ADDR_SPI
	if(of_device_is_compatible(np, "pilot,eth-1"))
	  read_spi_mac_addr(dev->dev_addr, 1);
	else
	  read_spi_mac_addr(dev->dev_addr, 0);
#else
	  mac_address = of_get_mac_address(pdev->dev.of_node);
	  if (mac_address)
		  /* Set the MAC address. */
		  memcpy(dev->dev_addr, mac_address, ETH_ALEN);
	  else
		  printk(KERN_WARNING "%s:No MAC address found\n", dev->name);
#endif
	dev->irq = irq;
	
	/* The lower four bits are the media type. */
	if (board_idx >= 0  &&  board_idx < MAX_UNITS) {
	  if (options[board_idx] & MEDIA_MASK)
	    tp->default_port = options[board_idx] & MEDIA_MASK;
	  if ((options[board_idx] & FullDuplex) || full_duplex[board_idx] > 0)
	    tp->full_duplex = 1;
	  if (mtu[board_idx] > 0)
	    dev->mtu = mtu[board_idx];
	}
	if (dev->mem_start & MEDIA_MASK)
	  tp->default_port = dev->mem_start & MEDIA_MASK;
	if (tp->default_port) {
	  printk(KERN_INFO "tulip%d: Transceiver selection forced to %s.\n",
		 board_idx, medianame[tp->default_port & MEDIA_MASK]);
	  tp->medialock = 1;
	  if (tulip_media_cap[tp->default_port] & MediaAlwaysFD)
	    tp->full_duplex = 1;
	}
	if (tp->full_duplex)
	  tp->full_duplex_lock = 1;
	
	if (tulip_media_cap[tp->default_port] & MediaIsMII) {
	  u16 media2advert[] = { 0x20, 0x40, 0x03e0, 0x60, 0x80, 0x100, 0x200 };
	  tp->mii_advertise = media2advert[tp->default_port - 9];
	  tp->mii_advertise |= (tp->flags & HAS_8023X); /* Matching bits! */
	}
	
	if (tp->flags & HAS_MEDIA_TABLE) {
	  sprintf(dev->name, "tulip%d", board_idx);	/* hack */
	  tulip_parse_eeprom(dev);
	  strcpy(dev->name, "eth%d");			/* un-hack */
	}
	
	dev->netdev_ops = &tulip_netdev_ops;
	dev->watchdog_timeo = TX_TIMEOUT;
	tp->dev = dev;
#ifdef CONFIG_TULIP_NAPI
//	dev->poll = tulip_poll;
//	dev->weight = 16;
	netif_napi_add(dev, &tp->napi, tulip_poll, 16);
#endif
	dev->ethtool_ops = &ops;
#ifdef CONFIG_PILOT_SG
	dev->features   |= NETIF_F_GSO;
	dev->features   |= NETIF_F_SG;
#endif

	if (register_netdev(dev))
	  goto err_out_free_ring;

	printk(KERN_INFO "%s: %s rev %d at %p,", dev->name, chip_name, chip_rev, ioaddr);
	pilot2_devices[0] = dev;
	if (eeprom_missing)
	  printk(" EEPROM not present,");
	for (i = 0; i < 6; i++)
	  printk("%c%2.2X", i ? ':' : ' ', dev->dev_addr[i]);
	printk(", IRQ %d.\n", irq);
	
	if (tp->chip_id == PNIC2)
	  tp->link_change = pnic2_lnk_change;
	else if (tp->flags & HAS_NWAY)
	  tp->link_change = t21142_lnk_change;
	else if (tp->flags & HAS_PNICNWAY)
	  tp->link_change = pnic_lnk_change;
	
	/* Reset the xcvr interface and turn on heartbeat. */
	if (tp->mtable)
	  iowrite32(tp->mtable->csr12dir | 0x100, ioaddr + CSR12);
	
	/* put the chip in snooze mode until opened */
	tulip_set_power_state (tp, 0, 1);

	return 0;

 err_out_free_ring:
	dma_free_coherent (NULL,
			   sizeof (struct tulip_rx_desc) * RX_RING_SIZE +
			   sizeof (struct tulip_tx_desc) * TX_RING_SIZE,
			   tp->rx_ring, tp->rx_ring_dma);
	
 err_out_mtable:
	kfree (tp->mtable);
	
 err_out_free_res:
	release_region((unsigned long)ioaddr, tulip_tbl[chip_idx].io_size);
	
 err_out_free_netdev:
	free_netdev (dev);
	printk("Pilot2 Tulip initialization failed\n");
	return -ENODEV;
}


#ifdef CONFIG_PM
#if 0
static int tulip_suspend (struct pci_dev *pdev, pm_message_t state)
{
	struct net_device *dev = pci_get_drvdata(pdev);

	if (!dev)
		return -EINVAL;

	if (netif_running(dev))
		tulip_down(dev);

	netif_device_detach(dev);
	free_irq(dev->irq, dev);

	pci_save_state(pdev);
	pci_disable_device(pdev);
	pci_set_power_state(pdev, pci_choose_state(pdev, state));

	return 0;
}


static int tulip_resume(struct pci_dev *pdev)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	int retval;

	if (!dev)
		return -EINVAL;

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);

	pci_enable_device(pdev);

	if ((retval = request_irq(dev->irq, &tulip_interrupt, SA_SHIRQ, dev->name, dev))) {
	//if ((retval = request_irq(dev->irq, &tulip_interrupt, IRQ_DISABLED, dev->name, dev))) {
		printk (KERN_ERR "tulip: request_irq failed in resume\n");
		return retval;
	}

	netif_device_attach(dev);

	if (netif_running(dev))
		tulip_up(dev);

	return 0;
}
#endif
#endif /* CONFIG_PM */


static void tulip_remove_one (struct platform_device *pdev)
{
	struct net_device *dev = pilot2_devices[0];
	struct tulip_private *tp;
	void __iomem *ioaddr;
    
	if (!dev)
		return;

	tp = netdev_priv(dev);
    ioaddr = tp->base_addr;
	unregister_netdev(dev);
	dma_free_coherent (NULL,
			     sizeof (struct tulip_rx_desc) * RX_RING_SIZE +
			     sizeof (struct tulip_tx_desc) * TX_RING_SIZE,
			     tp->rx_ring, tp->rx_ring_dma);
	kfree (tp->mtable);
	free_netdev (dev);
    release_region((unsigned long)ioaddr, tulip_tbl[tp->chip_id].io_size);
    pilot2_devices[0] = NULL;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * Polling 'interrupt' - used by things like netconsole to send skbs
 * without having to re-enable interrupts. It's not called while
 * the interrupt routine is executing.
 */

static void poll_tulip (struct net_device *dev)
{
	/* disable_irq here is not very nice, but with the lockless
	   interrupt handler we have no other choice. */
	disable_irq(dev->irq);
	tulip_interrupt (dev->irq, dev, NULL);
	enable_irq(dev->irq);
}
#endif

int macstats_read_proc (struct seq_file *m, void *v, volatile long *dev_base)
{
	volatile unsigned char *stat_cntr = (unsigned char *)(dev_base) + 0x200;

	seq_printf( m, "Rx64 \t\t %u\n", ioread32(stat_cntr) );
	seq_printf( m, "Rx65to127 \t %u\n", ioread32(stat_cntr + 0x08) );
	seq_printf( m, "Rx128to255 \t %u\n", ioread32(stat_cntr + 0x10) );
	seq_printf( m, "Rx256to511 \t %u\n", ioread32(stat_cntr + 0x18) );
	seq_printf( m, "Rx512to1023 \t %u\n", ioread32(stat_cntr + 0x20) );
	seq_printf( m, "Rx1024to1518 \t %u\n", ioread32(stat_cntr + 0x28) );
	seq_printf( m, "RxTooLong \t %u\n", ioread32(stat_cntr + 0x30) );
//	seq_printf( m, "Unimplemented %u\n", ioread32(stat_cntr + 0x38) );
	seq_printf( m, "RxOctOk \t %u\n", ioread32(stat_cntr + 0x40) );
	seq_printf( m, "RxUniOk \t %u\n", ioread32(stat_cntr + 0x48) );
	seq_printf( m, "RxMultiOk \t %u\n", ioread32(stat_cntr + 0x50) );
	seq_printf( m, "RxBroadOk \t %u\n", ioread32(stat_cntr + 0x58) );
	seq_printf( m, "RxPauseOk \t %u\n", ioread32(stat_cntr + 0x60) );
	seq_printf( m, "RxAlignErr \t %u\n", ioread32(stat_cntr + 0x68) );
	seq_printf( m, "RxFCSErr \t %u\n", ioread32(stat_cntr + 0x70) ) ;
	seq_printf( m, "RxMIIErr \t %u\n", ioread32(stat_cntr + 0x78) );

	seq_printf( m, "\n");
	stat_cntr = (unsigned char *)(dev_base) + 0x300;

	seq_printf( m, "Tx64 \t\t %u\n", ioread32(stat_cntr) );
	seq_printf( m, "Tx65to127 \t %u\n", ioread32(stat_cntr + 0x08) );
	seq_printf( m, "Tx128to255 \t %u\n", ioread32(stat_cntr + 0x10) );
	seq_printf( m, "Tx256to511 \t %u\n", ioread32(stat_cntr + 0x18) );
	seq_printf( m, "Tx512to1023 \t %u\n", ioread32(stat_cntr + 0x20) );
	seq_printf( m, "Tx1024to1518 \t %u\n", ioread32(stat_cntr + 0x28) );
//	seq_printf( m, "Unimplemented %u\n", ioread32(stat_cntr + 0x30) );
//	seq_printf( m, "Unimplemented %u\n", ioread32(stat_cntr + 0x38) );
	seq_printf( m, "TxOctOk \t %u\n", ioread32(stat_cntr + 0x40) );
	seq_printf( m, "TxUniOk \t %u\n", ioread32(stat_cntr + 0x48) );
	seq_printf( m, "TxMultiOk \t %u\n", ioread32(stat_cntr + 0x50) );
	seq_printf( m, "TxBroadOk \t %u\n", ioread32(stat_cntr + 0x58) );
	seq_printf( m, "TxPauseOk \t %u\n", ioread32(stat_cntr + 0x60) );
	seq_printf( m, "TxColl0 \t %u\n", ioread32(stat_cntr + 0x68) );
	seq_printf( m, "TxColl1 \t %u\n", ioread32(stat_cntr + 0x70) );
	seq_printf( m, "TxCollMulti \t %u\n", ioread32(stat_cntr + 0x78) );
	seq_printf( m, "TxDefer \t %u\n", ioread32(stat_cntr + 0x80) );
	seq_printf( m, "TxExColl \t %u\n", ioread32(stat_cntr + 0x88) );
	seq_printf( m, "TxLCErr \t %u\n", ioread32(stat_cntr + 0x90) );
	seq_printf( m, "TxCSErr \t %u\n", ioread32(stat_cntr + 0x98) );
	seq_printf( m, "TxMACErr \t %u\n", ioread32(stat_cntr + 0xa0) );

	seq_printf( m, "\n");

	return 0;
}


int read_proc_mac0(struct seq_file *m, void *v)
{
	return macstats_read_proc (m, v, dev_iobaseaddr0);
}

int read_proc_mac1(struct seq_file *m, void *v)
{
	return macstats_read_proc (m, v, dev_iobaseaddr1);
}


static int open_proc_mac0(struct inode *inode, struct  file *file)
{
	return single_open(file, read_proc_mac0, NULL);
}

static int open_proc_mac1(struct inode *inode, struct  file *file)
{
	return single_open(file, read_proc_mac1, NULL);
}

static const struct file_operations proc_mac0_fops = {
	.owner = THIS_MODULE,
	.open = open_proc_mac0,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations proc_mac1_fops = {
	.owner = THIS_MODULE,
	.open = open_proc_mac1,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int tulip_init (struct platform_device *pdev)
{
	int status;
	struct device_node *np = pdev->dev.of_node;
#ifdef MODULE
	printk (KERN_INFO "%s", version);
#endif

	/* copy module parms into globals */
	tulip_rx_copybreak = rx_copybreak;
	tulip_max_interrupt_work = max_interrupt_work;

	status = tulip_init_one(pdev);
	if(status < 0)
	{
		tulip_remove_one(pdev);
		return -1;
	}

	if(of_device_is_compatible(np, "pilot,eth-1"))
		proc_mac1stats = proc_create(filename1, 0, NULL, &proc_mac1_fops);
	else
		proc_mac0stats = proc_create(filename0, 0, NULL, &proc_mac0_fops);


	return 0;
}


static int tulip_cleanup (struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;

    tulip_remove_one(pdev);
    tulip_remove_one(pdev);
	if(of_device_is_compatible(np, "pilot,eth-0"))
	if (proc_mac0stats)
		remove_proc_entry(filename0, NULL);
	if(of_device_is_compatible(np, "pilot,eth-1"))
	if (proc_mac1stats)
		remove_proc_entry(filename1, NULL);

	return 0;
}

static const struct of_device_id pilot_eth_of_match[] = {
	{ .compatible = "pilot,eth-0" },
	{ .compatible = "pilot,eth-1" },
	{ }
};
MODULE_DEVICE_TABLE(of, pilot_eth_of_match);

static struct platform_driver pilot_mac_driver = {
	.driver = {
		.name  = "tulip-pilot",
		.of_match_table = pilot_eth_of_match,
	},
	.probe   = tulip_init,
	.remove  = tulip_cleanup,
	.suspend = NULL,
	.resume  = NULL,
};

module_platform_driver(pilot_mac_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ashok Reddy Soma <ashok.soma@aspeedtech.com>");
MODULE_DESCRIPTION("Aspeed' Pilot4 MAC Driver");
