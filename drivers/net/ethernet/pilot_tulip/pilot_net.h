// SPDX-License-Identifier: GPL-2.0
/*******************************************************************************
 *
 * Copyright 2019 Aspeed Technology Inc
 * Ashok Reddy Soma <ashok.soma@aspeedtech.com>
 * This file is based on drivers/net/ethernet/dec/tulip/tulip.h
 * which was written by Donald Becker.
 *
 ********************************************************************************/
#ifndef __PILOT_NET_H__
#define __PILOT_NET_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/netdevice.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/irq.h>

/* undefine, or define to various debugging levels (>4 == obscene levels) */
#define TULIP_DEBUG 1
#undef TULIP_DEBUG

/* undefine USE_IO_OPS for MMIO, define for PIO */
#ifdef CONFIG_TULIP_MMIO
# undef USE_IO_OPS
#else
# define USE_IO_OPS 1
#endif

#define PHY_BROADCOM    0x1E
#define PHY_SMSC        0x0C
#define PHY_REALTEK     0x11
//#define PHY_MICREL    0x21
#define PHY_MICREL      0x22
#define PHY_MARVELL     0x1D
#define PHY_SMSC_REVB   0x0F

#define PILOT_RT_PHYSR  0x11
#define BMCR_SET_DUPLX                  0x100
#define REG0x11_FULL_DUPLX_MODE         0x00002000
#define REG0x11_10Mbps                  0x00000000
#define REG0x11_100Mbps                 0x00004000
#define REG0x11_1000Mbps                0x00008000

#define Mircel_Phy_Control    0x1F
#define Mircel_Intr_Sts       0x1B

extern volatile long *dev_iobaseaddr0 ;
extern volatile long *dev_iobaseaddr1 ;
#define IOADDR_INTERFACE_ETH_A (dev_iobaseaddr0)
#define IOADDR_INTERFACE_ETH_B (dev_iobaseaddr1)

struct tulip_chip_table {
	char *chip_name;
	int io_size;
	int valid_intrs;	/* CSR7 interrupt enable settings */
	int flags;
	void (*media_timer) (struct timer_list *);
        work_func_t media_task;
};


enum tbl_flag {
	HAS_MII			= 0x0001,
	HAS_MEDIA_TABLE		= 0x0002,
	CSR12_IN_SROM		= 0x0004,
	ALWAYS_CHECK_MII	= 0x0008,
	HAS_ACPI		= 0x0010,
	MC_HASH_ONLY		= 0x0020, /* Hash-only multicast filter. */
	HAS_PNICNWAY		= 0x0080,
	HAS_NWAY		= 0x0040, /* Uses internal NWay xcvr. */
	HAS_INTR_MITIGATION	= 0x0100,
	IS_ASIX			= 0x0200,
	HAS_8023X		= 0x0400,
	COMET_MAC_ADDR		= 0x0800,
	HAS_PCI_MWI		= 0x1000,
	HAS_PHY_IRQ		= 0x2000,
	HAS_SWAPPED_SEEPROM	= 0x4000,
	NEEDS_FAKE_MEDIA_TABLE	= 0x8000,
};


/* chip types.  careful!  order is VERY IMPORTANT here, as these
 * are used throughout the driver as indices into arrays */
enum chips {
	DC21040 = 0,
	DC21041 = 1,
	DC21140 = 2,
	DC21142 = 3, DC21143 = 3,
	LC82C168,
	MX98713,
	MX98715,
	MX98725,
	AX88140,
	PNIC2,
	COMET,
	COMPEX9881,
	I21145,
	DM910X,
	CONEXANT,
	PILOT_TULIP,
};

#define MAC_RT    (0xf<<20)
#define MAC_NTP   (0x7<<24)

#define MAC_TIM          (0x1ffff)  //2secs
#define MAC_TIM_100MB    (0x16000)  //2secs
#define MAC_TIM_10MB     (0x10500) //2secs

//this is default continous 2 sec timer mode and  rx time out
#define CSR11_TIM            (MAC_TIM|MAC_RT)         // 1GIG NORMAL GTE
#define CSR11_TIM_10MB       (MAC_TIM_10MB|MAC_RT)  
#define CSR11_TIM_100MB      (MAC_TIM_100MB|MAC_RT)  

#define SE_RD_DIV  16
  
#define SPEED_NEG_10    10
#define SPEED_NEG_100	100 
#define SPEED_NEG_1000	1000
  
enum MediaIs {
	MediaIsFD = 1,
	MediaAlwaysFD = 2,
	MediaIsMII = 4,
	MediaIsFx = 8,
	MediaIs100 = 16
};


/* Offsets to the Command and Status Registers, "CSRs".  All accesses
   must be longword instructions and quadword aligned. */
enum tulip_offsets {
	CSR0  = 0,
	CSR1  = 0x08,
	CSR2  = 0x10,
	CSR3  = 0x18,
	CSR4  = 0x20,
	CSR5  = 0x28,
	CSR6  = 0x30,
	CSR7  = 0x38,
	CSR8  = 0x40,
	CSR9  = 0x48,
	CSR10 = 0x50,
	CSR11 = 0x58,
	CSR12 = 0x60,
	CSR13 = 0x68,
	CSR14 = 0x70,
	CSR15 = 0x78,
	CSR16 = 0x80,
	CSR17 = 0x88,
	CSR18 = 0x90,
	CSR19 = 0x98,
	CSR20 = 0xA0,
	CSR21 = 0xA8,
	CSR22 = 0xB0
};

#define RxPollInt (RxIntr|RxNoBuf|RxDied|RxJabber)

/* The bits in the CSR5 status registers, mostly interrupt sources. */
enum status_bits {
	TimerInt = 0x800,
	SystemError = 0x2000,
	TPLnkFail = 0x1000,
	TPLnkPass = 0x10,
	NormalIntr = 0x10000,
	AbnormalIntr = 0x8000,
	RxJabber = 0x200,
	RxDied = 0x100,
	RxNoBuf = 0x80,
	RxIntr = 0x40,
	TxFIFOUnderflow = 0x20,
	TxJabber = 0x08,
	TxNoBuf = 0x04,
	TxDied = 0x02,
	TxIntr = 0x01,
};

/* bit mask for CSR5 TX/RX process state */
#define CSR5_TS	0x00700000
#define CSR5_RS	0x000e0000

enum tulip_mode_bits {
	TxThreshold		= (1 << 22),
	FullDuplex		= (1 << 9),
	TxOn			= 0x2000,
	AcceptBroadcast		= 0x0100,
	AcceptAllMulticast	= 0x0080,
	AcceptAllPhys		= 0x0040,
	AcceptRunt		= 0x0008,
	RxOn			= 0x0002,
	RxTx			= (TxOn | RxOn),
};

enum tulip_busconfig_bits {
	MWI			= (1 << 24),
	MRL			= (1 << 23),
	MRM			= (1 << 21),
	CALShift		= 14,
	BurstLenShift		= 8,
};

/* The Tulip Rx and Tx buffer descriptors. */
struct tulip_rx_desc {
	s32 status;
	s32 length;
	u32 buffer1;
	u32 buffer2;
};

struct tulip_tx_desc {
	s32 status;
	s32 length;
	u32 buffer1;
	u32 buffer2;		/* We use only buffer 1.  */
};

enum desc_status_bits {
	DescOwned = 0x80000000,
	RxDescFatalErr = 0x8000,
	RxWholePkt = 0x0300,
};

/* Keep the ring sizes a power of two for efficiency.
   Making the Tx ring too large decreases the effectiveness of channel
   bonding and packet priority.
   There are no ill effects from too-large receive rings. */

#define TX_RING_SIZE	64
#define RX_RING_SIZE	256 
#define MEDIA_MASK     31

#define PKT_BUF_SZ		1536	/* Size of each temporary Rx buffer. */

#define TULIP_MIN_CACHE_LINE	8	/* in units of 32-bit words */

#if defined(__sparc__) || defined(__hppa__)
/* The UltraSparc PCI controllers will disconnect at every 64-byte
 * crossing anyways so it makes no sense to tell Tulip to burst
 * any more than that.
 */
#define TULIP_MAX_CACHE_LINE	16	/* in units of 32-bit words */
#else
#define TULIP_MAX_CACHE_LINE	32	/* in units of 32-bit words */
#endif


/* Ring-wrap flag in length field, use for last ring entry.
	0x01000000 means chain on buffer2 address,
	0x02000000 means use the ring start address in CSR2/3.
   Note: Some work-alike chips do not function correctly in chained mode.
   The ASIX chip works only in chained mode.
   Thus we indicates ring mode, but always write the 'next' field for
   chained mode as well.
*/
#define DESC_RING_WRAP 0x02000000

#define EEPROM_SIZE 512 	/* 2 << EEPROM_ADDRLEN */
#define RUN_AT(x) (jiffies + (x))

#if defined(__i386__)			/* AKA get_unaligned() */
#define get_u16(ptr) (*(u16 *)(ptr))
#else
#define get_u16(ptr) (((u8*)(ptr))[0] + (((u8*)(ptr))[1]<<8))
#endif

struct medialeaf {
	u8 type;
	u8 media;
	unsigned char *leafdata;
};

struct mediatable {
	u16 defaultmedia;
	u8 leafcount;
	u8 csr12dir;		/* General purpose pin directions. */
	unsigned has_mii:1;
	unsigned has_nonmii:1;
	unsigned has_reset:6;
	u32 csr15dir;
	u32 csr15val;		/* 21143 NWay setting. */
	struct medialeaf mleaf[0];
};

struct mediainfo {
	struct mediainfo *next;
	int info_type;
	int index;
	unsigned char *info;
};

struct ring_info {
	struct sk_buff	*skb;
	dma_addr_t	mapping;
#ifdef CONFIG_PILOT_SG
	int maplen;
#endif

};

struct tulip_private {
	const char *product_name;
	struct net_device *next_module;
	struct tulip_rx_desc *rx_ring;
	struct tulip_tx_desc *tx_ring;
	dma_addr_t rx_ring_dma;
	dma_addr_t tx_ring_dma;
	/* The saved address of a sent-in-place packet/buffer, for skfree(). */
	struct ring_info tx_buffers[TX_RING_SIZE];
	/* The addresses of receive-in-place skbuffs. */
	struct ring_info rx_buffers[RX_RING_SIZE];
	u16 setup_frame[96];	/* Pseudo-Tx frame to init address table. */
	int chip_id;
	int revision;
	int flags;
	struct napi_struct napi;
	struct net_device_stats stats;
	struct timer_list timer;	/* Media selection timer. */
	struct timer_list oom_timer;    /* Out of memory timer. */
	u32 mc_filter[2];
	spinlock_t lock;
	spinlock_t mii_lock;
	unsigned int cur_rx, cur_tx;	/* The next free ring entry */
	unsigned int dirty_rx, dirty_tx;	/* The ring entries to be free()ed. */
        unsigned int p_tx_coalising_en;		      

#ifdef 	CONFIG_TULIP_NAPI_HW_MITIGATION
        int mit_on;
#endif
	unsigned int full_duplex:1;	/* Full-duplex operation requested. */
	unsigned int full_duplex_lock:1;
	unsigned int fake_addr:1;	/* Multiport board faked address. */
	unsigned int default_port:4;	/* Last dev->if_port value. */
	unsigned int media2:4;	/* Secondary monitored media port. */
	unsigned int medialock:1;	/* Don't sense media type. */
	unsigned int mediasense:1;	/* Media sensing in progress. */
	unsigned int nway:1, nwayset:1;		/* 21143 internal NWay. */
	unsigned int timeout_recovery:1;
	unsigned int csr0;	/* CSR0 setting. */
	unsigned int csr6;	/* Current CSR6 control settings. */
	unsigned char eeprom[EEPROM_SIZE];	/* Serial EEPROM contents. */
	void (*link_change) (struct net_device * dev, int csr5);
	u16 sym_advertise, mii_advertise; /* NWay capabilities advertised.  */
	u16 lpar;		/* 21143 Link partner ability. */
	u16 advertising[4];
	int old_link;          /* used by tp_adjust_link */
	int old_speed;
	int old_duplex;
	struct phy_device *phy_dev;
	struct mii_bus *mii_bus;
	signed char phys[4], mii_cnt;	/* MII device addresses. */
       unsigned int    phy_manuf_model_num;
		      unsigned int    speed_neg;
		      
        u16     prev_isr_contents[4];

	struct mediatable *mtable;
	int cur_index;		/* Current media index. */
	int saved_if_port;
	struct pci_dev *pdev;
	int ttimer;
	int susp_rx;
	unsigned long nir;
	void __iomem *base_addr;
	int csr12_shadow;
	int pad0;		/* Used for 8-byte alignment */
	struct work_struct media_work;
	struct net_device *dev;
	struct device *ddev;
};

void Lan_MII_MDIO_WriteReg16(
        struct net_device *dev,        /* Lan port */
        unsigned char  RegNum,      /* MII PHY register number */
        unsigned char phy_id,
        unsigned int Data         /* Data to be written */
);
unsigned int Lan_MII_MDIO_ReadReg16(
        struct net_device *dev,
        unsigned char phy_id, 
        unsigned char RegNum        /* MII PHY register number */
);

/* pilot_intr.c */
extern unsigned int tulip_max_interrupt_work;
extern int tulip_rx_copybreak;
irqreturn_t tulip_interrupt(int irq, void *dev_instance);
int tulip_refill_rx(struct net_device *dev);
#ifdef CONFIG_TULIP_NAPI
int tulip_poll(struct napi_struct *napi, int budget);
#endif

/* pilot_phy.c */
int tulip_mdio_read(struct net_device *dev, int phy_id, int location);
void tulip_mdio_write(struct net_device *dev, int phy_id, int location, int value);
void tulip_select_media(struct net_device *dev, int startup);
int tulip_check_duplex(struct net_device *dev);
void tulip_find_mii (struct net_device *dev, int board_idx);
void tulip_media_task(struct work_struct *work);

/* pilot_net.c */
extern int tulip_debug;
extern const char * const medianame[];
extern const char tulip_media_cap[];
extern struct tulip_chip_table tulip_tbl[];
void oom_timer(unsigned long data);

static inline void tulip_start_rxtx(struct tulip_private *tp)
{
	void __iomem *ioaddr = tp->base_addr;
	iowrite32(tp->csr6 | RxTx, ioaddr + CSR6);
	barrier();
	(void) ioread32(ioaddr + CSR6); /* mmio sync */
}

static inline void tulip_stop_rxtx(struct tulip_private *tp)
{
	void __iomem *ioaddr = tp->base_addr;
	u32 csr6 = ioread32(ioaddr + CSR6);

	if (csr6 & RxTx) 
	{
		unsigned i=1300/10;
		iowrite32(csr6 & ~RxTx, ioaddr + CSR6);
		barrier();
		/* wait until in-flight frame completes.
		 * Max time @ 10BT: 1500*8b/10Mbps == 1200us (+ 100us margin)
		 * Typically expect this loop to end in < 50 us on 100BT.
		 */

		while (--i && (ioread32(ioaddr + CSR5) & (CSR5_TS|CSR5_RS)))
		{
			udelay(10);
		}

		if (!i)
		{
			printk(KERN_DEBUG "%s: tulip_stop_rxtx() failed\n", __FUNCTION__);
		}
	}
}

static inline void tulip_restart_rxtx(struct tulip_private *tp)
{
	tulip_stop_rxtx(tp);
	udelay(5);
	tulip_start_rxtx(tp);
}

static inline void tulip_tx_timeout_complete(struct tulip_private *tp, void __iomem *ioaddr)
{
	/* Stop and restart the chip's Tx processes. */
	tulip_restart_rxtx(tp);
	/* Trigger an immediate transmit demand. */
	iowrite32(0, ioaddr + CSR1);

	tp->dev->stats.tx_errors++;
}

#endif /* __PILOT_NET_H__ */
