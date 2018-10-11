/*******************************************************************************
 *
 * Copyright (C) 2004-2014 Emulex. All rights reserved.
 * EMULEX is a trademark of Emulex.
 * www.emulex.com
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of version 2 of the GNU General Public License as published by the
 * Free Software Foundation.
 * This program is distributed in the hope that it will be useful. ALL EXPRESS
 * OR IMPLIED CONDITIONS, REPRESENTATIONS AND WARRANTIES, INCLUDING ANY IMPLIED
 * WARRANTY OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
 * NON-INFRINGEMENT, ARE DISCLAIMED, EXCEPT TO THE EXTENT THAT SUCH DISCLAIMERS
 * ARE HELD TO BE LEGALLY INVALID. See the GNU General Public License for more
 * details, a copy of which can be found in the file COPYING included
 * with this package.
 *
 ********************************************************************************/
#ifndef __PILOT_REALTEK_PHY_H__
#define __PILOT_REALTEK_PHY_H__
//PHY reg offsets 
#define PILOT_RT_BMCR	0x00 
#define PILOT_RT_BMSR	0x01 
#define PILOT_RT_PHYID1	0x02 
#define PILOT_RT_PHYID2	0x03 
#define PILOT_RT_ANAR	0x04 
#define PILOT_RT_ANLPAR	0x05 
#define PILOT_RT_ANER	0x06 
#define PILOT_RT_ANNPTR	0x07 
#define PILOT_RT_ANNPRR	0x08 
#define PILOT_RT_GBCR	0x09 
#define PILOT_RT_GBSR	0x0A 
#define PILOT_RT_GBESR	0x0F 
#define PILOT_RT_PHYCR	0x10 
#define PILOT_RT_PHYSR	0x11 
#define PILOT_RT_INER	0x12 
#define PILOT_RT_INSR	0x13 
#define PILOT_RT_RXERC	0x14 
#define PILOT_RT_LEDCR	0x15 
#define PILOT_RT_PAGSEL	0x1F 

#define BITS1		0x0001
#define BITS2		0x0003
#define BITS3		0x0007
#define BITS4		0x000F
#define BITS5		0x001F
#define BITS6		0x003F
#define BITS7		0x007F
#define BITS8		0x00FF
#define BITS9		0x01FF
#define BITSA		0x03FF
#define BITSB		0x07FF
#define BITSC		0x0FFF
#define BITSD		0x1FFF
#define BITSE		0x3FFF
#define BITSF		0x7FFF
//Reg 0x00 - BMCR register defines
#define BMCR_SET_DUPLX                  0x100
//Reg 0x11 defines - PHY specific Status register
#define REG0x11_FULL_DUPLX_MODE         0x00002000
#define REG0x11_10Mbps                  0x00000000
#define REG0x11_100Mbps                 0x00004000
#define REG0x11_1000Mbps                0x00008000

//Reg 0x12 & 0x13 defines - INER and INSR registers
#define AUTO_NEGO_ERR_INT       0x8000
#define SPEED_CHNG_INT          0x4000
#define DUPLX_MOD_CHNG_INT      0x2000
#define PAGE_RECVD_INT          0x1000
#define AUTO_NEGO_COMPL_INT     0x0800
#define LINK_STS_CHNG_INT       0x0400
#define SYMBL_ERR_INT           0x0200
#define FALSE_CARR_INT          0x0100
#define MDI_CROSS_CHNG_INT      0x0040
#define POL_CHNG_INT            0x0002
#define JABBER_INT              0x0001

//MICREL REG Defines
#define AN_LP_ABILITY         0x5
#define Extended_Mii_Sts      0xf     
#define Mircel_Phy_Control    0x1F
#define Mircel_Intr_Sts       0x1B  

  
#if 1
#define PHY_BROADCOM	0x1E 
#define PHY_SMSC	0x0C
#define PHY_REALTEK	0x11
//#define PHY_MICREL	0x21
#define PHY_MICREL	0x22
#define PHY_MARVELL	0x1D
#define PHY_SMSC_REVB	0x0F
    
#endif

  
#endif


  
