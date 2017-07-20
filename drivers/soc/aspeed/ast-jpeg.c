/********************************************************************************
* File Name     : ast_jpeg.c
* Author         : Ryan Chen
* Description   : AST JPEG Engine Controller
* 
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
*
*   Version      : 1.0
*   History      : 
*      1. 2013/04/30 Ryan Chen create this file 
*    
********************************************************************************/

#include <linux/slab.h>
#include <linux/sched.h>

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>

#include <linux/hwmon-sysfs.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/platform.h>
#include <mach/aspeed.h>
#include <mach/ast-scu.h>

#include <linux/dma-mapping.h>

#define CONFIG_AST_JPEG_DEBUG

#ifdef CONFIG_AST_JPEG_DEBUG
	//#define JPEG_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)	
	#define JPEG_DBG(fmt, args...) printk("%s() " fmt,__FUNCTION__, ## args)
#else
	#define JPEG_DBG(fmt, args...)
#endif

/*************************************  Registers for JPEG ***********************************************/ 
#define AST_JPEG_PROTECT			0x000		/*	protection key register	*/
#define AST_JPEG_SEQ_CTRL			0x004		/*	JPEG Sequence Control register	*/

#define AST_JPEG_SIZE_SETTING		0x034		/*	JPEG Size Setting Register */

#define AST_JPEG_HEADER_BUFF		0x040		/*	JPEG Base Address of JPEG Header Buffer Register when VR004[13]=1  */
#define AST_JPEG_SOURCE_BUFF0		0x044		/*	JPEG Base Address of JPEG Source Buffer #1 Register */
#define AST_JPEG_SOURCE_SCAN_LINE	0x048		/*	JPEG Scan Line Offset of JPEG Source Buffer Register */
#define AST_JPEG_SOURCE_BUFF1		0x04C		/*	JPEG Base Address of JPEG Source Buffer #2 Register */

#define AST_JPEG_STREAM_BUFF		0x054		/*	JPEG Base Address of Compressed JPEG Stream Buffer Register */

#define AST_JPEG_COMPRESS_CTRL	0x060		/*   JPEG Compression Control Register */
#define AST_JPEG_EFFECT_CTRL		0x064		/*   JPEG JPEG effecive bit control Register   */
#define AST_JPEG_QUANTIZ_TABLE		0x068		/*   JPEG Quantization value     */

#define AST_JPEG_STREAM_SIZE		0x070		/*	JPEG Total Size of Compressed JPEG Stream Read Back Register */
#define AST_JPEG_BLOCK_COUNT		0x074		/*	JPEG Total Number of Compressed JPEG Blocks Read Back Register */

#define AST_JPEG_CTRL				0x300		/*	JPEG Control Register */
#define AST_JPEG_IER					0x304		/*	JPEG interrupt Enable */
#define AST_JPEG_ISR					0x308		/*	JPEG interrupt status */
#define AST_JPEG_RESTRICT_START	0x310		/*	JPEG Memory Restriction Area Starting Address Register */
#define AST_JPEG_RESTRICT_END		0x314		/*	JPEG Memory Restriction Area Starting Address Register */

#define AST_JPEG_HUFFMAN_TABLE	0x55C		/*	JPEG Huffman Table */
/**************************************************************************************************/ 

/*	AST_JPEG_PROTECT: 0x000  - protection key register */
#define JPEG_PROTECT_UNLOCK			0x1A038AA8

/*	AST_JPEG_SEQ_CTRL		0x004		JPEG Sequence Control register	*/
#define JPEG_HALT_ENG_STS				(1 << 21)

#define JPEG_COMPRESS_BUSY			(1 << 18)
#define JPEG_COMPRESS_MODE_ENABLE	(1 << 13)	/*  0: ASPEED proprietary compression mode,  1: Enable JPEG compatible mode  */
#define JPEG_HALT_ENG_TRIGGER			(1 << 12)
#define JPEG_COMPRESS_MODE_MASK		(3 << 10)
//only support YUV420 no YUV444
#define JPEG_YUY2_COMPRESS			(1 << 10)
#define JPEG_NV12_COMPRESS			(3 << 10)


#define JPEG_AUTO_COMPRESS			(1 << 5)
#define JPEG_COMPRESS_TRIGGER			(1 << 4)
#define JPEG_CAPTURE_MULTI_FRAME		(1 << 3)
#define JPEG_COMPRESS_FORCE_IDLE		(1 << 2)

/*	 AST_JPEG_COMPRESS_WIN	0x034		JPEG Compression Window Setting Register */
#define JPEG_COMPRESS_H(x)				(((x) & 0x1fff) << 16)
#define JPEG_GET_COMPRESS_H(x)			(((x) >> 16) & 0x1fff)

#define JPEG_COMPRESS_V(x)				((x) & 0x1fff)
#define JPEG_GET_COMPRESS_V(x)			((x) & 0x1fff)

/* AST_JPEG_COMPRESS_CTRL	0x060		JPEG Compression Control Register */
#define JPEG_DCT_TEST(x)					((x) << 17)
#define JPEG_DCT_MASK					(0x3ff << 6)

/* AST_JPEG_CTRL			0x300		JPEG Control Register */
#define JPEG_QUANT_TABLE_LOAD			(1 << 30)

#define JPEG_DATA_WRITE_DISABLE		(1 << 28)
#define JPEG_YUV2JFIF					(1 << 27)

#define JPEG_VERTICAL_BORDER_MASK		(1 << 23)

#define JPEG_PROGRAM_QUANT_TABLE_EN	(1 << 18)

/* AST_JPEG_IER			0x304		JPEG interrupt Enable */
/* AST_JPEG_ISR			0x308		JPEG interrupt status */
#define JPEG_HANG_WDT_ISR				(1 << 9)
#define JPEG_HALT_RDY_ISR				(1 << 8)

#define JPEG_COMPLETE_ISR				(1 << 5)

#define JPEG_COMPRESS_COMPLETE		(1 << 3)

/***********************************************************************/
struct ast_jpeg_data {
	struct device		*misc_dev;
	void __iomem		*reg_base;			/* virtual */
	int 	irq;				//JPEG IRQ number 
	u32                             *jpeg_tbl_virt;         /* virt */
	dma_addr_t 			jpeg_tbl_dma_addr;
	
	phys_addr_t             *jpeg_phy;            /* phy */
	u32                             *jpeg_virt;           /* virt */
	phys_addr_t             *buff0_phy;             /* phy */
	u32                             *buff0_virt;            /* virt */
	phys_addr_t             *buff1_phy;             /* phy */
	u32                             *buff1_virt;            /* virt */

	u32		jpeg_mem_size;			/* phy size*/		
	u32		raw_buff0_offset;			/* buff0 offset*/
	u32		raw_buff1_offset;			/* buff1 offset*/
	
	struct completion	jpeg_complete;		

	u32		flag;
	u32		sts;

	struct mutex lock;	

	bool is_open;
};

/***********************************************************************/
#define JPEG_RAW_BUFF0_OFFSET 0x1000000
#define JPEG_RAW_BUFF1_OFFSET 0x2000000
/***********************************************************************/
struct ast_jpeg_mode
{
	u16		x;
	u16		y;
	u8		format; //0: YUY2,  1:NV12
};

#define JPEGIOC_BASE				'J'

#define AST_JPEG_SET_CONFIG					_IOW(JPEGIOC_BASE, 0x0, struct ast_jpeg_mode*)
#define AST_JPEG_GET_CONFIG					_IOR(JPEGIOC_BASE, 0x1, struct ast_jpeg_mode*)
#define AST_JPEG_GET_MEM_SIZE_IOCRX			_IOR(JPEGIOC_BASE, 0x2, unsigned long)
#define AST_JPEG_GET_BUFFER0_OFFSET_IOCRX	_IOR(JPEGIOC_BASE, 0x3, unsigned long)
#define AST_JPEG_GET_BUFFER1_OFFSET_IOCRX	_IOR(JPEGIOC_BASE, 0x4, unsigned long)
#define AST_JPEG_TRIGGER						_IOR(JPEGIOC_BASE, 0x5, unsigned long)

/***********************************************************************/

static inline void
ast_jpeg_write(struct ast_jpeg_data *ast_jpeg, u32 val, u32 reg)
{
//	JPEG_DBG("write offset: %x, val: %x \n",reg,val);
#ifdef CONFIG_AST_JPEG_LOCK
	//unlock 
	writel(JPEG_PROTECT_UNLOCK, ast_jpeg->reg_base);
	writel(val, ast_jpeg->reg_base + reg);
	//lock
	writel(0xaa,ast_jpeg->reg_base);	
#else
	//JPEG is lock after reset, need always unlock 
	//unlock 
	writel(JPEG_PROTECT_UNLOCK, ast_jpeg->reg_base);
	writel(val, ast_jpeg->reg_base + reg);
#endif	
}

static inline u32
ast_jpeg_read(struct ast_jpeg_data *ast_jpeg, u32 reg)
{
	u32 val = readl(ast_jpeg->reg_base + reg);
//	JPEG_DBG("read offset: %x, val: %x \n",reg,val);
	return val;
}

/************************************************ JPEG ***************************************************************************************/
void ast_init_jpeg_table(struct ast_jpeg_data *ast_jpeg)
{
	int i = 0;
	int base=0;
	//JPEG header default value:
	ast_jpeg->jpeg_tbl_virt[base + 0] = 0xE0FFD8FF;
	ast_jpeg->jpeg_tbl_virt[base + 1] = 0x464A1000;
	ast_jpeg->jpeg_tbl_virt[base + 2] = 0x01004649;
	ast_jpeg->jpeg_tbl_virt[base + 3] = 0x60000101;
	ast_jpeg->jpeg_tbl_virt[base + 4] = 0x00006000;
	ast_jpeg->jpeg_tbl_virt[base + 5] = 0x0F00FEFF;
	ast_jpeg->jpeg_tbl_virt[base + 6] = 0x00002D05;
	ast_jpeg->jpeg_tbl_virt[base + 7] = 0x00000000;
	ast_jpeg->jpeg_tbl_virt[base + 8] = 0x00000000;
	ast_jpeg->jpeg_tbl_virt[base + 9] = 0x00DBFF00;
	ast_jpeg->jpeg_tbl_virt[base + 44] = 0x081100C0;
	ast_jpeg->jpeg_tbl_virt[base + 45] = 0x00000000;
	ast_jpeg->jpeg_tbl_virt[base + 47] = 0x03011102;
	ast_jpeg->jpeg_tbl_virt[base + 48] = 0xC4FF0111;
	ast_jpeg->jpeg_tbl_virt[base + 49] = 0x00001F00;
	ast_jpeg->jpeg_tbl_virt[base + 50] = 0x01010501;
	ast_jpeg->jpeg_tbl_virt[base + 51] = 0x01010101;
	ast_jpeg->jpeg_tbl_virt[base + 52] = 0x00000000;
	ast_jpeg->jpeg_tbl_virt[base + 53] = 0x00000000;
	ast_jpeg->jpeg_tbl_virt[base + 54] = 0x04030201;
	ast_jpeg->jpeg_tbl_virt[base + 55] = 0x08070605;
	ast_jpeg->jpeg_tbl_virt[base + 56] = 0xFF0B0A09;
	ast_jpeg->jpeg_tbl_virt[base + 57] = 0x10B500C4;
	ast_jpeg->jpeg_tbl_virt[base + 58] = 0x03010200;
	ast_jpeg->jpeg_tbl_virt[base + 59] = 0x03040203;
	ast_jpeg->jpeg_tbl_virt[base + 60] = 0x04040505;
	ast_jpeg->jpeg_tbl_virt[base + 61] = 0x7D010000;
	ast_jpeg->jpeg_tbl_virt[base + 62] = 0x00030201;
	ast_jpeg->jpeg_tbl_virt[base + 63] = 0x12051104;
	ast_jpeg->jpeg_tbl_virt[base + 64] = 0x06413121;
	ast_jpeg->jpeg_tbl_virt[base + 65] = 0x07615113;
	ast_jpeg->jpeg_tbl_virt[base + 66] = 0x32147122;
	ast_jpeg->jpeg_tbl_virt[base + 67] = 0x08A19181;
	ast_jpeg->jpeg_tbl_virt[base + 68] = 0xC1B14223;
	ast_jpeg->jpeg_tbl_virt[base + 69] = 0xF0D15215;
	ast_jpeg->jpeg_tbl_virt[base + 70] = 0x72623324;
	ast_jpeg->jpeg_tbl_virt[base + 71] = 0x160A0982;
	ast_jpeg->jpeg_tbl_virt[base + 72] = 0x1A191817;
	ast_jpeg->jpeg_tbl_virt[base + 73] = 0x28272625;
	ast_jpeg->jpeg_tbl_virt[base + 74] = 0x35342A29;
	ast_jpeg->jpeg_tbl_virt[base + 75] = 0x39383736;
	ast_jpeg->jpeg_tbl_virt[base + 76] = 0x4544433A;
	ast_jpeg->jpeg_tbl_virt[base + 77] = 0x49484746;
	ast_jpeg->jpeg_tbl_virt[base + 78] = 0x5554534A;
	ast_jpeg->jpeg_tbl_virt[base + 79] = 0x59585756;
	ast_jpeg->jpeg_tbl_virt[base + 80] = 0x6564635A;
	ast_jpeg->jpeg_tbl_virt[base + 81] = 0x69686766;
	ast_jpeg->jpeg_tbl_virt[base + 82] = 0x7574736A;
	ast_jpeg->jpeg_tbl_virt[base + 83] = 0x79787776;
	ast_jpeg->jpeg_tbl_virt[base + 84] = 0x8584837A;
	ast_jpeg->jpeg_tbl_virt[base + 85] = 0x89888786;
	ast_jpeg->jpeg_tbl_virt[base + 86] = 0x9493928A;
	ast_jpeg->jpeg_tbl_virt[base + 87] = 0x98979695;
	ast_jpeg->jpeg_tbl_virt[base + 88] = 0xA3A29A99;
	ast_jpeg->jpeg_tbl_virt[base + 89] = 0xA7A6A5A4;
	ast_jpeg->jpeg_tbl_virt[base + 90] = 0xB2AAA9A8;
	ast_jpeg->jpeg_tbl_virt[base + 91] = 0xB6B5B4B3;
	ast_jpeg->jpeg_tbl_virt[base + 92] = 0xBAB9B8B7;
	ast_jpeg->jpeg_tbl_virt[base + 93] = 0xC5C4C3C2;
	ast_jpeg->jpeg_tbl_virt[base + 94] = 0xC9C8C7C6;
	ast_jpeg->jpeg_tbl_virt[base + 95] = 0xD4D3D2CA;
	ast_jpeg->jpeg_tbl_virt[base + 96] = 0xD8D7D6D5;
	ast_jpeg->jpeg_tbl_virt[base + 97] = 0xE2E1DAD9;
	ast_jpeg->jpeg_tbl_virt[base + 98] = 0xE6E5E4E3;
	ast_jpeg->jpeg_tbl_virt[base + 99] = 0xEAE9E8E7;
	ast_jpeg->jpeg_tbl_virt[base + 100] = 0xF4F3F2F1;
	ast_jpeg->jpeg_tbl_virt[base + 101] = 0xF8F7F6F5;
	ast_jpeg->jpeg_tbl_virt[base + 102] = 0xC4FFFAF9;
	ast_jpeg->jpeg_tbl_virt[base + 103] = 0x00011F00;
	ast_jpeg->jpeg_tbl_virt[base + 104] = 0x01010103;
	ast_jpeg->jpeg_tbl_virt[base + 105] = 0x01010101;
	ast_jpeg->jpeg_tbl_virt[base + 106] = 0x00000101;
	ast_jpeg->jpeg_tbl_virt[base + 107] = 0x00000000;
	ast_jpeg->jpeg_tbl_virt[base + 108] = 0x04030201;
	ast_jpeg->jpeg_tbl_virt[base + 109] = 0x08070605;
	ast_jpeg->jpeg_tbl_virt[base + 110] = 0xFF0B0A09;
	ast_jpeg->jpeg_tbl_virt[base + 111] = 0x11B500C4;
	ast_jpeg->jpeg_tbl_virt[base + 112] = 0x02010200;
	ast_jpeg->jpeg_tbl_virt[base + 113] = 0x04030404;
	ast_jpeg->jpeg_tbl_virt[base + 114] = 0x04040507;
	ast_jpeg->jpeg_tbl_virt[base + 115] = 0x77020100;
	ast_jpeg->jpeg_tbl_virt[base + 116] = 0x03020100;
	ast_jpeg->jpeg_tbl_virt[base + 117] = 0x21050411;
	ast_jpeg->jpeg_tbl_virt[base + 118] = 0x41120631;
	ast_jpeg->jpeg_tbl_virt[base + 119] = 0x71610751;
	ast_jpeg->jpeg_tbl_virt[base + 120] = 0x81322213;
	ast_jpeg->jpeg_tbl_virt[base + 121] = 0x91421408;
	ast_jpeg->jpeg_tbl_virt[base + 122] = 0x09C1B1A1;
	ast_jpeg->jpeg_tbl_virt[base + 123] = 0xF0523323;
	ast_jpeg->jpeg_tbl_virt[base + 124] = 0xD1726215;
	ast_jpeg->jpeg_tbl_virt[base + 125] = 0x3424160A;
	ast_jpeg->jpeg_tbl_virt[base + 126] = 0x17F125E1;
	ast_jpeg->jpeg_tbl_virt[base + 127] = 0x261A1918;
	ast_jpeg->jpeg_tbl_virt[base + 128] = 0x2A292827;
	ast_jpeg->jpeg_tbl_virt[base + 129] = 0x38373635;
	ast_jpeg->jpeg_tbl_virt[base + 130] = 0x44433A39;
	ast_jpeg->jpeg_tbl_virt[base + 131] = 0x48474645;
	ast_jpeg->jpeg_tbl_virt[base + 132] = 0x54534A49;
	ast_jpeg->jpeg_tbl_virt[base + 133] = 0x58575655;
	ast_jpeg->jpeg_tbl_virt[base + 134] = 0x64635A59;
	ast_jpeg->jpeg_tbl_virt[base + 135] = 0x68676665;
	ast_jpeg->jpeg_tbl_virt[base + 136] = 0x74736A69;
	ast_jpeg->jpeg_tbl_virt[base + 137] = 0x78777675;
	ast_jpeg->jpeg_tbl_virt[base + 138] = 0x83827A79;
	ast_jpeg->jpeg_tbl_virt[base + 139] = 0x87868584;
	ast_jpeg->jpeg_tbl_virt[base + 140] = 0x928A8988;
	ast_jpeg->jpeg_tbl_virt[base + 141] = 0x96959493;
	ast_jpeg->jpeg_tbl_virt[base + 142] = 0x9A999897;
	ast_jpeg->jpeg_tbl_virt[base + 143] = 0xA5A4A3A2;
	ast_jpeg->jpeg_tbl_virt[base + 144] = 0xA9A8A7A6;
	ast_jpeg->jpeg_tbl_virt[base + 145] = 0xB4B3B2AA;
	ast_jpeg->jpeg_tbl_virt[base + 146] = 0xB8B7B6B5;
	ast_jpeg->jpeg_tbl_virt[base + 147] = 0xC3C2BAB9;
	ast_jpeg->jpeg_tbl_virt[base + 148] = 0xC7C6C5C4;
	ast_jpeg->jpeg_tbl_virt[base + 149] = 0xD2CAC9C8;
	ast_jpeg->jpeg_tbl_virt[base + 150] = 0xD6D5D4D3;
	ast_jpeg->jpeg_tbl_virt[base + 151] = 0xDAD9D8D7;
	ast_jpeg->jpeg_tbl_virt[base + 152] = 0xE5E4E3E2;
	ast_jpeg->jpeg_tbl_virt[base + 153] = 0xE9E8E7E6;
	ast_jpeg->jpeg_tbl_virt[base + 154] = 0xF4F3F2EA;
	ast_jpeg->jpeg_tbl_virt[base + 155] = 0xF8F7F6F5;
	ast_jpeg->jpeg_tbl_virt[base + 156] = 0xDAFFFAF9;
	ast_jpeg->jpeg_tbl_virt[base + 157] = 0x01030C00;
	ast_jpeg->jpeg_tbl_virt[base + 158] = 0x03110200;
	ast_jpeg->jpeg_tbl_virt[base + 159] = 0x003F0011;

	ast_jpeg->jpeg_tbl_virt[base + 10] = 0x01020043;
	ast_jpeg->jpeg_tbl_virt[base + 11] = 0x01010101;
	ast_jpeg->jpeg_tbl_virt[base + 12] = 0x01010102;
	ast_jpeg->jpeg_tbl_virt[base + 13] = 0x02020202;
	ast_jpeg->jpeg_tbl_virt[base + 14] = 0x03030503;
	ast_jpeg->jpeg_tbl_virt[base + 15] = 0x06030202;
	ast_jpeg->jpeg_tbl_virt[base + 16] = 0x05030404;
	ast_jpeg->jpeg_tbl_virt[base + 17] = 0x07070607;
	ast_jpeg->jpeg_tbl_virt[base + 18] = 0x06070607;
	ast_jpeg->jpeg_tbl_virt[base + 19] = 0x090B0908;
	ast_jpeg->jpeg_tbl_virt[base + 20] = 0x080A0808;
	ast_jpeg->jpeg_tbl_virt[base + 21] = 0x0D0A0706;
	ast_jpeg->jpeg_tbl_virt[base + 22] = 0x0C0B0A0A;
	ast_jpeg->jpeg_tbl_virt[base + 23] = 0x070C0D0C;
	ast_jpeg->jpeg_tbl_virt[base + 24] = 0x0E0F0E09;
	ast_jpeg->jpeg_tbl_virt[base + 25] = 0x0C0B0F0C;
	ast_jpeg->jpeg_tbl_virt[base + 26] = 0xDBFF0C0C;
	ast_jpeg->jpeg_tbl_virt[base + 27] = 0x03014300;
	ast_jpeg->jpeg_tbl_virt[base + 28] = 0x03040303;
	ast_jpeg->jpeg_tbl_virt[base + 29] = 0x04040804;
	ast_jpeg->jpeg_tbl_virt[base + 30] = 0x0A0C1208;
	ast_jpeg->jpeg_tbl_virt[base + 31] = 0x1212120C;
	ast_jpeg->jpeg_tbl_virt[base + 32] = 0x12121212;
	ast_jpeg->jpeg_tbl_virt[base + 33] = 0x12121212;
	ast_jpeg->jpeg_tbl_virt[base + 34] = 0x12121212;
	ast_jpeg->jpeg_tbl_virt[base + 35] = 0x12121212;
	ast_jpeg->jpeg_tbl_virt[base + 36] = 0x12121212;
	ast_jpeg->jpeg_tbl_virt[base + 37] = 0x12121212;
	ast_jpeg->jpeg_tbl_virt[base + 38] = 0x12121212;
	ast_jpeg->jpeg_tbl_virt[base + 39] = 0x12121212;
	ast_jpeg->jpeg_tbl_virt[base + 40] = 0x12121212;
	ast_jpeg->jpeg_tbl_virt[base + 41] = 0x12121212;
	ast_jpeg->jpeg_tbl_virt[base + 42] = 0x12121212;
	ast_jpeg->jpeg_tbl_virt[base + 43] = 0xFF121212;

	for(i = 0; i<12; i++) {
		base = (1024*i);
		ast_jpeg->jpeg_tbl_virt[base + 46] = 0x00220103; //for YUV420 mode
	}
	
}

static irqreturn_t ast_jpeg_isr(int this_irq, void *dev_id)
{
	u32 status;
	struct ast_jpeg_data *ast_jpeg = dev_id;

	status = ast_jpeg_read(ast_jpeg, AST_JPEG_ISR);

	JPEG_DBG("%x \n", status);

	if(status & JPEG_HANG_WDT_ISR) {
		printk("JPEG_HANG_WDT_ISR\n");
		ast_jpeg_write(ast_jpeg, JPEG_HANG_WDT_ISR, AST_JPEG_ISR);
		complete(&ast_jpeg->jpeg_complete);
	}
		
	if(status & JPEG_COMPLETE_ISR) {
		printk("JPEG_COMPLETE_ISR\n");
		ast_jpeg_write(ast_jpeg, JPEG_COMPLETE_ISR, AST_JPEG_ISR);
		complete(&ast_jpeg->jpeg_complete);
	}		

	if(status & JPEG_COMPRESS_COMPLETE) {
		printk("JPEG_COMPRESS_COMPLETE\n");
		ast_jpeg_write(ast_jpeg, JPEG_COMPRESS_COMPLETE, AST_JPEG_ISR);
		complete(&ast_jpeg->jpeg_complete);
	}		

	return IRQ_HANDLED;
}

static void ast_jpeg_config(struct ast_jpeg_data *ast_jpeg, struct ast_jpeg_mode *jpeg_mode)
{
	u32 scan_line;

	JPEG_DBG("x : %d, y : %d \n", jpeg_mode->x, jpeg_mode->y);
	ast_jpeg_write(ast_jpeg, JPEG_COMPRESS_H(jpeg_mode->x) | JPEG_COMPRESS_V(jpeg_mode->y), AST_JPEG_SIZE_SETTING);

	ast_jpeg_write(ast_jpeg, 0x80000, AST_JPEG_COMPRESS_CTRL);
	
	if(jpeg_mode->format) { //NV12
		ast_jpeg_write(ast_jpeg, jpeg_mode->x, AST_JPEG_SOURCE_SCAN_LINE);
		ast_jpeg_write(ast_jpeg, (ast_jpeg_read(ast_jpeg, AST_JPEG_SEQ_CTRL) & ~JPEG_COMPRESS_MODE_MASK) | 
						JPEG_NV12_COMPRESS, AST_JPEG_SEQ_CTRL);	
		ast_jpeg_write(ast_jpeg, (u32)ast_jpeg->buff0_phy + (jpeg_mode->x * jpeg_mode->y), AST_JPEG_SOURCE_BUFF1);	
	} else {
		ast_jpeg_write(ast_jpeg, jpeg_mode->x * 2, AST_JPEG_SOURCE_SCAN_LINE);	
		ast_jpeg_write(ast_jpeg, (ast_jpeg_read(ast_jpeg, AST_JPEG_SEQ_CTRL) & ~JPEG_COMPRESS_MODE_MASK) | 
						JPEG_YUY2_COMPRESS, AST_JPEG_SEQ_CTRL);
	}	
}

static u32 ast_jpeg_compression(struct ast_jpeg_data *ast_jpeg, unsigned long *jpeg_size)
{
	int timeout = 0;	

	JPEG_DBG("\n");

	init_completion(&ast_jpeg->jpeg_complete);

	ast_jpeg_write(ast_jpeg, ast_jpeg_read(ast_jpeg, AST_JPEG_SEQ_CTRL) & ~JPEG_COMPRESS_TRIGGER, AST_JPEG_SEQ_CTRL);
	//If CPU is too fast, pleas read back and trigger 
	ast_jpeg_write(ast_jpeg, ast_jpeg_read(ast_jpeg, AST_JPEG_SEQ_CTRL) | JPEG_COMPRESS_TRIGGER, AST_JPEG_SEQ_CTRL);
	
	timeout = wait_for_completion_interruptible_timeout(&ast_jpeg->jpeg_complete, HZ/2);
	
	if (timeout == 0) { 
//		printk("compression timeout sts %x \n", ast_jpeg_read(ast_jpeg, AST_JPEG_ISR));
		*jpeg_size = ast_jpeg_read(ast_jpeg, AST_JPEG_STREAM_SIZE);
//		printk("size %d \n", *jpeg_size);
		return 0;
	} else {
		*jpeg_size = ast_jpeg_read(ast_jpeg, AST_JPEG_STREAM_SIZE);
		JPEG_DBG("compress size %d \n",*jpeg_size);
		return 1;
	}	
}

static long ast_jpeg_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	int ret = 1;
	struct miscdevice *c = fp->private_data;
	struct ast_jpeg_data *ast_jpeg = dev_get_drvdata(c->this_device);
	
	struct ast_jpeg_mode mode;	
	unsigned long jpeg_size = 0;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
		case AST_JPEG_SET_CONFIG:
			ret = copy_from_user(&mode, argp, sizeof(struct ast_jpeg_mode));
			ast_jpeg_config(ast_jpeg, &mode);
			ret = copy_to_user(argp, &mode, sizeof(struct ast_jpeg_mode));			
			break;
		case AST_JPEG_GET_MEM_SIZE_IOCRX:
			ret = __put_user(ast_jpeg->jpeg_mem_size, (unsigned long __user *)arg);			
			break;
		case AST_JPEG_GET_BUFFER0_OFFSET_IOCRX:
			ret = __put_user(ast_jpeg->raw_buff0_offset, (unsigned long __user *)arg);
			break;
		case AST_JPEG_GET_BUFFER1_OFFSET_IOCRX:
			ret = __put_user(ast_jpeg->raw_buff1_offset, (unsigned long __user *)arg);
			break;
		case AST_JPEG_TRIGGER:
			ast_jpeg_compression(ast_jpeg, &jpeg_size);
//			 if(ast_jpeg_compression(ast_jpeg, &jpeg_size))
			 	ret = __put_user(jpeg_size, (unsigned long __user *)arg);
//			 else
//			 	ret = 3;
			break;
		default:
			ret = 3;
			break;
	}
	return ret;

}

/** @note munmap handler is done by vma close handler */
static int ast_jpeg_mmap(struct file * file, struct vm_area_struct * vma)
{
        struct miscdevice *c = file->private_data;
        struct ast_jpeg_data *ast_jpeg = dev_get_drvdata(c->this_device);
        size_t size = vma->vm_end - vma->vm_start;
        vma->vm_private_data = ast_jpeg;

        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

        if (io_remap_pfn_range(vma, vma->vm_start,
                        ((u32)ast_jpeg->jpeg_phy >> PAGE_SHIFT),
                        size,
                        vma->vm_page_prot)) {
                printk(KERN_ERR "remap_pfn_range faile at %s()\n", __func__);
                return -EAGAIN;
        }

        return 0;
	
}

static int ast_jpeg_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_jpeg_data *ast_jpeg = dev_get_drvdata(c->this_device);

	JPEG_DBG("\n");

	if(ast_jpeg->is_open == true)
		return -1;
	else
		ast_jpeg->is_open = true;
        return 0;
}

static int ast_jpeg_release(struct inode *inode, struct file *file)
{
        struct miscdevice *c = file->private_data;
        struct ast_jpeg_data *ast_jpeg = dev_get_drvdata(c->this_device);

        JPEG_DBG("\n");

        ast_jpeg->is_open = false;
        return 0;
}

static const struct file_operations ast_jpeg_fops = {
	.owner 			= THIS_MODULE,
	.llseek 			= no_llseek,
	.unlocked_ioctl 	= ast_jpeg_ioctl,
	.open 			= ast_jpeg_open,
	.release 			= ast_jpeg_release,
	.mmap 			= ast_jpeg_mmap,
};

struct miscdevice ast_jpeg_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-jpeg",
	.fops = &ast_jpeg_fops,
};

static void ast_jpeg_ctrl_init(struct ast_jpeg_data *ast_jpeg)
{
	JPEG_DBG("\n");

	ast_jpeg_write(ast_jpeg, (u32)ast_jpeg->jpeg_tbl_dma_addr, AST_JPEG_HEADER_BUFF);

	ast_jpeg_write(ast_jpeg, (u32)ast_jpeg->buff0_phy, AST_JPEG_SOURCE_BUFF0);
	ast_jpeg_write(ast_jpeg, (u32)ast_jpeg->buff1_phy, AST_JPEG_SOURCE_BUFF1);
	ast_jpeg_write(ast_jpeg, (u32)ast_jpeg->jpeg_phy, AST_JPEG_STREAM_BUFF);

	ast_jpeg_write(ast_jpeg, JPEG_COMPRESS_MODE_ENABLE, AST_JPEG_SEQ_CTRL);

	ast_jpeg_write(ast_jpeg, 0x0, AST_JPEG_EFFECT_CTRL);
	ast_jpeg_write(ast_jpeg, 0x05, AST_JPEG_QUANTIZ_TABLE);		//128x5 = 640

	//clr int sts
	ast_jpeg_write(ast_jpeg, 0xffffffff, AST_JPEG_ISR);

	ast_init_jpeg_table(ast_jpeg);

	//for CONFIG_QUANTIZATION_TABLE
	ast_jpeg_write(ast_jpeg, JPEG_QUANT_TABLE_LOAD | JPEG_VERTICAL_BORDER_MASK | JPEG_PROGRAM_QUANT_TABLE_EN, AST_JPEG_CTRL); //[18] Vr_TblBufEnable, [30] Vr_SRAMTblSel[0]
	
	ast_jpeg_write(ast_jpeg, 0x04ed0672, 0x400);
	ast_jpeg_write(ast_jpeg, 0x080006ca, 0x404);
	ast_jpeg_write(ast_jpeg, 0x0c3f0d9b, 0x408);
	ast_jpeg_write(ast_jpeg, 0x08000b89, 0x40C);
	ast_jpeg_write(ast_jpeg, 0x02aa03cd, 0x410);
	ast_jpeg_write(ast_jpeg, 0x05c503ac, 0x414);
	ast_jpeg_write(ast_jpeg, 0x08d404e8, 0x418);
	ast_jpeg_write(ast_jpeg, 0x0b890851, 0x41C);
	ast_jpeg_write(ast_jpeg, 0x02840409, 0x420);
	ast_jpeg_write(ast_jpeg, 0x03100299, 0x424);
	ast_jpeg_write(ast_jpeg, 0x04b00535, 0x428);
	ast_jpeg_write(ast_jpeg, 0x0c3f08d4, 0x42C);
	ast_jpeg_write(ast_jpeg, 0x0284041c, 0x430);
	ast_jpeg_write(ast_jpeg, 0x01f2022a, 0x434);
	ast_jpeg_write(ast_jpeg, 0x037903db, 0x438);
	ast_jpeg_write(ast_jpeg, 0x06ce04e8, 0x43C);
	ast_jpeg_write(ast_jpeg, 0x02770424, 0x440);
	ast_jpeg_write(ast_jpeg, 0x02000209, 0x444);
	ast_jpeg_write(ast_jpeg, 0x02730245, 0x448);
	ast_jpeg_write(ast_jpeg, 0x055503d8, 0x44C);
	ast_jpeg_write(ast_jpeg, 0x02820627, 0x450);
	ast_jpeg_write(ast_jpeg, 0x019101fe, 0x454);
	ast_jpeg_write(ast_jpeg, 0x023a01bb, 0x458);
	ast_jpeg_write(ast_jpeg, 0x04130219, 0x45C);
	ast_jpeg_write(ast_jpeg, 0x03a408ee, 0x460);
	ast_jpeg_write(ast_jpeg, 0x027702b0, 0x464);
	ast_jpeg_write(ast_jpeg, 0x02d40284, 0x468);
	ast_jpeg_write(ast_jpeg, 0x04ed030c, 0x46C);
	ast_jpeg_write(ast_jpeg, 0x08ee1184, 0x470);
	ast_jpeg_write(ast_jpeg, 0x067206b6, 0x474);
	ast_jpeg_write(ast_jpeg, 0x0657070c, 0x478);
	ast_jpeg_write(ast_jpeg, 0x084906f8, 0x47C);
	ast_jpeg_write(ast_jpeg, 0x01a40339, 0x480);
	ast_jpeg_write(ast_jpeg, 0x00e40122, 0x484);
	ast_jpeg_write(ast_jpeg, 0x031001b3, 0x488);
	ast_jpeg_write(ast_jpeg, 0x055503d8, 0x48C);
	ast_jpeg_write(ast_jpeg, 0x012f0253, 0x490);
	ast_jpeg_write(ast_jpeg, 0x00a400d1, 0x494);
	ast_jpeg_write(ast_jpeg, 0x023500d1, 0x498);
	ast_jpeg_write(ast_jpeg, 0x03d802c6, 0x49C);
	ast_jpeg_write(ast_jpeg, 0x01420277, 0x4a0);
	ast_jpeg_write(ast_jpeg, 0x00ae00de, 0x4a4);
	ast_jpeg_write(ast_jpeg, 0x00f00094, 0x4a8);
	ast_jpeg_write(ast_jpeg, 0x03100235, 0x4aC);
	ast_jpeg_write(ast_jpeg, 0x016602bd, 0x4b0);
	ast_jpeg_write(ast_jpeg, 0x00c200f6, 0x4b4);
	ast_jpeg_write(ast_jpeg, 0x009400a5, 0x4b8);
	ast_jpeg_write(ast_jpeg, 0x01b300d1, 0x4bC);
	ast_jpeg_write(ast_jpeg, 0x01a40339, 0x4c0);
	ast_jpeg_write(ast_jpeg, 0x00e40122, 0x4c4);
	ast_jpeg_write(ast_jpeg, 0x00ae00c2, 0x4c8);
	ast_jpeg_write(ast_jpeg, 0x00e400a4, 0x4cC);
	ast_jpeg_write(ast_jpeg, 0x0217041a, 0x4d0);
	ast_jpeg_write(ast_jpeg, 0x01220171, 0x4d4);
	ast_jpeg_write(ast_jpeg, 0x00de00f6, 0x4d8);
	ast_jpeg_write(ast_jpeg, 0x012200d1, 0x4dC);
	ast_jpeg_write(ast_jpeg, 0x030905f4, 0x4e0);
	ast_jpeg_write(ast_jpeg, 0x01a40217, 0x4e4);
	ast_jpeg_write(ast_jpeg, 0x01420166, 0x4e8);
	ast_jpeg_write(ast_jpeg, 0x01a4012f, 0x4eC);
	ast_jpeg_write(ast_jpeg, 0x05f40bad, 0x4f0);
	ast_jpeg_write(ast_jpeg, 0x0339041a, 0x4f4);
	ast_jpeg_write(ast_jpeg, 0x027702bd, 0x4f8);
	ast_jpeg_write(ast_jpeg, 0x03390253, 0x4fC);

	//enable ier 
	ast_jpeg_write(ast_jpeg, JPEG_COMPRESS_COMPLETE , AST_JPEG_IER);
}

/************************************************** SYS FS End ***********************************************************/
static int ast_jpeg_probe(struct platform_device *pdev)
{
	struct resource *res0, *res1;
	int ret=0;
	struct ast_jpeg_data *ast_jpeg;

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res0) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	if (!request_mem_region(res0->start, resource_size(res0), res0->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		ret = -ENXIO;
		goto out;
	}

	if(!(ast_jpeg = kzalloc(sizeof(struct ast_jpeg_data), GFP_KERNEL))) {
		return -ENOMEM;
		goto out;
        }
	
	ast_jpeg->reg_base = ioremap(res0->start, resource_size(res0));
	if (!ast_jpeg->reg_base) {
		ret = -EIO;
		goto out_region0;
	}
	
	res1 = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!res1)
		return -ENODEV;
	
	if (!request_mem_region(res1->start, resource_size(res1), res1->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		ret = -ENXIO;
		goto out_region0;
	}

	ast_jpeg->jpeg_tbl_virt = dma_alloc_coherent(NULL,
					 1024 * 4 * 12,
					 &ast_jpeg->jpeg_tbl_dma_addr, GFP_KERNEL);

	JPEG_DBG("JPEG TABLE DMA %x, virt = %x \n", ast_jpeg->jpeg_tbl_dma_addr, ast_jpeg->jpeg_tbl_virt);
	
	//Phy assign
	ast_jpeg->jpeg_mem_size = resource_size(res1);
	JPEG_DBG("jpeg_mem_size %d MB\n",ast_jpeg->jpeg_mem_size/1024/1024);

	//Total = 16 * 3 = 48MB

	//Dest JPEG 
	//4096 * 2048 / 2 = 4MB
	
	ast_jpeg->jpeg_phy = (phys_addr_t *) res1->start;		// 4M * 2 = 8MB

	//SRC raw x*y 
	//YUYV = 4096x2048x2 = 16MB, 
	//NV12 = Y:4096x2048x1 = 8MB, CbCr : 4096x2048x0.5 = 4MB
	
	ast_jpeg->buff0_phy = (phys_addr_t *) (res1->start + JPEG_RAW_BUFF0_OFFSET);  //24M : 16M 
	ast_jpeg->raw_buff0_offset = JPEG_RAW_BUFF0_OFFSET;
	ast_jpeg->buff1_phy = (phys_addr_t *) (res1->start + JPEG_RAW_BUFF1_OFFSET);  //24M + 16M : 16M 
	ast_jpeg->raw_buff1_offset = JPEG_RAW_BUFF1_OFFSET;

	JPEG_DBG("\n jpeg_phy: %x, buff0_phy: %x, buff1_phy:%x \n",
	        (u32)ast_jpeg->jpeg_phy, (u32)ast_jpeg->buff0_phy, (u32)ast_jpeg->buff1_phy);

	//virt assign
	ast_jpeg->jpeg_virt = ioremap(res1->start, resource_size(res1));
	if (!ast_jpeg->jpeg_virt) {
	        ret = -EIO;
	        goto out_region1;
	}

	ast_jpeg->buff0_virt = ast_jpeg->jpeg_virt + JPEG_RAW_BUFF0_OFFSET; //24M : size 10MB
	ast_jpeg->buff1_virt = ast_jpeg->jpeg_virt + JPEG_RAW_BUFF1_OFFSET; //34M : size 10MB

	JPEG_DBG("\n jpeg_virt: %x, buff0_virt: %x, buff1_virt:%x \n",
	        (u32)ast_jpeg->jpeg_virt, (u32)ast_jpeg->buff0_virt, (u32)ast_jpeg->buff1_virt);

	memset(ast_jpeg->jpeg_virt, 0, resource_size(res1));	

	ast_jpeg->irq = platform_get_irq(pdev, 0);
	if (ast_jpeg->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region1;
	}

	ret = misc_register(&ast_jpeg_misc);
	if (ret){		
		printk(KERN_ERR "JPEG : failed to request interrupt\n");
		goto out_irq;
	}

	platform_set_drvdata(pdev, ast_jpeg);

	dev_set_drvdata(ast_jpeg_misc.this_device, ast_jpeg);

	ast_jpeg_ctrl_init(ast_jpeg);

	ret = request_irq(ast_jpeg->irq, ast_jpeg_isr, IRQF_SHARED, "ast-jpeg", ast_jpeg);
	if (ret) {
		printk(KERN_INFO "JPEG: Failed request irq %d\n", ast_jpeg->irq);
		goto out_region1;
	}

	printk(KERN_INFO "ast_jpeg: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(ast_jpeg->irq, NULL);

out_region1:
	release_mem_region(res1->start, res1->end - res1->start + 1);	

out_region0:
	release_mem_region(res0->start, res0->end - res0->start + 1);
	
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;

}

static int ast_jpeg_remove(struct platform_device *pdev)
{
	struct resource *res0, *res1;
	struct ast_jpeg_data *ast_jpeg = platform_get_drvdata(pdev);
	JPEG_DBG("ast_jpeg_remove\n");

	misc_deregister(&ast_jpeg_misc);

	free_irq(ast_jpeg->irq, ast_jpeg);

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_jpeg->reg_base);

	release_mem_region(res0->start, res0->end - res0->start + 1);

	res1 = platform_get_resource(pdev, IORESOURCE_DMA, 0);

	iounmap(ast_jpeg->jpeg_virt);

	release_mem_region(res1->start, res1->end - res1->start + 1);

	return 0;	
}

#ifdef CONFIG_PM
static int 
ast_jpeg_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("ast_jpeg_suspend : TODO \n");
	return 0;
}

static int 
ast_jpeg_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define ast_jpeg_suspend        NULL
#define ast_jpeg_resume         NULL
#endif

static struct platform_driver ast_jpeg_driver = {
	.probe	= ast_jpeg_probe,
	.remove	= ast_jpeg_remove,
	.suspend	= ast_jpeg_suspend, /* optional but recommended */
	.resume	= ast_jpeg_resume,   /* optional but recommended */
	.driver = {
		.name = "ast-jpeg",
		.owner  = THIS_MODULE,
	},
};

static struct platform_device *ast_jpeg_device;

static int __init ast_jpeg_init(void)
{
	int ret;
	static const struct resource jpeg_resources[] = {
		[0] = {
			.start = AST_JPEG_BASE,
			.end = AST_JPEG_BASE + SZ_2K - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = IRQ_JPEG,
			.end = IRQ_JPEG,
			.flags = IORESOURCE_IRQ,
		},
		[2] = {
			.start = AST_JPEG_MEM,
			.end = AST_JPEG_MEM + AST_JPEG_MEM_SIZE - 1,
			.flags = IORESOURCE_DMA,
		},	
	};

	ret = platform_driver_register(&ast_jpeg_driver);

	ast_scu_init_jpeg(0);

	if (!ret) {
		ast_jpeg_device = platform_device_register_simple("ast-jpeg", 0,
								jpeg_resources, ARRAY_SIZE(jpeg_resources));
		if (IS_ERR(ast_jpeg_device)) {
			platform_driver_unregister(&ast_jpeg_driver);
			ret = PTR_ERR(ast_jpeg_device);
		}
	}

	return ret;
}

static void __exit ast_jpeg_exit(void)
{
	JPEG_DBG("ast_jpeg_exit \n");
	platform_device_unregister(ast_jpeg_device);
	platform_driver_unregister(&ast_jpeg_driver);
}

module_init(ast_jpeg_init);
module_exit(ast_jpeg_exit);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST JPEG Encoder Device Driver");
MODULE_LICENSE("GPL");
