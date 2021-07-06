/*
 * This file is part of the ASPEED Linux Device Driver for ASPEED Baseboard Management Controller.
 * Refer to the README file included with this package for driver version and adapter compatibility.
 *
 * Copyright (C) 2019-2021 ASPEED Technology Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it under the terms of version 2
 * of the GNU General Public License as published by the Free Software Foundation.
 * This program is distributed in the hope that it will be useful. ALL EXPRESS OR IMPLIED CONDITIONS,
 * REPRESENTATIONS AND WARRANTIES, INCLUDING ANY IMPLIED WARRANTY OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE, OR NON-INFRINGEMENT, ARE DISCLAIMED, EXCEPT TO THE EXTENT THAT SUCH DISCLAIMERS ARE HELD TO BE
 * LEGALLY INVALID. See the GNU General Public License for more details, a copy of which can be found in
 * the file COPYING included with this package.
 */

#ifndef __HARDWAREENGINES_H__
#define __HARDWAREENGINES_H__

#include <linux/semaphore.h>
#include "video_ioctl.h"

#define MAX_NUM_CONTEXT				(8)
#define MAX_NUM_MEM_TBL				(24)//each context has 3

#define MAX_DESC_SIZE 				(PAGE_SIZE) // (0x400)

#define ENGINE_TIMEOUT_IN_SECONDS 		(3)
#define TFE_TIMEOUT_IN_MS			(750)
#define DESCRIPTOR_SIZE				(16)
#define TILE_SIZE				(32)
#define MAX_LMEM_BUCKET_SIZE			(0x80)

#define EIGHT_BYTE_ALIGMENT_MASK		(0xfffffff7)
#define SIXTEEN_BYTE_ALIGMENT_MASK		(0x8)
#define TFCTL_DESCRIPTOR_IN_DDR_MASK		(0xffffff7f)
#define BSCMD_MASK				(0xffff0f37)

#define TEXT_MODE_BUFFER_ALIGNMENT		(16)
#define MODE_13_CHAR_WIDTH			(32)
#define BSE_MEMORY_ACCESS_MASK			(0x00ffffff)
#define MEM_TABLE_SIZE_INCR			(8)
#define MEMORY_TABLE_GROW_INCR			(8)

#define MAX_TEXT_DATA_SIZE			(8192)


//SCU
#define SCU000_Protection_Key_Register (0x000)
#define SCU040_Module_Reset_Control_Register_Set_1 (0x040)
#define SCU044_Module_Reset_Control_Clear_Register_1 (0x044)
#define SCU080_Clock_Stop_Control_Register_Set_1 (0x080)
#define SCU084_Clock_Stop_Control_Clear_Register (0x084)
#define SCU500_Hardware_Strap1_Register (0x500)
#define SCU418_Pin_Ctrl (0x418)
#define SCU0C0_Misc1_Ctrl (0x0C0)
#define SCU0D0_Misc3_Ctrl (0x0D0)
//SCU418
#define VGAVS_ENBL 			(1<<31)
#define VGAHS_ENBL 			(1<<30)
//SCU0C0
#define VGA_CRT_DISBL 		(1<<6)
//SCU0D0
#define PWR_OFF_VDAC 		(1<<3)

#define SCU_UNLOCK_PWD									(0x1688A8A8)
#define SCU_RVAS_ENGINE_BIT							BIT(9)
#define SCU_RVAS_STOP_CLOCK_BIT						BIT(25)


//DP
#define DPTX_Configuration_Register			(0x100)
#define DPTX_PHY_Configuration_Register		(0x104)
//DPTX100
#define AUX_RESETN							(24)
//DPTX104
#define DP_TX_I_MAIN_ON						(8)

//TOP REG
#define TOP_REG_OFFSET				(0x0)
#define TOP_REG_CTL				(TOP_REG_OFFSET + 0x00)
#define TOP_REG_STS				(TOP_REG_OFFSET + 0x04)
#define LMEM_BASE_REG_3				(TOP_REG_OFFSET + 0x2c)
#define LMEM_LIMIT_REG_3			(TOP_REG_OFFSET + 0x3c)
#define LMEM11_P0				(TOP_REG_OFFSET + 0x4c)
#define LMEM12_P0				(TOP_REG_OFFSET + 0x50)
#define LMEM10_P1				(TOP_REG_OFFSET + 0x80)
#define LMEM11_P1				(TOP_REG_OFFSET + 0x84)
#define LMEM10_P2				(TOP_REG_OFFSET + 0xA0)
#define LMEM11_P2				(TOP_REG_OFFSET + 0xA4)

#define TSE_SnoopCommand_Register_Offset	(0x0400)
#define TSE_TileCount_Register_Offset		(0x0418)
#define TSE_Status_Register_Offset		(0x0404)
#define TSE_CS0Reg				(0x0408)
#define TSE_CS1Reg				(0x040c)
#define TSE_RS0Reg				(0x0410)
#define TSE_RS1Reg				(0x0414)
#define TSE_TileSnoop_Interrupt_Count		(0x0420)
#define TSE_FrameBuffer_Offset			(0x041c)
#define TSE_UpperLimit_Offset		(0x0424)
#define TSE_SnoopMap_Offset			(0x0600)


#define TFE_Descriptor_Table_Offset		(0x0108)
#define TFE_Descriptor_Control_Resgister	(0x0100)
#define TFE_Status_Register			(0x0104)
#define TFE_RLE_CheckSum			(0x010C)
#define TFE_RLE_Byte_Count			(0x0110)
#define TFE_RLE_LIMITOR				(0x0114)

#define BSE_REG_BASE				(0x0200)
#define BSE_Command_Register			(0x0200)
#define BSE_Status_Register			(0x0204)
#define BSE_Descriptor_Table_Base_Register	(0x0208)
#define BSE_Destination_Buket_Size_Resgister	(0x020c)
#define BSE_Bit_Position_Register_0		(0x0210)
#define BSE_Bit_Position_Register_1		(0x0214)
#define BSE_Bit_Position_Register_2		(0x0218)
#define BSE_LMEM_Temp_Buffer_Offset		(0x0000)
#define BSE_ENABLE_MULT_BUCKET_SZS		(1<<12)
#define BSE_BUCK_SZ_INDEX_POS			(4)
#define BSE_MAX_BUCKET_SIZE_REGS		(16)
#define BSE_BIT_MASK_Register_Offset		(0x54)

#define LDMA_Control_Register			(0x0300)
#define LDMA_Status_Register			(0x0304)
#define LDMA_Descriptor_Table_Base_Register	(0x0308)
#define LDMA_CheckSum_Register			(0x030c)
#define LDMA_LMEM_Descriptor_Offset		(0x4000)

//Shadow
#define GRCE_SIZE				(0x800)
#define GRCE_ATTR_OFFSET			(0x0)
#define GRCE_ATTR_VGAIR0_OFFSET	(0x18)
#define GRCE_SEQ_OFFSET				(0x20)
#define GRCE_GCTL_OFFSET			(0x30)
#define GRCE_GRCCTL0_OFFSET			(0x58)
#define GRCE_GRCSTS_OFFSET			(0x5c)
#define GRCE_CRTC_OFFSET			(0x60)
#define GRCE_CRTCEXT_OFFSET			(0x80)
#define GRCE_XCURCTL_OFFSET			(0xc8)
#define GRCE_PAL_OFFSET				(0x400)
//size
#define GRCELT_RAM_SIZE				(0x400)
#define GRCE_XCURCOL_SIZE			(0x40)
#define GRCE_XCURCTL_SIZE			(0x40)
#define GRCE_CRTC_SIZE				(0x40)
#define GRCE_CRTCEXT_SIZE			(0x8)
#define GRCE_SEQ_SIZE				(0x8)
#define GRCE_GCTL_SIZE				(0x8)
#define GRCE_ATTR_SIZE				(0x20)

#define GRCELT_RAM				(GRCE_PAL_OFFSET)
#define GRCE_XCURCTL				(GRCE_XCURCTL_OFFSET)
#define GRCE_CRTC				(GRCE_CRTC_OFFSET)
#define GRCE_CRTCEXT				(GRCE_CRTCEXT_OFFSET)
#define GRCE_SEQ				(GRCE_SEQ_OFFSET)
#define GRCE_GCTL				(GRCE_GCTL_OFFSET)
#define GRCE_CTL0				(GRCE_GRCCTL0_OFFSET)
#define GRCE_STATUS_REGISTER			(GRCE_GRCSTS_OFFSET)
#define GRCE_ATTR				(GRCE_ATTR_OFFSET)
#define AST_VIDEO_SCRATCH_34C			(0x8c)
#define AST_VIDEO_SCRATCH_350			(0x90)
#define AST_VIDEO_SCRATCH_354			(0x94)
#define MODE_GET_INFO_DE			(0xA8)

//GRC interrupt
#define GRC_FIQ_MASK				(0x000003ff)
#define GRC_IRQ_MASK				(0x000003ff)
#define GRC_INT_STS_MASK			(0x000003ff)
#define GRCSTS_XCUR_POS  			(1<<9)
#define GRCSTS_XCUR_DDR  			(1<<8)
#define GRCSTS_XCUR_CTL  			(1<<7)
#define GRCSTS_PLT_RAM   			(1<<6)
#define GRCSTS_XCRTC     			(1<<5)
#define GRCSTS_CRTC  	 			(1<<4)
#define GRCSTS_GCTL      			(1<<3)
#define GRCSTS_SEQ       			(1<<2)
#define GRCSTS_ATTR1     			(1<<1)
#define GRCSTS_ATTR0     			(1<<0)
#define SNOOP_RESTART (GRCSTS_XCUR_CTL|GRCSTS_XCRTC|GRCSTS_CRTC|GRCSTS_GCTL)

//snoop TSE
#define SNOOP_TSE_MASK				(0x00000001)
#define SNOOP_IRQ_MASK				(0x00000100)
#define SNOOP_FIQ_MASK				(0x00000200)
#define	TSCMD_SCREEN_OWNER			(1<<15)
#define TSCMD_PITCH_BIT				(16)
#define TSCMD_INT_ENBL_BIT			(8)
#define TSCMD_CPT_BIT				(6)
#define TSCMD_RPT_BIT				(4)
#define TSCMD_BPP_BIT				(2)
#define TSCMD_VGA_MODE_BIT			(1)
#define TSCMD_TSE_ENBL_BIT			(0)
#define TSSTS_FIFO_OVFL				(1<<5)
#define TSSTS_FONT 					(1<<4)
#define TSSTS_ATTR 					(1<<3)
#define TSSTS_ASCII 				(1<<2)
#define TSSTS_TC_SCREEN1 			(1<<1)
#define TSSTS_TC_SCREEN0 			(1<<0)
#define TSSTS_ALL					(0x3f)



#define TSE_INTR_COUNT				(0xCB700) 	//50MHz clock ~1/60 sec
//#define TSE_INTR_COUNT			(0x196E00) 	//50MHz clock ~1/30 sec
#define TIMER_INTR_COUNT			(0x65000) 	// 25MHz clock ~1/60 sec

//Timer
/* Register byte offsets */
// AST2600 Timer registers
#define TIMER_STATUS_BIT( x )       (1 << ((x) - 1))

#define OFFSET_TIMER1         0x00                      /* * timer 1 offset */
#define OFFSET_TIMER2         0x10                      /* * timer 2 offset */
#define OFFSET_TIMER3         0x20                      /* * timer 3 offset */
#define OFFSET_TIMER4         0x40                      /* * timer 4 offset */
#define OFFSET_TIMER5         0x50                      /* * timer 5 offset */
#define OFFSET_TIMER6         0x60                      /* * timer 6 offset */
#define OFFSET_TIMER7         0x70                      /* * timer 7 offset */
#define OFFSET_TIMER8         0x80                      /* * timer 8 offset */

#define OFF_TIMER_REG_CURR_CNT   0x00
#define OFF_TIMER_REG_LOAD_CNT   0x04
#define OFF_TIMER_REG_EO0        0x08                    /* Read to clear interrupt */
#define OFF_TIMER_REG_EOI        0x0c                    /* Read to clear interrupt */
#define OFF_TIMER_REG_STAT       0x10                    /* Timer Interrupt Status */
#define OFF_TIMER_REG_CONTROL    0x30							/* Control Register */
#define OFF_TIMER_REG_STATUS     0x34							/* Status Register */
#define OFF_TIMER_REG_CLEAR_CONTROL    0x3C							/* Control Register */
#define RB_OFF_TIMERS_STAT       0xA0                    /* * timers status offset */

#define CTRL_TIMER1           (0)
#define CTRL_TIMER2           (4)
#define CTRL_TIMER3           (8)
#define CTRL_TIMER4           (12)
#define CTRL_TIMER5           (16)
#define CTRL_TIMER6           (20)
#define CTRL_TIMER7           (24)
#define CTRL_TIMER8           (28)
#define BIT_TIMER_ENBL           (1 << 0)
#define BIT_TIMER_CLK_SEL        (1 << 1)
#define BIT_INTERRUPT_ENBL       (1 << 2)
#define BIT_TIMER_STAT           (1 << 0)

#define SNOOP_MAP_QWORD_COUNT			(64)
#define BSE_UPPER_LIMIT				(0x900000) //(0x540000)
#define FULL_BUCKETS_COUNT			(16)
#define MODE13_HEIGHT				(200)
#define MODE13_WIDTH				(320)

#define NUM_SNOOP_ROWS				(64)

//vga memory information
#define SCU500						(0x500)
#define DDR_SIZE_CONFIG_BITS    	(0x3)
#define VGA_MEM_SIZE_CONFIG_BITS 	(0x3)
#define VGA_MEM_SIZE_CONFIG_BIT_POS (13)
#define DDR_BASE					(0x80000000)


//grce
#define VGACR0_REG					(0x60)
#define VGACR9F_REG					(0x9F)


typedef struct tagContextTable {
	struct inode* pin;
	struct file* pf;
	SnoopAggregate sa;
	u64 aqwSnoopMap[NUM_SNOOP_ROWS];
	RVASContext rc;
	EventMap emEventWaiting;
	EventMap emEventReceived;
	u32 dwEventWaitInMs;
	void *desc_virt;
	u32 desc_phy;
} ContextTable;

typedef struct tagMemoryMapTable {
	struct file* pf;
	void *pvVirtualAddr;
	u32 dwPhysicalAddr;
	u32 dwLength;
	u8 byDmaAlloc;
	u8 byReserved[3];
} MemoryMapTable;

typedef union tagEmDwordUnion {
	EventMap em;
	u32 dw;
} EmDwordUnion;

typedef struct tagDescriptor {
	u32 dw0General;
	u32 dw1FetchWidthLine;
	u32 dw2SourceAddr;
	u32 dw3DestinationAddr;
} Descriptor;

typedef struct tagBSEAggregateRegister {
	u32 dwBSCR;
	u32 dwBSDBS;
	u32 adwBSBPS[3];
} BSEAggregateRegister;

typedef enum tagSkipByteMode {
	NoByteSkip = 0, SkipOneByte = 1, SkipTwoByte = 2, SkipThreeByte = 3
} SkipByteMode;

typedef enum tagStartBytePosition {
	StartFromByte0 = 0,
	StartFromByte1 = 1,
	StartFromByte2 = 2,
	StartFromByte3 = 3
} StartBytePosition;

typedef struct tagVGAMemInfo {
	u32 dwVGASize;
	u32 dwFBPhysStart;
} VGAMemInfo;

typedef struct tagVideoDataBufferInfo {
	u32 dwSize;
	u32 dwPhys;
	u32 dwVirt;
} VideoDataBufferInfo;

typedef enum tagColorMode {
	MODE_EGA = 0x0, //4bpp eg. mode 12/6A
	MODE_VGA = 0x1, //mode 13
	MODE_BPP15 = 0x2,
	MODE_BPP16 = 0x3,
	MODE_BPP32 = 0x4,
	MODE_TEXT = 0xE,
	MODE_CGA = 0xF
} ColorMode;

typedef struct tagModeInfo {
	u8 byColorMode;
	u8 byRefreshRateIndex;
	u8 byModeID;
	u8 byScanLines;
} ModeInfo;

typedef struct tagNewModeInfoHeader {
	u8 byReserved;
	u8 byDisplayInfo;
	u8 byColorDepth;
	u8 byMhzPixelClock;
} NewModeInfoHeader;

typedef struct tagDisplayEnd {
	u16 HDE;
	u16 VDE;
} DisplayEnd;

typedef struct tagResolution {
	u16 wWidth;
	u16 wHeight;
} Resolution;

typedef struct tagVideo_OsSleepStruct {
	wait_queue_head_t queue;
	struct timer_list tim;
	volatile u8 Timeout;
} Video_OsSleepStruct;

typedef struct tagEngineInfo {
	struct semaphore sem;
	Video_OsSleepStruct wait;
	u8 finished;
} EngineInfo;


typedef struct tagAstRVAS {
	struct miscdevice *rvas_dev;
	void *pdev;
	int irq_fge;	//FrameGrabber IRQ number
	int irq_vga; // VGA IRQ number
	int irq_video;
	u32 fg_reg_base;
	u32 grce_reg_base;
	u32 video_reg_base;
	struct regmap *scu;
   struct reset_control *rvas_reset;
   struct reset_control *video_engine_reset;
	VGAMemInfo FBInfo;
	u64 accrued_sm[SNOOP_MAP_QWORD_COUNT];
	SnoopAggregate accrued_sa;
	VideoGeometry current_vg;
	u32 snoop_stride;
	u32 tse_tsicr;
	EngineInfo tfe_engine;
	EngineInfo bse_engine;
	EngineInfo ldma_engine;
	EngineInfo video_engine;
	struct semaphore mem_sem;
	struct semaphore context_sem;
	Video_OsSleepStruct video_wait;
	u8 video_intr_occured;
	u8 timer_irq_requested;
	u8 reserved[2];
	ContextTable *ppctContextTable[MAX_NUM_CONTEXT];
	u32 dwMemoryTableSize;
	u32 dwScreenOffset;
	MemoryMapTable *ppmmtMemoryTable[MAX_NUM_MEM_TBL];
	struct completion  video_compression_complete;
	struct completion  video_capture_complete;
	struct clk 			*vclk;
	struct clk 			*eclk;
	struct clk			*rvasclk;
} AstRVAS;

//
// IOCTL function
//
void ioctl_get_video_geometry(RvasIoctl *ri, AstRVAS *ast_rvas);
void ioctl_wait_for_video_event(RvasIoctl *ri, AstRVAS *ast_rvas);
void ioctl_get_grc_register(RvasIoctl *ri, AstRVAS *ast_rvas);
void ioctl_read_snoop_map(RvasIoctl *ri, AstRVAS *ast_rvas);
void ioctl_read_snoop_aggregate(RvasIoctl *ri, AstRVAS *ast_rvas);
void ioctl_set_tse_tsicr(RvasIoctl *ri, AstRVAS *ast_rvas);
void ioctl_get_tse_tsicr(RvasIoctl *ri, AstRVAS *ast_rvas);
void ioctl_reset_video_engine(RvasIoctl *ri, AstRVAS *ast_rvas);


void ioctl_fetch_video_tiles(RvasIoctl *ri, AstRVAS *ast_rvas);
void ioctl_fetch_video_slices(RvasIoctl *ri, AstRVAS *ast_rvas);
void ioctl_run_length_encode_data(RvasIoctl *ri, AstRVAS *ast_rvas);
void ioctl_fetch_text_data(RvasIoctl *ri, AstRVAS *ast_rvas);
void ioctl_fetch_mode_13_data(RvasIoctl *ri, AstRVAS *ast_rvas);
u32 get_phy_fb_start_address(AstRVAS *ast_rvas);
bool video_geometry_change(AstRVAS *ast_rvas, u32 dwGRCEStatus);
void update_video_geometry(AstRVAS *ast_rvas);
//interrupts
void enable_grce_tse_interrupt(AstRVAS *ast_rvas);
void disable_grce_tse_interrupt(AstRVAS *ast_rvas);
u32 clear_tse_interrupt(AstRVAS *ast_rvas);
bool clear_ldma_interrupt(AstRVAS *ast_rvas);
bool clear_tfe_interrupt(AstRVAS *ast_rvas);
bool clear_bse_interrupt(AstRVAS *ast_rvas);
u32 get_screen_offset(AstRVAS *ast_rvas);
//
void setup_lmem(AstRVAS *ast_rvas);
//
// helper functions
//

BSEAggregateRegister setUp_bse_bucket(u8* abyBitIndexes, u8 byTotalBucketCount,
        u8 byBSBytesPerPixel, u32 dwFetchWidthPixels, u32 dwFetchHeight);
void prepare_bse_descriptor(Descriptor* pDAddress, u32 dwSourceAddress,
        u32 dwDestAddress, bool bNotLastEntry, u16 wStride, u8 bytesPerPixel,
        u32 dwFetchWidthPixels, u32 dwFetchHeight,
        bool bInterrupt);

void prepare_tfe_descriptor(Descriptor* pDAddress, u32 dwSourceAddress,
        u32 dwDestAddress, bool bNotLastEntry, u8 bCheckSum,
        bool bEnabledRLE, u16 wStride, u8 bytesPerPixel, u32 dwFetchWidthPixels,
        u32 dwFetchHeight, SelectedByteMode sbm,
        bool bRLEOverFLow, bool bInterrupt);
void prepare_tfe_text_descriptor(Descriptor* pDAddress, u32 dwSourceAddress,
        u32 dwDestAddress, bool bEnabledRLE, u32 dwFetchWidth,
        u32 dwFetchHeight, DataProccessMode dpm, bool bRLEOverFLow,
        bool bInterrupt);
void prepare_ldma_descriptor(Descriptor* pDAddress, u32 dwSourceAddress,
        u32 dwDestAddress, u32 dwLDMASize, u8 byNotLastEntry);

void OnFetchVideoTileChaining(RvasIoctl* ri);
void OnFetchVideoTileNoChainingWithRLE(RvasIoctl* ri);
void WaitWhileEngineBusy(u32 theAddress);
u8 get_text_mode_character_per_line(AstRVAS *ast_rvas, u16 wScreenWidth);
u16 get_text_mode_fetch_lines(AstRVAS *ast_rvas, u16 wScreenHeight);
void on_fetch_text_data(RvasIoctl* ri, bool bRLEOn, AstRVAS *ast_rvas);

void reset_snoop_engine(AstRVAS *ast_rvas);
void set_snoop_engine(bool b_geom_chg, AstRVAS *ast_rvas);
u64 reinterpret_32bpp_snoop_row_as_24bpp(u64);
void Convert4BppMapTo3Bpp(u64*, u32);
void convert_snoop_map(AstRVAS *ast_rvas);
void update_all_snoop_context(AstRVAS *ast_rvas);
void get_snoop_map_data(AstRVAS *ast_rvas);
void get_snoop_aggregate(AstRVAS *ast_rvas);

void sleep_on_ldma_busy(AstRVAS *ast_rvas, u32 dwAddress);
bool sleep_on_tfe_busy(AstRVAS *ast_rvas, u32 dwTFEDescriptorAddr,
        u32 dwTFEControlR, u32 dwTFERleLimitor, u32 *pdwRLESize,
        u32 *pdwCheckSum);

bool sleep_on_tfe_text_busy(AstRVAS *ast_rvas, u32 dwTFEDescriptorAddr,
        u32 dwTFEControlR, u32 dwTFERleLimitor, u32 *pdwRLESize,
        u32 *pdwCheckSum);

bool sleep_on_bse_busy(AstRVAS *ast_rvas, u32 dwBSEDescriptorAddr,
        BSEAggregateRegister aBSEAR, u32 size);

void enable_grce_tse_interrupt(AstRVAS *ast_rvas);
void disable_grce_tse_interrupt(AstRVAS *ast_rvas);

void disable_interrupts(AstRVAS*ast_rvas);
void enable_interrupts(AstRVAS*ast_rvas);

bool host_suspended(AstRVAS *pAstRVAS);
#endif // __HARDWAREENGINES_H__
