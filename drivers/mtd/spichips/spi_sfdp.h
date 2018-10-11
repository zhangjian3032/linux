/*******************************************************************************
 *
 * Copyright (C) 2004-2015 Emulex. All rights reserved.
 * EMULEX is a trademark of Emulex.
 * www.emulex.com
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of version 2 of the GNU General Public License as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful. ALL EXPRESS
 * OR IMPLIED CONDITIONS, REPRESENTATIONS AND WARRANTIES, INCLUDING ANY IMPLIED
 * WARRANTY OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
 * NON-INFRINGEMENT, ARE DISCLAIMED, EXCEPT TO THE EXTENT THAT SUCH DISCLAIMERS
 * ARE HELD TO BE LEGALLY INVALID. See the GNU General Public License for more
 * details, a copy of which can be found in the file COPYING included
 * with this package.
 *
 ********************************************************************************/


#ifndef _SPI_SFDP_H
#define _SPI_SFDP_H

// SFDP table structure
// Though 16 Dwords are declared for "basic parameter table" based on JESD216B
// the spi driver is compliant to JESD216 only which declares only 9 Dwords
struct SFDPHeader
{
	union {
		struct {
			unsigned char byte0;					// byte 0 for signature
			unsigned char byte1;					// byte 1 for signature
			unsigned char byte2;					// byte 2 for signature
			unsigned char byte3;					// byte 3 for signature
		} __attribute__((packed)) sig;
		unsigned long signature;					// signature
	} __attribute__((packed));
	unsigned long minor_rev: 8;						// minor revision
	unsigned long major_rev: 8;						// major revision
	unsigned long no_of_headers: 8;					// no of headers, 0's based
	unsigned long unused: 8;						// unused
} __attribute__((packed));

struct ParameterHeader
{
	unsigned long pid_lsb: 8;						// parameter id LSB
	unsigned long pminor_rev: 8;					// parameter minor revision
	unsigned long pmajor_rev: 8;					// parameter major revision
	unsigned long plength: 8;						// parameter length table, 1's based
	unsigned long ptable_pointer: 24;				// parameter table pointer
	unsigned long pid_msb: 8;						// parameter id MSB
} __attribute__((packed));

struct BFPTable_DWORD1
{
	unsigned long erase_size: 2;					// block/sector erase size
	unsigned long write_granularity: 1;				// write granularity
	volatile unsigned long status_reg_protect: 1;	// volatile status register block protect bits
	volatile unsigned long status_reg_eanble: 1;	// writing to volatile status register if it is protected
	unsigned long unused: 3;						// unused
	unsigned long erase_inst: 8;					// 4KB erase instruction
	unsigned long fast_read_112: 1;					// fast read (1-1-2)
	unsigned long address_bytes: 2;					// address bytes
	unsigned long double_transfer_rate: 1;			// double transfer support
	unsigned long fast_read_122: 1;					// fast read (1-2-2) support
	unsigned long fast_read_144: 1;					// fast read (1-4-4) support
	unsigned long fast_read_114: 1;					// fast read (1-1-4) support
	unsigned long unused2: 2;						// always FFh
} __attribute__((packed));

struct BFPTable_DWORD2
{
	unsigned long flash_mem_density;				// flash memory density
} __attribute__((packed));

struct BFPTable_DWORD3
{
	unsigned long dummy_fast_read_144: 5;			// no of dummy cycles needed for fast read (1-1-4)
	unsigned long mode_clock_fast_read_144: 3;		// fast read (1-4-4) no of mode clocks
	unsigned long ins_fast_read_144: 8;				// fast read (1-4-4) instruction
	unsigned long dummy_fast_read_114: 5;			// no of dummy cycles needed for fast read (1-1-4)
	unsigned long mode_clock_fast_read_114: 3;		// fast read (1-1-4) no of mode clock
	unsigned long ins_fast_read_114: 8;				// fast read (1-1-4) instruction
} __attribute__((packed));

struct BFPTable_DWORD4
{
	unsigned long dummy_fast_read_112: 5;			// dummy cycles for fast read (1-1-2)
	unsigned long mode_clock_fast_read_112: 3;		// fast read (1-1-2) mode clock
	unsigned long ins_fast_read_112: 8;				// fast read (1-1-2) instruction
	unsigned long dummy_fast_read_122: 5;			// dummy cycles for fast read (1-2-2)
	unsigned long mode_clock_fast_read_122: 3;		// fast read (1-2-2) mode clock
	unsigned long ins_fast_read_122: 8;				// fast read (1-2-2) instruction
} __attribute__((packed));

struct BFPTable_DWORD5
{
	unsigned long fast_read_222: 1;					// fast read (2-2-2) suppport
	unsigned long reserverd: 3;						// reserved, all 1's
	unsigned long fast_read_444: 1;					// fast read (4-4-4) support
	unsigned long reserved2: 27;					// reserved, all 1's
} __attribute__((packed));

struct BFPTable_DWORD6
{
	unsigned long reserved: 16;						// reserved, all 1's
	unsigned long dummy_fast_read_222: 5;			// dummy cycles for fast read (2-2-2)
	unsigned long mode_clock_fast_read_222: 3;		// fast read (2-2-2) mode clock
	unsigned long ins_fast_read_222: 8;				// fast read (2-2-2) instruction
} __attribute__((packed));

struct BFPTable_DWORD7
{
	unsigned long reserved: 16;						// reserved, all 1's
	unsigned long dummy_fast_read_444: 5;			// dummy cycles for fast read (4-4-4)
	unsigned long mode_clock_fast_read_444: 3;		// fast read (4-4-4) mode clock
	unsigned long ins_fast_read_444: 8;				// fast read (4-4-4) instruction
} __attribute__((packed));

struct BFPTable_DWORD8
{
	unsigned long erase_type1_size: 8;				// erase type 1 size
	unsigned long erase_type1_ins: 8;				// erase type 1 instruction
	unsigned long erase_type2_size: 8;				// erase type 2 size
	unsigned long erase_type2_ins: 8;				// erase type 2 instruction
} __attribute__((packed));

struct BFPTable_DWORD9
{
	unsigned long erase_type3_size: 8;				// erase type 3 size
	unsigned long erase_type3_ins: 8;				// erase type 3 instruction
	unsigned long erase_type4_size: 8;				// erase type 4 size
	unsigned long erase_type4_ins: 8;				// erase type 4 instruction
} __attribute__((packed));;

struct BFPTable_DWORD10
{
	unsigned long mult: 4;							// multiplier from typical erase time, 0's based
	unsigned long time_erase1: 7;					// typical erase 1 time
	unsigned long time_erase2: 7;					// typical erase 2 time
	unsigned long time_erase3: 7;					// typical erase 3 time
	unsigned long time_erase4: 7;					// typical erase 4 time
} __attribute__((packed));

struct BFPTable_DWORD11
{
	unsigned long mult: 4;							// multiplier from typical time to max time for page or byte program
	unsigned long page_size: 4;						// page size
	unsigned long typical_page_prog_time: 6;		// page program typical time
	unsigned long typical_byte_prog_time1: 5;		// byte program typical time, first byte
	unsigned long typical_byte_prog_time2: 5;		// byte program typical time, additional byte
	unsigned long typical_chip_erase_time: 7;		// chip erase typical time
	unsigned long reserved: 1;						// reserved
} __attribute__((packed));

struct BFPTable_DWORD12
{
	unsigned long suspend_prohibit: 4;				// prohibited operations during program suspend
	unsigned long erase_prohibit: 4;				// prohibited opeartions during erase suspend
	unsigned long reserved: 1;						// reserved
	unsigned long prog_resume_suspend_latency: 4;	// program resume to suspend latency
	unsigned long suspend_prog_max_latency: 7;		// suspend in progress program max latency
	unsigned long erase_resume_suspend_latency: 4;	// erase resume to suspend latency
	unsigned long suspend_erase_max_latency: 7;		// suspend in progress
	unsigned long suspend_resume: 1;				// suspend resume supported, 0 supported
} __attribute__((packed));

struct BFPTable_DWORD13
{
	unsigned long prog_resume_ins: 8;				// program resume instruction
	unsigned long prog_suspend_ins: 8;				// program suspend instruction
	unsigned long resume_ins: 8;					// program resume instruction
	unsigned long suspend_ins: 8;					// program suspend instruction
} __attribute__((packed));

struct BFPTable_DWORD14
{
	unsigned long reserved: 2;						// reserved
	unsigned long status_register: 6;				// status register polling device busy
	unsigned long exit_deep_next_op_delay: 7;		// exit deep powerdown to next operation delay
	unsigned long exit_deep_ins: 8;					// exit deep powerdown instruction
	unsigned long enter_deep_ins: 8;				// enter deep powerdown instruction
	unsigned long deep_powerdown_support: 1;		// support for deep powerdown
} __attribute__((packed));

struct BFPTable_DWORD15
{
	unsigned long disable_mode_444:	4;				// (4-4-4) mode disable sequance
	unsigned long enable_mode_444: 5;				// (4-4-4) mode enable seqance
	unsigned long mode_support_044: 1;				// (0-4-4) mode supported
	unsigned long exit_mode_044:	6;				// (0-4-4) mode exit method
	unsigned long entry_mode_044:	4;				// (0-4-4) mode entry method
	unsigned long quad_enable_requirement: 3;		// Quad enable requirement
	unsigned long hold_reset_disable: 1;			// hold or reset disable
	unsigned long reseerved: 8;						// reserved
} __attribute__((packed));

struct BFPTable_DWORD16
{
	unsigned long write_enable_status_reg1: 7;		// volatile or non-volatile and write enable instruction for status register 1
	unsigned long reserved: 1;						// reserved
	unsigned long soft_reset_rescue: 6;				// soft reset and resue sequance support
	unsigned long exit_4byte_addressing: 10;		// exit 4 byte addressing
	unsigned long enter_4byte_addressing: 8;		// enter 4 byte addressing
} __attribute__((packed));

struct Basic_Flash_Parameter
{
	// Corresponding location of DWORD should not be changed
	struct BFPTable_DWORD1 dword1;					// Parameter Header 1 DWORD 1
	struct BFPTable_DWORD2 dword2;					// Parameter Header 1 DWORD 2
	struct BFPTable_DWORD3 dword3;					// Parameter Header 1 DWORD 3
	struct BFPTable_DWORD4 dword4;					// Parameter Header 1 DWORD 4
	struct BFPTable_DWORD5 dword5;					// Parameter Header 1 DWORD 5
	struct BFPTable_DWORD6 dword6;					// Parameter Header 1 DWORD 6
	struct BFPTable_DWORD7 dword7;					// Parameter Header 1 DWORD 7
	struct BFPTable_DWORD8 dword8;					// Parameter Header 1 DWORD 8
	struct BFPTable_DWORD9 dword9;					// Parameter Header 1 DWORD 9
	struct BFPTable_DWORD10 dword10;				// Parameter Header 1 DWORD 10
	struct BFPTable_DWORD11 dword11;				// Parameter Header 1 DWORD 11
	struct BFPTable_DWORD12 dword12;				// Parameter Header 1 DWORD 12
	struct BFPTable_DWORD13 dword13;				// Parameter Header 1 DWORD 13
	struct BFPTable_DWORD14 dword14;				// Parameter Header 1 DWORD 14
	struct BFPTable_DWORD15 dword15;				// Parameter Header 1 DWORD 15
	struct BFPTable_DWORD16 dword16;				// Parameter Header 1 DWORD 16
} __attribute__((packed));

struct SFDP_DATA
{
	struct SFDPHeader head;							// Details for SFDP Header
	struct ParameterHeader param_head;				// Parameter Header 1 details
	// Place to insert vendor specific parameter header
	struct Basic_Flash_Parameter basic_param_table;	// Details for Basic Flash Parameter 1 
	// Place to insert vendor specific parameter header DWORDS
} __attribute__((packed));

#endif // _SPI_SFDP_H
