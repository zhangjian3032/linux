/******************************************************************************
 * video.h
 *
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

#ifndef __RVAS_VIDEO_H__
#define __RVAS_VIDEO_H__

#define RVAS_DRIVER_NAME "rvas"
#define Stringify(x) #x

//
//functions
//


void ioctl_new_context(struct file *file, RvasIoctl *pri, AstRVAS *pAstRVAS);
void ioctl_delete_context(RvasIoctl *, AstRVAS *);
void ioctl_alloc(struct file *file, RvasIoctl *, AstRVAS *);
void ioctl_free(RvasIoctl *, AstRVAS *);
void ioctl_update_lms(u8 lms_on, AstRVAS *ast_rvas);
u32 ioctl_get_lm_status(AstRVAS *ast_rvas);


//void* get_from_rsvd_mem(u32 size, u32* phys_add, AstRVAS *pAstRVAS);
void* get_virt_add_rsvd_mem(u32 index, AstRVAS *pAstRVAS);
u32 get_phys_add_rsvd_mem(u32 index, AstRVAS *pAstRVAS);
u32 get_len_rsvd_mem(u32 index, AstRVAS *pAstRVAS);

//int release_rsvd_mem(u32 size, u32 phys_add);
bool virt_is_valid_rsvd_mem(u32 index, u32 size, AstRVAS *pAstRVAS);


ContextTable* get_new_context_table_entry(AstRVAS *pAstRVAS);
ContextTable* get_context_entry(const RVASContext crc, AstRVAS *pAstRVAS);
bool remove_context_table_entry(const RVASMemoryHandle, AstRVAS *pAstRVAS);

#endif // __RVAS_VIDEO_H__
