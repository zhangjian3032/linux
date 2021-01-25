/*
 * Copyright (C) 2019-2021  ASPEED Technology Inc.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef AST_VIDEO_DEBUG_H_
#define AST_VIDEO_DEBUG_H_

#include <linux/string.h>
#include <linux/types.h>
#include <linux/fcntl.h>

#if 0
#define RVAS_VIDEO_DEBUG
#endif

#if 0
#define VIDEO_ENGINE_DEBUG
#endif

#if 0
#define HARDWARE_ENGINE_DEBUG
#endif


#ifdef RVAS_VIDEO_DEBUG
#define VIDEO_DBG(fmt, args...) do { printk(KERN_INFO "%s() " fmt,__FUNCTION__, ## args); } while (0)
#else
#define VIDEO_DBG(fmt, args...) do ; while (0)
#endif // RVAS_VIDEO_DEBUG

#ifdef VIDEO_ENGINE_DEBUG
#define VIDEO_ENG_DBG(fmt, args...) do { printk(KERN_INFO "%s() " fmt,__FUNCTION__, ## args); } while (0)
#else
#define VIDEO_ENG_DBG(fmt, args...) do ; while (0)
#endif // RVAS_VIDEO_DEBUG


#endif // AST_VIDEO_DEBUG_H_
