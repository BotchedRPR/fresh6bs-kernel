/*
 * linux/drivers/video/fbdev/exynos/panel/lx83807/lx83807_csot_small_panel.h
 *
 * Header file for S6W36W5x01 Dimming Driver
 *
 * Copyright (c) 2016 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LX83807_CSOT_SMALL_PANEL_H__
#define __LX83807_CSOT_SMALL_PANEL_H__
#include "../panel.h"
#include "../panel_drv.h"
#include "lx8380x.h"
#include "lx8380x_dimming.h"
#include "lx83807_csot_small_panel_dimming.h"
#include "lx83807_csot_small_resol.h"
#ifdef CONFIG_USDM_PANEL_COPR
#include "lx83807_csot_small_panel_copr.h"
#endif
#ifdef CONFIG_USDM_PANEL_SELF_DISPLAY
#include "lx83807_csot_small_aod_panel.h"
#include "../aod/aod_drv.h"
#endif

#ifdef CONFIG_USDM_PANEL_MAFPC
#include "lx83807_csot_small_abc_data.h"
#endif

#undef __pn_name__
#define __pn_name__	csot_small

#undef __PN_NAME__
#define __PN_NAME__	CSOT_SMALL


static u8 lx83807_csot_small_brt_table[LX8380X_TOTAL_STEP][2] = {
	{ 0x00, 0x1F }, { 0x00, 0x1F }, { 0x00, 0x1F }, { 0x00, 0x20 }, { 0x00, 0x21 }, 
	{ 0x00, 0x21 }, { 0x00, 0x21 }, { 0x00, 0x22 }, { 0x00, 0x22 }, { 0x00, 0x23 }, 
	{ 0x00, 0x23 }, { 0x00, 0x24 }, { 0x00, 0x24 }, { 0x00, 0x24 }, { 0x00, 0x25 }, 	
	{ 0x00, 0x25 }, { 0x00, 0x26 }, { 0x00, 0x26 }, { 0x00, 0x26 }, { 0x00, 0x27 }, 	
	{ 0x00, 0x27 }, { 0x00, 0x27 }, { 0x00, 0x27 }, { 0x00, 0x28 }, { 0x00, 0x28 }, 	
	{ 0x00, 0x28 }, { 0x00, 0x29 }, { 0x00, 0x29 }, { 0x00, 0x29 }, { 0x00, 0x29 }, 	
	{ 0x00, 0x2A }, { 0x00, 0x2A }, { 0x00, 0x2A }, { 0x00, 0x2A }, { 0x00, 0x2B }, 	
	{ 0x00, 0x2B }, { 0x00, 0x2B }, { 0x00, 0x2C }, { 0x00, 0x2C }, { 0x00, 0x2C }, 	
	{ 0x00, 0x2C }, { 0x00, 0x2D }, { 0x00, 0x2D }, { 0x00, 0x2D }, { 0x00, 0x2E }, 	
	{ 0x00, 0x2E }, { 0x00, 0x2E }, { 0x00, 0x2F }, { 0x00, 0x2F }, { 0x00, 0x30 }, 
	{ 0x00, 0x30 }, { 0x00, 0x30 }, { 0x00, 0x31 }, { 0x00, 0x31 }, { 0x00, 0x31 }, 	
	{ 0x00, 0x32 }, { 0x00, 0x32 }, { 0x00, 0x33 }, { 0x00, 0x33 }, { 0x00, 0x33 }, 	
	{ 0x00, 0x34 }, { 0x00, 0x34 }, { 0x00, 0x34 }, { 0x00, 0x35 }, { 0x00, 0x35 }, 	
	{ 0x00, 0x36 }, { 0x00, 0x36 }, { 0x00, 0x37 }, { 0x00, 0x37 }, { 0x00, 0x38 }, 	
	{ 0x00, 0x38 }, { 0x00, 0x39 }, { 0x00, 0x39 }, { 0x00, 0x3A }, { 0x00, 0x3A }, 	
	{ 0x00, 0x3B }, { 0x00, 0x3B }, { 0x00, 0x3C }, { 0x00, 0x3C }, { 0x00, 0x3D }, 	
	{ 0x00, 0x3D }, { 0x00, 0x3E }, { 0x00, 0x3F }, { 0x00, 0x3F }, { 0x00, 0x40 }, 	
	{ 0x00, 0x40 }, { 0x00, 0x41 }, { 0x00, 0x41 }, { 0x00, 0x42 }, { 0x00, 0x43 }, 	
	{ 0x00, 0x43 }, { 0x00, 0x44 }, { 0x00, 0x45 }, { 0x00, 0x45 }, { 0x00, 0x46 }, 	
	{ 0x00, 0x47 }, { 0x00, 0x47 }, { 0x00, 0x48 }, { 0x00, 0x49 }, { 0x00, 0x4A }, 	
	{ 0x00, 0x4A }, { 0x00, 0x4B }, { 0x00, 0x4C }, { 0x00, 0x4D }, { 0x00, 0x4D }, 	
	{ 0x00, 0x4E }, { 0x00, 0x4F }, { 0x00, 0x50 }, { 0x00, 0x50 }, { 0x00, 0x51 }, 	
	{ 0x00, 0x52 }, { 0x00, 0x53 }, { 0x00, 0x54 }, { 0x00, 0x55 }, { 0x00, 0x56 }, 	
	{ 0x00, 0x57 }, { 0x00, 0x57 }, { 0x00, 0x58 }, { 0x00, 0x59 }, { 0x00, 0x5A }, 	
	{ 0x00, 0x5B }, { 0x00, 0x5C }, { 0x00, 0x5D }, { 0x00, 0x5E }, { 0x00, 0x5F }, 	
	{ 0x00, 0x60 }, { 0x00, 0x61 }, { 0x00, 0x62 }, { 0x00, 0x63 }, { 0x00, 0x64 }, 	
	{ 0x00, 0x65 }, { 0x00, 0x66 }, { 0x00, 0x67 }, { 0x00, 0x68 }, { 0x00, 0x69 }, 	
	{ 0x00, 0x6A }, { 0x00, 0x6B }, { 0x00, 0x6C }, { 0x00, 0x6E }, { 0x00, 0x6F }, 	
	{ 0x00, 0x70 }, { 0x00, 0x71 }, { 0x00, 0x72 }, { 0x00, 0x73 }, { 0x00, 0x74 }, 	
	{ 0x00, 0x76 }, { 0x00, 0x77 }, { 0x00, 0x78 }, { 0x00, 0x79 }, { 0x00, 0x7A }, 	
	{ 0x00, 0x7B }, { 0x00, 0x7D }, { 0x00, 0x7E }, { 0x00, 0x7F }, { 0x00, 0x81 }, 	
	{ 0x00, 0x82 }, { 0x00, 0x83 }, { 0x00, 0x85 }, { 0x00, 0x86 }, { 0x00, 0x87 }, 	
	{ 0x00, 0x89 }, { 0x00, 0x8A }, { 0x00, 0x8B }, { 0x00, 0x8D }, { 0x00, 0x8E }, 	
	{ 0x00, 0x90 }, { 0x00, 0x91 }, { 0x00, 0x93 }, { 0x00, 0x94 }, { 0x00, 0x96 }, 	
	{ 0x00, 0x97 }, { 0x00, 0x99 }, { 0x00, 0x9A }, { 0x00, 0x9C }, { 0x00, 0x9D }, 	
	{ 0x00, 0x9F }, { 0x00, 0xA1 }, { 0x00, 0xA2 }, { 0x00, 0xA3 }, { 0x00, 0xA5 }, 	
	{ 0x00, 0xA7 }, { 0x00, 0xA8 }, { 0x00, 0xAA }, { 0x00, 0xAC }, { 0x00, 0xAD }, 	
	{ 0x00, 0xAF }, { 0x00, 0xB1 }, { 0x00, 0xB2 }, { 0x00, 0xB4 }, { 0x00, 0xB6 }, 	
	{ 0x00, 0xB7 }, { 0x00, 0xB9 }, { 0x00, 0xBB }, { 0x00, 0xBD }, { 0x00, 0xBF }, 	
	{ 0x00, 0xC1 }, { 0x00, 0xC2 }, { 0x00, 0xC4 }, { 0x00, 0xC6 }, { 0x00, 0xC8 }, 	
	{ 0x00, 0xCA }, { 0x00, 0xCC }, { 0x00, 0xCD }, { 0x00, 0xCF }, { 0x00, 0xD2 }, 	
	{ 0x00, 0xD3 }, { 0x00, 0xD5 }, { 0x00, 0xD7 }, { 0x00, 0xD9 }, { 0x00, 0xDB }, 	
	{ 0x00, 0xDD }, { 0x00, 0xDF }, { 0x00, 0xE1 }, { 0x00, 0xE3 }, { 0x00, 0xE6 }, 	
	{ 0x00, 0xE8 }, { 0x00, 0xEA }, { 0x00, 0xEC }, { 0x00, 0xEE }, { 0x00, 0xF0 }, 	
	{ 0x00, 0xF2 }, { 0x00, 0xF5 }, { 0x00, 0xF7 }, { 0x00, 0xF9 }, { 0x00, 0xFB }, 	
	{ 0x00, 0xFE }, { 0x01, 0x00 }, { 0x01, 0x02 }, { 0x01, 0x04 }, { 0x01, 0x07 }, 	
	{ 0x01, 0x09 }, { 0x01, 0x0B }, { 0x01, 0x0D }, { 0x01, 0x10 }, { 0x01, 0x12 }, 	
	{ 0x01, 0x15 }, { 0x01, 0x17 }, { 0x01, 0x1A }, { 0x01, 0x1C }, { 0x01, 0x1E }, 	
	{ 0x01, 0x21 }, { 0x01, 0x23 }, { 0x01, 0x26 }, { 0x01, 0x28 }, { 0x01, 0x2B }, 	
	{ 0x01, 0x2D }, { 0x01, 0x2F }, { 0x01, 0x32 }, { 0x01, 0x35 }, { 0x01, 0x37 }, 	
	{ 0x01, 0x3A }, { 0x01, 0x3C }, { 0x01, 0x3F }, { 0x01, 0x42 }, { 0x01, 0x44 }, 	
	{ 0x01, 0x47 }, { 0x01, 0x49 }, { 0x01, 0x4A }, { 0x01, 0x4C }, { 0x01, 0x4E }, 	
	{ 0x01, 0x4F }, { 0x01, 0x51 }, { 0x01, 0x52 }, { 0x01, 0x54 }, { 0x01, 0x55 }, 	
	{ 0x01, 0x57 }, { 0x01, 0x58 }, { 0x01, 0x5A }, { 0x01, 0x5B }, { 0x01, 0x5D }, 	
	{ 0x01, 0x5E }, { 0x01, 0x60 }, { 0x01, 0x62 }, { 0x01, 0x63 }, { 0x01, 0x65 }, 	
	{ 0x01, 0x66 }, { 0x01, 0x68 }, { 0x01, 0x69 }, { 0x01, 0x6B }, { 0x01, 0x6C }, 	
	{ 0x01, 0x6E }, { 0x01, 0x6F }, { 0x01, 0x71 }, { 0x01, 0x72 }, { 0x01, 0x74 }, 	
	{ 0x01, 0x76 }, { 0x01, 0x79 }, { 0x01, 0x7C }, { 0x01, 0x7E }, { 0x01, 0x7F }, 	
	{ 0x01, 0x81 }, { 0x01, 0x83 }, { 0x01, 0x85 }, { 0x01, 0x86 }, { 0x01, 0x88 }, 	
	{ 0x01, 0x8A }, { 0x01, 0x8B }, { 0x01, 0x8D }, { 0x01, 0x8F }, { 0x01, 0x90 }, 	
	{ 0x01, 0x92 }, { 0x01, 0x94 }, { 0x01, 0x96 }, { 0x01, 0x97 }, { 0x01, 0x99 }, 	
	{ 0x01, 0x9B }, { 0x01, 0x9C }, { 0x01, 0x9E }, { 0x01, 0xA0 }, { 0x01, 0xA1 }, 	
	{ 0x01, 0xA3 }, { 0x01, 0xA5 }, { 0x01, 0xA7 }, { 0x01, 0xA8 }, { 0x01, 0xAA }, 	
	{ 0x01, 0xAD }, { 0x01, 0xAF }, { 0x01, 0xB1 }, { 0x01, 0xB2 }, { 0x01, 0xB4 }, 	
	{ 0x01, 0xB6 }, { 0x01, 0xB7 }, { 0x01, 0xB9 }, { 0x01, 0xBB }, { 0x01, 0xBC }, 	
	{ 0x01, 0xBE }, { 0x01, 0xC0 }, { 0x01, 0xC1 }, { 0x01, 0xC3 }, { 0x01, 0xC5 }, 	
	{ 0x01, 0xC6 }, { 0x01, 0xC8 }, { 0x01, 0xCA }, { 0x01, 0xCB }, { 0x01, 0xCD }, 	
	{ 0x01, 0xCF }, { 0x01, 0xD0 }, { 0x01, 0xD2 }, { 0x01, 0xD4 }, { 0x01, 0xD5 }, 	
	{ 0x01, 0xD7 }, { 0x01, 0xD9 }, { 0x01, 0xDA }, { 0x01, 0xDC }, { 0x01, 0xDE }, 	
	{ 0x01, 0xE0 }, { 0x01, 0xE2 }, { 0x01, 0xE4 }, { 0x01, 0xE6 }, { 0x01, 0xE8 }, 	
	{ 0x01, 0xEA }, { 0x01, 0xEB }, { 0x01, 0xED }, { 0x01, 0xEF }, { 0x01, 0xF1 }, 	
	{ 0x01, 0xF3 }, { 0x01, 0xF5 }, { 0x01, 0xF7 }, { 0x01, 0xF9 }, { 0x01, 0xFB }, 	
	{ 0x01, 0xFD }, { 0x01, 0xFF }, { 0x02, 0x01 }, { 0x02, 0x03 }, { 0x02, 0x05 }, 	
	{ 0x02, 0x07 }, { 0x02, 0x08 }, { 0x02, 0x0A }, { 0x02, 0x0C }, { 0x02, 0x0E }, 	
	{ 0x02, 0x12 }, { 0x02, 0x13 }, { 0x02, 0x15 }, { 0x02, 0x17 }, { 0x02, 0x19 }, 	
	{ 0x02, 0x1B }, { 0x02, 0x1C }, { 0x02, 0x1E }, { 0x02, 0x20 }, { 0x02, 0x22 }, 	
	{ 0x02, 0x23 }, { 0x02, 0x25 }, { 0x02, 0x27 }, { 0x02, 0x29 }, { 0x02, 0x2A }, 	
	{ 0x02, 0x2C }, { 0x02, 0x2E }, { 0x02, 0x30 }, { 0x02, 0x32 }, { 0x02, 0x33 }, 	
	{ 0x02, 0x35 }, { 0x02, 0x37 }, { 0x02, 0x39 }, { 0x02, 0x3A }, { 0x02, 0x3C }, 	
	{ 0x02, 0x3E }, { 0x02, 0x41 }, { 0x02, 0x43 }, { 0x02, 0x45 }, { 0x02, 0x48 }, 	
	{ 0x02, 0x4A }, { 0x02, 0x4D }, { 0x02, 0x4F }, { 0x02, 0x52 }, { 0x02, 0x54 }, 	
	{ 0x02, 0x56 }, { 0x02, 0x59 }, { 0x02, 0x5B }, { 0x02, 0x5E }, { 0x02, 0x60 }, 	
	{ 0x02, 0x63 }, { 0x02, 0x65 }, { 0x02, 0x67 }, { 0x02, 0x6A }, { 0x02, 0x6C }, 	
	{ 0x02, 0x6F }, { 0x02, 0x71 }, { 0x02, 0x73 }, { 0x02, 0x75 }, { 0x02, 0x78 }, 	
	{ 0x02, 0x7A }, { 0x02, 0x7C }, { 0x02, 0x7E }, { 0x02, 0x80 }, { 0x02, 0x82 }, 	
	{ 0x02, 0x84 }, { 0x02, 0x87 }, { 0x02, 0x89 }, { 0x02, 0x8B }, { 0x02, 0x8D }, 	
	{ 0x02, 0x8F }, { 0x02, 0x91 }, { 0x02, 0x93 }, { 0x02, 0x96 }, { 0x02, 0x98 }, 	
	{ 0x02, 0x9A }, { 0x02, 0x9C }, { 0x02, 0x9E }, { 0x02, 0xA0 }, { 0x02, 0xA2 }, 	
	{ 0x02, 0xA4 }, { 0x02, 0xA6 }, { 0x02, 0xA8 }, { 0x02, 0xAA }, { 0x02, 0xAC }, 	
	{ 0x02, 0xAE }, { 0x02, 0xB0 }, { 0x02, 0xB2 }, { 0x02, 0xB4 }, { 0x02, 0xB5 }, 	
	{ 0x02, 0xB7 }, { 0x02, 0xB9 }, { 0x02, 0xBB }, { 0x02, 0xBD }, { 0x02, 0xBF }, 	
	{ 0x02, 0xC1 }, { 0x02, 0xC3 }, { 0x02, 0xC5 }, { 0x02, 0xC6 }, { 0x02, 0xC8 }, 	
	{ 0x02, 0xCA }, { 0x02, 0xCC }, { 0x02, 0xCE }, { 0x02, 0xD0 }, { 0x02, 0xD2 }, 	
	{ 0x02, 0xD4 }, { 0x02, 0xD6 }, { 0x02, 0xD8 }, { 0x02, 0xDA }, { 0x02, 0xDC }, 	
	{ 0x02, 0xDE }, { 0x02, 0xE0 }, { 0x02, 0xE3 }, { 0x02, 0xE5 }, { 0x02, 0xE7 }, 	
	{ 0x02, 0xE9 }, { 0x02, 0xEB }, { 0x02, 0xED }, { 0x02, 0xEF }, { 0x02, 0xF2 }, 	
	{ 0x02, 0xF4 }, { 0x02, 0xF6 }, { 0x02, 0xF8 }, { 0x02, 0xFA }, { 0x02, 0xFC }, 	
	{ 0x02, 0xFE }, { 0x03, 0x01 }, { 0x03, 0x03 }, { 0x03, 0x05 }, { 0x03, 0x07 }, 	
	{ 0x03, 0x09 }, { 0x03, 0x0B }, { 0x03, 0x0D }, { 0x03, 0x10 }, { 0x03, 0x12 }, 	
	{ 0x03, 0x14 }, { 0x03, 0x16 }, { 0x03, 0x18 }, { 0x03, 0x1A }, { 0x03, 0x1D }, 	
	{ 0x03, 0x1F }, { 0x03, 0x21 }, { 0x03, 0x23 }, { 0x03, 0x25 }, { 0x03, 0x27 }, 	
	{ 0x03, 0x29 }, { 0x03, 0x2C }, { 0x03, 0x2E }, { 0x03, 0x30 }, { 0x03, 0x32 }, 	
	{ 0x03, 0x34 }, { 0x03, 0x36 }, { 0x03, 0x39 }, { 0x03, 0x3B }, { 0x03, 0x3D }, 	
	{ 0x03, 0x3F }, { 0x03, 0x41 }, { 0x03, 0x44 }, { 0x03, 0x46 }, { 0x03, 0x48 }, 	
	{ 0x03, 0x4A }, { 0x03, 0x4C }, { 0x03, 0x4E }, { 0x03, 0x51 }, { 0x03, 0x53 }, 	
	{ 0x03, 0x55 }, { 0x03, 0x57 }, { 0x03, 0x59 }, { 0x03, 0x5C }, { 0x03, 0x5E }, 	
	{ 0x03, 0x60 }, { 0x03, 0x62 }, { 0x03, 0x64 }, { 0x03, 0x67 }, { 0x03, 0x69 }, 	
	{ 0x03, 0x6B }, { 0x03, 0x6D }, { 0x03, 0x70 }, { 0x03, 0x72 }, { 0x03, 0x74 }, 	
	{ 0x03, 0x77 }, { 0x03, 0x79 }, { 0x03, 0x7B }, { 0x03, 0x7E }, { 0x03, 0x80 }, 	
	{ 0x03, 0x82 }, { 0x03, 0x85 }, { 0x03, 0x87 }, { 0x03, 0x89 }, { 0x03, 0x8C }, 	
	{ 0x03, 0x8E }, { 0x03, 0x90 }, { 0x03, 0x93 }, { 0x03, 0x95 }, { 0x03, 0x97 }, 	
	{ 0x03, 0x9A }, { 0x03, 0x9C }, { 0x03, 0x9E }, { 0x03, 0xA1 }, { 0x03, 0xA3 }, 	
	{ 0x03, 0xA5 }, { 0x03, 0xA8 }, { 0x03, 0xAA }, { 0x03, 0xAC }, { 0x03, 0xAF }, 	
	{ 0x03, 0xB1 }, { 0x03, 0xB3 }, { 0x03, 0xB5 }, { 0x03, 0xB8 }, { 0x03, 0xBA }, 	
	{ 0x03, 0xBC }, { 0x03, 0xBF }, { 0x03, 0xC1 }, { 0x03, 0xC3 }, { 0x03, 0xC6 }, 	
	{ 0x03, 0xC8 }, { 0x03, 0xCA }, { 0x03, 0xCD }, { 0x03, 0xCF }, { 0x03, 0xD1 }, 	
	{ 0x03, 0xD3 }, { 0x03, 0xD6 }, { 0x03, 0xD8 }, { 0x03, 0xDA }, { 0x03, 0xDD }, 	
	{ 0x03, 0xDF }, { 0x03, 0xE1 }, { 0x03, 0xE4 }, { 0x03, 0xE6 }, { 0x03, 0xE8 }, 	
	{ 0x03, 0xEA }, { 0x03, 0xED }, { 0x03, 0xEF }, { 0x03, 0xF1 }, { 0x03, 0xF4 }, 	
	{ 0x03, 0xF6 }, { 0x03, 0xF8 }, { 0x03, 0xFA }, { 0x03, 0xFD }, { 0x03, 0xFF },
};
// maptbl table info

static u8 lx83807_csot_small_acl_control_table[MAX_LX8380X_ACL_RATIO][1] = {
	[LX8380X_ACL_RATIO_0] = { 0x0C },
	[LX8380X_ACL_RATIO_8] = { 0x0D },
	[LX8380X_ACL_RATIO_15] = { 0x0D },
};

static u8 lx83807_csot_small_elvss_temp_table[MAX_TEMP][1] = {
	[GE_TEMP_1] = { 0x00 },
	[EQ_TEMP_0] = { 0x00 },
	[LE_TEMP_MINUS1] = { 0x8F },
	[LE_TEMP_MINUS15] = { 0x90 },
};

#ifdef CONFIG_USDM_PANEL_MAFPC
static u8 lx83807_csot_small_mafpc_enable_table[2][1] = {
	{ 0x00 },
	{ 0x71 },
};
#endif

static struct maptbl lx83807_csot_small_maptbl[MAX_MAPTBL] = {
	[GAMMA_MODE2_MAPTBL] = DEFINE_2D_MAPTBL(lx83807_csot_small_brt_table, &DDI_FUNC(LX8380X_MAPTBL_INIT_GAMMA_MODE2_BRT), &OLED_FUNC(OLED_MAPTBL_GETIDX_GM2_BRT), &OLED_FUNC(OLED_MAPTBL_COPY_DEFAULT)),
	[ACL_CONTROL_MAPTBL] = DEFINE_2D_MAPTBL(lx83807_csot_small_acl_control_table, &OLED_FUNC(OLED_MAPTBL_INIT_DEFAULT), &DDI_FUNC(LX8380X_MAPTBL_GETIDX_ACL_CONTROL), &OLED_FUNC(OLED_MAPTBL_COPY_DEFAULT)),
	[ELVSS_TEMP_MAPTBL] = DEFINE_2D_MAPTBL(lx83807_csot_small_elvss_temp_table, &OLED_FUNC(OLED_MAPTBL_INIT_DEFAULT), &DDI_FUNC(LX8380X_MAPTBL_GETIDX_TSET), &OLED_FUNC(OLED_MAPTBL_COPY_DEFAULT)),
#ifdef CONFIG_USDM_PANEL_MAFPC
	[MAFPC_ENA_MAPTBL] = DEFINE_2D_MAPTBL(lx83807_csot_small_mafpc_enable_table, &OLED_FUNC(OLED_MAPTBL_INIT_DEFAULT), &DDI_FUNC(LX8380X_MAPTBL_GETIDX_MAFPC_ENABLE), &DDI_FUNC(LX8380X_MAPTBL_COPY_MAFPC_ENABLE)),
	[MAFPC_CTRL_MAPTBL] = DEFINE_0D_MAPTBL(lx83807_csot_small_mafpc_ctrl, &OLED_FUNC(OLED_MAPTBL_INIT_DEFAULT), NULL, &DDI_FUNC(LX8380X_MAPTBL_COPY_MAFPC_CTRL)),
	[MAFPC_SCALE_MAPTBL] = DEFINE_0D_MAPTBL(lx83807_csot_small_mafpc_scale, &OLED_FUNC(OLED_MAPTBL_INIT_DEFAULT), NULL, &DDI_FUNC(LX8380X_MAPTBL_COPY_MAFPC_SCALE)),
	[MAFPC_IMG_MAPTBL] = DEFINE_0D_MAPTBL(lx83807_csot_small_mafpc_img, &OLED_FUNC(OLED_MAPTBL_INIT_DEFAULT), NULL, &DDI_FUNC(LX8380X_MAPTBL_COPY_MAFPC_IMG)),
#endif
};

static DEFINE_PANEL_MDELAY(lx83807_csot_small_wait_1msec, 1);
static DEFINE_PANEL_MDELAY(lx83807_csot_small_wait_5msec, 5);
static DEFINE_PANEL_MDELAY(lx83807_csot_small_wait_10msec, 10);
static DEFINE_PANEL_MDELAY(lx83807_csot_small_wait_17msec, 17);
static DEFINE_PANEL_MDELAY(lx83807_csot_small_wait_34msec, 34);
static DEFINE_PANEL_MDELAY(lx83807_csot_small_wait_42msec, 42);
static DEFINE_PANEL_MDELAY(lx83807_csot_small_wait_20msec, 20);
static DEFINE_PANEL_MDELAY(lx83807_csot_small_wait_66msec, 66);
static DEFINE_PANEL_MDELAY(lx83807_csot_small_wait_80msec, 80);
static DEFINE_PANEL_MDELAY(lx83807_csot_small_wait_120msec, 120);
static DEFINE_PANEL_MDELAY(lx83807_csot_small_wait_133msec, 133);

static DEFINE_PANEL_TIMER_MDELAY(lx83807_csot_small_wait_sleep_out_120msec, 120);
static DEFINE_PANEL_TIMER_BEGIN(lx83807_csot_small_wait_sleep_out_120msec,
		TIMER_DLYINFO(&lx83807_csot_small_wait_sleep_out_120msec));
static DEFINE_PANEL_TIMER_MDELAY(lx83807_csot_small_wait_sleep_out_76msec, 76);
static DEFINE_PANEL_TIMER_BEGIN(lx83807_csot_small_wait_sleep_out_76msec,
		TIMER_DLYINFO(&lx83807_csot_small_wait_sleep_out_76msec));

static u8 LX83807_CSOT_SMALL_SLEEP_OUT[] = { 0x11 };
static u8 LX83807_CSOT_SMALL_SLEEP_IN[] = { 0x10 };
static u8 LX83807_CSOT_SMALL_DISPLAY_OFF[] = { 0x28 };
static u8 LX83807_CSOT_SMALL_DISPLAY_ON[] = { 0x29 };
static u8 LX83807_CSOT_SMALL_TE_ON[] = { 0x35, 0x00, 0x00 };
static u8 LX83807_CSOT_SMALL_ENTER_IDLE[] = { 0x39 };
static u8 LX83807_CSOT_SMALL_EXIT_IDLE[] = { 0x38 };

static DEFINE_STATIC_PACKET(lx83807_csot_small_sleep_out, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_SLEEP_OUT, 0);
static DEFINE_STATIC_PACKET(lx83807_csot_small_sleep_in, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_SLEEP_IN, 0);
static DEFINE_STATIC_PACKET(lx83807_csot_small_display_on, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_DISPLAY_ON, 0);
static DEFINE_STATIC_PACKET(lx83807_csot_small_display_off, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_DISPLAY_OFF, 0);
static DEFINE_STATIC_PACKET(lx83807_csot_small_te_on, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_TE_ON, 0);
static DEFINE_STATIC_PACKET(lx83807_csot_small_enter_idle, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_ENTER_IDLE, 0);
static DEFINE_STATIC_PACKET(lx83807_csot_small_exit_idle, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_EXIT_IDLE, 0);

static u8 LX83807_CSOT_SMALL_TEST_A1_ON[] = { 0xB0, 0xA1 };
static u8 LX83807_CSOT_SMALL_TEST_A2_ON[] = { 0xB0, 0xA2 };
static u8 LX83807_CSOT_SMALL_TEST_A3_ON[] = { 0xB0, 0xA3 };
static u8 LX83807_CSOT_SMALL_TEST_A4_ON[] = { 0xB0, 0xA4 };
static u8 LX83807_CSOT_SMALL_TEST_A6_ON[] = { 0xB0, 0xA6 };
static u8 LX83807_CSOT_SMALL_TEST_A7_ON[] = { 0xB0, 0xA7 };
static u8 LX83807_CSOT_SMALL_TEST_OFF[] = { 0xB0, 0xCA };
static u8 LX83807_CSOT_SMALL_GPARA_ENABLE[] = { 0xA3, 0x01 };
static u8 LX83807_CSOT_SMALL_GPARA_DISABLE[] = {	0xA3, 0x00 };

static DEFINE_STATIC_PACKET(lx83807_csot_small_test_a1_on, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_TEST_A1_ON, 0);
static DEFINE_STATIC_PACKET(lx83807_csot_small_test_a2_on, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_TEST_A2_ON, 0);
static DEFINE_STATIC_PACKET(lx83807_csot_small_test_a3_on, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_TEST_A3_ON, 0);
static DEFINE_STATIC_PACKET(lx83807_csot_small_test_a4_on, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_TEST_A4_ON, 0);
static DEFINE_STATIC_PACKET(lx83807_csot_small_test_a6_on, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_TEST_A6_ON, 0);
static DEFINE_STATIC_PACKET(lx83807_csot_small_test_a7_on, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_TEST_A7_ON, 0);
static DEFINE_STATIC_PACKET(lx83807_csot_small_test_off, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_TEST_OFF, 0);
static DEFINE_STATIC_PACKET(lx83807_csot_small_gpara_enable, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_GPARA_ENABLE, 0);
static DEFINE_STATIC_PACKET(lx83807_csot_small_gpara_disable, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_GPARA_DISABLE, 0);

static u8 LX83807_CSOT_SMALL_CASET[] = {
	0x2A,
	0x00, 0x00, 0x01, 0xAF
};
static u8 LX83807_CSOT_SMALL_PASET[] = {
	0x2B,
	0x00, 0x00, 0x01, 0xAF,
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_caset, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_CASET, 0);
static DEFINE_STATIC_PACKET(lx83807_csot_small_paset, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_PASET, 0);

static u8 LX83807_CSOT_SMALL_OSC_MIPI_SPEED[] = {
	0x68,
	0x14, 0x80, 0x00
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_osc_mipi_speed, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_OSC_MIPI_SPEED, 0);

static u8 LX83807_CSOT_SMALL_TOUCH_VSYNC_ON[] = {
	0xFC,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_touch_vsync_on, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_TOUCH_VSYNC_ON, 0);

static u8 LX83807_CSOT_SMALL_INIT_SMOOTH_DIMMING_FRAME[] = {
	0xB3,
	0xFC, 0x81, 0x00
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_init_smooth_dimming_frame, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_INIT_SMOOTH_DIMMING_FRAME, 0);

static u8 LX83807_CSOT_SMALL_BL_SMOOTH_DIMMING_FRAME[] = {       // 8 or 32
	0xB3,
	0xFC, 0xA0, 0x00
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_bl_smooth_dimming_frame, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_BL_SMOOTH_DIMMING_FRAME, 0);

static u8 LX83807_CSOT_SMALL_LFD_30[] = {
	0x3D,
	0x04, 0x04, 0x10
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_lfd_30, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_LFD_30, 0);

static u8 LX83807_CSOT_SMALL_LFD_1[] = {
	0x3D,
	0x04, 0x04, 0x11
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_lfd_1, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_LFD_1, 0);

static u8 LX83807_CSOT_SMALL_CLOCK_GATING[] = {
	0x5D,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_clock_gating, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_CLOCK_GATING, 0);

static u8 LX83807_CSOT_SMALL_REF_VOLTAGE_MODE[] = {
	0xFB,
	0x00, 0x10, 0x00, 0x00, 0x05, 0x01, 0x00, 0x00
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_ref_voltage_mode, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_REF_VOLTAGE_MODE, 0);

static u8 LX83807_CSOT_SMALL_GAMMA_WRCTRLD[] = {
	0x53,
	0x20,
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_wrctrld, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_GAMMA_WRCTRLD, 0);

static u8 LX83807_CSOT_SMALL_WRDISBV[] = {
	0x51,
	0x01, 0x47
};
static DEFINE_PKTUI(lx83807_csot_small_wrdisbv, &lx83807_csot_small_maptbl[GAMMA_MODE2_MAPTBL], 1);
static DEFINE_VARIABLE_PACKET(lx83807_csot_small_wrdisbv, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_WRDISBV, 0);

static u8 LX83807_CSOT_SMALL_ACL_FOR_HBM_1[] = {
	0xB1,
	0x3C, 0xBC, 0xB2, 0x03, 0x00, 0x00, 0x1F, 0x5D,
	0x57
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_acl_for_hbm_1, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_ACL_FOR_HBM_1, 0);

static u8 LX83807_CSOT_SMALL_ACL_FOR_HBM_2[] = {
	0x55,
	0x0F
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_acl_for_hbm_2, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_ACL_FOR_HBM_2, 0);

static u8 LX83807_CSOT_SMALL_TE_SETTING_1[] = {
	0xB5,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 
	0x01, 0x00, 0x00
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_te_setting_1, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_TE_SETTING_1, 0);

static u8 LX83807_CSOT_SMALL_ACL_CONTROL[] = {
	0x55,
	0x0C, 0x00
};
static DEFINE_PKTUI(lx83807_csot_small_acl_control, &lx83807_csot_small_maptbl[ACL_CONTROL_MAPTBL], 1);
static DEFINE_VARIABLE_PACKET(lx83807_csot_small_acl_control, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_ACL_CONTROL, 0);

static u8 LX83807_CSOT_SMALL_ELVSS_TEMP[] = {
	0x4D,
	0x00
};
static DEFINE_PKTUI(lx83807_csot_small_elvss_temp, &lx83807_csot_small_maptbl[ELVSS_TEMP_MAPTBL], 1);
static DEFINE_VARIABLE_PACKET(lx83807_csot_small_elvss_temp, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_ELVSS_TEMP, 0x00);

static u8 LX83807_CSOT_SMALL_FACTORY_LPM_WRDISBV[] = {
	0x51,
	0x00, 0x35			// 50nit
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_factory_lpm_wrdisbv, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_FACTORY_LPM_WRDISBV, 0);

// for copr
static u8 LX83807_CSOT_SMALL_MCD_FSKIP_CG_ENABLE[] = {
	0xB1,
	0x7E, 0x00
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mcd_fskip_cg_enable, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MCD_FSKIP_CG_ENABLE, 0);

static u8 LX83807_CSOT_SMALL_MCD_ON_1[] = {
	0x69,
	0x08,
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mcd_on_1, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MCD_ON_1, 0);

static u8 LX83807_CSOT_SMALL_MCD_OFF_1[] = {
	0x69,
	0x00
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mcd_off_1, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MCD_OFF_1, 0);

#ifdef CONFIG_USDM_PANEL_MAFPC
static u8 LX83807_CSOT_SMALL_MAFPC_GRAM_ENABLE[] = {
	0xB1, 0x7E,
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mafpc_gram_enable, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MAFPC_GRAM_ENABLE, 0);

static u8 LX83807_CSOT_SMALL_MAFPC_SF_SEL_ENABLE[] = {
	0x75, 0x40,
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mafpc_sf_sel_enable, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MAFPC_SF_SEL_ENABLE, 0);

static u8 LX83807_CSOT_SMALL_MAFPC_SF_SEL_DISABLE[] = {
	0x75, 0x00,
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mafpc_sf_sel_disable, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MAFPC_SF_SEL_DISABLE, 0);

static DEFINE_PKTUI(lx83807_small_mafpc_default_img, &lx83807_csot_small_maptbl[MAFPC_IMG_MAPTBL], 0);
static DEFINE_VARIABLE_PACKET_WITH_OPTION(lx83807_small_mafpc_default_img, DSI_PKT_TYPE_WR_SR_FAST,
	LX83807_SMALL_MAFPC_DEFAULT_IMG, 0, PKT_OPTION_SR_ALIGN_12);

static u8 LX83807_CSOT_SMALL_MAFPC_DISABLE[] = {
	0x87, 0x00,
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mafpc_disable, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MAFPC_DISABLE, 0);

static u8 LX83807_CSOT_SMALL_MAFPC_ENABLE[] = {
	0x87,
	0x71, 0x13, 0x08, 0x02, 0x00, 0x00, 0x66, 0x66, 0x66, 0xFF,
	0xFF, 0xFF, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0x00, 0x00, 0x00, 0x40, 0x36, 0x2D, 0x90
};
static DECLARE_PKTUI(lx83807_csot_small_mafpc_enable) = {
	{ .offset = LX8380X_MAFPC_ENA_REG + 1, .maptbl = &lx83807_csot_small_maptbl[MAFPC_ENA_MAPTBL] },
	{ .offset = LX8380X_MAFPC_CTRL_CMD_OFFSET + 1, .maptbl = &lx83807_csot_small_maptbl[MAFPC_CTRL_MAPTBL] },
};
static DEFINE_VARIABLE_PACKET(lx83807_csot_small_mafpc_enable, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MAFPC_ENABLE, 0);

static u8 LX83807_CSOT_SMALL_MAFPC_CRC_BIST_ON[] = {
	0xFD,
	0x80, 0x00, 0xFF, 0xFF, 0xFF
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mafpc_crc_bist_on, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MAFPC_CRC_BIST_ON, 0x00);

static u8 LX83807_CSOT_SMALL_MAFPC_CRC_BIST_OFF[] = {
	0xFD,
	0x00, 0x00, 0x00, 0x00
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mafpc_crc_bist_off, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MAFPC_CRC_BIST_OFF, 0x00);

static u8 LX83807_CSOT_SMALL_MAFPC_CRC_ENABLE[] = {
	0x87,
	0x71, 0x19, 0x08, 0x02, 0x00, 0x00, 0x66, 0x66,
	0x66, 0xFF, 0xFF, 0xFF, 0x33, 0x33, 0x33, 0x33,
	0x33, 0x33, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0x00, 0x00, 0x00, 0x40, 0x3C, 0x38, 0x40
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mafpc_crc_enable, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MAFPC_CRC_ENABLE, 0);

static u8 LX83807_CSOT_SMALL_MAFPC_LUMINANCE_UPDATE_1[] = {
	0x51,
	0x00, 0x00
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mafpc_luminance_update_1, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MAFPC_LUMINANCE_UPDATE_1, 0);

static u8 LX83807_CSOT_SMALL_MAFPC_LUMINANCE_UPDATE_2[] = {
	0x51,
	0x01, 0x47
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mafpc_luminance_update_2, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MAFPC_LUMINANCE_UPDATE_2, 0);

static u8 LX83807_CSOT_SMALL_MAFPC_SELF_MASK_OFF[] = {
	0x7C,
	0x00
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mafpc_self_mask_off, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MAFPC_SELF_MASK_OFF, 0);

static u8 LX83807_CSOT_SMALL_MAFPC_SELF_MASK_ON[] = {
	0x7C,
	0x01
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mafpc_self_mask_on, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MAFPC_SELF_MASK_ON, 0);

static u8 LX83807_CSOT_SMALL_MAFPC_CRC_ON[] = {
	0xB1,
	0x7E, 0x00, 0x00, 0x00, 0x18
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mafpc_crc_on, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MAFPC_CRC_ON, 0);

static u8 LX83807_CSOT_SMALL_MAFPC_CRC_OFF[] = {
	0xB1,
	0x7E, 0x00, 0x00, 0x00, 0x10
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mafpc_crc_off, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MAFPC_CRC_OFF, 0);

static DEFINE_PANEL_UDELAY(lx83807_csot_small_wait_1usec, 1);
static DEFINE_PANEL_UDELAY(lx83807_csot_small_wait_100usec, 100);
static DEFINE_PANEL_MDELAY(lx83807_csot_small_crc_wait, 34);

static void *lx83807_csot_small_mafpc_image_cmdtbl[] = {
	&PKTINFO(lx83807_csot_small_test_a4_on),
	&PKTINFO(lx83807_csot_small_mafpc_gram_enable),
	&PKTINFO(lx83807_csot_small_mafpc_sf_sel_enable),
	&DLYINFO(lx83807_csot_small_wait_1usec),
	&PKTINFO(lx83807_csot_small_mafpc_disable),
	&DLYINFO(lx83807_csot_small_wait_17msec),
	&PKTINFO(lx83807_small_mafpc_default_img),
	&DLYINFO(lx83807_csot_small_wait_100usec),
	&PKTINFO(lx83807_csot_small_mafpc_sf_sel_disable),
	&PKTINFO(lx83807_csot_small_test_off),
};

static void *lx83807_csot_small_mafpc_on_cmdtbl[] = {
	&PKTINFO(lx83807_csot_small_test_a4_on),
	&PKTINFO(lx83807_csot_small_mafpc_enable),
	&PKTINFO(lx83807_csot_small_test_off),
};

static void *lx83807_csot_small_mafpc_off_cmdtbl[] = {
	&PKTINFO(lx83807_csot_small_test_a4_on),
	&PKTINFO(lx83807_csot_small_mafpc_disable),
	&PKTINFO(lx83807_csot_small_test_off),
};

static void *lx83807_csot_small_mafpc_check_cmdtbl[] = {
	&PKTINFO(lx83807_csot_small_test_a4_on),
	&PKTINFO(lx83807_csot_small_mafpc_crc_on),
	&PKTINFO(lx83807_csot_small_test_off),
	&PKTINFO(lx83807_csot_small_test_a6_on),
	&PKTINFO(lx83807_csot_small_mafpc_crc_bist_on),
	&PKTINFO(lx83807_csot_small_test_off),
	&PKTINFO(lx83807_csot_small_test_a4_on),
	&PKTINFO(lx83807_csot_small_mafpc_crc_enable),
	&PKTINFO(lx83807_csot_small_mafpc_luminance_update_1),
	&PKTINFO(lx83807_csot_small_mafpc_luminance_update_2),
	&PKTINFO(lx83807_csot_small_mafpc_self_mask_off),
	&DLYINFO(lx83807_csot_small_crc_wait),
	&lx8380x_restbl[RES_MAFPC_CRC],
	&PKTINFO(lx83807_csot_small_mafpc_crc_off),
	&PKTINFO(lx83807_csot_small_mafpc_disable),
	&PKTINFO(lx83807_csot_small_mafpc_self_mask_on),
	&PKTINFO(lx83807_csot_small_test_off),
	&PKTINFO(lx83807_csot_small_test_a6_on),
	&PKTINFO(lx83807_csot_small_mafpc_crc_bist_off),
	&PKTINFO(lx83807_csot_small_test_off),
};

static u8 LX83807_CSOT_SMALL_MAFPC_SCALE_GPARAM[] = {
	0xA0,
	0x00, 0x09, 0x87,
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_mafpc_scale_gparam, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MAFPC_SCALE_GPARAM, 0);


static u8 LX83807_CSOT_SMALL_MAFPC_SCALE[] = {
	0x87,
	0xFF, 0xFF, 0xFF,
};
static DEFINE_PKTUI(lx83807_csot_small_mafpc_scale, &lx83807_csot_small_maptbl[MAFPC_SCALE_MAPTBL], 1);
static DEFINE_VARIABLE_PACKET(lx83807_csot_small_mafpc_scale, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_MAFPC_SCALE, 0);
#endif

static DEFINE_FUNC_BASED_COND(lx83807_csot_small_cond_is_normal_to_hbm,
	&DDI_FUNC(LX8380X_COND_IS_NORMAL_TO_HBM));
static DEFINE_FUNC_BASED_COND(lx83807_csot_small_cond_is_smooth_dimming_available,
	&DDI_FUNC(LX8380X_COND_IS_SMOOTH_DIMMING_AVAILABLE));
static struct seqinfo SEQINFO(lx83807_csot_small_set_bl_param_seq);

static void *lx83807_csot_small_init_cmdtbl[] = {
	&PKTINFO(lx83807_csot_small_test_a6_on),
	&DLYINFO(lx83807_csot_small_wait_1msec),
	&PKTINFO(lx83807_csot_small_ref_voltage_mode),
	&DLYINFO(lx83807_csot_small_wait_1msec),
	&PKTINFO(lx83807_csot_small_touch_vsync_on),

	&PKTINFO(lx83807_csot_small_test_off),
	&PKTINFO(lx83807_csot_small_caset),
	&PKTINFO(lx83807_csot_small_paset),
	&PKTINFO(lx83807_csot_small_clock_gating),
	&PKTINFO(lx83807_csot_small_osc_mipi_speed),
	&DLYINFO(lx83807_csot_small_wait_1msec),	
	&PKTINFO(lx83807_csot_small_lfd_30),
	&PKTINFO(lx83807_csot_small_elvss_temp),
	&PKTINFO(lx83807_csot_small_wrctrld),
	&PKTINFO(lx83807_csot_small_acl_control),
	&PKTINFO(lx83807_csot_small_te_on),
	&PKTINFO(lx83807_csot_small_sleep_out),
	&TIMER_DLYINFO_BEGIN(lx83807_csot_small_wait_sleep_out_120msec),

#ifdef CONFIG_USDM_PANEL_MAFPC
	&DLYINFO(lx83807_csot_small_wait_10msec),
	&PKTINFO(lx83807_csot_small_test_a4_on),
	&PKTINFO(lx83807_csot_small_mafpc_gram_enable),
	&PKTINFO(lx83807_csot_small_mafpc_sf_sel_enable),
	&DLYINFO(lx83807_csot_small_wait_1usec),
	&PKTINFO(lx83807_csot_small_mafpc_disable),
	&DLYINFO(lx83807_csot_small_wait_17msec),
	&PKTINFO(lx83807_small_mafpc_default_img),
	&DLYINFO(lx83807_csot_small_wait_100usec),
	&PKTINFO(lx83807_csot_small_mafpc_sf_sel_disable),
	&PKTINFO(lx83807_csot_small_test_off),
#endif
	&TIMER_DLYINFO(lx83807_csot_small_wait_sleep_out_120msec),

	&PKTINFO(lx83807_csot_small_test_a3_on),
	&PKTINFO(lx83807_csot_small_init_smooth_dimming_frame),
	&PKTINFO(lx83807_csot_small_test_a1_on),
	&PKTINFO(lx83807_csot_small_te_setting_1),	
	&PKTINFO(lx83807_csot_small_test_off),
	&SEQINFO(lx83807_csot_small_set_bl_param_seq),
	&PKTINFO(lx83807_csot_small_test_a4_on),
	&PKTINFO(lx83807_csot_small_mcd_fskip_cg_enable),
	&PKTINFO(lx83807_csot_small_test_off),	
#ifdef CONFIG_USDM_PANEL_COPR
	&SEQINFO(lx83807_csot_small_set_copr_seq),
#endif
};

static void *lx83807_csot_small_display_on_cmdtbl[] = {
#ifdef CONFIG_USDM_PANEL_MAFPC
	&PKTINFO(lx83807_csot_small_test_a4_on),
	&PKTINFO(lx83807_csot_small_mafpc_enable),
	&PKTINFO(lx83807_csot_small_gpara_enable),
	&PKTINFO(lx83807_csot_small_mafpc_scale_gparam),
	&PKTINFO(lx83807_csot_small_mafpc_scale),
	&PKTINFO(lx83807_csot_small_gpara_disable),
	&PKTINFO(lx83807_csot_small_test_off),
#endif
	&PKTINFO(lx83807_csot_small_display_on),
	&DLYINFO(lx83807_csot_small_wait_20msec),
};

static void *lx83807_csot_small_display_off_cmdtbl[] = {
	&PKTINFO(lx83807_csot_small_display_off),
};

static void *lx83807_csot_small_check_condition_cmdtbl[] = {
	&lx8380x_dmptbl[DUMP_ERROR_FLAG],
	&lx8380x_dmptbl[DUMP_RDDPM],
};

static void *lx83807_csot_small_exit_cmdtbl[] = {
	&lx8380x_dmptbl[DUMP_ERROR_FLAG],
	&lx8380x_dmptbl[DUMP_RDDPM_SLEEP_IN],
#ifdef CONFIG_USDM_PANEL_DPUI
	&lx8380x_dmptbl[DUMP_DSI_ERR],
	&lx8380x_dmptbl[DUMP_SELF_DIAG],
#endif
	&PKTINFO(lx83807_csot_small_display_off),
	&DLYINFO(lx83807_csot_small_wait_20msec),
	&PKTINFO(lx83807_csot_small_sleep_in),
	&DLYINFO(lx83807_csot_small_wait_133msec),
};

static void *lx83807_csot_small_set_bl_param_cmdtbl[] = {
#ifdef CONFIG_USDM_PANEL_MAFPC
	&PKTINFO(lx83807_csot_small_test_a4_on),
	&PKTINFO(lx83807_csot_small_gpara_enable),
	&PKTINFO(lx83807_csot_small_mafpc_scale_gparam),
	&PKTINFO(lx83807_csot_small_mafpc_scale),
	&PKTINFO(lx83807_csot_small_gpara_disable),
	&PKTINFO(lx83807_csot_small_test_off),
#endif
	&PKTINFO(lx83807_csot_small_test_a3_on),
	&CONDINFO_IF(lx83807_csot_small_cond_is_smooth_dimming_available),
		&PKTINFO(lx83807_csot_small_bl_smooth_dimming_frame),
	&CONDINFO_FI(lx83807_csot_small_cond_is_smooth_dimming_available),
	&PKTINFO(lx83807_csot_small_test_off),
	&PKTINFO(lx83807_csot_small_wrctrld),
	&PKTINFO(lx83807_csot_small_wrdisbv),
#if 0
	&CONDINFO_IF(lx83807_csot_small_cond_is_normal_to_hbm),
		&PKTINFO(lx83807_csot_small_test_a3_on),
		&PKTINFO(lx83807_csot_small_acl_for_hbm_1),
		&PKTINFO(lx83807_csot_small_test_off),
		&PKTINFO(lx83807_csot_small_acl_for_hbm_2),
		&DLYINFO(lx83807_csot_small_wait_34msec),
	&CONDINFO_FI(lx83807_csot_small_cond_is_normal_to_hbm),
#endif	
	&PKTINFO(lx83807_csot_small_acl_control),
	&PKTINFO(lx83807_csot_small_elvss_temp),
};
static DEFINE_SEQINFO(lx83807_csot_small_set_bl_param_seq, lx83807_csot_small_set_bl_param_cmdtbl);

static void *lx83807_csot_small_set_bl_cmdtbl[] = {
	&SEQINFO(lx83807_csot_small_set_bl_param_seq),
};

static void *lx83807_csot_small_dummy_cmdtbl[] = {
	NULL,
};

static void *lx83807_csot_small_res_init_cmdtbl[] = {
	&PKTINFO(lx83807_csot_small_test_a6_on),
	&lx8380x_restbl[RES_CHIP_ID],
	&PKTINFO(lx83807_csot_small_test_off),
	&lx8380x_restbl[RES_ID],
	&lx8380x_restbl[RES_CSOT_CELL_ID],
};

static void *lx83807_csot_small_alpm_enter_cmdtbl[] = {
	&PKTINFO(lx83807_csot_small_test_off),
	&PKTINFO(lx83807_csot_small_lfd_1),
#if defined(CONFIG_USDM_FACTORY)
	&PKTINFO(lx83807_csot_small_factory_lpm_wrdisbv),
#else
	&PKTINFO(lx83807_csot_small_wrdisbv),
#endif
};

static void *lx83807_csot_small_alpm_exit_cmdtbl[] = {
	&PKTINFO(lx83807_csot_small_test_off),
	&PKTINFO(lx83807_csot_small_lfd_30),
#if defined(CONFIG_USDM_FACTORY)
	&SEQINFO(lx83807_csot_small_set_bl_param_seq),
#endif
};

static void *lx83807_csot_small_mcd_on_cmdtbl[] = {
	&PKTINFO(lx83807_csot_small_test_off),
	&PKTINFO(lx83807_csot_small_mcd_on_1),
};

static void *lx83807_csot_small_mcd_off_cmdtbl[] = {
	&PKTINFO(lx83807_csot_small_test_off),
	&PKTINFO(lx83807_csot_small_mcd_off_1),
};

static void *lx83807_csot_small_set_fps_cmdtbl[] = {
	&SEQINFO(lx83807_csot_small_set_bl_param_seq),
};


#if defined(CONFIG_USDM_LPD_AUTO_BR)
static void *lx83807_csot_small_lpd_br_cmdtbl[] = {
#ifdef CONFIG_USDM_PANEL_MAFPC
	&PKTINFO(lx83807_csot_small_test_a4_on),
	&PKTINFO(lx83807_csot_small_gpara_enable),
	&PKTINFO(lx83807_csot_small_mafpc_scale_gparam),
	&PKTINFO(lx83807_csot_small_mafpc_scale),
	&PKTINFO(lx83807_csot_small_gpara_disable),
	&PKTINFO(lx83807_csot_small_test_off),
#endif
	&PKTINFO(lx83807_csot_small_wrdisbv),
};
#endif

static struct seqinfo lx83807_csot_small_seqtbl[] = {
	SEQINFO_INIT(PANEL_INIT_SEQ, lx83807_csot_small_init_cmdtbl),
	SEQINFO_INIT(PANEL_RES_INIT_SEQ, lx83807_csot_small_res_init_cmdtbl),
	SEQINFO_INIT(PANEL_SET_BL_SEQ, lx83807_csot_small_set_bl_cmdtbl),

	SEQINFO_INIT(PANEL_DISPLAY_ON_SEQ, lx83807_csot_small_display_on_cmdtbl),
	SEQINFO_INIT(PANEL_DISPLAY_OFF_SEQ, lx83807_csot_small_display_off_cmdtbl),
	SEQINFO_INIT(PANEL_DISPLAY_MODE_SEQ, lx83807_csot_small_set_fps_cmdtbl),

	SEQINFO_INIT(PANEL_CHECK_CONDITION_SEQ, lx83807_csot_small_check_condition_cmdtbl),
	SEQINFO_INIT(PANEL_DUMP_SEQ, lx83807_csot_small_check_condition_cmdtbl),

	SEQINFO_INIT(PANEL_EXIT_SEQ, lx83807_csot_small_exit_cmdtbl),
	SEQINFO_INIT(PANEL_MCD_ON_SEQ, lx83807_csot_small_mcd_on_cmdtbl),
	SEQINFO_INIT(PANEL_MCD_OFF_SEQ, lx83807_csot_small_mcd_off_cmdtbl),
	SEQINFO_INIT(PANEL_ALPM_SET_BL_SEQ, lx83807_csot_small_alpm_enter_cmdtbl),
	SEQINFO_INIT(PANEL_ALPM_EXIT_SEQ, lx83807_csot_small_alpm_exit_cmdtbl),
#ifdef CONFIG_USDM_PANEL_MAFPC
	SEQINFO_INIT(PANEL_MAFPC_IMG_SEQ, lx83807_csot_small_mafpc_image_cmdtbl),
	SEQINFO_INIT(PANEL_MAFPC_CHECKSUM_SEQ, lx83807_csot_small_mafpc_check_cmdtbl),
	SEQINFO_INIT(PANEL_MAFPC_ON_SEQ, lx83807_csot_small_mafpc_on_cmdtbl),
	SEQINFO_INIT(PANEL_MAFPC_OFF_SEQ, lx83807_csot_small_mafpc_off_cmdtbl),
#endif
#if defined(CONFIG_USDM_LPD_AUTO_BR)
	SEQINFO_INIT(PANEL_LPD_BR_SEQ, lx83807_csot_small_lpd_br_cmdtbl),
#endif


	SEQINFO_INIT(PANEL_DUMMY_SEQ, lx83807_csot_small_dummy_cmdtbl),
};

struct common_panel_info lx83807_csot_small_panel_info = {
	.ldi_name = "lx83807",
	.name = "lx83807_watch6_small",
	.model = "AMB675TG01",
	.vendor = "CSO",
	.id = 0x400000,
	.rev = 0,
	.ddi_props = {
		.err_fg_recovery = false,
		.support_vrr = true,
	},
	.ddi_ops = {
		.get_cell_id = lx83807_get_cell_id,
		.get_manufacture_date = lx83807_get_manufacture_date,
		.get_octa_id = lx83805_get_octa_id,
		.get_manufacture_code = lx8380x_get_manufacture_code,
	},
#if defined(CONFIG_USDM_PANEL_DISPLAY_MODE)
	.common_panel_modes = &lx83807_csot_small_display_modes,
#endif
	.mres = {
		.nr_resol = ARRAY_SIZE(lx83807_csot_small_default_resol),
		.resol = lx83807_csot_small_default_resol,
	},
	.vrrtbl = lx83807_csot_small_default_vrrtbl,
	.nr_vrrtbl = ARRAY_SIZE(lx83807_csot_small_default_vrrtbl),
	.maptbl = lx83807_csot_small_maptbl,
	.nr_maptbl = ARRAY_SIZE(lx83807_csot_small_maptbl),
	.seqtbl = lx83807_csot_small_seqtbl,
	.nr_seqtbl = ARRAY_SIZE(lx83807_csot_small_seqtbl),
	.rditbl = lx8380x_rditbl,
	.nr_rditbl = ARRAY_SIZE(lx8380x_rditbl),
	.restbl = lx8380x_restbl,
	.nr_restbl = ARRAY_SIZE(lx8380x_restbl),
	.dumpinfo = lx8380x_dmptbl,
	.nr_dumpinfo = ARRAY_SIZE(lx8380x_dmptbl),
	.panel_dim_info = {
		[PANEL_BL_SUBDEV_TYPE_DISP] = &lx83807_csot_panel_dimming_info,
	},
#ifdef CONFIG_USDM_PANEL_COPR
	.copr_data = &lx83807_csot_small_copr_data,
#endif
#ifdef CONFIG_USDM_PANEL_SELF_DISPLAY
	.aod_tune = &lx83807_csot_small_aod,
#endif
#ifdef CONFIG_USDM_PANEL_MAFPC
	.mafpc_info = &lx83807_csot_small_mafpc,
#endif
};
#endif /* __LX83807_LX83807_CSOT_SMALL_PANEL_H__ */
