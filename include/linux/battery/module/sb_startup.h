/*
 * sb_startup.h
 * Samsung Mobile Battery Startup Header
 *
 * Copyright (C) 2023 Samsung Electronics, Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __SB_STARTUP_H
#define __SB_STARTUP_H __FILE__

#include <linux/battery/sb_def.h>

#define SB_STARTUP_DISABLE	(-3804)
#if IS_ENABLED(CONFIG_SB_STARTUP)
int sb_startup_register(void *pdata, sb_event_func cb_func, unsigned int event, unsigned int time);

bool sb_startup_is_activated(void);
#else
static inline int sb_startup_register(void *pdata, sb_event_func cb_func, unsigned int event, unsigned int time)
{ return SB_STARTUP_DISABLE; }

static inline bool sb_startup_is_activated(void)
{ return false; }
#endif

#endif /* __SB_STARTUP_H */
