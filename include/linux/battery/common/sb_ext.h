/*
 * sb_ext.h
 * Samsung Mobile Battery Extern Header
 *
 * Copyright (C) 2022 Samsung Electronics, Inc.
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

#ifndef __SB_EXTERN_H
#define __SB_EXTERN_H __FILE__

#include <linux/of.h>

#if IS_ENABLED(CONFIG_SEC_ABC)
#include <linux/sti/abc_common.h>
#else
#define sec_abc_send_event(str)
#endif

#if IS_ENABLED(CONFIG_BATTERY_SAMSUNG)
const char *sb_get_ct_str(int cable_type);
const char *sb_get_cm_str(unsigned int charging_mode);
const char *sb_get_bst_str(int status);
const char *sb_get_hl_str(int health);
const char *sb_get_tz_str(int thermal_zone);

int sb_get_misc_test(void);
int sb_get_factory_mode(void);
int sb_get_boot_mode(void);
int sb_get_lpcharge(void);
int sb_get_wirelessd(void);
int sb_get_fg_reset(void);
const char *sb_get_sales_code(void);
#else
static inline const char *sb_get_ct_str(int cable_type)
{ return "Unknown"; }
static inline const char *sb_get_cm_str(unsigned int charging_mode)
{ return "Unknown"; }
static inline const char *sb_get_bst_str(int status)
{ return "Unknown"; }
static inline const char *sb_get_hl_str(int health)
{ return "Unknown"; }
static inline const char *sb_get_tz_str(int thermal_zone)
{ return "Unknown"; }

static inline int sb_get_misc_test(void)
{ return 0; }
static inline int sb_get_factory_mode(void)
{ return 0; }
static inline int sb_get_boot_mode(void)
{ return 0; }
static inline int sb_get_lpcharge(void)
{ return 0; }
static inline int sb_get_wirelessd(void)
{ return 0; }
static inline int sb_get_fg_reset(void)
{ return 0; }
static inline const char *sb_get_sales_code(void)
{ return "Unknown"; }
#endif

#define sb_of_parse_u32(np, pdata, value, deft) \
	({ \
		int ret = 0; \
		ret = of_property_read_u32(np, #value, (unsigned int *)&pdata->value); \
		if (!ret) \
			pr_info("%s: %s - write "#value" to %d\n", __func__, np->name, pdata->value); \
		else \
			pdata->value = deft; \
		ret;\
	})

#define sb_of_parse_str(np, pdata, value) \
	({ \
		int ret = 0; \
		ret = of_property_read_string(np, #value, (const char **)&pdata->value); \
		if (!ret) \
			pr_info("%s: %s - write "#value" to %s\n", __func__, np->name, pdata->value); \
		ret;\
	})

#define sb_of_parse_bool(np, pdata, value) \
	({ \
		pdata->value = of_property_read_bool(np, #value); \
		pr_info("%s: %s - write "#value" to %d\n", __func__, np->name, pdata->value); \
	})

#endif /* __SB_EXTERN_H */
