/*
 * sb_adc.h
 * Samsung Mobile Battery ADC Header
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

#ifndef __SB_ADC_H
#define __SB_ADC_H __FILE__

#include <linux/err.h>

#define BATT_TEMP	"batt_temp"
#define BLK_TEMP	"blk_temp"

struct list_head;
struct device_node;

struct sb_adc;
struct sb_adc_param_list {
	const char *name;

	struct list_head list;
	unsigned int count;
};

typedef int (*sb_adc_it)(void *pdata, int *data);

#define SB_ADC_DISABLE	(-3801)
#if IS_ENABLED(CONFIG_SB_ADC)
struct sb_adc *sb_adc_add(const char *name, void *pdata, sb_adc_it adc_it);

void sb_adc_del(struct sb_adc *adc);
int sb_adc_get_adc(struct sb_adc *adc, int *data);
int sb_adc_get_value(struct sb_adc *adc, int *value);
int sb_adc_get_adc_now(struct sb_adc *adc, int *data);
int sb_adc_get_value_now(struct sb_adc *adc, int *value);
int sb_adc_update(struct sb_adc *adc);

void sb_adc_delf(const char *name);
int sb_adc_get_adcf(const char *name, int *data);
int sb_adc_get_valuef(const char *name, int *value);
int sb_adc_get_adc_nowf(const char *name, int *data);
int sb_adc_get_value_nowf(const char *name, int *value);
int sb_adc_updatef(const char *name);

struct sb_adc *sb_adc_get(const char *name);
int sb_adc_set(const char *name, bool forced, int data, int value);

void sb_adc_all_update(void);

struct sb_adc_param_list *sb_adc_get_param_list(struct device_node *np, const char *name);
void sb_adc_free_param_list(struct sb_adc_param_list *list);
unsigned int sb_adc_check_param_list(struct sb_adc_param_list *param_list);
#else
static inline struct sb_adc *sb_adc_add(const char *name, void *pdata, sb_adc_it adc_it)
{ return ERR_PTR(SB_ADC_DISABLE); }

static inline void sb_adc_del(struct sb_adc *adc)
{}
static inline int sb_adc_get_adc(struct sb_adc *adc, int *data)
{ return SB_ADC_DISABLE; }
static inline int sb_adc_get_value(struct sb_adc *adc, int *value)
{ return SB_ADC_DISABLE; }
static inline int sb_adc_get_adc_now(struct sb_adc *adc, int *data)
{ return SB_ADC_DISABLE; }
static inline int sb_adc_get_value_now(struct sb_adc *adc, int *value)
{ return SB_ADC_DISABLE; }
static inline int sb_adc_update(struct sb_adc *adc)
{ return SB_ADC_DISABLE; }

static inline void sb_adc_delf(const char *name)
{}
static inline int sb_adc_get_adcf(const char *name, int *data)
{ return SB_ADC_DISABLE; }
static inline int sb_adc_get_valuef(const char *name, int *value)
{ return SB_ADC_DISABLE; }
static inline int sb_adc_get_adc_nowf(const char *name, int *data)
{ return SB_ADC_DISABLE; }
static inline int sb_adc_get_value_nowf(const char *name, int *value)
{ return SB_ADC_DISABLE; }
static inline int sb_adc_updatef(const char *name)
{ return SB_ADC_DISABLE; }

static inline struct sb_adc *sb_adc_get(const char *name)
{ return ERR_PTR(SB_ADC_DISABLE); }
static inline int sb_adc_set(const char *name, bool forced, int data, int value)
{ return SB_ADC_DISABLE; }

static inline void sb_adc_all_update(void)
{}

static inline struct sb_adc_param_list *sb_adc_get_param_list(struct device_node *np, const char *name)
{ return ERR_PTR(SB_ADC_DISABLE); }
static inline void sb_adc_free_param_list(struct sb_adc_param_list *list)
{}
static inline unsigned int sb_adc_check_param_list(struct sb_adc_param_list *param_list)
{ return SB_ADC_DISABLE; }
#endif

#endif /* __SB_ADC_H */

