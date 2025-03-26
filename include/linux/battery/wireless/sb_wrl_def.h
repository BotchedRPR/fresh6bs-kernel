/*
 * sb_wrl_def.h
 * Samsung Mobile Wireless Define Header
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

#ifndef __SB_WRL_DEF_H
#define __SB_WRL_DEF_H __FILE__

/* Sec Battery Wireless Charger - charging mode */
enum sb_wrl_charge_mode {
	SB_WRL_CHG_MODE_CC = 0, /* DEFAULT */
	SB_WRL_CHG_MODE_CV,
	SB_WRL_CHG_MODE_HALF_BRIDGE,

	SB_WRL_CHG_MODE_MAX,
};

static inline const char *get_chg_mode_str(int chg_mode)
{
	switch (chg_mode) {
	case SB_WRL_CHG_MODE_CC:
		return "CC";
	case SB_WRL_CHG_MODE_CV:
		return "CV";
	case SB_WRL_CHG_MODE_HALF_BRIDGE:
		return "HB";
	}

	return "Unknown";
}

enum sb_wrl_pad_type {
	SB_WRL_PAD_TYPE_NORMAL	= 0,
	SB_WRL_PAD_TYPE_D2D,
	SB_WRL_PAD_TYPE_MULTI,
	SB_WRL_PAD_TYPE_ILLEGAL,
	SB_WRL_PAD_TYPE_FAST_MULTI,
	SB_WRL_PAD_TYPE_BPACK,
	SB_WRL_PAD_TYPE_FACTORY
};

#define SB_WRL_FACTORY_MODE_TX_ID	0x5F

static inline bool is_normal_pad_type(int pad_type)
{
	return (pad_type == SB_WRL_PAD_TYPE_NORMAL);
}

static inline bool is_d2d_pad_type(int pad_type)
{
	return (pad_type == SB_WRL_PAD_TYPE_D2D);
}

static inline bool is_multi_pad_type(int pad_type)
{
	return (pad_type == SB_WRL_PAD_TYPE_MULTI) ||
		(pad_type == SB_WRL_PAD_TYPE_FAST_MULTI);
}

static inline bool is_illegal_pad_type(int pad_type)
{
	return (pad_type == SB_WRL_PAD_TYPE_ILLEGAL);
}

/* Sec Battery Wireless Charger - TX structure */
#define sb_wrl_tx_type	unsigned long long
struct sb_wrl_tx {
	union {
		/* max size = 8 bytes */
		sb_wrl_tx_type info_data;

		struct {
			unsigned	id : 8,
						recv_id : 1,
						pad_type : 4,
						phm : 1,
						phm_state : 1,
						phm_time : 16,
						auth : 1,
						auth_state : 4,
						rx_pwr_state : 1,
						fw_state : 1,
						tx_pwr_budg : 8,
						darkzone : 1; /* 47b */
		} info;
	};

	sb_wrl_tx_type chg_mode[SB_WRL_CHG_MODE_MAX];
};

static inline void sb_wrl_tx_init(struct sb_wrl_tx *tx)
{
	int i = SB_WRL_CHG_MODE_MAX - 1;

	if (tx == NULL)
		return;

	tx->info_data = 0;
	for (; i >= 0; i--)
		tx->chg_mode[i] = 0;
}

static inline void sb_wrl_tx_copy(struct sb_wrl_tx *dst, struct sb_wrl_tx *src)
{
	int i = SB_WRL_CHG_MODE_MAX - 1;

	if (dst == NULL)
		return;

	if (src == NULL) {
		sb_wrl_tx_init(dst);
		return;
	}

	dst->info_data = src->info_data;
	for (; i >= 0; i--)
		dst->chg_mode[i] = src->chg_mode[i];
}

static inline struct sb_wrl_tx *
sb_wrl_tx_find(struct sb_wrl_tx *tx_table, int size, int tx_id)
{
	int start, end, mid;

	if ((tx_id < 0) || (tx_id > 0xFF) ||
		(tx_table == NULL) || (size <= 0))
		return NULL;

	start = 0; end = size;
	while (start <= end) {
		mid = (start + end) / 2;

		if (tx_table[mid].info.id > tx_id)
			end = mid - 1;
		else if (tx_table[mid].info.id < tx_id)
			start = mid + 1;
		else
			return &tx_table[mid];
	}

	return NULL;
}

#define SB_WRL_HDR_STEP		10
#define SB_WRL_VOUT_STEP	1
#define SB_WRL_ICHG_STEP	4

union sb_wrl_vm {
	/* max size = 8 bytes */
	sb_wrl_tx_type		value;

	struct {
		unsigned	type : 8,
					resv : 24;
	} base;

	struct {
		unsigned	type : 8,
					resv : 24,
					hdr : 8,
					ichg : 8,
					high_vout : 16;
	} dc;

	struct {
		unsigned	type : 8,
					resv : 24,
					low_vout : 16,
					vbat_hdr : 8,
					hdr : 8;
	} bt;

	struct {
		unsigned	type : 8,
					resv : 24,
					vout : 16,
					hdr : 8;
	} fx;
};

#endif /* __SB_WRL_DEF_H */
