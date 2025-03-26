/*
 * sb_work.h
 * Samsung Mobile Battery Work Header
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

#ifndef __SB_WORK_H
#define __SB_WORK_H __FILE__

#include <linux/err.h>
#include <linux/battery/sb_def.h>

/* WORK BIT */
#define SB_WORK_NONE		0
#define SB_WORK_REMOVE		(1 << 3)
#define SB_WORK_SINGLE		(0x1 << 4)
//#define SB_WORK_ERROR		(0xFF << 23)

/* WORK STATE */
#define SB_WORK_STATE_NONE		0
#define SB_WORK_STATE_PENDING	(1 << 8)
#define SB_WORK_STATE_RUNNING	(1 << 9)
#define SB_WORK_STATE_ACTIVE	(SB_WORK_STATE_PENDING | SB_WORK_STATE_RUNNING)

struct device;
struct sb_work_wq;
struct sb_work_op;

#define SB_WORK_DISABLE	(-3803)
#if IS_ENABLED(CONFIG_SB_WORK)
struct sb_work_wq *sb_work_create_wq(struct device *dev, const char *name);
struct sb_work_wq *sb_work_find_wq(const char *name);
int sb_work_clear_wq(struct sb_work_wq *wq);
int sb_work_flush_wq(struct sb_work_wq *wq);
int sb_work_destroy_wq(struct sb_work_wq *wq);

struct sb_work_op *sb_work_create_op(const char *name, struct sb_work_wq *wq,
		void *pdata, unsigned int flag, sb_func cb_work, sb_event_func cb_done);
int sb_work_change_wq(struct sb_work_op *op, struct sb_work_wq *wq);
int sb_work_run(struct sb_work_op *op, sb_data data, unsigned int delay);
struct sb_work_op *sb_work_run_once(void *pdata, sb_data data, unsigned int delay,
		sb_func cb_work, sb_event_func cb_done);
int sb_work_run_wait(struct sb_work_op *op, sb_data data, unsigned int delay);
int sb_work_cancel(struct sb_work_op *op);
int sb_work_destroy_op(struct sb_work_op *op);
int sb_work_get_state(struct sb_work_op *op);
#else
static inline struct sb_work_wq *sb_work_create_wq(struct device *dev, const char *name)
{ return ERR_PTR(SB_WORK_DISABLE); }
static inline struct sb_work_wq *sb_work_find_wq(const char *name)
{ return ERR_PTR(SB_WORK_DISABLE); }
static inline int sb_work_clear_wq(struct sb_work_wq *wq)
{ return SB_WORK_DISABLE; }
static inline int sb_work_flush_wq(struct sb_work_wq *wq)
{ return SB_WORK_DISABLE; }
static inline int sb_work_destroy_wq(struct sb_work_wq *wq)
{ return SB_WORK_DISABLE; }

static inline struct sb_work_op *sb_work_create_op(const char *name, struct sb_work_wq *wq,
		void *pdata, unsigned int flag, sb_func cb_work, sb_event_func cb_done)
{ return ERR_PTR(SB_WORK_DISABLE); }
static inline int sb_work_change_wq(struct sb_work_op *op, struct sb_work_wq *wq)
{ return SB_WORK_DISABLE; }
static inline int sb_work_run(struct sb_work_op *op, sb_data data, unsigned int delay)
{ return SB_WORK_DISABLE; }
static inline struct sb_work_op *sb_work_run_once(void *pdata, sb_data data, unsigned int delay,
		sb_func cb_work, sb_event_func cb_done)
{ return ERR_PTR(SB_WORK_DISABLE); }
static inline int sb_work_run_wait(struct sb_work_op *op, sb_data data, unsigned int delay)
{ return SB_WORK_DISABLE; }
static inline int sb_work_cancel(struct sb_work_op *op)
{ return SB_WORK_DISABLE; }
static inline int sb_work_destroy_op(struct sb_work_op *op)
{ return SB_WORK_DISABLE; }
static inline int sb_work_get_state(struct sb_work_op *op)
{ return SB_WORK_DISABLE; }
#endif

#endif /* __SB_WORK_H */
