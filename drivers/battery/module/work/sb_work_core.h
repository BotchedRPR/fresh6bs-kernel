/*
 * sb_work_core.h
 * Samsung Mobile Battery Work Core Header
 *
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

#ifndef __SB_WORK_CORE_H
#define __SB_WORK_CORE_H __FILE__

#include <linux/list.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/workqueue.h>

#include <linux/battery/sb_def.h>

#define SB_WORK_NAME		"sb-work"
#define work_log(str, ...)  pr_info("[sb-work]:%s: "str, __func__, ##__VA_ARGS__)

enum {
	SB_WORK_TYPE_ROOT = 0,
	SB_WORK_TYPE_WQ,
	SB_WORK_TYPE_OP,
	SB_WORK_TYPE_WORK,
};

struct sb_work_obj {
	struct mutex		lock;
	struct list_head	head;
	struct list_head	list;
	struct list_head	junk;
	struct sb_work_obj	*parent;

	const char		*name;
	union {
		unsigned int value;

		struct {
			unsigned type : 3,
				remove : 1;
		} base;

		struct {
			unsigned type : 3,
				remove : 1,
				single : 1;
		} op;
	} state;
	atomic_t		act_cnt;
};

struct sb_work_wq {
	struct sb_work_obj	obj;

	struct workqueue_struct	*wq;
	struct wakeup_source	*ws;
};

struct sb_work_op {
	struct sb_work_obj	obj;

	void 				*pdata;
	sb_func 			cb_work;
	sb_event_func 		cb_done;
};

struct sb_work {
	struct sb_work_obj	obj;

	struct delayed_work	work;
	wait_queue_head_t	wait;
	sb_data				data;
};

struct sb_work_offset {
	unsigned int max_active_cnt;
	unsigned int max_work_cnt;
	unsigned int max_wait_delay;
};

struct sb_work_dev {
	struct device *dev;
	
	/* offset */
	struct sb_work_offset offset;

	struct sb_work_wq *g_wq;

	/* object list head */
	struct sb_work_obj	wq;
	struct sb_work_obj	work;
	struct sb_work_obj	junk;
};

/* Work Dev APIs */
struct sb_work_dev *get_work_dev(void);

/* Work Core APIs */
void obj_lock(struct sb_work_obj *obj);
void obj_unlock(struct sb_work_obj *obj);

void *convert_obj(struct sb_work_obj *obj, unsigned int type);

void init_obj(struct sb_work_obj *obj, const char *name, unsigned int type);
void set_obj_state(struct sb_work_obj *obj, unsigned int state, unsigned int mask);
unsigned int get_obj_state(struct sb_work_obj *obj);
void set_obj_act(struct sb_work_obj *obj, bool act);
unsigned int get_obj_act(struct sb_work_obj *obj);
bool check_obj_act(struct sb_work_obj *obj, bool act);
void add_obj_act(struct sb_work_obj *to_obj, struct sb_work_obj *from_obj);
void sub_obj_act(struct sb_work_obj *to_obj, struct sb_work_obj *from_obj);
bool is_obj_act(struct sb_work_obj *obj);

struct sb_work_obj *find_obj(struct sb_work_obj *parent, const char *name);
struct sb_work_obj *pop_obj(struct sb_work_obj *parent);
struct sb_work_obj *get_first_obj(struct sb_work_obj *parent);
void add_obj(struct sb_work_obj *obj, struct sb_work_obj *parent);
void del_obj(struct sb_work_obj *obj);
void remove_obj(struct sb_work_obj *obj);

struct sb_work_wq *create_wq(struct device *dev, const char *name, unsigned int act_cnt);
void destroy_wq(struct sb_work_wq *wq);
void pm_set_wq(struct sb_work_wq *wq);
void pm_relax_wq(struct sb_work_wq *wq);
struct sb_work_op *create_op(const char *name,
		void *pdata, unsigned int flag, sb_func cb_work, sb_event_func cb_done);
void destroy_op(struct sb_work_op *op);
void cancel_op(struct sb_work_op *op);
struct sb_work *create_work(void);
void destroy_work(struct sb_work *work);
void push_work(struct sb_work_dev *wdev, struct sb_work *work);
struct sb_work *pop_work(struct sb_work_dev *wdev);
void queue_dwork(struct sb_work_wq *wq, struct sb_work *work, unsigned int delay);
void queue_dwork_wait(struct sb_work_wq *wq, struct sb_work *work, unsigned int delay);

#endif /* __SB_WORK_CORE_H */
