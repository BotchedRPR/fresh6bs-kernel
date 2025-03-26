/*
 *  sb_work.c
 *  Samsung Mobile Battery Work
 *
 *  Copyright (C) 2022 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/battery/module/sb_work.h>

#include "sb_work_core.h"

void obj_lock(struct sb_work_obj *obj)
{
	mutex_lock(&obj->lock);
}

void obj_unlock(struct sb_work_obj *obj)
{
	mutex_unlock(&obj->lock);
}

void *convert_obj(struct sb_work_obj *obj, unsigned int type)
{
	if (obj == NULL)
		return NULL;

	if (obj->state.base.type != type)
		return NULL;

	switch (type) {
	case SB_WORK_TYPE_WQ:
		return container_of(obj, struct sb_work_wq, obj);
	case SB_WORK_TYPE_OP:
		return container_of(obj, struct sb_work_op, obj);
	case SB_WORK_TYPE_WORK:
		return container_of(obj, struct sb_work, obj);
	}

	return NULL;
}

void init_obj(struct sb_work_obj *obj, const char *name, unsigned int type)
{
	mutex_init(&obj->lock);
	INIT_LIST_HEAD(&obj->head);
	INIT_LIST_HEAD(&obj->list);
	INIT_LIST_HEAD(&obj->junk);
	obj->parent = NULL;

	obj->name = name;
	obj->state.value = 0;
	obj->state.base.type = type;

	atomic_set(&obj->act_cnt, 0);
}

void set_obj_state(struct sb_work_obj *obj, unsigned int state, unsigned int mask)
{
	obj_lock(obj);
	obj->state.value &= (~mask);
	obj->state.value |= (state | mask);
	obj_unlock(obj);
}

unsigned int get_obj_state(struct sb_work_obj *obj)
{
	unsigned int ret = 0;

	obj_lock(obj);
	ret = obj->state.value;
	obj_unlock(obj);

	return ret;
}

void set_obj_act(struct sb_work_obj *obj, bool act)
{
	if (obj == NULL)
		return;

	obj_lock(obj);
	if (act)
		atomic_inc(&obj->act_cnt);
	else if (is_obj_act(obj))
		atomic_dec(&obj->act_cnt);
	else
		atomic_set(&obj->act_cnt, 0);
	obj_unlock(obj);

	set_obj_act(obj->parent, act);
}

bool check_obj_act(struct sb_work_obj *obj, bool act)
{
	bool ret = false;

	obj_lock(obj);
	ret = is_obj_act(obj);
	if (act) {
		atomic_inc(&obj->act_cnt);
		ret = true;
	} else if (ret) {
		atomic_dec(&obj->act_cnt);
	}
	obj_unlock(obj);

	return ret;
}

void add_obj_act(struct sb_work_obj *to_obj, struct sb_work_obj *from_obj)
{
	obj_lock(to_obj);
	obj_lock(from_obj);

	atomic_add(atomic_read(&from_obj->act_cnt), &to_obj->act_cnt);

	obj_unlock(from_obj);
	obj_unlock(to_obj);
}

void sub_obj_act(struct sb_work_obj *to_obj, struct sb_work_obj *from_obj)
{
	obj_lock(to_obj);
	obj_lock(from_obj);

	atomic_sub(atomic_read(&from_obj->act_cnt), &to_obj->act_cnt);

	obj_unlock(from_obj);
	obj_unlock(to_obj);
}

bool is_obj_act(struct sb_work_obj *obj)
{
	return (atomic_read(&obj->act_cnt) >= 1); 
}

struct sb_work_obj *find_obj(struct sb_work_obj *parent, const char *name)
{
	struct sb_work_obj *obj, *ret = NULL;

	obj_lock(parent);
	list_for_each_entry(obj, &parent->head, list) {
		if (!strcmp(obj->name, name)) {
			ret = obj;
			break;
		}
	}
	obj_unlock(parent);

	return ret;
}

struct sb_work_obj *pop_obj(struct sb_work_obj *parent)
{
	struct sb_work_obj *obj = NULL;

	obj_lock(parent);
	if (!list_empty(&parent->head)) {
		obj = list_first_entry(&parent->head, struct sb_work_obj, list);
		list_del(&obj->list);
	}
	obj_unlock(parent);

	return obj;
}

struct sb_work_obj *get_first_obj(struct sb_work_obj *parent)
{
	struct sb_work_obj *obj = NULL;

	obj_lock(parent);
	if (!list_empty(&parent->head))
		obj = list_first_entry(&parent->head, struct sb_work_obj, list);
	obj_unlock(parent);

	return obj;
}

void add_obj(struct sb_work_obj *obj, struct sb_work_obj *parent)
{
	obj_lock(obj);

	obj_lock(parent);
	list_add(&obj->list, &parent->head);
	obj_unlock(parent);

	obj->parent = parent;
	obj_unlock(obj);
}

void del_obj(struct sb_work_obj *obj)
{
	obj_lock(obj);

	if (obj->parent != NULL) {
		struct sb_work_obj *parent = obj->parent;

		obj_lock(parent);
		list_del(&obj->list);
		obj_unlock(parent);
	}
	obj->parent = NULL;

	obj_unlock(obj);
}

void remove_obj(struct sb_work_obj *obj)
{
	struct sb_work_obj *parent = &(get_work_dev()->junk);

	obj_lock(parent);
	list_add(&obj->junk, &parent->head);
	obj_unlock(parent);

	set_obj_state(obj, SB_WORK_REMOVE, SB_WORK_REMOVE);
}

struct sb_work_wq *create_wq(struct device *dev, const char *name, unsigned int act_cnt)
{
	struct sb_work_wq *wq;

	wq = kzalloc(sizeof(struct sb_work_wq), GFP_KERNEL);
	if (!wq)
		return NULL;

	wq->wq = create_workqueue(name);
	if (!wq->wq)
		goto err_wq;
	workqueue_set_max_active(wq->wq, act_cnt);

	wq->ws = wakeup_source_register(dev, name);
	if (!wq->ws)
		goto err_ws;

	init_obj(&wq->obj, name, SB_WORK_TYPE_WQ);
	return wq;

err_ws:
	destroy_workqueue(wq->wq);
err_wq:
	kfree(wq);
	return NULL;
}

void destroy_wq(struct sb_work_wq *wq)
{
	wakeup_source_unregister(wq->ws);
	destroy_workqueue(wq->wq);
	kfree(wq);
}

void pm_set_wq(struct sb_work_wq *wq)
{
	if (wq == NULL)
		return;

	obj_lock(&wq->obj);
	if (is_obj_act(&wq->obj))
		__pm_stay_awake(wq->ws);
	obj_unlock(&wq->obj);
}

void pm_relax_wq(struct sb_work_wq *wq)
{
	if (wq == NULL)
		return;

	obj_lock(&wq->obj);
	if (!is_obj_act(&wq->obj))
		__pm_relax(wq->ws);
	obj_unlock(&wq->obj);
}

struct sb_work_op *create_op(const char *name,
		void *pdata, unsigned int flag, sb_func cb_work, sb_event_func cb_done)
{
	struct sb_work_op *op;

	op = kzalloc(sizeof(struct sb_work_op), GFP_KERNEL);
	if (!op)
		return NULL;

	op->pdata = pdata;
	op->cb_work = cb_work;
	op->cb_done = cb_done;

	init_obj(&op->obj, name, SB_WORK_TYPE_OP);
	return op;
}

void destroy_op(struct sb_work_op *op)
{
	kfree(op);
}

void cancel_op(struct sb_work_op *op)
{
	struct sb_work_dev *wdev = get_work_dev();
	struct sb_work_obj *obj;
	struct sb_work *work;

	while ((obj = pop_obj(&op->obj)) != NULL) {
		set_obj_act(obj, false);

		work = convert_obj(obj, SB_WORK_TYPE_WORK);
		if (!work)
			continue;

		cancel_delayed_work(&work->work);
		push_work(wdev, work);
	}
}

static void cb_work_func(struct work_struct *wk);
struct sb_work *create_work(void)
{
	struct sb_work *work;

	work = kzalloc(sizeof(struct sb_work), GFP_KERNEL);
	if (!work)
		return NULL;

	INIT_DELAYED_WORK(&work->work, cb_work_func);
	init_waitqueue_head(&work->wait);
	work->data = 0;

	init_obj(&work->obj, "sb-work", SB_WORK_TYPE_WORK);
	return work;
}

void destroy_work(struct sb_work *work)
{
	kfree(work);
}

void push_work(struct sb_work_dev *wdev, struct sb_work *work)
{
	add_obj(&work->obj, &wdev->work);
	set_obj_act(&wdev->work, true);
}

struct sb_work *pop_work(struct sb_work_dev *wdev)
{
	struct sb_work_obj *obj;

	obj = pop_obj(&wdev->work);
	set_obj_act(&wdev->work, false);
	return (obj) ?
		convert_obj(obj, SB_WORK_TYPE_WORK) : create_work();
}

void queue_dwork(struct sb_work_wq *wq, struct sb_work *work, unsigned int delay)
{
	pm_set_wq(wq);
	queue_delayed_work(wq->wq, &work->work, msecs_to_jiffies(delay));
}

void queue_dwork_wait(struct sb_work_wq *wq, struct sb_work *work, unsigned int delay)
{
	struct sb_work_dev *wdev = get_work_dev();

	pm_set_wq(wq);
	queue_delayed_work(wq->wq, &work->work, msecs_to_jiffies(delay));
	wait_event_interruptible_timeout(work->wait,
		(!is_obj_act(&work->obj)), delay + wdev->offset.max_wait_delay);
}

void cb_work_func(struct work_struct *wk)
{
	struct sb_work *work = container_of(wk, struct sb_work, work.work);
	struct sb_work_dev *wdev = get_work_dev();
	struct sb_work_wq *wq;
	struct sb_work_op *op;
	int work_ret, done_ret;

	if (!check_obj_act(&work->obj, false))
		goto end_work;

	op = convert_obj(work->obj.parent, SB_WORK_TYPE_OP);
	if (!op)
		goto end_work;

	wq = convert_obj(op->obj.parent, SB_WORK_TYPE_WQ);
	if (!wq)
		goto end_work;

	del_obj(&work->obj);
	work_ret = op->cb_work(op->pdata, work->data);
	done_ret = op->cb_done(op->pdata, work_ret, work->data);
	wake_up_interruptible(&work->wait);
	work_log("[%s] finish(wr = %d, dr = %d)\n", op->obj.name, work_ret, done_ret);
	set_obj_act(&op->obj, false);

	pm_relax_wq(wq);

end_work:
	push_work(wdev, work);
}
