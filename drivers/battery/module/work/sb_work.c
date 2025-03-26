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

static int dummy_cb_work(void *pdata, sb_data data)
{
	return 0;
}

static int dummy_cb_done(void *pdata, unsigned int event, sb_data data)
{
	return 0;
}

struct sb_work_wq *sb_work_create_wq(struct device *dev, const char *name)
{
	struct sb_work_dev *wdev = get_work_dev();
	struct sb_work_wq *wq;

	if (name == NULL)
		return ERR_PTR(-EINVAL);

	wq = create_wq(((dev) ? dev : wdev->dev), name, 1);
	if (!wq)
		return ERR_PTR(-ENOMEM);

	add_obj(&wq->obj, &wdev->wq);
	return wq;
}
EXPORT_SYMBOL(sb_work_create_wq);

struct sb_work_wq *sb_work_find_wq(const char *name)
{
	struct sb_work_dev *wdev = get_work_dev();
	struct sb_work_obj *wq_obj = NULL;

	if (name == NULL)
		return ERR_PTR(-EINVAL);

	wq_obj = find_obj(&wdev->wq, name);
	if (!wq_obj)
		return ERR_PTR(-ENODEV);

	if (get_obj_state(wq_obj) | SB_WORK_REMOVE)
		return ERR_PTR(-EACCES);

	return container_of(wq_obj, struct sb_work_wq, obj);
}
EXPORT_SYMBOL(sb_work_find_wq);

int sb_work_clear_wq(struct sb_work_wq *wq)
{
	struct sb_work_obj *obj;
	struct sb_work_op *op;

	if (IS_ERR_OR_NULL(wq))
		return -EINVAL;

	if (get_obj_state(&wq->obj) | SB_WORK_REMOVE)
		return -EACCES;

	if (!is_obj_act(&wq->obj))
		return 0;

	obj_lock(&wq->obj);
	list_for_each_entry(obj, &wq->obj.head, list) {
		op = convert_obj(obj, SB_WORK_TYPE_OP);
		if (!op)
			continue;

		cancel_op(op);
	}
	obj_unlock(&wq->obj);

	pm_relax_wq(wq);
	return 0;
}
EXPORT_SYMBOL(sb_work_clear_wq);

int sb_work_flush_wq(struct sb_work_wq *wq)
{
	if (IS_ERR_OR_NULL(wq))
		return -EINVAL;

	if (get_obj_state(&wq->obj) | SB_WORK_REMOVE)
		return -EACCES;

	if (!is_obj_act(&wq->obj))
		return 0;

	flush_workqueue(wq->wq);
	pm_relax_wq(wq);
	return 0;
}
EXPORT_SYMBOL(sb_work_flush_wq);

int sb_work_destroy_wq(struct sb_work_wq *wq)
{
	if (IS_ERR_OR_NULL(wq))
		return -EINVAL;

	sb_work_clear_wq(wq);
	remove_obj(&wq->obj);
	return 0;
}
EXPORT_SYMBOL(sb_work_destroy_wq);

struct sb_work_op *sb_work_create_op(const char *name, struct sb_work_wq *wq,
		void *pdata, unsigned int flag, sb_func cb_work, sb_event_func cb_done)
{
	struct sb_work_dev *wdev = get_work_dev();
	struct sb_work_op *op;

	if ((name == NULL) || (pdata == NULL))
		return ERR_PTR(-EINVAL);

	op = create_op(name, pdata, flag,
		((cb_work) ? cb_work : dummy_cb_work),
		((cb_done) ? cb_done : dummy_cb_done));
	if (!op)
		return ERR_PTR(-ENOMEM);

	if (IS_ERR_OR_NULL(wq))
		add_obj(&op->obj, &wdev->g_wq->obj);
	else
		add_obj(&op->obj, &wq->obj);
	return op;
}
EXPORT_SYMBOL(sb_work_create_op);

int sb_work_change_wq(struct sb_work_op *op, struct sb_work_wq *wq)
{
	struct sb_work_dev *wdev = get_work_dev();
	struct sb_work_wq *org_wq;

	if (IS_ERR_OR_NULL(op))
		return -EINVAL;

	if (get_obj_state(&op->obj) | SB_WORK_REMOVE)
		return -EACCES;

	org_wq = convert_obj(op->obj.parent, SB_WORK_TYPE_WQ);
	if (!org_wq)
		goto set_new_wq;

	sub_obj_act(&org_wq->obj, &op->obj);
	pm_relax_wq(org_wq);

set_new_wq:
	del_obj(&op->obj);

	if (IS_ERR_OR_NULL(wq))
		add_obj(&op->obj, &wdev->g_wq->obj);
	else
		add_obj(&op->obj, &wq->obj);

	add_obj_act(op->obj.parent, &op->obj);
	return 0;
}
EXPORT_SYMBOL(sb_work_change_wq);

int sb_work_run(struct sb_work_op *op, sb_data data, unsigned int delay)
{
	struct sb_work_dev *wdev = get_work_dev();
	struct sb_work_wq *wq;
	struct sb_work *work;

	if (IS_ERR_OR_NULL(op))
		return -EINVAL;

	if (get_obj_state(&op->obj) | SB_WORK_REMOVE)
		return -EACCES;

	wq = convert_obj(op->obj.parent, SB_WORK_TYPE_WQ);
	if (!wq)
		return -ENODEV;

	if (get_obj_state(&wq->obj) | SB_WORK_REMOVE)
		return -EACCES;

	if ((get_obj_state(&op->obj) | SB_WORK_SINGLE) &&
		is_obj_act(&op->obj))
		cancel_op(op);

	work = pop_work(wdev);
	if (!work)
		return -ENOMEM;

	work->data = data;
	add_obj(&work->obj, &op->obj);
	set_obj_act(&work->obj, true);

	queue_dwork(wq, work, delay);
	return 0;
}
EXPORT_SYMBOL(sb_work_run);

struct sb_work_op *sb_work_run_once(void *pdata, sb_data data, unsigned int delay,
		sb_func cb_work, sb_event_func cb_done)
{
	struct sb_work_dev *wdev = get_work_dev();
	struct sb_work_wq *wq;
	struct sb_work_op *op;
	struct sb_work *work;

	op = sb_work_create_op("sb-work-op", NULL, pdata, SB_WORK_REMOVE, cb_work, cb_done);
	if (IS_ERR_OR_NULL(op))
		return op;

	wq = convert_obj(op->obj.parent, SB_WORK_TYPE_WQ);
	if (!wq)
		goto err_wq;

	work = pop_work(wdev);
	if (!work)
		goto err_work;

	work->data = data;
	remove_obj(&op->obj);
	add_obj(&work->obj, &op->obj);
	set_obj_act(&work->obj, true);

	queue_dwork(wq, work, delay);
	return op;

err_work:
	del_obj(&op->obj);
err_wq:
	destroy_op(op);
	return ERR_PTR(-ENOMEM);	
}
EXPORT_SYMBOL(sb_work_run_once);

int sb_work_run_wait(struct sb_work_op *op, sb_data data, unsigned int delay)
{
	struct sb_work_dev *wdev = get_work_dev();
	struct sb_work_wq *wq;
	struct sb_work_op *temp_op;
	struct sb_work *work;

	if (IS_ERR_OR_NULL(op))
		return -EINVAL;

	if (get_obj_state(&op->obj) | SB_WORK_REMOVE)
		return -EACCES;

	temp_op = sb_work_create_op(op->obj.name, NULL,
		op->pdata, (op->obj.state.value | SB_WORK_REMOVE), op->cb_work, op->cb_done);
	if (!temp_op)
		return -ENOMEM;

	wq = convert_obj(op->obj.parent, SB_WORK_TYPE_WQ);
	if (!wq)
		goto err_wq;

	work = pop_work(wdev);
	if (!work)
		goto err_work;

	work->data = data;
	remove_obj(&temp_op->obj);
	add_obj(&work->obj, &temp_op->obj);
	set_obj_act(&work->obj, true);

	queue_dwork_wait(wq, work, delay);
	return 0;

err_work:
	del_obj(&temp_op->obj);
err_wq:
	destroy_op(temp_op);
	return -ENOMEM;
}
EXPORT_SYMBOL(sb_work_run_wait);

int sb_work_cancel(struct sb_work_op *op)
{
	struct sb_work_wq *wq;

	if (IS_ERR_OR_NULL(op))
		return -EINVAL;

	if (get_obj_state(&op->obj) | SB_WORK_REMOVE)
		return -EACCES;

	if (!is_obj_act(&op->obj))
		return 0;

	cancel_op(op);

	wq = convert_obj(op->obj.parent, SB_WORK_TYPE_WQ);
	if (wq)
		pm_relax_wq(wq);

	return 0;
}
EXPORT_SYMBOL(sb_work_cancel);

int sb_work_destroy_op(struct sb_work_op *op)
{
	if (IS_ERR_OR_NULL(op))
		return -EINVAL;

	if (get_obj_state(&op->obj) | SB_WORK_REMOVE)
		return 0;

	if (!is_obj_act(&op->obj)) {
		del_obj(&op->obj);
		destroy_op(op);
		return 0;
	}

	remove_obj(&op->obj);
	return 0;
}
EXPORT_SYMBOL(sb_work_destroy_op);

int sb_work_get_state(struct sb_work_op *op)
{
	if (IS_ERR_OR_NULL(op))
		return -EINVAL;

	if (!is_obj_act(&op->obj))
		return SB_WORK_STATE_NONE;

	if (get_obj_state(&op->obj) | SB_WORK_SINGLE) {
		struct sb_work *work;
		unsigned int work_state;

		work = convert_obj(get_first_obj(&op->obj), SB_WORK_TYPE_WORK);
		if (!work)
			return SB_WORK_STATE_NONE;

		work_state = (work_busy(&work->work.work) & (WORK_BUSY_PENDING | WORK_BUSY_RUNNING));
		switch (work_state) {
		case (WORK_BUSY_PENDING | WORK_BUSY_RUNNING):
			return SB_WORK_STATE_ACTIVE;
		case WORK_BUSY_PENDING:
			return SB_WORK_STATE_PENDING;
		case WORK_BUSY_RUNNING:
			return SB_WORK_STATE_RUNNING;
		}

		return SB_WORK_STATE_NONE;
	}

	return SB_WORK_STATE_ACTIVE;
}
EXPORT_SYMBOL(sb_work_get_state);
