/*
 *  sb_startup.c
 *  Samsung Mobile Battery Startup Module
 *
 *  Copyright (C) 2023 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/alarmtimer.h>

#include <linux/battery/sb_pqueue.h>
#include <linux/battery/module/sb_startup.h>

#define SB_MODULE_NAME	"sb-startup"
#define sup_log(str, ...) pr_info("[%s]:%s: "str, SB_MODULE_NAME, __func__, ##__VA_ARGS__)

#define DEFAULT_START_TIME	0
#define DEFAULT_SET_TIME	60	/* 1 min */
#define DEFAULT_QUEUE_SIZE	10

struct sup_data {
	void *pdata;
	sb_event_func	cb_func;
	unsigned int event;
	unsigned int time;
};

struct sb_startup {
	struct alarm sup_alarm;

	struct workqueue_struct *wq;
	struct wakeup_source *ws;
	struct delayed_work sup_work;

	struct mutex lock;
	struct sb_pqueue *pq;

	atomic_t set_time;
	atomic_t now_time;
};

static bool sup_cmp(sb_data data1, sb_data data2)
{
	struct sup_data *sd1, *sd2;

	sd1 = (struct sup_data *)data1;
	sd2 = (struct sup_data *)data2;

	return (sd1->time < sd2->time);
}

static enum alarmtimer_restart sup_alarm(struct alarm *alarm, ktime_t now)
{
	struct sb_startup *sup = container_of(alarm, struct sb_startup, sup_alarm);
	struct timespec64 ts;

	ts = ktime_to_timespec64(now);
	atomic_set(&sup->now_time, (int)ts.tv_sec);
	sup_log("now = %lld, now time = %d, set time = %d\n",
		ts.tv_sec, atomic_read(&sup->now_time), atomic_read(&sup->set_time));

	__pm_stay_awake(sup->ws);
	queue_delayed_work(sup->wq, &sup->sup_work, 0);

	return ALARMTIMER_NORESTART;
}

static void sup_work(struct work_struct *work)
{
	struct sb_startup *sup = container_of(work, struct sb_startup, sup_work.work);
	struct sup_data *sdata;
	sb_data ptr_sd;
	int ret = 0;

	ret = sb_pq_pop(sup->pq, &ptr_sd);
	if (ret < 0)
		goto end_work;

	sdata = (struct sup_data *)ptr_sd;
	sdata->cb_func(sdata->pdata, sdata->event, atomic_read(&sup->now_time));
	kfree(sdata);

	ret = sb_pq_top(sup->pq, &ptr_sd);
	if (ret < 0)
		goto end_work;

	mutex_lock(&sup->lock);
	sdata = (struct sup_data *)ptr_sd;
	atomic_set(&sup->set_time, sdata->time);
	alarm_start(&sup->sup_alarm,
		ktime_add(0, ktime_set(sdata->time, 0)));
	mutex_unlock(&sup->lock);

end_work:
	__pm_relax(sup->ws);
}

static struct sb_startup *sup_inst(void)
{
	static struct sb_startup *sup;

	if (sup)
		return sup;

	sup = kzalloc(sizeof(struct sb_startup), GFP_KERNEL);
	if (sup == NULL)
		goto err_alloc;

	sup->wq = create_singlethread_workqueue(SB_MODULE_NAME);
	if (!sup->wq)
		goto err_wq;

	sup->ws = wakeup_source_register(NULL, SB_MODULE_NAME);
	if (!sup->ws)
		goto err_ws;

	sup->pq = sb_pq_create((PQF_REMOVE & PQF_PRIORITY), DEFAULT_QUEUE_SIZE, sup_cmp);
	if (IS_ERR(sup->pq))
		goto err_pq;

	mutex_init(&sup->lock);

	alarm_init(&sup->sup_alarm, ALARM_BOOTTIME, sup_alarm);
	INIT_DELAYED_WORK(&sup->sup_work, sup_work);

	atomic_set(&sup->set_time, DEFAULT_START_TIME);
	atomic_set(&sup->now_time, DEFAULT_START_TIME);

	sup_log("done!\n");
	return sup;

err_pq:
	wakeup_source_unregister(sup->ws);
err_ws:
	destroy_workqueue(sup->wq);
err_wq:
	kfree(sup);
err_alloc:
	sup_log("err!\n");
	sup = ERR_PTR(-ENOMEM);
	return sup;
}

static struct sup_data *create_sup_data(void *pdata, sb_event_func cb_func, unsigned int event, unsigned int time)
{
	struct sup_data *sdata;

	if ((pdata == NULL) || (cb_func == NULL))
		return NULL;

	sdata = kzalloc(sizeof(struct sup_data), GFP_KERNEL);
	if (!sdata)
		return NULL;

	sdata->pdata = pdata;
	sdata->cb_func = cb_func;
	sdata->event = event;
	sdata->time = time;
	return sdata;
}

int sb_startup_register(void *pdata, sb_event_func cb_func, unsigned int event, unsigned int time)
{
	struct sb_startup *sup = sup_inst();
	struct sup_data *sdata;
	int ret = 0;

	if (IS_ERR(sup))
		return -ENODEV;

	sdata = create_sup_data(pdata, cb_func, event, time);
	if (!sdata)
		return -ENOMEM;

	ret = sb_pq_push(sup->pq, 0, (sb_data)sdata);
	if (ret < 0) {
		kfree(sdata);
		return ret;
	}

	mutex_lock(&sup->lock);
	if (time < atomic_read(&sup->set_time)) {
		atomic_set(&sup->set_time, time);

		/* alarm restart */
		alarm_cancel(&sup->sup_alarm);
		alarm_start(&sup->sup_alarm,
			ktime_add(0, ktime_set(time, 0)));
	}
	mutex_unlock(&sup->lock);

	return 0;
}
EXPORT_SYMBOL(sb_startup_register);

bool sb_startup_is_activated(void)
{
	struct sb_startup *sup = sup_inst();

	if (IS_ERR(sup))
		return false;

	return (sup->sup_alarm.state & ALARMTIMER_STATE_ENQUEUED);
}
EXPORT_SYMBOL(sb_startup_is_activated);

static int sup_dummy_func(void *pdata, unsigned int event, sb_data data)
{
	sup_log("time = %d\n", (int)data);
	return 0;
}

static int __init sb_startup_init(void)
{
	struct sb_startup *sup = sup_inst();
	int ret = 0;

	if (IS_ERR(sup))
		return -ENODEV;

	ret = sb_startup_register(sup, sup_dummy_func, 0, DEFAULT_SET_TIME);
	sup_log("add dummy data, ret = %d\n", ret);
	if (!ret) {
		mutex_lock(&sup->lock);
		atomic_set(&sup->set_time, DEFAULT_SET_TIME);
		alarm_start(&sup->sup_alarm,
			ktime_add(0, ktime_set(DEFAULT_SET_TIME, 0)));
		mutex_unlock(&sup->lock);
	}

	return 0;
}
module_init(sb_startup_init);

static void __exit sb_startup_exit(void)
{
	struct sb_startup *sup = sup_inst();

	if (IS_ERR(sup))
		return;

	alarm_cancel(&sup->sup_alarm);
	cancel_delayed_work(&sup->sup_work);
	flush_workqueue(sup->wq);
	__pm_relax(sup->ws);
}
module_exit(sb_startup_exit);

MODULE_DESCRIPTION("Samsung Battery StartUp");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
