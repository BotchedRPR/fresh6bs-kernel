/*
 *  sb_notify.c
 *  Samsung Mobile Battery Notify
 *
 *  Copyright (C) 2022 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/workqueue.h>

#include <linux/battery/sb_notify.h>
#include <linux/battery/sb_pqueue.h>

#define SB_NOTIFY_NAME		"sb-notify"
#define SB_NOTIFY_PQ_SIZE	20

#define noti_log(str, ...)	pr_info("[sb-notify]:%s: "str, __func__, ##__VA_ARGS__)

struct sbn_dev {
	const char *name;
	enum sb_dev_type type;

	struct notifier_block *nb;
	struct list_head list;
};

struct sbn {
	struct blocking_notifier_head nb_head;

	struct list_head dev_list;
	unsigned int dev_count;

	bool init_state;

	struct delayed_work		work;
	struct workqueue_struct	*wq;
	struct wakeup_source	*ws;
	struct sb_pqueue		*pq;
};

struct sbn_data {
	enum sbn_type type;
	sb_data data;
};

static struct sbn sb_notify;
static DEFINE_MUTEX(noti_lock);

static struct sbn_dev *sbn_create_device(const char *name, enum sb_dev_type type)
{
	struct sbn_dev *ndev;

	ndev = kzalloc(sizeof(struct sbn_dev), GFP_KERNEL);
	if (!ndev)
		return NULL;

	ndev->name = name;
	ndev->type = type;
	return ndev;
}

static void sbn_destroy_device(struct sbn_dev *ndev)
{
	kfree(ndev);
}

static struct sbn_dev *sbn_find_device(struct notifier_block *nb)
{
	struct sbn_dev *ndev = NULL;

	if (!nb)
		return NULL;

	mutex_lock(&noti_lock);

	list_for_each_entry(ndev, &sb_notify.dev_list, list) {
		if (ndev->nb == nb)
			break;
	}

	mutex_unlock(&noti_lock);

	return ndev;
}

static void sbn_work(struct work_struct *work)
{
	struct sbn_data *noti_data = NULL;
	int ret = 0;

	if (sb_pq_pop(sb_notify.pq, (sb_data *)&noti_data))
		goto end_work;

	mutex_lock(&noti_lock);
	ret = blocking_notifier_call_chain(&(sb_notify.nb_head),
		noti_data->type, (void *)noti_data->data);
	mutex_unlock(&noti_lock);
	noti_log("end noti(ret = %d, type = %d)\n", ret, noti_data->type);

	kfree(noti_data);
	if (!sb_pq_empty(sb_notify.pq)) {
		queue_delayed_work(sb_notify.wq, &sb_notify.work, 0);
		return;
	}

end_work:
	__pm_relax(sb_notify.ws);
}

static bool noti_cmp(sb_data data1, sb_data data2)
{
	return false;
}

static int sb_notify_init(void)
{
	int ret = 0;

	mutex_lock(&noti_lock);

	if (sb_notify.init_state)
		goto skip_init;

	sb_notify.pq = sb_pq_create(0, SB_NOTIFY_PQ_SIZE, noti_cmp);
	if (IS_ERR(sb_notify.pq)) {
		ret = PTR_ERR(sb_notify.pq);
		noti_log("Failed to create pq(%d)\n", ret);
		goto err_pq;
	}

	sb_notify.wq = create_singlethread_workqueue(SB_NOTIFY_NAME);
	if (!sb_notify.wq) {
		ret = -EINVAL;
		noti_log("Failed to create wq\n");
		goto err_wq;
	}

	sb_notify.ws = wakeup_source_register(NULL, SB_NOTIFY_NAME);
	if (!sb_notify.ws) {
		ret = -EINVAL;
		noti_log("Failed to register ws\n");
		goto err_ws;
	}

	INIT_DELAYED_WORK(&sb_notify.work, sbn_work);

	BLOCKING_INIT_NOTIFIER_HEAD(&(sb_notify.nb_head));

	INIT_LIST_HEAD(&sb_notify.dev_list);

	sb_notify.init_state = true;

skip_init:
	mutex_unlock(&noti_lock);
	return 0;

err_ws:
	destroy_workqueue(sb_notify.wq);
err_wq:
	sb_pq_destroy(sb_notify.pq);
err_pq:
	mutex_unlock(&noti_lock);
	return ret;
}

int sb_notify_call(enum sbn_type type, sb_data data)
{
	int ret = 0;

	if (type <= SB_NOTIFY_UNKNOWN || type >= SB_NOTIFY_MAX)
		return -EINVAL;

	ret = blocking_notifier_call_chain(&(sb_notify.nb_head),
		type, (void *)data);

	return 0;
}
EXPORT_SYMBOL(sb_notify_call);

int sb_notify_call_work(enum sbn_type type, sb_data data)
{
	struct sbn_data *noti_data = NULL;
	int ret = 0;

	if (type <= SB_NOTIFY_UNKNOWN || type >= SB_NOTIFY_MAX)
		return -EINVAL;

	if (sb_notify_init() < 0)
		return -ENOENT;

	noti_data = kzalloc(sizeof(struct sbn_data), GFP_KERNEL);
	if (!noti_data)
		return -ENOMEM;

	noti_data->type = type;
	noti_data->data = data;
	ret = sb_pq_push(sb_notify.pq, 0, (sb_data)noti_data);
	if (ret < 0) {
		kfree(noti_data);
		return ret;
	}

	__pm_stay_awake(sb_notify.ws);
	queue_delayed_work(sb_notify.wq, &sb_notify.work, 0);

	return 0;
}
EXPORT_SYMBOL(sb_notify_call_work);

int sb_notify_register(struct notifier_block *nb, notifier_fn_t notifier,
		const char *name, enum sb_dev_type type)
{
	struct sbn_dev *ndev;
	int ret = 0;

	if (!nb || !notifier || !name)
		return -EINVAL;

	if (sb_notify_init() < 0)
		return -ENOENT;

	ndev = sbn_create_device(name, type);
	if (!ndev)
		return -ENOMEM;

	nb->notifier_call = notifier;
	nb->priority = 0;
	ndev->nb = nb;

	ret = sb_notify_call(SB_NOTIFY_DEV_PROBE, (sb_data)name);
	if (ret < 0)
		noti_log("failed to broadcast dev_probe, ret = %d\n", ret);

	mutex_lock(&noti_lock);

	ret = blocking_notifier_chain_register(&(sb_notify.nb_head), nb);
	if (ret < 0) {
		noti_log("failed to register nb(%s, %d)", name, ret);
		sbn_destroy_device(ndev);
		goto skip_register;
	}

	if (sb_notify.dev_count > 0) {
		struct sbn_dev_list dev_list;
		struct sbn_dev *tmp_dev;

		dev_list.list = kcalloc(sb_notify.dev_count, sizeof(char *), GFP_KERNEL);
		if (dev_list.list) {
			int i = 0;

			list_for_each_entry(tmp_dev, &sb_notify.dev_list, list) {
				dev_list.list[i++] = tmp_dev->name;
			}

			dev_list.count = sb_notify.dev_count;
			nb->notifier_call(nb, SB_NOTIFY_DEV_LIST, &dev_list);

			kfree(dev_list.list);
		}
	}

	list_add(&ndev->list, &sb_notify.dev_list);
	sb_notify.dev_count++;

skip_register:
	mutex_unlock(&noti_lock);
	return ret;
}
EXPORT_SYMBOL(sb_notify_register);

int sb_notify_unregister(struct notifier_block *nb)
{
	struct sbn_dev *ndev;
	int ret = 0;

	if (!sb_notify.init_state)
		return 0;

	ndev = sbn_find_device(nb);
	if (!ndev)
		return -ENODEV;

	mutex_lock(&noti_lock);

	list_del(&ndev->list);
	sb_notify.dev_count--;

	ret = blocking_notifier_chain_unregister(&sb_notify.nb_head, nb);
	if (ret < 0)
		noti_log("failed to unregister nb(%s, %d)",
			ndev->name, ret);

	nb->notifier_call = NULL;
	nb->priority = -1;

	mutex_unlock(&noti_lock);

	ret = sb_notify_call(SB_NOTIFY_DEV_SHUTDOWN, (sb_data)ndev->name);
	kfree(ndev);
	return ret;
}
EXPORT_SYMBOL(sb_notify_unregister);
