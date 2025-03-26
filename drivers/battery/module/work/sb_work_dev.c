/*
 *  sb_work_dev.c
 *  Samsung Mobile Battery Work Device
 *
 *  Copyright (C) 2022 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/of.h>
#include <linux/platform_device.h>

#include <linux/battery/common/sb_ext.h>
#include <linux/battery/module/sb_work.h>

#include "sb_work_core.h"

static struct sb_work_dev *work_dev;
struct sb_work_dev *get_work_dev(void)
{
	return work_dev;
}

#define DEFAULT_ACTIVE_CNT	4
#define DEFAULT_WORK_CNT	10
#define DEFAULT_WAIT_DELAY	(HZ)
static int _parse_dt(struct device_node *np, struct sb_work_offset *offset)
{
	sb_of_parse_u32(np, offset, max_active_cnt, DEFAULT_ACTIVE_CNT);
	sb_of_parse_u32(np, offset, max_work_cnt, DEFAULT_WORK_CNT);
	sb_of_parse_u32(np, offset, max_wait_delay, DEFAULT_WAIT_DELAY);
	return 0;
}

static int _init(struct sb_work_dev *wdev)
{
	struct sb_work *work;
	int i = 0;

	init_obj(&wdev->wq, "wq root", SB_WORK_TYPE_ROOT);
	init_obj(&wdev->work, "work root", SB_WORK_TYPE_ROOT);
	init_obj(&wdev->junk, "junk root", SB_WORK_TYPE_ROOT);

	wdev->g_wq = create_wq(wdev->dev, SB_WORK_NAME, wdev->offset.max_active_cnt);
	if (!wdev->g_wq)
		return -ENOMEM;

	for (; i < wdev->offset.max_work_cnt; i++) {
		work = create_work();
		if (!work)
			goto err_work;

		push_work(wdev, work);
	}

	return 0;

err_work:
	for (; i > 0; i--) {
		work = pop_work(wdev);
		if (!work)
			break;

		destroy_work(work);
	}
	destroy_wq(wdev->g_wq);
	return -ENOMEM;
}

static void test_print(void)
{
	work_log("mutex = %d, obj = %d, wq = %d, op = %d, work = %d\n",
		(int)sizeof(struct mutex),
		(int)sizeof(struct sb_work_obj),
		(int)sizeof(struct sb_work_wq),
		(int)sizeof(struct sb_work_op),
		(int)sizeof(struct sb_work));
}

static int _probe(struct platform_device *pdev)
{
	struct sb_work_dev *wdev;
	int ret = -ENOMEM;

	test_print();

	work_log("Start\n");
	wdev = kzalloc(sizeof(struct sb_work_dev), GFP_KERNEL);
	if (!wdev)
		goto err_alloc;

	wdev->dev = &pdev->dev;
	ret = _parse_dt(pdev->dev.of_node, &wdev->offset);
	if (ret)
		goto err_parse;

	ret = _init(wdev);
	if (ret)
		goto err_init;

	work_dev = wdev;
	work_log("End\n");

	return 0;

err_init:
err_parse:
	kfree(wdev);
err_alloc:
	work_log("error = %d\n", ret);
	return ret;
}

static int _remove(struct platform_device *pdev)
{
	//struct sb_work_dev *wdev = platform_get_drvdata(pdev);

	work_log("\n");
	return 0;
}

static void _shutdown(struct platform_device *pdev)
{
	//struct sb_work_dev *wdev = platform_get_drvdata(pdev);

	work_log("\n");
}

static int _prepare(struct device *dev)
{
	//struct sb_work_dev *wdev = dev_get_drvdata(pdev);

	work_log("\n");
	return 0;
}

static int _suspend(struct device *dev)
{
	//struct sb_work_dev *wdev = dev_get_drvdata(pdev);

	work_log("\n");
	return 0;
}

static int _resume(struct device *dev)
{
	//struct sb_work_dev *wdev = dev_get_drvdata(pdev);

	work_log("\n");
	return 0;
}

static void _complete(struct device *dev)
{
	//struct sb_work_dev *wdev = dev_get_drvdata(pdev);

	work_log("\n");
}

static const struct of_device_id _dt_ids[] = {
	{ .compatible = "samsung,sb-work" },
	{ }
};
MODULE_DEVICE_TABLE(of, _dt_ids);

static const struct dev_pm_ops _pm_ops = {
	.prepare = _prepare,
	.suspend = _suspend,
	.resume = _resume,
	.complete = _complete,
};

static struct platform_driver _driver = {
	.driver = {
		.name = SB_WORK_NAME,
		.owner = THIS_MODULE,
		.pm = &_pm_ops,
		.of_match_table = _dt_ids,
	},
	.probe = _probe,
	.remove = _remove,
	.shutdown = _shutdown,
};

static int __init sb_work_init(void)
{
	return platform_driver_register(&_driver);
}
module_init(sb_work_init);

static void __exit sb_work_exit(void)
{
	platform_driver_unregister(&_driver);
}
module_exit(sb_work_exit);

MODULE_DESCRIPTION("Samsung Battery Work");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
