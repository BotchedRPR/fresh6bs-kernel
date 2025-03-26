/*
 *  sb_wrl.c
 *  Samsung Mobile Battery Wrl
 *
 *  Copyright (C) 2023 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/battery/wireless/sb_wrl.h>
#include <linux/battery/sb_vote.h>
#include <linux/battery/common/sb_vote_event.h>

#define wrl_log(str, ...) pr_info("[SB-WRL]:%s: "str, __func__, ##__VA_ARGS__)
#define ERROR_CNT_MAX	255

struct sb_wrl_data {
	const struct sb_wrl_op *op;
	void *pdata;

	union  {
		sb_data raw_data;

		unsigned char data[SB_WRL_ERROR_TYPE_MAX];
	} error;
};

struct sb_wrl_data wrl_data;

int sb_wrl_set_vm_phm(int event, int en, sb_data data)
{
	return wrl_data.op->set_vm_phm(wrl_data.pdata, event, en, data);
}
EXPORT_SYMBOL(sb_wrl_set_vm_phm);

int sb_wrl_get_vm_phm(sb_data *data)
{
	return wrl_data.op->get_vm_phm(wrl_data.pdata, data);
}
EXPORT_SYMBOL(sb_wrl_get_vm_phm);

static bool check_op(const struct sb_wrl_op *op)
{
	if (op == NULL)
		return false;

	if (op->set_vm_phm == NULL)
		return false;

	return true;
}

int sb_wrl_register(const struct sb_wrl_op *op, void *pdata)
{
	if (!check_op(op))
		return -EINVAL;

	wrl_data.pdata = pdata;
	wrl_data.op = op;

	return 0;
}
EXPORT_SYMBOL(sb_wrl_register);

static bool check_err_type(enum sb_wrl_error err)
{
	return ((err >= SB_WRL_ERROR_TYPE_SSP_MISSING) &&
		(err < SB_WRL_ERROR_TYPE_MAX));
}

int sb_wrl_inc_error(enum sb_wrl_error err)
{
	if (!check_err_type(err))
		return -EINVAL;

	wrl_data.error.data[err] = (wrl_data.error.data[err] + 1) % ERROR_CNT_MAX;
	return wrl_data.error.data[err];
}
EXPORT_SYMBOL(sb_wrl_inc_error);

int sb_wrl_clr_error(enum sb_wrl_error err)
{
	if (!check_err_type(err))
		return -EINVAL;

	wrl_data.error.data[err] = 0;
	return 0;
}
EXPORT_SYMBOL(sb_wrl_clr_error);

sb_data sb_wrl_error_raw_data(void)
{
	return wrl_data.error.raw_data;
}
EXPORT_SYMBOL(sb_wrl_error_raw_data);

static int default_set_vm_phm(void *pdata, int event, int en, sb_data data)
{
	wrl_log("Not Support\n");
	return 0;
}

static int default_get_vm_phm(void *pdata, sb_data *data)
{
	wrl_log("Not Support\n");
	return 0;
}

static const struct sb_wrl_op default_op = {
	.set_vm_phm = default_set_vm_phm,
	.get_vm_phm = default_get_vm_phm,
};

static int __init sb_wrl_init(void)
{
	wrl_log("\n");
	return sb_wrl_register(&default_op, NULL);
}
module_init(sb_wrl_init);

static void __exit sb_wrl_exit(void)
{
}
module_exit(sb_wrl_exit);

MODULE_DESCRIPTION("Samsung Battery Wrl");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
