// SPDX-License-Identifier: GPL
/*
 * Samsung Exynos SoC series Pablo driver
 *
 * Exynos Pablo image subsystem functions
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include "pablo-smc.h"

static struct pablo_smc_ops psmc_ops;

struct pablo_smc_ops *pablo_get_smc_ops(void)
{
	return &psmc_ops;
}
EXPORT_SYMBOL_GPL(pablo_get_smc_ops);

static unsigned long pablo_smc(unsigned long cmd, unsigned long arg0,
		unsigned long arg1, unsigned long arg2)
{
	return exynos_smc(cmd, arg0, arg1, arg2);
}

static int __init pablo_smc_init(void)
{
	if (IS_ENABLED(CONFIG_ARCH_VELOCE_HYCON))
		psmc_ops.call = NULL;
	else
		psmc_ops.call = pablo_smc;

	return 0;
}
module_init(pablo_smc_init);

static void __exit pablo_smc_exit(void)
{
}
module_exit(pablo_smc_exit)

MODULE_DESCRIPTION("Exynos Pablo SMC controller");
MODULE_LICENSE("GPL");
