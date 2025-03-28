// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ALSA SoC - Samsung Abox Debug driver
 *
 * Copyright (c) 2016 Samsung Electronics Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/iommu.h>
#include <linux/of_reserved_mem.h>
#include <linux/pm_runtime.h>
#include <linux/sched/clock.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <asm/cacheflush.h>

#include "abox_util.h"
#include "abox_proc.h"
#include "abox_gic.h"
#include "abox_core.h"
#include "abox_oem.h"
#include "abox_dbg.h"
#include "abox_memlog.h"

#define ABOX_DBG_DUMP_MAGIC_SRAM	0x3935303030504D44ull /* DMP00059 */
#define ABOX_DBG_DUMP_MAGIC_DRAM	0x3231303038504D44ull /* DMP80012 */
#define ABOX_DBG_DUMP_MAGIC_PARA	0x3230303034504D44ull /* DMP40002 */
#define ABOX_DBG_DUMP_MAGIC_LOG		0x3142303038504D44ull /* DMP800B1 */
#define ABOX_DBG_DUMP_MAGIC_SFR		0x5246533030504D44ull /* DMP00SFR */
#define ABOX_DBG_DUMP_LIMIT_NS		(5 * NSEC_PER_SEC)

static struct platform_device *p_pdev;

void abox_dbg_print_gpr_from_addr(struct device *dev, struct abox_data *data,
		unsigned int *addr)
{
	abox_core_print_gpr_dump(addr);
}

void abox_dbg_print_gpr(struct device *dev, struct abox_data *data)
{
	abox_core_print_gpr();
}

struct abox_dbg_dump_sram {
	unsigned long long magic;
	char dump[SRAM_FIRMWARE_SIZE];
} __packed;

struct abox_dbg_dump_dram {
	unsigned long long magic;
	char dump[DRAM_FIRMWARE_SIZE];
} __packed;

struct abox_dbg_dump_dram_para {
	unsigned long long magic;
	char dump[DRAM_PARAMETER_SIZE];
} __packed;

struct abox_dbg_dump_log {
	unsigned long long magic;
	char dump[ABOX_LOG_SIZE];
} __packed;

struct abox_dbg_dump_sfr {
	unsigned long long magic;
	u32 dump[SZ_64K / sizeof(u32)];
} __packed;

struct abox_dbg_dump {
	struct abox_dbg_dump_sram sram;
	struct abox_dbg_dump_dram dram;
	struct abox_dbg_dump_dram_para dram_para;
	struct abox_dbg_dump_sfr sfr;
	u32 sfr_gic_gicd[SZ_4K / sizeof(u32)];
	unsigned int gpr[SZ_128];
	long long time;
	char reason[SZ_32];
	unsigned int previous_gpr;
	unsigned int previous_mem;
} __packed;

struct abox_dbg_dump_min {
	struct abox_dbg_dump_sram sram;
	struct abox_dbg_dump_log log;
	struct abox_dbg_dump_log log_01;
	struct abox_dbg_dump_sfr sfr;
	u32 sfr_gic_gicd[SZ_4K / sizeof(u32)];
	unsigned int gpr[SZ_128];
	long long time;
	char reason[SZ_32];
	unsigned int previous_gpr;
	unsigned int previous_mem;
} __packed;

struct abox_dbg_dump_info {
	struct abox_proc_bin sram;
	struct abox_proc_bin dram;
	struct abox_proc_bin dram_para;
	struct abox_proc_bin log;
	struct abox_proc_bin log_01;
	struct abox_proc_bin sfr;
	struct abox_proc_bin gicd;
	struct abox_proc_bin gpr;
	struct abox_proc_bin reason;
};

static struct abox_dbg_dump (*p_abox_dbg_dump)[ABOX_DBG_DUMP_COUNT];
static struct abox_dbg_dump_min (*p_abox_dbg_dump_min)[ABOX_DBG_DUMP_COUNT];
static struct abox_dbg_dump_info abox_dbg_dump_info[ABOX_DBG_DUMP_COUNT];

/* revisited free_reserved_area() of /mm/page_alloc.c */
static unsigned long __free_reserved_area(phys_addr_t start, phys_addr_t end, const char *s)
{
	phys_addr_t pos;
	unsigned long pages = 0;

	start = PAGE_ALIGN(start);
	end &= PAGE_MASK;
	for (pos = start; pos < end; pos += PAGE_SIZE, pages++)
		free_reserved_page(phys_to_page(pos));

	if (pages && s)
		pr_info("Freeing %s memory: %ldK\n", s, pages << (PAGE_SHIFT - 10));

	return pages;
}

static void abox_dbg_resize_rmem(struct device *dev, struct reserved_mem *rmem,
		size_t new_size, const char *tag)
{
	size_t old_size = rmem->size;

	if (old_size < new_size) {
		abox_warn(dev, "%s: new size %#zx is bigger than reserved size %#zx\n",
				tag, new_size, old_size);
		return;
	}

	rmem->size = new_size;
	__free_reserved_area(rmem->base + new_size, rmem->base + old_size, tag);
	abox_info(dev, "%s: %s new size %#lx\n", __func__, tag, rmem->size);
}

static struct reserved_mem *abox_dbg_slog;

static int __init abox_dbg_slog_setup(struct reserved_mem *rmem)
{
	pr_info("%s: size=%pa\n", __func__, &rmem->size);
	abox_dbg_slog = rmem;
	return 0;
}

RESERVEDMEM_OF_DECLARE(abox_dbg_slog, "exynos,abox_slog", abox_dbg_slog_setup);

static void abox_dbg_slog_init(struct abox_data *data)
{
	struct device *dev_abox = data->dev;

	abox_info(dev_abox, "%s: size=%pa\n", __func__, &abox_dbg_slog->size);

	data->slog_phys = abox_dbg_slog->base;
	data->slog_size = abox_dbg_slog->size;
	data->slog_base = rmem_vmap(abox_dbg_slog);
	abox_iommu_map(dev_abox, IOVA_SILENT_LOG, data->slog_phys,
			data->slog_size, data->slog_base);
}

static struct reserved_mem *abox_dbg_rmem;

static int __init abox_dbg_rmem_setup(struct reserved_mem *rmem)
{
	pr_info("%s: size=%pa\n", __func__, &rmem->size);
	abox_dbg_rmem = rmem;
	return 0;
}

RESERVEDMEM_OF_DECLARE(abox_dbg_rmem, "exynos,abox_dbg", abox_dbg_rmem_setup);

static bool abox_dbg_dump_valid(int idx)
{
	bool ret = false;

	if (idx >= ABOX_DBG_DUMP_COUNT)
		return false;

	if (p_abox_dbg_dump) {
		struct abox_dbg_dump *p_dump = &(*p_abox_dbg_dump)[idx];

		ret = (p_dump->sfr.magic == ABOX_DBG_DUMP_MAGIC_SFR);
	} else if (p_abox_dbg_dump_min) {
		struct abox_dbg_dump_min *p_dump = &(*p_abox_dbg_dump_min)[idx];

		ret = (p_dump->sfr.magic == ABOX_DBG_DUMP_MAGIC_SFR);
	}

	return ret;
}

static void abox_dbg_clear_valid(int idx)
{
	if (idx >= ABOX_DBG_DUMP_COUNT)
		return;

	if (p_abox_dbg_dump) {
		struct abox_dbg_dump *p_dump = &(*p_abox_dbg_dump)[idx];

		p_dump->sfr.magic = 0;
	} else if (p_abox_dbg_dump_min) {
		struct abox_dbg_dump_min *p_dump = &(*p_abox_dbg_dump_min)[idx];

		p_dump->sfr.magic = 0;
	}
}

static ssize_t abox_dbg_read_valid(struct file *file, char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	int idx = (int)(long)abox_proc_data(file);
	bool valid = abox_dbg_dump_valid(idx);
	char buf_val[4]; /* enough to store a bool and "\n\0" */

	if (valid)
		buf_val[0] = 'Y';
	else
		buf_val[0] = 'N';
	buf_val[1] = '\n';
	buf_val[2] = 0x00;
	return simple_read_from_buffer(user_buf, count, ppos, buf_val, 2);
}

static const struct proc_ops abox_dbg_fops_valid = {
	.proc_open = simple_open,
	.proc_read = abox_dbg_read_valid,
	.proc_lseek = default_llseek,
};

static ssize_t abox_dbg_read_clear(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	int idx = (int)(long)abox_proc_data(file);

	abox_dbg_clear_valid(idx);

	return 0;
}

static ssize_t abox_dbg_write_clear(struct file *file,
				    const char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	int idx = (int)(long)abox_proc_data(file);

	abox_dbg_clear_valid(idx);

	return 0;
}

static const struct proc_ops abox_dbg_fops_clear = {
	.proc_open = simple_open,
	.proc_read = abox_dbg_read_clear,
	.proc_write = abox_dbg_write_clear,
	.proc_lseek = no_llseek,
};

static void dump_gpr_from_addr_full(struct device *dev, unsigned int *addr,
		enum abox_dbg_dump_src src, const char *reason,
		unsigned long long time)
{
	struct abox_dbg_dump *p_dump = &(*p_abox_dbg_dump)[src];

	p_dump->time = time;
	p_dump->previous_gpr = 0;
	strncpy(p_dump->reason, reason, sizeof(p_dump->reason) - 1);
	abox_core_dump_gpr_dump(p_dump->gpr, addr);
}

static void dump_gpr_from_addr_half(struct device *dev, unsigned int *addr,
		enum abox_dbg_dump_src src, const char *reason,
		unsigned long long time)
{
	struct abox_dbg_dump *p_dump = &(*p_abox_dbg_dump)[0];

	if (src == ABOX_DBG_DUMP_KERNEL) {
		if (abox_dbg_dump_valid(0) && p_dump->previous_gpr == 0) {
			abox_info(dev, "%s(%d): skipped\n", __func__, src);
			return;
		}
	}

	p_dump->time = time;
	p_dump->previous_gpr = 0;
	strncpy(p_dump->reason, reason, sizeof(p_dump->reason) - 1);
	abox_core_dump_gpr_dump(p_dump->gpr, addr);
}

static void dump_gpr_from_addr_min(struct device *dev, unsigned int *addr,
		enum abox_dbg_dump_src src, const char *reason,
		unsigned long long time)
{
	struct abox_dbg_dump_min *p_dump = &(*p_abox_dbg_dump_min)[src];

	p_dump->time = time;
	p_dump->previous_gpr = 0;
	strncpy(p_dump->reason, reason, sizeof(p_dump->reason) - 1);
	abox_core_dump_gpr_dump(p_dump->gpr, addr);
}

static void dump_gpr_full(struct device *dev, struct abox_data *data,
		enum abox_dbg_dump_src src, const char *reason,
		unsigned long long time)
{
	struct abox_dbg_dump *p_dump = &(*p_abox_dbg_dump)[src];

	p_dump->time = time;
	p_dump->previous_gpr = 0;
	strncpy(p_dump->reason, reason, sizeof(p_dump->reason) - 1);
	abox_core_dump_gpr(p_dump->gpr);
}

static void dump_gpr_half(struct device *dev, struct abox_data *data,
		enum abox_dbg_dump_src src, const char *reason,
		unsigned long long time)
{
	struct abox_dbg_dump *p_dump = &(*p_abox_dbg_dump)[0];

	if (src == ABOX_DBG_DUMP_KERNEL) {
		if (abox_dbg_dump_valid(0) && p_dump->previous_gpr == 0) {
			abox_info(dev, "%s(%d): skipped\n", __func__, src);
			return;
		}
	}

	p_dump->time = time;
	p_dump->previous_gpr = 0;
	strncpy(p_dump->reason, reason, sizeof(p_dump->reason) - 1);
	abox_core_dump_gpr(p_dump->gpr);
}

static void dump_gpr_min(struct device *dev, struct abox_data *data,
		enum abox_dbg_dump_src src, const char *reason,
		unsigned long long time)
{
	struct abox_dbg_dump_min *p_dump = &(*p_abox_dbg_dump_min)[src];

	p_dump->time = time;
	p_dump->previous_gpr = 0;
	strncpy(p_dump->reason, reason, sizeof(p_dump->reason) - 1);
	abox_core_dump_gpr(p_dump->gpr);
}

static void dump_mem_full(struct device *dev, struct abox_data *data,
		enum abox_dbg_dump_src src, const char *reason,
		unsigned long long time)
{
	struct abox_dbg_dump *p_dump = &(*p_abox_dbg_dump)[src];

	p_dump->time = time;
	p_dump->previous_mem = 0;
	strncpy(p_dump->reason, reason, sizeof(p_dump->reason) - 1);
	memcpy_fromio(p_dump->sram.dump, data->sram_base,
			SRAM_FIRMWARE_SIZE);
	p_dump->sram.magic = ABOX_DBG_DUMP_MAGIC_SRAM;
	memcpy(p_dump->dram.dump, data->dram_base, DRAM_FIRMWARE_SIZE);
	p_dump->dram.magic = ABOX_DBG_DUMP_MAGIC_DRAM;
	memcpy(p_dump->dram_para.dump, data->dram_para_base, DRAM_PARAMETER_SIZE);
	p_dump->dram_para.magic = ABOX_DBG_DUMP_MAGIC_PARA;
	memcpy_fromio(p_dump->sfr.dump, data->sfr_base,
			sizeof(p_dump->sfr.dump));
	p_dump->sfr.magic = ABOX_DBG_DUMP_MAGIC_SFR;
	abox_gicd_dump(data->dev_gic, (char *)p_dump->sfr_gic_gicd, 0,
			sizeof(p_dump->sfr_gic_gicd));
}

static void dump_mem_half(struct device *dev, struct abox_data *data,
		enum abox_dbg_dump_src src, const char *reason,
		unsigned long long time)
{
	struct abox_dbg_dump *p_dump = &(*p_abox_dbg_dump)[0];

	if (src == ABOX_DBG_DUMP_KERNEL) {
		if (abox_dbg_dump_valid(0) && p_dump->previous_mem == 0) {
			abox_info(dev, "%s(%d): skipped\n", __func__, src);
			return;
		}
	}

	p_dump->time = time;
	p_dump->previous_mem = 0;
	strncpy(p_dump->reason, reason, sizeof(p_dump->reason) - 1);
	memcpy_fromio(p_dump->sram.dump, data->sram_base,
			SRAM_FIRMWARE_SIZE);
	p_dump->sram.magic = ABOX_DBG_DUMP_MAGIC_SRAM;
	memcpy(p_dump->dram.dump, data->dram_base, DRAM_FIRMWARE_SIZE);
	p_dump->dram.magic = ABOX_DBG_DUMP_MAGIC_DRAM;
	memcpy(p_dump->dram_para.dump, data->dram_para_base, DRAM_PARAMETER_SIZE);
	p_dump->dram_para.magic = ABOX_DBG_DUMP_MAGIC_PARA;
	memcpy_fromio(p_dump->sfr.dump, data->sfr_base,
			sizeof(p_dump->sfr.dump));
	p_dump->sfr.magic = ABOX_DBG_DUMP_MAGIC_SFR;
	abox_gicd_dump(data->dev_gic, (char *)p_dump->sfr_gic_gicd, 0,
			sizeof(p_dump->sfr_gic_gicd));
}

static void dump_mem_min(struct device *dev, struct abox_data *data,
		enum abox_dbg_dump_src src, const char *reason,
		unsigned long long time)
{
	struct abox_dbg_dump_min *p_dump = &(*p_abox_dbg_dump_min)[src];

	p_dump->time = time;
	p_dump->previous_mem = 0;
	strncpy(p_dump->reason, reason, sizeof(p_dump->reason) - 1);
	memcpy_fromio(p_dump->sram.dump, data->sram_base,
			SRAM_FIRMWARE_SIZE);
	p_dump->sram.magic = ABOX_DBG_DUMP_MAGIC_SRAM;

	memcpy(p_dump->log.dump, data->dram_base + data->log_addr,
			ABOX_LOG_SIZE);
	p_dump->log.magic = ABOX_DBG_DUMP_MAGIC_LOG;

	memcpy(p_dump->log_01.dump, data->dram_base + data->log_major_addr,
			ABOX_LOG_MAJOR_SIZE);
	p_dump->log_01.magic = ABOX_DBG_DUMP_MAGIC_LOG;

	memcpy_fromio(p_dump->sfr.dump, data->sfr_base,
			sizeof(p_dump->sfr.dump));
	p_dump->sfr.magic = ABOX_DBG_DUMP_MAGIC_SFR;
	abox_gicd_dump(data->dev_gic, (char *)p_dump->sfr_gic_gicd, 0,
			sizeof(p_dump->sfr_gic_gicd));
}

static int abox_dbg_dump_count;
static void (*p_dump_gpr_from_addr)(struct device *dev, unsigned int *addr,
		enum abox_dbg_dump_src src, const char *reason,
		unsigned long long time);
static void (*p_dump_gpr)(struct device *dev, struct abox_data *data,
		enum abox_dbg_dump_src src, const char *reason,
		unsigned long long time);
static void (*p_dump_mem)(struct device *dev, struct abox_data *data,
		enum abox_dbg_dump_src src, const char *reason,
		unsigned long long time);

static int abox_dbg_dump_create_file(struct abox_data *data)
{
	const char *dir_fmt = "snapshot_%d";
	struct proc_dir_entry *dir;
	struct abox_dbg_dump_info *info;
	char *dir_name;
	int i;

	abox_dbg(data->dev, "%s\n", __func__);

	if (p_abox_dbg_dump) {
		struct abox_dbg_dump *p_dump;

		for (i = 0; i < abox_dbg_dump_count; i++) {
			dir_name = kasprintf(GFP_KERNEL, dir_fmt, i);
			dir = abox_proc_mkdir(dir_name, NULL);
			p_dump = &(*p_abox_dbg_dump)[i];
			info = &abox_dbg_dump_info[i];

			info->sram.data = p_dump->sram.dump;
			info->sram.size = sizeof(p_dump->sram.dump);
			abox_proc_create_bin("sram", 0440, dir, &info->sram);

			info->dram.data = p_dump->dram.dump;
			info->dram.size = sizeof(p_dump->dram.dump);
			abox_proc_create_bin("dram", 0440, dir, &info->dram);

			info->dram_para.data = p_dump->dram_para.dump;
			info->dram_para.size = sizeof(p_dump->dram_para.dump);
			abox_proc_create_bin("dram_para", 0440, dir, &info->dram_para);

			info->log.data = p_dump->dram.dump + data->log_addr;
			info->log.size = ABOX_LOG_SIZE;
			abox_proc_create_bin("log", 0444, dir, &info->log);

			info->sfr.data = p_dump->sfr.dump;
			info->sfr.size = sizeof(p_dump->sfr.dump);
			abox_proc_create_bin("sfr", 0440, dir, &info->sfr);

			info->gicd.data = p_dump->sfr_gic_gicd;
			info->gicd.size = sizeof(p_dump->sfr_gic_gicd);
			abox_proc_create_bin("gicd", 0440, dir, &info->gicd);

			info->gpr.data = p_dump->gpr;
			info->gpr.size = sizeof(p_dump->gpr);
			abox_proc_create_bin("gpr", 0440, dir, &info->gpr);

			abox_proc_create_u64("time", 0440, dir, &p_dump->time);

			info->reason.data = p_dump->reason;
			info->reason.size = sizeof(p_dump->reason);
			abox_proc_create_bin("reason", 0440, dir,
					&info->reason);

			abox_proc_create_u32("previous", 0440, dir,
					&p_dump->previous_mem);

			abox_proc_create_file("valid", 0440, dir,
					&abox_dbg_fops_valid,
					(void *)(long)i, 0);

			abox_proc_create_file("clear", 0440, dir,
					&abox_dbg_fops_clear,
					(void *)(long)i, 0);

			kfree(dir_name);
		}
	} else if (p_abox_dbg_dump_min) {
		struct abox_dbg_dump_min *p_dump;

		for (i = 0; i < abox_dbg_dump_count; i++) {
			dir_name = kasprintf(GFP_KERNEL, dir_fmt, i);
			dir = abox_proc_mkdir(dir_name, NULL);
			p_dump = &(*p_abox_dbg_dump_min)[i];
			info = &abox_dbg_dump_info[i];

			info->sram.data = p_dump->sram.dump;
			info->sram.size = sizeof(p_dump->sram.dump);
			abox_proc_create_bin("sram", 0440, dir, &info->sram);

			info->log.data = p_dump->log.dump;
			info->log.size = ABOX_LOG_SIZE;
			abox_proc_create_bin("log", 0444, dir, &info->log);

			info->log_01.data = p_dump->log_01.dump;
			info->log_01.size = ABOX_LOG_SIZE;
			abox_proc_create_bin("log-01", 0444, dir, &info->log_01);

			info->sfr.data = p_dump->sfr.dump;
			info->sfr.size = sizeof(p_dump->sfr.dump);
			abox_proc_create_bin("sfr", 0440, dir, &info->sfr);

			info->gicd.data = p_dump->sfr_gic_gicd;
			info->gicd.size = sizeof(p_dump->sfr_gic_gicd);
			abox_proc_create_bin("gicd", 0440, dir, &info->gicd);

			info->gpr.data = p_dump->gpr;
			info->gpr.size = sizeof(p_dump->gpr);
			abox_proc_create_bin("gpr", 0440, dir, &info->gpr);

			abox_proc_create_u64("time", 0440, dir, &p_dump->time);

			info->reason.data = p_dump->reason;
			info->reason.size = sizeof(p_dump->reason);
			abox_proc_create_bin("reason", 0440, dir,
					&info->reason);

			abox_proc_create_u32("previous", 0440, dir,
					&p_dump->previous_mem);

			abox_proc_create_file("valid", 0440, dir,
					&abox_dbg_fops_valid,
					(void *)(long)i, 0);

			abox_proc_create_file("clear", 0440, dir,
					&abox_dbg_fops_clear,
					(void *)(long)i, 0);

			kfree(dir_name);
		}
	} else {
		return -ENOMEM;
	}

	return 0;
}

static void abox_dbg_rmem_init(struct abox_data *data)
{
	struct device *dev_abox = data->dev;
	int i;

	abox_info(dev_abox, "%s: size=%pa\n", __func__, &abox_dbg_rmem->size);

	if (sizeof(*p_abox_dbg_dump) <= abox_dbg_rmem->size) {
		struct abox_dbg_dump *p_dump;

		p_abox_dbg_dump = rmem_vmap(abox_dbg_rmem);
		data->dump_base = p_abox_dbg_dump;
		for (i = 0; i < ABOX_DBG_DUMP_COUNT; i++) {
			p_dump = &(*p_abox_dbg_dump)[i];
			if (p_dump->sfr.magic == ABOX_DBG_DUMP_MAGIC_SFR) {
				p_dump->previous_gpr++;
				p_dump->previous_mem++;
			} else {
				p_dump->previous_gpr = 0;
				p_dump->previous_mem = 0;
			}
		}

		abox_dbg_dump_count = ABOX_DBG_DUMP_COUNT;
		p_dump_gpr_from_addr = dump_gpr_from_addr_full;
		p_dump_gpr = dump_gpr_full;
		p_dump_mem = dump_mem_full;
		abox_info(dev_abox, "%s debug dump\n", "full");
	} else if (sizeof((*p_abox_dbg_dump)[0]) <= abox_dbg_rmem->size) {
		struct abox_dbg_dump *p_dump;

		p_abox_dbg_dump = rmem_vmap(abox_dbg_rmem);
		data->dump_base = p_abox_dbg_dump;
		p_dump = &(*p_abox_dbg_dump)[0];
		if (p_dump->sfr.magic == ABOX_DBG_DUMP_MAGIC_SFR) {
			p_dump->previous_gpr++;
			p_dump->previous_mem++;
		} else {
			p_dump->previous_gpr = 0;
			p_dump->previous_mem = 0;
		}

		abox_dbg_dump_count = 1;
		p_dump_gpr_from_addr = dump_gpr_from_addr_half;
		p_dump_gpr = dump_gpr_half;
		p_dump_mem = dump_mem_half;
		abox_info(dev_abox, "%s debug dump\n", "half");
	} else if (sizeof(*p_abox_dbg_dump_min) <= abox_dbg_rmem->size) {
		struct abox_dbg_dump_min *p_dump;

		p_abox_dbg_dump_min = rmem_vmap(abox_dbg_rmem);
		data->dump_base = p_abox_dbg_dump_min;
		for (i = 0; i < ABOX_DBG_DUMP_COUNT; i++) {
			p_dump = &(*p_abox_dbg_dump_min)[i];
			if (p_dump->sfr.magic == ABOX_DBG_DUMP_MAGIC_SFR) {
				p_dump->previous_gpr++;
				p_dump->previous_mem++;
			} else {
				p_dump->previous_gpr = 0;
				p_dump->previous_mem = 0;
			}
		}

		abox_dbg_dump_count = ABOX_DBG_DUMP_COUNT;
		p_dump_gpr_from_addr = dump_gpr_from_addr_min;
		p_dump_gpr = dump_gpr_min;
		p_dump_mem = dump_mem_min;
		abox_info(dev_abox, "%s debug dump\n", "min");
	}

	data->dump_phys = abox_dbg_rmem->base;
	abox_iommu_map(dev_abox, IOVA_DUMP_BUFFER, abox_dbg_rmem->base,
			abox_dbg_rmem->size, data->dump_base);
}

void abox_dbg_dump_gpr_from_addr(struct device *dev, struct abox_data *data,
		unsigned int *addr, enum abox_dbg_dump_src src, const char *reason)
{
	static unsigned long long called[ABOX_DBG_DUMP_COUNT];
	unsigned long long time = sched_clock();

	abox_dbg(dev, "%s\n", __func__);

	if (pm_runtime_suspended(data->dev)) {
		abox_info(dev, "%s is skipped due to no power\n", __func__);
		return;
	}

	if (called[src] && time - called[src] < ABOX_DBG_DUMP_LIMIT_NS) {
		dev_dbg_ratelimited(dev, "%s(%d): skipped\n", __func__, src);
		called[src] = time;
		return;
	}
	called[src] = time;

	if (p_dump_gpr_from_addr)
		p_dump_gpr_from_addr(dev, addr, src, reason, time);
	else
		abox_err(dev, "abox dbg isn't initialized\n");
}

void abox_dbg_dump_gpr(struct device *dev, struct abox_data *data,
		enum abox_dbg_dump_src src, const char *reason)
{
	static unsigned long long called[ABOX_DBG_DUMP_COUNT];
	unsigned long long time = sched_clock();

	abox_dbg(dev, "%s\n", __func__);

	if (pm_runtime_suspended(data->dev)) {
		abox_info(dev, "%s is skipped due to no power\n", __func__);
		return;
	}

	if (called[src] && time - called[src] < ABOX_DBG_DUMP_LIMIT_NS) {
		dev_dbg_ratelimited(dev, "%s(%d): skipped\n", __func__, src);
		called[src] = time;
		return;
	}
	called[src] = time;

	if (p_dump_gpr)
		p_dump_gpr(dev, data, src, reason, time);
	else
		abox_err(dev, "abox dbg isn't initialized\n");
}

struct dump_mem_work_struct {
	struct work_struct work;
	struct device *dev;
	struct abox_data *data;
	enum abox_dbg_dump_src src;
	const char *reason;
	unsigned long long time;
};

static void dump_mem_work_func(struct work_struct *work)
{
	struct dump_mem_work_struct *dmw;

	dmw = container_of(work, typeof(*dmw), work);
	p_dump_mem(dmw->dev, dmw->data, dmw->src, dmw->reason, dmw->time);
}

static void dump_mem(struct device *dev, struct abox_data *data,
		enum abox_dbg_dump_src src, const char *reason, bool atomic)
{
	static unsigned long long called[ABOX_DBG_DUMP_COUNT];
	unsigned long long time = sched_clock();

	abox_dbg(dev, "%s\n", __func__);

	if (!abox_is_on()) {
		abox_info(dev, "%s is skipped due to no power\n", __func__);
		return;
	}

	if (called[src] && time - called[src] < ABOX_DBG_DUMP_LIMIT_NS) {
		dev_dbg_ratelimited(dev, "%s(%d): skipped\n", __func__, src);
		called[src] = time;
		return;
	}
	called[src] = time;

	if (p_dump_mem) {
		if (atomic) {
			p_dump_mem(dev, data, src, reason, time);
		} else {
			struct dump_mem_work_struct dmw;

			INIT_WORK_ONSTACK(&dmw.work, dump_mem_work_func);
			dmw.dev = dev;
			dmw.data = data;
			dmw.src = src;
			dmw.reason = reason;
			dmw.time = time;
			queue_work(system_unbound_wq, &dmw.work);
			flush_work(&dmw.work);
			destroy_work_on_stack(&dmw.work);
		}
	} else {
		abox_err(dev, "abox dbg isn't initialized\n");
	}
}

void abox_dbg_dump_mem(struct device *dev, struct abox_data *data,
		enum abox_dbg_dump_src src, const char *reason)
{
	dump_mem(dev, data, src, reason, true);
}

void abox_dbg_dump_gpr_mem(struct device *dev, struct abox_data *data,
		enum abox_dbg_dump_src src, const char *reason)
{
	abox_dbg_dump_gpr(dev, data, src, reason);
	dump_mem(dev, data, src, reason, false);
}

struct abox_dbg_dump_simple {
	struct abox_dbg_dump_sram sram;
	struct abox_dbg_dump_log log;
	struct abox_dbg_dump_sfr sfr;
	u32 sfr_gic_gicd[SZ_4K / sizeof(u32)];
	unsigned int gpr[SZ_128];
	long long time;
	char reason[SZ_32];
} __packed;

static struct abox_dbg_dump_simple abox_dump_simple;

void abox_dbg_dump_simple(struct device *dev, struct abox_data *data,
		const char *reason)
{
	static unsigned long long called;
	unsigned long long time = sched_clock();

	abox_info(dev, "%s\n", __func__);

	if (pm_runtime_suspended(data->dev)) {
		abox_info(dev, "%s is skipped due to no power\n", __func__);
		return;
	}

	if (called && time - called < ABOX_DBG_DUMP_LIMIT_NS) {
		dev_dbg_ratelimited(dev, "%s: skipped\n", __func__);
		called = time;
		return;
	}
	called = time;

	abox_dump_simple.time = time;
	strncpy(abox_dump_simple.reason, reason,
			sizeof(abox_dump_simple.reason) - 1);
	abox_core_dump_gpr(abox_dump_simple.gpr);
	memcpy_fromio(abox_dump_simple.sram.dump, data->sram_base,
			SRAM_FIRMWARE_SIZE);
	abox_dump_simple.sram.magic = ABOX_DBG_DUMP_MAGIC_SRAM;
	memcpy(abox_dump_simple.log.dump, data->dram_base + data->log_addr,
			ABOX_LOG_SIZE);
	abox_dump_simple.log.magic = ABOX_DBG_DUMP_MAGIC_LOG;
	memcpy_fromio(abox_dump_simple.sfr.dump, data->sfr_base,
			sizeof(abox_dump_simple.sfr.dump));
	abox_dump_simple.sfr.magic = ABOX_DBG_DUMP_MAGIC_SFR;
	abox_gicd_dump(data->dev_gic, (char *)abox_dump_simple.sfr_gic_gicd, 0,
			sizeof(abox_dump_simple.sfr_gic_gicd));
}

static struct abox_dbg_dump_simple abox_dump_suspend;

void abox_dbg_dump_suspend(struct device *dev, struct abox_data *data)
{
	abox_dbg(dev, "%s\n", __func__);

	abox_dump_suspend.time = sched_clock();
	strncpy(abox_dump_suspend.reason, "suspend",
			sizeof(abox_dump_suspend.reason) - 1);
	abox_core_dump_gpr(abox_dump_suspend.gpr);
	memcpy_fromio(abox_dump_suspend.sram.dump, data->sram_base,
			SRAM_FIRMWARE_SIZE);
	abox_dump_suspend.sram.magic = ABOX_DBG_DUMP_MAGIC_SRAM;
	memcpy(abox_dump_suspend.log.dump, data->dram_base + data->log_addr,
			ABOX_LOG_SIZE);
	abox_dump_suspend.log.magic = ABOX_DBG_DUMP_MAGIC_LOG;
	memcpy_fromio(abox_dump_suspend.sfr.dump, data->sfr_base,
			sizeof(abox_dump_suspend.sfr.dump));
	abox_dump_suspend.sfr.magic = ABOX_DBG_DUMP_MAGIC_SFR;
	abox_gicd_dump(data->dev_gic, (char *)abox_dump_suspend.sfr_gic_gicd, 0,
			sizeof(abox_dump_suspend.sfr_gic_gicd));
}

static ssize_t calliope_sram_read(struct file *file, struct kobject *kobj,
		struct bin_attribute *battr, char *buf,
		loff_t off, size_t size)
{
	struct device *dev = kobj_to_dev(kobj);
	struct device *dev_abox = dev->parent;

	abox_dbg(dev, "%s(%lld, %zu)\n", __func__, off, size);

	if (pm_runtime_get_if_in_use(dev_abox) > 0) {
		if (off == 0)
			abox_core_flush();
		memcpy_fromio(buf, battr->private + off, size);
		pm_runtime_put(dev_abox);
	} else {
		memset(buf, 0x0, size);
	}

	return size;
}

static ssize_t calliope_dram_read(struct file *file, struct kobject *kobj,
		struct bin_attribute *battr, char *buf,
		loff_t off, size_t size)
{
	struct device *dev = kobj_to_dev(kobj);
	struct device *dev_abox = dev->parent;

	abox_dbg(dev, "%s(%lld, %zu)\n", __func__, off, size);

	/* if the area isn't existed, private(=base address) is 0 */
	if (!battr->private)
		return 0;

	if (pm_runtime_get_if_in_use(dev_abox) > 0) {
		if (off == 0)
			abox_core_flush();
		pm_runtime_put(dev_abox);
	}
	memcpy(buf, battr->private + off, size);
	return size;
}

static ssize_t calliope_log_read(struct file *file, struct kobject *kobj,
		struct bin_attribute *battr, char *buf,
		loff_t off, size_t size)
{
	return calliope_dram_read(file, kobj, battr, buf, off, size);
}

static ssize_t calliope_slog_read(struct file *file, struct kobject *kobj,
		struct bin_attribute *battr, char *buf,
		loff_t off, size_t size)
{
	return calliope_dram_read(file, kobj, battr, buf, off, size);
}

static ssize_t gicd_read(struct file *file, struct kobject *kobj,
		struct bin_attribute *battr, char *buf,
		loff_t off, size_t size)
{
	struct device *dev = kobj_to_dev(kobj);
	struct device *dev_abox = dev->parent;
	struct abox_data *data = dev_get_drvdata(dev_abox);

	abox_dbg(dev, "%s(%lld, %zu)\n", __func__, off, size);

	pm_runtime_get(dev_abox);
	abox_gicd_dump(data->dev_gic, buf, off, size);
	pm_runtime_put(dev_abox);

	return size;
}

/* size will be updated later */
static BIN_ATTR(calliope_sram, 0440, calliope_sram_read, NULL, 0);
static BIN_ATTR(calliope_dram, 0440, calliope_dram_read, NULL,
		DRAM_FIRMWARE_SIZE);
static BIN_ATTR_RO(calliope_log, ABOX_LOG_SIZE);
static BIN_ATTR(calliope_slog, 0440, calliope_slog_read, NULL, 0);
static BIN_ATTR(gicd, 0440, gicd_read, NULL, SZ_4K);
static struct bin_attribute *calliope_bin_attrs[] = {
	&bin_attr_calliope_sram,
	&bin_attr_calliope_dram,
	&bin_attr_calliope_log,
	&bin_attr_calliope_slog,
	&bin_attr_gicd,
};

static ssize_t gpr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct device *dev_abox = dev->parent;

	if (pm_runtime_suspended(dev_abox)) {
		abox_info(dev, "%s is skipped due to no power\n", __func__);
		return -EFAULT;
	}

	return abox_core_show_gpr(buf);
}

static DEVICE_ATTR(gpr, 0440, gpr_show, NULL);

struct abox_dbg_file_info {
	struct proc_dir_entry *file;
	struct abox_data *abox_data;
	const char *name;
	const struct proc_ops *fops;
	void __iomem *io_base;
	void *base;
	void *buf;
	size_t size;
};

static ssize_t abox_dbg_file_read(struct file *file, char __user *buf,
		size_t count, loff_t *pos)
{
	struct abox_dbg_file_info *info = abox_proc_data(file);

	return simple_read_from_buffer(buf, count, pos, info->buf, info->size);
}

static int abox_dbg_file_open_sram(struct inode *inode, struct file *file)
{
	struct abox_dbg_file_info *info = abox_proc_data(file);
	struct device *dev_abox = info->abox_data->dev;

	info->buf = vmalloc(info->size);
	if (!info->buf)
		return -ENOMEM;

	if (pm_runtime_get_if_in_use(dev_abox) > 0) {
		abox_core_flush();
		memcpy_fromio(info->buf, info->io_base, info->size);
		pm_runtime_put(dev_abox);
	} else {
		memset(info->buf, 0x0, info->size);
	}

	return simple_open(inode, file);
}

static int abox_dbg_file_open_dram(struct inode *inode, struct file *file)
{
	struct abox_dbg_file_info *info = abox_proc_data(file);
	struct device *dev_abox = info->abox_data->dev;

	info->buf = vmalloc(info->size);
	if (!info->buf)
		return -ENOMEM;

	if (pm_runtime_get_if_in_use(dev_abox) > 0) {
		abox_core_flush();
		memcpy(info->buf, info->base, info->size);
		pm_runtime_put(dev_abox);
	} else {
		memcpy(info->buf, info->base, info->size);
	}

	return simple_open(inode, file);
}

static int abox_dbg_file_open_sfr(struct inode *inode, struct file *file)
{
	struct abox_dbg_file_info *info = abox_proc_data(file);
	struct device *dev_abox = info->abox_data->dev;

	info->buf = vmalloc(info->size);
	if (!info->buf)
		return -ENOMEM;

	if (pm_runtime_get_if_in_use(dev_abox) > 0) {
		memcpy_fromio(info->buf, info->io_base, info->size);
		pm_runtime_put(dev_abox);
	} else {
		memset(info->buf, 0x0, info->size);
	}

	return simple_open(inode, file);
}

static int abox_dbg_file_open_gpr(struct inode *inode, struct file *file)
{
	struct abox_dbg_file_info *info = abox_proc_data(file);
	struct device *dev_abox = info->abox_data->dev;

	info->buf = vmalloc(info->size);
	if (!info->buf)
		return -ENOMEM;

	if (pm_runtime_get_if_in_use(dev_abox) > 0) {
		abox_core_show_gpr(info->buf);
		pm_runtime_put(dev_abox);
	} else {
		memset(info->buf, 0x0, info->size);
	}

	return simple_open(inode, file);
}

static int abox_dbg_file_open_gicd(struct inode *inode, struct file *file)
{
	struct abox_dbg_file_info *info = abox_proc_data(file);
	struct device *dev_abox = info->abox_data->dev;

	info->buf = vmalloc(info->size);
	if (!info->buf)
		return -ENOMEM;

	if (pm_runtime_get_if_in_use(dev_abox) > 0) {
		abox_gicd_dump(info->abox_data->dev_gic, info->buf, 0,
				info->size);
		pm_runtime_put(dev_abox);
	} else {
		memset(info->buf, 0x0, info->size);
	}

	return simple_open(inode, file);
}

static int abox_dbg_file_release(struct inode *inode, struct file *file)
{
	struct abox_dbg_file_info *info = abox_proc_data(file);

	vfree(info->buf);

	return 0;
}

static const struct proc_ops abox_dbg_file_fops_sram = {
	.proc_read =	abox_dbg_file_read,
	.proc_open =	abox_dbg_file_open_sram,
	.proc_release =	abox_dbg_file_release,
	.proc_lseek =	default_llseek,
};

static const struct proc_ops abox_dbg_file_fops_dram = {
	.proc_read =	abox_dbg_file_read,
	.proc_open =	abox_dbg_file_open_dram,
	.proc_release =	abox_dbg_file_release,
	.proc_lseek =	default_llseek,
};

static const struct proc_ops abox_dbg_file_fops_sfr = {
	.proc_read =	abox_dbg_file_read,
	.proc_open =	abox_dbg_file_open_sfr,
	.proc_release =	abox_dbg_file_release,
	.proc_lseek =	default_llseek,
};

static const struct proc_ops abox_dbg_file_fops_gpr = {
	.proc_read =	abox_dbg_file_read,
	.proc_open =	abox_dbg_file_open_gpr,
	.proc_release =	abox_dbg_file_release,
	.proc_lseek =	default_llseek,
};

static const struct proc_ops abox_dbg_file_fops_gicd = {
	.proc_read =	abox_dbg_file_read,
	.proc_open =	abox_dbg_file_open_gicd,
	.proc_release =	abox_dbg_file_release,
	.proc_lseek =	default_llseek,
};

static struct abox_dbg_file_info sram_info = {
	.name = "sram",
	.fops = &abox_dbg_file_fops_sram,
};
static struct abox_dbg_file_info dram_info = {
	.name = "dram",
	.fops = &abox_dbg_file_fops_dram,
};
static struct abox_dbg_file_info log_info = {
	.name = "log",
	.fops = &abox_dbg_file_fops_dram,
};
static struct abox_dbg_file_info slog_info = {
	.name = "slog",
	.fops = &abox_dbg_file_fops_dram,
};
static struct abox_dbg_file_info sfr_info = {
	.name = "sfr",
	.fops = &abox_dbg_file_fops_sfr,
};
static struct abox_dbg_file_info gpr_info = {
	.name = "gpr",
	.fops = &abox_dbg_file_fops_gpr,
};
static struct abox_dbg_file_info gicd_info = {
	.name = "gicd",
	.fops = &abox_dbg_file_fops_gicd,
};

static int abox_dbg_create_file(struct abox_dbg_file_info *info,
		struct proc_dir_entry *parent, struct abox_data *data,
		void __iomem *io_base, void *base, size_t size)
{
	info->abox_data = data;
	info->io_base = io_base;
	info->base = base;
	info->size = size;
	info->file = abox_proc_create_file(info->name, 0444, parent, info->fops,
			info, info->size);

	return PTR_ERR_OR_ZERO(info->file);
}

static void abox_dbg_create_files(struct abox_data *data)
{
	struct proc_dir_entry *dir;

	dir = abox_proc_mkdir("runtime", NULL);

	abox_dbg_create_file(&sram_info, dir, data, data->sram_base,
			NULL, SRAM_FIRMWARE_SIZE);
	abox_dbg_create_file(&dram_info, dir, data, NULL,
			data->dram_base, DRAM_FIRMWARE_SIZE);
	abox_dbg_create_file(&log_info, dir, data, NULL,
			data->dram_base + data->log_addr, ABOX_LOG_SIZE);
	abox_dbg_create_file(&slog_info, dir, data, NULL,
			data->slog_base, data->slog_size);
	abox_dbg_create_file(&sfr_info, dir, data, data->sfr_base,
			NULL, data->sfr_size);
	abox_dbg_create_file(&gpr_info, dir, data, NULL,
			NULL, PAGE_SIZE);
	abox_dbg_create_file(&gicd_info, dir, data, NULL,
			NULL, PAGE_SIZE);
}

void abox_dbg_create(struct abox_data *data)
{
	struct device *dev = &p_pdev->dev;
	int i, ret;

	abox_dbg(dev, "%s\n", __func__);

	abox_dbg_dump_create_file(data);
	abox_dbg_create_files(data);
	ret = device_create_file(dev, &dev_attr_gpr);
	if (ret < 0)
		abox_warn(dev, "Failed to create file: %s\n",
				dev_attr_gpr.attr.name);
	bin_attr_calliope_sram.size = SRAM_FIRMWARE_SIZE;
	bin_attr_calliope_sram.private = data->sram_base;
	bin_attr_calliope_dram.private = data->dram_base;
	bin_attr_calliope_log.private = data->dram_base + data->log_addr;
	if (data->slog_size >= ABOX_SLOG_DATA_OFFSET) {
		bin_attr_calliope_slog.size = data->slog_size - ABOX_SLOG_DATA_OFFSET;
		bin_attr_calliope_slog.private = data->slog_base + ABOX_SLOG_DATA_OFFSET;
	} else {
		bin_attr_calliope_slog.size = data->slog_size;
		bin_attr_calliope_slog.private = data->slog_base;
	}
	for (i = 0; i < ARRAY_SIZE(calliope_bin_attrs); i++) {
		struct bin_attribute *battr = calliope_bin_attrs[i];

		ret = device_create_bin_file(dev, battr);
		if (ret < 0)
			abox_warn(dev, "Failed to create file: %s\n",
					battr->attr.name);
	}
}
static int samsung_abox_debug_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device *dev_abox = dev->parent;
	struct abox_data *data = dev_get_drvdata(dev_abox);
	ssize_t new_size;

	abox_dbg(dev, "%s\n", __func__);

	p_pdev = pdev;

	if (!abox_dbg_rmem) {
		struct device_node *np_tmp;

		np_tmp = of_parse_phandle(dev->of_node, "memory-region", 0);
		if (np_tmp)
			abox_dbg_rmem = of_reserved_mem_lookup(np_tmp);
	}

	if (abox_dbg_rmem) {
		new_size = abox_oem_resize_reserved_memory(ABOX_OEM_RESERVED_MEMORY_DBG);
		if (new_size >= 0)
			abox_dbg_resize_rmem(dev, abox_dbg_rmem, new_size, "abox_dbg");

		abox_dbg_rmem_init(data);
	}

	if (!abox_dbg_slog) {
		struct device_node *np_tmp;

		np_tmp = of_parse_phandle(dev->of_node, "memory-region", 1);
		if (np_tmp)
			abox_dbg_slog = of_reserved_mem_lookup(np_tmp);
	}

	if (abox_dbg_slog) {
		new_size = abox_oem_resize_reserved_memory(ABOX_OEM_RESERVED_MEMORY_SLOG);
		if (new_size >= 0)
			abox_dbg_resize_rmem(dev, abox_dbg_slog, new_size, "abox_slog");

		abox_dbg_slog_init(data);
	}

	abox_proc_symlink_dev(dev, "debug");

	return 0;
}

static int samsung_abox_debug_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	abox_dbg(dev, "%s\n", __func__);

	return 0;
}

static const struct of_device_id samsung_abox_debug_match[] = {
	{
		.compatible = "samsung,abox-debug",
	},
	{},
};
MODULE_DEVICE_TABLE(of, samsung_abox_debug_match);

struct platform_driver samsung_abox_debug_driver = {
	.probe  = samsung_abox_debug_probe,
	.remove = samsung_abox_debug_remove,
	.driver = {
		.name = "abox-debug",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(samsung_abox_debug_match),
	},
};
