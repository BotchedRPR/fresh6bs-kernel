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
#include <linux/firmware.h>

#include "pablo-icpu-core.h"
#include "pablo-icpu-firmware.h"

static struct icpu_logger _log = {
	.level = LOGLEVEL_INFO,
	.prefix = "[ICPU-FIRMWARE]",
};

struct icpu_logger *get_icpu_firmware_log(void)
{
	return &_log;
}

static struct icpu_fw_config {
	bool fw_mem_alloc_boot;
} _config = { .fw_mem_alloc_boot = false, };

#define ICPU_FIRMWARE_VERSION_OFFSET 80
#define ICPU_FIRMWARE_MEM_SIZE_OFFSET (ICPU_FIRMWARE_VERSION_OFFSET + 4)
#define ICPU_FIRMWARE_BINARY_NAME "pablo_icpufw.bin"
static int __alloc_firmware_mem(struct icpu_core *core, size_t size)
{
	struct is_priv_buf *pb_icpu;

	pb_icpu = CALL_PTR_MEMOP(&core->mem, alloc, core->mem.priv, size, "system-uncached", 0);
	if (IS_ERR_OR_NULL(pb_icpu))
		goto alloc_fail;

	core->fw_dmabuf = pb_icpu;
	core->fw_kva = CALL_BUFOP(pb_icpu, kvaddr, pb_icpu);
	core->fw_dva = CALL_BUFOP(pb_icpu, dvaddr, pb_icpu);

	return 0;

alloc_fail:
	core->fw_dmabuf = NULL;

	return -ENOMEM;
}

static void __free_firmware_mem(struct icpu_core *core)
{
	if (core->fw_dmabuf)
		CALL_VOID_BUFOP(core->fw_dmabuf, free, core->fw_dmabuf);

	core->fw_dmabuf = NULL;
}

static inline void __print_firmware_version(const struct firmware *fw_entry)
{
	ICPU_INFO("%s", fw_entry->data + fw_entry->size - ICPU_FIRMWARE_VERSION_OFFSET);
}

#define ICPU_FIRMWARE_SIZE_LEN (4) /* 4 character for firmware size */
static size_t __get_size_firmware(const struct firmware *fw_entry)
{
	int ret;
	char carray[5] = { 0, };
	size_t size = 0;

	memcpy(carray, fw_entry->data + fw_entry->size - ICPU_FIRMWARE_MEM_SIZE_OFFSET, ICPU_FIRMWARE_SIZE_LEN);
	ret = kstrtol(carray, 16, &size);
	if (ret) {
		ICPU_ERR("Get fw mem size fail, str:%s, ret(%d)", carray, ret);
		return 0;
	}

	/* Size in the binary is upper 16bit */
	size = size << 16;

	ICPU_INFO("Firmware mem size str:%s, 0x%x, %dMB", carray, size, size / 1024 / 1024);

	return size;
}

int init_firmware_mem(struct icpu_core *core, size_t size)
{
	return __alloc_firmware_mem(core, size);
}

void deinit_firmware_mem(struct icpu_core *core)
{
	__free_firmware_mem(core);
}

void config_firmware(bool fw_mem_alloc_boot)
{
	_config.fw_mem_alloc_boot = fw_mem_alloc_boot;
}

#define ICPU_FIRMWARE_MEMORY_SIZE (size_t)(19 * SZ_1M)
static int __prepare_fw_mem(struct icpu_core *core, size_t size)
{
	int ret = 0;
	bool need_alloc = false;
	bool buf_size = core->fw_dmabuf ? core->fw_dmabuf->size : 0;

	if (_config.fw_mem_alloc_boot) {
		if (buf_size == 0 || (size && buf_size != size)) {
			ICPU_ERR("Firmware mem size(%d) mismatch to actual(%d), re-allocate",
					size, buf_size);
			deinit_firmware_mem(core);
			size = size ? size : ICPU_FIRMWARE_MEMORY_SIZE;
			need_alloc = true;
		}
	} else {
		if (size < ICPU_FIRMWARE_MEMORY_SIZE) {
			ICPU_WARN("Firmware mem size(%d) is smaller than expect(%d)",
					size, ICPU_FIRMWARE_MEMORY_SIZE);
			size = ICPU_FIRMWARE_MEMORY_SIZE;
		}
		need_alloc = true;
	}

	if (need_alloc)
		ret = init_firmware_mem(core, size);

	return ret;
}

int load_firmware(struct icpu_core *core)
{
	int ret;
	size_t size = 0;
	const struct firmware *fw_entry;
	char *name = ICPU_FIRMWARE_BINARY_NAME;

	/* request_firmware */
	ret = request_firmware(&fw_entry, name, get_icpu_dev(core));
	if (ret)
		return ret;

	__print_firmware_version(fw_entry);

	size = __get_size_firmware(fw_entry);

	ret = __prepare_fw_mem(core, size);
	if (ret)
		goto prepare_fw_mem_fail;

	memcpy((void *)core->fw_kva, fw_entry->data, fw_entry->size);

	if (!IS_ENABLED(ICPU_IO_COHERENCY))
		CALL_BUFOP(core->fw_dmabuf, sync_for_device, core->fw_dmabuf,
				0, fw_entry->size, DMA_TO_DEVICE);

prepare_fw_mem_fail:
	release_firmware(fw_entry);

	return ret;
}

void teardown_firmware(struct icpu_core *core)
{
	if (!_config.fw_mem_alloc_boot)
		deinit_firmware_mem(core);
}
