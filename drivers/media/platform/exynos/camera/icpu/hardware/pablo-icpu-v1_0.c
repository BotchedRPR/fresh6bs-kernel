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

#include <linux/regmap.h>

#include "../pablo-icpu.h"
#include "pablo-icpu-hw.h"

static struct icpu_logger _log = {
	.level = LOGLEVEL_INFO,
	.prefix = "[ICPU-HW-V1_0]",
};

struct icpu_logger *get_icpu_hw_log(void)
{
	return &_log;
}

/* HW CONFIGS */

#define ICPU_CPU_REMAPS0_NS_REG_OFFSET 0x0
#define ICPU_CPU_REMAPS0_S_REG_OFFSET 0x4
#define ICPU_CPU_REMAPD0_NS_REG_OFFSET 0x8
#define ICPU_CPU_REMAPD0_S_REG_OFFSET 0xC
#define ICPU_CPU_WFI_STATUS 0xC4

#define ICPU_SYSCTRL_DBG_REG0 0x0C
#define ICPU_SYSCTRL_DBG_REG1 0x10
#define ICPU_SYSCTRL_DBG_REG2 0x14
#define ICPU_SYSCTRL_DBG_REG3 0x18
#define ICPU_SYSCTRL_AXI_TRANSFER_BLOCK 0x24
#define ICPU_SYSCTRL_NUM_ACTIVE_TRANSACTION 0x28

#define ICPU_CSTIRE_DMA_FLUSH 0x30
#define ICPU_CSTIRE_STATUS 0x34

static int __wait_reg_val(void __iomem *addr, u32 val, u32 timeout_ms)
{
	const unsigned long duration = 150;
	u32 retry = timeout_ms * 1000 / duration / 2;

	while ((ioread32(addr) != val) && --retry)
		fsleep(duration);

	return retry ? 0 : -ETIMEDOUT;
}

static int __wait_reg_field(void __iomem *addr, u32 mask, u32 val, u32 timeout_ms)
{
	const unsigned long duration = 150;
	u32 retry = timeout_ms * 1000 / duration / 2;

	while (((ioread32(addr) & mask) != val) && --retry)
		fsleep(duration);

	return retry ? 0 : -ETIMEDOUT;
}

/* Enables or releases the nCORERESET
   0 = Enables(Hold, Assert) reset
   1 = Releases reset
 */
static int __set_icpu_reset(struct regmap *reg_reset,
		unsigned int bit, unsigned int on)
{
	int ret;
	unsigned int mask;
	unsigned int val;

	if (!reg_reset) {
		ICPU_ERR("No sysreg remap");
		return 0;
	}

	mask = BIT(bit);
	val = on ? BIT(bit) : 0;

	ret = regmap_update_bits(reg_reset, 0, mask, val);
	if (ret)
		ICPU_ERR("Failed to icpu sysreg %s (reset bit=%d)", on ? "Release" : "Enable", bit);

	return ret;
}

#if (IS_ENABLED(CONFIG_CAMERA_CIS_ZEBU_OBJ))
static int __set_hw_cfg_for_veloce(void)
{
	void __iomem *reg;

	/* ICPU: s2mpu*/
	reg = ioremap(0x18070054, 0x4);
	writel(0xFF, reg);
	iounmap(reg);

	/* TZPC control non-secure*/
	reg = ioremap(0x18010204, 0x4);
	writel(0xFFFFFFFF, reg);
	iounmap(reg);

	/* TZPC control non-secure*/
	reg = ioremap(0x18010214 , 0x4);
	writel(0xFFFFFFFF, reg);
	iounmap(reg);

	/* TZPC AXPROT_SEL */
	reg = ioremap(0x18020400 , 0x4);
	writel(0xF, reg);
	iounmap(reg);

	/* TZPC AXPROT_VAL */
	reg = ioremap(0x18020404 , 0x4);
	writel(0x3, reg);
	iounmap(reg);

	return 0;
}
#endif

static int __set_base_addr(void __iomem *base_addr, u32 dst_addr)
{

	if (!base_addr)
		return -EINVAL;

	if (dst_addr == 0)
		return -EINVAL;

	/* source
	   [31-1] : upper 31bit of source address. normally 0.
	   [0] : remap enable
	 */
	writel(1, base_addr + ICPU_CPU_REMAPS0_NS_REG_OFFSET);
	writel(1, base_addr + ICPU_CPU_REMAPS0_S_REG_OFFSET);

	writel(dst_addr, base_addr + ICPU_CPU_REMAPD0_NS_REG_OFFSET);
	writel(dst_addr, base_addr + ICPU_CPU_REMAPD0_S_REG_OFFSET);

	return 0;
}

static int __wait_wfi_state_timeout(void __iomem *base_addr, u32 timeout_ms)
{
	int ret = 0;
	u32 wait_time = 0;

	do {
		if (readl(base_addr + ICPU_CPU_WFI_STATUS) == 1) {
			ICPU_INFO("wait ICPU_CPU_WFI_STATUS for %d ms", wait_time);
			break;
		}

		msleep(10);
		wait_time += 10;
	} while (wait_time <= timeout_ms);

	if (wait_time > timeout_ms) {
		ICPU_ERR("wait ICPU_CPU_WFI_STATUS timeout!!");
		ret = -ETIMEDOUT;
	}

	return ret;
}

static void __force_powerdown(void __iomem *sysctrl_base, void __iomem *cstore_base)
{
	ICPU_TIME_DECLARE();
	int ret;

	ICPU_TIME_BEGIN();

	ICPU_INFO("Enter force powerdown sequence active transaction(%x), c_store_status(%x)",
			ioread32(sysctrl_base + ICPU_SYSCTRL_NUM_ACTIVE_TRANSACTION),
			ioread32(cstore_base + ICPU_CSTIRE_STATUS));

	/* Enable CA32 Transaction block */
	iowrite32(0x1, sysctrl_base + ICPU_SYSCTRL_AXI_TRANSFER_BLOCK);

	/* DMA Flush */
	iowrite32(0x3, cstore_base + ICPU_CSTIRE_DMA_FLUSH);

	/* Wait until # of CA32 transaction is 0 */
	ret = __wait_reg_val(sysctrl_base + ICPU_SYSCTRL_NUM_ACTIVE_TRANSACTION, 0x0, 3000);
	ICPU_ERR_IF(ret, "Wait CA32 transaction 0 timeout!!, ICPU_SYSCTRL_NUM_ACTIVE_TRANSACTION(0x%x)",
			ioread32(sysctrl_base + ICPU_SYSCTRL_NUM_ACTIVE_TRANSACTION));

	/* Wait DMA flush is done */
	ret = __wait_reg_field(cstore_base + ICPU_CSTIRE_STATUS, 0x24, 0x24, 6000);
	ICPU_ERR_IF(ret, "Wait DMA flush timeout!!, C_STORE_STATUS(0x%x)",
			ioread32(cstore_base + ICPU_CSTIRE_STATUS));

	/* Q-Ch Dependency remove */
	iowrite32(0x0, sysctrl_base + 0x1c);

	ICPU_INFO("Exit force powerdown sequence");
}

static void __panic_handler(void __iomem *sysctrl_base)
{
	ICPU_INFO("Print ICPU Debug regs: 0x%x, 0x%x, 0x%x, 0x%x",
			ioread32(sysctrl_base + ICPU_SYSCTRL_DBG_REG0),
			ioread32(sysctrl_base + ICPU_SYSCTRL_DBG_REG1),
			ioread32(sysctrl_base + ICPU_SYSCTRL_DBG_REG2),
			ioread32(sysctrl_base + ICPU_SYSCTRL_DBG_REG3));
}

void icpu_hw_init(struct icpu_hw *hw)
{
	memset(hw, 0x0, sizeof(struct icpu_hw));

	hw->set_base_addr = __set_base_addr;
#if (IS_ENABLED(CONFIG_CAMERA_CIS_ZEBU_OBJ))
	hw->hw_misc_prepare = __set_hw_cfg_for_veloce;
#endif
	hw->reset = __set_icpu_reset;
	hw->wait_wfi_state_timeout = __wait_wfi_state_timeout;
	hw->force_powerdown = __force_powerdown;
	hw->panic_handler = __panic_handler;
}
KUNIT_EXPORT_SYMBOL(icpu_hw_init);
