// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos SoC series Pablo driver
 *
 * Exynos Pablo image subsystem functions
 *
 * Copyright (c) 2022 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "pablo-kunit-test.h"
#include "pablo-kunit-test-utils.h"
#include "votf/pablo-votf.h"
#include "is-core.h"
#include "is-hw-chain.h"

extern struct pablo_kunit_subdev_mcs_func *pablo_kunit_get_subdev_mcs_func(void);
static struct pablo_kunit_subdev_mcs_func *fn;
static struct is_device_ischain *idi;
static struct is_subdev *mcs;
static struct is_frame *frame;

static void pablo_subdev_mcs_tag_hf_kunit_test(struct kunit *test)
{
	int ret;
	struct param_mcs_output *mcs_output;
	struct is_framemgr *votf_framemgr;
	dma_addr_t dva = 0x12345678;

	/* VOTF OFF */
	ret = fn->mcs_tag_hf(idi, mcs, frame);
	KUNIT_EXPECT_TRUE(test, ret == 0);

	/* VOTF ON */
	mcs_output = pablo_kunit_get_param_from_device(idi, PARAM_MCS_OUTPUT5);
	mcs_output->dma_cmd = DMA_OUTPUT_VOTF_ENABLE;
	mcs_output->plane = 1;

	ret = fn->mcs_tag_hf(idi, mcs, frame);
	KUNIT_EXPECT_TRUE(test, ret == -EINVAL);
	ret = pablo_kunit_compare_param(frame, PARAM_MCS_OUTPUT5, mcs_output);
	KUNIT_EXPECT_TRUE(test, ret == 0);

	/* VOTF ON & set VOTF FRAME */
	votf_framemgr = GET_SUBDEV_I_FRAMEMGR(mcs);
	votf_framemgr->frames = kunit_kzalloc(test, sizeof(struct is_frame), 0);
	votf_framemgr->frames[0].dvaddr_buffer[0] = dva;
	votf_framemgr->frames[0].planes = 1;

	ret = fn->mcs_tag_hf(idi, mcs, frame);
	KUNIT_EXPECT_TRUE(test, ret == 0);
	KUNIT_EXPECT_TRUE(test, frame->sc5TargetAddress[0] == dva);
	KUNIT_EXPECT_TRUE(test, frame->dva_rgbp_hf[0] == dva);

	kunit_kfree(test, votf_framemgr->frames);
}

static int pablo_subdev_mcs_kunit_test_init(struct kunit *test)
{
	fn = pablo_kunit_get_subdev_mcs_func();
	idi = kunit_kzalloc(test, sizeof(struct is_device_ischain), 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, idi);

	idi->is_region = kunit_kzalloc(test, sizeof(struct is_region), 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, idi->is_region);

	mcs = &idi->group_mcs.leader;

	frame = kunit_kzalloc(test, sizeof(struct is_frame), 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, frame);

	frame->parameter = kunit_kzalloc(test, sizeof(struct is_param_region), 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, frame->parameter);

	votfitf_kunit_mock_init(test);

	return 0;
}

static void pablo_subdev_mcs_kunit_test_exit(struct kunit *test)
{
	votfitf_kunit_mock_deinit(test);
	kunit_kfree(test, frame->parameter);
	kunit_kfree(test, frame);
	kunit_kfree(test, idi->is_region);
	kunit_kfree(test, idi);
}

static struct kunit_case pablo_subdev_mcs_kunit_test_cases[] = {
	KUNIT_CASE(pablo_subdev_mcs_tag_hf_kunit_test),
	{},
};

struct kunit_suite pablo_subdev_mcs_kunit_test_suite = {
	.name = "pablo-subdev-mcs-kunit-test",
	.init = pablo_subdev_mcs_kunit_test_init,
	.exit = pablo_subdev_mcs_kunit_test_exit,
	.test_cases = pablo_subdev_mcs_kunit_test_cases,
};
define_pablo_kunit_test_suites(&pablo_subdev_mcs_kunit_test_suite);

MODULE_LICENSE("GPL v2");
