/*
 * Exynos FMP test driver
 *
 * Copyright (C) 2015 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/miscdevice.h>
#include <linux/crypto.h>
#include <linux/buffer_head.h>
#include <linux/genhd.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <crypto/authenc.h>
#include <crypto/fmp.h>

#include "fmp_fips_main.h"
#include "fmp_fips_fops.h"
#include "fmp_test.h"
#include "fmp_testvec.h"
#include "fmp_fips_cipher.h"

static const char pass[] = "passed";
static const char fail[] = "failed";

static int TestVectorNum;
static int TestSetOverall;
static int TestKeyNum = -1;
static int zeroization;

static void hexdump(uint8_t *buf, uint32_t len)
{
	print_hex_dump(KERN_CONT, "", DUMP_PREFIX_OFFSET,
			16, 1,
			buf, len, false);
}

static int alloc_buf(char *buf[XBUFSIZE])
{
	int i;

	for (i = 0; i < XBUFSIZE; i++) {
		buf[i] = (void *)__get_free_page(GFP_KERNEL);
		if (!buf[i])
			goto err_free_buf;
	}

	return 0;

err_free_buf:
	while (i-- > 0)
		free_page((unsigned long)buf[i]);

	return -ENOMEM;
}

static void free_buf(char *buf[XBUFSIZE])
{
	int i;

	for (i = 0; i < XBUFSIZE; i++)
		free_page((unsigned long)buf[i]);
}

static ssize_t fmp_simulator_selftest_run_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct exynos_fmp *fmp = get_fmp();

	exynos_fmp_fips_test(fmp);

	return count;
}

static ssize_t fmp_fips_set_tv_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t len)
{
	int tvnum = 0;
	int i;

	if (len <= 1) {
		dev_err(dev, "%s: fail to set test vector number, because of param\n", __func__);
		goto out;
	}

	for (i = 0; i < len - 1; i++) {
		tvnum *= 10;
		tvnum += (buf[i] - '0');
	}

	if (tvnum >= SIM_XTS_TEST_VECTORS) {
		dev_err(dev, "%s: fail to set (%d) test vector number\n", __func__, tvnum);
		goto out;
	}

	TestVectorNum = tvnum;
	dev_info(dev, "%s: success to set (%d) test vector number\n", __func__, TestVectorNum);

out:
	return len;
}

static ssize_t fmp_fips_set_tk_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t len)
{
	int tknum = 0;
	int i;

	if (len <= 1) {
		dev_err(dev, "%s: fail to set test key, because of param\n", __func__);
		goto out;
	}

	for (i = 0; i < len - 1; i++) {
		tknum *= 10;
		tknum += (buf[i] - '0');
	}

	if (tknum >= SIM_XTS_TEST_VECTORS) {
		dev_err(dev, "%s: fail to set (%d) test key\n", __func__, tknum);
		goto out;
	}

	TestKeyNum = tknum;
	dev_info(dev, "%s: success to set (%d) test key\n", __func__, TestKeyNum);

out:
	return len;
}

static ssize_t fmp_fips_zeroise_key_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t len)
{
	if (zeroization) {
		zeroization = 0;
		dev_info(dev, "Turn OFF printing zeroization (%d)\n", zeroization);
	}
	else {
		zeroization = 1;
		dev_info(dev, "Turn ON printing zeroization (%d)\n", zeroization);
	}

	return len;
}

static ssize_t fmp_fips_encrypt_data_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct exynos_fmp *fmp = get_fmp();
	char *inbuf[XBUFSIZE];
	char *outbuf[XBUFSIZE];
	void *indata;
	void *outdata;
	int ret;
	int tk;

	TestSetOverall = fmp->result.overall;
	fmp->result.overall = 0;
	dev_info(dev, "%s: save result.overall (%d)\n", __func__, TestSetOverall);

	if (alloc_buf(inbuf)) {
		dev_err(dev, "%s: Fail to alloc input buf.\n", __func__);
		goto err_alloc_inbuf;
	}

	if (alloc_buf(outbuf)) {
		dev_err(dev, "%s: Fail to alloc input buf.\n", __func__);
		goto err_alloc_outbuf;
	}

	fmp->test_data = fmp_test_init(fmp);
	if (!fmp->test_data) {
		dev_err(dev, "%s: fails to initialize fips test.\n", __func__);
		goto out;
	}

	fmp->test_data->zeroization = zeroization;

	indata = inbuf[0];
	outdata = outbuf[0];
	memset(indata, 0, FMP_BLK_SIZE);
	memset(outdata, 0, FMP_BLK_SIZE);
	memcpy(indata, aes_xts_KAT[TestVectorNum].input, aes_xts_KAT[TestVectorNum].ilen);

	ret = fmp_cipher_set_iv(fmp->test_data, aes_xts_KAT[TestVectorNum].iv, 16);
	if (ret) {
		dev_err(dev, "%s: Fail to set IV. ret(%d)\n", __func__, ret);
		goto out;
	}

	fmp->test_data->ci.algo_mode = EXYNOS_FMP_ALGO_MODE_AES_XTS;

	if(TestKeyNum == -1)
		tk = TestVectorNum;
	else
		tk = TestKeyNum;

	ret = fmp_cipher_set_key(fmp->test_data, aes_xts_KAT[tk].key, aes_xts_KAT[tk].klen);
	if (ret) {
		dev_err(dev, "%s: Fail to set %d key. ret(%d)\n",
				__func__, tk, ret);
		goto out;
	}

	ret = fmp_cipher_run(fmp, fmp->test_data, indata, aes_xts_KAT[TestVectorNum].ilen,
				0, WRITE_MODE, NULL, &fmp->test_data->ci);
	if (ret) {
		dev_err(dev, "%s: Fail to run. ret(%d)\n", __func__, ret);
		goto out;
	}

	dev_info(dev, "%s: plain text\n", __func__);
	hexdump(indata, aes_xts_KAT[TestVectorNum].ilen);
	dev_info(dev, "%s: Success to write data\n", __func__);

	ret = fmp_cipher_run(fmp, fmp->test_data, outdata, aes_xts_KAT[TestVectorNum].rlen, 1,
				READ_MODE, NULL, &fmp->test_data->ci);
	if (ret) {
		dev_err(dev, "%s: Fail to run. ret(%d)\n", __func__, ret);
		goto out;
	}

	dev_info(dev, "%s: written Cipher text\n", __func__);
	hexdump(outdata, aes_xts_KAT[TestVectorNum].rlen);
	dev_info(dev, "%s: Success to read data\n", __func__);

out:
	free_buf(outbuf);
err_alloc_outbuf:
	free_buf(inbuf);
err_alloc_inbuf:
	fmp->result.overall = TestSetOverall;
	return len;
}

static ssize_t fmp_fips_decrypt_data_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct exynos_fmp *fmp = get_fmp();
	char *outbuf[XBUFSIZE];
	void *outdata;
	int ret;
	int tk;

	TestSetOverall = fmp->result.overall;
	fmp->result.overall = 0;
	dev_info(dev, "%s: save result.overall (%d)\n", __func__, TestSetOverall);

	if (alloc_buf(outbuf)) {
		dev_err(dev, "%s: Fail to alloc input buf.\n", __func__);
		goto err_alloc_outbuf;
	}

	fmp->test_data = fmp_test_init(fmp);
	if (!fmp->test_data) {
		dev_err(dev, "%s: fails to initialize fips test.\n", __func__);
		goto out;
	}

	fmp->test_data->zeroization = zeroization;

	outdata = outbuf[0];
	memset(outdata, 0, FMP_BLK_SIZE);

	ret = fmp_cipher_set_iv(fmp->test_data, aes_xts_KAT[TestVectorNum].iv, 16);
	if (ret) {
		dev_err(dev, "%s: Fail to set IV. ret(%d)\n", __func__, ret);
		goto out;
	}

	fmp->test_data->ci.algo_mode = EXYNOS_FMP_ALGO_MODE_AES_XTS;

	if(TestKeyNum == -1)
		tk = TestVectorNum;
	else
		tk = TestKeyNum;

	ret = fmp_cipher_set_key(fmp->test_data, aes_xts_KAT[tk].key, aes_xts_KAT[tk].klen);
	if (ret) {
		dev_err(dev, "%s: Fail to set %d key. ret(%d)\n",
				 __func__, tk, ret);
		goto out;
	}

	ret = fmp_cipher_run(fmp, fmp->test_data, outdata, aes_xts_KAT[TestVectorNum].rlen, 0,
				READ_MODE, NULL, &fmp->test_data->ci);
	if (ret) {
		dev_err(dev, "%s: Fail to run. ret(%d)\n", __func__, ret);
		goto out;
	}

	dev_info(dev, "%s: Cipher text\n", __func__);
	hexdump(outdata, aes_xts_KAT[TestVectorNum].rlen);
	dev_info(dev, "%s: Success to read data\n", __func__);

out:
	free_buf(outbuf);
err_alloc_outbuf:
	fmp->result.overall = TestSetOverall;
	return len;
}

static DEVICE_ATTR_WO(fmp_simulator_selftest_run);
static DEVICE_ATTR_WO(fmp_fips_set_tv);
static DEVICE_ATTR_WO(fmp_fips_set_tk);
static DEVICE_ATTR_WO(fmp_fips_zeroise_key);
static DEVICE_ATTR_WO(fmp_fips_encrypt_data);
static DEVICE_ATTR_WO(fmp_fips_decrypt_data);

static struct attribute *fmp_fips_attr[] = {
	&dev_attr_fmp_simulator_selftest_run.attr,
	&dev_attr_fmp_fips_set_tv.attr,
	&dev_attr_fmp_fips_set_tk.attr,
	&dev_attr_fmp_fips_zeroise_key.attr,
	&dev_attr_fmp_fips_encrypt_data.attr,
	&dev_attr_fmp_fips_decrypt_data.attr,
	NULL,
};

static struct attribute_group fmp_fips_attr_group = {
	.name	= "fmp-simulator",
	.attrs	= fmp_fips_attr,
};

static const struct file_operations fmp_fips_fops = {
	.owner		= THIS_MODULE,
	.open		= fmp_fips_open,
	.release	= fmp_fips_release,
	.unlocked_ioctl = fmp_fips_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= fmp_fips_compat_ioctl,
#endif
};

int exynos_fmp_fips_init(struct exynos_fmp *fmp)
{
	int ret = 0;

	if (!fmp || !fmp->dev) {
		pr_err("%s: Invalid exynos fmp dev\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	fmp->test_data = fmp_test_init(fmp);
	if (!fmp->test_data) {
		dev_err(fmp->dev,
			"%s: fails to initialize fips test.\n", __func__);
		ret = -EINVAL;
		goto out;
	}

out:
	return ret;
}

int exynos_fmp_fips_register(struct exynos_fmp *fmp)
{
	int ret;

	if (!fmp || !fmp->dev) {
		pr_err("%s: Invalid exynos fmp dev\n", __func__);
		goto err;
	}

	fmp->miscdev.minor = MISC_DYNAMIC_MINOR;
	fmp->miscdev.name = "fmp";
	fmp->miscdev.fops = &fmp_fips_fops;
	ret = misc_register(&fmp->miscdev);
	if (ret) {
		dev_err(fmp->dev, "%s: Fail to register misc device. ret(%d)\n",
				__func__, ret);
		goto err;
	}

	ret = sysfs_create_group(&fmp->dev->kobj, &fmp_fips_attr_group);
	if (ret) {
		dev_err(fmp->dev, "%s: Fail to create sysfs. ret(%d)\n",
				__func__, ret);
		goto err_misc;
	}

	dev_info(fmp->dev, "%s: FMP register misc device. ret(%d)\n",
			__func__, ret);
	return 0;

err_misc:
	misc_deregister(&fmp->miscdev);
err:
	return -EINVAL;
}

void exynos_fmp_fips_deregister(struct exynos_fmp *fmp)
{
	sysfs_remove_group(&fmp->dev->kobj, &fmp_fips_attr_group);
	misc_deregister(&fmp->miscdev);
}
