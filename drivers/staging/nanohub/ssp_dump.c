/*
 *  Copyright (C) 2018, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/time.h>
//#include <linux/io.h>
//#include <linux/slab.h>
//#include <linux/vmalloc.h>

#include "ssp.h"
#include "ssp_platform.h"
#include "ssp_dump.h"
#include "ssp_iio.h"
#include "ssp_bigdata.h"
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0))
#include <linux/time64.h>
#define SEC_TIMESPEC timespec64
#define SEC_GETTIMEOFDAY ktime_get_real_ts64
#define SEC_RTC_TIME_TO_TM rtc_time64_to_tm
#else
#include <linux/time.h>
#define SEC_TIMESPEC timeval
#define SEC_GETTIMEOFDAY do_gettimeofday
#define SEC_RTC_TIME_TO_TM rtc_time_to_tm
#endif

#define SENSORHUB_DUMP_NOTI_EVENT               (0xFC)

#define NANOHUB_LOG_MODE_RESET		(1)
#define NANOHUB_LOG_MODE_AUTO		(2)
#define NANOHUB_LOG_MODE_MANUAL		(3)
#define NANOHUB_LOG_MODE_BUGREPORT	(4)

void reset_noti_task(struct work_struct *work)
{
	struct ssp_data *data = container_of((struct work_struct *)work,
							struct ssp_data, work_reset_noti);
	char buffer[4] = {0x02, 0x01, 0x00, 0x00};
	char* log_buf = ssp_get_chub_bin_logbuf_ptr(data);
	unsigned int size = ssp_get_logbuf_write_index(data);

	mutex_lock(&(data->chub_log_dump_mutex));
	memset(data->chub_log_dump_buf, 0, CHUB_LOG_BUF_SIZE);
	memcpy(data->chub_log_dump_buf, log_buf, size);
	mutex_unlock(&(data->chub_log_dump_mutex));

	buffer[2] = SENSORHUB_DUMP_NOTI_EVENT;
	buffer[3] = data->chub_err_type;
	report_scontext_data(data, buffer, 4);

	ssp_infof("log_size=%d", size);
}

void write_ssp_dump_file(struct ssp_data *data, char *info, void *dump, int size, int type, u32 *gpr, int num_gpr, u32 *hardfault_info)
{
	int i;
	struct SEC_TIMESPEC tv;
	struct rtc_time tm;
	unsigned long local_time;

	SEC_GETTIMEOFDAY(&tv);
	local_time = (u32)(tv.tv_sec - (sys_tz.tz_minuteswest * 60));
	SEC_RTC_TIME_TO_TM(local_time, &tm);

	data->chub_err_type = type;

	data->crash_mini_dump.month = tm.tm_mon+1;
	data->crash_mini_dump.day = tm.tm_mday;
	data->crash_mini_dump.hour = tm.tm_hour;
	data->crash_mini_dump.min = tm.tm_min;
	data->crash_mini_dump.sec = tm.tm_sec;

	for (i = 0 ; i <= num_gpr-2 ; i++) {
		data->crash_mini_dump.gpr_dump[i] = gpr[i];
	}
	data->crash_mini_dump.gpr_dump[num_gpr-1] = gpr[num_gpr-1];
	memcpy((u32 *)data->crash_mini_dump.hardfault_info, hardfault_info, sizeof(u32) * HARDFAULT_INFO_LEN);
		
	queue_work(data->debug_wq, &data->work_reset_noti);

	ssp_big_queue_work(data, SSP_BIG_CRASH_MINIDUMP, true);
}

int ssp_dumpstate(struct ssp_data *data, char *out_name)
{
	nanohub_info("[SSP]: %s \n", __func__);

	/* check current dump size & force shorten for over case */
	if(ssp_get_logbuf_write_index(data) > SZ_1M - SENSOR_REG_DUMP_STR_LEN)
		ssp_set_logbuf_write_index(data, SZ_1M - SENSOR_REG_DUMP_STR_LEN);

	sensor_reg_dump_read(data, ACCELEROMETER_SENSOR);
	print_acc_esn_data(data);
	sensor_reg_dump_read(data, PRESSURE_SENSOR);	
	sensor_reg_dump_read(data, AUTO_BRIGHTNESS_SENSOR); 
	sensor_reg_dump_read(data, GEOMAGNETIC_CALIB_SENSOR);

	chub_log_dump_start(data, ssp_get_chub_bin_logbuf_ptr(data), 
		ssp_get_logbuf_write_index(data), true);

	return SUCCESS;
}

unsigned int ssp_nanohub_log_buf_check(struct ssp_data *data)
{
	unsigned int widx;

	widx = ssp_get_logbuf_write_index(data);

	if (widx < data->uLogWIdx && data->uLogWIdx != 0xFFFFFFFF) {
		data->is_log_full = true;
	}
	data->uLogWIdx = widx;

	return widx;
}

void chub_log_dump_start(struct ssp_data *data, char* log_buf, unsigned int size, bool isDumpstate)
{
	mutex_lock(&(data->chub_log_dump_mutex));

	ssp_infof("log_size=%d", size);

	/* mem copy */
	memset(data->chub_log_dump_buf, 0, CHUB_LOG_BUF_SIZE);
	memcpy(data->chub_log_dump_buf, log_buf, size);
	if (isDumpstate) {
		queue_work(data->chub_log_dump_wq, &data->work_chub_dumpstate);
	} else {
		queue_work(data->chub_log_dump_wq, &data->work_chub_log_dump);	
	}

	mutex_unlock(&(data->chub_log_dump_mutex));
}

void chub_log_dump_task(struct work_struct *work)
{
	struct ssp_data *data = container_of((struct work_struct *)work,
							struct ssp_data, work_chub_log_dump);
	char buffer[4] = {0x02, 0x01, 0xFC, 0xA0};

	report_scontext_data(data, buffer, 4);

	ssp_infof("");
}

void chub_dumpstate_task(struct work_struct *work)
{
	struct ssp_data *data = container_of((struct work_struct *)work,
							struct ssp_data, work_chub_dumpstate);
	char buffer[4] = {0x02, 0x01, 0xFC, 0xA1};

	ssp_chub_dump_store(data);

	report_scontext_data(data, buffer, 4);

	ssp_infof("");
}


#define N_MINORS	1
static struct ssp_data *ssp_dump_data = NULL;


static int ssp_log_mmap_open(struct inode *inode, struct file *filp)
{
	ssp_infof("ssp_log_mmap_open");
	return 0;
}

static int ssp_log_mmap_release(struct inode *inode, struct file *filp)
{
	ssp_infof("ssp_log_mmap_release");
	return 0;
}

static int ssp_log_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	/* Ignore the vma->vm_pgoff , we are forcing mmap to
	 * start on offset 0 */
	unsigned long offset = 0;
	unsigned long page, pos;

	ssp_infof();

	if(ssp_dump_data == NULL)
	{
		ssp_infof("ssp_log_mmap_null_pointer_error");
		goto null_pointer_error;
	}

	mutex_lock(&(ssp_dump_data->chub_log_dump_mutex));

	if (size > CHUB_LOG_BUF_SIZE)
	{
		ssp_infof("ssp_log_mmap_read=%u,buf=%u", size, CHUB_LOG_BUF_SIZE);
		goto error;
	}
	if (offset > CHUB_LOG_BUF_SIZE - size)
	{
		ssp_infof("ssp_log_mmap,offset=%u,buf_s-read_s=%u", offset, 
			CHUB_LOG_BUF_SIZE - size);
		goto error;
	}

	if (ssp_dump_data->chub_log_dump_buf == NULL) {
		ssp_infof("ssp_log_mmap_buf is NULL");
		goto error;
	}

	pos = (unsigned long)(ssp_dump_data->chub_log_dump_buf) + offset;

	ssp_infof("ssp_log_mmap size:%u offset:%u", size, offset);

	while (size > 0) {
		page = vmalloc_to_pfn((void *)pos);
		if (remap_pfn_range(vma, start, page, PAGE_SIZE, PAGE_SHARED))
			goto error;

		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}

	mutex_unlock(&(ssp_dump_data->chub_log_dump_mutex));
	return 0;
error:
	mutex_unlock(&(ssp_dump_data->chub_log_dump_mutex));
null_pointer_error:
	return -EAGAIN;
}


static const struct file_operations ssp_log_mmap_fops = {
	.owner          = THIS_MODULE,
	.open           = ssp_log_mmap_open,
	.mmap           = ssp_log_mmap,
	.release        = ssp_log_mmap_release,
};

static int ssp_chub_dram_mmap_open(struct inode *inode, struct file *filp)
{
	ssp_infof("ssp_chub_dram_mmap_open");

	return 0;
}

static int ssp_chub_dram_mmap_release(struct inode *inode, struct file *filp)
{
	ssp_infof("ssp_chub_dram_mmap_release");
	return 0;
}

static int ssp_chub_dram_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	/* Ignore the vma->vm_pgoff , we are forcing mmap to
	 * start on offset 0 */
	unsigned long offset = 0;
	unsigned long page, pos;
	unsigned long *addr = get_sensorhub_dram_ptr();
	int dump_size = get_sensorhub_dram_size();

	ssp_infof(" %d", dump_size);

	if(ssp_dump_data == NULL){
		ssp_infof("ssp_chub_dram_mmap_null_pointer_error");
		goto null_pointer_error;
	}

	mutex_lock(&(ssp_dump_data->chub_dram_dump_mutex));

	if (size > dump_size) {
		ssp_infof("ssp_chub_dram_mmap_read=%u,buf=%u", size, dump_size);
		goto error;
	}
	if (offset > dump_size - size) {
		ssp_infof("ssp_chub_dram_mmap,offset=%u,buf_s-read_s=%u", offset, 
			dump_size - size);
		goto error;
	}

	pos = (unsigned long)(addr) + offset;

	ssp_infof("ssp_chub_dram_mmap size:%u offset:%u", size, offset);

	while (size > 0) {
		page = vmalloc_to_pfn((void *)pos);
		if (remap_pfn_range(vma, start, page, PAGE_SIZE, PAGE_SHARED))
			goto error;

		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}

	mutex_unlock(&(ssp_dump_data->chub_dram_dump_mutex));
	return 0;
error:
	mutex_unlock(&(ssp_dump_data->chub_dram_dump_mutex));
null_pointer_error:
	return -EAGAIN;
}

static const struct file_operations ssp_chub_dram_mmap_fops = {
	.owner          = THIS_MODULE,
	.open           = ssp_chub_dram_mmap_open,
	.mmap           = ssp_chub_dram_mmap,
	.release        = ssp_chub_dram_mmap_release,
};
int initialize_chub_log_dump_node(struct ssp_data *data)
{
	int ret;
	dev_t dev0;
	dev_t dev1;
	bool dev_err = 0;

	ssp_dump_data = data;

	/* map1 */
	ret = alloc_chrdev_region(&(data->ssp_mmap_dev_num), 0, N_MINORS, "ssp_mmap");
	if (ret) {
		ssp_infof("ssp_alloc_chrdev_region failed");
		goto error;
	}

	data->ssp_mmap_dev_major = MAJOR(data->ssp_mmap_dev_num);
	dev0 = MKDEV(MAJOR(data->ssp_mmap_dev_num), 0);
	
	cdev_init(&(data->ssp_mmap_cdev), &ssp_log_mmap_fops);
	ret = cdev_add(&(data->ssp_mmap_cdev), dev0, 1);
	if (ret) {
		ssp_infof("cdev_add failed");
		goto error_cdev;
	}

	/* map2 */
	ret = alloc_chrdev_region(&(data->ssp_mmap_dev_num1), 0, N_MINORS, "ssp_chub_dram_mmap");
	if (ret) {
		ssp_infof("ssp_alloc_chrdev_region failed");
		goto error;
	}

	data->ssp_mmap_dev_major1 = MAJOR(data->ssp_mmap_dev_num1);
	dev1 = MKDEV(MAJOR(data->ssp_mmap_dev_num1), 0);
	
	cdev_init(&(data->ssp_mmap_cdev1), &ssp_chub_dram_mmap_fops);
	ret = cdev_add(&(data->ssp_mmap_cdev1), dev1, 1);
	if (ret) {
		ssp_infof("cdev_add failed");
		goto error_cdev;
	}	

	/* class create - common */
	data->ssp_mmap_class = class_create(THIS_MODULE, "ssp_mmap_class");
	if (IS_ERR(data->ssp_mmap_class)) {
		ret = PTR_ERR(data->ssp_mmap_class);
		goto error_class;
	}


	/* create device */
	data->ssp_mmap_cdev.owner = THIS_MODULE;
	data->ssp_mmap_dev = device_create(data->ssp_mmap_class, NULL, dev0, NULL, "ssp_mmap_dev");
	if (IS_ERR(data->ssp_mmap_dev)) {
		ssp_infof("ssp_mmpa_device_create failed");
		ret = PTR_ERR(data->ssp_mmap_dev);
		cdev_del(&(data->ssp_mmap_cdev));
		dev_err = true;
	}

	data->ssp_mmap_cdev1.owner = THIS_MODULE;
	data->ssp_mmap_dev = device_create(data->ssp_mmap_class, NULL, dev1, NULL, "ssp_dram_mmap_dev");
	if (IS_ERR(data->ssp_mmap_dev)) {
		ssp_infof("ssp_mmpa_device_create failed");
		ret = PTR_ERR(data->ssp_mmap_dev);
		cdev_del(&(data->ssp_mmap_cdev));
		if (dev_err)
			goto error_dev;
	}

	return SUCCESS;

error_class:
	cdev_del(&(data->ssp_mmap_cdev));
	cdev_del(&(data->ssp_mmap_cdev1));	
error_cdev:
	unregister_chrdev_region(data->ssp_mmap_dev_num, N_MINORS);
	unregister_chrdev_region(data->ssp_mmap_dev_num1, N_MINORS);
error_dev:
	class_destroy(data->ssp_mmap_class);	
error:
	return ERROR;

}

void remove_chub_log_dump_node(struct ssp_data *data)
{
	device_destroy(data->ssp_mmap_class, MKDEV(MAJOR(data->ssp_mmap_dev_num), 0));
	cdev_del(&(data->ssp_mmap_cdev));
	cdev_del(&(data->ssp_mmap_cdev1));
	class_destroy(data->ssp_mmap_class);
	unregister_chrdev_region(data->ssp_mmap_dev_num, 1);
	unregister_chrdev_region(data->ssp_mmap_dev_num1, 1);
	kvfree(data->chub_log_dump_buf);
	cancel_work_sync(&data->work_chub_log_dump);
	cancel_work_sync(&data->work_chub_dumpstate);
	destroy_workqueue(data->chub_log_dump_wq);

	mutex_destroy(&data->chub_log_dump_mutex);
	mutex_destroy(&data->chub_dram_dump_mutex);
}

