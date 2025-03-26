/*
 *  Copyright (C) 2015, Samsung Electronics Co. Ltd. All Rights Reserved.
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
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/events.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/types.h>
#include <linux/slab.h>
#include <linux/kfifo.h>

//#include <linux/cpufreq_pmqos_boost.h>

#include "ssp.h"
#include "ssp_iio.h"
#include "ssp_data.h"
#include "ssp_iio.h"
#include "ssp_scontext.h"
#include "ssp_type_define.h"
#include "ssp_bigdata.h"

#define IIO_CHANNEL             -1

// from lego, it is 0(it was 3) 
#define IIO_SCAN_INDEX          0 // 3
#define IIO_SIGN                's'
#define IIO_SHIFT               0

#define META_EVENT              0
#define META_TIMESTAMP          0

#define PROX_AVG_READ_NUM       80
enum {
	PROX_RAW_NUM = 0,
	PROX_RAW_MIN,
	PROX_RAW_SUM,
	PROX_RAW_MAX,
	PROX_RAW_DATA_SIZE,
};

#define SCONTEXT_HEADER_LEN     8
#define SCONTEXT_LENGTH_LEN		4
#define SCONTEXT_START_LEN		2
#define SCONTEXT_END_LEN		2
#define SCONTEXT_DATA_LEN       (SCONTEXT_DATA_SIZE - SCONTEXT_HEADER_LEN) //56
#define MAX_DATA_LEN            4096

#define IIO_ST(si, rb, sb, sh)	\
	{ .sign = si, .realbits = rb, .storagebits = sb, .shift = sh }


static int ssp_preenable(struct iio_dev *indio_dev)
{
	return 0;
}

static int ssp_predisable(struct iio_dev *indio_dev)
{
	return 0;
}

static const struct iio_buffer_setup_ops ssp_iio_ring_setup_ops = {
	.preenable = &ssp_preenable,
	.predisable = &ssp_predisable,
};

struct iio_kfifo {
	struct iio_buffer buffer;
	struct kfifo kf;
	struct mutex user_lock;
	int update_needed;
};

#define iio_to_kfifo(r) container_of(r, struct iio_kfifo, buffer)

static inline int __iio_allocate_kfifo(struct iio_kfifo *buf,
			size_t bytes_per_datum, unsigned int length)
{
	if ((length == 0) || (bytes_per_datum == 0))
		return -EINVAL;

	/*
	 * Make sure we don't overflow an unsigned int after kfifo rounds up to
	 * the next power of 2.
	 */
	if (roundup_pow_of_two(length) > (UINT_MAX / bytes_per_datum))
		return -EINVAL;

	return __kfifo_alloc((struct __kfifo *)&buf->kf, length,
			     bytes_per_datum, GFP_KERNEL);
}

static int iio_request_update_kfifo(struct iio_buffer *r)
{
	int ret = 0;
	struct iio_kfifo *buf = iio_to_kfifo(r);

	mutex_lock(&buf->user_lock);
	if (buf->update_needed) {
		kfifo_free(&buf->kf);
		ret = __iio_allocate_kfifo(buf, buf->buffer.bytes_per_datum,
				   buf->buffer.length);
		if (ret >= 0)
			buf->update_needed = false;
	} else {
		kfifo_reset_out(&buf->kf);
	}
	mutex_unlock(&buf->user_lock);

	return ret;
}

static int iio_mark_update_needed_kfifo(struct iio_buffer *r)
{
	struct iio_kfifo *kf = iio_to_kfifo(r);
	kf->update_needed = true;
	return 0;
}

static int iio_set_bytes_per_datum_kfifo(struct iio_buffer *r, size_t bpd)
{
	/*
		pr_info("[SSP-IIO]: %s, r->bytes_per_datum=%d bpd=%d",__func__, r->bytes_per_datum,bpd);
	

	if (r->bytes_per_datum != bpd) {
		r->bytes_per_datum = bpd;
		iio_mark_update_needed_kfifo(r);
	}

	*/
	
	return 0;
}

static int iio_set_length_kfifo(struct iio_buffer *r, unsigned int length)
{
	/* Avoid an invalid state */
	if (length < 2)
		length = 2;
	if (r->length != length) {
		r->length = length;
		iio_mark_update_needed_kfifo(r);
	}
	return 0;
}

static int iio_store_to_kfifo(struct iio_buffer *r,
			      const void *data)
{
	int ret;
	struct iio_kfifo *kf = iio_to_kfifo(r);
	
	ret = kfifo_in(&kf->kf, data, 1);
	if (ret != 1)
		return -EBUSY;
	
	return 0;
}

static int iio_read_kfifo(struct iio_buffer *r,
			   size_t n, char __user *buf)
{
	int ret, copied;
	struct iio_kfifo *kf = iio_to_kfifo(r);

	if (mutex_lock_interruptible(&kf->user_lock))
		return -ERESTARTSYS;

	if (!kfifo_initialized(&kf->kf) || n < kfifo_esize(&kf->kf))
		ret = -EINVAL;
	else
		ret = kfifo_to_user(&kf->kf, buf, n, &copied);
	
	//pr_err("[SSP_IIO] size=%d, kfifo_init=%d, kfifo_esize=%d", n, kfifo_initialized(&kf->kf), kfifo_esize(&kf->kf));
	
	mutex_unlock(&kf->user_lock);
	if (ret < 0)
		return ret;

	return copied;
}

static size_t iio_kfifo_buf_data_available(struct iio_buffer *r)
{
	struct iio_kfifo *kf = iio_to_kfifo(r);
	size_t samples;

	mutex_lock(&kf->user_lock);
	samples = kfifo_len(&kf->kf);
	mutex_unlock(&kf->user_lock);

	return samples;
}

static void iio_kfifo_buffer_release(struct iio_buffer *buffer)
{
	struct iio_kfifo *kf = iio_to_kfifo(buffer);

	mutex_destroy(&kf->user_lock);
	kfifo_free(&kf->kf);
	kfree(kf);
}

static const struct iio_buffer_access_funcs kfifo_access_funcs = {
	.store_to = &iio_store_to_kfifo,
	.read = &iio_read_kfifo,
	.data_available = iio_kfifo_buf_data_available,
	.request_update = &iio_request_update_kfifo,
	.set_bytes_per_datum = &iio_set_bytes_per_datum_kfifo,
	.set_length = &iio_set_length_kfifo,
	.release = &iio_kfifo_buffer_release,

	.modes = INDIO_BUFFER_SOFTWARE | INDIO_BUFFER_TRIGGERED,
};


static struct iio_buffer * ssp_iio_kfifo_allocate(void)
{
    struct iio_kfifo *kf;

    kf = kzalloc(sizeof(*kf), GFP_KERNEL);
    if (!kf)
        return NULL;

    kf->update_needed = true;
    iio_buffer_init(&kf->buffer);
    kf->buffer.access = &kfifo_access_funcs;
    kf->buffer.length = 2;
    mutex_init(&kf->user_lock);

    return &kf->buffer;
}

static void ssp_iio_kfifo_free(struct iio_buffer *r)
{
    iio_buffer_put(r);
}


static int ssp_iio_configure_ring(struct iio_dev *indio_dev, const int byte_size)
{
	struct iio_buffer *ring;

	ring = ssp_iio_kfifo_allocate();
	if (!ring) {
		return -ENOMEM;
	}

	ring->scan_timestamp = true;
	ring->bytes_per_datum = byte_size;

	ring->scan_mask = bitmap_zalloc(1, GFP_KERNEL);
	set_bit(0, ring->scan_mask);
	iio_device_attach_buffer(indio_dev, ring);

	indio_dev->setup_ops = &ssp_iio_ring_setup_ops;
	indio_dev->modes |= INDIO_BUFFER_SOFTWARE;

	return 0;
}

static void ssp_iio_push_buffers(struct iio_dev *indio_dev, u64 timestamp,
					char *data, int data_len)
{
	int buf_len = data_len + sizeof(timestamp);
	u8 *buf = NULL;

	if (!indio_dev || !data) {
		return;
	}
	buf = kzalloc(buf_len, GFP_KERNEL);

	memcpy(buf, data, data_len);
	memcpy(buf + data_len, &timestamp, sizeof(timestamp));
	mutex_lock(&indio_dev->mlock);
	iio_push_to_buffers(indio_dev, buf);
	mutex_unlock(&indio_dev->mlock);

	kfree(buf);
}

#ifdef CONFIG_SENSORS_SSP_PROXIMITY
static void report_prox_raw_data(struct ssp_data *data, int type,
					struct sensor_value *proxrawdata)
{
	if (data->prox_raw_avg[PROX_RAW_NUM]++ >= PROX_AVG_READ_NUM) {
		data->prox_raw_avg[PROX_RAW_SUM] /= PROX_AVG_READ_NUM;
		data->buf[type].prox_raw[1] = (u16)data->prox_raw_avg[1];
		data->buf[type].prox_raw[2] = (u16)data->prox_raw_avg[2];
		data->buf[type].prox_raw[3] = (u16)data->prox_raw_avg[3];

		data->prox_raw_avg[PROX_RAW_NUM] = 0;
		data->prox_raw_avg[PROX_RAW_MIN] = 0;
		data->prox_raw_avg[PROX_RAW_SUM] = 0;
		data->prox_raw_avg[PROX_RAW_MAX] = 0;
	} else {
		data->prox_raw_avg[PROX_RAW_SUM] += proxrawdata->prox_raw[0];

		if (data->prox_raw_avg[PROX_RAW_NUM] == 1) {
			data->prox_raw_avg[PROX_RAW_MIN] = proxrawdata->prox_raw[0];
		} else if (proxrawdata->prox_raw[0] < data->prox_raw_avg[PROX_RAW_MIN]) {
			data->prox_raw_avg[PROX_RAW_MIN] = proxrawdata->prox_raw[0];
		}

		if (proxrawdata->prox_raw[0] > data->prox_raw_avg[PROX_RAW_MAX]) {
			data->prox_raw_avg[PROX_RAW_MAX] = proxrawdata->prox_raw[0];
		}
	}

	data->buf[type].prox_raw[0] = proxrawdata->prox_raw[0];
}

static void report_prox_cal_data(struct ssp_data *data, int type,
					struct sensor_value *p_cal_data)
{
	data->prox_thresh[0] = p_cal_data->prox_cal[0];
	data->prox_thresh[1] = p_cal_data->prox_cal[1];
	ssp_info("prox thresh %u %u", data->prox_thresh[0], data->prox_thresh[1]);

	proximity_calibration_off(data);
}
#endif

#ifdef CONFIG_SENSORS_SSP_LIGHT_SENSOR
#define OPT3007_REG_EXPONENT(n)		((n) >> 12)
#define OPT3007_REG_MANTISSA(n)		((n) & 0xfff)
#define SWAP16(s) (((s) & 0xFF) << 8 | (((s) >> 8) & 0xff))
static void report_light_data_iio(struct ssp_data *data, int type,
					struct sensor_value *lightdata)
{
#if defined(CONFIG_SENSORS_SSP_LUCKY)
	u16 mantissa;
	u8 exponent;
	u16 swap_data;

	swap_data = SWAP16(lightdata->rdata);

	exponent = (u8)(OPT3007_REG_EXPONENT(swap_data));
	mantissa = (u16)(OPT3007_REG_MANTISSA(swap_data));

	data->buf[LIGHT_SENSOR].rdata = swap_data;
	data->buf[LIGHT_SENSOR].gain = lightdata->gain;
	data->lux = light_get_lux(data, mantissa, exponent);
#else
	data->lux = lightdata->rdata; 
#endif
}
#endif

#ifdef CONFIG_SENSORS_SSP_PRESSURE_SENSOR
static void report_pressure_data_iio(struct ssp_data *data, int type, struct sensor_value *predata)
{
#ifndef CONFIG_SENSORS_SSP_LUCKY
#if IS_ENABLED(CONFIG_SEC_FACTORY)
	data->buf[PRESSURE_SENSOR].pressure[0] =
		predata->pressure[0] - data->iPressureCal;
#else
	data->buf[PRESSURE_SENSOR].pressure[0] =
		predata->pressure[0] - data->iPressureCal - (data->sw_offset * data->convert_coef);
#endif
#else
	data->buf[PRESSURE_SENSOR].pressure[0] =
		predata->pressure[0] - data->iPressureCal;
#endif
	data->buf[PRESSURE_SENSOR].pressure[1] = predata->pressure[1];
	data->buf[PRESSURE_SENSOR].pressure[2] = data->sealevelpressure;
}
#endif

void report_sensor_data(struct ssp_data *data, int type,
			struct sensor_value *event)
{
	int i;

	if (!(data->info[type].enable)) {
		//pr_info("[SSP] Unexpected sensor type: %d\n", type);
		return;
	}

	if (type == ACCELEROMETER_SENSOR || type == GYROSCOPE_SENSOR || type == ACCELEROMETER_SENSOR_32G) {
		memcpy(&data->buf[type], (char *)event, data->info[type].report_data_len);
	} else if (type == PRESSURE_SENSOR) {
		report_pressure_data_iio(data, type, event);
	} else if (type == LIGHT_SENSOR) {
		report_light_data_iio(data, type, event);
		ssp_iio_push_buffers(data->indio_devs[type], event->timestamp,
			(char *)&data->lux, data->info[type].report_data_len);
		return;
	} else if (type == HRM_LIB_SENSOR) {
		memcpy(&data->buf[type], (char *)event, data->info[type].report_data_len);
		// pr_info("[SSP] hvalue %d %d %d\n", data->buf[type].hr, data->buf[type].rri, data->buf[type].snr);
	} else if (type == HRM_ECG_LIB_SENSOR) {
		for (i = 0 ; i < HRM_ECG_LIB_DATA_SIZE ; i++) {
			data->buf[HRM_ECG_LIB_SENSOR].ecg_data_value[i] = event->ecg_data_value[i];
		}
	} else {
		memcpy(&data->buf[type], (char *)event, data->info[type].report_data_len);
	}

	ssp_iio_push_buffers(data->indio_devs[type], event->timestamp,
			     (char *)&data->buf[type], data->info[type].report_data_len);

	print_report_debug(data, event, type);
}

void report_meta_data(struct ssp_data *data, struct sensor_value *s)
{
	char *meta_event = kzalloc(data->info[s->meta_data.sensor].report_data_len, GFP_KERNEL);

	ssp_infof("what: %d, sensor: %d", s->meta_data.what, s->meta_data.sensor);

	memset(meta_event, META_EVENT,
	       data->info[s->meta_data.sensor].report_data_len);
	ssp_iio_push_buffers(data->indio_devs[s->meta_data.sensor],
			     META_TIMESTAMP, meta_event,
			     data->info[s->meta_data.sensor].report_data_len);
	kfree(meta_event);
}

void report_scontext_data(struct ssp_data *data, char *data_buf, u32 length)
{
	char buf[SCONTEXT_HEADER_LEN + SCONTEXT_DATA_LEN] = {0, };
	u16 start, end;
	u64 timestamp;
	static int sample_count;
	unsigned int delayms = 300;
	bool isLibData = ((data_buf[0] == 0x01) && (data_buf[1] == 0x01)) ? true : false;

	//pr_info("[SSP] %s, isLibData (%d)\n", __func__, isLibData);

	if(length > MAX_DATA_LEN) {
		ssp_errf("invalid length(%d)", length);
		ssp_scontext_log(__func__, data_buf, length);
		return;
	}

	mutex_lock(&(data->scontext_mutex));

	/* For the following library data, output the log once per 100 samples.
	 * BLOODPRESSURE(25), SPO2(41)
	 */
	if ((length > 2) && isLibData && (data_buf[2] == SH_LIBRARY_BLOODPRESSURE || data_buf[2] == SH_LIBRARY_SPO2)) {
		sample_count++;
		if (sample_count >= 100) {
			ssp_scontext_log(__func__, data_buf, length);
			sample_count = 0;
		}
	} else {
		ssp_scontext_log(__func__, data_buf, length);
		if (data_buf[2] == SH_LIBRARY_SLEEP_DETECT) {
			if (data->bWakeupByMe) {
				print_current_time("LIBRARY_SLEEP_DETECT(S)", false);
				data->bSavePostSuspendTime = true;
				data->bIsSleep = true;
			} else {
				print_current_time("LIBRARY_SLEEP_DETECT(W)", true);
			}
			delayms = 600;
		}
		if (isLibData && data->bWakeupByMe && data_buf[2] < SH_LIBRARY_MAX) {
			data->msgBeforWakeup[(u8)data_buf[2]] += 1;
			data->msgBeforWakeupCnt++;
			if(data->msgBeforWakeup[(u8)data_buf[2]] >= 0xFFFF) {
				data->msgBeforWakeup[(u8)data_buf[2]] = 0;
			}
		}
	}

	start = 0;
	memcpy(buf, &length, SCONTEXT_LENGTH_LEN);
	timestamp = get_current_timestamp();

	if (length > 2 && data_buf[2] == SH_LIBRARY_WRIST_UP && data->uLastAPState == MSG2SSP_AP_STATUS_LCD_OFF) {
		wristup_booster_turn_on(data);
		ssp_wake_lock_timeout(data->ssp_wake_lock, 75); // 300 ms
		pr_info("[SSP] %s, boosting CPU (%d)\n", __func__, data->wristup_boost_initialized);
	}

	ssp_wake_lock_timeout(data->ssp_wake_lock, msecs_to_jiffies(delayms));

	while (start < length) {
		if (start + SCONTEXT_DATA_LEN < length) {
			end = start + SCONTEXT_DATA_LEN - 1;
		} else {
			memset(buf + SCONTEXT_HEADER_LEN, 0, SCONTEXT_DATA_LEN);
			end = length - 1;
		}

		memcpy(buf + SCONTEXT_LENGTH_LEN, &start, SCONTEXT_START_LEN);
		memcpy(buf + SCONTEXT_LENGTH_LEN + SCONTEXT_START_LEN, &end, SCONTEXT_END_LEN);
		memcpy(buf + SCONTEXT_HEADER_LEN, data_buf + start, end - start + 1);

		ssp_iio_push_buffers(data->indio_devs[SENSOR_TYPE_SCONTEXT_T], timestamp,
				     buf, data->info[SENSOR_TYPE_SCONTEXT_T].report_data_len);

		start = end + 1;
	}

	mutex_unlock(&(data->scontext_mutex));

	if (data->bWakeupByMe && isLibData) {
		report_wakeup_info(data, data_buf[2]);
	}
	if (isLibData && data_buf[2] > SH_LIBRARY_MAX + 10) {
		ssp_errf("invalid library(%d : %d)", data_buf[2], length);
		report_wakeup_info_err(data, MSG2AP_INST_RESERVED2, data_buf[2]);
	}
}

#define BIG_DATA_OFFSET 	(4)
void report_scontext_big_data(struct ssp_data *data, char *data_buf, u32 length)
{
	char buf[SCONTEXT_HEADER_LEN + SCONTEXT_DATA_LEN] = {0, };
	u16 start = 0, end = 0;
	u16 start_pos = 0, end_pos = 0;
	u64 timestamp;
	int offset = BIG_DATA_OFFSET;
	int total_length = length + offset;

	if(length > MAX_DATA_LEN) {
		ssp_errf("invalid length(%d)", length);
		ssp_scontext_log(__func__, data_buf, length);
		return;
	}

	mutex_lock(&(data->scontext_mutex));

	buf[SCONTEXT_HEADER_LEN] = 0x1;
	buf[SCONTEXT_HEADER_LEN+1] = 0x1;
	buf[SCONTEXT_HEADER_LEN+2] = 0x5;
	buf[SCONTEXT_HEADER_LEN+3] = 0x1; // event_type : sensor = 1

	ssp_scontext_log(__func__, data_buf, length);

	memcpy(buf, &total_length, SCONTEXT_LENGTH_LEN);
	timestamp = get_current_timestamp();

	//ssp_wake_lock_timeout(data->ssp_wake_lock, msecs_to_jiffies(300));

	while (start < length) {
		if (start + SCONTEXT_DATA_LEN - offset < length) {
			end = start + SCONTEXT_DATA_LEN - offset - 1;
		} else {
			memset(buf + SCONTEXT_HEADER_LEN + offset, 0, SCONTEXT_DATA_LEN - offset);
			end = length - 1;
		}
		end_pos = end + BIG_DATA_OFFSET;

		memcpy(buf + SCONTEXT_LENGTH_LEN, &start_pos, SCONTEXT_START_LEN);
		memcpy(buf + SCONTEXT_LENGTH_LEN + SCONTEXT_START_LEN, &end_pos, SCONTEXT_END_LEN);
		memcpy(buf + SCONTEXT_HEADER_LEN + offset, data_buf + start, end - start + 1);

		//ssp_errf("serene %u %u %u %u\n", start, end, start_pos, end_pos);
		//ssp_scontext_log(__func__, buf, SCONTEXT_HEADER_LEN+ offset + end - start + 1);

		ssp_iio_push_buffers(data->indio_devs[SENSOR_TYPE_SCONTEXT_T], timestamp,
				     buf, data->info[SENSOR_TYPE_SCONTEXT_T].report_data_len);

		start = end + 1;
		start_pos = end_pos + 1;
		offset = 0;
	}

	mutex_unlock(&(data->scontext_mutex));
}

void report_scontext_notice_data(struct ssp_data *data, char notice)
{
	char notice_buf[4] = {0x02, 0x01, 0x00, 0x00};
	int len = 3;

	notice_buf[2] = notice;
	if (notice == MSG2SSP_AP_STATUS_RESET) {
		len = 4;
		notice_buf[3] = convert_reset_type_to_reason(data->reset_type);
	}

	report_scontext_data(data, notice_buf, len);

	if (notice == MSG2SSP_AP_STATUS_RESET) {
		ssp_infof("reset (%d, %d)", data->reset_type, notice_buf[3]);
	} else {
		ssp_errf("invalid notice(0x%x)", notice);
	}
}

void report_sensorhub_data(struct ssp_data *data, char *buf)
{
	ssp_infof();
	ssp_iio_push_buffers(data->indio_devs[SENSOR_TYPE_SCONTEXT_T], get_current_timestamp(),
							buf, data->info[SENSOR_TYPE_SCONTEXT_T].report_data_len);
}

int file_manager_read(struct ssp_data *data, char *filename, char *buf, int size, long long pos)
{
	int ret = 0;
	char cmd[9] = {0,};

	mutex_lock(&data->indio_file_manager_dev->mlock);

	memset(data->fm_rx_buffer, 0, sizeof(data->fm_rx_buffer));
	memset(data->fm_tx_buffer, 0, sizeof(data->fm_tx_buffer));

	data->fm_read_done.done = 0;

	data->fm_tx_buffer_length = snprintf(data->fm_tx_buffer, sizeof(data->fm_tx_buffer), "%s,%d,%lld,0", filename, size, pos);
	pr_err("[SSP] file_manager_read: %s(%s)",data->fm_tx_buffer,filename);

	ret = iio_push_to_buffers(data->indio_file_manager_dev, &cmd);
	if(ret < 0) {
		pr_err("[SSP] iio_push_fail(%d)",ret);
	} else {
		ret = wait_for_completion_timeout(&data->fm_read_done, 0.1*HZ);
		if (ret == 0) {
			pr_err("[SSP] file_manager_read, completion timeout");
		}
	}

	memcpy(buf, data->fm_rx_buffer, size);
	mutex_unlock(&data->indio_file_manager_dev->mlock);
	return size;
}

static char output[1024 * 4] = {0,};
static int output_size = 0;

int file_manager_write(struct ssp_data *data, char *filename, char *buf, int size, long long pos)
{
	int ret = 0;
	char cmd[9] = {1,};
	int i = 0;


	mutex_lock(&data->indio_file_manager_dev->mlock);
	
	memset(output, 0, sizeof(output));
	output_size = 0;

	memset(data->fm_tx_buffer, 0, sizeof(data->fm_tx_buffer));
	data->fm_write_done.done = 0;
	data->fm_tx_buffer_length = snprintf(data->fm_tx_buffer, sizeof(data->fm_tx_buffer), "%s,%d,%lld,", filename, size, pos);
	memcpy(data->fm_tx_buffer + data->fm_tx_buffer_length, buf, size);

	for(i = 0; i < size; i++)
		output_size += sprintf(output + output_size, "0x%x ", buf[i]);

	pr_err("[SSP] file_manager_write: %s(%s)",data->fm_tx_buffer,filename);
	pr_err("[SSP] file_manager_write: %s",output);

	data->fm_tx_buffer_length += size;

	ret = iio_push_to_buffers(data->indio_file_manager_dev, &cmd);
	if (ret < 0) {
		pr_err("[SSP] iio_push_fail(%d)", ret);
	} else {
		ret = wait_for_completion_timeout(&data->fm_write_done, 0.1*HZ);
		if (ret == 0) {
			pr_err("[SSP] file_manager_write, completion timeout");
		}
	}

	mutex_unlock(&data->indio_file_manager_dev->mlock);
	return size;
}

static void *init_indio_device(struct device *dev, struct ssp_data *data,
				const struct iio_info *info,
				const struct iio_chan_spec *channels,
				const char *device_name, const int byte_size)
{
	struct iio_dev *indio_dev;
	int ret = 0;

	indio_dev = iio_device_alloc(dev, sizeof(dev));
	if (!indio_dev) {
		goto err_alloc;
	}

	indio_dev->name = device_name;
	indio_dev->dev.parent = dev;
	indio_dev->info = info;
	indio_dev->channels = channels;
	indio_dev->num_channels = 1;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;

	ret = ssp_iio_configure_ring(indio_dev, byte_size);
	if (ret) {
		goto err_config_ring;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		goto err_register_device;
	}

	return indio_dev;

err_register_device:
	ssp_err("fail to register %s device", device_name);
	ssp_iio_kfifo_free(indio_dev->buffer);
err_config_ring:
	ssp_err("failed to configure %s buffer\n", indio_dev->name);
	iio_device_unregister(indio_dev);
err_alloc:
	ssp_err("fail to allocate memory for iio %s device", device_name);
	return NULL;
}

static void init_iio_channel(struct iio_chan_spec *chan_spec, int realbits_size)
{
	int repeat_size = 1;
	int	data_size = 0;

	repeat_size = realbits_size / 256 + 1;
	data_size = realbits_size / repeat_size + (realbits_size - (realbits_size / repeat_size) * repeat_size);

	chan_spec->type = IIO_TIMESTAMP;
	chan_spec->channel = IIO_CHANNEL;
	chan_spec->scan_index = IIO_SCAN_INDEX;
	chan_spec->scan_type.sign = IIO_SIGN;
	chan_spec->scan_type.realbits = data_size;
	chan_spec->scan_type.storagebits = data_size;
	chan_spec->scan_type.shift = IIO_SHIFT;
	chan_spec->scan_type.repeat = repeat_size;
}


static const struct iio_info indio_info;

static const struct iio_chan_spec indio_file_manager_channels[] = {
	{
		.type = IIO_TIMESTAMP,
		.channel = IIO_CHANNEL,
		.scan_index = 0,
		.scan_type = IIO_ST('s', 9 * 8, 9 * 8, 0)
	}
};

int initialize_indio_dev(struct device *dev, struct ssp_data *data)
{
	int type;
	int realbits_size = 0;

	/* file manager iio */
	data->indio_file_manager_dev 
			= (struct iio_dev *)init_indio_device(dev, data,
							      &indio_info, indio_file_manager_channels,
							      "file_manager", 9);

	if (!data->indio_file_manager_dev) {
		ssp_err("fail to init file manager iio dev");
		return ERROR;
	}

	/* sensor iio */
	for (type = 0; type <= SENSOR_TYPE_SCONTEXT_T; type++) {
		if (!data->info[type].enable || (data->info[type].report_data_len == 0)) {
			continue;
		}

		realbits_size = (data->info[type].report_data_len+sizeof(data->buf[type].timestamp)) * BITS_PER_BYTE;
		init_iio_channel(&(data->indio_channels[type]), realbits_size);

		data->indio_devs[type]
			= (struct iio_dev *)init_indio_device(dev, data,
							      &indio_info, &data->indio_channels[type],
							      data->info[type].name, realbits_size / BITS_PER_BYTE);

		if (!data->indio_devs[type]) {
			ssp_err("fail to init %s iio dev", data->info[type].name);
			remove_indio_dev(data);
			return ERROR;
		}
	}

	return SUCCESS;
}

void remove_indio_dev(struct ssp_data *data)
{
	int type;

	for (type = SENSOR_TYPE_MAX - 1; type >= 0; type--) {
		if (data->indio_devs[type]) {
			iio_device_unregister(data->indio_devs[type]);
		}
	}
}

