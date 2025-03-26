/*
 *  sb_cisd.c
 *  Samsung Mobile Battery CISD
 *  Cell Internal Short Detection
 *
 *  Copyright (C) 2022 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/list.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/module.h>

#include <acpi/platform/acgcc.h>

#include <linux/battery/sb_def.h>
#include <linux/battery/sb_sysfs.h>
#include <linux/battery/common/sb_ext.h>
#include <linux/battery/module/sb_cisd.h>

#define CISD_VER			9
#define MAX_PAD_ID			0xFF

#define cisd_log(str, ...) pr_info("[sb-cisd]:%s: "str, __func__, ##__VA_ARGS__)

static const char *sb_cisd_name[] = FOREACH_CISD(GEN_CISD_STRING);

union raw_data {
	sb_data raw;

	struct {
		unsigned set : 1,
			resv : 31;
		signed value;
	} base;

	/* external data */
	struct {
		unsigned int id;
		unsigned int count;
	} pad;
};

struct ext_data {
	struct list_head list;
	union raw_data data;
};

struct ext_data_list {
	struct list_head list;
	unsigned int count;
};

struct sb_cisd {
	/* state */
	int now_state;
	bool is_update;

	/* lock */
	struct mutex lock;

	/* cisd base data buf & list */
	union raw_data buf[CISD_MAX];

	/* ext list */
	struct ext_data_list pad_list;

	/* dt */
};
static struct sb_cisd *g_cisd;

/*
 * base cisd data
 */
static bool check_valid_type(enum CISD_ENUM type)
{
	return ((type >= 0) && (type < CISD_MAX));
}

static void set_data(struct sb_cisd *cisd, enum CISD_ENUM type, int value)
{
	union raw_data *raw = &cisd->buf[type];

	raw->base.value = value;
	raw->base.set = 1;

	cisd->is_update = true;
}

static int get_data(struct sb_cisd *cisd, enum CISD_ENUM type, int *value)
{
	union raw_data *raw = &cisd->buf[type];

	if (raw->base.set) {
		*value = raw->base.value;
		return 0;
	}

	return -EINVAL;
}

static void count_data(struct sb_cisd *cisd, enum CISD_ENUM type)
{
	union raw_data *raw = &cisd->buf[type];

	raw->base.value += 1;
	raw->base.set = 1;

	cisd->is_update = true;
}

static void init_data(struct sb_cisd *cisd, enum CISD_ENUM start, enum CISD_ENUM end)
{
	int i = start;

	for (; i < end; i++)
		cisd->buf[i].raw = 0;
}

/*
 * ext cisd data
 */
static struct ext_data *create_ext_data(sb_data raw)
{
	struct ext_data *data = NULL;

	data = kzalloc(sizeof(struct ext_data), GFP_KERNEL);
	if (!data)
		return NULL;

	data->data.raw = raw;
	INIT_LIST_HEAD(&data->list);

	cisd_log("raw = %lld(0x%llx)\n", raw, raw);
	return data;
}

static void init_ext_list(struct ext_data_list *ext_list)
{
	INIT_LIST_HEAD(&ext_list->list);
	ext_list->count = 0;
}

static void clear_ext_list(struct ext_data_list *ext_list)
{
	struct ext_data *data = NULL;

	while (!list_empty(&ext_list->list)) {
		data = list_first_entry(&ext_list->list, struct ext_data, list);
		list_del(&data->list);
		kfree(data);
	}

	init_ext_list(ext_list);
}

static int parse_dt(struct sb_cisd *cisd)
{
	return 0;
}

static void init(struct sb_cisd *cisd)
{
	/* init base data */
	init_data(cisd, RESET_ALG, CISD_MAX);

	/* set cisd version */
	set_data(cisd, ALG_INDEX, CISD_VER);

	/* init ext data */
	clear_ext_list(&cisd->pad_list);

	/* init state */
	cisd->now_state = 0;
	cisd->is_update = false;
}

static ssize_t show_cisd_data(struct sb_cisd *cisd, char *buf, unsigned int size, enum CISD_ENUM start, enum CISD_ENUM end)
{
	unsigned int len = 0, buf_size = 0;

	if (cisd->is_update)
		cisd->now_state = !cisd->now_state;
	cisd->is_update = false;

	/* write state */
	len = snprintf(buf, size, "%d ", cisd->now_state);

	/* write buffer */
	buf_size = sizeof(union raw_data) * CISD_MAX;
	memcpy(buf + len, &cisd->buf, buf_size);

	return len + buf_size;
}

static ssize_t show_cisd_data_json(struct sb_cisd *cisd, char *buf, unsigned int size, enum CISD_ENUM start, enum CISD_ENUM end)
{
	unsigned int len = 0, i = start;
	union raw_data *temp;

	for (; i < end; i++) {
		temp = &cisd->buf[i];
		if (temp->base.set)
			len += snprintf(buf + len, size - len, "\"%s\":\"%d\",", sb_cisd_name[i], temp->base.value);
	}

	if (len <= 0)
		return 0;

	len += snprintf(buf + len - 1, size - len + 1, "\n");
	return len;
}

static void store_cisd_data(struct sb_cisd *cisd, const char *buf, size_t size)
{
	unsigned int buf_size = 0;
	int x = 0, temp = 0;

	if (sscanf(buf, "%d %n", &temp, &x) <= 0)
		return;

	/* init */
	init(cisd);

	/* reset state */
	if ((temp < 0) || sb_get_fg_reset())
		return;

	cisd->now_state = temp;

	/* check size */
	buf_size = sizeof(union raw_data) * CISD_MAX;
	if (size - x != buf_size) {
		cisd_log("invalid size (%ld, %d, %d)\n", size, x, buf_size);
		return;
	}

	memcpy(&cisd->buf, buf + x, buf_size);
}

static int show_pad_data(void *buf, unsigned int size, sb_data data)
{
	struct ext_data *temp = (struct ext_data *)data;

	return snprintf(buf, size, " 0x%02x:%d", temp->data.pad.id, temp->data.pad.count);
}

static int show_pad_data_json(void *buf, unsigned int size, sb_data data)
{
	struct ext_data *temp = (struct ext_data *)data;

	return snprintf(buf, size, ",\"PAD_0x%02x\":\"%d\"", temp->data.pad.id, temp->data.pad.count);
}

static ssize_t show_ext_data(struct ext_data_list *ext_list, sb_event_func print_func, char *buf, unsigned int size, const char *fmt)
{
	struct ext_data *data = NULL;
	unsigned int len = 0;

	len = snprintf(buf, size, fmt, ext_list->count);

	list_for_each_entry(data, &ext_list->list, list) {
		len += print_func(buf + len, size - len, (sb_data)data);
	}

	len += snprintf(buf + len, size - len, "\n");
	return len;
}

static bool cmp_pad_data(sb_data data1, sb_data data2)
{
	union raw_data r1 = { data1, },
		r2 = { data2,};

	return (r1.pad.id == r2.pad.id);
}

static struct ext_data *find_ext_data(struct ext_data_list *ext_list, sb_cmp_func cmp_func, sb_data value)
{
	struct ext_data *data = NULL;

	list_for_each_entry(data, &ext_list->list, list) {
		if (cmp_func(data->data.raw, value))
			return data;
	}

	return NULL;
}

static int store_pad_data(void *buf, sb_data data)
{
	union raw_data *pad_data = (union raw_data *)data;
	int x = 0, ret = 0;

	ret = sscanf(buf, "0x%02x:%10u %n", &pad_data->pad.id, &pad_data->pad.count, &x);
	return (ret == 2) ? x : -EINVAL;
}

static void store_ext_data(struct ext_data_list *ext_list, unsigned int max_count, sb_func scanf_func, sb_cmp_func cmd_func, const char *buf)
{
	struct ext_data *data = NULL;
	union raw_data temp = { 0, };
	unsigned int total_cnt = 0;
	int i = 0, x, err_cnt = 0;

	/* init list */
	clear_ext_list(ext_list);

	if (sscanf(buf, "%10u %n", &total_cnt, &x) <= 0)
		return;
	buf += (size_t)x;

	if ((total_cnt <= 0) || (total_cnt >= max_count))
		return;

	for (; i < total_cnt; i++) {
		temp.raw = 0;

		x = scanf_func((void *)buf, (sb_data)&temp);
		if (x < 0) {
			clear_ext_list(ext_list);
			return;
		}
		buf += (size_t)x;

		data = find_ext_data(ext_list, cmd_func, temp.raw);
		if (!data) {
			data = create_ext_data(temp.raw);
			if (!data) {
				err_cnt++;
				continue;
			}

			list_add(&data->list, &ext_list->list);
		} else {
			data->data.raw = temp.raw;
		}
	}

	ext_list->count = total_cnt - err_cnt;
}

static ssize_t show_attrs(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t store_attrs(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

#define CISD_ATTR(_name)	\
{	\
	.attr = {.name = #_name, .mode = 0664},	\
	.show = show_attrs,	\
	.store = store_attrs,	\
}

static struct device_attribute sysfs_cisd_attrs[] = {
	CISD_ATTR(cisd_data),
	CISD_ATTR(cisd_data_json),
	CISD_ATTR(cisd_data_d_json),
	CISD_ATTR(cisd_wire_count),
	CISD_ATTR(cisd_wc_data),
	CISD_ATTR(cisd_wc_data_json),
};

enum {
	CISD_DATA = 0,
	CISD_DATA_JSON,
	CISD_DATA_D_JSON,
	CISD_WIRE_COUNT,
	CISD_WC_DATA,
	CISD_WC_DATA_JSON,
};

static ssize_t show_attrs(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	const ptrdiff_t offset = attr - sysfs_cisd_attrs;
	int i = 0;

	if (g_cisd == NULL)
		return -EINVAL;

	mutex_lock(&g_cisd->lock);

	switch (offset) {
	case CISD_DATA:
		i = show_cisd_data(g_cisd, buf, PAGE_SIZE, RESET_ALG, CISD_MAX);
		break;
	case CISD_DATA_JSON:
		i = show_cisd_data_json(g_cisd, buf, PAGE_SIZE, RESET_ALG, FULL_CNT_D);
		break;
	case CISD_DATA_D_JSON:
		i = show_cisd_data_json(g_cisd, buf, PAGE_SIZE, FULL_CNT_D, CISD_MAX);

		init_data(g_cisd, FULL_CNT_D, CISD_MAX);
		break;
	case CISD_WIRE_COUNT:
	{
		unsigned int data = 0;

		i += snprintf(buf, PAGE_SIZE, "%d\n", ((get_data(g_cisd, WIRE_CNT, &data) < 0) ? 0 : data));
	}
		break;
	case CISD_WC_DATA:
		i += show_ext_data(&g_cisd->pad_list, show_pad_data, buf, PAGE_SIZE, "%d");
		break;
	case CISD_WC_DATA_JSON:
		i += show_ext_data(&g_cisd->pad_list, show_pad_data_json, buf, PAGE_SIZE, "\"INDEX\":\"%d\"");
		break;
	default:
		mutex_unlock(&g_cisd->lock);
		return -EINVAL;
	}

	mutex_unlock(&g_cisd->lock);

	return i;
}

static ssize_t store_attrs(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	const ptrdiff_t offset = attr - sysfs_cisd_attrs;

	if (g_cisd == NULL)
		return -EINVAL;

	mutex_lock(&g_cisd->lock);

	switch (offset) {
	case CISD_DATA:
		cisd_log("CISD_DATA\n");
		store_cisd_data(g_cisd, buf, count);
		break;
	case CISD_DATA_JSON:
	{
		char cmd = 0;

		cisd_log("CISD_DATA_JSON\n");
		if (sscanf(buf, "%1c\n", &cmd) == 1) {
			switch (cmd) {
			case 'c':
				init(g_cisd);
				break;
			default:
				break;
			}
		}
	}
		break;
	case CISD_DATA_D_JSON:
		break;
	case CISD_WIRE_COUNT:
	{
		unsigned int x = 0;

		cisd_log("CISD_WIRE_COUNT\n");
		if (sscanf(buf, "%10d\n", &x) == 1) {
			set_data(g_cisd, WIRE_CNT, x);
			count_data(g_cisd, WIRE_CNT_D);
		}
	}
		break;
	case CISD_WC_DATA:
		cisd_log("CISD_WC_DATA\n");
		store_ext_data(&g_cisd->pad_list, MAX_PAD_ID, store_pad_data, cmp_pad_data, buf);
		break;
	case CISD_WC_DATA_JSON:
		break;
	default:
		mutex_unlock(&g_cisd->lock);
		return -EINVAL;
	}

	mutex_unlock(&g_cisd->lock);
	return count;
}

int sb_cisd_set_data(enum CISD_ENUM type, int value)
{
	if (g_cisd == NULL)
		return -ENODEV;
	if (!check_valid_type(type))
		return -EINVAL;

	cisd_log("%s - %d\n", sb_cisd_name[type], value);
	mutex_lock(&g_cisd->lock);
	set_data(g_cisd, type, value);
	mutex_unlock(&g_cisd->lock);

	return 0;
}
EXPORT_SYMBOL(sb_cisd_set_data);

int sb_cisd_get_data(enum CISD_ENUM type, int *value)
{
	int ret = 0;

	if (g_cisd == NULL)
		return -ENODEV;
	if (!check_valid_type(type))
		return -EINVAL;

	mutex_lock(&g_cisd->lock);
	ret = get_data(g_cisd, type, value);
	mutex_unlock(&g_cisd->lock);

	return ret;
}
EXPORT_SYMBOL(sb_cisd_get_data);

int sb_cisd_count(unsigned int count, ...)
{
	enum CISD_ENUM type;
	int i = 0, ret = 0;
	va_list ap;

	if (g_cisd == NULL)
		return -ENODEV;
	if (!count || count > CISD_MAX)
		return -EINVAL;

	cisd_log("%d\n", count);
	mutex_lock(&g_cisd->lock);
	va_start(ap, count);

	for (; i < count; i++) {
		type = (enum CISD_ENUM)va_arg(ap, int);
		if (!check_valid_type(type)) {
			ret = -EINVAL;
			break;
		}

		count_data(g_cisd, type);
		cisd_log("%s + 1\n", sb_cisd_name[type]);
	}

	va_end(ap);
	mutex_unlock(&g_cisd->lock);
	return ret;
}
EXPORT_SYMBOL(sb_cisd_count);

int sb_cisd_count_pad(unsigned int id)
{
	struct ext_data *data = NULL;
	union raw_data temp = { 0, };

	if (g_cisd == NULL)
		return -ENODEV;

	cisd_log("pad_id = 0x%x, pad_count = %d\n", id, g_cisd->pad_list.count);
	mutex_lock(&g_cisd->lock);

	temp.pad.id = id;
	data = find_ext_data(&g_cisd->pad_list, cmp_pad_data, temp.raw);
	if (!data) {
		data = create_ext_data(temp.raw);
		if (!data) {
			mutex_unlock(&g_cisd->lock);
			return -ENOMEM;
		}

		list_add(&data->list, &g_cisd->pad_list.list);
		g_cisd->pad_list.count++;
	}
	data->data.pad.count += 1;

	mutex_unlock(&g_cisd->lock);
	return 0;
}
EXPORT_SYMBOL(sb_cisd_count_pad);

static int __init sb_cisd_init(void)
{
	struct sb_cisd *cisd;
	int ret = 0;

	cisd_log("\n");
	cisd = kzalloc(sizeof(struct sb_cisd), GFP_KERNEL);
	if (!cisd)
		return -ENOMEM;

	ret = parse_dt(cisd);
	if (ret) {
		cisd_log("failed to parse dt (ret = %d)\n", ret);
		goto err_parse;
	}

	mutex_init(&cisd->lock);
	init_ext_list(&cisd->pad_list);

	init(cisd);

	g_cisd = cisd;
	sb_sysfs_add_attrs("sb_cisd_attr", PSY_BATTERY_NAME,
		sysfs_cisd_attrs, ARRAY_SIZE(sysfs_cisd_attrs));
	return 0;

err_parse:
	kfree(cisd);
	return ret;
}
module_init(sb_cisd_init);

static void __exit sb_cisd_exit(void)
{
	cisd_log("\n");
	if (!g_cisd)
		return;

	mutex_destroy(&g_cisd->lock);
	sb_sysfs_remove_attrs("sb_cisd_attr");
	init(g_cisd);
	kfree(g_cisd);
}
module_exit(sb_cisd_exit);

MODULE_DESCRIPTION("Samsung Battery CISD");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
