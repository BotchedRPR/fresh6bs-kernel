/*
 * Samsung Exynos5 SoC series Sensor driver
 *
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef IS_CIS_H
#define IS_CIS_H

/* if you need color-bar, change a below macro to 2 */
#define CIS_TEST_PATTERN_MODE 0
#define CIS_STREAM_OFF_WAIT_TIME 2000	/* 2ms */
#define CIS_STREAM_ON_WAIT_TIME 2000	/* 2ms */

struct sensor_pll_info_compact {
	u32 ext_clk;
	u64 mipi_datarate;
	u64 pclk;
	u32 frame_length_lines;
	u32 line_length_pck;
};

struct sensor_pll_info {
	u32 ext_clk;
	u32 vt_pix_clk_div;
	u32 vt_sys_clk_div;
	u32 pre_pll_clk_div;
	u32 pll_multiplier;
	u32 op_pix_clk_div;
	u32 op_sys_clk_div;

	u32 secnd_pre_pll_clk_div;
	u32 secnd_pll_multiplier;
	u32 frame_length_lines;
	u32 line_length_pck;
};

struct sensor_crop_xy_info {
	u32 crop_x;
	u32 crop_y;
};

/* FIXME: make it dynamic parsing */
#define I2C_READ  4
#define I2C_WRITE 3
#define I2C_BYTE  2
#define I2C_DATA  1
#define I2C_ADDR  0

enum i2c_write {
	I2C_WRITE_ADDR8_DATA8 = 0x0,
	I2C_WRITE_ADDR16_DATA8,
	I2C_WRITE_ADDR16_DATA16
};

enum ixc_type {
	I2C_TYPE = 0,
	I3C_TYPE,
};

struct is_cis_log {
	unsigned int	type; /* read / write */
	unsigned int	size; /* 8bit, 16bit */
	unsigned int	addr;
	unsigned int	data;
	char		*name;
};

struct sensor_common_mode_attr {
	u32 max_analog_gain;
	u32 max_digital_gain;
	u32 min_coarse_integration_time;
	u32 max_margin_coarse_integration_time;
};

int sensor_cis_set_registers(struct v4l2_subdev *subdev, const u32 *regs, const u32 size);
int sensor_cis_set_registers_addr8(struct v4l2_subdev *subdev, const u32 *regs, const u32 size);
int sensor_cis_check_rev_on_init(struct v4l2_subdev *subdev);
int sensor_cis_check_rev(struct is_cis *cis);

u32 sensor_cis_calc_again_code(u32 permile);
u32 sensor_cis_calc_again_permile(u32 code);

u32 sensor_cis_calc_dgain_code(u32 permile);
u32 sensor_cis_calc_dgain_permile(u32 code);

int sensor_cis_compensate_gain_for_extremely_br(struct v4l2_subdev *subdev, u32 expo, u32 *again, u32 *dgain);

int sensor_cis_dump_registers(struct v4l2_subdev *subdev, const u32 *regs, const u32 size);

u32 sensor_cis_do_div64(u64 num, u32 den);

int sensor_cis_parse_dt(struct device *dev, struct v4l2_subdev *subdev);

void sensor_cis_recover_i2c_fail(struct v4l2_subdev *subdev_cis);
int sensor_cis_wait_streamoff(struct v4l2_subdev *subdev);
int sensor_cis_wait_streamoff_mipi_end(struct v4l2_subdev *subdev);
int sensor_cis_wait_streamon(struct v4l2_subdev *subdev);

int sensor_cis_set_initial_exposure(struct v4l2_subdev *subdev);
int sensor_cis_active_test(struct v4l2_subdev *subdev);
int sensor_cis_get_bayer_order(struct v4l2_subdev *subdev, u32 *bayer_order);
void sensor_cis_log_status(struct is_cis *cis, struct i2c_client *client,
	const struct is_cis_log *log, unsigned int num, char *fn);
int sensor_cis_probe(void *client, struct device *client_dev,
		struct is_device_sensor_peri **sensor_peri, enum ixc_type type);
#endif
