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

 #include "ssp.h"
void test_ssp_dump(struct ssp_data *data);
void write_ssp_dump_file(struct ssp_data *data, char *info, void *buf, int size, int type, u32 *gpr, int num_gpr, u32 *hardfault_info);
int ssp_dumpstate(struct ssp_data *data, char *out_name);
unsigned int ssp_nanohub_log_buf_check(struct ssp_data *data);

