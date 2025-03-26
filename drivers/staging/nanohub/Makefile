#
# Makefile for nanohub
#

obj-$(CONFIG_NANOHUB) += nanohub.o
nanohub-$(CONFIG_SENSOR_DRV) := main.o comms.o
nanohub-y += chub.o chub_bootup.o chub_dbg.o chub_log.o chub_exynos.o
nanohub-y += chub_ipc_if.o ipc_chub.o ipc_chub_ap.o ipc_common.o ipc_hw.o

#
# Makefile for the sensor drivers.
#

# Each configuration option enables a list of files.

nanohub-$(CONFIG_SENSORS_SSP) += ssp_dev.o ssp_data.o ssp_sysfs.o ssp_iio.o ssp_scontext.o \
							ssp_firmware.o ssp_debug.o ssp_dump.o \
							sensors_core.o ssp_input.o  ssp_bigdata.o \
							factory/ssp_factory.o  factory/mcu_chub.o \
							ssp_comm.o ssp_platform.o ssp_lpm.o ssp_motor.o \
							factory/accel_lsm6dsl.o factory/accel_lsm6dsl_32g.o factory/gyro_lsm6dsl.o \
							factory/pressure_lps25h.o factory/hrm_pps960.o factory/hrm_pps960_bia.o factory/hrm_pps960_ecg.o \
							factory/magnetic_ak09918c.o factory/temperature_mlx90632.o \
							factory/thermistor.o

nanohub-$(CONFIG_SENSORS_SSP_HALLIC_SENSOR) += factory/digital_hall_ak09973d.o
nanohub-$(CONFIG_SENSORS_SSP_LUCKY)	+= factory/light_opt3007.o
nanohub-$(CONFIG_SENSORS_SSP_FRESH6)	+= factory/light_stk31e15.o
nanohub-$(CONFIG_SENSORS_SSP_WISE6)	+= factory/light_stk31e15.o
