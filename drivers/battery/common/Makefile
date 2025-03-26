# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the kernel battery driver.
#

# Object files in subdirectories

ifneq ($(CONFIG_SB_SYSFS), )
obj-$(CONFIG_BATTERY_SAMSUNG)	+= sysfs_battery.o sysfs_device.o sysfs_event.o
obj-$(CONFIG_BATTERY_SAMSUNG)	+= sysfs_fota.o
ifneq ($(CONFIG_ENG_BATTERY_CONCEPT), )
obj-$(CONFIG_BATTERY_SAMSUNG)	+= sysfs_eng.o
endif
endif

obj-$(CONFIG_BATTERY_SAMSUNG)		+= sb-ext.o
sb-ext-$(CONFIG_BATTERY_SAMSUNG)	+= sb_ext.o

ifneq ($(CONFIG_SB_CISD), )
obj-$(CONFIG_BATTERY_SAMSUNG)	+= sec_cisd.o
endif

obj-$(CONFIG_BATTERY_SAMSUNG)	+= sec-battery.o
sec-battery-$(CONFIG_BATTERY_SAMSUNG) += sec_battery_ttf.o sec_battery_thermal.o
sec-battery-$(CONFIG_BATTERY_SAMSUNG) += sec_battery.o
ifneq ($(CONFIG_SB_STARTUP), )
sec-battery-$(CONFIG_BATTERY_SAMSUNG) += sec_startup.o
endif

subdir-ccflags-y := -Wformat
