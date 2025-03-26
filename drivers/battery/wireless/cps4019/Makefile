# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the kernel CPS4019 wireless driver.
#

# Object files in subdirectories

obj-$(CONFIG_WIRELESS_CPS4019)				+= cps4019-charger.o
cps4019-charger-$(CONFIG_WIRELESS_CPS4019)	+= cps4019_core.o cps4019_firmware.o cps4019_charger.o
ifneq ($(CONFIG_SB_STARTUP), )
cps4019-charger-$(CONFIG_WIRELESS_CPS4019)	+= cps4019_startup.o
endif

subdir-ccflags-y := -Wformat
