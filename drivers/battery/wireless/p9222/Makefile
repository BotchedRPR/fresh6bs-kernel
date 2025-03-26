# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the kernel P9222 wireless driver.
#

# Object files in subdirectories

obj-$(CONFIG_WIRELESS_P9222)			+= p9222-charger.o
p9222-charger-$(CONFIG_WIRELESS_P9222)	+= p9222_charger.o
ifneq ($(CONFIG_SB_STARTUP), )
p9222-charger-$(CONFIG_WIRELESS_P9222)	+= p9222_startup.o
endif

subdir-ccflags-y := -Wformat
