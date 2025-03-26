# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the kernel wireless drivers.
#

# Object files in subdirectories

obj-$(CONFIG_SB_WRL)	+= sb-wrl.o
sb-wrl-$(CONFIG_SB_WRL)	+= sb_wrl.o
obj-$(CONFIG_WIRELESS_P9222)		+=	p9222/
obj-$(CONFIG_WIRELESS_CPS4019)		+=	cps4019/

subdir-ccflags-y := -Wformat
