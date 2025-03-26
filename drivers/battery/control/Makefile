# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the kernel battery control modules.
#

# Object files in subdirectories

obj-$(CONFIG_SB_STEP)			+=	step/
obj-$(CONFIG_SB_TEMP)			+=	temp/

obj-$(CONFIG_SB_CONTROL)		+= sb-ctrl.o
sb-ctrl-$(CONFIG_SB_CONTROL)	+= sb_ctrl.o

subdir-ccflags-y := -Wformat
