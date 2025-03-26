# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the kernel work module.
#

# Object files in subdirectories

obj-$(CONFIG_SB_WORK)		+= sb-work.o
sb-work-$(CONFIG_SB_WORK)	+= sb_work_core.o sb_work.o sb_work_dev.o

subdir-ccflags-y := -Wformat
