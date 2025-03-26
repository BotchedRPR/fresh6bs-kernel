# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the kernel startup module.
#

# Object files in subdirectories

obj-$(CONFIG_SB_STARTUP)		+= sb-startup.o
sb-startup-$(CONFIG_SB_STARTUP)	+= sb_startup.o

subdir-ccflags-y := -Wformat
