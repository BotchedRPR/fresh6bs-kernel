# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the kernel bc step modules.
#

# Object files in subdirectories

obj-$(CONFIG_SB_STEP)			+= sb-step-base.o
sb-step-base-$(CONFIG_SB_STEP)	+= sb_step_base.o

obj-$(CONFIG_SB_STEP_WATCH5) 			+= sb-step-watch5.o
sb-step-watch5-$(CONFIG_SB_STEP_WATCH5)	+= sb_step_watch5.o

obj-$(CONFIG_SB_STEP_WATCH6) 			+= sb-step-watch6.o
sb-step-watch6-$(CONFIG_SB_STEP_WATCH6)	+= sb_step_watch6.o

subdir-ccflags-y := -Wformat
