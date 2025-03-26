# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the kernel battery core api.
#

# Object files in subdirectories

obj-$(CONFIG_SB_CORE)	+= sb-core.o
sb-core-$(CONFIG_SB_CORE)	+= sb_core.o
sb-core-$(CONFIG_SB_PQUEUE)	+= sb_pqueue.o
sb-core-$(CONFIG_SB_NOTIFY)	+= sb_notify.o
sb-core-$(CONFIG_SB_SYSFS)	+= sb_sysfs.o
sb-core-$(CONFIG_SB_MISC)	+= sb_misc.o
sb-core-$(CONFIG_SB_VOTE)	+= sb_vote.o

subdir-ccflags-y := -Wformat
