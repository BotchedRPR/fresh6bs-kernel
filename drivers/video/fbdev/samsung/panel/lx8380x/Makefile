# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Linux kernel panel device drivers.
#

subdir-ccflags-$(CONFIG_SEC_KUNIT) += \
	-I$(srctree)/$(src)/.. \
	-Wframe-larger-than=8192 \
	-Wno-unused-variable \
	-Wunused-function \
	-Wno-missing-braces \
	-Wno-implicit-fallthrough \
	-Wformat

subdir-ccflags-y += -I$(srctree)/$(src)/../oled_common


subdir-ccflags-y += -I$(srctree)/$(src)/../oled_common

mcd-panel-lx83805_boe_large-y += lx8380x.o lx83805_boe_large.o

obj-$(CONFIG_USDM_PANEL_LX8380X) += mcd-panel-lx83805_boe_large.o


subdir-ccflags-y += -I$(srctree)/$(src)/../oled_common

mcd-panel-lx83806_sdc_large-y += lx8380x.o lx83806_sdc_large.o

obj-$(CONFIG_USDM_PANEL_LX8380X) += mcd-panel-lx83806_sdc_large.o

