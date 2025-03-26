#
# Copyright (c) 2013 Samsung Electronics Co., Ltd.
#		http://www.samsung.com
#
# Licensed under GPLv2
#


obj-$(CONFIG_EXYNOS_DPU30) += dpu.o

dpu-y += dpp_drv.o dsim_drv.o
dpu-y += decon_core.o decon_dsi.o helper.o win_update.o decon_wb.o cursor.o fence.o event_log.o format.o
dpu-$(CONFIG_SOC_S5E5515) += ./cal_5515/dsim_reg.o ./cal_5515/dpp_reg.o ./cal_5515/decon_reg.o
dpu-$(CONFIG_EXYNOS_BTS) += bts.o

ifneq ($(filter y m,$(CONFIG_USDM_PANEL)),)
dpu-$(CONFIG_USDM_PANEL) += ./panels/common_panel_drv.o\
				./panels/common_mipi_lcd.o\
				./mcd_decon.o
dpu-$(CONFIG_USDM_PANEL_DISPLAY_MODE) += ./panels/exynos_panel_modes.o
else
dpu-$(CONFIG_EXYNOS_COMMON_PANEL) += ./panels/exynos_panel_drv.o\
									./panels/panel-samsung-s6e3hab.o\
									./panels/panel-samsung-s6e3ha9.o\
									./panels/panel-samsung-s6e3ha8.o\
									./panels/panel-samsung-s6e3fa0.o\
									./panels/panel-samsung-s6e3fab.o\
									./panels/panel-samsung-s6e3had.o\
									./panels/panel-samsung-emul.o
endif
#dpu-y += panels/
