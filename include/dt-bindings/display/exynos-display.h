/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *
 * Author: Jiun Yu <jiun.yu@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Device Tree binding constants for exynos display.
*/

#ifndef _DT_BINDINGS_EXYNOS_DISPLAY_H
#define _DT_BINDINGS_EXYNOS_DISPLAY_H


/* RGB 8bit display */
/* 4byte */
#define	DECON_PIXEL_FORMAT_ARGB_8888		0
#define DECON_PIXEL_FORMAT_ABGR_8888		1
#define DECON_PIXEL_FORMAT_RGBA_8888		2
#define DECON_PIXEL_FORMAT_BGRA_8888		3
#define DECON_PIXEL_FORMAT_XRGB_8888		4
#define DECON_PIXEL_FORMAT_XBGR_8888		5
#define DECON_PIXEL_FORMAT_RGBX_8888		6
#define DECON_PIXEL_FORMAT_BGRX_8888		7

/* 2byte */
#define	DECON_PIXEL_FORMAT_RGBA_5551		8
#define DECON_PIXEL_FORMAT_BGRA_5551		9
#define DECON_PIXEL_FORMAT_ABGR_4444		10
#define DECON_PIXEL_FORMAT_RGBA_4444		11
#define DECON_PIXEL_FORMAT_BGRA_4444		12
#define DECON_PIXEL_FORMAT_RGB_565		13
#define DECON_PIXEL_FORMAT_BGR_565		14

/* RGB 10bit display */
/* 4byte */
#define DECON_PIXEL_FORMAT_ARGB_2101010		15
#define DECON_PIXEL_FORMAT_ABGR_2101010		16
#define DECON_PIXEL_FORMAT_RGBA_1010102		17
#define DECON_PIXEL_FORMAT_BGRA_1010102		18

/* YUV 8bit display */
/* YUV422 2P */
#define DECON_PIXEL_FORMAT_NV16			19
#define DECON_PIXEL_FORMAT_NV61			20

/* YUV422 3P */
#define DECON_PIXEL_FORMAT_YVU422_3P		21

/* YUV420 2P */
#define DECON_PIXEL_FORMAT_NV12			22
#define DECON_PIXEL_FORMAT_NV21			23
#define DECON_PIXEL_FORMAT_NV12M		24
#define DECON_PIXEL_FORMAT_NV21M		25

/* YUV420 3P */
#define DECON_PIXEL_FORMAT_YUV420		26
#define DECON_PIXEL_FORMAT_YVU420		27
#define DECON_PIXEL_FORMAT_YUV420M		28
#define DECON_PIXEL_FORMAT_YVU420M		29

/* YUV - 2 planes but 1 buffer */
#define DECON_PIXEL_FORMAT_NV12N		30
#define DECON_PIXEL_FORMAT_NV12N_10B		31

/* YUV 10bit display */
/* YUV420 2P */
#define DECON_PIXEL_FORMAT_NV12M_P010		32
#define DECON_PIXEL_FORMAT_NV21M_P010		33

/* YUV420(P8+2) 4P */
#define DECON_PIXEL_FORMAT_NV12M_S10B		34
#define DECON_PIXEL_FORMAT_NV21M_S10B		35

/* YUV422 2P */
#define DECON_PIXEL_FORMAT_NV16M_P210		36
#define DECON_PIXEL_FORMAT_NV61M_P210		37

/* YUV422(P8+2) 4P */
#define DECON_PIXEL_FORMAT_NV16M_S10B		38
#define DECON_PIXEL_FORMAT_NV61M_S10B		39

#define DECON_PIXEL_FORMAT_NV12_P010		40

/* formats for lossless SBWC case */
#define DECON_PIXEL_FORMAT_NV12M_SBWC_8B	41
#define DECON_PIXEL_FORMAT_NV12M_SBWC_10B	42
#define DECON_PIXEL_FORMAT_NV21M_SBWC_8B	43
#define DECON_PIXEL_FORMAT_NV21M_SBWC_10B	44
#define DECON_PIXEL_FORMAT_NV12N_SBWC_8B	45
#define DECON_PIXEL_FORMAT_NV12N_SBWC_10B	46

/* formats for lossy SBWC case */
#define DECON_PIXEL_FORMAT_NV12M_SBWC_8B_L50	47
#define DECON_PIXEL_FORMAT_NV12M_SBWC_8B_L75	48
#define DECON_PIXEL_FORMAT_NV12N_SBWC_8B_L50	49
#define DECON_PIXEL_FORMAT_NV12N_SBWC_8B_L75	50
#define DECON_PIXEL_FORMAT_NV12M_SBWC_10B_L40	51
#define DECON_PIXEL_FORMAT_NV12M_SBWC_10B_L60	52
#define DECON_PIXEL_FORMAT_NV12M_SBWC_10B_L80	53
#define DECON_PIXEL_FORMAT_NV12N_SBWC_10B_L40	54
#define DECON_PIXEL_FORMAT_NV12N_SBWC_10B_L60	55
#define DECON_PIXEL_FORMAT_NV12N_SBWC_10B_L80	56

/*
 * Attention: Keep the DRM_FORMAT_* bit definitions in sync with
 * include/uapi/drm/drm_fourcc.h bit definitions.
 */
#define fourcc_code(a, b, c, d) ((a) | ((b) << 8) | \
				 ((c) << 16) | ((d) << 24))

#define DRM_FORMAT_ARGB8888	fourcc_code('A', 'R', '2', '4') /* [31:0] A:R:G:B 8:8:8:8 little endian */
#define DRM_FORMAT_ABGR8888	fourcc_code('A', 'B', '2', '4') /* [31:0] A:B:G:R 8:8:8:8 little endian */
#define DRM_FORMAT_RGBA8888	fourcc_code('R', 'A', '2', '4') /* [31:0] R:G:B:A 8:8:8:8 little endian */
#define DRM_FORMAT_BGRA8888	fourcc_code('B', 'A', '2', '4') /* [31:0] B:G:R:A 8:8:8:8 little endian */
#define DRM_FORMAT_XRGB8888	fourcc_code('X', 'R', '2', '4') /* [31:0] x:R:G:B 8:8:8:8 little endian */
#define DRM_FORMAT_XBGR8888	fourcc_code('X', 'B', '2', '4') /* [31:0] x:B:G:R 8:8:8:8 little endian */
#define DRM_FORMAT_RGBX8888	fourcc_code('R', 'X', '2', '4') /* [31:0] R:G:B:x 8:8:8:8 little endian */
#define DRM_FORMAT_BGRX8888	fourcc_code('B', 'X', '2', '4') /* [31:0] B:G:R:x 8:8:8:8 little endian */
#define DRM_FORMAT_RGB565	fourcc_code('R', 'G', '1', '6') /* [15:0] R:G:B 5:6:5 little endian */
#define DRM_FORMAT_BGR565	fourcc_code('B', 'G', '1', '6') /* [15:0] B:G:R 5:6:5 little endian */
#define DRM_FORMAT_ARGB2101010	fourcc_code('A', 'R', '3', '0') /* [31:0] A:R:G:B 2:10:10:10 little endian */
#define DRM_FORMAT_ABGR2101010	fourcc_code('A', 'B', '3', '0') /* [31:0] A:B:G:R 2:10:10:10 little endian */
#define DRM_FORMAT_RGBA1010102	fourcc_code('R', 'A', '3', '0') /* [31:0] R:G:B:A 10:10:10:2 little endian */
#define DRM_FORMAT_BGRA1010102	fourcc_code('B', 'A', '3', '0') /* [31:0] B:G:R:A 10:10:10:2 little endian */
#define DRM_FORMAT_ARGB16161616F fourcc_code('A', 'R', '4', 'H') /* [63:0] A:R:G:B 16:16:16:16 little endian */
#define DRM_FORMAT_ABGR16161616F fourcc_code('A', 'B', '4', 'H') /* [63:0] A:B:G:R 16:16:16:16 little endian */
#define DRM_FORMAT_NV12		fourcc_code('N', 'V', '1', '2') /* 2x2 subsampled Cr:Cb plane */
#define DRM_FORMAT_NV21		fourcc_code('N', 'V', '2', '1') /* 2x2 subsampled Cb:Cr plane */
#define DRM_FORMAT_P010		fourcc_code('P', '0', '1', '0') /* 2x2 subsampled Cr:Cb plane 10 bits per channel */

/*
 * Attention: Keep the DRM_MODE_TYPE_* bit definitions in sync with
 * include/uapi/drm/drm_mode.h bit definitions.
 */
#define DRM_MODE_TYPE_PREFERRED	(1 << 3)
#define DRM_MODE_TYPE_USERDEF	(1 << 5)
#define DRM_MODE_TYPE_DRIVER	(1 << 6)

/*
 * Attention: Keep the DPP_ATTR_* bit definitions in sync with
 * drivers/gpu/drm/samsung/dpu/cal_common/dpp_cal.h bit definitions.
 */
#define DPP_ATTR_AFBC		(1 << 0)
#define DPP_ATTR_SAJC		(1 << 0)
#define DPP_ATTR_BLOCK		(1 << 1)
#define DPP_ATTR_FLIP		(1 << 2)
#define DPP_ATTR_ROT		(1 << 3)
#define DPP_ATTR_CSC		(1 << 4)
#define DPP_ATTR_SCALE		(1 << 5)
#define DPP_ATTR_HDR		(1 << 6)
#define DPP_ATTR_C_HDR		(1 << 7)
#define DPP_ATTR_C_HDR10_PLUS	(1 << 8)
#define DPP_ATTR_WCG            (1 << 9)
#define DPP_ATTR_SBWC           (1 << 10)
#define DPP_ATTR_HDR10_PLUS	(1 << 11)
#define DPP_ATTR_IDMA		(1 << 16)
#define DPP_ATTR_ODMA		(1 << 17)
#define DPP_ATTR_DPP		(1 << 18)
#define DPP_ATTR_SRAMC		(1 << 19)
#define DPP_ATTR_HDR_COMM	(1 << 20)

/*
 * Attention: Keep the HDR_FORMAT_* bit definitions in sync with
 * drivers/gpu/drm/samsung/dpu/exynos_drm_connector.h HDR_* bit definitions.
 */
#define HDR_FORMAT_DV		(1 << 1)
#define HDR_FORMAT_HDR10	(1 << 2)
#define HDR_FORMAT_HLG		(1 << 3)
#define HDR_FORMAT_HDR10P	(1 << 4)
#endif	/* _DT_BINDINGS_EXYNOS_DISPLAY_H */
