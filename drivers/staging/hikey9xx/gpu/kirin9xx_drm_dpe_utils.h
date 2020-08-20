/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2013-2014, Hisilicon Tech. Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 */

#ifndef KIRIN_DRM_DPE_UTILS_H
#define KIRIN_DRM_DPE_UTILS_H

#include <linux/kernel.h>

#include <drm/drm_plane.h>
#include <drm/drm_crtc.h>

#include "kirin9xx_drm_drv.h"
#include "kirin9xx_dpe.h"

enum dss_channel {
	DSS_CH1 = 0,	/* channel 1 for primary plane */
	DSS_CH_NUM
};

#define PRIMARY_CH	DSS_CH1 /* primary plane */

enum hisi_fb_pixel_format {
	HISI_FB_PIXEL_FORMAT_RGB_565 = 0,
	HISI_FB_PIXEL_FORMAT_RGBX_4444,
	HISI_FB_PIXEL_FORMAT_RGBA_4444,
	HISI_FB_PIXEL_FORMAT_RGBX_5551,
	HISI_FB_PIXEL_FORMAT_RGBA_5551,
	HISI_FB_PIXEL_FORMAT_RGBX_8888,
	HISI_FB_PIXEL_FORMAT_RGBA_8888,

	HISI_FB_PIXEL_FORMAT_BGR_565,
	HISI_FB_PIXEL_FORMAT_BGRX_4444,
	HISI_FB_PIXEL_FORMAT_BGRA_4444,
	HISI_FB_PIXEL_FORMAT_BGRX_5551,
	HISI_FB_PIXEL_FORMAT_BGRA_5551,
	HISI_FB_PIXEL_FORMAT_BGRX_8888,
	HISI_FB_PIXEL_FORMAT_BGRA_8888,

	HISI_FB_PIXEL_FORMAT_YUV_422_I,

	/* YUV Semi-planar */
	HISI_FB_PIXEL_FORMAT_YCbCr_422_SP,	/* NV16 */
	HISI_FB_PIXEL_FORMAT_YCrCb_422_SP,
	HISI_FB_PIXEL_FORMAT_YCbCr_420_SP,
	HISI_FB_PIXEL_FORMAT_YCrCb_420_SP,	/* NV21 */

	/* YUV Planar */
	HISI_FB_PIXEL_FORMAT_YCbCr_422_P,
	HISI_FB_PIXEL_FORMAT_YCrCb_422_P,
	HISI_FB_PIXEL_FORMAT_YCbCr_420_P,
	HISI_FB_PIXEL_FORMAT_YCrCb_420_P,	/* HISI_FB_PIXEL_FORMAT_YV12 */

	/* YUV Package */
	HISI_FB_PIXEL_FORMAT_YUYV_422,
	HISI_FB_PIXEL_FORMAT_UYVY_422,
	HISI_FB_PIXEL_FORMAT_YVYU_422,
	HISI_FB_PIXEL_FORMAT_VYUY_422,
	HISI_FB_PIXEL_FORMAT_MAX,

	HISI_FB_PIXEL_FORMAT_UNSUPPORT = 800
};

struct dss_hw_ctx {
	void __iomem *base;
	struct regmap *noc_regmap;
	struct reset_control *reset;
	u32 g_dss_version_tag;

	void __iomem *noc_dss_base;
	void __iomem *peri_crg_base;
	void __iomem *pmc_base;
	void __iomem *sctrl_base;
	void __iomem *media_crg_base;
	void __iomem *pctrl_base;
	void __iomem *mmbuf_crg_base;
	void __iomem *pmctrl_base;

	struct clk *dss_axi_clk;
	struct clk *dss_pclk_dss_clk;
	struct clk *dss_pri_clk;
	struct clk *dss_pxl0_clk;
	struct clk *dss_pxl1_clk;
	struct clk *dss_mmbuf_clk;
	struct clk *dss_pclk_mmbuf_clk;

	struct dss_clk_rate *dss_clk;

	struct regulator *dpe_regulator;
	struct regulator *mmbuf_regulator;
	struct regulator *mediacrg_regulator;

	bool power_on;
	int irq;

	wait_queue_head_t vactive0_end_wq;
	u32 vactive0_end_flag;
	ktime_t vsync_timestamp;
	ktime_t vsync_timestamp_prev;

	struct iommu_domain *mmu_domain;
	char __iomem *screen_base;
	unsigned long smem_start;
	unsigned long screen_size;

	/* Version-specific data */
	u32 g_dss_module_base[DSS_CHN_MAX_DEFINE][MODULE_CHN_MAX];
	u32 g_dss_module_ovl_base[DSS_MCTL_IDX_MAX][MODULE_OVL_MAX];
	int g_scf_lut_chn_coef_idx[DSS_CHN_MAX_DEFINE];
	u32 g_dss_module_cap[DSS_CHN_MAX_DEFINE][MODULE_CAP_MAX];
	u32 g_dss_chn_sid_num[DSS_CHN_MAX_DEFINE];
	u32 g_dss_smmu_smrx_idx[DSS_CHN_MAX_DEFINE];
	u32 smmu_offset;
	u32 afbc_header_addr_align;
	u32 dss_mmbuf_clk_rate_power_off;
	u32 rot_mem_ctrl;
	u32 dither_mem_ctrl;
	u32 arsr2p_lb_mem_ctrl;
	u32 pxl0_clk_rate_power_off;
};

void kirin960_dpe_defs(struct dss_hw_ctx *ctx);
void kirin970_dpe_defs(struct dss_hw_ctx *ctx);

struct dss_clk_rate {
	u64 dss_pri_clk_rate;
	u64 dss_pclk_dss_rate;
	u64 dss_pclk_pctrl_rate;
	u64 dss_mmbuf_rate;
	u32 dss_voltage_value; /* 0:0.7v, 2:0.8v */
	u32 reserved;
};

struct dss_crtc {
	struct drm_crtc base;
	struct dss_hw_ctx *ctx;
	bool enable;
	u32 out_format;
	u32 bgr_fmt;
};

struct dss_plane {
	struct drm_plane base;
	/* void *ctx; */
	void *acrtc;
	u8 ch; /* channel */
};

struct dss_data {
	struct dss_crtc acrtc;
	struct dss_plane aplane[DSS_CH_NUM];
	struct dss_hw_ctx ctx;
};

/* ade-format info: */
struct dss_format {
	u32 pixel_format;
	enum hisi_fb_pixel_format dss_format;
};

struct dss_img {
	u32 format;
	u32 width;
	u32 height;
	u32 bpp;		/* bytes per pixel */
	u32 buf_size;
	u32 stride;
	u32 stride_plane1;
	u32 stride_plane2;
	u64 phy_addr;
	u64 vir_addr;
	u32 offset_plane1;
	u32 offset_plane2;

	u64 afbc_header_addr;
	u64 afbc_payload_addr;
	u32 afbc_header_stride;
	u32 afbc_payload_stride;
	u32 afbc_scramble_mode;
	u32 mmbuf_base;
	u32 mmbuf_size;

	u32 mmu_enable;
	u32 csc_mode;
	u32 secure_mode;
	s32 shared_fd;
	u32 reserved0;
};

struct dss_rect {
	s32 x;
	s32 y;
	s32 w;
	s32 h;
};

struct drm_dss_layer {
	struct dss_img img;
	struct dss_rect src_rect;
	struct dss_rect src_rect_mask;
	struct dss_rect dst_rect;
	u32 transform;
	s32 blending;
	u32 glb_alpha;
	u32 color;		/* background color or dim color */
	s32 layer_idx;
	s32 chn_idx;
	u32 need_cap;
	s32 acquire_fence;
};

static inline void set_reg(char __iomem *addr, uint32_t val, uint8_t bw,
			   uint8_t bs)
{
	u32 mask = (1UL << bw) - 1UL;
	u32 tmp = 0;

	tmp = readl(addr);
	tmp &= ~(mask << bs);

	writel(tmp | ((val & mask) << bs), addr);
}

u32 set_bits32(u32 old_val, uint32_t val, uint8_t bw, uint8_t bs);

void init_dbuf(struct dss_crtc *acrtc);
void init_dpp(struct dss_crtc *acrtc);
void init_other(struct dss_crtc *acrtc);
void init_ldi(struct dss_crtc *acrtc);

void deinit_ldi(struct dss_crtc *acrtc);
void enable_ldi(struct dss_crtc *acrtc);
void disable_ldi(struct dss_crtc *acrtc);

void dss_inner_clk_pdp_enable(struct dss_hw_ctx *ctx);
void dss_inner_clk_pdp_disable(struct dss_hw_ctx *ctx);
void dss_inner_clk_common_enable(struct dss_hw_ctx *ctx);
void dss_inner_clk_common_disable(struct dss_hw_ctx *ctx);
void dpe_interrupt_clear(struct dss_crtc *acrtc);
void dpe_interrupt_unmask(struct dss_crtc *acrtc);
void dpe_interrupt_mask(struct dss_crtc *acrtc);
int dpe_common_clk_enable(struct dss_hw_ctx *ctx);
int dpe_common_clk_disable(struct dss_hw_ctx *ctx);
int dpe_inner_clk_enable(struct dss_hw_ctx *ctx);
int dpe_inner_clk_disable(struct dss_hw_ctx *ctx);
int dpe_regulator_enable(struct dss_hw_ctx *ctx);
int dpe_regulator_disable(struct dss_hw_ctx *ctx);
int mediacrg_regulator_enable(struct dss_hw_ctx *ctx);
int mediacrg_regulator_disable(struct dss_hw_ctx *ctx);
int dpe_set_clk_rate(struct dss_hw_ctx *ctx);

int dpe_irq_enable(struct dss_crtc *acrtc);
int dpe_irq_disable(struct dss_crtc *acrtc);

int dpe_init(struct dss_crtc *acrtc);
int dpe_deinit(struct dss_crtc *acrtc);
void dpe_check_itf_status(struct dss_crtc *acrtc);
int dpe_set_clk_rate_on_pll0(struct dss_hw_ctx *ctx);
int dpe_set_common_clk_rate_on_pll0(struct dss_hw_ctx *ctx);
int dpe_set_pixel_clk_rate_on_pll0(struct dss_hw_ctx *ctx);

void hisifb_dss_on(struct dss_hw_ctx *ctx);
void hisi_dss_mctl_on(struct dss_hw_ctx *ctx);

void hisi_dss_unflow_handler(struct dss_hw_ctx *ctx, bool unmask);
int hisi_dss_mctl_mutex_lock(struct dss_hw_ctx *ctx);
int hisi_dss_mctl_mutex_unlock(struct dss_hw_ctx *ctx);
int hisi_dss_ovl_base_config(struct dss_hw_ctx *ctx, u32 xres, u32 yres);

void hisi_fb_pan_display(struct drm_plane *plane);
void hisi_dss_online_play(struct kirin_fbdev *fbdev, struct drm_plane *plane, struct drm_dss_layer *layer);

u32 dss_get_format(u32 pixel_format);

#endif
