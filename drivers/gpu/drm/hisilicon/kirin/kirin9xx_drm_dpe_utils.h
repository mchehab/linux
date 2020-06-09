/* Copyright (c) 2013-2014, Hisilicon Tech. Co., Ltd. All rights reserved.
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

#if defined (CONFIG_HISI_FB_970)
#include "kirin970_dpe_reg.h"
#else
#include "kirin_dpe_reg.h"
#endif
#include "kirin_drm_drv.h"

/*#define CONFIG_HISI_FB_OV_BASE_USED*/
/*#define CONFIG_HISI_FB_DPP_COLORBAR_USED*/
/*#define CONFIG_HISI_FB_LDI_COLORBAR_USED*/

void set_reg(char __iomem *addr, uint32_t val, uint8_t bw, uint8_t bs);
uint32_t set_bits32(uint32_t old_val, uint32_t val, uint8_t bw, uint8_t bs);

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
void hisi_dss_online_play(struct kirin_fbdev *fbdev, struct drm_plane *plane, drm_dss_layer_t *layer);

u32 dss_get_format(u32 pixel_format);

#endif
