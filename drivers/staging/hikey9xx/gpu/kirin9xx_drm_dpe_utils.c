// SPDX-License-Identifier: GPL-2.0
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
#include <drm/drm_drv.h>
#include <drm/drm_mipi_dsi.h>

#include "kirin9xx_drm_dpe_utils.h"
#include "kirin9xx_dpe.h"

DEFINE_SEMAPHORE(hisi_fb_dss_regulator_sem);

struct mipi_ifbc_division g_mipi_ifbc_division[MIPI_DPHY_NUM][IFBC_TYPE_MAX] = {
	/* single mipi */
	{
			/* none */
		{
			XRES_DIV_1, YRES_DIV_1, IFBC_COMP_MODE_0, PXL0_DIV2_GT_EN_CLOSE,
			PXL0_DIV4_GT_EN_CLOSE, PXL0_DIVCFG_0, PXL0_DSI_GT_EN_1
		}, {
			/* orise2x */
			XRES_DIV_2, YRES_DIV_1, IFBC_COMP_MODE_0, PXL0_DIV2_GT_EN_OPEN,
			PXL0_DIV4_GT_EN_CLOSE, PXL0_DIVCFG_1, PXL0_DSI_GT_EN_3
		}, {
			/* orise3x */
			XRES_DIV_3, YRES_DIV_1, IFBC_COMP_MODE_1, PXL0_DIV2_GT_EN_OPEN,
			PXL0_DIV4_GT_EN_CLOSE, PXL0_DIVCFG_2, PXL0_DSI_GT_EN_3
		}, {
			/* himax2x */
			XRES_DIV_2, YRES_DIV_1, IFBC_COMP_MODE_2, PXL0_DIV2_GT_EN_OPEN,
			PXL0_DIV4_GT_EN_CLOSE, PXL0_DIVCFG_1, PXL0_DSI_GT_EN_3
		}, {
			/* rsp2x */
			XRES_DIV_2, YRES_DIV_1, IFBC_COMP_MODE_3, PXL0_DIV2_GT_EN_CLOSE,
			PXL0_DIV4_GT_EN_OPEN, PXL0_DIVCFG_1, PXL0_DSI_GT_EN_3
		}, {
			/*
			 * rsp3x
			 * NOTE: in reality: xres_div = 1.5, yres_div = 2,
			 * amended in "mipi_ifbc_get_rect" function
			 */
			XRES_DIV_3, YRES_DIV_1, IFBC_COMP_MODE_4, PXL0_DIV2_GT_EN_CLOSE,
			PXL0_DIV4_GT_EN_OPEN, PXL0_DIVCFG_2, PXL0_DSI_GT_EN_3
		}, {
			/* vesa2x_1pipe */
			XRES_DIV_2, YRES_DIV_1, IFBC_COMP_MODE_5, PXL0_DIV2_GT_EN_CLOSE,
			PXL0_DIV4_GT_EN_CLOSE, PXL0_DIVCFG_1, PXL0_DSI_GT_EN_3
		}, {
			/* vesa3x_1pipe */
			XRES_DIV_3, YRES_DIV_1, IFBC_COMP_MODE_5, PXL0_DIV2_GT_EN_CLOSE,
			PXL0_DIV4_GT_EN_CLOSE, PXL0_DIVCFG_2, PXL0_DSI_GT_EN_3
		}, {
			/* vesa2x_2pipe */
			XRES_DIV_2, YRES_DIV_1, IFBC_COMP_MODE_6, PXL0_DIV2_GT_EN_OPEN,
			PXL0_DIV4_GT_EN_CLOSE, PXL0_DIVCFG_1, PXL0_DSI_GT_EN_3
		}, {
			/* vesa3x_2pipe */
			XRES_DIV_3, YRES_DIV_1, IFBC_COMP_MODE_6, PXL0_DIV2_GT_EN_OPEN,
			PXL0_DIV4_GT_EN_CLOSE, PXL0_DIVCFG_2, PXL0_DSI_GT_EN_3
		}

	/* dual mipi */
	}, {
		{
			/* none */
			XRES_DIV_2, YRES_DIV_1, IFBC_COMP_MODE_0, PXL0_DIV2_GT_EN_CLOSE,
			PXL0_DIV4_GT_EN_CLOSE, PXL0_DIVCFG_1, PXL0_DSI_GT_EN_3
		}, {
			/* orise2x */
			XRES_DIV_4, YRES_DIV_1, IFBC_COMP_MODE_0, PXL0_DIV2_GT_EN_OPEN,
			PXL0_DIV4_GT_EN_CLOSE, PXL0_DIVCFG_3, PXL0_DSI_GT_EN_3
		}, {
			/* orise3x */
			XRES_DIV_6, YRES_DIV_1, IFBC_COMP_MODE_1, PXL0_DIV2_GT_EN_OPEN,
			PXL0_DIV4_GT_EN_CLOSE, PXL0_DIVCFG_5, PXL0_DSI_GT_EN_3
		}, {
			/* himax2x */
			XRES_DIV_4, YRES_DIV_1, IFBC_COMP_MODE_2, PXL0_DIV2_GT_EN_OPEN,
			PXL0_DIV4_GT_EN_CLOSE, PXL0_DIVCFG_3, PXL0_DSI_GT_EN_3
		}, {
			/* rsp2x */
			XRES_DIV_4, YRES_DIV_1, IFBC_COMP_MODE_3, PXL0_DIV2_GT_EN_CLOSE,
			PXL0_DIV4_GT_EN_OPEN, PXL0_DIVCFG_3, PXL0_DSI_GT_EN_3
		}, {
			/* rsp3x */
			XRES_DIV_3, YRES_DIV_2, IFBC_COMP_MODE_4, PXL0_DIV2_GT_EN_CLOSE,
			PXL0_DIV4_GT_EN_OPEN, PXL0_DIVCFG_5, PXL0_DSI_GT_EN_3
		}, {
			/* vesa2x_1pipe */
			XRES_DIV_4, YRES_DIV_1, IFBC_COMP_MODE_5, PXL0_DIV2_GT_EN_CLOSE,
			PXL0_DIV4_GT_EN_CLOSE, PXL0_DIVCFG_3, PXL0_DSI_GT_EN_3
		}, {
			/* vesa3x_1pipe */
			XRES_DIV_6, YRES_DIV_1, IFBC_COMP_MODE_5, PXL0_DIV2_GT_EN_CLOSE,
			PXL0_DIV4_GT_EN_CLOSE, PXL0_DIVCFG_5, PXL0_DSI_GT_EN_3
		}, {
			/* vesa2x_2pipe */
			XRES_DIV_4, YRES_DIV_1, IFBC_COMP_MODE_6, PXL0_DIV2_GT_EN_OPEN,
			PXL0_DIV4_GT_EN_CLOSE, PXL0_DIVCFG_3, PXL0_DSI_GT_EN_3
		}, {
			/* vesa3x_2pipe */
			XRES_DIV_6, YRES_DIV_1, IFBC_COMP_MODE_6, PXL0_DIV2_GT_EN_OPEN,
			PXL0_DIV4_GT_EN_CLOSE, PXL0_DIVCFG_5, 3
		}
	}
};

u32 set_bits32(u32 old_val, uint32_t val, uint8_t bw, uint8_t bs)
{
	u32 mask = (1UL << bw) - 1UL;
	u32 tmp = 0;

	tmp = old_val;
	tmp &= ~(mask << bs);

	return (tmp | ((val & mask) << bs));
}

static int mipi_ifbc_get_rect(struct dss_rect *rect)
{
	u32 ifbc_type;
	u32 mipi_idx;
	u32 xres_div;
	u32 yres_div;

	ifbc_type = IFBC_TYPE_NONE;
	mipi_idx = 0;

	xres_div = g_mipi_ifbc_division[mipi_idx][ifbc_type].xres_div;
	yres_div = g_mipi_ifbc_division[mipi_idx][ifbc_type].yres_div;

	if ((rect->w % xres_div) > 0)
		DRM_ERROR("xres(%d) is not division_h(%d) pixel aligned!\n", rect->w, xres_div);

	if ((rect->h % yres_div) > 0)
		DRM_ERROR("yres(%d) is not division_v(%d) pixel aligned!\n", rect->h, yres_div);

	/*
	 * NOTE: rsp3x && single_mipi CMD mode amended xres_div = 1.5,
	 *  yres_div = 2,
	 * VIDEO mode amended xres_div = 3, yres_div = 1
	 */
	rect->w /= xres_div;
	rect->h /= yres_div;

	return 0;
}

static void init_ldi_pxl_div(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx;
	char __iomem *ldi_base;
	struct drm_display_mode *mode;
	struct drm_display_mode *adj_mode;

	u32 ifbc_type = 0;
	u32 mipi_idx = 0;
	u32 pxl0_div2_gt_en = 0;
	u32 pxl0_div4_gt_en = 0;
	u32 pxl0_divxcfg = 0;
	u32 pxl0_dsi_gt_en = 0;

	ctx = acrtc->ctx;
	if (!ctx) {
		DRM_ERROR("ctx is NULL!\n");
		return;
	}

	mode = &acrtc->base.state->mode;
	adj_mode = &acrtc->base.state->adjusted_mode;

	ldi_base = ctx->base + DSS_LDI0_OFFSET;

	ifbc_type = IFBC_TYPE_NONE;
	mipi_idx = 0;

	pxl0_div2_gt_en = g_mipi_ifbc_division[mipi_idx][ifbc_type].pxl0_div2_gt_en;
	pxl0_div4_gt_en = g_mipi_ifbc_division[mipi_idx][ifbc_type].pxl0_div4_gt_en;
	pxl0_divxcfg = g_mipi_ifbc_division[mipi_idx][ifbc_type].pxl0_divxcfg;
	pxl0_dsi_gt_en = g_mipi_ifbc_division[mipi_idx][ifbc_type].pxl0_dsi_gt_en;

	set_reg(ldi_base + LDI_PXL0_DIV2_GT_EN, pxl0_div2_gt_en, 1, 0);
	set_reg(ldi_base + LDI_PXL0_DIV4_GT_EN, pxl0_div4_gt_en, 1, 0);
	set_reg(ldi_base + LDI_PXL0_GT_EN, 0x1, 1, 0);
	set_reg(ldi_base + LDI_PXL0_DSI_GT_EN, pxl0_dsi_gt_en, 2, 0);
	set_reg(ldi_base + LDI_PXL0_DIVXCFG, pxl0_divxcfg, 3, 0);
}

void init_other(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx;
	char __iomem *dss_base;

	ctx = acrtc->ctx;
	if (!ctx) {
		DRM_ERROR("ctx is NULL!\n");
		return;
	}

	dss_base = ctx->base;

	/**
	 * VESA_CLK_SEL is set to 0 for initial,
	 * 1 is needed only by vesa dual pipe compress
	 */
	set_reg(dss_base + DSS_LDI0_OFFSET + LDI_VESA_CLK_SEL, 0, 1, 0);
}

void init_ldi(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx;
	char __iomem *ldi_base;
	struct drm_display_mode *mode;
	struct drm_display_mode *adj_mode;

	struct dss_rect rect = {0, 0, 0, 0};
	u32 hfp, hbp, hsw, vfp, vbp, vsw;
	u32 vsync_plr = 0;
	u32 hsync_plr = 0;
	u32 pixelclk_plr = 0;
	u32 data_en_plr = 0;

	ctx = acrtc->ctx;
	if (!ctx) {
		DRM_ERROR("ctx is NULL!\n");
		return;
	}

	mode = &acrtc->base.state->mode;
	adj_mode = &acrtc->base.state->adjusted_mode;

	hfp = mode->hsync_start - mode->hdisplay;
	hbp = mode->htotal - mode->hsync_end;
	hsw = mode->hsync_end - mode->hsync_start;
	vfp = mode->vsync_start - mode->vdisplay;
	vbp = mode->vtotal - mode->vsync_end;
	vsw = mode->vsync_end - mode->vsync_start;

	ldi_base = ctx->base + DSS_LDI0_OFFSET;

	rect.x = 0;
	rect.y = 0;
	rect.w = mode->hdisplay;
	rect.h = mode->vdisplay;
	mipi_ifbc_get_rect(&rect);

	init_ldi_pxl_div(acrtc);

	writel(hfp | ((hbp + DSS_WIDTH(hsw)) << 16),
	       ldi_base + LDI_DPI0_HRZ_CTRL0);
	writel(0, ldi_base + LDI_DPI0_HRZ_CTRL1);
	writel(DSS_WIDTH(rect.w), ldi_base + LDI_DPI0_HRZ_CTRL2);
	writel(vfp | (vbp << 16), ldi_base + LDI_VRT_CTRL0);
	writel(DSS_HEIGHT(vsw), ldi_base + LDI_VRT_CTRL1);
	writel(DSS_HEIGHT(rect.h), ldi_base + LDI_VRT_CTRL2);

	writel(vsync_plr | (hsync_plr << 1) | (pixelclk_plr << 2) | (data_en_plr << 3),
	       ldi_base + LDI_PLR_CTRL);

	/* bpp */
	set_reg(ldi_base + LDI_CTRL, acrtc->out_format, 2, 3);
	/* bgr */
	set_reg(ldi_base + LDI_CTRL, acrtc->bgr_fmt, 1, 13);

	/* for ddr pmqos */
	writel(vfp, ldi_base + LDI_VINACT_MSK_LEN);

	/* cmd event sel */
	writel(0x1, ldi_base + LDI_CMD_EVENT_SEL);

	/* for 1Hz LCD and mipi command LCD */
	set_reg(ldi_base + LDI_DSI_CMD_MOD_CTRL, 0x1, 1, 1);

	/* ldi_data_gate(ctx, true); */

	/* normal */
	set_reg(ldi_base + LDI_WORK_MODE, 0x1, 1, 0);

	/* ldi disable */
	set_reg(ldi_base + LDI_CTRL, 0x0, 1, 0);
}

void deinit_ldi(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx;
	char __iomem *ldi_base;

	ctx = acrtc->ctx;
	if (!ctx) {
		DRM_ERROR("ctx is NULL!\n");
		return;
	}

	ldi_base = ctx->base + DSS_LDI0_OFFSET;

	/* ldi disable */
	set_reg(ldi_base + LDI_CTRL, 0, 1, 0);
}

void init_dbuf(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx;
	struct drm_display_mode *mode;
	struct drm_display_mode *adj_mode;
	char __iomem *dbuf_base;

	int sram_valid_num = 0;
	int sram_max_mem_depth = 0;
	int sram_min_support_depth = 0;

	u32 thd_rqos_in = 0;
	u32 thd_rqos_out = 0;
	u32 thd_wqos_in = 0;
	u32 thd_wqos_out = 0;
	u32 thd_cg_in = 0;
	u32 thd_cg_out = 0;
	u32 thd_wr_wait = 0;
	u32 thd_cg_hold = 0;
	u32 thd_flux_req_befdfs_in = 0;
	u32 thd_flux_req_befdfs_out = 0;
	u32 thd_flux_req_aftdfs_in = 0;
	u32 thd_flux_req_aftdfs_out = 0;
	u32 thd_dfs_ok = 0;
	u32 dfs_ok_mask = 0;
	u32 thd_flux_req_sw_en = 1;
	u32 hfp, hbp, hsw, vfp, vbp, vsw;

	int dfs_time = 0;
	int dfs_time_min = 0;
	int depth = 0;
	int dfs_ram = 0;

	ctx = acrtc->ctx;
	if (!ctx) {
		DRM_ERROR("ctx is NULL!\n");
		return;
	}

	mode = &acrtc->base.state->mode;
	adj_mode = &acrtc->base.state->adjusted_mode;

	hfp = mode->hsync_start - mode->hdisplay;
	hbp = mode->htotal - mode->hsync_end;
	hsw = mode->hsync_end - mode->hsync_start;
	vfp = mode->vsync_start - mode->vdisplay;
	vbp = mode->vtotal - mode->vsync_end;
	vsw = mode->vsync_end - mode->vsync_start;

	dbuf_base = ctx->base + DSS_DBUF0_OFFSET;

	if (mode->hdisplay * mode->vdisplay >= RES_4K_PHONE) {
		dfs_time_min = DFS_TIME_MIN_4K;
		dfs_ram = 0x0;
	} else {
		dfs_time_min = DFS_TIME_MIN;
		dfs_ram = 0xF00;
	}

	dfs_time = DFS_TIME;
	depth = DBUF0_DEPTH;

	DRM_DEBUG("dfs_time=%d,\n"
		"adj_mode->clock=%d\n"
		"hsw=%d\n"
		"hbp=%d\n"
		"hfp=%d\n"
		"htotal=%d\n"
		"vfp = %d\n"
		"vbp = %d\n"
		"vsw = %d\n"
		"vtotal=%d\n"
		"mode->hdisplay=%d\n"
		"mode->vdisplay=%d\n",
		dfs_time,
		adj_mode->clock,
		hsw,
		hbp,
		hfp,
		mode->htotal,
		vfp,
		vbp,
		vsw,
		mode->vtotal,
		mode->hdisplay,
		mode->vdisplay);

	/*
	 * int K = 0;
	 * int Tp = 1000000  / adj_mode->clock;
	 * K = (hsw + hbp + mode->hdisplay +
	 *	hfp) / mode->hdisplay;
	 * thd_cg_out = dfs_time / (Tp * K * 6);
	 */
	thd_cg_out = (dfs_time * adj_mode->clock * 1000UL * mode->hdisplay) /
		     (((hsw + hbp + hfp) + mode->hdisplay) * 6 * 1000000UL);

	sram_valid_num = thd_cg_out / depth;
	thd_cg_in = (sram_valid_num + 1) * depth - 1;

	sram_max_mem_depth = (sram_valid_num + 1) * depth;

	thd_rqos_in = thd_cg_out * 85 / 100;
	thd_rqos_out = thd_cg_out;
	thd_flux_req_befdfs_in = GET_FLUX_REQ_IN(sram_max_mem_depth);
	thd_flux_req_befdfs_out = GET_FLUX_REQ_OUT(sram_max_mem_depth);

	sram_min_support_depth = dfs_time_min * mode->hdisplay / (1000000 / 60 / (mode->vdisplay +
				 vbp + vfp + vsw) * (DBUF_WIDTH_BIT / 3 / BITS_PER_BYTE));

	/* thd_flux_req_aftdfs_in   =[(sram_valid_num+1)*depth - 50*HSIZE/((1000000/60/(VSIZE+VFP+VBP+VSW))*6)]/3 */
	thd_flux_req_aftdfs_in = (sram_max_mem_depth - sram_min_support_depth) / 3;
	/* thd_flux_req_aftdfs_out  =  2*[(sram_valid_num+1)* depth - 50*HSIZE/((1000000/60/(VSIZE+VFP+VBP+VSW))*6)]/3 */
	thd_flux_req_aftdfs_out = 2 * (sram_max_mem_depth - sram_min_support_depth) / 3;

	thd_dfs_ok = thd_flux_req_befdfs_in;

	DRM_DEBUG("hdisplay=%d\n"
		"vdisplay=%d\n"
		"sram_valid_num=%d,\n"
		"thd_rqos_in=0x%x\n"
		"thd_rqos_out=0x%x\n"
		"thd_cg_in=0x%x\n"
		"thd_cg_out=0x%x\n"
		"thd_flux_req_befdfs_in=0x%x\n"
		"thd_flux_req_befdfs_out=0x%x\n"
		"thd_flux_req_aftdfs_in=0x%x\n"
		"thd_flux_req_aftdfs_out=0x%x\n"
		"thd_dfs_ok=0x%x\n",
		mode->hdisplay,
		mode->vdisplay,
		sram_valid_num,
		thd_rqos_in,
		thd_rqos_out,
		thd_cg_in,
		thd_cg_out,
		thd_flux_req_befdfs_in,
		thd_flux_req_befdfs_out,
		thd_flux_req_aftdfs_in,
		thd_flux_req_aftdfs_out,
		thd_dfs_ok);

	writel(mode->hdisplay * mode->vdisplay, dbuf_base + DBUF_FRM_SIZE);
	writel(DSS_WIDTH(mode->hdisplay), dbuf_base + DBUF_FRM_HSIZE);
	writel(sram_valid_num, dbuf_base + DBUF_SRAM_VALID_NUM);

	writel((thd_rqos_out << 16) | thd_rqos_in, dbuf_base + DBUF_THD_RQOS);
	writel((thd_wqos_out << 16) | thd_wqos_in, dbuf_base + DBUF_THD_WQOS);
	writel((thd_cg_out << 16) | thd_cg_in, dbuf_base + DBUF_THD_CG);
	writel((thd_cg_hold << 16) | thd_wr_wait, dbuf_base + DBUF_THD_OTHER);
	writel((thd_flux_req_befdfs_out << 16) | thd_flux_req_befdfs_in,
	       dbuf_base + DBUF_THD_FLUX_REQ_BEF);
	writel((thd_flux_req_aftdfs_out << 16) | thd_flux_req_aftdfs_in,
	       dbuf_base + DBUF_THD_FLUX_REQ_AFT);
	writel(thd_dfs_ok, dbuf_base + DBUF_THD_DFS_OK);
	writel((dfs_ok_mask << 1) | thd_flux_req_sw_en,
	       dbuf_base + DBUF_FLUX_REQ_CTRL);

	writel(0x1, dbuf_base + DBUF_DFS_LP_CTRL);
	if (ctx->g_dss_version_tag == FB_ACCEL_KIRIN970)
		writel(dfs_ram, dbuf_base + DBUF_DFS_RAM_MANAGE);
}

void init_dpp(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx;
	struct drm_display_mode *mode;
	struct drm_display_mode *adj_mode;
	char __iomem *dpp_base;
	char __iomem *mctl_sys_base;

	ctx = acrtc->ctx;
	if (!ctx) {
		DRM_ERROR("ctx is NULL!\n");
		return;
	}

	mode = &acrtc->base.state->mode;
	adj_mode = &acrtc->base.state->adjusted_mode;

	dpp_base = ctx->base + DSS_DPP_OFFSET;
	mctl_sys_base = ctx->base + DSS_MCTRL_SYS_OFFSET;

	writel((DSS_HEIGHT(mode->vdisplay) << 16) | DSS_WIDTH(mode->hdisplay),
	       dpp_base + DPP_IMG_SIZE_BEF_SR);
	writel((DSS_HEIGHT(mode->vdisplay) << 16) | DSS_WIDTH(mode->hdisplay),
	       dpp_base + DPP_IMG_SIZE_AFT_SR);
}

void enable_ldi(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx;
	char __iomem *ldi_base;

	ctx = acrtc->ctx;
	if (!ctx) {
		DRM_ERROR("ctx is NULL!\n");
		return;
	}

	ldi_base = ctx->base + DSS_LDI0_OFFSET;

	/* ldi enable */
	set_reg(ldi_base + LDI_CTRL, 0x1, 1, 0);
}

void disable_ldi(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx;
	char __iomem *ldi_base;

	ctx = acrtc->ctx;
	if (!ctx) {
		DRM_ERROR("ctx is NULL!\n");
		return;
	}

	ldi_base = ctx->base + DSS_LDI0_OFFSET;

	/* ldi disable */
	set_reg(ldi_base + LDI_CTRL, 0x0, 1, 0);
}

void dpe_interrupt_clear(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx;
	char __iomem *dss_base;
	u32 clear;

	ctx = acrtc->ctx;
	if (!ctx) {
		DRM_ERROR("ctx is NULL!\n");
		return;
	}

	dss_base = ctx->base;

	clear = ~0;
	writel(clear, dss_base + GLB_CPU_PDP_INTS);
	writel(clear, dss_base + DSS_LDI0_OFFSET + LDI_CPU_ITF_INTS);
	writel(clear, dss_base + DSS_DPP_OFFSET + DPP_INTS);

	writel(clear, dss_base + DSS_DBG_OFFSET + DBG_MCTL_INTS);
	writel(clear, dss_base + DSS_DBG_OFFSET + DBG_WCH0_INTS);
	writel(clear, dss_base + DSS_DBG_OFFSET + DBG_WCH1_INTS);
	writel(clear, dss_base + DSS_DBG_OFFSET + DBG_RCH0_INTS);
	writel(clear, dss_base + DSS_DBG_OFFSET + DBG_RCH1_INTS);
	writel(clear, dss_base + DSS_DBG_OFFSET + DBG_RCH2_INTS);
	writel(clear, dss_base + DSS_DBG_OFFSET + DBG_RCH3_INTS);
	writel(clear, dss_base + DSS_DBG_OFFSET + DBG_RCH4_INTS);
	writel(clear, dss_base + DSS_DBG_OFFSET + DBG_RCH5_INTS);
	writel(clear, dss_base + DSS_DBG_OFFSET + DBG_RCH6_INTS);
	writel(clear, dss_base + DSS_DBG_OFFSET + DBG_RCH7_INTS);
	writel(clear, dss_base + DSS_DBG_OFFSET + DBG_DSS_GLB_INTS);
}

void dpe_interrupt_unmask(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx;
	char __iomem *dss_base;
	u32 unmask;

	ctx = acrtc->ctx;
	if (!ctx) {
		DRM_ERROR("ctx is NULL!\n");
		return;
	}

	dss_base = ctx->base;

	unmask = ~0;
	unmask &= ~(BIT_ITF0_INTS | BIT_MMU_IRPT_NS);
	writel(unmask, dss_base + GLB_CPU_PDP_INT_MSK);

	unmask = ~0;
	unmask &= ~(BIT_VSYNC | BIT_LDI_UNFLOW);

	writel(unmask, dss_base + DSS_LDI0_OFFSET + LDI_CPU_ITF_INT_MSK);
}

void dpe_interrupt_mask(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx;
	char __iomem *dss_base;
	u32 mask;

	ctx = acrtc->ctx;
	if (!ctx) {
		DRM_ERROR("ctx is NULL!\n");
		return;
	}

	dss_base = ctx->base;

	mask = ~0;
	writel(mask, dss_base + GLB_CPU_PDP_INT_MSK);
	writel(mask, dss_base + DSS_LDI0_OFFSET + LDI_CPU_ITF_INT_MSK);
	writel(mask, dss_base + DSS_DPP_OFFSET + DPP_INT_MSK);
	writel(mask, dss_base + DSS_DBG_OFFSET + DBG_DSS_GLB_INT_MSK);
	writel(mask, dss_base + DSS_DBG_OFFSET + DBG_MCTL_INT_MSK);
	writel(mask, dss_base + DSS_DBG_OFFSET + DBG_WCH0_INT_MSK);
	writel(mask, dss_base + DSS_DBG_OFFSET + DBG_WCH1_INT_MSK);
	writel(mask, dss_base + DSS_DBG_OFFSET + DBG_RCH0_INT_MSK);
	writel(mask, dss_base + DSS_DBG_OFFSET + DBG_RCH1_INT_MSK);
	writel(mask, dss_base + DSS_DBG_OFFSET + DBG_RCH2_INT_MSK);
	writel(mask, dss_base + DSS_DBG_OFFSET + DBG_RCH3_INT_MSK);
	writel(mask, dss_base + DSS_DBG_OFFSET + DBG_RCH4_INT_MSK);
	writel(mask, dss_base + DSS_DBG_OFFSET + DBG_RCH5_INT_MSK);
	writel(mask, dss_base + DSS_DBG_OFFSET + DBG_RCH6_INT_MSK);
	writel(mask, dss_base + DSS_DBG_OFFSET + DBG_RCH7_INT_MSK);
}

int dpe_init(struct dss_crtc *acrtc)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *adj_mode;

	mode = &acrtc->base.state->mode;
	adj_mode = &acrtc->base.state->adjusted_mode;

	init_dbuf(acrtc);
	init_dpp(acrtc);
	init_other(acrtc);
	init_ldi(acrtc);

	hisifb_dss_on(acrtc->ctx);
	hisi_dss_mctl_on(acrtc->ctx);

	hisi_dss_mctl_mutex_lock(acrtc->ctx);

	hisi_dss_ovl_base_config(acrtc->ctx, mode->hdisplay, mode->vdisplay);

	hisi_dss_mctl_mutex_unlock(acrtc->ctx);

	enable_ldi(acrtc);

	mdelay(60);

	return 0;
}

int dpe_deinit(struct dss_crtc *acrtc)
{
	deinit_ldi(acrtc);

	return 0;
}

void dpe_check_itf_status(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx;
	char __iomem *mctl_sys_base = NULL;
	int tmp = 0;
	int delay_count = 0;
	bool is_timeout = true;
	int itf_idx = 0;

	ctx = acrtc->ctx;
	if (!ctx) {
		DRM_ERROR("ctx is NULL!\n");
		return;
	}

	itf_idx = 0;
	mctl_sys_base =  ctx->base + DSS_MCTRL_SYS_OFFSET;

	while (1) {
		tmp = readl(mctl_sys_base + MCTL_MOD17_STATUS + itf_idx * 0x4);
		if (((tmp & 0x10) == 0x10) || delay_count > 100) {
			is_timeout = (delay_count > 100) ? true : false;
			delay_count = 0;
			break;
		}
		mdelay(1);
		++delay_count;
	}

	if (is_timeout)
		DRM_DEBUG_DRIVER("mctl_itf%d not in idle status,ints=0x%x !\n", itf_idx, tmp);
}

void dss_inner_clk_pdp_disable(struct dss_hw_ctx *ctx)
{
}

void dss_inner_clk_pdp_enable(struct dss_hw_ctx *ctx)
{
	char __iomem *dss_base;

	if (!ctx) {
		DRM_ERROR("ctx is NULL!\n");
		return;
	}
	dss_base = ctx->base;

	writel(0x00000088, dss_base + DSS_IFBC_OFFSET + IFBC_MEM_CTRL);
	writel(0x00000888, dss_base + DSS_DSC_OFFSET + DSC_MEM_CTRL);
	writel(0x00000008, dss_base + DSS_LDI0_OFFSET + LDI_MEM_CTRL);
	writel(0x00000008, dss_base + DSS_DBUF0_OFFSET + DBUF_MEM_CTRL);
	writel(0x00000008, dss_base + DSS_DPP_DITHER_OFFSET + ctx->dither_mem_ctrl);
}

void dss_inner_clk_common_enable(struct dss_hw_ctx *ctx)
{
	char __iomem *dss_base;

	if (!ctx) {
		DRM_ERROR("NULL Pointer!\n");
		return;
	}

	dss_base = ctx->base;

	/* core/axi/mmbuf */
	writel(0x00000008, dss_base + DSS_CMDLIST_OFFSET + CMD_MEM_CTRL);  /* cmd mem */

	writel(0x00000088,
	       dss_base + DSS_RCH_VG0_SCL_OFFSET + SCF_COEF_MEM_CTRL); /* rch_v0 ,scf mem */
	writel(0x00000008,
	       dss_base + DSS_RCH_VG0_SCL_OFFSET + SCF_LB_MEM_CTRL); /* rch_v0 ,scf mem */
	writel(0x00000008,
	       dss_base + DSS_RCH_VG0_ARSR_OFFSET + ctx->arsr2p_lb_mem_ctrl); /* rch_v0 ,arsr2p mem */
	writel(0x00000008, dss_base + DSS_RCH_VG0_DMA_OFFSET + VPP_MEM_CTRL); /* rch_v0 ,vpp mem */
	writel(0x00000008,
	       dss_base + DSS_RCH_VG0_DMA_OFFSET + DMA_BUF_MEM_CTRL); /* rch_v0 ,dma_buf mem */
	writel(0x00008888, dss_base + DSS_RCH_VG0_DMA_OFFSET + AFBCD_MEM_CTRL); /* rch_v0 ,afbcd mem */

	writel(0x00000088,
	       dss_base + DSS_RCH_VG1_SCL_OFFSET + SCF_COEF_MEM_CTRL); /* rch_v1 ,scf mem */
	writel(0x00000008,
	       dss_base + DSS_RCH_VG1_SCL_OFFSET + SCF_LB_MEM_CTRL); /* rch_v1 ,scf mem */
	writel(0x00000008,
	       dss_base + DSS_RCH_VG1_DMA_OFFSET + DMA_BUF_MEM_CTRL); /* rch_v1 ,dma_buf mem */
	writel(0x00008888, dss_base + DSS_RCH_VG1_DMA_OFFSET + AFBCD_MEM_CTRL); /* rch_v1 ,afbcd mem */

	if (ctx->g_dss_version_tag == FB_ACCEL_KIRIN970) {
		writel(0x88888888,
		       dss_base + DSS_RCH_VG0_DMA_OFFSET + HFBCD_MEM_CTRL);
		writel(0x00000888,
		       dss_base + DSS_RCH_VG0_DMA_OFFSET + HFBCD_MEM_CTRL_1);
		writel(0x88888888,
		       dss_base + DSS_RCH_VG1_DMA_OFFSET + HFBCD_MEM_CTRL);
		writel(0x00000888,
		       dss_base + DSS_RCH_VG1_DMA_OFFSET + HFBCD_MEM_CTRL_1);
	} else {
		writel(0x00000088,
		       dss_base + DSS_RCH_VG2_SCL_OFFSET + SCF_COEF_MEM_CTRL); /* rch_v2 ,scf mem */
		writel(0x00000008,
		       dss_base + DSS_RCH_VG2_SCL_OFFSET + SCF_LB_MEM_CTRL); /* rch_v2 ,scf mem */
	}

	writel(0x00000008,
	       dss_base + DSS_RCH_VG2_DMA_OFFSET + DMA_BUF_MEM_CTRL); /* rch_v2 ,dma_buf mem */

	writel(0x00000088,
	       dss_base + DSS_RCH_G0_SCL_OFFSET + SCF_COEF_MEM_CTRL); /* rch_g0 ,scf mem */
	writel(0x0000008, dss_base + DSS_RCH_G0_SCL_OFFSET + SCF_LB_MEM_CTRL); /* rch_g0 ,scf mem */
	writel(0x00000008,
	       dss_base + DSS_RCH_G0_DMA_OFFSET + DMA_BUF_MEM_CTRL); /* rch_g0 ,dma_buf mem */
	writel(0x00008888, dss_base + DSS_RCH_G0_DMA_OFFSET + AFBCD_MEM_CTRL); /* rch_g0 ,afbcd mem */

	writel(0x00000088,
	       dss_base + DSS_RCH_G1_SCL_OFFSET + SCF_COEF_MEM_CTRL); /* rch_g1 ,scf mem */
	writel(0x0000008, dss_base + DSS_RCH_G1_SCL_OFFSET + SCF_LB_MEM_CTRL); /* rch_g1 ,scf mem */
	writel(0x00000008,
	       dss_base + DSS_RCH_G1_DMA_OFFSET + DMA_BUF_MEM_CTRL); /* rch_g1 ,dma_buf mem */
	writel(0x00008888, dss_base + DSS_RCH_G1_DMA_OFFSET + AFBCD_MEM_CTRL); /* rch_g1 ,afbcd mem */

	writel(0x00000008,
	       dss_base + DSS_RCH_D0_DMA_OFFSET + DMA_BUF_MEM_CTRL); /* rch_d0 ,dma_buf mem */
	writel(0x00008888, dss_base + DSS_RCH_D0_DMA_OFFSET + AFBCD_MEM_CTRL); /* rch_d0 ,afbcd mem */
	writel(0x00000008,
	       dss_base + DSS_RCH_D1_DMA_OFFSET + DMA_BUF_MEM_CTRL); /* rch_d1 ,dma_buf mem */
	writel(0x00000008,
	       dss_base + DSS_RCH_D2_DMA_OFFSET + DMA_BUF_MEM_CTRL); /* rch_d2 ,dma_buf mem */
	writel(0x00000008,
	       dss_base + DSS_RCH_D3_DMA_OFFSET + DMA_BUF_MEM_CTRL); /* rch_d3 ,dma_buf mem */

	writel(0x00000008, dss_base + DSS_WCH0_DMA_OFFSET + DMA_BUF_MEM_CTRL); /* wch0 DMA/AFBCE mem */
	writel(0x00000888, dss_base + DSS_WCH0_DMA_OFFSET + AFBCE_MEM_CTRL); /* wch0 DMA/AFBCE mem */
	writel(0x00000008, dss_base + DSS_WCH0_DMA_OFFSET + ctx->rot_mem_ctrl); /* wch0 rot mem */
	writel(0x00000008, dss_base + DSS_WCH1_DMA_OFFSET + DMA_BUF_MEM_CTRL); /* wch1 DMA/AFBCE mem */
	writel(0x00000888, dss_base + DSS_WCH1_DMA_OFFSET + AFBCE_MEM_CTRL); /* wch1 DMA/AFBCE mem */
	writel(0x00000008, dss_base + DSS_WCH1_DMA_OFFSET + ctx->rot_mem_ctrl); /* wch1 rot mem */
	if (ctx->g_dss_version_tag == FB_ACCEL_KIRIN970) {
		writel(0x00000088,
		       dss_base + DSS_WCH1_DMA_OFFSET + WCH_SCF_COEF_MEM_CTRL);
		writel(0x00000008,
		       dss_base + DSS_WCH1_DMA_OFFSET + WCH_SCF_LB_MEM_CTRL);
		writel(0x02605550, dss_base + GLB_DSS_MEM_CTRL);
	} else {
		writel(0x00000008,
		       dss_base + DSS_WCH2_DMA_OFFSET + DMA_BUF_MEM_CTRL); /* wch2 DMA/AFBCE mem */
		writel(0x00000008,
		       dss_base + DSS_WCH2_DMA_OFFSET + ctx->rot_mem_ctrl); /* wch2 rot mem */
		/* outp32(dss_base + DSS_WCH2_DMA_OFFSET + DMA_BUF_MEM_CTRL, 0x00000008); */
		/* outp32(dss_base + DSS_WCH2_DMA_OFFSET + DMA_BUF_MEM_CTRL, 0x00000008); */
	}
}

int dpe_irq_enable(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx;

	ctx = acrtc->ctx;
	if (!ctx) {
		DRM_ERROR("ctx is NULL!\n");
		return -1;
	}

	if (ctx->irq)
		enable_irq(ctx->irq);

	return 0;
}

int dpe_irq_disable(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx;

	ctx = acrtc->ctx;
	if (!ctx) {
		DRM_ERROR("ctx is NULL!\n");
		return -1;
	}

	if (ctx->irq)
		disable_irq(ctx->irq);

	/* disable_irq_nosync(ctx->irq); */

	return 0;
}

int dpe_common_clk_enable(struct dss_hw_ctx *ctx)
{
	int ret = 0;
	struct clk *clk_tmp = NULL;

	if (!ctx) {
		DRM_ERROR("ctx is NULL point!\n");
		return -EINVAL;
	}

	clk_tmp = ctx->dss_mmbuf_clk;
	if (clk_tmp) {
		ret = clk_prepare(clk_tmp);
		if (ret) {
			DRM_ERROR(" dss_mmbuf_clk clk_prepare failed, error=%d!\n", ret);
			return -EINVAL;
		}

		ret = clk_enable(clk_tmp);
		if (ret) {
			DRM_ERROR(" dss_mmbuf_clk clk_enable failed, error=%d!\n", ret);
			return -EINVAL;
		}
	}

	clk_tmp = ctx->dss_axi_clk;
	if (clk_tmp) {
		ret = clk_prepare(clk_tmp);
		if (ret) {
			DRM_ERROR(" dss_axi_clk clk_prepare failed, error=%d!\n", ret);
			return -EINVAL;
		}

		ret = clk_enable(clk_tmp);
		if (ret) {
			DRM_ERROR(" dss_axi_clk clk_enable failed, error=%d!\n", ret);
			return -EINVAL;
		}
	}

	clk_tmp = ctx->dss_pclk_dss_clk;
	if (clk_tmp) {
		ret = clk_prepare(clk_tmp);
		if (ret) {
			DRM_ERROR(" dss_pclk_dss_clk clk_prepare failed, error=%d!\n", ret);
			return -EINVAL;
		}

		ret = clk_enable(clk_tmp);
		if (ret) {
			DRM_ERROR(" dss_pclk_dss_clk clk_enable failed, error=%d!\n", ret);
			return -EINVAL;
		}
	}

	return 0;
}

int dpe_common_clk_disable(struct dss_hw_ctx *ctx)
{
	struct clk *clk_tmp = NULL;

	if (!ctx) {
		DRM_ERROR("ctx is NULL point!\n");
		return -EINVAL;
	}

	clk_tmp = ctx->dss_pclk_dss_clk;
	if (clk_tmp) {
		clk_disable(clk_tmp);
		clk_unprepare(clk_tmp);
	}

	clk_tmp = ctx->dss_axi_clk;
	if (clk_tmp) {
		clk_disable(clk_tmp);
		clk_unprepare(clk_tmp);
	}

	clk_tmp = ctx->dss_mmbuf_clk;
	if (clk_tmp) {
		clk_disable(clk_tmp);
		clk_unprepare(clk_tmp);
	}

	return 0;
}

int dpe_inner_clk_enable(struct dss_hw_ctx *ctx)
{
	int ret = 0;
	struct clk *clk_tmp = NULL;

	if (!ctx) {
		DRM_ERROR("ctx is NULL point!\n");
		return -EINVAL;
	}

	clk_tmp = ctx->dss_pri_clk;
	if (clk_tmp) {
		ret = clk_prepare(clk_tmp);
		if (ret) {
			DRM_ERROR(" dss_pri_clk clk_prepare failed, error=%d!\n", ret);
			return -EINVAL;
		}

		ret = clk_enable(clk_tmp);
		if (ret) {
			DRM_ERROR(" dss_pri_clk clk_enable failed, error=%d!\n", ret);
			return -EINVAL;
		}
	}

	clk_tmp = ctx->dss_pxl0_clk;
	if (clk_tmp) {
		ret = clk_prepare(clk_tmp);
		if (ret) {
			DRM_ERROR(" dss_pxl0_clk clk_prepare failed, error=%d!\n", ret);
			return -EINVAL;
		}

		ret = clk_enable(clk_tmp);
		if (ret) {
			DRM_ERROR(" dss_pxl0_clk clk_enable failed, error=%d!\n", ret);
			return -EINVAL;
		}
	}

	return 0;
}

int dpe_inner_clk_disable(struct dss_hw_ctx *ctx)
{
	struct clk *clk_tmp = NULL;

	if (!ctx) {
		DRM_ERROR("ctx is NULL point!\n");
		return -EINVAL;
	}

	clk_tmp = ctx->dss_pxl0_clk;
	if (clk_tmp) {
		clk_disable(clk_tmp);
		clk_unprepare(clk_tmp);
	}

	clk_tmp = ctx->dss_pri_clk;
	if (clk_tmp) {
		clk_disable(clk_tmp);
		clk_unprepare(clk_tmp);
	}

	return 0;
}

int dpe_regulator_enable(struct dss_hw_ctx *ctx)
{
	int ret = 0;

	DRM_INFO("enabling DPE regulator\n");
	if (!ctx) {
		DRM_ERROR("NULL ptr.\n");
		return -EINVAL;
	}

	ret = regulator_enable(ctx->dpe_regulator);
	if (ret) {
		DRM_ERROR(" dpe regulator_enable failed, error=%d!\n", ret);
		return -EINVAL;
	}

	DRM_INFO("-.\n");

	return ret;
}

int dpe_regulator_disable(struct dss_hw_ctx *ctx)
{
	int ret = 0;

	if (!ctx) {
		DRM_ERROR("NULL ptr.\n");
		return -EINVAL;
	}

	if (ctx->g_dss_version_tag == FB_ACCEL_KIRIN970) {
		dpe_set_pixel_clk_rate_on_pll0(ctx);
		dpe_set_common_clk_rate_on_pll0(ctx);
	}

	ret = regulator_disable(ctx->dpe_regulator);
	if (ret != 0) {
		DRM_ERROR("dpe regulator_disable failed, error=%d!\n", ret);
		return -EINVAL;
	}

	return ret;
}

int mediacrg_regulator_enable(struct dss_hw_ctx *ctx)
{
	int ret = 0;

	if (!ctx) {
		DRM_ERROR("NULL ptr.\n");
		return -EINVAL;
	}

	/* ret = regulator_enable(ctx->mediacrg_regulator); */
	if (ret)
		DRM_ERROR("mediacrg regulator_enable failed, error=%d!\n", ret);

	return ret;
}

int mediacrg_regulator_disable(struct dss_hw_ctx *ctx)
{
	int ret = 0;

	if (!ctx) {
		DRM_ERROR("NULL ptr.\n");
		return -EINVAL;
	}

	/* ret = regulator_disable(ctx->mediacrg_regulator); */
	if (ret != 0) {
		DRM_ERROR("mediacrg regulator_disable failed, error=%d!\n", ret);
		return -EINVAL;
	}

	return ret;
}

int dpe_set_clk_rate(struct dss_hw_ctx *ctx)
{
	u64 clk_rate;
	int ret = 0;

	if (!ctx) {
		DRM_ERROR("NULL Pointer!\n");
		return -EINVAL;
	}

	clk_rate = DEFAULT_DSS_CORE_CLK_RATE_L1;
	ret = clk_set_rate(ctx->dss_pri_clk, DEFAULT_DSS_CORE_CLK_RATE_L1);
	if (ret < 0) {
		DRM_ERROR("dss_pri_clk clk_set_rate failed, error=%d!\n", ret);
		return -EINVAL;
	}
	DRM_INFO("dss_pri_clk:[%llu]->[%llu].\n",
		 clk_rate, (uint64_t)clk_get_rate(ctx->dss_pri_clk));

#if 0 /* it will be set on dss_ldi_set_mode func */
	ret = clk_set_rate(ctx->dss_pxl0_clk, pinfo->pxl_clk_rate);
	if (ret < 0) {
		DRM_ERROR("fb%d dss_pxl0_clk clk_set_rate(%llu) failed, error=%d!\n",
			  ctx->index, pinfo->pxl_clk_rate, ret);
		if (g_fpga_flag == 0)
			return -EINVAL;
	}

	DRM_INFO("dss_pxl0_clk:[%llu]->[%llu].\n",
		 pinfo->pxl_clk_rate, (uint64_t)clk_get_rate(ctx->dss_pxl0_clk));
#endif

	clk_rate = DEFAULT_DSS_MMBUF_CLK_RATE_L1;
	ret = clk_set_rate(ctx->dss_mmbuf_clk, DEFAULT_DSS_MMBUF_CLK_RATE_L1);
	if (ret < 0) {
		DRM_ERROR("dss_mmbuf clk_set_rate failed, error=%d!\n", ret);
		return -EINVAL;
	}

	DRM_INFO("dss_mmbuf_clk:[%llu]->[%llu].\n",
		 clk_rate, (uint64_t)clk_get_rate(ctx->dss_mmbuf_clk));

	return ret;
}

int dpe_set_pixel_clk_rate_on_pll0(struct dss_hw_ctx *ctx)
{
	int ret;
	u64 clk_rate;

	DRM_INFO("+.\n");
	if (!ctx) {
		DRM_ERROR("NULL Pointer!\n");
		return -EINVAL;
	}

	clk_rate = ctx->pxl0_clk_rate_power_off;
	ret = clk_set_rate(ctx->dss_pxl0_clk, clk_rate);
	if (ret < 0) {
		DRM_ERROR("dss_pxl0_clk clk_set_rate(%llu) failed, error=%d!\n",
			  clk_rate, ret);
		return -EINVAL;
	}
	DRM_INFO("dss_pxl0_clk:[%llu]->[%llu].\n",
		 clk_rate, (uint64_t)clk_get_rate(ctx->dss_pxl0_clk));

	return ret;
}

int dpe_set_common_clk_rate_on_pll0(struct dss_hw_ctx *ctx)
{
	int ret;
	u64 clk_rate;

	DRM_INFO("+.\n");
	if (!ctx) {
		DRM_ERROR("NULL Pointer!\n");
		return -EINVAL;
	}

	clk_rate = ctx->dss_mmbuf_clk_rate_power_off;
	ret = clk_set_rate(ctx->dss_mmbuf_clk, clk_rate);
	if (ret < 0) {
		DRM_ERROR("dss_mmbuf clk_set_rate(%llu) failed, error=%d!\n",
			  clk_rate, ret);
		return -EINVAL;
	}
	DRM_INFO("dss_mmbuf_clk:[%llu]->[%llu].\n",
		 clk_rate, (uint64_t)clk_get_rate(ctx->dss_mmbuf_clk));

	clk_rate = DEFAULT_DSS_CORE_CLK_RATE_POWER_OFF;
	ret = clk_set_rate(ctx->dss_pri_clk, clk_rate);
	if (ret < 0) {
		DRM_ERROR("dss_pri_clk clk_set_rate(%llu) failed, error=%d!\n",
			  clk_rate, ret);
		return -EINVAL;
	}
	DRM_INFO("dss_pri_clk:[%llu]->[%llu].\n",
		 clk_rate, (uint64_t)clk_get_rate(ctx->dss_pri_clk));

	return ret;
}
