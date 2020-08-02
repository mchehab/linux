// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2008-2011, Hisilicon Tech. Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>

#include "kirin9xx_drm_dpe_utils.h"
#include "kirin9xx_drm_drv.h"
#include "kirin960_dpe_reg.h"

/*
 * dss_chn_idx
 * DSS_RCHN_D2 = 0,	DSS_RCHN_D3,	DSS_RCHN_V0,	DSS_RCHN_G0,	DSS_RCHN_V1,
 * DSS_RCHN_G1,	DSS_RCHN_D0,	DSS_RCHN_D1,	DSS_WCHN_W0,	DSS_WCHN_W1,
 * DSS_RCHN_V2,   DSS_WCHN_W2,
 */
static const u32 kirin960_g_dss_module_base[DSS_CHN_MAX_DEFINE][MODULE_CHN_MAX] = {
	/* D0 */
	{
	MIF_CH0_OFFSET,
	AIF0_CH0_OFFSET,
	AIF1_CH0_OFFSET,
	MCTL_CTL_MUTEX_RCH0,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH0_FLUSH_EN,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH0_OV_OEN,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH0_STARTY,
	DSS_MCTRL_SYS_OFFSET + MCTL_MOD0_DBG,
	DSS_RCH_D0_DMA_OFFSET,
	DSS_RCH_D0_DFC_OFFSET,
	0,
	0,
	0,
	0,
	0,
	0,
	DSS_RCH_D0_CSC_OFFSET,
	},

	/* D1 */
	{
	MIF_CH1_OFFSET,
	AIF0_CH1_OFFSET,
	AIF1_CH1_OFFSET,
	MCTL_CTL_MUTEX_RCH1,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH1_FLUSH_EN,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH1_OV_OEN,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH1_STARTY,
	DSS_MCTRL_SYS_OFFSET + MCTL_MOD1_DBG,
	DSS_RCH_D1_DMA_OFFSET,
	DSS_RCH_D1_DFC_OFFSET,
	0,
	0,
	0,
	0,
	0,
	0,
	DSS_RCH_D1_CSC_OFFSET,
	},

	/* V0 */
	{
	MIF_CH2_OFFSET,
	AIF0_CH2_OFFSET,
	AIF1_CH2_OFFSET,
	MCTL_CTL_MUTEX_RCH2,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH2_FLUSH_EN,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH2_OV_OEN,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH2_STARTY,
	DSS_MCTRL_SYS_OFFSET + MCTL_MOD2_DBG,
	DSS_RCH_VG0_DMA_OFFSET,
	DSS_RCH_VG0_DFC_OFFSET,
	DSS_RCH_VG0_SCL_OFFSET,
	DSS_RCH_VG0_SCL_LUT_OFFSET,
	DSS_RCH_VG0_ARSR_OFFSET,
	DSS_RCH_VG0_ARSR_LUT_OFFSET,
	DSS_RCH_VG0_POST_CLIP_OFFSET,
	DSS_RCH_VG0_PCSC_OFFSET,
	DSS_RCH_VG0_CSC_OFFSET,
	},

	/* G0 */
	{
	MIF_CH3_OFFSET,
	AIF0_CH3_OFFSET,
	AIF1_CH3_OFFSET,
	MCTL_CTL_MUTEX_RCH3,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH3_FLUSH_EN,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH3_OV_OEN,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH3_STARTY,
	DSS_MCTRL_SYS_OFFSET + MCTL_MOD3_DBG,
	DSS_RCH_G0_DMA_OFFSET,
	DSS_RCH_G0_DFC_OFFSET,
	DSS_RCH_G0_SCL_OFFSET,
	0,
	0,
	0,
	DSS_RCH_G0_POST_CLIP_OFFSET,
	0,
	DSS_RCH_G0_CSC_OFFSET,
	},

	/* V1 */
	{
	MIF_CH4_OFFSET,
	AIF0_CH4_OFFSET,
	AIF1_CH4_OFFSET,
	MCTL_CTL_MUTEX_RCH4,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH4_FLUSH_EN,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH4_OV_OEN,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH4_STARTY,
	DSS_MCTRL_SYS_OFFSET + MCTL_MOD4_DBG,
	DSS_RCH_VG1_DMA_OFFSET,
	DSS_RCH_VG1_DFC_OFFSET,
	DSS_RCH_VG1_SCL_OFFSET,
	DSS_RCH_VG1_SCL_LUT_OFFSET,
	0,
	0,
	DSS_RCH_VG1_POST_CLIP_OFFSET,
	0,
	DSS_RCH_VG1_CSC_OFFSET,
	},

	/* G1 */
	{
	MIF_CH5_OFFSET,
	AIF0_CH5_OFFSET,
	AIF1_CH5_OFFSET,
	MCTL_CTL_MUTEX_RCH5,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH5_FLUSH_EN,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH5_OV_OEN,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH5_STARTY,
	DSS_MCTRL_SYS_OFFSET + MCTL_MOD5_DBG,
	DSS_RCH_G1_DMA_OFFSET,
	DSS_RCH_G1_DFC_OFFSET,
	DSS_RCH_G1_SCL_OFFSET,
	0,
	0,
	0,
	DSS_RCH_G1_POST_CLIP_OFFSET,
	0,
	DSS_RCH_G1_CSC_OFFSET,
	},

	/* D2 */
	{
	MIF_CH6_OFFSET,
	AIF0_CH6_OFFSET,
	AIF1_CH6_OFFSET,
	MCTL_CTL_MUTEX_RCH6,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH6_FLUSH_EN,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH6_OV_OEN,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH6_STARTY,
	DSS_MCTRL_SYS_OFFSET + MCTL_MOD6_DBG,
	DSS_RCH_D2_DMA_OFFSET,
	DSS_RCH_D2_DFC_OFFSET,
	0,
	0,
	0,
	0,
	0,
	0,
	DSS_RCH_D2_CSC_OFFSET,
	},

	/* D3 */
	{
	MIF_CH7_OFFSET,
	AIF0_CH7_OFFSET,
	AIF1_CH7_OFFSET,
	MCTL_CTL_MUTEX_RCH7,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH7_FLUSH_EN,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH7_OV_OEN,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH7_STARTY,
	DSS_MCTRL_SYS_OFFSET + MCTL_MOD7_DBG,
	DSS_RCH_D3_DMA_OFFSET,
	DSS_RCH_D3_DFC_OFFSET,
	0,
	0,
	0,
	0,
	0,
	0,
	DSS_RCH_D3_CSC_OFFSET,
	},

	/* W0 */
	{
	MIF_CH8_OFFSET,
	AIF0_CH8_OFFSET,
	AIF1_CH8_OFFSET,
	MCTL_CTL_MUTEX_WCH0,
	DSS_MCTRL_SYS_OFFSET + MCTL_WCH0_FLUSH_EN,
	DSS_MCTRL_SYS_OFFSET + MCTL_WCH0_OV_IEN,
	0,
	0,
	DSS_WCH0_DMA_OFFSET,
	DSS_WCH0_DFC_OFFSET,
	0,
	0,
	0,
	0,
	0,
	0,
	DSS_WCH0_CSC_OFFSET,
	},

	/* W1 */
	{
	MIF_CH9_OFFSET,
	AIF0_CH9_OFFSET,
	AIF1_CH9_OFFSET,
	MCTL_CTL_MUTEX_WCH1,
	DSS_MCTRL_SYS_OFFSET + MCTL_WCH1_FLUSH_EN,
	DSS_MCTRL_SYS_OFFSET + MCTL_WCH1_OV_IEN,
	0,
	0,
	DSS_WCH1_DMA_OFFSET,
	DSS_WCH1_DFC_OFFSET,
	0,
	0,
	0,
	0,
	0,
	0,
	DSS_WCH1_CSC_OFFSET,
	},
	/* V2 */
	{
	MIF_CH10_OFFSET,
	AIF0_CH11_OFFSET,
	AIF1_CH11_OFFSET,
	MCTL_CTL_MUTEX_RCH8,
	DSS_MCTRL_SYS_OFFSET + MCTL_RCH8_FLUSH_EN,
	0,
	0,
	DSS_MCTRL_SYS_OFFSET + MCTL_MOD8_DBG,
	DSS_RCH_VG2_DMA_OFFSET,
	DSS_RCH_VG2_DFC_OFFSET,
	DSS_RCH_VG2_SCL_OFFSET,
	DSS_RCH_VG2_SCL_LUT_OFFSET,
	0,
	0,
	DSS_RCH_VG2_POST_CLIP_OFFSET,
	0,
	DSS_RCH_VG2_CSC_OFFSET,
	},
	/* W2 */
	{
	MIF_CH11_OFFSET,
	AIF0_CH12_OFFSET,
	AIF1_CH12_OFFSET,
	MCTL_CTL_MUTEX_WCH2,
	DSS_MCTRL_SYS_OFFSET + MCTL_WCH2_FLUSH_EN,
	0,
	0,
	0,
	DSS_WCH2_DMA_OFFSET,
	DSS_WCH2_DFC_OFFSET,
	0,
	0,
	0,
	0,
	0,
	0,
	DSS_WCH2_CSC_OFFSET,
	},
};

static const u32 kirin960_g_dss_module_ovl_base[DSS_MCTL_IDX_MAX][MODULE_OVL_MAX] = {
	{DSS_OVL0_OFFSET,
	DSS_MCTRL_CTL0_OFFSET},

	{DSS_OVL1_OFFSET,
	DSS_MCTRL_CTL1_OFFSET},

	{DSS_OVL2_OFFSET,
	DSS_MCTRL_CTL2_OFFSET},

	{DSS_OVL3_OFFSET,
	DSS_MCTRL_CTL3_OFFSET},

	{0,
	DSS_MCTRL_CTL4_OFFSET},

	{0,
	DSS_MCTRL_CTL5_OFFSET},
};

/*SCF_LUT_CHN coef_idx*/
static const int kirin960_g_scf_lut_chn_coef_idx[DSS_CHN_MAX_DEFINE] = {
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
};

static const u32 kirin960_g_dss_module_cap[DSS_CHN_MAX_DEFINE][MODULE_CAP_MAX] = {
	/* D2 */
	{0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1},
	/* D3 */
	{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
	/* V0 */
	{0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1},
	/* G0 */
	{0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0},
	/* V1 */
	{0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1},
	/* G1 */
	{0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0},
	/* D0 */
	{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
	/* D1 */
	{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},

	/* W0 */
	{1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1},
	/* W1 */
	{1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1},

	/* V2 */
	{0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1},
	/* W2 */
	{1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1},
};

/* number of smrx idx for each channel */
static const u32 kirin960_g_dss_chn_sid_num[DSS_CHN_MAX_DEFINE] = {
	4, 1, 4, 4, 4, 4, 1, 1, 3, 3, 3, 2
};

/* start idx of each channel */
/* smrx_idx = g_dss_smmu_smrx_idx[chn_idx] + (0 ~ g_dss_chn_sid_num[chn_idx]) */
static const u32 kirin960_g_dss_smmu_smrx_idx[DSS_CHN_MAX_DEFINE] = {
	0, 4, 5, 9, 13, 17, 21, 22, 26, 29, 23, 32
};

static const u32 kirin960_g_dss_mif_sid_map[DSS_CHN_MAX] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

void kirin960_dpe_defs(struct dss_hw_ctx *ctx)
{
	memcpy(&ctx->g_dss_module_base, &kirin960_g_dss_module_base,
		sizeof(kirin960_g_dss_module_base));
	memcpy(&ctx->g_dss_module_ovl_base, &kirin960_g_dss_module_ovl_base,
		sizeof(kirin960_g_dss_module_ovl_base));
	memcpy(&ctx->g_scf_lut_chn_coef_idx, &kirin960_g_scf_lut_chn_coef_idx,
		sizeof(kirin960_g_scf_lut_chn_coef_idx));
	memcpy(&ctx->g_dss_module_cap, &kirin960_g_dss_module_cap,
		sizeof(kirin960_g_dss_module_cap));
	memcpy(&ctx->g_dss_chn_sid_num, &kirin960_g_dss_chn_sid_num,
		sizeof(kirin960_g_dss_chn_sid_num));
	memcpy(&ctx->g_dss_smmu_smrx_idx, &kirin960_g_dss_smmu_smrx_idx,
		sizeof(kirin960_g_dss_smmu_smrx_idx));

	ctx->smmu_offset = DSS_SMMU_OFFSET;
	ctx->afbc_header_addr_align = AFBC_HEADER_ADDR_ALIGN;
	ctx->dss_mmbuf_clk_rate_power_off = DEFAULT_DSS_MMBUF_CLK_RATE_POWER_OFF;
	ctx->rot_mem_ctrl = ROT_MEM_CTRL;
	ctx->dither_mem_ctrl = DITHER_MEM_CTRL;
	ctx->arsr2p_lb_mem_ctrl = ARSR2P_LB_MEM_CTRL;
	ctx->pxl0_clk_rate_power_off = DEFAULT_DSS_PXL0_CLK_RATE_POWER_OFF;
}
