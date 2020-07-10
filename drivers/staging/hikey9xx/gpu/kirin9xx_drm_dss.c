/*
 * Hisilicon Hi6220 SoC ADE(Advanced Display Engine)'s crtc&plane driver
 *
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2014-2016 Hisilicon Limited.
 *
 * Author:
 *	Xinliang Liu <z.liuxinliang@hisilicon.com>
 *	Xinliang Liu <xinliang.liu@linaro.org>
 *	Xinwei Kong <kong.kongxinwei@hisilicon.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <video/display_timing.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_modeset_helper_vtables.h>

#include "kirin9xx_drm_drv.h"
#include "kirin9xx_drm_dpe_utils.h"
#if defined (CONFIG_DRM_HISI_KIRIN970)
#include "kirin970_dpe_reg.h"
#else
#include "kirin960_dpe_reg.h"
#endif

//#define DSS_POWER_UP_ON_UEFI

#if defined (CONFIG_DRM_HISI_KIRIN970)
#define DTS_COMP_DSS_NAME "hisilicon,kirin970-dpe"
#else
#define DTS_COMP_DSS_NAME "hisilicon,hi3660-dpe"
#endif

#define DSS_DEBUG	0

static const struct dss_format dss_formats[] = {
	/* 16bpp RGB: */
	{ DRM_FORMAT_RGB565, HISI_FB_PIXEL_FORMAT_RGB_565 },
	{ DRM_FORMAT_BGR565, HISI_FB_PIXEL_FORMAT_BGR_565 },
	/* 32bpp [A]RGB: */
	{ DRM_FORMAT_XRGB8888, HISI_FB_PIXEL_FORMAT_RGBX_8888 },
	{ DRM_FORMAT_XBGR8888, HISI_FB_PIXEL_FORMAT_BGRX_8888 },
	{ DRM_FORMAT_RGBA8888, HISI_FB_PIXEL_FORMAT_RGBA_8888 },
	{ DRM_FORMAT_BGRA8888, HISI_FB_PIXEL_FORMAT_BGRA_8888 },
	/*{ DRM_FORMAT_ARGB8888,  },*/
	/*{ DRM_FORMAT_ABGR8888,  },*/
};

static const u32 channel_formats1[] = {
	DRM_FORMAT_RGB565, DRM_FORMAT_BGR565,
	DRM_FORMAT_XRGB8888, DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBA8888, DRM_FORMAT_BGRA8888
};

u32 dss_get_channel_formats(u8 ch, const u32 **formats)
{
	switch (ch) {
	case DSS_CH1:
		*formats = channel_formats1;
		return ARRAY_SIZE(channel_formats1);
	default:
		DRM_ERROR("no this channel %d\n", ch);
		*formats = NULL;
		return 0;
	}
}

/* convert from fourcc format to dss format */
u32 dss_get_format(u32 pixel_format)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dss_formats); i++)
		if (dss_formats[i].pixel_format == pixel_format)
			return dss_formats[i].dss_format;

	/* not found */
	DRM_ERROR("Not found pixel format!!fourcc_format= %d\n",
		  pixel_format);
	return HISI_FB_PIXEL_FORMAT_UNSUPPORT;
}

/*******************************************************************************
**
*/
int hdmi_ceil(uint64_t a, uint64_t b)
{
	if (b == 0)
		return -1;

	if (a%b != 0) {
		return a/b + 1;
	} else {
		return a/b;
	}
}

int hdmi_pxl_ppll7_init(struct dss_hw_ctx *ctx, uint64_t pixel_clock)
{
	uint64_t refdiv, fbdiv, frac, postdiv1, postdiv2;
	uint64_t vco_min_freq_output = KIRIN970_VCO_MIN_FREQ_OUPUT;
	uint64_t sys_clock_fref = KIRIN970_SYS_19M2;
	uint64_t ppll7_freq_divider;
	uint64_t vco_freq_output;
	uint64_t frac_range = 0x1000000;/*2^24*/
	uint64_t pixel_clock_ori;
	uint64_t pixel_clock_cur;
	uint32_t ppll7ctrl0;
	uint32_t ppll7ctrl1;
	uint32_t ppll7ctrl0_val;
	uint32_t ppll7ctrl1_val;
	int i, ret;
	int ceil_temp;
	int freq_divider_list[22] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
			12, 14, 15, 16, 20, 21, 24,
			25, 30, 36, 42, 49};

	int postdiv1_list[22] = {1, 2, 3, 4, 5, 6, 7, 4, 3, 5,
		   4, 7, 5, 4, 5, 7, 6, 5, 6, 6,
			7, 7};

	int postdiv2_list[22] = {1, 1, 1, 1, 1, 1, 1, 2, 3, 2,
		3, 2, 3, 4, 4, 3, 4, 5, 5, 6,
			6, 7};
	ret = 0;
	postdiv1 = 0;
	postdiv2 = 0;
	if (pixel_clock == 0)
		return -EINVAL;

	if (ctx == NULL) {
		DRM_ERROR("NULL Pointer\n");
		return -EINVAL;
	}

	pixel_clock_ori = pixel_clock;

	if (pixel_clock_ori <= 255000000)
		pixel_clock_cur = pixel_clock * 7;
	else if (pixel_clock_ori <= 415000000)
		pixel_clock_cur = pixel_clock * 5;
	else if (pixel_clock_ori <= 594000000)
		pixel_clock_cur = pixel_clock * 3;
	else {
		DRM_ERROR("Clock don't support!!\n");
		return -EINVAL;
	}

	pixel_clock_cur = pixel_clock_cur / 1000;
	ceil_temp = hdmi_ceil(vco_min_freq_output, pixel_clock_cur);

	if (ceil_temp < 0)
		return -EINVAL;

	ppll7_freq_divider = (uint64_t)ceil_temp;

	for (i = 0; i < 22; i++) {
		if (freq_divider_list[i] >= ppll7_freq_divider) {
			ppll7_freq_divider = freq_divider_list[i];
			postdiv1 = postdiv1_list[i];
			postdiv2 = postdiv2_list[i];
			DRM_INFO("postdiv1=0x%llx, POSTDIV2=0x%llx\n", postdiv1, postdiv2);
			break;
		}
	}

	vco_freq_output = ppll7_freq_divider * pixel_clock_cur;
	if (vco_freq_output == 0)
		return -EINVAL;

	ceil_temp = hdmi_ceil(400000, vco_freq_output);

	if (ceil_temp < 0)
		return -EINVAL;

	refdiv = ((vco_freq_output * ceil_temp) >= 494000) ? 1 : 2;
	DRM_DEBUG("refdiv=0x%llx\n", refdiv);

	fbdiv = (vco_freq_output * ceil_temp) * refdiv / sys_clock_fref;
	DRM_DEBUG("fbdiv=0x%llx\n", fbdiv);

	frac = (uint64_t)(ceil_temp * vco_freq_output - sys_clock_fref / refdiv * fbdiv) * refdiv * frac_range;
	frac = (uint64_t)frac / sys_clock_fref;
	DRM_DEBUG("frac=0x%llx\n", frac);

	ppll7ctrl0 = inp32(ctx->pmctrl_base + MIDIA_PPLL7_CTRL0);
	ppll7ctrl0 &= ~MIDIA_PPLL7_FREQ_DEVIDER_MASK;

	ppll7ctrl0_val = 0x0;
	ppll7ctrl0_val |= (uint32_t)(postdiv2 << 23 | postdiv1 << 20 | fbdiv << 8 | refdiv << 2);
	ppll7ctrl0_val &= MIDIA_PPLL7_FREQ_DEVIDER_MASK;
	ppll7ctrl0 |= ppll7ctrl0_val;

	outp32(ctx->pmctrl_base + MIDIA_PPLL7_CTRL0, ppll7ctrl0);

	ppll7ctrl1 = inp32(ctx->pmctrl_base + MIDIA_PPLL7_CTRL1);
	ppll7ctrl1 &= ~MIDIA_PPLL7_FRAC_MODE_MASK;

	ppll7ctrl1_val = 0x0;
	ppll7ctrl1_val |= (uint32_t)(1 << 25 | 0 << 24 | frac);
	ppll7ctrl1_val &= MIDIA_PPLL7_FRAC_MODE_MASK;
	ppll7ctrl1 |= ppll7ctrl1_val;

	outp32(ctx->pmctrl_base + MIDIA_PPLL7_CTRL1, ppll7ctrl1);

#if 1
	ret = clk_set_rate(ctx->dss_pxl0_clk, 144000000UL);
#else
	/*comfirm ldi1 switch ppll7*/
	if (pixel_clock_ori <= 255000000)
		ret = clk_set_rate(ctx->dss_pxl0_clk, DEFAULT_MIDIA_PPLL7_CLOCK_FREQ/7);
	else if (pixel_clock_ori <= 415000000)
		ret = clk_set_rate(ctx->dss_pxl0_clk, DEFAULT_MIDIA_PPLL7_CLOCK_FREQ/5);
	else if (pixel_clock_ori <= 594000000)
		ret = clk_set_rate(ctx->dss_pxl0_clk, DEFAULT_MIDIA_PPLL7_CLOCK_FREQ/3);
	else {
		DRM_ERROR("Clock don't support!!\n");
		return -EINVAL;
	}
#endif

	if (ret < 0) {
		DRM_ERROR("dss_pxl0_clk clk_set_rate(%llu) failed, error=%d!\n",
			pixel_clock_cur, ret);
	}
	return ret;
}

/*******************************************************************************
 **
 */
static void dss_ldi_set_mode(struct dss_crtc *acrtc)
{
	int ret;
	uint64_t clk_Hz;
	struct dss_hw_ctx *ctx = acrtc->ctx;
	struct drm_display_mode *mode = &acrtc->base.state->mode;
	struct drm_display_mode *adj_mode = &acrtc->base.state->adjusted_mode;

	DRM_INFO("mode->clock(org) = %u\n", mode->clock);
	if (acrtc->ctx->g_dss_version_tag == FB_ACCEL_KIRIN970) {
		if (mode->clock == 148500)
			clk_Hz = 144000 * 1000UL;
		else if (mode->clock == 83496)
			clk_Hz = 84000 * 1000UL;
		else if (mode->clock == 74440)
			clk_Hz = 72000 * 1000UL;
		else if (mode->clock == 74250)
			clk_Hz = 72000 * 1000UL;
		else
			clk_Hz = mode->clock * 1000UL;

		/* Adjust pixel clock for compatibility with 10.1 inch special displays. */
		if (mode->clock == 148500 && mode->width_mm == 532 && mode->height_mm == 299)
			clk_Hz = 152000 * 1000UL;

		DRM_INFO("HDMI real need clock = %llu \n", clk_Hz);
		hdmi_pxl_ppll7_init(ctx, clk_Hz);
		adj_mode->clock = clk_Hz / 1000;
	} else {
		if (mode->clock == 148500)
			clk_Hz = 144000 * 1000UL;
		else if (mode->clock == 83496)
			clk_Hz = 80000 * 1000UL;
		else if (mode->clock == 74440)
			clk_Hz = 72000 * 1000UL;
		else if (mode->clock == 74250)
			clk_Hz = 72000 * 1000UL;
		else
			clk_Hz = mode->clock * 1000UL;

		/*
		 * Success should be guaranteed in mode_valid call back,
		 * so failure shouldn't happen here
		 */
		ret = clk_set_rate(ctx->dss_pxl0_clk, clk_Hz);
		if (ret) {
			DRM_ERROR("failed to set pixel clk %llu Hz (%d)\n", clk_Hz, ret);
		}
		adj_mode->clock = clk_get_rate(ctx->dss_pxl0_clk) / 1000;
	}

	DRM_INFO("dss_pxl0_clk [%llu]->[%lu] \n", clk_Hz, clk_get_rate(ctx->dss_pxl0_clk));

	dpe_init(acrtc);
}

static int dss_power_up(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx = acrtc->ctx;
	int ret = 0;

#if defined (CONFIG_DRM_HISI_KIRIN970)
	mediacrg_regulator_enable(ctx);
	dpe_common_clk_enable(ctx);
	dpe_inner_clk_enable(ctx);
	#ifndef DSS_POWER_UP_ON_UEFI
	dpe_regulator_enable(ctx);
	#endif
	dpe_set_clk_rate(ctx);
#else
	ret = clk_prepare_enable(ctx->dss_pxl0_clk);
	if (ret) {
		DRM_ERROR("failed to enable dss_pxl0_clk (%d)\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(ctx->dss_pri_clk);
	if (ret) {
		DRM_ERROR("failed to enable dss_pri_clk (%d)\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(ctx->dss_pclk_dss_clk);
	if (ret) {
		DRM_ERROR("failed to enable dss_pclk_dss_clk (%d)\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(ctx->dss_axi_clk);
	if (ret) {
		DRM_ERROR("failed to enable dss_axi_clk (%d)\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(ctx->dss_mmbuf_clk);
	if (ret) {
		DRM_ERROR("failed to enable dss_mmbuf_clk (%d)\n", ret);
		return ret;
	}
#endif

	dss_inner_clk_common_enable(ctx);
	dss_inner_clk_pdp_enable(ctx);

	dpe_interrupt_mask(acrtc);
	dpe_interrupt_clear(acrtc);
	dpe_irq_enable(acrtc);
	dpe_interrupt_unmask(acrtc);

	ctx->power_on = true;

	return ret;
}

static void dss_power_down(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx = acrtc->ctx;

	dpe_interrupt_mask(acrtc);
	dpe_irq_disable(acrtc);
	dpe_deinit(acrtc);

	//FIXME:
	dpe_check_itf_status(acrtc);
	dss_inner_clk_pdp_disable(ctx);

	if (ctx->g_dss_version_tag & FB_ACCEL_KIRIN970 ) {
		dpe_regulator_disable(ctx);
		dpe_inner_clk_disable(ctx);
		dpe_common_clk_disable(ctx);
		mediacrg_regulator_disable(ctx);
	} else {
		dpe_regulator_disable(ctx);
		dpe_inner_clk_disable(ctx);
		dpe_common_clk_disable(ctx);
	}

	ctx->power_on = false;
}

static int dss_enable_vblank(struct drm_crtc *crtc)
{
	struct dss_crtc *acrtc = to_dss_crtc(crtc);
	struct dss_hw_ctx *ctx = acrtc->ctx;

	DRM_INFO("%s\n", __func__);
	if (!ctx->power_on) {
		DRM_INFO("Enabling vblank\n");
		(void)dss_power_up(acrtc);
	}

	return 0;
}

static void dss_disable_vblank(struct drm_crtc *crtc)
{
	struct dss_crtc *acrtc = to_dss_crtc(crtc);
	struct dss_hw_ctx *ctx = acrtc->ctx;

	DRM_INFO("%s\n", __func__);
	if (!ctx->power_on) {
		DRM_ERROR("power is down! vblank disable fail\n");
		return;
	}
}

static irqreturn_t dss_irq_handler(int irq, void *data)
{
	struct dss_crtc *acrtc = data;
	struct drm_crtc *crtc = &acrtc->base;
	struct dss_hw_ctx *ctx = acrtc->ctx;
	void __iomem *dss_base = ctx->base;

	u32 isr_s1 = 0;
	u32 isr_s2 = 0;
	u32 mask = 0;

	isr_s1 = inp32(dss_base + GLB_CPU_PDP_INTS);
	isr_s2 = inp32(dss_base + DSS_LDI0_OFFSET + LDI_CPU_ITF_INTS);
	DRM_INFO_ONCE("isr_s1 = 0x%x!\n", isr_s1);
	DRM_INFO_ONCE("isr_s2 = 0x%x!\n", isr_s2);

	outp32(dss_base + DSS_LDI0_OFFSET + LDI_CPU_ITF_INTS, isr_s2);
	outp32(dss_base + GLB_CPU_PDP_INTS, isr_s1);

	isr_s1 &= ~(inp32(dss_base + GLB_CPU_PDP_INT_MSK));
	isr_s2 &= ~(inp32(dss_base + DSS_LDI0_OFFSET + LDI_CPU_ITF_INT_MSK));

	if (isr_s2 & BIT_VACTIVE0_END) {
		ctx->vactive0_end_flag++;
		wake_up_interruptible_all(&ctx->vactive0_end_wq);
	}

	if (isr_s2 & BIT_VSYNC) {
		ctx->vsync_timestamp = ktime_get();
		drm_crtc_handle_vblank(crtc);
	}

	if (isr_s2 & BIT_LDI_UNFLOW) {
		mask = inp32(dss_base + DSS_LDI0_OFFSET + LDI_CPU_ITF_INT_MSK);
		mask |= BIT_LDI_UNFLOW;
		outp32(dss_base + DSS_LDI0_OFFSET + LDI_CPU_ITF_INT_MSK, mask);

		DRM_ERROR("ldi underflow!\n");
	}

	return IRQ_HANDLED;
}

static void dss_crtc_enable(struct drm_crtc *crtc,
			    struct drm_crtc_state *old_state)
{
	struct dss_crtc *acrtc = to_dss_crtc(crtc);
	struct dss_hw_ctx *ctx = acrtc->ctx;
	int ret;

	if (acrtc->enable)
		return;

	if (!ctx->power_on) {
		ret = dss_power_up(acrtc);
		if (ret)
			return;
	}

	acrtc->enable = true;
	drm_crtc_vblank_on(crtc);
}

static void dss_crtc_disable(struct drm_crtc *crtc,
			     struct drm_crtc_state *old_state)
{
	struct dss_crtc *acrtc = to_dss_crtc(crtc);

	if (!acrtc->enable)
		return;

	dss_power_down(acrtc);
	acrtc->enable = false;
	drm_crtc_vblank_off(crtc);
}

static void dss_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct dss_crtc *acrtc = to_dss_crtc(crtc);
	struct dss_hw_ctx *ctx = acrtc->ctx;

	if (!ctx->power_on)
		(void)dss_power_up(acrtc);
	dss_ldi_set_mode(acrtc);
}

static void dss_crtc_atomic_begin(struct drm_crtc *crtc,
				  struct drm_crtc_state *old_state)
{
	struct dss_crtc *acrtc = to_dss_crtc(crtc);
	struct dss_hw_ctx *ctx = acrtc->ctx;

	if (!ctx->power_on)
		(void)dss_power_up(acrtc);
}

static void dss_crtc_atomic_flush(struct drm_crtc *crtc,
				  struct drm_crtc_state *old_state)

{
	struct drm_pending_vblank_event *event = crtc->state->event;

	if (event) {
		crtc->state->event = NULL;

		spin_lock_irq(&crtc->dev->event_lock);
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&crtc->dev->event_lock);
	}

}

static const struct drm_crtc_helper_funcs dss_crtc_helper_funcs = {
	.atomic_enable	= dss_crtc_enable,
	.atomic_disable	= dss_crtc_disable,
	.mode_set_nofb	= dss_crtc_mode_set_nofb,
	.atomic_begin	= dss_crtc_atomic_begin,
	.atomic_flush	= dss_crtc_atomic_flush,
};

static const struct drm_crtc_funcs dss_crtc_funcs = {
	.destroy	= drm_crtc_cleanup,
	.set_config	= drm_atomic_helper_set_config,
	.page_flip	= drm_atomic_helper_page_flip,
	.reset		= drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = dss_enable_vblank,
	.disable_vblank = dss_disable_vblank,
};

static int dss_crtc_init(struct drm_device *dev, struct drm_crtc *crtc,
			 struct drm_plane *plane)
{
	struct kirin_drm_private *priv = dev->dev_private;
	struct device_node *port;
	int ret;

	/* set crtc port so that
	 * drm_of_find_possible_crtcs call works
	 */
	port = of_get_child_by_name(dev->dev->of_node, "port");
	if (!port) {
		DRM_ERROR("no port node found in %s\n",
			  dev->dev->of_node->full_name);
		return -EINVAL;
	}
	of_node_put(port);
	crtc->port = port;

	ret = drm_crtc_init_with_planes(dev, crtc, plane, NULL,
					&dss_crtc_funcs, NULL);
	if (ret) {
		DRM_ERROR("failed to init crtc.\n");
		return ret;
	}

	drm_crtc_helper_add(crtc, &dss_crtc_helper_funcs);
	priv->crtc[drm_crtc_index(crtc)] = crtc;

	return 0;
}

static int dss_plane_atomic_check(struct drm_plane *plane,
				  struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	struct drm_crtc *crtc = state->crtc;
	struct drm_crtc_state *crtc_state;
	u32 src_x = state->src_x >> 16;
	u32 src_y = state->src_y >> 16;
	u32 src_w = state->src_w >> 16;
	u32 src_h = state->src_h >> 16;
	int crtc_x = state->crtc_x;
	int crtc_y = state->crtc_y;
	u32 crtc_w = state->crtc_w;
	u32 crtc_h = state->crtc_h;
	u32 fmt;

	if (!crtc || !fb)
		return 0;

	fmt = dss_get_format(fb->format->format);
	if (fmt == HISI_FB_PIXEL_FORMAT_UNSUPPORT)
		return -EINVAL;

	crtc_state = drm_atomic_get_crtc_state(state->state, crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	if (src_w != crtc_w || src_h != crtc_h) {
		DRM_ERROR("Scale not support!!!\n");
		return -EINVAL;
	}

	if (src_x + src_w > fb->width ||
	    src_y + src_h > fb->height)
		return -EINVAL;

	if (crtc_x < 0 || crtc_y < 0)
		return -EINVAL;

	if (crtc_x + crtc_w > crtc_state->adjusted_mode.hdisplay ||
	    crtc_y + crtc_h > crtc_state->adjusted_mode.vdisplay)
		return -EINVAL;

	return 0;
}

static void dss_plane_atomic_update(struct drm_plane *plane,
				    struct drm_plane_state *old_state)
{
	hisi_fb_pan_display(plane);
}

static void dss_plane_atomic_disable(struct drm_plane *plane,
				     struct drm_plane_state *old_state)
{
	// FIXME: Maybe this?
#if 0
	struct dss_plane *aplane = to_dss_plane(plane);
	struct dss_crtc *acrtc = aplane->acrtc;

	disable_ldi(acrtc);
	hisifb_mctl_sw_clr(acrtc);
#endif
}

static const struct drm_plane_helper_funcs dss_plane_helper_funcs = {
	.atomic_check = dss_plane_atomic_check,
	.atomic_update = dss_plane_atomic_update,
	.atomic_disable = dss_plane_atomic_disable,
};

static struct drm_plane_funcs dss_plane_funcs = {
	.update_plane	= drm_atomic_helper_update_plane,
	.disable_plane	= drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static int dss_plane_init(struct drm_device *dev, struct dss_plane *aplane,
			  enum drm_plane_type type)
{
	const u32 *fmts;
	u32 fmts_cnt;
	int ret = 0;

	/* get properties */
	fmts_cnt = dss_get_channel_formats(aplane->ch, &fmts);
	if (ret)
		return ret;

	ret = drm_universal_plane_init(dev, &aplane->base, 1, &dss_plane_funcs,
				       fmts, fmts_cnt, NULL,
				       type, NULL);
	if (ret) {
		DRM_ERROR("fail to init plane, ch=%d\n", aplane->ch);
		return ret;
	}

	drm_plane_helper_add(&aplane->base, &dss_plane_helper_funcs);

	return 0;
}

static int dss_enable_iommu(struct platform_device *pdev, struct dss_hw_ctx *ctx)
{
	struct device *dev = NULL;

	dev = &pdev->dev;

	/* create iommu domain */
	ctx->mmu_domain = iommu_domain_alloc(dev->bus);
	if (!ctx->mmu_domain) {
		pr_err("iommu_domain_alloc failed!\n");
		return -EINVAL;
	}

	iommu_attach_device(ctx->mmu_domain, dev);

	return 0;
}

static int dss_dts_parse(struct platform_device *pdev, struct dss_hw_ctx *ctx)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = NULL;
	u32 dss_version_tag;
	int ret = 0;

	np = of_find_compatible_node(NULL, NULL, DTS_COMP_DSS_NAME);
	if (!np) {
		DRM_ERROR("NOT FOUND device node %s!\n",
			DTS_COMP_DSS_NAME);
		return -ENXIO;
	}

#if defined (CONFIG_DRM_HISI_KIRIN970)
	ret = of_property_read_u32(np, "dss_version_tag", &dss_version_tag);
	if (ret) {
		DRM_ERROR("failed to get dss_version_tag.\n");
	}
	ctx->g_dss_version_tag = dss_version_tag;
	DRM_INFO("dss_version_tag=0x%x.\n", ctx->g_dss_version_tag);
#else
	ctx->g_dss_version_tag = FB_ACCEL_HI366x;
	DRM_INFO("dss_version_tag=0x%x.\n", ctx->g_dss_version_tag);
#endif

	ctx->base = of_iomap(np, 0);
	if (!(ctx->base)) {
		DRM_ERROR ("failed to get dss base resource.\n");
		return -ENXIO;
	}

	ctx->peri_crg_base  = of_iomap(np, 1);
	if (!(ctx->peri_crg_base)) {
		DRM_ERROR ("failed to get dss peri_crg_base resource.\n");
		return -ENXIO;
	}

	ctx->sctrl_base  = of_iomap(np, 2);
	if (!(ctx->sctrl_base)) {
		DRM_ERROR ("failed to get dss sctrl_base resource.\n");
		return -ENXIO;
	}

	if (ctx->g_dss_version_tag == FB_ACCEL_KIRIN970) {
		ctx->pctrl_base = of_iomap(np, 3);
		if (!(ctx->pctrl_base)) {
			DRM_ERROR ("failed to get dss pctrl_base resource.\n");
			return -ENXIO;
		}
	} else {
		ctx->pmc_base = of_iomap(np, 3);
		if (!(ctx->pmc_base)) {
			DRM_ERROR ("failed to get dss pmc_base resource.\n");
			return -ENXIO;
		}
	}

	ctx->noc_dss_base = of_iomap(np, 4);
	if (!(ctx->noc_dss_base)) {
		DRM_ERROR ("failed to get noc_dss_base resource.\n");
		return -ENXIO;
	}

#if defined (CONFIG_DRM_HISI_KIRIN970)
	ctx->pmctrl_base = of_iomap(np, 5);
	if (!(ctx->pmctrl_base)) {
		DRM_ERROR ("failed to get dss pmctrl_base resource.\n");
		return -ENXIO;
	}

	ctx->media_crg_base = of_iomap(np, 6);
	if (!(ctx->media_crg_base)) {
		DRM_ERROR ("failed to get dss media_crg_base resource.\n");
		return -ENXIO;
	}
#endif

	/* get irq no */
	ctx->irq = irq_of_parse_and_map(np, 0);
	if (ctx->irq <= 0) {
		DRM_ERROR("failed to get irq_pdp resource.\n");
		return -ENXIO;
	}

	DRM_INFO("dss irq = %d. \n", ctx->irq);

#ifndef DSS_POWER_UP_ON_UEFI
#if defined (CONFIG_DRM_HISI_KIRIN970)
	ctx->dpe_regulator = devm_regulator_get(dev, REGULATOR_PDP_NAME);
	if (!ctx->dpe_regulator) {
		DRM_ERROR("failed to get dpe_regulator resource! ret=%d.\n", ret);
		return -ENXIO;
	}

	ctx->mediacrg_regulator = devm_regulator_get(dev, REGULATOR_MEDIA_NAME);
	if (!ctx->mediacrg_regulator) {
		DRM_ERROR("failed to get mediacrg_regulator resource! ret=%d.\n", ret);
		return -ENXIO;
	}
#endif
#endif

	ctx->dss_mmbuf_clk = devm_clk_get(dev, "clk_dss_axi_mm");
	if (!ctx->dss_mmbuf_clk) {
		DRM_ERROR("failed to parse dss_mmbuf_clk\n");
		return -ENODEV;
	}

	ctx->dss_axi_clk = devm_clk_get(dev, "aclk_dss");
	if (!ctx->dss_axi_clk) {
		DRM_ERROR("failed to parse dss_axi_clk\n");
		return -ENODEV;
	}

	ctx->dss_pclk_dss_clk = devm_clk_get(dev, "pclk_dss");
	if (!ctx->dss_pclk_dss_clk) {
		DRM_ERROR("failed to parse dss_pclk_dss_clk\n");
		return -ENODEV;
	}

	ctx->dss_pri_clk = devm_clk_get(dev, "clk_edc0");
	if (!ctx->dss_pri_clk) {
		DRM_ERROR("failed to parse dss_pri_clk\n");
	    return -ENODEV;
	}

	if (ctx->g_dss_version_tag != FB_ACCEL_KIRIN970) {
		ret = clk_set_rate(ctx->dss_pri_clk, DEFAULT_DSS_CORE_CLK_07V_RATE);
		if (ret < 0) {
			DRM_ERROR("dss_pri_clk clk_set_rate(%lu) failed, error=%d!\n",
				DEFAULT_DSS_CORE_CLK_07V_RATE, ret);
			return -EINVAL;
		}

		DRM_INFO("dss_pri_clk:[%lu]->[%llu].\n",
			DEFAULT_DSS_CORE_CLK_07V_RATE, (uint64_t)clk_get_rate(ctx->dss_pri_clk));
	}

	ctx->dss_pxl0_clk = devm_clk_get(dev, "clk_ldi0");
	if (!ctx->dss_pxl0_clk) {
		DRM_ERROR("failed to parse dss_pxl0_clk\n");
		return -ENODEV;
	}

	if (ctx->g_dss_version_tag != FB_ACCEL_KIRIN970) {
		ret = clk_set_rate(ctx->dss_pxl0_clk, DSS_MAX_PXL0_CLK_144M);
		if (ret < 0) {
			DRM_ERROR("dss_pxl0_clk clk_set_rate(%lu) failed, error=%d!\n",
				DSS_MAX_PXL0_CLK_144M, ret);
			return -EINVAL;
		}

		DRM_INFO("dss_pxl0_clk:[%lu]->[%llu].\n",
			DSS_MAX_PXL0_CLK_144M, (uint64_t)clk_get_rate(ctx->dss_pxl0_clk));
	}

	/* regulator enable */
	dss_enable_iommu(pdev, ctx);

	return 0;
}

static int dss_drm_init(struct drm_device *dev)
{
	struct platform_device *pdev = to_platform_device(dev->dev);
	struct dss_data *dss;
	struct dss_hw_ctx *ctx;
	struct dss_crtc *acrtc;
	struct dss_plane *aplane;
	enum drm_plane_type type;
	int ret;
	int i;

	dss = devm_kzalloc(dev->dev, sizeof(*dss), GFP_KERNEL);
	if (!dss) {
		DRM_ERROR("failed to alloc dss_data\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, dss);

	ctx = &dss->ctx;
	acrtc = &dss->acrtc;
	acrtc->ctx = ctx;
	acrtc->out_format = LCD_RGB888;
	acrtc->bgr_fmt = LCD_RGB;

	ret = dss_dts_parse(pdev, ctx);
	if (ret)
		return ret;

	ctx->ion_client = NULL;
	ctx->ion_handle = NULL;
	ctx->screen_base = 0;
	ctx->screen_size = 0;
	ctx->smem_start = 0;

	ctx->vactive0_end_flag = 0;
	init_waitqueue_head(&ctx->vactive0_end_wq);

	/*
	 * plane init
	 * TODO: Now only support primary plane, overlay planes
	 * need to do.
	 */
	for (i = 0; i < DSS_CH_NUM; i++) {
		aplane = &dss->aplane[i];
		aplane->ch = i;
		/*aplane->ctx = ctx;*/
		aplane->acrtc = acrtc;
		type = i == PRIMARY_CH ? DRM_PLANE_TYPE_PRIMARY :
			DRM_PLANE_TYPE_OVERLAY;

		ret = dss_plane_init(dev, aplane, type);
		if (ret)
			return ret;
	}

	/* crtc init */
	ret = dss_crtc_init(dev, &acrtc->base, &dss->aplane[PRIMARY_CH].base);
	if (ret)
		return ret;

	/* vblank irq init */
	ret = devm_request_irq(dev->dev, ctx->irq, dss_irq_handler,
			       IRQF_SHARED, dev->driver->name, acrtc);
	if (ret) {
		DRM_ERROR("fail to  devm_request_irq, ret=%d!", ret);
		return ret;
	}

	disable_irq(ctx->irq);

	return 0;
}

static void dss_drm_cleanup(struct drm_device *dev)
{
	struct platform_device *pdev = to_platform_device(dev->dev);
	struct dss_data *dss = platform_get_drvdata(pdev);
	struct drm_crtc *crtc = &dss->acrtc.base;

	drm_crtc_cleanup(crtc);
}

static int  dss_drm_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct dss_data *dss = platform_get_drvdata(pdev);
	struct drm_crtc *crtc = &dss->acrtc.base;

	dss_crtc_disable(crtc, NULL);

	return 0;
}

static int  dss_drm_resume(struct platform_device *pdev)
{
	struct dss_data *dss = platform_get_drvdata(pdev);
	struct drm_crtc *crtc = &dss->acrtc.base;

	dss_crtc_mode_set_nofb(crtc);
	dss_crtc_enable(crtc, NULL);

	return 0;
}

const struct kirin_dc_ops dss_dc_ops = {
	.init = dss_drm_init,
	.cleanup = dss_drm_cleanup,
	.suspend = dss_drm_suspend,
	.resume = dss_drm_resume,
};
