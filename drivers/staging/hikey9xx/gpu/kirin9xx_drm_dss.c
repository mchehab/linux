// SPDX-License-Identifier: GPL-2.0
/*
 * Hisilicon Hi6220 SoC ADE(Advanced Display Engine)'s crtc&plane driver
 *
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2014-2016 Hisilicon Limited.
 * Copyright (c) 2014-2020, Huawei Technologies Co., Ltd
 *
 * Author:
 *	Xinliang Liu <z.liuxinliang@hisilicon.com>
 *	Xinliang Liu <xinliang.liu@linaro.org>
 *	Xinwei Kong <kong.kongxinwei@hisilicon.com>
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
#include "kirin9xx_dpe.h"

static const struct dss_format dss_formats[] = {
	/* 16bpp RGB: */
	{ DRM_FORMAT_RGB565, HISI_FB_PIXEL_FORMAT_RGB_565 },
	{ DRM_FORMAT_BGR565, HISI_FB_PIXEL_FORMAT_BGR_565 },
	/* 32bpp [A]RGB: */
	{ DRM_FORMAT_XRGB8888, HISI_FB_PIXEL_FORMAT_RGBX_8888 },
	{ DRM_FORMAT_XBGR8888, HISI_FB_PIXEL_FORMAT_BGRX_8888 },
	{ DRM_FORMAT_RGBA8888, HISI_FB_PIXEL_FORMAT_RGBA_8888 },
	{ DRM_FORMAT_BGRA8888, HISI_FB_PIXEL_FORMAT_BGRA_8888 },
	{ DRM_FORMAT_ARGB8888, HISI_FB_PIXEL_FORMAT_RGBA_8888 },
	{ DRM_FORMAT_ABGR8888, HISI_FB_PIXEL_FORMAT_BGRA_8888 },
};

static const u32 channel_formats1[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
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

/*****************************************************************************/

int hdmi_pxl_ppll7_init(struct dss_hw_ctx *ctx, u64 pixel_clock)
{
	u64 vco_min_freq_output = KIRIN970_VCO_MIN_FREQ_OUTPUT;
	u64 refdiv, fbdiv, frac, postdiv1 = 0, postdiv2 = 0;
	u64 dss_pxl0_clk = 7 * 144000000UL;
	u64 sys_clock_fref = KIRIN970_SYS_19M2;
	u64 ppll7_freq_divider;
	u64 vco_freq_output;
	u64 frac_range = 0x1000000; /* 2^24 */
	u64 pixel_clock_ori;
	u64 pixel_clock_cur;
	u32 ppll7ctrl0;
	u32 ppll7ctrl1;
	u32 ppll7ctrl0_val;
	u32 ppll7ctrl1_val;
	int ceil_temp;
	int i, ret;
	const int freq_divider_list[22] = {
		1,  2,  3,  4,  5,  6,  7,  8,
		9, 10, 12, 14, 15, 16, 20, 21,
		24, 25, 30, 36, 42, 49
	};
	const int postdiv1_list[22] = {
		1, 2, 3, 4, 5, 6, 7, 4, 3, 5,
		4, 7, 5, 4, 5, 7, 6, 5, 6, 6,
		7, 7
	};
	const int postdiv2_list[22] = {
		1, 1, 1, 1, 1, 1, 1, 2, 3, 2,
		3, 2, 3, 4, 4, 3, 4, 5, 5, 6,
		6, 7
	};

	if (!pixel_clock) {
		DRM_ERROR("Pixel clock can't be zero!\n");
		return -EINVAL;
	}

	pixel_clock_ori = pixel_clock;
	pixel_clock_cur = pixel_clock_ori;

	if (pixel_clock_ori <= 255000000) {
		pixel_clock_cur *= 7;
		dss_pxl0_clk /= 7;
	} else if (pixel_clock_ori <= 415000000) {
		pixel_clock_cur *= 5;
		dss_pxl0_clk /= 5;
	} else if (pixel_clock_ori <= 594000000) {
		pixel_clock_cur *= 3;
		dss_pxl0_clk /= 3;
	} else {
		DRM_ERROR("Clock not supported!\n");
		return -EINVAL;
	}

	pixel_clock_cur = pixel_clock_cur / 1000;
	if (!pixel_clock_cur) {
		DRM_ERROR("pixel_clock_cur can't be zero!\n");
		return -EINVAL;
	}

	ceil_temp = DIV_ROUND_UP(vco_min_freq_output, pixel_clock_cur);

	ppll7_freq_divider = (u64)ceil_temp;

	for (i = 0; i < ARRAY_SIZE(freq_divider_list); i++) {
		if (freq_divider_list[i] >= ppll7_freq_divider) {
			ppll7_freq_divider = freq_divider_list[i];
			postdiv1 = postdiv1_list[i];
			postdiv2 = postdiv2_list[i];
			break;
		}
	}

	if (i == ARRAY_SIZE(freq_divider_list)) {
		DRM_ERROR("Can't find a valid setting for PLL7!\n");
		return -EINVAL;
	}

	vco_freq_output = ppll7_freq_divider * pixel_clock_cur;
	if (!vco_freq_output) {
		DRM_ERROR("Can't find a valid setting for VCO_FREQ!\n");
		return -EINVAL;
	}

	ceil_temp = DIV_ROUND_UP(400000, vco_freq_output);

	refdiv = ((vco_freq_output * ceil_temp) >= 494000) ? 1 : 2;
	fbdiv = (vco_freq_output * ceil_temp) * refdiv / sys_clock_fref;

	frac = (u64)(ceil_temp * vco_freq_output - sys_clock_fref / refdiv * fbdiv) * refdiv * frac_range;
	frac = (u64)frac / sys_clock_fref;

	ppll7ctrl0 = readl(ctx->pmctrl_base + MIDIA_PPLL7_CTRL0);
	ppll7ctrl0 &= ~MIDIA_PPLL7_FREQ_DEVIDER_MASK;

	ppll7ctrl0_val = 0x0;
	ppll7ctrl0_val |= (u32)(postdiv2 << 23 | postdiv1 << 20 | fbdiv << 8 | refdiv << 2);
	ppll7ctrl0_val &= MIDIA_PPLL7_FREQ_DEVIDER_MASK;
	ppll7ctrl0 |= ppll7ctrl0_val;

	writel(ppll7ctrl0, ctx->pmctrl_base + MIDIA_PPLL7_CTRL0);

	ppll7ctrl1 = readl(ctx->pmctrl_base + MIDIA_PPLL7_CTRL1);
	ppll7ctrl1 &= ~MIDIA_PPLL7_FRAC_MODE_MASK;

	ppll7ctrl1_val = 0x0;
	ppll7ctrl1_val |= (u32)(1 << 25 | 0 << 24 | frac);
	ppll7ctrl1_val &= MIDIA_PPLL7_FRAC_MODE_MASK;
	ppll7ctrl1 |= ppll7ctrl1_val;

	writel(ppll7ctrl1, ctx->pmctrl_base + MIDIA_PPLL7_CTRL1);

	DRM_INFO("PLL7 set to (0x%0x, 0x%0x)\n", ppll7ctrl0, ppll7ctrl1);

	ret = clk_set_rate(ctx->dss_pxl0_clk, dss_pxl0_clk);
	if (ret < 0)
		DRM_ERROR("%s: clk_set_rate(dss_pxl0_clk, %llu) failed: %d!\n",
			  __func__, dss_pxl0_clk, ret);
	else
		DRM_INFO("dss_pxl0_clk:[%llu]->[%lu].\n",
			 dss_pxl0_clk, clk_get_rate(ctx->dss_pxl0_clk));

	return ret;
}

static u64 dss_calculate_clock(struct dss_crtc *acrtc,
			       const struct drm_display_mode *mode)
{
	u64 clk_Hz;

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
	}

	return clk_Hz;
}

static void dss_ldi_set_mode(struct dss_crtc *acrtc)
{
	struct drm_display_mode *adj_mode = &acrtc->base.state->adjusted_mode;
	struct drm_display_mode *mode = &acrtc->base.state->mode;
	struct dss_hw_ctx *ctx = acrtc->ctx;
	u32 clock = mode->clock;
	u64 clk_Hz;
	int ret;

	clk_Hz = dss_calculate_clock(acrtc, mode);

	DRM_INFO("Requested clock %u kHz, setting to %llu kHz\n",
		 clock, clk_Hz / 1000);

	if (acrtc->ctx->g_dss_version_tag == FB_ACCEL_KIRIN970) {
		ret = hdmi_pxl_ppll7_init(ctx, clk_Hz);
	} else {
		ret = clk_set_rate(ctx->dss_pxl0_clk, clk_Hz);
		if (!ret) {
			clk_Hz = clk_get_rate(ctx->dss_pxl0_clk);
			DRM_INFO("dss_pxl0_clk:[%llu]->[%lu].\n",
				 clk_Hz, clk_get_rate(ctx->dss_pxl0_clk));
		}
	}

	if (ret)
		DRM_ERROR("failed to set pixel clock\n");
	else
		adj_mode->clock = clk_Hz / 1000;

	dpe_init(acrtc);
}

static int dss_power_up(struct dss_crtc *acrtc)
{
	struct dss_hw_ctx *ctx = acrtc->ctx;
	int ret = 0;

	if (ctx->g_dss_version_tag == FB_ACCEL_KIRIN970) {
		dpe_common_clk_enable(ctx);
		dpe_inner_clk_enable(ctx);
		dpe_set_clk_rate(ctx);
	} else {
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
	}

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

	dpe_check_itf_status(acrtc);
	dss_inner_clk_pdp_disable(ctx);

	dpe_inner_clk_disable(ctx);
	dpe_common_clk_disable(ctx);

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

	isr_s1 = readl(dss_base + GLB_CPU_PDP_INTS);
	isr_s2 = readl(dss_base + DSS_LDI0_OFFSET + LDI_CPU_ITF_INTS);
	DRM_INFO_ONCE("isr_s1 = 0x%x!\n", isr_s1);
	DRM_INFO_ONCE("isr_s2 = 0x%x!\n", isr_s2);

	writel(isr_s2, dss_base + DSS_LDI0_OFFSET + LDI_CPU_ITF_INTS);
	writel(isr_s1, dss_base + GLB_CPU_PDP_INTS);

	isr_s1 &= ~(readl(dss_base + GLB_CPU_PDP_INT_MSK));
	isr_s2 &= ~(readl(dss_base + DSS_LDI0_OFFSET + LDI_CPU_ITF_INT_MSK));

	if (isr_s2 & BIT_VSYNC) {
		ctx->vsync_timestamp = ktime_get();
		drm_crtc_handle_vblank(crtc);
	}

	if (isr_s2 & BIT_LDI_UNFLOW) {
		mask = readl(dss_base + DSS_LDI0_OFFSET + LDI_CPU_ITF_INT_MSK);
		mask |= BIT_LDI_UNFLOW;
		writel(mask, dss_base + DSS_LDI0_OFFSET + LDI_CPU_ITF_INT_MSK);

		DRM_ERROR("ldi underflow!\n");
	}

	return IRQ_HANDLED;
}

static bool dss_crtc_mode_fixup(struct drm_crtc *crtc,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adj_mode)
{
	struct dss_crtc *acrtc = to_dss_crtc(crtc);
	struct dss_hw_ctx *ctx = acrtc->ctx;
	u64 clk_Hz;

	/* Check if clock is too high */
	if (mode->clock > 594000)
		return false;
	/*
	 * FIXME: the code should, instead, do some calculus instead of
	 * hardcoding the modes. Clearly, there's something missing at
	 * dss_calculate_clock()
	 */

	/*
	 * HACK: reports at Hikey 970 mailing lists with the downstream
	 * Official Linaro 4.9 driver seem to indicate that some monitor
	 * modes aren't properly set. There, this hack was added.
	 *
	 * On my monitors, this wasn't needed, but better to keep this
	 * code here, together with this notice, just in case.
	 */
	if ((mode->hdisplay    == 1920 && mode->vdisplay == 1080 && mode->clock == 148500)
	    || (mode->hdisplay == 1920 && mode->vdisplay == 1080 && mode->clock == 148352)
	    || (mode->hdisplay == 1920 && mode->vdisplay == 1080 && mode->clock ==  80192)
	    || (mode->hdisplay == 1920 && mode->vdisplay == 1080 && mode->clock ==  74250)
	    || (mode->hdisplay == 1920 && mode->vdisplay == 1080 && mode->clock ==  61855)
	    || (mode->hdisplay == 1680 && mode->vdisplay == 1050 && mode->clock == 147116)
	    || (mode->hdisplay == 1680 && mode->vdisplay == 1050 && mode->clock == 146250)
	    || (mode->hdisplay == 1680 && mode->vdisplay == 1050 && mode->clock == 144589)
	    || (mode->hdisplay == 1600 && mode->vdisplay == 1200 && mode->clock == 160961)
	    || (mode->hdisplay == 1600 && mode->vdisplay == 900  && mode->clock == 118963)
	    || (mode->hdisplay == 1440 && mode->vdisplay == 900  && mode->clock == 126991)
	    || (mode->hdisplay == 1280 && mode->vdisplay == 1024 && mode->clock == 128946)
	    || (mode->hdisplay == 1280 && mode->vdisplay == 1024 && mode->clock ==  98619)
	    || (mode->hdisplay == 1280 && mode->vdisplay == 960  && mode->clock == 102081)
	    || (mode->hdisplay == 1280 && mode->vdisplay == 800  && mode->clock ==  83496)
	    || (mode->hdisplay == 1280 && mode->vdisplay == 720  && mode->clock ==  74440)
	    || (mode->hdisplay == 1280 && mode->vdisplay == 720  && mode->clock ==  74250)
	    || (mode->hdisplay == 1024 && mode->vdisplay == 768  && mode->clock ==  78800)
	    || (mode->hdisplay == 1024 && mode->vdisplay == 768  && mode->clock ==  75000)
	    || (mode->hdisplay == 1024 && mode->vdisplay == 768  && mode->clock ==  81833)
	    || (mode->hdisplay == 800  && mode->vdisplay == 600  && mode->clock ==  48907)
	    || (mode->hdisplay == 800  && mode->vdisplay == 600  && mode->clock ==  40000)
	    || (mode->hdisplay == 800  && mode->vdisplay == 480  && mode->clock ==  32000)
	   ) {
		clk_Hz = dss_calculate_clock(acrtc, mode);

		/*
		 * On Kirin970, PXL0 clock is set to a const value,
		 * independently of the pixel clock.
		 */
		if (acrtc->ctx->g_dss_version_tag != FB_ACCEL_KIRIN970)
			clk_Hz = clk_round_rate(ctx->dss_pxl0_clk, mode->clock * 1000);

		adj_mode->clock = clk_Hz / 1000;

		return true;
	}

	return false;
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
	.mode_fixup	= dss_crtc_mode_fixup,
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
	struct kirin_drm_private *priv = to_drm_private(dev);
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
	struct drm_plane_state *state = plane->state;

	if (!state->fb) {
		state->visible = false;
		return;
	}

	hisi_fb_pan_display(plane);
}

static void dss_plane_atomic_disable(struct drm_plane *plane,
				     struct drm_plane_state *old_state)
{
	/* FIXME: Maybe this? */
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
#if 0
/*
 * FIXME:
 *
 * Right now, the IOMMU support is actually disabled. See the caller of
 * hisi_dss_smmu_config(). Yet, if we end enabling it, this should be
 * ported to use io-pgtable directly.
 */
	struct device *dev = NULL;

	dev = &pdev->dev;

	/* create iommu domain */
	ctx->mmu_domain = iommu_domain_alloc(dev->bus);
	if (!ctx->mmu_domain) {
		pr_err("iommu_domain_alloc failed!\n");
		return -EINVAL;
	}

	iommu_attach_device(ctx->mmu_domain, dev);
#endif
	return 0;
}

static int dss_dts_parse(struct platform_device *pdev, struct dss_hw_ctx *ctx)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = NULL;
	const char *compatible;
	int ret = 0;

	if (ctx->g_dss_version_tag == FB_ACCEL_KIRIN970)
		compatible = "hisilicon,kirin970-dpe";
	else
		compatible = "hisilicon,hi3660-dpe";

	np = of_find_compatible_node(NULL, NULL, compatible);
	if (!np) {
		DRM_ERROR("NOT FOUND device node %s!\n", compatible);
		return -ENXIO;
	}

	/* Initialize version-specific data */
	if (ctx->g_dss_version_tag == FB_ACCEL_HI366x)
		kirin960_dpe_defs(ctx);
	else
		kirin970_dpe_defs(ctx);

	ctx->base = of_iomap(np, 0);
	if (!(ctx->base)) {
		DRM_ERROR("failed to get dss base resource.\n");
		return -ENXIO;
	}

	ctx->peri_crg_base  = of_iomap(np, 1);
	if (!(ctx->peri_crg_base)) {
		DRM_ERROR("failed to get dss peri_crg_base resource.\n");
		return -ENXIO;
	}

	ctx->sctrl_base  = of_iomap(np, 2);
	if (!(ctx->sctrl_base)) {
		DRM_ERROR("failed to get dss sctrl_base resource.\n");
		return -ENXIO;
	}

	if (ctx->g_dss_version_tag == FB_ACCEL_KIRIN970) {
		ctx->pctrl_base = of_iomap(np, 3);
		if (!(ctx->pctrl_base)) {
			DRM_ERROR("failed to get dss pctrl_base resource.\n");
			return -ENXIO;
		}
	} else {
		ctx->pmc_base = of_iomap(np, 3);
		if (!(ctx->pmc_base)) {
			DRM_ERROR("failed to get dss pmc_base resource.\n");
			return -ENXIO;
		}
	}

	ctx->noc_dss_base = of_iomap(np, 4);
	if (!(ctx->noc_dss_base)) {
		DRM_ERROR("failed to get noc_dss_base resource.\n");
		return -ENXIO;
	}

	if (ctx->g_dss_version_tag == FB_ACCEL_KIRIN970) {
		ctx->pmctrl_base = of_iomap(np, 5);
		if (!(ctx->pmctrl_base)) {
			DRM_ERROR("failed to get dss pmctrl_base resource.\n");
			return -ENXIO;
		}

		ctx->media_crg_base = of_iomap(np, 6);
		if (!(ctx->media_crg_base)) {
			DRM_ERROR("failed to get dss media_crg_base resource.\n");
			return -ENXIO;
		}
	}

	/* get irq no */
	ctx->irq = irq_of_parse_and_map(np, 0);
	if (ctx->irq <= 0) {
		DRM_ERROR("failed to get irq_pdp resource.\n");
		return -ENXIO;
	}

	DRM_INFO("dss irq = %d.\n", ctx->irq);

	if (ctx->g_dss_version_tag == FB_ACCEL_KIRIN970) {
		ctx->dpe_regulator = devm_regulator_get(dev, REGULATOR_PDP_NAME);
		if (!ctx->dpe_regulator) {
			DRM_ERROR("failed to get dpe_regulator resource! ret=%d.\n", ret);
			return -ENXIO;
		}
	}

	ctx->dss_mmbuf_clk = devm_clk_get(dev, "clk_dss_axi_mm");
	ret = PTR_ERR_OR_ZERO(ctx->dss_mmbuf_clk);
	if (ret == -EPROBE_DEFER) {
		return ret;
	} else if (ret) {
		DRM_ERROR("failed to parse dss_mmbuf_clk: %d\n", ret);
		return ret;
	}

	ctx->dss_axi_clk = devm_clk_get(dev, "aclk_dss");
	ret = PTR_ERR_OR_ZERO(ctx->dss_axi_clk);
	if (ret == -EPROBE_DEFER) {
		return ret;
	} else if (ret) {
		DRM_ERROR("failed to parse dss_axi_clk: %d\n", ret);
		return ret;
	}

	ctx->dss_pclk_dss_clk = devm_clk_get(dev, "pclk_dss");
	ret = PTR_ERR_OR_ZERO(ctx->dss_pclk_dss_clk);
	if (ret == -EPROBE_DEFER) {
		return ret;
	} else if (ret) {
		DRM_ERROR("failed to parse dss_pclk_dss_clk: %d\n", ret);
		return ret;
	}

	ctx->dss_pri_clk = devm_clk_get(dev, "clk_edc0");
	ret = PTR_ERR_OR_ZERO(ctx->dss_pri_clk);
	if (ret == -EPROBE_DEFER) {
		return ret;
	} else if (ret) {
		DRM_ERROR("failed to parse dss_pri_clk: %d\n", ret);
		return ret;
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
	ret = PTR_ERR_OR_ZERO(ctx->dss_pri_clk);
	if (ret == -EPROBE_DEFER) {
		return ret;
	} else if (ret) {
		DRM_ERROR("failed to parse dss_pxl0_clk: %d\n", ret);
		return ret;
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

	/* enable IOMMU */
	dss_enable_iommu(pdev, ctx);

	return 0;
}

int dss_drm_init(struct drm_device *dev, u32 g_dss_version_tag)
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
	ctx->g_dss_version_tag = g_dss_version_tag;

	ret = dss_dts_parse(pdev, ctx);
	if (ret)
		return ret;

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
		/* aplane->ctx = ctx; */
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

void kirin9xx_dss_drm_cleanup(struct drm_device *dev)
{
	struct platform_device *pdev = to_platform_device(dev->dev);
	struct dss_data *dss = platform_get_drvdata(pdev);
	struct drm_crtc *crtc = &dss->acrtc.base;

	drm_crtc_cleanup(crtc);
}

int kirin9xx_dss_drm_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct dss_data *dss = platform_get_drvdata(pdev);
	struct drm_crtc *crtc = &dss->acrtc.base;

	dss_crtc_disable(crtc, NULL);

	return 0;
}

int kirin9xx_dss_drm_resume(struct platform_device *pdev)
{
	struct dss_data *dss = platform_get_drvdata(pdev);
	struct drm_crtc *crtc = &dss->acrtc.base;

	dss_crtc_mode_set_nofb(crtc);
	dss_crtc_enable(crtc, NULL);

	return 0;
}
