/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2014-2016 Hisilicon Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __KIRIN_DRM_DRV_H__
#define __KIRIN_DRM_DRV_H__

#include <drm/drm_crtc.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_print.h>

#include <linux/iommu.h>

#define MAX_CRTC	2

struct kirin_drm_private {
	struct drm_fb_helper *fbdev;
	struct drm_crtc *crtc[MAX_CRTC];
};

struct kirin_fbdev {
	struct drm_fb_helper fb_helper;
	struct drm_framebuffer *fb;
};

/* provided by kirin9xx_drm_dss.c */
void kirin9xx_dss_drm_cleanup(struct drm_device *dev);
int kirin9xx_dss_drm_suspend(struct platform_device *pdev, pm_message_t state);
int kirin9xx_dss_drm_resume(struct platform_device *pdev);
int dss_drm_init(struct drm_device *dev, u32 g_dss_version_tag);

void dsi_set_output_client(struct drm_device *dev);

#endif /* __KIRIN_DRM_DRV_H__ */
