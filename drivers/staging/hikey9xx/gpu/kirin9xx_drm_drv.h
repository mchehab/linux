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

/* display controller init/cleanup ops */
struct kirin_dc_ops {
	int (*init)(struct drm_device *dev);
	void (*cleanup)(struct drm_device *dev);
	int (*suspend)(struct platform_device *pdev, pm_message_t state);
	int (*resume)(struct platform_device *pdev);
};

struct kirin_drm_private {
	struct drm_fb_helper *fbdev;
	struct drm_crtc *crtc[MAX_CRTC];
};

struct kirin_fbdev {
	struct drm_fb_helper fb_helper;
	struct drm_framebuffer *fb;

	struct ion_client *ion_client;
	struct ion_handle *ion_handle;
	void *screen_base;
	unsigned long smem_start;
	unsigned long screen_size;
	int shared_fd;
};

extern const struct kirin_dc_ops kirin960_dss_dc_ops;
extern const struct kirin_dc_ops kirin970_dss_dc_ops;
void dsi_set_output_client(struct drm_device *dev);

struct drm_framebuffer *kirin_framebuffer_init(struct drm_device *dev,
					       struct drm_mode_fb_cmd2 *mode_cmd);

#endif /* __KIRIN_DRM_DRV_H__ */
