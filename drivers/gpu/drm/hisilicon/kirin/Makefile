# SPDX-License-Identifier: GPL-2.0-only
kirin6220-drm-y := kirin_drm_drv.o \
		   kirin_drm_ade.o

kirin9xx-drm-y := kirin9xx_drm_drv.o \
		  kirin9xx_fbdev.o \
		  kirin9xx_fb.o \
		  kirin9xx_drm_dss.o \
		  kirin9xx_drm_dpe_utils.o \
		  kirin9xx_drm_overlay_utils.o \
		  kirin9xx_pwm.o

obj-$(CONFIG_DRM_HISI_KIRIN6220) += kirin6220-drm.o
obj-$(CONFIG_DRM_HISI_KIRIN9XX) += kirin9xx-drm.o
obj-$(CONFIG_DRM_HISI_KIRIN_DSI) += dw_drm_dsi.o
