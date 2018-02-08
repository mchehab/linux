EXTRA_CFLAGS += \
		-Iinclude/drm

kirin-drm-y := kirin_fbdev.o \
		kirin_fb.o \
		kirin_drm_drv.o \
		kirin_drm_dss.o \
		kirin_drm_dpe_utils.o \
		kirin_drm_overlay_utils.o \
		kirin_pwm.o \
		hdmi/adv7535.o \


obj-$(CONFIG_HDMI_ADV7511_AUDIO) += hdmi/adv7535_audio.o
obj-$(CONFIG_DRM_KIRIN_960) += kirin-drm.o
obj-$(CONFIG_HISI_KIRIN_DW_DSI) += dw_drm_dsi.o
