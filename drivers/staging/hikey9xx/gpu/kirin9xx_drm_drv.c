// SPDX-License-Identifier: GPL-2.0
/*
 * Hisilicon Kirin SoCs drm master driver
 *
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2014-2016 Hisilicon Limited.
 *
 * Author:
 *	<cailiwei@hisilicon.com>
 *	<zhengwanchun@hisilicon.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/of_platform.h>
#include <linux/component.h>
#include <linux/of_graph.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_managed.h>

#include "kirin9xx_dpe.h"
#include "kirin9xx_drm_drv.h"

static int kirin_drm_kms_cleanup(struct drm_device *dev)
{
	struct kirin_drm_private *priv = to_drm_private(dev);

	if (priv->fbdev)
		priv->fbdev = NULL;

	drm_kms_helper_poll_fini(dev);
	kirin9xx_dss_drm_cleanup(dev);

	return 0;
}

static void kirin_fbdev_output_poll_changed(struct drm_device *dev)
{
	struct kirin_drm_private *priv = to_drm_private(dev);

	dsi_set_output_client(dev);

	drm_fb_helper_hotplug_event(priv->fbdev);
}

static const struct drm_mode_config_funcs kirin_drm_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.output_poll_changed = kirin_fbdev_output_poll_changed,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int kirin_drm_kms_init(struct drm_device *dev)
{
	long kirin_type;
	int ret;

	dev_set_drvdata(dev->dev, dev);

	ret = drmm_mode_config_init(dev);
	if (ret)
		return ret;

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;
	dev->mode_config.max_width = 2048;
	dev->mode_config.max_height = 2048;
	dev->mode_config.preferred_depth = 32;

	dev->mode_config.funcs = &kirin_drm_mode_config_funcs;

	/* display controller init */
	kirin_type = (long)of_device_get_match_data(dev->dev);
	if (WARN_ON(!kirin_type))
		return -ENODEV;

	ret = dss_drm_init(dev, kirin_type);
	if (ret)
		return ret;

	/* bind and init sub drivers */
	ret = component_bind_all(dev->dev, dev);
	if (ret) {
		DRM_ERROR("failed to bind all component.\n");
		return ret;
	}

	/* vblank init */
	ret = drm_vblank_init(dev, dev->mode_config.num_crtc);
	if (ret) {
		DRM_ERROR("failed to initialize vblank.\n");
		return ret;
	}
	/* with irq_enabled = true, we can use the vblank feature. */
	dev->irq_enabled = true;

	/* reset all the states of crtc/plane/encoder/connector */
	drm_mode_config_reset(dev);

	/* init kms poll for handling hpd */
	drm_kms_helper_poll_init(dev);

	return 0;
}

DEFINE_DRM_GEM_CMA_FOPS(kirin_drm_fops);

static int kirin_drm_connectors_register(struct drm_device *dev)
{
	struct drm_connector_list_iter conn_iter;
	struct drm_connector *failed_connector;
	struct drm_connector *connector;
	int ret;

	mutex_lock(&dev->mode_config.mutex);
	drm_connector_list_iter_begin(dev, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter) {
		ret = drm_connector_register(connector);
		if (ret) {
			failed_connector = connector;
			goto err;
		}
	}
	mutex_unlock(&dev->mode_config.mutex);

	return 0;

err:
	drm_connector_list_iter_begin(dev, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter) {
		if (failed_connector == connector)
			break;
		drm_connector_unregister(connector);
	}
	mutex_unlock(&dev->mode_config.mutex);

	return ret;
}

static struct drm_driver kirin_drm_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET |
				  DRIVER_ATOMIC | DRIVER_RENDER,

	.fops			= &kirin_drm_fops,
	.name			= "kirin9xx",
	.desc			= "Hisilicon Kirin9xx SoCs' DRM Driver",
	.date			= "20170309",
	.major			= 1,
	.minor			= 0,

	DRM_GEM_CMA_VMAP_DRIVER_OPS
};


static int compare_of(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static int kirin_drm_bind(struct device *dev)
{
	struct kirin_drm_private *priv;
	struct drm_device *drm;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	drm = &priv->drm;

	ret = devm_drm_dev_init(dev, drm, &kirin_drm_driver);
	if (ret) {
		kfree(priv);
		return ret;
	}
	drmm_add_final_kfree(drm, priv);

	ret = kirin_drm_kms_init(drm);
	if (ret)
		return ret;

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	drm_fbdev_generic_setup(drm, 0);

	/* connectors should be registered after drm device register */
	ret = kirin_drm_connectors_register(drm);
	if (ret)
		goto err_drm_dev_unregister;

	return 0;

err_drm_dev_unregister:
	drm_dev_unregister(drm);
	kirin_drm_kms_cleanup(drm);
	drm_dev_put(drm);

	return ret;
}

static void kirin_drm_unbind(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	drm_dev_unregister(drm_dev);
	drm_atomic_helper_shutdown(drm_dev);
	kirin_drm_kms_cleanup(drm_dev);
	drm_dev_put(drm_dev);
}

static const struct component_master_ops kirin_drm_ops = {
	.bind = kirin_drm_bind,
	.unbind = kirin_drm_unbind,
};

static int kirin_drm_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct component_match *match = NULL;
	struct device_node *remote;

	remote = of_graph_get_remote_node(np, 0, 0);
	if (!remote)
		return -ENODEV;

	drm_of_component_match_add(dev, &match, compare_of, remote);
	of_node_put(remote);

	return component_master_add_with_match(dev, &kirin_drm_ops, match);
}

static int kirin_drm_platform_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &kirin_drm_ops);
	return 0;
}

static const struct of_device_id kirin_drm_dt_ids[] = {
	{ .compatible = "hisilicon,hi3660-dpe",
	  .data = (void *)FB_ACCEL_HI366x,
	},
	{ .compatible = "hisilicon,kirin970-dpe",
	  .data = (void *)FB_ACCEL_KIRIN970,
	},
	{ /* end node */ },
};
MODULE_DEVICE_TABLE(of, kirin_drm_dt_ids);

static struct platform_driver kirin_drm_platform_driver = {
	.probe = kirin_drm_platform_probe,
	.remove = kirin_drm_platform_remove,
	.suspend = kirin9xx_dss_drm_suspend,
	.resume = kirin9xx_dss_drm_resume,
	.driver = {
		.name = "kirin9xx-drm",
		.of_match_table = kirin_drm_dt_ids,
	},
};

module_platform_driver(kirin_drm_platform_driver);

MODULE_AUTHOR("cailiwei <cailiwei@hisilicon.com>");
MODULE_AUTHOR("zhengwanchun <zhengwanchun@hisilicon.com>");
MODULE_DESCRIPTION("hisilicon Kirin SoCs' DRM master driver");
MODULE_LICENSE("GPL v2");
