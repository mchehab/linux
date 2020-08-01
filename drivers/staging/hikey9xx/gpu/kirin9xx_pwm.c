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

#include <linux/clk.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>

#include "kirin9xx_drm_dpe_utils.h"
#include "kirin9xx_fb_panel.h"
#include "kirin9xx_dw_dsi_reg.h"

#if defined(CONFIG_DRM_HISI_KIRIN970)
#include "kirin970_dpe_reg.h"
#else
#include "kirin960_dpe_reg.h"
#endif

/* default pwm clk */
#define DEFAULT_PWM_CLK_RATE	(80 * 1000000L)

static char __iomem *hisifd_pwm_base;
static char __iomem *hisi_peri_crg_base;
static struct clk *g_pwm_clk;
static struct platform_device *g_pwm_pdev;
static int g_pwm_on;

static struct pinctrl_data pwmpctrl;

static struct pinctrl_cmd_desc pwm_pinctrl_init_cmds[] = {
	{DTYPE_PINCTRL_GET, &pwmpctrl, 0},
	{DTYPE_PINCTRL_STATE_GET, &pwmpctrl, DTYPE_PINCTRL_STATE_DEFAULT},
	{DTYPE_PINCTRL_STATE_GET, &pwmpctrl, DTYPE_PINCTRL_STATE_IDLE},
};

static struct pinctrl_cmd_desc pwm_pinctrl_normal_cmds[] = {
	{DTYPE_PINCTRL_SET, &pwmpctrl, DTYPE_PINCTRL_STATE_DEFAULT},
};

static struct pinctrl_cmd_desc pwm_pinctrl_lowpower_cmds[] = {
	{DTYPE_PINCTRL_SET, &pwmpctrl, DTYPE_PINCTRL_STATE_IDLE},
};

static struct pinctrl_cmd_desc pwm_pinctrl_finit_cmds[] = {
	{DTYPE_PINCTRL_PUT, &pwmpctrl, 0},
};

#define PWM_LOCK_OFFSET	(0x0000)
#define PWM_CTL_OFFSET	(0X0004)
#define PWM_CFG_OFFSET	(0x0008)
#define PWM_PR0_OFFSET	(0x0100)
#define PWM_PR1_OFFSET	(0x0104)
#define PWM_C0_MR_OFFSET	(0x0300)
#define PWM_C0_MR0_OFFSET	(0x0304)

#define PWM_OUT_PRECISION	(800)

int pinctrl_cmds_tx(struct platform_device *pdev, struct pinctrl_cmd_desc *cmds, int cnt)
{
	int ret = 0;

	int i = 0;
	struct pinctrl_cmd_desc *cm = NULL;

	cm = cmds;

	for (i = 0; i < cnt; i++) {
		if (!cm) {
			DRM_ERROR("cm is null! index=%d\n", i);
			continue;
		}

		if (cm->dtype == DTYPE_PINCTRL_GET) {
			if (!pdev) {
				DRM_ERROR("pdev is NULL");
				return -EINVAL;
			}
			cm->pctrl_data->p = devm_pinctrl_get(&pdev->dev);
			if (IS_ERR(cm->pctrl_data->p)) {
				ret = -1;
				DRM_ERROR("failed to get p, index=%d!\n", i);
				goto err;
			}
		} else if (cm->dtype == DTYPE_PINCTRL_STATE_GET) {
			if (cm->mode == DTYPE_PINCTRL_STATE_DEFAULT) {
				cm->pctrl_data->pinctrl_def = pinctrl_lookup_state(cm->pctrl_data->p, PINCTRL_STATE_DEFAULT);
				if (IS_ERR(cm->pctrl_data->pinctrl_def)) {
					ret = -1;
					DRM_ERROR("failed to get pinctrl_def, index=%d!\n", i);
					goto err;
				}
			} else if (cm->mode == DTYPE_PINCTRL_STATE_IDLE) {
				cm->pctrl_data->pinctrl_idle = pinctrl_lookup_state(cm->pctrl_data->p, PINCTRL_STATE_IDLE);
				if (IS_ERR(cm->pctrl_data->pinctrl_idle)) {
					ret = -1;
					DRM_ERROR("failed to get pinctrl_idle, index=%d!\n", i);
					goto err;
				}
			} else {
				ret = -1;
				DRM_ERROR("unknown pinctrl type to get!\n");
				goto err;
			}
		} else if (cm->dtype == DTYPE_PINCTRL_SET) {
			if (cm->mode == DTYPE_PINCTRL_STATE_DEFAULT) {
				if (cm->pctrl_data->p && cm->pctrl_data->pinctrl_def) {
					ret = pinctrl_select_state(cm->pctrl_data->p, cm->pctrl_data->pinctrl_def);
					if (ret) {
						DRM_ERROR("could not set this pin to default state!\n");
						ret = -1;
						goto err;
					}
				}
			} else if (cm->mode == DTYPE_PINCTRL_STATE_IDLE) {
				if (cm->pctrl_data->p && cm->pctrl_data->pinctrl_idle) {
					ret = pinctrl_select_state(cm->pctrl_data->p, cm->pctrl_data->pinctrl_idle);
					if (ret) {
						DRM_ERROR("could not set this pin to idle state!\n");
						ret = -1;
						goto err;
					}
				}
			} else {
				ret = -1;
				DRM_ERROR("unknown pinctrl type to set!\n");
				goto err;
			}
		} else if (cm->dtype == DTYPE_PINCTRL_PUT) {
			if (cm->pctrl_data->p)
				pinctrl_put(cm->pctrl_data->p);
		} else {
			DRM_ERROR("not supported command type!\n");
			ret = -1;
			goto err;
		}

		cm++;
	}

	return 0;

err:
	return ret;
}

int hisi_pwm_set_backlight(struct backlight_device *bl, uint32_t bl_level)
{
	char __iomem *pwm_base = NULL;
	u32 bl_max = bl->props.max_brightness;

	pwm_base = hisifd_pwm_base;
	if (!pwm_base) {
		DRM_ERROR("pwm_base is null!\n");
		return -EINVAL;
	}

	DRM_INFO("bl_level=%d.\n", bl_level);

	if (bl_max < 1) {
		DRM_ERROR("bl_max(%d) is out of range!!", bl_max);
		return -EINVAL;
	}

	if (bl_level > bl_max)
		bl_level = bl_max;

	bl_level = (bl_level * PWM_OUT_PRECISION) / bl_max;

	writel(0x1acce551, pwm_base + PWM_LOCK_OFFSET);
	writel(0x0, pwm_base + PWM_CTL_OFFSET);
	writel(0x2, pwm_base + PWM_CFG_OFFSET);
	writel(0x1, pwm_base + PWM_PR0_OFFSET);
	writel(0x2, pwm_base + PWM_PR1_OFFSET);
	writel(0x1, pwm_base + PWM_CTL_OFFSET);
	writel((PWM_OUT_PRECISION - 1), pwm_base + PWM_C0_MR_OFFSET);
	writel(bl_level, pwm_base + PWM_C0_MR0_OFFSET);

	return 0;
}

int hisi_pwm_on(void)
{
	struct clk *clk_tmp = NULL;
	char __iomem *pwm_base = NULL;
	char __iomem *peri_crg_base = NULL;
	int ret = 0;

	DRM_INFO(" +.\n");

	peri_crg_base = hisi_peri_crg_base;
	if (!peri_crg_base) {
		DRM_ERROR("peri_crg_base is NULL");
		return -EINVAL;
	}

	pwm_base = hisifd_pwm_base;
	if (!pwm_base) {
		DRM_ERROR("pwm_base is null!\n");
		return -EINVAL;
	}

	if (g_pwm_on == 1)
		return 0;

	// dis-reset pwm
	writel(0x1, peri_crg_base + PERRSTDIS2);

	clk_tmp = g_pwm_clk;
	if (clk_tmp) {
		ret = clk_prepare(clk_tmp);
		if (ret) {
			DRM_ERROR("dss_pwm_clk clk_prepare failed, error=%d!\n", ret);
			return -EINVAL;
		}

		ret = clk_enable(clk_tmp);
		if (ret) {
			DRM_ERROR("dss_pwm_clk clk_enable failed, error=%d!\n", ret);
			return -EINVAL;
		}

		DRM_INFO("dss_pwm_clk clk_enable succeeded, ret=%d!\n", ret);
	}

	ret = pinctrl_cmds_tx(g_pwm_pdev, pwm_pinctrl_normal_cmds,
			      ARRAY_SIZE(pwm_pinctrl_normal_cmds));

	//if enable PWM, please set IOMG_004 in IOC_AO module
	//set IOMG_004: select PWM_OUT0

	g_pwm_on = 1;

	return ret;
}

int hisi_pwm_off(void)
{
	struct clk *clk_tmp = NULL;
	char __iomem *pwm_base = NULL;
	char __iomem *peri_crg_base = NULL;
	int ret = 0;

	peri_crg_base = hisi_peri_crg_base;
	if (!peri_crg_base) {
		DRM_ERROR("peri_crg_base is NULL");
		return -EINVAL;
	}

	pwm_base = hisifd_pwm_base;
	if (!pwm_base) {
		DRM_ERROR("pwm_base is null!\n");
		return -EINVAL;
	}

	if (g_pwm_on == 0)
		return 0;

	ret = pinctrl_cmds_tx(g_pwm_pdev, pwm_pinctrl_lowpower_cmds,
			      ARRAY_SIZE(pwm_pinctrl_lowpower_cmds));

	clk_tmp = g_pwm_clk;
	if (clk_tmp) {
		clk_disable(clk_tmp);
		clk_unprepare(clk_tmp);
	}

	//reset pwm
	writel(0x1, peri_crg_base + PERRSTEN2);

	g_pwm_on = 0;

	return ret;
}

static int hisi_pwm_probe(struct platform_device *pdev)
{
	struct device_node *np = NULL;
	int ret = 0;

	if (!pdev) {
		DRM_ERROR("pdev is NULL");
		return -EINVAL;
	}

	g_pwm_pdev = pdev;

	np = of_find_compatible_node(NULL, NULL, DTS_COMP_PWM_NAME);
	if (!np) {
		DRM_ERROR("NOT FOUND device node %s!\n", DTS_COMP_PWM_NAME);
		ret = -ENXIO;
		goto err_return;
	}

	/* get pwm reg base */
	hisifd_pwm_base = of_iomap(np, 0);
	if (!hisifd_pwm_base) {
		DRM_ERROR("failed to get pwm_base resource.\n");
		return -ENXIO;
	}

	/* get peri_crg_base */
	hisi_peri_crg_base = of_iomap(np, 1);
	if (!hisi_peri_crg_base) {
		DRM_ERROR("failed to get peri_crg_base resource.\n");
		return -ENXIO;
	}

	/* pwm pinctrl init */
	ret = pinctrl_cmds_tx(pdev, pwm_pinctrl_init_cmds,
			      ARRAY_SIZE(pwm_pinctrl_init_cmds));
	if (ret != 0) {
		DRM_ERROR("Init pwm pinctrl failed! ret=%d.\n", ret);
		goto err_return;
	}

	/* get pwm clk resource */
	g_pwm_clk = of_clk_get(np, 0);
	if (IS_ERR(g_pwm_clk)) {
		DRM_ERROR("%s clock not found: %d!\n",
			  np->name, (int)PTR_ERR(g_pwm_clk));
		ret = -ENXIO;
		goto err_return;
	}

	DRM_INFO("dss_pwm_clk:[%lu]->[%lu].\n",
		 DEFAULT_PWM_CLK_RATE, clk_get_rate(g_pwm_clk));

	return 0;

err_return:
	return ret;
}

static int hisi_pwm_remove(struct platform_device *pdev)
{
	struct clk *clk_tmp = NULL;
	int ret = 0;

	ret = pinctrl_cmds_tx(pdev, pwm_pinctrl_finit_cmds,
			      ARRAY_SIZE(pwm_pinctrl_finit_cmds));

	clk_tmp = g_pwm_clk;
	if (clk_tmp) {
		clk_put(clk_tmp);
		clk_tmp = NULL;
	}

	return ret;
}

static const struct of_device_id hisi_pwm_match_table[] = {
	{
		.compatible = "hisilicon,hisipwm",
		.data = NULL,
	},
	{},
};
MODULE_DEVICE_TABLE(of, hisi_pwm_match_table);

static struct platform_driver this_driver = {
	.probe = hisi_pwm_probe,
	.remove = hisi_pwm_remove,
	.suspend = NULL,
	.resume = NULL,
	.shutdown = NULL,
	.driver = {
		.name = DEV_NAME_PWM,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(hisi_pwm_match_table),
	},
};

static int __init hisi_pwm_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&this_driver);
	if (ret) {
		DRM_ERROR("platform_driver_register failed, error=%d!\n", ret);
		return ret;
	}

	return ret;
}

module_init(hisi_pwm_init);

MODULE_AUTHOR("cailiwei <cailiwei@hisilicon.com>");
MODULE_AUTHOR("zhangxiubin <zhangxiubin1@huawei.com>");
MODULE_DESCRIPTION("hisilicon Kirin SoCs' pwm driver");
MODULE_LICENSE("GPL v2");
