/* Copyright (c) 2013-2014, Hisilicon Tech. Co., Ltd. All rights reserved.
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
#ifndef KIRIN_FB_PANEL_H
#define KIRIN_FB_PANEL_H

/* dts initial */
#define DTS_FB_RESOURCE_INIT_READY	BIT(0)
#define DTS_PWM_READY	BIT(1)
/* #define DTS_BLPWM_READY	BIT(2) */
#define DTS_SPI_READY	BIT(3)
#define DTS_PANEL_PRIMARY_READY	BIT(4)
#define DTS_PANEL_EXTERNAL_READY	BIT(5)
#define DTS_PANEL_OFFLINECOMPOSER_READY	BIT(6)
#define DTS_PANEL_WRITEBACK_READY	BIT(7)
#define DTS_PANEL_MEDIACOMMON_READY	 BIT(8)

/* device name */
#define DEV_NAME_DSS_DPE		"dss_dpe"
#define DEV_NAME_SPI			"spi_dev0"
#define DEV_NAME_HDMI			"hdmi"
#define DEV_NAME_DP				"dp"
#define DEV_NAME_MIPI2RGB		"mipi2rgb"
#define DEV_NAME_RGB2MIPI		"rgb2mipi"
#define DEV_NAME_MIPIDSI		"mipi_dsi"
#define DEV_NAME_FB				"hisi_fb"
#define DEV_NAME_PWM			"hisi_pwm"
#define DEV_NAME_BLPWM			"hisi_blpwm"
#define DEV_NAME_LCD_BKL		"lcd_backlight0"

/* vcc name */
#define REGULATOR_PDP_NAME	"ldo3"


/* irq name */
#define IRQ_PDP_NAME	"irq_pdp"
#define IRQ_SDP_NAME	"irq_sdp"
#define IRQ_ADP_NAME	"irq_adp"
#define IRQ_MDC_NAME	"irq_mdc"
#define IRQ_DSI0_NAME	"irq_dsi0"
#define IRQ_DSI1_NAME	"irq_dsi1"

/* dts compatible */
#define DTS_COMP_FB_NAME	"hisilicon,hisifb"
#define DTS_COMP_PWM_NAME	"hisilicon,hisipwm"
#define DTS_COMP_BLPWM_NAME	"hisilicon,hisiblpwm"
#define DTS_PATH_LOGO_BUFFER	"/reserved-memory/logo-buffer"

/* lcd resource name */
#define LCD_BL_TYPE_NAME	"lcd-bl-type"
#define FPGA_FLAG_NAME "fpga_flag"
#define LCD_DISPLAY_TYPE_NAME	"lcd-display-type"
#define LCD_IFBC_TYPE_NAME	"lcd-ifbc-type"

/* backlight type */
#define BL_SET_BY_NONE	BIT(0)
#define BL_SET_BY_PWM	BIT(1)
#define BL_SET_BY_BLPWM	BIT(2)
#define BL_SET_BY_MIPI	BIT(3)
#define BL_SET_BY_SH_BLPWM	BIT(4)

/* supported display effect type */
#define COMFORM_MODE			BIT(0)
#define ACM_COLOR_ENHANCE_MODE	BIT(1)
#define IC_COLOR_ENHANCE_MODE	BIT(2)
#define CINEMA_MODE				BIT(3)
#define VR_MODE                     BIT(4)
#define FPS_30_60_SENCE_MODE   BIT(5)
#define LED_RG_COLOR_TEMP_MODE	BIT(16)
#define GAMMA_MAP    BIT(19)

#define LCD_BL_IC_NAME_MAX	(50)

#define DEV_DSS_VOLTAGE_ID (20)

enum MIPI_LP11_MODE {
	MIPI_NORMAL_LP11 = 0,
	MIPI_SHORT_LP11 = 1,
	MIPI_DISABLE_LP11 = 2,
};

/* resource desc */
struct resource_desc {
	uint32_t flag;
	char *name;
	uint32_t *value;
};

/* dtype for vcc */
enum {
	DTYPE_VCC_GET,
	DTYPE_VCC_PUT,
	DTYPE_VCC_ENABLE,
	DTYPE_VCC_DISABLE,
	DTYPE_VCC_SET_VOLTAGE,
};

/* vcc desc */
struct vcc_desc {
	int dtype;
	char *id;
	struct regulator **regulator;
	int min_uV;
	int max_uV;
	int waittype;
	int wait;
};

/* pinctrl operation */
enum {
	DTYPE_PINCTRL_GET,
	DTYPE_PINCTRL_STATE_GET,
	DTYPE_PINCTRL_SET,
	DTYPE_PINCTRL_PUT,
};

/* pinctrl state */
enum {
	DTYPE_PINCTRL_STATE_DEFAULT,
	DTYPE_PINCTRL_STATE_IDLE,
};

/* pinctrl data */
struct pinctrl_data {
	struct pinctrl *p;
	struct pinctrl_state *pinctrl_def;
	struct pinctrl_state *pinctrl_idle;
};
struct pinctrl_cmd_desc {
	int dtype;
	struct pinctrl_data *pctrl_data;
	int mode;
};

/* dtype for gpio */
enum {
	DTYPE_GPIO_REQUEST,
	DTYPE_GPIO_FREE,
	DTYPE_GPIO_INPUT,
	DTYPE_GPIO_OUTPUT,
};

/* gpio desc */
struct gpio_desc {
	int dtype;
	int waittype;
	int wait;
	char *label;
	uint32_t *gpio;
	int value;
};

enum bl_control_mode {
	REG_ONLY_MODE = 1,
	PWM_ONLY_MODE,
	MUTI_THEN_RAMP_MODE,
	RAMP_THEN_MUTI_MODE,
	I2C_ONLY_MODE = 6,
	BLPWM_AND_CABC_MODE,
	COMMON_IC_MODE = 8,
};

/*******************************************************************************
** FUNCTIONS PROTOTYPES
*/
#define MIPI_DPHY_NUM	(2)

extern uint32_t g_dts_resouce_ready;

int resource_cmds_tx(struct platform_device *pdev,
	struct resource_desc *cmds, int cnt);
int vcc_cmds_tx(struct platform_device *pdev, struct vcc_desc *cmds, int cnt);
int pinctrl_cmds_tx(struct platform_device *pdev, struct pinctrl_cmd_desc *cmds, int cnt);
int gpio_cmds_tx(struct gpio_desc *cmds, int cnt);
extern struct spi_device *g_spi_dev;
int hisi_pwm_set_backlight(struct backlight_device *bl, uint32_t bl_level);

int hisi_pwm_off(void);
int hisi_pwm_on(void);

#endif /* KIRIN_FB_PANEL_H */
