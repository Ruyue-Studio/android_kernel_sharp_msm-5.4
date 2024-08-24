/*
 *
 * Copyright (C) 2021 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/iio/consumer.h>
#include <video/mipi_display.h>
//#include "../dsi/dsi_panel.h"
#include "../dsi/dsi_display.h"
#include "../sde/sde_kms.h"
#include "drm_bias.h"
#include "drm_cmn.h"
#include "drm_oneshot.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define BIAS_CALC_COEFFICIENT	1000000000000

#define BIAS_LOGARITHMS_NUM  120
#define BIAS_VOLTAGES_NUM	  64
#define BIAS_FREQUENCY_NUM	3
#define BIAS_LOGARITHMS_MIN  0
#define BIAS_LOGARITHMS_MAX  100
#define BIAS_LOGARITHMS_SPECIAL 119
#define BIAS_VOLTAGES_MIN	  0
#define BIAS_VOLTAGES_MAX	  61
#define BIAS_VOLTAGES_ERROR_VALUE 255
#define BIAS_REGISTER_MIN	  0
#define BIAS_REGISTER_MAX	  63

#define BIAS_DUTY_MIN		 10
#define BIAS_DUTY_MAX		100

#define BIAS_WAIT_TIME_LCD_THERM 10000
#define BIAS_MIN_VALUE_TEMPERATURE (-40960)
#define BIAS_MAX_VALUE_TEMPERATURE 130048
#define BIAS_INVALID_VALUE_TEMPERATURE (BIAS_MAX_VALUE_TEMPERATURE+1)
#define BIAS_CONVERT_VALUE_TO_KELVIN 273150

#define BIAS_NORMAL_THRESHOLD1			362
#define BIAS_NORMAL_THRESHOLD2			2828
#define BIAS_GAME_THRESHOLD1			362
#define BIAS_GAME_THRESHOLD2			2020
#define BIAS_GRADATION_12BIT_THRESHOLD	0
#define BIAS_HIGH_BRIGHTNESS_THRESHOLD	2828

static const int64_t drm_bias_calc_bias[2][9] = {
    //BIAS_GRADATION_LOW
	{      -37709499 ,  //-0.0000377094995507137 * BIAS_CALC_COEFFICIENT
	     11147870804 ,  // 0.0111478708046797    * BIAS_CALC_COEFFICIENT
	       -46459142 ,  //-0.000046459142717954  * BIAS_CALC_COEFFICIENT
	      1375628956 ,  // 0.00137562895675595   * BIAS_CALC_COEFFICIENT
	     30112676404 ,  // 0.030112676404383     * BIAS_CALC_COEFFICIENT
	   -323424022319 ,  //-0.323424022319734     * BIAS_CALC_COEFFICIENT
	      1553608871 ,  // 0.00155360887153845   * BIAS_CALC_COEFFICIENT
	   2523115058073 ,  // 2.52311505807356      * BIAS_CALC_COEFFICIENT
	              34 }, // 3.4 * 10
    //BIAS_GRADATION_HIGH
	{     8852000000 ,  // 0.008852              * BIAS_CALC_COEFFICIENT
	  -2857000000000 ,  //-2.857                 * BIAS_CALC_COEFFICIENT
	      6363000000 ,  // 0.006363              * BIAS_CALC_COEFFICIENT
	    -20840000000 ,  //-0.02084               * BIAS_CALC_COEFFICIENT
	  -3983000000000 ,  //-3.983                 * BIAS_CALC_COEFFICIENT
	   6853000000000 ,  // 6.853                 * BIAS_CALC_COEFFICIENT
	    -10330000000 ,  //-0.01033               * BIAS_CALC_COEFFICIENT
	   9316000000000 ,  // 9.316                 * BIAS_CALC_COEFFICIENT
	              33 }, //3.3 * 10
};

static const int drm__bias_logarithms[BIAS_LOGARITHMS_NUM] = {
0  ,0  ,30 ,48 ,60 ,70 ,78 ,85 ,90 ,95 ,	//n= 0..9
100,104,108,111,115,118,120,123,126,128,	//n= 10..19
130,132,134,136,138,140,141,143,145,146,	//n= 20..29
148,149,151,152,153,154,156,157,158,159,	//n= 20..39
160,161,162,163,164,165,166,167,168,169,	//n= 40..49
170,171,172,172,173,174,175,176,176,177,	//n= 50..59
178,179,179,180,181,181,182,183,183,184,	//n= 60..69
185,185,186,186,187,188,188,189,189,190,	//n= 70..79
190,191,191,192,192,193,193,194,194,195,	//n= 80..89
195,196,196,197,197,198,198,199,199,200,	//n= 90..99
200,-1 ,-1 ,-1 ,-1 ,-1 ,-1 ,-1 ,-1 ,-1 ,	//n=100..109
-1 ,-1 ,-1 ,-1 ,-1 ,-1 ,-1 ,-1 ,-1 ,208  //n=110..119
};

static const int drm_bias_frequencys[BIAS_FREQUENCY_NUM] = {
1,30,60
};

static const int64_t drm_bias_calc_mode1[3][4] = {
	{       0,  1000000000,        0,   25500000000},
	{ 2433090,   119221411,        0,   25500000000},
	{ 2367798,   303867403,        0,   25500000000},
};

static const int64_t drm_bias_calc_mode2[3][4] = {
	{       0,  1000000000,        0,   25500000000},
	{ 2412545,   126658625,        0,   25500000000},
	{ 2409639,   132530121,        0,   25500000000},
};

static const int64_t drm_bias_calc_mode5[2][4] = {
	{       0,    7000000000,  9016973,           0},
	{ 2367798,     303867403,        0, 25500000000},
};

/* ------------------------------------------------------------------------- */
/* STRUCTURE                                                                 */
/* ------------------------------------------------------------------------- */
struct drm_bias_ctx {
	struct workqueue_struct *therm_delayedwkq;
	struct delayed_work therm_delayedwk;
	struct dsi_display *display;
	bool enable;
	int display_therm_temperature;
};
static struct drm_bias_ctx drm_bias_ctx = {0};

/* ------------------------------------------------------------------------- */
/* FUNCTION                                                                  */
/* ------------------------------------------------------------------------- */
static int drm_bias_bias_calc(struct drm_bias_calc_param *param);
static int drm_bias_calc_duty_gmm(struct dsi_panel *panel, int *duty, int *gamma);
static int drm_bias_get_logarithm(int n);
static int drm_bias_get_voltage_for_register(unsigned char register_value);
static unsigned char drm_bias_get_register_for_voltage(int voltage);
static int drm_bias_calc_x1(int duty, int gradation);
static int drm_bias_calc_x2(int frequency);
static int drm_bias_send_bias_voltage(struct dsi_panel *panel, unsigned char voltage_reg);
static int drm_bias_calc_temperature(int value);
static int drm_bias_proc_best_bias(struct dsi_panel *panel);
static int drm_bias_calc_best_bias(struct dsi_panel *panel);
static int drm_bias_get_display_thermal(int *temperature, struct dsi_panel *panel);
static void drm_bias_display_therm_delayed_work(struct work_struct *therm_delayedwk);
static void drm_bias_start_notify_therm(void);
static void drm_bias_stop_notify_therm(void);

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int drm_bias_bias_calc(struct drm_bias_calc_param *param)
{
	int64_t Y = -EINVAL;
	int ret = -EINVAL;

	if (!param) {
		pr_err("%s:param is NULL\n", __func__);
		goto error;
	}
	pr_debug("%s:in gradation=%d,x1=%d,x2=%d,x3=%d,vob_otp=%d\n",
		__func__, param->gradation, param->x1, param->x2, param->x3, param->vob_otp);

	switch(param->gradation)
	{
	case BIAS_GRADATION_HIGH:
		if (param->x1 < 0 || param->x1 > 200) {
			pr_err("%s:out of range x1=%d,gradation=%d\n", __func__, param->x1,param->gradation);
			goto error;
		}
	    break;
	case BIAS_GRADATION_LOW:
		if (param->x1 < 1000 || param->x1 > 10000) {
			pr_err("%s:out of range x1=%d,gradation=%d\n", __func__, param->x1,param->gradation);
			goto error;
		}
	    break;
	default:
	    return -EINVAL;
	}
	if (param->x2 < 0 || param->x2 > 208) {
		pr_err("%s:out of range x2=%d\n", __func__, param->x2);
		goto error;
	}
	if (param->x3 < 232 || param->x3 > 403) {
		pr_err("%s:out of range x3=%d\n", __func__, param->x3);
		goto error;
	}
	if (param->vob_otp < 0 || param->vob_otp > 61) {
		pr_err("%s:out of range vob_otp=%d\n", __func__, param->vob_otp);
		goto error;
	}

	Y  = drm_bias_calc_bias[param->gradation][0] * param->x1/100 * param->x2/100 * param->x3;
	Y += drm_bias_calc_bias[param->gradation][1] * param->x1/100 * param->x2/100;
	Y += drm_bias_calc_bias[param->gradation][2] * param->x3 * param->x1/100;
	Y += drm_bias_calc_bias[param->gradation][3] * param->x2/100 * param->x3;
	Y += drm_bias_calc_bias[param->gradation][4] * param->x1/100;
	Y += drm_bias_calc_bias[param->gradation][5] * param->x2/100;
	Y += drm_bias_calc_bias[param->gradation][6] * param->x3;
	Y += drm_bias_calc_bias[param->gradation][7];
	Y *= 10;
	Y += 5 * (BIAS_CALC_COEFFICIENT / 10);
	Y /= BIAS_CALC_COEFFICIENT;
	Y += param->vob_otp - drm_bias_calc_bias[param->gradation][8];

	pr_debug("%s:Y=%ld\n", __func__, Y);

	ret = (int)Y;

	if (ret < 2)
		ret = 2;
	if (ret > 61)
		ret = 61;

error:
	pr_debug("%s:out ret=%d\n", __func__, ret);

	return ret;
}

int drm_bias_set_duty_gmm(struct dsi_panel *panel)
{
	int duty = 0;
	int gamma = 0;
	int rc = 0;
	struct device *dev = NULL;

	if (!panel) {
		pr_err("%s:dsi_panel is null\n", __func__);
		return -EINVAL;
	}
	rc = drm_bias_calc_duty_gmm(panel, &duty, &gamma);
	if (rc < 0) {
		pr_debug("%s: warning calc duty and gamma rc=%d\n",__func__, rc);
		return 0;
	}
	panel->duty = duty;
	panel->gmm = gamma;
	dev = panel->parent;
	if (dev) {
		sysfs_notify(&dev->kobj, "backlight", "duty");
		pr_debug("%s:sysfs_notify duty = %d ,gmm = %d\n", 
				__func__, panel->duty, panel->gmm);
	}
	return 0;
}

int drm_bias_clear_duty_gmm(struct dsi_panel *panel)
{
	if (!panel) {
		pr_err("%s:dsi_panel is null\n", __func__);
		return -EINVAL;
	}
	panel->duty = 0;
	panel->gmm = 0;
	pr_debug("%s:clear duty gmm\n",__func__);
	return 0;
}

static int drm_bias_calc_duty_gmm(struct dsi_panel *panel, int *duty, int *gamma)

{
	int64_t calc_duty = 0;
	int64_t calc_gamma = 0;
	enum msm_drive_mode drive_mode = 0;
	int x = 0;
	unsigned long nonlinear_bl = 0;
	struct dsi_backlight_config *bl = NULL;
	int64_t duty_slope = 0;
	int64_t duty_intercept = 0;
	int64_t gmm_slope = 0;
	int64_t gmm_intercept = 0;

	if (!panel || !duty || !gamma) {
		pr_err("%s:input param is null\n", __func__);
		return -EINVAL;
	}
	drive_mode = panel->drive_mode;
	bl = &panel->bl_config;
	if (!bl) {
		pr_err("%s: Invalid bkl_config data\n", __func__);
		return -EINVAL;
	}
	nonlinear_bl = bl->nonlinear_brightness;
	if (nonlinear_bl >= MAX_BL_LEVEL) {
		pr_err("%s:brightness over\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s:in drive_mode:%d, bl:%ld\n",
				__func__, drive_mode, nonlinear_bl);

	switch (drive_mode) {
	case MSM_DISPLAY_DRIVE_MODE_NORMAL:
	case MSM_DISPLAY_DRIVE_MODE_GRADATION_12BIT:
	case MSM_DISPLAY_DRIVE_MODE_SUPER_BRIGHT:
		if (nonlinear_bl <= BIAS_NORMAL_THRESHOLD1) {
			x = 0;
		} else if (nonlinear_bl <= BIAS_NORMAL_THRESHOLD2) {
			x = 1;
		} else {
			x = 2;
		}
		duty_slope = drm_bias_calc_mode1[x][0];
		duty_intercept = drm_bias_calc_mode1[x][1];
		gmm_slope = drm_bias_calc_mode1[x][2];
		gmm_intercept = drm_bias_calc_mode1[x][3];
		break;
	case MSM_DISPLAY_DRIVE_MODE_GAME:
		if (nonlinear_bl <= BIAS_GAME_THRESHOLD1) {
			x = 0;
		} else if (nonlinear_bl <= BIAS_GAME_THRESHOLD2) {
			x = 1;
		} else {
			x = 2;
		}
		duty_slope = drm_bias_calc_mode2[x][0];
		duty_intercept = drm_bias_calc_mode2[x][1];
		gmm_slope = drm_bias_calc_mode2[x][2];
		gmm_intercept = drm_bias_calc_mode2[x][3];
		break;
	case MSM_DISPLAY_DRIVE_MODE_HIGH_BRIGHTNESS:
		if (nonlinear_bl <= BIAS_HIGH_BRIGHTNESS_THRESHOLD) {
			x = 0;
		} else {
			x = 1;
		}
		duty_slope = drm_bias_calc_mode5[x][0];
		duty_intercept = drm_bias_calc_mode5[x][1];
		gmm_slope = drm_bias_calc_mode5[x][2];
		gmm_intercept = drm_bias_calc_mode5[x][3];
		break;
	case MSM_DISPLAY_DRIVE_MODE_DISP_OFF:
		pr_debug("%s:drive_mode off\n", __func__);
		return -EINVAL;
	default:
		pr_err("%s:other mode drive_mode:%d\n",
					__func__, drive_mode);
		return -EINVAL;
	}

	calc_duty = ((nonlinear_bl * duty_slope)
				+ duty_intercept + 50000000) / 100000000;
	calc_gamma = ((nonlinear_bl * gmm_slope)
				+ gmm_intercept + 50000000) / 100000000;
	pr_debug("%s:out Duty:%d out Gamma:%d\n",
			__func__, (int)calc_duty, (int)calc_gamma);

	if ((calc_duty < 0) || (calc_duty > BIAS_DUTY_MAX)) {
		pr_err("%s:duty is out of range\n", __func__);
		return -EINVAL;
	}
	if ((calc_gamma < 0) || (calc_gamma > 255)) {
		pr_err("%s:gamma is out of range\n", __func__);
		return -EINVAL;
	}
	*duty = (int)calc_duty;
	*gamma = (int)calc_gamma;

	return 0;
}

int drm_bias_best_bias_setting(struct dsi_panel *panel)
{
	int rc = 0;

	pr_debug("%s:start. caller=%pS\n",
			__func__, __builtin_return_address(0));

	if (!panel) {
		pr_err("%s: Invalid dsi_panel data\n", __func__);
		return -EINVAL;
	}

	if (drm_bias_ctx.display_therm_temperature == BIAS_INVALID_VALUE_TEMPERATURE) {
		pr_debug("%s:display_therm_temperature is invalid\n", __func__);
		return 0;
	}

	switch (panel->drive_mode) {
	case MSM_DISPLAY_DRIVE_MODE_NORMAL:
	case MSM_DISPLAY_DRIVE_MODE_GAME:
	case MSM_DISPLAY_DRIVE_MODE_GRADATION_12BIT:
		rc = drm_bias_proc_best_bias(panel);
		if (rc < 0) {
			pr_err("%s: error drm_bias_proc_best_bias%d\n",__func__, rc);
#ifdef CONFIG_DEBUG_FS
			if (rc == -EINVAL) {
				BUG();
			}
#endif /* CONFIG_DEBUG_FS */
			return -EINVAL;
		}
		break;
	default:
		pr_debug("%s:other mode\n", __func__);
		return 0;
	}
	return 0;
}

int drm_bias_default_bias_setting(struct dsi_panel *panel)
{
	unsigned char voltage_reg = 0;
	int rc = 0;

	pr_debug("%s:in\n", __func__);

	if (!panel) {
		pr_err("%s: Invalid dsi_panel data\n", __func__);
		return -EINVAL;
	}

	voltage_reg = drm_bias_get_otp_voltage();
	rc = drm_bias_send_bias_voltage(panel, voltage_reg);
	pr_debug("%s:out set default voltage_reg=0x%02x rc=%d\n", __func__, voltage_reg, rc);
	return rc;
}

static int drm_bias_get_logarithm(int n)
{
	pr_debug("%s:in n=%d\n", __func__, n);
	
	//parameter check
	if ((n < BIAS_LOGARITHMS_MIN || n > BIAS_LOGARITHMS_MAX) && n != BIAS_LOGARITHMS_SPECIAL) {
		//out of range
		pr_err("%s:out of range n=%d\n", __func__, n);
		return -EINVAL;
	}
	
	pr_debug("%s:out drm__bias_logarithms[n]=%d\n", __func__, drm__bias_logarithms[n]);
	return drm__bias_logarithms[n];
}

static int drm_bias_get_voltage_for_register(unsigned char register_value)
{
	int voltage = 0;
	pr_debug("%s:in register_value=0x%02x\n", __func__, register_value);
	
	//parameter check
	if (register_value < BIAS_REGISTER_MIN || register_value > BIAS_REGISTER_MAX) {
		//out of range
		pr_err("%s:out of range register_value=0x%02x\n", __func__, register_value);
		return -EINVAL;
	}

	if (register_value > 3) {
		voltage = register_value - 2;
	}
	
	pr_debug("%s:out voltage=%d\n", __func__, voltage);
	return voltage;
}

static unsigned char drm_bias_get_register_for_voltage(int voltage)
{
	unsigned char register_value = 0;
	
	pr_debug("%s:in voltage=%d\n", __func__, voltage);
	
	//parameter check
	if (voltage < BIAS_VOLTAGES_MIN || voltage > BIAS_VOLTAGES_MAX) {
		//out of range
		pr_err("%s:out of range voltage=%d\n", __func__, voltage);
		return BIAS_VOLTAGES_ERROR_VALUE;
	}

	if (voltage > 3) {
		register_value = voltage + 2;
	} else {
		register_value = 3;
	}
	pr_debug("%s:out register_value=0x%02x\n", __func__, register_value);
	return register_value;
}

static int drm_bias_calc_x1(int duty, int gradation)
{
	int x1 = -EINVAL;

	pr_debug("%s:in duty=%d\n", __func__, duty);

	//parameter check
	if( duty < BIAS_DUTY_MIN || duty > BIAS_DUTY_MAX ) {
		//out of range
		pr_err("%s:out of range duty=%d\n", __func__, duty);
		return -EINVAL;
	}

	switch(gradation)
	{
	case BIAS_GRADATION_HIGH:
		x1 = drm_bias_get_logarithm(100 - duty);
	    break;
	case BIAS_GRADATION_LOW:
		x1 = duty * 100;
	    break;
	default:
	    return -EINVAL;
	}

	pr_debug("%s:out x1=%d\n", __func__, x1);
	return x1;
}

static int drm_bias_calc_x2(int frequency)
{
	int x2 = -EINVAL;
	bool check = false;
	int count = 0;
	
	pr_debug("%s:in frequency=%d\n", __func__, frequency);
	
	//parameter check
	for( count = 0; count < BIAS_FREQUENCY_NUM; count++ ) {
		if(drm_bias_frequencys[count] == frequency) {
			check = true;
			break;
		}
	}
	if( check == false ) {
		//out of range
		pr_err("%s:out of range frequency=%d\n", __func__, frequency);
		return -EINVAL;
	}
	
	x2 = drm_bias_get_logarithm((120 / frequency) - 1);
	
	pr_debug("%s:out x2=%d\n", __func__, x2);
	return x2;
}

unsigned char drm_bias_get_otp_voltage(void)
{
	unsigned char voltage = 0;

	voltage = drm_cmn_get_otp_bias();
	pr_debug("%s: Voltage:0x%02x\n",__func__, voltage);
	return voltage;
}

static int drm_bias_send_bias_voltage(struct dsi_panel *panel, unsigned char voltage_reg)
{
	int rc = 0;
	struct dsi_display *pdisp = NULL;
	unsigned char switch_page1[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01};
	unsigned char index_param1[]  = {0x6F, 0x02};
	unsigned char index_param2[]  = {0x6F, 0x1A};
	unsigned char voltage_buf[]  = {0xB9, 0x00};
	unsigned char default_page[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00};
	struct dsi_cmd_desc voltage_cmd[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM, 0, 0,
			sizeof(switch_page1), switch_page1,
			0, NULL}, 0, 0},
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM, 0, 0,
			sizeof(index_param1), index_param1,
			0, NULL}, 0, 0},
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM, 0, 0,
			sizeof(voltage_buf), voltage_buf,
			0, NULL}, 0, 0},
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM, 0, 0,
			sizeof(index_param2), index_param2,
			0, NULL}, 0, 0},
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM, 0, 0,
			sizeof(voltage_buf), voltage_buf,
			0, NULL}, 1, 0},
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, MIPI_DSI_MSG_USE_LPM, 0, 0,
			sizeof(default_page), default_page,
			0, NULL}, 1, 0},
	};

	if (!panel) {
		pr_err("%s: Invalid dsi_panel data\n", __func__);
		return -EINVAL;
	}

	if (!dsi_panel_initialized(panel)) {
		pr_err("%s: panel not yet initialized\n", __func__);
		return 0;
	}

	if (atomic_read(&panel->esd_recovery_pending)) {
		DSI_DEBUG("ESD recovery pending\n");
		return 0;
	}

	pdisp = container_of(panel->host, struct dsi_display, host);
	if (!pdisp) {
		pr_err("%s: dsi_display is NULL.\n", __func__);
		return -EINVAL;
	}

	if (panel->bias_register == voltage_reg) {
		pr_debug("%s: already bias transfered=0x%02x\n",__func__, voltage_reg);
		return 0;
	}

	voltage_buf[1] = voltage_reg;

	rc = drm_cmn_dsi_cmds_transfer(pdisp, voltage_cmd,
		ARRAY_SIZE(voltage_cmd));
	if (rc < 0) {
		pr_err("%s: error bias transfer\n", __func__);
		return rc;
	}
	pr_debug("%s: bias transfer(0x%02x) to (0x%02x)\n", 
			__func__, panel->bias_register, voltage_reg);
	panel->bias_register = voltage_reg;

	return rc;
}

static int drm_bias_calc_temperature(int temperature)
{
	int kelvin_value;

	if (temperature < BIAS_MIN_VALUE_TEMPERATURE ||
			BIAS_MAX_VALUE_TEMPERATURE < temperature) {
		pr_err("%s: temperature is out of range(%d)\n", __func__, temperature);
		return -EINVAL;
	}

	kelvin_value = (temperature + BIAS_CONVERT_VALUE_TO_KELVIN + 500) / 1000;
	pr_debug("%s: temperature(%d) to kelvin(%d)\n",
			__func__, temperature, kelvin_value);
	return kelvin_value;
}

static int drm_bias_proc_best_bias(struct dsi_panel *panel)
{
	int rc = 0;
	int Y = 0;
	unsigned char bias_reg = 0;

	Y = drm_bias_calc_best_bias(panel);
	if (Y < 0) {
		pr_err("%s: error calc_duty%d\n",__func__, rc);
		return -EINVAL;
	}
	bias_reg = drm_bias_get_register_for_voltage(Y);
	if (bias_reg == BIAS_VOLTAGES_ERROR_VALUE) {
		pr_err("%s: error bias_reg=0x%02x\n", __func__, bias_reg);
		return -EINVAL;
	}
	rc = drm_bias_send_bias_voltage(panel, bias_reg);
	return rc;
}

static int drm_bias_calc_best_bias(struct dsi_panel *panel)
{
	int Y = 0;
	int x1 = 0;
	int x2 = 0;
	int x3 = 0;
	int voltage = 0;
	int temperature = 0;
	int gradation = 0;
	unsigned char otp = 0;
	struct drm_bias_calc_param param;

	if (!panel) {
		pr_err("%s:dsi_panel is null\n", __func__);
		return -EINVAL;
	}

	if (panel->mfr == 60) {
		gradation = BIAS_GRADATION_HIGH;
	} else {
		if (drm_get_hist1hz()) {
			gradation = BIAS_GRADATION_HIGH;
		} else {
			gradation = BIAS_GRADATION_LOW;
		}
	}

	x1 = drm_bias_calc_x1(panel->duty,gradation);
	if (x1 < 0) {
		pr_err("%s: error x1=%d\n",__func__, x1);
		return -EINVAL;
	}
	

	x2 = drm_bias_calc_x2(panel->mfr);
	if (x2 < 0) {
		pr_err("%s: error x2=%d\n",__func__, x2);
		return -EINVAL;
	}
	
	otp = drm_bias_get_otp_voltage();
	voltage = drm_bias_get_voltage_for_register(otp);
	if (voltage < 0) {
		pr_err("%s: error voltage=%d\n",__func__, voltage);
		return -EINVAL;
	}
	

	temperature = drm_bias_calc_temperature(drm_bias_ctx.display_therm_temperature);
	if (temperature < 0) {
		pr_err("%s: error temperature=%d\n",__func__, temperature);
		return -EINVAL;
	}
	x3 = temperature;

	param.x1	      = x1;
	param.x2	      = x2;
	param.x3	      = x3;
	param.vob_otp     = voltage;
	param.gradation   = gradation;

	Y = drm_bias_bias_calc(&param);

	return Y;
}

static int drm_bias_get_display_thermal(int *temperature, struct dsi_panel *panel)
{
	int rc = 0;
    int temp = -ENODATA;

	if (!panel || !panel->parent) {
		pr_err("%s: Invalid dsi_panel\n", __func__);
		return -EINVAL;
	}

	if (!panel->display_therm) {
		pr_err("%s: Invalid display_therm\n", __func__);
		return -ENODATA;
	}

	rc = iio_read_channel_processed(panel->display_therm, &temp);
	if (rc < 0) {
		pr_err("%s: Error in reading display_therm, rc:%d\n", __func__, rc);
		return rc;
	}
	if (temp < BIAS_MIN_VALUE_TEMPERATURE || BIAS_MAX_VALUE_TEMPERATURE < temp) {
		pr_err("%s: temperature is out of range(%d)\n", __func__, temp);
		return -ERANGE;
	}
	*temperature = temp;

	pr_debug("%s: read temperature(%d)\n", __func__, temp);

	return 0;
}

static void drm_bias_display_therm_delayed_work(struct work_struct *therm_delayedwk)
{
	int rc = 0;
	int temperature = 0;
	int kelvin_pre = 0;
	int kelvin_now = 0;

	pr_debug("%s: in function\n", __func__);

	if (!drm_bias_ctx.display || !drm_bias_ctx.display->panel) {
		pr_err("%s: invalid input\n", __func__);
		return;
	}

	rc = drm_bias_get_display_thermal(&temperature, drm_bias_ctx.display->panel);
	if (!rc) {
		if (drm_bias_ctx.display_therm_temperature != temperature) {
			kelvin_now = drm_bias_calc_temperature(temperature);
			kelvin_pre = drm_bias_calc_temperature(drm_bias_ctx.display_therm_temperature);
			mutex_lock(&drm_bias_ctx.display->panel->panel_lock);
			drm_bias_ctx.display_therm_temperature = temperature;
			if (kelvin_pre != kelvin_now) {
				pr_debug("%s: update temperature (%d) to (%d)\n",
					 __func__, kelvin_pre, kelvin_now);
				kelvin_pre = kelvin_now;
				drm_bias_best_bias_setting(drm_bias_ctx.display->panel);
			}
			mutex_unlock(&drm_bias_ctx.display->panel->panel_lock);
		}
	}

	rc = queue_delayed_work(drm_bias_ctx.therm_delayedwkq,
		&drm_bias_ctx.therm_delayedwk,
		msecs_to_jiffies(BIAS_WAIT_TIME_LCD_THERM));
	if (rc == 0) {
		pr_debug("%s:failed to queue_work(). rc=%d\n", __func__, rc);
	}

	pr_debug("%s: out function\n", __func__);
	return;
}

static void drm_bias_start_notify_therm(void)
{
	int rc = 0;
	int temperature = 0;

	pr_debug("%s: in function\n", __func__);

	if (!drm_bias_ctx.display || !drm_bias_ctx.display->panel) {
		pr_err("%s: invalid input\n", __func__);
		return;
	}

	rc = drm_bias_get_display_thermal(&temperature, drm_bias_ctx.display->panel);
	mutex_lock(&drm_bias_ctx.display->panel->panel_lock);
	if (!rc) {
		drm_bias_ctx.display_therm_temperature = temperature;
	} else {
		drm_bias_ctx.display_therm_temperature = BIAS_INVALID_VALUE_TEMPERATURE;
	}
	mutex_unlock(&drm_bias_ctx.display->panel->panel_lock);

	rc = queue_delayed_work(drm_bias_ctx.therm_delayedwkq,
		&drm_bias_ctx.therm_delayedwk,
		msecs_to_jiffies(BIAS_WAIT_TIME_LCD_THERM));
	if (rc == 0) {
		pr_debug("%s:failed to queue_work(). rc=%d\n", __func__, rc);
	}

	pr_debug("%s: out function\n", __func__);
	return;
}

static void drm_bias_stop_notify_therm(void)
{
	pr_debug("%s: in function\n", __func__);

	if (!drm_bias_ctx.display || !drm_bias_ctx.display->panel) {
		pr_err("%s: invalid input\n", __func__);
		return;
	}

	if (cancel_delayed_work_sync(&drm_bias_ctx.therm_delayedwk) == true) {
		pr_debug("%s: cancel_delayed_work done.\n", __func__);
	}
	mutex_lock(&drm_bias_ctx.display->panel->panel_lock);
	drm_bias_ctx.display_therm_temperature = BIAS_INVALID_VALUE_TEMPERATURE;
	mutex_unlock(&drm_bias_ctx.display->panel->panel_lock);

	pr_debug("%s: out function\n", __func__);
	return;
}

void drm_bias_enable(bool en)
{
	if (en == drm_bias_ctx.enable) {
		pr_debug("%s: Onbias enable/disable is not changed. en=%d, enable=%d.\n",
				__func__, en, drm_bias_ctx.enable);
		return;
	}
	
	if (en) {
		drm_bias_ctx.enable = true;
		drm_bias_start_notify_therm();
	} else {
		drm_bias_ctx.enable = false;
		drm_bias_stop_notify_therm();
	}

	return;
}
void drm_bias_init(struct msm_drm_private *priv)
{
	struct msm_kms *kms = NULL;
	struct sde_kms *sde_kms = NULL;

	if (priv != NULL) {
		memset(&drm_bias_ctx, 0x00, sizeof(struct drm_bias_ctx));

		drm_bias_ctx.display_therm_temperature = BIAS_INVALID_VALUE_TEMPERATURE;

		drm_bias_ctx.therm_delayedwkq = create_singlethread_workqueue("drm_bias");
		if (IS_ERR_OR_NULL(drm_bias_ctx.therm_delayedwkq)) {
			pr_err("Error creating therm_delayedwkq\n");
			return;
		}

		kms = priv->kms;
		if (kms != NULL) {
			sde_kms = to_sde_kms(kms);
			if ((sde_kms->dsi_display_count > 0) &&
			    (sde_kms->dsi_displays != NULL)) {
				drm_bias_ctx.display = sde_kms->dsi_displays[DSI_PRIMARY];
				if (drm_bias_ctx.display == NULL) {
					pr_err("[%s]failed to display is null\n", __func__);
				}
			} else {
				pr_err("[%s]failed to dsi_displays count=%d addr=%p\n",
					__func__, sde_kms->dsi_display_count,
					sde_kms->dsi_displays);
			}
		} else {
			pr_err("[%s]failed to kms is null\n", __func__);
		}

		INIT_DELAYED_WORK(&drm_bias_ctx.therm_delayedwk, drm_bias_display_therm_delayed_work);
	} else {
		pr_err("[%s]failed to priv is null\n", __func__);
	}

	return;
}
