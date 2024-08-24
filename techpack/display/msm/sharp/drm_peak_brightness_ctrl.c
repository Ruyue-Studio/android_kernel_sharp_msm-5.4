/*
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
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/ctype.h>
#include <linux/fb.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/fb.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include "../dsi/dsi_display.h"
#include "../msm_drv.h"
#include "../sde/sde_kms.h"
#include "drm_cmn.h"
#include <video/mipi_display.h>
/* ------------------------------------------------------------------------- */
/* FUNCTION                                                                  */
/* ------------------------------------------------------------------------- */
static void drm_peak_brightness_ctrl_send_cmd(struct dsi_display *display);
static void drm_peak_brightness_ctrl_work(struct work_struct *delayedwk);
static void drm_peak_brightness_ctrl_update_ctx(unsigned char req_mode);
static void drm_peak_brightness_ctrl_req_mode_chg(unsigned char req_mode);
static ssize_t drm_peak_brightness_ctrl_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t drm_peak_brightness_ctrl_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count);
static void drm_peak_brightness_ctrl_panel_enable(void);
static void drm_peak_brightness_ctrl_panel_disable(void);
static int drm_peak_brightness_ctrl_check_status(void);

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define DRM_PEAK_BRIGHTNESS_CTRL_PARAM_MAX      0x11
#define DRM_PEAK_BRIGHTNESS_CTRL_ON             0x10
#define DRM_PEAK_BRIGHTNESS_CTRL_OFF            0x00
#define DRM_PEAK_BRIGHTNESS_CTRL_SBS            0x01
#define DRM_PEAK_BRIGHTNESS_CTRL_IMMED          0x00
#define DRM_PEAK_BRIGHTNESS_CTRL_OTHER          0xEE
#define DRM_PEAK_BRIGHTNESS_CTRL_DONE           0x04
#define DRM_PEAK_BRIGHTNESS_CTRL_LEVEL_ON       (sizeof(value_table) / sizeof(value_table[0]) - 1)
#define DRM_PEAK_BRIGHTNESS_CTRL_LEVEL_OFF      0
#define DRM_PEAK_BRIGHTNESS_CTRL_WAIT           1000
static DEVICE_ATTR(peak_control, S_IWUSR | S_IRUGO,
			drm_peak_brightness_ctrl_show, drm_peak_brightness_ctrl_store);

/* ------------------------------------------------------------------------- */
/* STRUCTURE                                                                 */
/* ------------------------------------------------------------------------- */
struct drm_peak_brightness_ctrl_ctx {
	unsigned char now_level;
	unsigned char set_level;
	unsigned char set_stepbystep;
	unsigned char request_value;
	struct workqueue_struct *delayedwkq;
	struct delayed_work delayedwk;
	struct mutex peak_ctrl_mutexlock;
	struct dsi_display *display;
	bool enable;
};
static struct drm_peak_brightness_ctrl_ctx peak_brightness_ctrl_ctx = {0};
static struct attribute *peak_brightness_ctrl_attrs[] = {
	&dev_attr_peak_control.attr,
	NULL
};
static struct attribute_group peak_brightness_ctrl_attr_group = {
	.attrs = peak_brightness_ctrl_attrs,
};

static const unsigned int value_table[][2] = {
	{0x19, 0x110},
	{0x1A, 0x10D},
	{0x1B, 0x10B},
	{0x1C, 0x109},
	{0x1E, 0x107},
	{0x1F, 0x105},
	{0x20, 0x103},
	{0x22, 0x101},
	{0x23, 0xFF},
	{0x24, 0xFD},
	{0x26, 0xFB},
	{0x27, 0xF8},
	{0x28, 0xF6},
	{0x29, 0xF4},
	{0x2B, 0xF2},
	{0x2C, 0xF0},
	{0x2D, 0xEE},
	{0x2F, 0xEC},
	{0x30, 0xEA},
	{0x31, 0xE8},
	{0x33, 0xE6},
	{0x34, 0xE3},
	{0x35, 0xE1},
	{0x36, 0xDF},
	{0x38, 0xDD},
	{0x39, 0xDB},
	{0x3A, 0xD9},
	{0x3C, 0xD7},
	{0x3D, 0xD5},
	{0x3E, 0xD3},
	{0x40, 0xD1},
};

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void drm_peak_brightness_ctrl_send_cmd(struct dsi_display *display)
{
	int ret = 0;
	unsigned char addr_value_delta_y_max_page[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00};
	unsigned char addr_value_delta_y_max_index[] = {0x6F, 0x03};
	unsigned char addr_value_delta_y_max_data[] = {0xB1, 0x00, 0x00};
	unsigned char addr_value_vgsp_page[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01};
	unsigned char addr_value_vgsp_data[] = {0xB6, 0x00, 0x00};

	struct dsi_cmd_desc cmd[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE	, MIPI_DSI_MSG_UNICAST, 0, 0,
				sizeof(addr_value_delta_y_max_page), addr_value_delta_y_max_page, 0, NULL}, 0, 0},
		{{0, MIPI_DSI_GENERIC_LONG_WRITE	, MIPI_DSI_MSG_UNICAST, 0, 0,
				sizeof(addr_value_delta_y_max_index), addr_value_delta_y_max_index, 0, NULL}, 0, 0},
		{{0, MIPI_DSI_GENERIC_LONG_WRITE	, MIPI_DSI_MSG_UNICAST, 0, 0,
				sizeof(addr_value_delta_y_max_data), addr_value_delta_y_max_data, 0, NULL}, 0, 0},
		{{0, MIPI_DSI_GENERIC_LONG_WRITE	, MIPI_DSI_MSG_UNICAST, 0, 0,
				sizeof(addr_value_vgsp_page), addr_value_vgsp_page, 0, NULL}, 0, 0},
		{{0, MIPI_DSI_GENERIC_LONG_WRITE	, MIPI_DSI_MSG_UNICAST, 0, 0,
				sizeof(addr_value_vgsp_data), addr_value_vgsp_data, 0, NULL}, 1, 0},
		{{0, MIPI_DSI_GENERIC_LONG_WRITE	, MIPI_DSI_MSG_UNICAST, 0, 0,
				sizeof(addr_value_delta_y_max_page), addr_value_delta_y_max_page, 0, NULL}, 1, 0},
	};

	if (!display) {
		pr_err("%s: Invalid display data\n", __func__);
		return;
	}

	ret = drm_peak_brightness_ctrl_check_status();
	if (ret) {
		return;
	}

	pr_debug("%s: START Write now level (%d) set(%d)\n", __func__,
			peak_brightness_ctrl_ctx.now_level, peak_brightness_ctrl_ctx.set_level);

	addr_value_delta_y_max_data[1] = value_table[peak_brightness_ctrl_ctx.now_level][0] >> 8 & 0xFF;
	addr_value_delta_y_max_data[2] = value_table[peak_brightness_ctrl_ctx.now_level][0] & 0xFF;
	addr_value_vgsp_data[1] = value_table[peak_brightness_ctrl_ctx.now_level][1] >> 8 & 0xFF;
	addr_value_vgsp_data[2] = value_table[peak_brightness_ctrl_ctx.now_level][1] & 0xFF;
	ret = drm_cmn_dsi_cmds_transfer(display, cmd, ARRAY_SIZE(cmd));
	if (ret) {
		pr_err("%s: <RESULT_FAILURE> write flicker ret=%d num=0x%02X.\n",
				__func__, ret, peak_brightness_ctrl_ctx.now_level);
		return;
	}

	pr_debug("%s: END\n", __func__);
	return;
}

static void drm_peak_brightness_ctrl_work(struct work_struct *delayedwk)
{
	struct dsi_display *display;
	int ret = 0;

	pr_debug("%s: START\n", __func__);

	display = peak_brightness_ctrl_ctx.display;
	if (!display || !display->panel) {
		pr_err("%s: Invalid input\n", __func__);
		return;
	}

	dsi_panel_acquire_panel_lock(display->panel);
	mutex_lock(&peak_brightness_ctrl_ctx.peak_ctrl_mutexlock);
	if (peak_brightness_ctrl_ctx.now_level == peak_brightness_ctrl_ctx.set_level) {
		pr_debug("%s: val is no change\n", __func__);
		mutex_unlock(&peak_brightness_ctrl_ctx.peak_ctrl_mutexlock);
		dsi_panel_release_panel_lock(display->panel);
		return;
	}

	ret = drm_peak_brightness_ctrl_check_status();
	if (ret) {
		mutex_unlock(&peak_brightness_ctrl_ctx.peak_ctrl_mutexlock);
		dsi_panel_release_panel_lock(display->panel);
		return;
	}

	if (peak_brightness_ctrl_ctx.now_level > peak_brightness_ctrl_ctx.set_level) {
		peak_brightness_ctrl_ctx.now_level--;
	} else {
		peak_brightness_ctrl_ctx.now_level++;
	}
	drm_peak_brightness_ctrl_send_cmd(display);
	if (peak_brightness_ctrl_ctx.set_level != peak_brightness_ctrl_ctx.now_level) {
		ret = queue_delayed_work(peak_brightness_ctrl_ctx.delayedwkq,
				&peak_brightness_ctrl_ctx.delayedwk,
				msecs_to_jiffies(DRM_PEAK_BRIGHTNESS_CTRL_WAIT));
		if (ret == 0) {
			pr_debug("%s:failed to queue_work(). ret=%d\n", __func__, ret);
		}
	}

	mutex_unlock(&peak_brightness_ctrl_ctx.peak_ctrl_mutexlock);
	dsi_panel_release_panel_lock(display->panel);

	pr_debug("%s: END\n", __func__);
	return;
}

static void drm_peak_brightness_ctrl_update_ctx(unsigned char buf)
{
	if (buf & DRM_PEAK_BRIGHTNESS_CTRL_ON) {
		peak_brightness_ctrl_ctx.set_level = DRM_PEAK_BRIGHTNESS_CTRL_LEVEL_ON;
	} else {
		peak_brightness_ctrl_ctx.set_level = DRM_PEAK_BRIGHTNESS_CTRL_LEVEL_OFF;
	}

	if (buf & DRM_PEAK_BRIGHTNESS_CTRL_SBS) {
		peak_brightness_ctrl_ctx.set_stepbystep = DRM_PEAK_BRIGHTNESS_CTRL_SBS;
	} else {
		peak_brightness_ctrl_ctx.set_stepbystep = DRM_PEAK_BRIGHTNESS_CTRL_IMMED;
	}
	peak_brightness_ctrl_ctx.request_value = buf;
	return;
}

static void drm_peak_brightness_ctrl_req_mode_chg(unsigned char req_mode)
{
	struct dsi_display *display;
	int ret = 0;

	display = peak_brightness_ctrl_ctx.display;
	if (!display || !display->panel) {
		pr_err("%s: Invalid input\n", __func__);
		return;
	}

	mutex_lock(&peak_brightness_ctrl_ctx.peak_ctrl_mutexlock);
	drm_peak_brightness_ctrl_update_ctx(req_mode);

	if (peak_brightness_ctrl_ctx.now_level == peak_brightness_ctrl_ctx.set_level) {
		mutex_unlock(&peak_brightness_ctrl_ctx.peak_ctrl_mutexlock);
		pr_debug("%s: val is no change\n", __func__);
		return;
	}
	mutex_unlock(&peak_brightness_ctrl_ctx.peak_ctrl_mutexlock);

	if (!peak_brightness_ctrl_ctx.delayedwkq) {
		pr_err("%s: No delayedwkq\n");
		return;
	}

	if(!peak_brightness_ctrl_ctx.set_stepbystep) { 
		if (cancel_delayed_work_sync(&peak_brightness_ctrl_ctx.delayedwk) == true) {
			pr_debug("%s: cancel_delayed_work done.\n", __func__);
		}
		dsi_panel_acquire_panel_lock(display->panel);
		mutex_lock(&peak_brightness_ctrl_ctx.peak_ctrl_mutexlock);
		peak_brightness_ctrl_ctx.now_level = peak_brightness_ctrl_ctx.set_level;
		drm_peak_brightness_ctrl_send_cmd(display);
		mutex_unlock(&peak_brightness_ctrl_ctx.peak_ctrl_mutexlock);
		dsi_panel_release_panel_lock(display->panel);
		return;
	}

	pr_debug("%s: request work Control Level(%d)\n", __func__,
			peak_brightness_ctrl_ctx.set_level);

	if (cancel_delayed_work_sync(&peak_brightness_ctrl_ctx.delayedwk) == true) {
		pr_debug("%s: cancel_delayed_work done.\n", __func__);
	}
	ret = queue_delayed_work(peak_brightness_ctrl_ctx.delayedwkq,
			&peak_brightness_ctrl_ctx.delayedwk, msecs_to_jiffies(0));
	if (ret == 0) {
		pr_debug("%s:failed to queue_work(). ret=%d\n", __func__, ret);
	}

	return;
}

static ssize_t drm_peak_brightness_ctrl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char result_value = 0;

	mutex_lock(&peak_brightness_ctrl_ctx.peak_ctrl_mutexlock);
	result_value = peak_brightness_ctrl_ctx.request_value;

	if (peak_brightness_ctrl_ctx.now_level == peak_brightness_ctrl_ctx.set_level) {
		result_value |= DRM_PEAK_BRIGHTNESS_CTRL_DONE;
	}
	mutex_unlock(&peak_brightness_ctrl_ctx.peak_ctrl_mutexlock);

	return snprintf(buf, PAGE_SIZE, "0x%02X", result_value);
}

static ssize_t drm_peak_brightness_ctrl_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	long req_mode = DRM_PEAK_BRIGHTNESS_CTRL_OFF;
	int ret = 0;
	struct dsi_display *display;

	pr_debug("%s: START\n", __func__);

	display = peak_brightness_ctrl_ctx.display;
	if (!display || !display->panel) {
		pr_err("%s: Invalid input\n", __func__);
		goto exit;
	}

	if (kstrtol(buf, 0, &req_mode)) {
		pr_err("%s: parameter number error\n", __func__);
		goto exit;
	}

	if (req_mode & ~DRM_PEAK_BRIGHTNESS_CTRL_PARAM_MAX) {
		pr_err("%s: Invalid param\n", __func__);
		goto exit;
	}

	dsi_panel_acquire_panel_lock(display->panel);
	ret = drm_peak_brightness_ctrl_check_status();
	if (ret) {
		dsi_panel_release_panel_lock(display->panel);
		goto exit;
	}
	dsi_panel_release_panel_lock(display->panel);

	drm_peak_brightness_ctrl_req_mode_chg((unsigned char)req_mode);

exit:
	pr_debug("%s: END now_level=%d, set_level=%d\n", __func__,
			peak_brightness_ctrl_ctx.now_level, peak_brightness_ctrl_ctx.set_level);

	return count;
}

static void drm_peak_brightness_ctrl_panel_enable(void)
{
	pr_debug("%s: START\n", __func__);

	mutex_lock(&peak_brightness_ctrl_ctx.peak_ctrl_mutexlock);
	peak_brightness_ctrl_ctx.now_level = DRM_PEAK_BRIGHTNESS_CTRL_LEVEL_OFF;
	peak_brightness_ctrl_ctx.set_level = DRM_PEAK_BRIGHTNESS_CTRL_LEVEL_OFF;
	mutex_unlock(&peak_brightness_ctrl_ctx.peak_ctrl_mutexlock);
	
	pr_debug("%s: END\n", __func__);
	return;
}

static void drm_peak_brightness_ctrl_panel_disable(void)
{

	pr_debug("%s: START\n", __func__);

	if (peak_brightness_ctrl_ctx.delayedwkq) {
		if (cancel_delayed_work_sync(&peak_brightness_ctrl_ctx.delayedwk) == true) {
			pr_debug("%s: cancel_delayed_work done.\n", __func__);
		}
	}
	mutex_lock(&peak_brightness_ctrl_ctx.peak_ctrl_mutexlock);
	peak_brightness_ctrl_ctx.now_level = DRM_PEAK_BRIGHTNESS_CTRL_LEVEL_OFF;
	peak_brightness_ctrl_ctx.set_level = DRM_PEAK_BRIGHTNESS_CTRL_LEVEL_OFF;
	mutex_unlock(&peak_brightness_ctrl_ctx.peak_ctrl_mutexlock);

	pr_debug("%s: END\n", __func__);
	return;
}

static int drm_peak_brightness_ctrl_check_status(void)
{
	struct dsi_display *display;

	display = peak_brightness_ctrl_ctx.display;
	if (!display || !display->panel) {
		pr_err("%s: Invalid input\n", __func__);
		return -EINVAL;
	}

	if (!display->panel->panel_initialized) {
		pr_err("%s: Invalid panel status, panel_initialized = %d\n",
				__func__, display->panel->panel_initialized);
		return -EINVAL;
	}

	if (atomic_read(&display->panel->esd_recovery_pending)) {
		pr_err("%s: Invalid panel status, esd_recovery_pending = %d\n",
				__func__, display->panel->esd_recovery_pending);
		return -EINVAL;
	}

	if (!peak_brightness_ctrl_ctx.enable) {
		pr_debug("%s: Disable peak_brightness_ctrl, enable = %d\n",
				__func__, peak_brightness_ctrl_ctx.enable);
		return -EINVAL;
	}

	if (display->panel->drive_mode != MSM_DISPLAY_DRIVE_MODE_SUPER_BRIGHT) {
		pr_debug("%s: Current mode is not mode %d, mode = %d\n",
				__func__, MSM_DISPLAY_DRIVE_MODE_SUPER_BRIGHT, display->panel->drive_mode);
		return -EINVAL;
	}

	return 0;
}

void drm_peak_brightness_ctrl_enable(bool en)
{
	if (en == peak_brightness_ctrl_ctx.enable) {
		pr_debug("%s: peak_brightness_ctrl enable/disable is not changed. en=%d, peak_ctrl_enable=%d.\n",
				__func__, en, peak_brightness_ctrl_ctx.enable);
		return;
	}
	
	if (en) {
		peak_brightness_ctrl_ctx.enable = true;
		drm_peak_brightness_ctrl_panel_enable();
	} else {
		peak_brightness_ctrl_ctx.enable = false;
		drm_peak_brightness_ctrl_panel_disable();
	}
	return;
}

void drm_peak_brightness_ctrl_init(struct device *dev, struct msm_drm_private *priv)
{
	int rc;
	struct msm_kms *kms = NULL;
	struct sde_kms *sde_kms = NULL;

	if ((dev != NULL) &&
		(priv != NULL)) {
		rc = sysfs_create_group(&dev->kobj,
					&peak_brightness_ctrl_attr_group);
		if (rc) {
			pr_err("%s: sysfs group creation failed, rc=%d\n", __func__, rc);
		}

		memset(&peak_brightness_ctrl_ctx, 0x00, sizeof(struct drm_peak_brightness_ctrl_ctx));
		peak_brightness_ctrl_ctx.now_level = DRM_PEAK_BRIGHTNESS_CTRL_LEVEL_OFF;
		peak_brightness_ctrl_ctx.set_level = DRM_PEAK_BRIGHTNESS_CTRL_LEVEL_OFF;

		mutex_init(&peak_brightness_ctrl_ctx.peak_ctrl_mutexlock);

		peak_brightness_ctrl_ctx.delayedwkq = create_singlethread_workqueue("peak_brightness_ctrl");
		if (IS_ERR_OR_NULL(peak_brightness_ctrl_ctx.delayedwkq)) {
			pr_err("%s: Error creating delayedwkq\n");
			return;
		}

		kms = priv->kms;
		if (kms != NULL) {
			sde_kms = to_sde_kms(kms);
			if ((sde_kms->dsi_display_count > 0) &&
				(sde_kms->dsi_displays != NULL)) {
				peak_brightness_ctrl_ctx.display = sde_kms->dsi_displays[DSI_PRIMARY];
				if (peak_brightness_ctrl_ctx.display == NULL) {
					pr_err("%s: failed to display is null\n", __func__);
				}
			} else {
				pr_err("%s: failed to dsi_displays count=%d addr=%p\n",
					__func__, sde_kms->dsi_display_count,
					sde_kms->dsi_displays);
			}
		} else {
			pr_err("%s: failed to kms is null\n", __func__);
		}

		INIT_DELAYED_WORK(&peak_brightness_ctrl_ctx.delayedwk, drm_peak_brightness_ctrl_work);
	} else {
		pr_err("%s: failed to priv or dev is null\n", __func__);
	}

	return;
}
