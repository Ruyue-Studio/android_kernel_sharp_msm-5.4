/*
 * Copyright (C) 2017 SHARP CORPORATION
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
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/notifier.h>
#include <linux/export.h>
#include <linux/debugfs.h>
#include <drm/sharp_drm.h>
#include "drm_notify.h"
#include "../dsi/dsi_display.h"
#include "../msm_drv.h"

static int show_blank_event_val = 1; /* panel_power_off = 1 */
static ssize_t drm_notify_show_blank_event(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t drm_show_restrict_fps(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t drm_store_restrict_fps(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len);
static struct dsi_display *display_ctx = NULL;
static ssize_t drm_show_drive_mode(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t drm_store_drive_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len);

/**
 * sysfs attribute
 */
static DEVICE_ATTR(show_blank_event, S_IRUGO, drm_notify_show_blank_event, NULL);
static DEVICE_ATTR(restrict_fps, S_IWUSR|S_IRUGO,
			 drm_show_restrict_fps, drm_store_restrict_fps);
static DEVICE_ATTR(drive_mode, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
			 drm_show_drive_mode, drm_store_drive_mode);
static struct attribute *drm_notify_attrs[] = {
	&dev_attr_show_blank_event.attr,
	&dev_attr_restrict_fps.attr,
	&dev_attr_drive_mode.attr,
	NULL
};

static struct attribute_group drm_notify_attr_group = {
    .name = "display",
	.attrs = drm_notify_attrs,
};

/**
 * sysfs create file
 */
int drm_notify_create_sysfs(struct dsi_display *display)
{
	int rc = 0;
	struct device *dev = NULL;

	if (!display->ctrl[0].ctrl || !display->ctrl[0].ctrl->pdev) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		return -EINVAL;
	}
	display_ctx = display;
	dev = &display->ctrl[0].ctrl->pdev->dev;
	if (dev) {
		pr_debug("%s: device_name = [%s]\n", __func__, dev->kobj.name);
		rc = sysfs_create_group(&dev->kobj,
					&drm_notify_attr_group);
		if (rc) {
			pr_err("%s: sysfs group creation failed, rc=%d\n", __func__, rc);
		}
	}

	/* display blank*/
	show_blank_event_val = 1;
	return rc;
}

/**
 * sysfs remove file
 */
void drm_notify_remove_sysfs(struct dsi_display *display)
{
	struct device *dev = NULL;

	if (!display->ctrl[0].ctrl || !display->ctrl[0].ctrl->pdev) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		return;
	}
	dev = &display->ctrl[0].ctrl->pdev->dev;
	if (dev) {
		sysfs_remove_group(&dev->kobj, &drm_notify_attr_group);
	}
}

/**
 * sysfs notifier
 */
void drm_sysfs_notifier(struct device *dev, int blank)
{
	pr_debug("%s: blank = %d start\n", __func__, blank);

	show_blank_event_val = blank;
	sysfs_notify(&dev->kobj, "display", "show_blank_event");
}

/**
 * sysfs notifier - sysfs update
 */
static ssize_t drm_notify_show_blank_event(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	pr_debug("%s: panel_power_on = %d\n", __func__, show_blank_event_val);

	ret = scnprintf(buf, PAGE_SIZE, "panel_power_on = %d\n",
						show_blank_event_val);
	return ret;
}

/**
 * sysfs notifier - cat
 */
static ssize_t drm_show_restrict_fps(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_display *display = NULL;
	struct msm_drm_private *priv = NULL;
	int ret = 0;

	display = display_ctx;
	if (!display || !display->drm_dev) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		return -EINVAL;
	}
	priv = display->drm_dev->dev_private;
	if (!priv) {
		return -EINVAL;
	}
	pr_debug("%s: restrict_fps = %d\n", __func__, priv->restrict_fps);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", priv->restrict_fps);
	return ret;
}

/**
 * sysfs notifier - echo
 */
static ssize_t drm_store_restrict_fps(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct dsi_display *display = NULL;
	struct msm_drm_private *priv = NULL;
	unsigned long val = 0;
	char *endp = NULL;

	display = display_ctx;
	if (!display || !display->drm_dev) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		return -EINVAL;
	}

	priv = display->drm_dev->dev_private;
	if (!priv) {
		return -EINVAL;
	}

	val = simple_strtoul(buf, &endp, 10);
	if (buf == endp)
		return -EINVAL;

	switch (val) {
		case 0:
		case DRM_BASE_FPS_30:
		case DRM_BASE_FPS_60:
			break;
		default:
			pr_err("%s: error restrict_fps %ld\n", __func__, val);
			return len;
	}

	pr_debug("%s: setting restrict_fps %ld\n", __func__, val);
	priv->restrict_fps = (int)val;
	return len;
}

/**
 * sysfs notifier
 */
void drm_notifier_drive_mode(struct device *dev)
{
	if (!dev) {
		pr_err("%s: Invalid device\n", __func__);
		return;
	}
	pr_debug("%s: in\n", __func__);
	sysfs_notify(&dev->kobj, "display", "drive_mode");
	pr_debug("%s: out\n", __func__);
}

/**
 * sysfs notifier - cat
 */
static ssize_t drm_show_drive_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct dsi_panel *panel = display_ctx->panel;

	if (!panel) {
		pr_err("%s: Invalid dsi_panel\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: drive_mode = %d\n", __func__, panel->drive_mode);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", panel->drive_mode);
	return ret;
}

/**
 * sysfs notifier - echo
 */
static ssize_t drm_store_drive_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
#if defined(CONFIG_DEBUG_FS)
	struct dsi_display *display = NULL;
	unsigned long val = 0;
	char *endp = NULL;
	int ret = 0;

	display = display_ctx;
	if (!display || !display->drm_dev) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		return -EINVAL;
	}

	val = simple_strtoul(buf, &endp, 10);
	if (buf == endp)
		return -EINVAL;

	ret = dsi_display_set_drive_mode(display, (enum msm_drive_mode)val);
	if (ret < 0) {
		pr_err("%s: error drive mode %ld\n", __func__, val);
		return len;
	}

	pr_debug("%s: setting drive mode %ld\n", __func__, val);
	return len;
#else
	return len;
#endif /* CONFIG_DEBUG_FS */
}
