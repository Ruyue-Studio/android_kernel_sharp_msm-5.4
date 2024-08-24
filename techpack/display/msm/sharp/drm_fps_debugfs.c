/*
 * Copyright (C) 2018 SHARP CORPORATION
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
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include "../sde/sde_crtc.h"
#include "../dsi/dsi_display.h"
#include "drm_fps_debugfs.h"

struct drm_mdp_fps_ctx {
	struct sde_crtc *sde_crtc;
	int fps;
	u64 commit_cnt;
	struct timespec tm;
	struct work_struct fps_work;
	struct timer_list fps_timer;
};
struct drm_mdp_fps_ctx fps_ctx;
static struct dsi_display *display_ctx = NULL;

int check_duration = 500;

module_param_named(check_duration, check_duration, int, S_IRUGO | S_IWUSR | S_IWGRP);

static void fps_check_timer_cb(struct timer_list *t)
{
	struct drm_mdp_fps_ctx *ctx = &fps_ctx;
	u32 cnt;
	struct timespec now;
	u32 duration;

	getnstimeofday(&now);

	pr_debug("%s commit_cnt = %llu, play_count = %llu\n",
		__func__, ctx->commit_cnt, ctx->sde_crtc->play_count);

	if (ctx->commit_cnt) {
		cnt = ctx->sde_crtc->play_count - ctx->commit_cnt;

		duration = (now.tv_sec - ctx->tm.tv_sec) * 1000;
		duration += (now.tv_nsec - ctx->tm.tv_nsec) / 1000 / 1000;

		ctx->fps = (1000 * cnt) / duration;
		pr_info("%s fps = %d\n", __func__, ctx->fps);

		schedule_work(&ctx->fps_work);
	}
	ctx->commit_cnt = ctx->sde_crtc->play_count;
	ctx->tm = now;

	mod_timer(&ctx->fps_timer, jiffies + msecs_to_jiffies(check_duration));
}

static ssize_t fps_check(struct file *file, const char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct msm_drm_private *priv = s->private;
	struct drm_crtc *crtc = NULL;
	struct sde_crtc *sde_crtc = NULL;

	char *buf;
	int param;
	struct drm_mdp_fps_ctx *ctx = &fps_ctx;

	if (!priv) {
		pr_err("%s: Invalid msm_drm_private is NULL\n", __func__);
		goto exit;
	}

#ifdef CONFIG_SHARP_DISPLAY
	if ((priv != NULL) && (priv->upper_unit_is_connected == DRM_UPPER_UNIT_IS_NOT_CONNECTED)) {
		pr_debug("%s: upper unit is not connected\n", __func__);
		goto exit;
	}
#endif /* CONFIG_SHARP_DISPLAY */

	crtc = priv->crtcs[0];
	if (!crtc) {
		pr_err("%s: Invalid drm_crtc is NULL\n", __func__);
		goto exit;
	}

	sde_crtc = to_sde_crtc(crtc);
	if (!sde_crtc) {
		pr_err("%s: Invalid sde_crtc is NULL\n", __func__);
		goto exit;
	}

	if (!display_ctx) {
		pr_err("%s: Invalid display data\n", __func__);
		goto exit;
	}

	if (!display_ctx->panel->panel_initialized) {
		pr_err("%s: display's power state is OFF\n", __func__);
		goto exit;
	}

	buf = kzalloc(sizeof(char) * count, GFP_KERNEL);
	if (!buf) {
		pr_err("%s: Failed to allocate buffer\n", __func__);
		goto exit;
	}

	if (copy_from_user(buf, ubuf, count)) {
		goto fail_free_all;
	}

	ctx->sde_crtc = sde_crtc;

	param = simple_strtol(buf, NULL, 10);

	pr_info("%s (%d)\n", __func__, param);

	cancel_work_sync(&ctx->fps_work);
	if (param) {
		timer_setup(&ctx->fps_timer, fps_check_timer_cb, 0);
		ctx->fps_timer.expires = jiffies + msecs_to_jiffies(8);
		add_timer(&ctx->fps_timer);
	}
	else {
		del_timer(&ctx->fps_timer);
		cancel_work_sync(&ctx->fps_work);
	}

fail_free_all:
	kfree(buf);
exit:

	return count;
}

static int fps_check_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, inode->i_private);
}

static const struct file_operations fps_check_fops = {
	.owner			= THIS_MODULE,
	.open			= fps_check_open,
	.write			= fps_check,
	.llseek			= seq_lseek,
	.release		= single_release,
};

void drm_fps_create_debugfs(struct msm_drm_private *priv)
{
	struct dentry *root;
	struct drm_mdp_fps_ctx *ctx = &fps_ctx;
	struct msm_kms *kms = NULL;
	struct sde_kms *sde_kms = NULL;

	pr_info("%s: create folder\n", __func__);

	if (!priv) {
		pr_err("%s: Invalid msm_drm_private is NULL\n", __func__);
	}
	else {
		kms = priv->kms;
		if (kms != NULL) {
			sde_kms = to_sde_kms(kms);
			if ((sde_kms->dsi_display_count > 0) &&
			    (sde_kms->dsi_displays != NULL)) {
				display_ctx = sde_kms->dsi_displays[DSI_PRIMARY];
				if (display_ctx == NULL) {
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
	}

	root = debugfs_create_dir("drm_debug", 0);

	if (!root) {
		pr_err("%s: dbgfs create dir failed\n", __func__);
	} else {
		if (!debugfs_create_file("fps_check", S_IWUSR, root, priv,
								&fps_check_fops)) {
			pr_err("%s: failed to create dbgfs fps check file\n",
								__func__);
			return;
		}
	}
	ctx->commit_cnt = 0;
}


