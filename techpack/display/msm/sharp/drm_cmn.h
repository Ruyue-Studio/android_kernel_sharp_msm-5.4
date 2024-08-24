/*
 *
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

#ifndef DRM_CMN_H
#define DRM_CMN_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#define __DSI_PLL_H
#include "../dsi/dsi_display.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* STRUCTURE                                                                 */
/* ------------------------------------------------------------------------- */
struct wake_lock {
	struct wakeup_source *ws;
};

/* ------------------------------------------------------------------------- */
/* EXTERN                                                                    */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* FUNCTION                                                                  */
/* ------------------------------------------------------------------------- */
void drm_cmn_init(void);
int drm_cmn_video_transfer_ctrl(struct dsi_display *pdisp, u8 onoff);
int drm_cmn_stop_video(struct dsi_display *pdisp);
int drm_cmn_start_video(struct dsi_display *pdisp);
int drm_cmn_dsi_cmds_transfer(struct dsi_display *pdisp,
			struct dsi_cmd_desc cmds[], int cmd_cnt);
int drm_cmn_panel_cmds_transfer(struct dsi_display *pdisp,
			struct dsi_cmd_desc cmds[], int cmd_cnt);
int drm_cmn_panel_cmds_transfer_videoctrl(struct dsi_display *pdisp,
			struct dsi_cmd_desc cmds[], int cmd_cnt);
int drm_cmn_panel_dcs_write0(struct dsi_display *pdisp, char addr);
int drm_cmn_panel_dcs_write1(struct dsi_display *pdisp,
			char addr, char data);
int drm_cmn_panel_dcs_read(struct dsi_display *pdisp, char addr,
			int rlen, char *rbuf);
int drm_cmn_dsi_dcs_read(struct dsi_display *pdisp, char addr,
				int rlen, char *rbuf);
int drm_cmn_get_panel_type(void);

bool drm_cmn_is_diag_mode(void);

int drm_cmn_get_hw_revision(void);
unsigned char drm_cmn_get_hw_handset(void);
unsigned char drm_cmn_get_upperunit(void);
unsigned char drm_cmn_get_disp_on_status(void);
unsigned char drm_cmn_get_otp_bias(void);
unsigned char drm_cmn_get_otp_vgsp(void);
unsigned char drm_cmn_get_otp_gamma_shift(void);

/* ------------------------------------------------------------------------- */
/* A wake_lock prevents the system from entering suspend or other low power
 * states when active. If the type is set to WAKE_LOCK_SUSPEND, the wake_lock
 * prevents a full system suspend.
 */
/* ------------------------------------------------------------------------- */
static inline int drm_cmn_wake_lock_init(struct wake_lock *lock, struct device *dev,
				  const char *name)
{
	if (!lock) {
		pr_err("%s: lock is NULL", __func__);
		return -EINVAL;
	}

	lock->ws = wakeup_source_register(dev, name);
	if(!lock->ws){
		pr_err("%s: not registered wakelock", __func__);
		return -ENOMEM;
	}
	return 0;
}

static inline void drm_cmn_wake_lock_destroy(struct wake_lock *lock)
{
	if (!lock || !lock->ws) {
		pr_err("%s: lock is NULL", __func__);
		return;
	}

	wakeup_source_unregister(lock->ws);
	lock->ws = NULL;
}

static inline void drm_cmn_wake_lock(struct wake_lock *lock)
{
	if (!lock || !lock->ws) {
		pr_err("%s: lock is NULL", __func__);
		return;
	}

	__pm_stay_awake(lock->ws);
}

static inline void drm_cmn_wake_lock_timeout(struct wake_lock *lock, long timeout)
{
	if (!lock || !lock->ws) {
		pr_err("%s: lock is NULL", __func__);
		return;
	}

	__pm_wakeup_event(lock->ws, jiffies_to_msecs(timeout));
}

static inline void drm_cmn_wake_unlock(struct wake_lock *lock)
{
	if (!lock || !lock->ws) {
		pr_err("%s: lock is NULL", __func__);
		return;
	}

	__pm_relax(lock->ws);
}

static inline int drm_cmn_wake_lock_active(struct wake_lock *lock)
{
	if (!lock) {
		pr_err("%s: lock is NULL", __func__);
		return -EINVAL;
	}

	return lock->ws->active;
}

void drm_cmn_set_wakelock(struct wake_lock *wakelock, bool en);

#endif /* DRM_CMN_H */
