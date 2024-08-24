/*
 * pt_mt_common.c
 * Parade TrueTouch(TM) Standard Product Multi-Touch Reports Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * TMA5XX
 * TMA448
 * TMA445A
 * TT21XXX
 * TT31XXX
 * TT4XXXX
 * TT7XXX
 * TC3XXX
 *
 * Copyright (C) 2015-2020 Parade Technologies
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Parade Technologies at www.paradetech.com <ttdrivers@paradetech.com>
 */

#include "pt_regs.h"
#include <linux/input/touchevent_notifier.h>

#define MT_PARAM_SIGNAL(md, sig_ost) PARAM_SIGNAL(md->pdata->frmwrk, sig_ost)
#define MT_PARAM_MIN(md, sig_ost) PARAM_MIN(md->pdata->frmwrk, sig_ost)
#define MT_PARAM_MAX(md, sig_ost) PARAM_MAX(md->pdata->frmwrk, sig_ost)
#define MT_PARAM_FUZZ(md, sig_ost) PARAM_FUZZ(md->pdata->frmwrk, sig_ost)
#define MT_PARAM_FLAT(md, sig_ost) PARAM_FLAT(md->pdata->frmwrk, sig_ost)

#define PT_MT_NOTIFIER_DIAG_TOUCH_ENABLE
#define PT_MT_SIMPLE_ORIENTATION_ENABLE

#define PT_MT_SYSFS_DEFINE(name, mode, show_func, store_func) \
static struct device_attribute pt_mt_sysfs_##name = \
	__ATTR(name, mode, show_func, store_func)

#define CALC_DIFF(x, y)			(((x) > (y)) ? ((x) - (y)) : ((y) - (x)))

#if defined(PT_MT_NOTIFIER_DIAG_TOUCH_ENABLE)
#define TOUCH_DIAGPOLL_TIME 100

struct touch_info{
	unsigned short	x;
	unsigned short	y;
	unsigned char	state;
	unsigned char	wx;
	unsigned char	wy;
	unsigned char	z;
};

struct touch_diag_info{
	u8							tm_mode;
	int							event;
	wait_queue_head_t			wait;
	struct touch_info			*fingers;
};
static struct touch_diag_info pt_mt_diag;

static ssize_t pt_mt_sysfs_poll_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;

	retval = wait_event_interruptible_timeout(pt_mt_diag.wait,
			pt_mt_diag.event == 1,
			msecs_to_jiffies(TOUCH_DIAGPOLL_TIME));

	if(0 == retval){
		/* time out */
		return -1;
	}

	return 0;
}
PT_MT_SYSFS_DEFINE(touch_poll, S_IRUGO,
			pt_mt_sysfs_poll_show,
			NULL);

static ssize_t pt_mt_sysfs_touch_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i;
	static char workbuf[512];
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_mt_data *md = &cd->md;
	struct pt_sysinfo *si = md->si;
	int max_tch = si->sensing_conf_data.max_tch;
	int len;

	if (pt_mt_diag.fingers == NULL) {
		return 0;
	}

	for (i = 0; i < max_tch; i++) {
		sprintf(workbuf, "id=%d,state=%d,x=%d,y=%d,wx=%d,wy=%d,z=%d\n",
							i,
							pt_mt_diag.fingers[i].state,
							pt_mt_diag.fingers[i].x,
							pt_mt_diag.fingers[i].y,
							pt_mt_diag.fingers[i].wx,
							pt_mt_diag.fingers[i].wy,
							pt_mt_diag.fingers[i].z);
		len = strlen(buf);
		strcpy(&buf[len], workbuf);
	}

	pt_mt_diag.event = 0;

	return( strlen(buf) );
}
PT_MT_SYSFS_DEFINE(touch_data, S_IRUGO,
			pt_mt_sysfs_touch_data_show,
			NULL);

#endif

#if defined(PARADE_LPWG_ENABLE)
static ssize_t pt_mt_sysfs_wake_event_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_mt_data *md = &cd->md;

	return scnprintf(buf, PAGE_SIZE, "%d,%d\n", md->lpwg.double_tap_x, md->lpwg.double_tap_y);
}
PT_MT_SYSFS_DEFINE(wake_event, S_IRUGO,
			pt_mt_sysfs_wake_event_show,
			NULL);
#endif /* PARADE_LPWG_ENABLE */

static struct device_attribute *pt_mt_attrs[] = {
#if defined(PT_MT_NOTIFIER_DIAG_TOUCH_ENABLE)
	&pt_mt_sysfs_touch_poll,
	&pt_mt_sysfs_touch_data,
#endif
#if defined(PARADE_LPWG_ENABLE)
	&pt_mt_sysfs_wake_event,
#endif /* PARADE_LPWG_ENABLE */
};

/*******************************************************************************
 * FUNCTION: pt_mt_lift_all
 *
 * SUMMARY: Reports touch liftoff action
 *
 * PARAMETERS:
 *     *md - pointer to touch data structure
 ******************************************************************************/
static void pt_mt_lift_all(struct pt_mt_data *md)
{
	int max = md->si->tch_abs[PT_TCH_T].max;

	if (md->num_prv_rec != 0) {
		if (md->mt_function.report_slot_liftoff)
			md->mt_function.report_slot_liftoff(md, max);
		input_sync(md->input);
		pt_debug(md->dev, DL_INFO,
			"%s: lift all done. num_prv_rec=%d\n",
			__func__, md->num_prv_rec);
		md->num_prv_rec = 0;
		touchevent_notifier_call_chain(0, NULL);
	}

#if defined(PT_MT_NOTIFIER_DIAG_TOUCH_ENABLE)
	if (pt_mt_diag.fingers) {
		int i;
		struct pt_sysinfo *si = md->si;
		int max_tch = si->sensing_conf_data.max_tch;

		for (i = 0; i < max_tch; i++) {
			pt_mt_diag.fingers[i].state = 0;
			pt_mt_diag.fingers[i].x = 0;
			pt_mt_diag.fingers[i].y = 0;
			pt_mt_diag.fingers[i].wx = 0;
			pt_mt_diag.fingers[i].wy = 0;
			pt_mt_diag.fingers[i].z = 0;
		}
	}
#endif
}

/*******************************************************************************
 * FUNCTION: pt_get_touch_axis
 *
 * SUMMARY: Calculates touch axis
 *
 * PARAMETERS:
 *     *md      - pointer to touch data structure
 *     *axis    - pointer to axis calculation result
 *      size    - size in byte
 *      max     - max value of result
 *     *xy_data - pointer to input data to be parsed
 *      bofs    - bit offset
 ******************************************************************************/
static void pt_get_touch_axis(struct pt_mt_data *md,
	int *axis, int size, int max, u8 *xy_data, int bofs)
{
	int nbyte;
	int next;

	for (nbyte = 0, *axis = 0, next = 0; nbyte < size; nbyte++) {
		pt_debug(md->dev, DL_DEBUG,
			"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p xy_data[%d]=%02X(%d) bofs=%d\n",
			__func__, *axis, *axis, size, max, xy_data, next,
			xy_data[next], xy_data[next], bofs);
		*axis = *axis + ((xy_data[next] >> bofs) << (nbyte * 8));
		next++;
	}

	*axis &= max - 1;

	pt_debug(md->dev, DL_DEBUG,
		"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p xy_data[%d]=%02X(%d)\n",
		__func__, *axis, *axis, size, max, xy_data, next,
		xy_data[next], xy_data[next]);
}

/*******************************************************************************
 * FUNCTION: pt_get_touch_hdr
 *
 * SUMMARY: Get the header of touch report
 *
 * PARAMETERS:
 *     *md      - pointer to touch data structure
 *     *touch   - pointer to pt_touch structure
 *     *xy_mode - pointer to touch mode data
 ******************************************************************************/
static void pt_get_touch_hdr(struct pt_mt_data *md,
	struct pt_touch *touch, u8 *xy_mode)
{
	struct device *dev = md->dev;
	struct pt_sysinfo *si = md->si;
	enum pt_tch_hdr hdr;

	for (hdr = PT_TCH_TIME; hdr < PT_TCH_NUM_HDR; hdr++) {
		if (!si->tch_hdr[hdr].report)
			continue;
		pt_get_touch_axis(md, &touch->hdr[hdr],
			si->tch_hdr[hdr].size,
			si->tch_hdr[hdr].max,
			xy_mode + si->tch_hdr[hdr].ofs,
			si->tch_hdr[hdr].bofs);
		pt_debug(dev, DL_DEBUG, "%s: get %s=%04X(%d)\n",
			__func__, pt_tch_hdr_string[hdr],
			touch->hdr[hdr], touch->hdr[hdr]);
	}

#if defined(PARADE_SUPPORT_DEGRADE_FLG_ENABLE)
	pt_debug(dev, DL_INFO,
		"%s: time=%X tch_num=%d lo=%d RNF=%d noise=%d BL_Init=%d DegDry=%d DegMoist=%d counter=%d\n",
		__func__,
		touch->hdr[PT_TCH_TIME],
		touch->hdr[PT_TCH_NUM],
		touch->hdr[PT_TCH_LO],
		touch->hdr[PT_TCH_RNF],
		touch->hdr[PT_TCH_NOISE],
		touch->hdr[PT_TCH_BL_INIT],
		touch->hdr[PT_TCH_DEGDRY],
		touch->hdr[PT_TCH_DEGMOIST],
		touch->hdr[PT_TCH_COUNTER]);
#else
	pt_debug(dev, DL_INFO,
		"%s: time=%X tch_num=%d lo=%d noise=%d counter=%d\n",
		__func__,
		touch->hdr[PT_TCH_TIME],
		touch->hdr[PT_TCH_NUM],
		touch->hdr[PT_TCH_LO],
		touch->hdr[PT_TCH_NOISE],
		touch->hdr[PT_TCH_COUNTER]);
#endif /* PARADE_SUPPORT_DEGRADE_FLG_ENABLE */
}

/*******************************************************************************
 * FUNCTION: pt_get_touch_record
 *
 * SUMMARY: Gets axis of touch report
 *
 * PARAMETERS:
 *     *md      - pointer to touch data structure
 *     *touch   - pointer to pt_touch structure
 *     *xy_data - pointer to touch data
 ******************************************************************************/
static void pt_get_touch_record(struct pt_mt_data *md,
	struct pt_touch *touch, u8 *xy_data)
{
	struct device *dev = md->dev;
	struct pt_sysinfo *si = md->si;
	enum pt_tch_abs abs;

	for (abs = PT_TCH_X; abs < PT_TCH_NUM_ABS; abs++) {
		if (!si->tch_abs[abs].report)
			continue;
		pt_get_touch_axis(md, &touch->abs[abs],
			si->tch_abs[abs].size,
			si->tch_abs[abs].max,
			xy_data + si->tch_abs[abs].ofs,
			si->tch_abs[abs].bofs);
		pt_debug(dev, DL_DEBUG, "%s: get %s=%04X(%d)\n",
			__func__, pt_tch_abs_string[abs],
			touch->abs[abs], touch->abs[abs]);
	}
}

#if defined(PT_MT_SIMPLE_ORIENTATION_ENABLE)
static int pt_mt_process_orientation(int orientation)
{
	orientation -= 128;
	if (orientation <= -128) {
		orientation = 255 + orientation;
	}

	if (orientation < 64 && orientation > -64) {
		orientation = 1;
	}
	else {
		orientation = 0;
	}
	return orientation;
}
#endif /* PT_MT_SIMPLE_ORIENTATION_ENABLE */


/*******************************************************************************
 * FUNCTION: pt_mt_process_touch
 *
 * SUMMARY: Process touch includes oritation,axis invert and
 *  convert MAJOR/MINOR from mm to resolution
 *
 * PARAMETERS:
 *     *md      - pointer to touch data structure
 *     *touch   - pointer to pt_touch structure
 ******************************************************************************/
static void pt_mt_process_touch(struct pt_mt_data *md,
	struct pt_touch *touch)
{
	struct device *dev = md->dev;
	struct pt_sysinfo *si = md->si;
	int tmp;
	bool flipped;


	/* Orientation is signed */
	touch->abs[PT_TCH_OR] = (int8_t)touch->abs[PT_TCH_OR];

	if (md->pdata->flags & PT_MT_FLAG_FLIP) {
		tmp = touch->abs[PT_TCH_X];
		touch->abs[PT_TCH_X] = touch->abs[PT_TCH_Y];
		touch->abs[PT_TCH_Y] = tmp;
		if (touch->abs[PT_TCH_OR] > 0)
			touch->abs[PT_TCH_OR] =
				md->or_max - touch->abs[PT_TCH_OR];
		else
			touch->abs[PT_TCH_OR] =
				md->or_min - touch->abs[PT_TCH_OR];
		flipped = true;
	} else
		flipped = false;

	/*
	 * 1 is subtracted from each touch location to make the location
	 * 0 based. e.g. If the resolution of touch panel is 1200x1600,
	 * the FW touch report must be (0~1199,0~1599). The driver
	 * should register the (min,max) value to Linux input system as
	 * (0~1199,0~1599). When the host needs to invert the
	 * coordinates, the driver would incorrectly use the resolution
	 * to subtract the reported point directly, such as
	 * 1200-(0~1199). The input system will lose the 0 point report
	 * and the 1200 point will be ignored.
	 */
	if (md->pdata->flags & PT_MT_FLAG_INV_X) {
		if (flipped)
			touch->abs[PT_TCH_X] = si->sensing_conf_data.res_y -
				touch->abs[PT_TCH_X] - 1;
		else
			touch->abs[PT_TCH_X] = si->sensing_conf_data.res_x -
				touch->abs[PT_TCH_X] - 1;
		touch->abs[PT_TCH_OR] *= -1;
	}
	if (md->pdata->flags & PT_MT_FLAG_INV_Y) {
		if (flipped)
			touch->abs[PT_TCH_Y] = si->sensing_conf_data.res_x -
				touch->abs[PT_TCH_Y] - 1;
		else
			touch->abs[PT_TCH_Y] = si->sensing_conf_data.res_y -
				touch->abs[PT_TCH_Y] - 1;
		touch->abs[PT_TCH_OR] *= -1;
	}

#if 0
	/* Convert MAJOR/MINOR from mm to resolution */
	tmp = touch->abs[PT_TCH_MAJ] * 100 * si->sensing_conf_data.res_x;
	touch->abs[PT_TCH_MAJ] = tmp / si->sensing_conf_data.len_x;
	tmp = touch->abs[PT_TCH_MIN] * 100 * si->sensing_conf_data.res_x;
	touch->abs[PT_TCH_MIN] = tmp / si->sensing_conf_data.len_x;
#endif

#if defined(PT_MT_SIMPLE_ORIENTATION_ENABLE)
	touch->abs[PT_TCH_OR] = pt_mt_process_orientation(touch->abs[PT_TCH_OR]);
#endif /* PT_MT_SIMPLE_ORIENTATION_ENABLE */

	pt_debug(dev, DL_INFO,
		"%s: flip=%s inv-x=%s inv-y=%s x=%04X(%d) y=%04X(%d)\n",
		__func__, flipped ? "true" : "false",
		md->pdata->flags & PT_MT_FLAG_INV_X ? "true" : "false",
		md->pdata->flags & PT_MT_FLAG_INV_Y ? "true" : "false",
		touch->abs[PT_TCH_X], touch->abs[PT_TCH_X],
		touch->abs[PT_TCH_Y], touch->abs[PT_TCH_Y]);
}

/*******************************************************************************
 * FUNCTION: pt_report_event
 *
 * SUMMARY: Reports touch event
 *
 * PARAMETERS:
 *     *md      - pointer to touch data structure
 *      event   - type of touch event
 *      value   - value of report event
 ******************************************************************************/
static void pt_report_event(struct pt_mt_data *md, int event,
		int value)
{
	int sig = MT_PARAM_SIGNAL(md, event);

	if (sig != PT_IGNORE_VALUE)
		input_report_abs(md->input, sig, value);
}

static void pt_common_init_parameter_after_initialize_baselines(struct pt_mt_data *md)
{
	struct pt_core_data *cd = dev_get_drvdata(md->dev);


#if defined(PARADE_SUPPORT_DEGRADE_FLG_ENABLE)
	md->degrade_flg_info.initialize_baselines_end_time_jiffies = jiffies;
#endif /* PARADE_SUPPORT_DEGRADE_FLG_ENABLE */

#if defined(PARADE_CLING_REJECT_ENABLE)
	if (cd->cling_reject_info.finger_info != NULL) {
		int t;
		for (t = 0; t < cd->cling_reject_info.finger_info_max; t++) {
			cd->cling_reject_info.finger_info[t].touch_check_enable = 0;
			cd->cling_reject_info.finger_info[t].repeat_touch_check_enable = 0;
			cd->cling_reject_info.finger_info[t].repeat_touch_check_time_index = 0;
		}
	}
#endif /* PARADE_CLING_REJECT_ENABLE */
#if defined(PARADE_REJECT_TU_CHATTERING_ENABLE)
	if (cd->reject_tu_chattering_info.finger_info != NULL) {
		int t;
		for (t = 0; t < cd->reject_tu_chattering_info.finger_info_max; t++) {
			cd->reject_tu_chattering_info.finger_info[t].check_status = 0;
		}
	}
#endif /* PARADE_REJECT_TU_CHATTERING_ENABLE */
#if defined(PARADE_DEGRADE_CLING_MEASURES_ENABLE)
	if (md->degrade_cling_measures_info.notch_finger_info != NULL) {
		int t;
		for (t = 0; t < md->degrade_cling_measures_info.finger_info_max; t++) {
			md->degrade_cling_measures_info.notch_finger_info[t].check_status = 0;
		}
	}
	if (md->degrade_cling_measures_info.lo_info != NULL) {
		int t;
		for (t = 0; t < md->degrade_cling_measures_info.finger_info_max; t++) {
			md->degrade_cling_measures_info.lo_info[t].check_skip = 0;
		}
		md->degrade_cling_measures_info.check_lo_skip = 0;
	}
	if (md->degrade_cling_measures_info.large_td_info != NULL) {
		int t;
		for (t = 0; t < md->degrade_cling_measures_info.finger_info_max; t++) {
			md->degrade_cling_measures_info.large_td_info[t].check_status = 0;
		}
	}
	md->degrade_cling_measures_info.detect_lo_flg = false;
	md->degrade_cling_measures_info.detect_finger_lo_flg = false;
#endif /* PARADE_DEGRADE_CLING_MEASURES_ENABLE */
}

#if defined( PARADE_FINGERPRINT_NOTIFY_SUPPRESS_TOUCHDOWN_ENABLE )
int pt_check_fingerprint_notify_suppress_touchdown(struct pt_mt_data *md, int t, int x, int y)
{
	struct pt_core_data *cd = dev_get_drvdata(md->dev);
	int is_suppress_touchdown = 0;

	if (time_after(jiffies, cd->fingerprint_notify_suppress_touchdown_notifytime + msecs_to_jiffies(PARADE_PRM_FINGERPRINT_NOTIFY_SUPPRESS_TOUCHDOWN_TIME_MS)) == true) {
		cd->fingerprint_notify_suppress_touchdown_enable = false;
	}
	else {
		if (pt_mt_diag.fingers[t].state == 0) {
			if (cd->fingerprint_aoi_req_state != 0) {
				if (cd->fingerprint_aoi_area.left < x && x < cd->fingerprint_aoi_area.right) {
					if (cd->fingerprint_aoi_area.top < y && y < cd->fingerprint_aoi_area.bottom) {
						is_suppress_touchdown = 1;
					}
				}
#if 1
				pt_debug(md->dev, DL_INFO, "%s: is_suppress_touchdown=%d (%d < x=%d < %d), (%d < y=%d < %d)\n", __func__,
					is_suppress_touchdown,
					cd->fingerprint_aoi_area.left, x, cd->fingerprint_aoi_area.right,
					cd->fingerprint_aoi_area.top, y, cd->fingerprint_aoi_area.bottom);
#endif
			}
		}
	}

	return is_suppress_touchdown;
}
#endif /* PARADE_FINGERPRINT_NOTIFY_SUPPRESS_TOUCHDOWN_ENABLE */

#if defined(PARADE_SUPPORT_DEGRADE_FLG_ENABLE)
static int pt_get_touchdown_count(struct pt_mt_data *md, int num_cur_tch)
{
	struct pt_sysinfo *si = md->si;
	int i, t = 0;
	u8 *tch_addr;
	struct pt_touch tch_local;
	struct pt_touch *tch = &tch_local;
	int touchdown_count = 0;

	for (i = 0; i < num_cur_tch; i++) {
		tch_addr = si->xy_data + (i * si->desc.tch_record_size);
		pt_get_touch_record(md, tch, tch_addr);

		/*  Discard proximity event */
		if (tch->abs[PT_TCH_O] == PT_OBJ_PROXIMITY) {
			continue;
		}

		/* Validate track_id */
		t = tch->abs[PT_TCH_T];
		if (t < md->t_min || t > md->t_max) {
			continue;
		}

		/* Lift-off */
		if (tch->abs[PT_TCH_E] == PT_EV_LIFTOFF) {
			continue;
		}

		touchdown_count++;
	}

	return touchdown_count;
}
static void pt_degrade_flg_initialize_baselines_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct pt_mt_support_degrade_flg_info *msdfi = container_of(dw, struct pt_mt_support_degrade_flg_info, degrade_flg_initialize_baselines_delayed_work);
	struct pt_mt_data *md = container_of(msdfi, struct pt_mt_data, degrade_flg_info);
	struct pt_core_data *cd = container_of(md, struct pt_core_data, md);
	u8 status;
	int rc;
	struct pt_core_commands *cmd;

	rc = pt_request_exclusive(cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR, "%s: Error on request exclusive r=%d\n",
				__func__, rc);

		md->degrade_flg_info.request_degrade_flg_initialize_baselines_flg = false;

		return;
	}

	if (md->degrade_flg_info.request_degrade_flg_initialize_baselines_flg == true) {
		if (md->is_suspended == false) {
			cmd = pt_get_commands();
			if (!cmd) {
				pt_debug(cd->dev, DL_ERROR, "%s: Error get_commands on initialize baselines\n",
					__func__);
			}
			else {
				pt_debug(cd->dev, DL_INFO, "%s: do initialize baselines.\n",
					__func__);
				/* suspend_scanning */
				rc = cmd->nonhid_cmd->suspend_scanning(cd->dev, false);

				/* initialize_baselines */
				rc = cmd->nonhid_cmd->initialize_baselines(cd->dev, false, PT_IB_SM_MUTCAP |
							PT_IB_SM_SELFCAP, &status);
				if (rc < 0) {
					pt_debug(cd->dev, DL_INFO, "%s: Error on initialize baselines rc=%d\n",
						__func__, rc);
				}

				pt_mt_lift_all(md);

				/* suspend_scanning */
				rc = cmd->nonhid_cmd->resume_scanning(cd->dev, false);

				md->degrade_flg_info.initialize_baselines_end_time_jiffies = jiffies;

				/* common init parameter */
				pt_common_init_parameter_after_initialize_baselines(md);

				if (PARADE_PRM_DEGRADE_CLING_MEASURES_FOS_ENABLE_PARAM_ENABLE != 0) {
					pt_debug(cd->dev, DL_INFO, "%s: request fos_enable(%d).\n",
						__func__, PARADE_PRM_DEGRADE_CLING_MEASURES_FOS_ENABLE_PARAM_ENABLE_TIME);
					pt_request_fos_enable_param_enable(cd, PARADE_PRM_DEGRADE_CLING_MEASURES_FOS_ENABLE_PARAM_ENABLE_TIME);
				}
			}
		}
		md->degrade_flg_info.request_degrade_flg_initialize_baselines_flg = false;
	}
	else {
		pt_debug(cd->dev, DL_INFO, "%s: Do noting by request_degrade_flg_initialize_baselines_flg false\n",
			__func__);
	}

	pt_release_exclusive(cd->dev);
	return;
}
#endif /* PARADE_SUPPORT_DEGRADE_FLG_ENABLE */

#if defined(PARADE_CLING_REJECT_ENABLE)
static void pt_cling_reject_initialize_baselines_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct pt_cling_reject_info *cling_reject_info = container_of(dw, struct pt_cling_reject_info, initialize_baselines_delayed_work);
	struct pt_core_data *cd = container_of(cling_reject_info, struct pt_core_data, cling_reject_info);
	struct pt_mt_data *md = &cd->md;
	u8 status;
	int rc;
	struct pt_core_commands *cmd;
	int t;

	mutex_lock(&md->mt_initialize_baselines_lock);

	if (cd->cling_reject_info.request_initialize_baselines_flg == true) {
		cmd = pt_get_commands();
		if (!cmd) {
			pt_debug(cd->dev, DL_INFO, "%s: Error get_commands on initialize baselines\n",
				__func__);
		}
		else {
			pt_debug(cd->dev, DL_INFO, "%s: do initialize baselines.\n",
				__func__);
			/* initialize_baselines */
			rc = cmd->nonhid_cmd->initialize_baselines(cd->dev, 1, PT_IB_SM_MUTCAP |
						PT_IB_SM_SELFCAP, &status);
			if (rc < 0) {
				pt_debug(cd->dev, DL_INFO, "%s: Error on initialize baselines rc=%d\n",
					__func__, rc);
			}

			pt_mt_lift_all(md);

			/* common init parameter */
			pt_common_init_parameter_after_initialize_baselines(md);
		}

		for (t = 0; t < cd->cling_reject_info.finger_info_max; t++) {
			cd->cling_reject_info.finger_info[t].touch_check_enable = 0;
			cd->cling_reject_info.finger_info[t].repeat_touch_check_enable = 0;
			cd->cling_reject_info.finger_info[t].repeat_touch_check_time_index = 0;
		}

		cd->cling_reject_info.request_initialize_baselines_flg = false;
	}

	mutex_unlock(&md->mt_initialize_baselines_lock);

	return;
}
static int pt_cling_reject_request_initialize_baselines(struct pt_mt_data *md)
{
	struct pt_core_data *cd = dev_get_drvdata(md->dev);
	if(cd->cling_reject_info.request_initialize_baselines_flg == false) {
		cd->cling_reject_info.request_initialize_baselines_flg = true;

		schedule_delayed_work(&cd->cling_reject_info.initialize_baselines_delayed_work, 0);

		pt_debug(cd->dev, DL_INFO, "%s: request_initialize_baselines\n",
			__func__);
	}
	return 0;
}
#endif /* PARADE_CLING_REJECT_ENABLE */

#if defined(PARADE_REJECT_TU_CHATTERING_ENABLE)
static void pt_reject_tu_chattering_notifier_td_finger(struct pt_mt_data *md, int t)
{
	struct pt_core_data *cd = dev_get_drvdata(md->dev);
	struct pt_reject_tu_chattering_touch_info *td_finger = &cd->reject_tu_chattering_info.finger_info[t].td_finger;

	pt_debug(cd->dev, DL_INFO, "%s: [reject_tu_chattering] hold event set t=%d x=%d y=%d\n",
		__func__, t, td_finger->x, td_finger->y);

	if (md->mt_function.input_report) {
		md->mt_function.input_report(md->input, ABS_MT_TRACKING_ID,
			t, PT_OBJ_STANDARD_FINGER);
	}
	pt_report_event(md, PT_ABS_X_OST, td_finger->x);
	pt_report_event(md, PT_ABS_Y_OST, td_finger->y);
	if (td_finger->obj == PT_OBJ_GLOVE) {
		pt_report_event(md, PT_ABS_P_OST, td_finger->z + 0x0100);
	}
	else {
		pt_report_event(md, PT_ABS_P_OST, td_finger->z);
	}
	pt_report_event(md, PT_ABS_MAJ_OST, td_finger->wx);
	pt_report_event(md, PT_ABS_MIN_OST, td_finger->wy);
	pt_report_event(md, PT_ABS_OR_OST, td_finger->ori);

	input_sync(md->input);
}

static int pt_reject_tu_chattering_update_tu_chattering_timer(struct pt_mt_data *md)
{
	struct pt_core_data *cd = dev_get_drvdata(md->dev);
	int t;

	long unsigned int current_jiffies = jiffies;
	long unsigned int expire_jiffies;
	long unsigned int next_expire_jiffies;

	cancel_delayed_work(&cd->reject_tu_chattering_info.timer_delayed_work);

	next_expire_jiffies = 0;
	for (t = 0; t < cd->reject_tu_chattering_info.finger_info_max; t++) {
		if (cd->reject_tu_chattering_info.finger_info[t].check_status == 2) {
			expire_jiffies = cd->reject_tu_chattering_info.finger_info[t].td_time + msecs_to_jiffies(PARADE_PRM_REJECT_TU_CHATTERING_TD_INHIBIT_TIME_MS);
			if(	(next_expire_jiffies == 0) ||
				(next_expire_jiffies > expire_jiffies)) {
				next_expire_jiffies = expire_jiffies;
			}
		}
	}
	if(next_expire_jiffies > 0) {
		pt_debug(cd->dev, DL_INFO, "%s: [reject_tu_chattering] update reject_tu_chattering_timer wait_jiffies=%d\n",
			__func__, next_expire_jiffies - current_jiffies);
		schedule_delayed_work(&cd->reject_tu_chattering_info.timer_delayed_work, next_expire_jiffies - current_jiffies);
	}

	return 0;
}
static void pt_reject_tu_chattering_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct pt_reject_tu_chattering_info *reject_tu_chattering_info = container_of(dw, struct pt_reject_tu_chattering_info, timer_delayed_work);
	struct pt_core_data *cd = container_of(reject_tu_chattering_info, struct pt_core_data, reject_tu_chattering_info);
	struct pt_mt_data *md = &cd->md;
	int t;

	mutex_lock(&md->mt_lock);

	for (t = 0; t < cd->reject_tu_chattering_info.finger_info_max; t++) {
		if (cd->reject_tu_chattering_info.finger_info[t].check_status == 2) {
			if (time_after_eq(jiffies, cd->reject_tu_chattering_info.finger_info[t].td_time + msecs_to_jiffies(PARADE_PRM_REJECT_TU_CHATTERING_TD_INHIBIT_TIME_MS)) == true) {
				pt_reject_tu_chattering_notifier_td_finger(md, t);
				cd->reject_tu_chattering_info.finger_info[t].check_status = 0;
			}
		}
	}

	pt_reject_tu_chattering_update_tu_chattering_timer(md);

	mutex_unlock(&md->mt_lock);

	return;
}
#endif /* PARADE_REJECT_TU_CHATTERING_ENABLE */

#if defined(PARADE_DEGRADE_CLING_MEASURES_ENABLE)
static void pt_degrade_cling_measures_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct pt_mt_degrade_cling_measures_info *mdcmi = container_of(dw, struct pt_mt_degrade_cling_measures_info, degrade_cling_measures_initialize_baselines_delayed_work);
	struct pt_mt_data *md = container_of(mdcmi, struct pt_mt_data, degrade_cling_measures_info);
	struct pt_core_data *cd = container_of(md, struct pt_core_data, md);
	u8 status;
	int rc;
	struct pt_core_commands *cmd;
	int t;

	rc = pt_request_exclusive(cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR, "%s: Error on request exclusive r=%d\n",
				__func__, rc);

		md->degrade_cling_measures_info.request_degrade_cling_measures_initialize_baselines_flg = false;

		return;
	}

	if (md->degrade_cling_measures_info.request_degrade_cling_measures_initialize_baselines_flg == true) {
		if (md->is_suspended == false) {
			cmd = pt_get_commands();
			if (!cmd) {
				pt_debug(cd->dev, DL_ERROR, "%s: Error get_commands on initialize baselines\n",
					__func__);
			}
			else {
				if (PARADE_PRM_DEGRADE_CLING_MEASURES_FOS_ENABLE_PARAM_ENABLE != 0) {
					pt_debug(cd->dev, DL_INFO, "%s: request fos_enable(%d).\n",
						__func__, PARADE_PRM_DEGRADE_CLING_MEASURES_FOS_ENABLE_PARAM_ENABLE_TIME);
					pt_request_fos_enable_param_enable(cd, PARADE_PRM_DEGRADE_CLING_MEASURES_FOS_ENABLE_PARAM_ENABLE_TIME);

					/* common init parameter */
					pt_common_init_parameter_after_initialize_baselines(md);
				}
				else {
					pt_debug(cd->dev, DL_INFO, "%s: do initialize baselines.\n",
						__func__);
					/* suspend_scanning */
					rc = cmd->nonhid_cmd->suspend_scanning(cd->dev, false);

					/* initialize_baselines */
					rc = cmd->nonhid_cmd->initialize_baselines(cd->dev, false, PT_IB_SM_MUTCAP |
								PT_IB_SM_SELFCAP, &status);
					if (rc < 0) {
						pt_debug(cd->dev, DL_INFO, "%s: Error on initialize baselines rc=%d\n",
							__func__, rc);
					}

					pt_mt_lift_all(md);

					/* suspend_scanning */
					rc = cmd->nonhid_cmd->resume_scanning(cd->dev, false);

					/* common init parameter */
					pt_common_init_parameter_after_initialize_baselines(md);
				}
			}
		}

		for (t = 0; t < md->degrade_cling_measures_info.finger_info_max; t++) {
			md->degrade_cling_measures_info.notch_finger_info[t].check_status = 0;
			md->degrade_cling_measures_info.lo_info[t].check_skip = 0;
			md->degrade_cling_measures_info.large_td_info[t].check_status = 0;
		}
		md->degrade_cling_measures_info.check_lo_skip = 0;
		md->degrade_cling_measures_info.detect_lo_flg = false;
		md->degrade_cling_measures_info.detect_finger_lo_flg = false;
		md->degrade_cling_measures_info.request_degrade_cling_measures_initialize_baselines_flg = false;
	}
	else {
		pt_debug(cd->dev, DL_INFO, "%s: Do noting by request_degrade_cling_measures_initialize_baselines_flg false\n",
			__func__);
	}

	pt_release_exclusive(cd->dev);
	return;
}
static int pt_degrade_cling_measures_request_initialize_baselines(struct pt_mt_data *md)
{
	struct pt_core_data *cd = dev_get_drvdata(md->dev);
	if(md->degrade_cling_measures_info.request_degrade_cling_measures_initialize_baselines_flg == false) {
		md->degrade_cling_measures_info.request_degrade_cling_measures_initialize_baselines_flg = true;

		schedule_delayed_work(&md->degrade_cling_measures_info.degrade_cling_measures_initialize_baselines_delayed_work, 0);

		pt_debug(cd->dev, DL_INFO, "%s: request_initialize_baselines\n",
			__func__);
	}
	return 0;
}
static int pt_degrade_cling_measures_check_notch_cling_finger(
	struct pt_mt_data *md,
	unsigned short	x,
	unsigned short	y,
	unsigned char	wx,
	unsigned char	wy,
	unsigned char	z
)
{
	if ((x >= PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_THRESH_X_MIN) &&
		(x <= PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_THRESH_X_MAX)) {
		if (y <= PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_THRESH_Y) {
			if ((wx >= PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_THRESH_WX) ||
				((z*100 / wx) <= PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_THRESH_ZWX)) {
#if 0
				if (wx >= PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_THRESH_WX) {
					pt_debug(md->dev, DL_INFO, "%s: [DEGRADE_CLING_MEASURES][NOTCH] wx(%d) >= %d\n",
						__func__, wx, PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_THRESH_WX);
				}
				else {
					pt_debug(md->dev, DL_INFO, "%s: [DEGRADE_CLING_MEASURES][NOTCH] ZWx(%d) <= %d\n",
						__func__, (z*100 / wx), PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_THRESH_ZWX);
				}
#endif
				return 1;
			}
		}
	}

	return 0;
}
static int pt_degrade_cling_measures_check_large_td_finger(struct pt_mt_data *md, struct pt_touch *cur_tch)
{
	unsigned char wx = cur_tch->abs[PT_TCH_OR] ? cur_tch->abs[PT_TCH_MAJ] : cur_tch->abs[PT_TCH_MIN];
	unsigned char wy = cur_tch->abs[PT_TCH_OR] ? cur_tch->abs[PT_TCH_MIN] : cur_tch->abs[PT_TCH_MAJ];

	if ((cur_tch->abs[PT_TCH_E] == PT_EV_TOUCHDOWN) &&
		(cur_tch->abs[PT_TCH_O] == PT_OBJ_STANDARD_FINGER)) {
		if ((wx >= PARADE_PRM_DEGRADE_CLING_MEASURES_LARGE_TD_THRESH_W) ||
			(wy >= PARADE_PRM_DEGRADE_CLING_MEASURES_LARGE_TD_THRESH_W)) {
#if 0
			pt_debug(md->dev, DL_INFO, "%s: [DEGRADE_CLING_MEASURES][LARGE_TD] wx(%d) or wy(%d) >= %d\n",
				__func__, wx, wy, PARADE_PRM_DEGRADE_CLING_MEASURES_LARGE_TD_THRESH_W);
#endif
			return 1;
		}
	}
	return 0;
}
static int pt_get_touchup_count(struct pt_mt_data *md, int num_cur_tch)
{
	struct pt_sysinfo *si = md->si;
	int i, t = 0;
	u8 *tch_addr;
	struct pt_touch tch_local;
	struct pt_touch *tch = &tch_local;
	int touchup_count = 0;

	for (i = 0; i < num_cur_tch; i++) {
		tch_addr = si->xy_data + (i * si->desc.tch_record_size);
		pt_get_touch_record(md, tch, tch_addr);

		/*  Discard proximity event */
		if (tch->abs[PT_TCH_O] == PT_OBJ_PROXIMITY) {
			continue;
		}

		/* Validate track_id */
		t = tch->abs[PT_TCH_T];
		if (t < md->t_min || t > md->t_max) {
			continue;
		}

		/* Lift-off */
		if (tch->abs[PT_TCH_E] == PT_EV_LIFTOFF) {
			touchup_count++;
		}
	}

	return touchup_count;
}
#endif /* PARADE_DEGRADE_CLING_MEASURES_ENABLE */

/*******************************************************************************
 * FUNCTION: pt_get_mt_touches
 *
 * SUMMARY: Parse and report touch event
 *
 * PARAMETERS:
 *     *md          - pointer to touch data structure
 *     *tch         - pointer to touch structure
 *      num_cur_tch - number of current touch
 ******************************************************************************/
static void pt_get_mt_touches(struct pt_mt_data *md,
		struct pt_touch *tch, int num_cur_tch)
{
	struct device *dev = md->dev;
	struct pt_sysinfo *si = md->si;
	int sig;
	int i, j, t = 0;
#if 1
	unsigned long *ids = NULL;
#else
	DECLARE_BITMAP(ids, si->tch_abs[PT_TCH_T].max);
#endif
	int mt_sync_count = 0;
	u8 *tch_addr;
	int fw_finger_num = 0;
#if defined( PARADE_FINGERPRINT_NOTIFY_SUPPRESS_TOUCHDOWN_ENABLE )
	struct pt_core_data *cd = dev_get_drvdata(dev);
#endif /* PARADE_FINGERPRINT_NOTIFY_SUPPRESS_TOUCHDOWN_ENABLE */
#if defined(PARADE_DEGRADE_CLING_MEASURES_ENABLE)
	int detect_finger_lo_count = 0;
	int detect_finger_count = 0;
#endif /* PARADE_DEGRADE_CLING_MEASURES_ENABLE */

#if 1
	ids = kzalloc(BITS_TO_LONGS(si->tch_abs[PT_TCH_T].max) * sizeof(unsigned long), GFP_KERNEL);
	if (!ids) {
		pt_debug(dev, DL_ERROR, "%s: ids kzalloc error\n", __func__);
		return;
	}
#endif
	bitmap_zero(ids, si->tch_abs[PT_TCH_T].max);
	memset(tch->abs, 0, sizeof(tch->abs));

#if defined(PARADE_CLING_REJECT_ENABLE)
	if (cd->cling_reject_info.is_running == true) {
		if (time_after(jiffies, cd->cling_reject_info.resume_time + msecs_to_jiffies(PARADE_PRM_CLING_REJECT_CHECK_TIME_MS)) == true) {
			cd->cling_reject_info.is_running = false;
		}
	}
#endif /* PARADE_CLING_REJECT_ENABLE */

#if defined(PARADE_DEGRADE_CLING_MEASURES_ENABLE)
	if ((PARADE_PRM_DEGRADE_CLING_MEASURES_FOS_ENABLE_PARAM_ENABLE == 0) || (cd->fos_enable_state == 0)) {
		if (PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_LO_ENABLE != 0) {
			if (PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_LO_SKIP_JOIN_FINGER_ENABLE != 0) {
				if (md->pre_fw_finger_num >= 2) {
					if((md->degrade_cling_measures_info.pre_fw_lo == false) &&
					   (md->degrade_cling_measures_info.detect_lo_flg == true)) {
						if (pt_get_touchup_count(md, num_cur_tch) > 0) {
							if (md->degrade_cling_measures_info.check_lo_skip == 0) {
								md->degrade_cling_measures_info.check_lo_skip = 1;
#if 0
								pt_debug(dev, DL_INFO, "%s: [DEGRADE_CLING_MEASURES] check skip LO.\n",
									__func__);
#endif
							}
						}
					}
				}
			}
		}
	}
#endif /* PARADE_DEGRADE_CLING_MEASURES_ENABLE */

	for (i = 0; i < num_cur_tch; i++) {
		tch_addr = si->xy_data + (i * si->desc.tch_record_size);
		pt_get_touch_record(md, tch, tch_addr);

		/*  Discard proximity event */
		if (tch->abs[PT_TCH_O] == PT_OBJ_PROXIMITY) {
			pt_debug(dev, DL_INFO,
				"%s: Discarding proximity event\n",
				__func__);
			continue;
		}

		/* Validate track_id */
		t = tch->abs[PT_TCH_T];
		if (t < md->t_min || t > md->t_max) {
			pt_debug(dev, DL_INFO,
				"%s: tch=%d -> bad trk_id=%d max_id=%d\n",
				__func__, i, t, md->t_max);
			if (md->mt_function.input_sync)
				md->mt_function.input_sync(md->input);
			mt_sync_count++;
			continue;
		}

		/* Lift-off */
		if (tch->abs[PT_TCH_E] == PT_EV_LIFTOFF) {
			pt_debug(dev, DL_INFO, "%s: t=%d e=%d lift-off\n",
				__func__, t, tch->abs[PT_TCH_E]);
#if defined(PARADE_CLING_REJECT_ENABLE)
			if (PARADE_PRM_CLING_REJECT_ENABLE != 0) {
				if (cd->cling_reject_info.finger_info != NULL) {
					cd->cling_reject_info.finger_info[t].touch_check_enable = 0;
					cd->cling_reject_info.finger_info[t].repeat_touch_check_enable = 0;
				}
			}
#endif /* PARADE_CLING_REJECT_ENABLE */
#if defined(PARADE_REJECT_TU_CHATTERING_ENABLE)
			if (PARADE_PRM_REJECT_TU_CHATTERING_ENABLE != 0) {
				if (cd->reject_tu_chattering_info.finger_info != NULL) {
					cd->reject_tu_chattering_info.finger_info[t].check_status = 1;
					cd->reject_tu_chattering_info.finger_info[t].tu_time = jiffies;

					// stop unnecessary timers
					pt_reject_tu_chattering_update_tu_chattering_timer(md);
				}
			}
#endif /* PARADE_REJECT_TU_CHATTERING_ENABLE */
#if defined(PARADE_DEGRADE_CLING_MEASURES_ENABLE)
			if ((PARADE_PRM_DEGRADE_CLING_MEASURES_FOS_ENABLE_PARAM_ENABLE == 0) || (cd->fos_enable_state == 0)) {
				if (PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_ENABLE != 0) {
					if (md->degrade_cling_measures_info.notch_finger_info != NULL) {
						if (md->degrade_cling_measures_info.notch_finger_info[t].check_status == 1) {
							if ((md->degrade_cling_measures_info.notch_finger_info[t].last_x >= PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_THRESH_X_MIN) &&
								(md->degrade_cling_measures_info.notch_finger_info[t].last_x <= PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_THRESH_X_MAX) &&
								(md->degrade_cling_measures_info.notch_finger_info[t].last_y <= PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_THRESH_Y)) {
								md->degrade_cling_measures_info.notch_finger_info[t].tu_count++;
								if (md->degrade_cling_measures_info.notch_finger_info[t].tu_count >= PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_TU_COUNT_MAX) {
									pt_debug(dev, DL_INFO, "%s: [DEGRADE_CLING_MEASURES] t=%d notch cling by TU count(%d). request initialize baselines.\n",
										__func__, t, md->degrade_cling_measures_info.notch_finger_info[t].tu_count);
									pt_degrade_cling_measures_request_initialize_baselines(md);
									md->degrade_cling_measures_info.notch_finger_info[t].check_status = 0;
								}
							}
						}
					}
				}
				if (md->degrade_cling_measures_info.lo_info != NULL) {
					md->degrade_cling_measures_info.lo_info[t].check_skip = 0;
				}
				if (PARADE_PRM_DEGRADE_CLING_MEASURES_LARGE_TD_ENABLE != 0) {
					if (md->degrade_cling_measures_info.large_td_info != NULL) {
						if (PARADE_PRM_DEGRADE_CLING_MEASURES_LARGE_TD_TU_ENABLE != 0) {
							if (md->degrade_cling_measures_info.large_td_info[t].check_status == 1) {
								pt_debug(dev, DL_INFO, "%s: [DEGRADE_CLING_MEASURES][LARGE_TD] [%d] detect Large TD TouchUp. request initialize baselines.\n",
									__func__, t);
								pt_degrade_cling_measures_request_initialize_baselines(md);
							}
						}
						md->degrade_cling_measures_info.large_td_info[t].check_status = 0;
					}
				}
			}
#endif /* PARADE_DEGRADE_CLING_MEASURES_ENABLE */
			goto pt_get_mt_touches_pr_tch;
		}

		fw_finger_num++;

		/* Process touch */
		pt_mt_process_touch(md, tch);

#if defined( PARADE_FINGERPRINT_NOTIFY_SUPPRESS_TOUCHDOWN_ENABLE )
		if (PARADE_PRM_FINGERPRINT_NOTIFY_SUPPRESS_TOUCHDOWN_ENABLE) {
			int is_suppress_touchdown = 0;

			if (cd->fingerprint_notify_suppress_touchdown_enable == true) {
				is_suppress_touchdown = pt_check_fingerprint_notify_suppress_touchdown(md, t, tch->abs[PT_TCH_X], tch->abs[PT_TCH_Y]);
				if (is_suppress_touchdown != 0) {
					pt_debug(dev, DL_INFO, "%s: t=%d x=%d y=%d fingerprint_notify_suppress_touchdown lift-off\n",
						__func__, t, tch->abs[PT_TCH_X], tch->abs[PT_TCH_Y]);
					tch->abs[PT_TCH_E] = PT_EV_LIFTOFF;
					goto pt_get_mt_touches_pr_tch;
				}
			}
		}
#endif /* PARADE_FINGERPRINT_NOTIFY_SUPPRESS_TOUCHDOWN_ENABLE */

#if defined(PARADE_CLING_REJECT_ENABLE)
		if (PARADE_PRM_CLING_REJECT_ENABLE != 0) {
			if (cd->cling_reject_info.finger_info != NULL) {
				if (cd->cling_reject_info.is_running == true) {
					if (cd->cling_reject_info.request_initialize_baselines_flg == false) {
						if (cd->cling_reject_info.finger_info[t].touch_check_enable == 0) {
							int check_flg = 0;
							if ((tch->abs[PT_TCH_O] == PT_OBJ_STANDARD_FINGER) &&
								((tch->abs[PT_TCH_MAJ] >= PARADE_PRM_CLING_REJECT_TOUCH_THRESH_W) ||
								 (tch->abs[PT_TCH_MIN] >= PARADE_PRM_CLING_REJECT_TOUCH_THRESH_W) ) &&
								(tch->abs[PT_TCH_P] <= PARADE_PRM_CLING_REJECT_TOUCH_THRESH_Z) ) {
								check_flg = 1;
							}
							else if ((tch->abs[PT_TCH_O] == PT_OBJ_GLOVE) &&
								((tch->abs[PT_TCH_MAJ] >= PARADE_PRM_CLING_REJECT_GLOVE_THRESH_W) ||
								 (tch->abs[PT_TCH_MIN] >= PARADE_PRM_CLING_REJECT_GLOVE_THRESH_W) ) &&
								(tch->abs[PT_TCH_P] <= PARADE_PRM_CLING_REJECT_GLOVE_THRESH_Z) ) {
								check_flg = 1;
							}

							if (check_flg == 1) {
								cd->cling_reject_info.finger_info[t].touch_check_enable = 1;
								cd->cling_reject_info.finger_info[t].touch_check_start_time = jiffies;
							}
						}
						else if (cd->cling_reject_info.finger_info[t].touch_check_enable == 1) {
							int check_flg = 0;
							if ((tch->abs[PT_TCH_O] == PT_OBJ_STANDARD_FINGER) &&
								((tch->abs[PT_TCH_MAJ] >= PARADE_PRM_CLING_REJECT_TOUCH_THRESH_W) ||
								 (tch->abs[PT_TCH_MIN] >= PARADE_PRM_CLING_REJECT_TOUCH_THRESH_W) ) &&
								(tch->abs[PT_TCH_P] <= PARADE_PRM_CLING_REJECT_TOUCH_THRESH_Z) ) {
								check_flg = 1;
							}
							else if ((tch->abs[PT_TCH_O] == PT_OBJ_GLOVE) &&
								((tch->abs[PT_TCH_MAJ] >= PARADE_PRM_CLING_REJECT_GLOVE_THRESH_W) ||
								 (tch->abs[PT_TCH_MIN] >= PARADE_PRM_CLING_REJECT_GLOVE_THRESH_W) ) &&
								(tch->abs[PT_TCH_P] <= PARADE_PRM_CLING_REJECT_GLOVE_THRESH_Z) ) {
								check_flg = 1;
							}

							if (check_flg == 1) {
								if (time_after(jiffies, cd->cling_reject_info.finger_info[t].touch_check_start_time + msecs_to_jiffies(PARADE_PRM_CLING_REJECT_TOUCH_THRESH_TIME_MS)) == true) {
									pt_cling_reject_request_initialize_baselines(md);
								}
							}
							else {
								cd->cling_reject_info.finger_info[t].touch_check_enable = 0;
							}
						}

						if (cd->cling_reject_info.finger_info[t].repeat_touch_check_enable == 0) {
							int check_flg = 0;
							if ((tch->abs[PT_TCH_O] == PT_OBJ_STANDARD_FINGER) &&
								((tch->abs[PT_TCH_MAJ] >= PARADE_PRM_CLING_REJECT_REPEAT_TOUCH_THRESH_W) ||
								 (tch->abs[PT_TCH_MIN] >= PARADE_PRM_CLING_REJECT_REPEAT_TOUCH_THRESH_W))) {
								check_flg = 1;
							}
							else if ((tch->abs[PT_TCH_O] == PT_OBJ_GLOVE) &&
								((tch->abs[PT_TCH_MAJ] >= PARADE_PRM_CLING_REJECT_REPEAT_GLOVE_THRESH_W) ||
								 (tch->abs[PT_TCH_MIN] >= PARADE_PRM_CLING_REJECT_REPEAT_GLOVE_THRESH_W))) {
								check_flg = 1;
							}

							if (check_flg == 1) {
								cd->cling_reject_info.finger_info[t].repeat_touch_check_enable = 1;
								cd->cling_reject_info.finger_info[t].repeat_touch_check_time[cd->cling_reject_info.finger_info[t].repeat_touch_check_time_index] = jiffies;

								if (cd->cling_reject_info.finger_info[t].repeat_touch_check_time_index >= PARADE_PRM_CLING_REJECT_REPEAT_TOUCH_THRESH_COUNT - 1) {
									if (time_after(jiffies, cd->cling_reject_info.finger_info[t].repeat_touch_check_time[0] + msecs_to_jiffies(PARADE_PRM_CLING_REJECT_REPEAT_TOUCH_THRESH_TIME_MS)) == false) {
										pt_cling_reject_request_initialize_baselines(md);
									}
									else {
										memmove(&cd->cling_reject_info.finger_info[t].repeat_touch_check_time[0], 
												&cd->cling_reject_info.finger_info[t].repeat_touch_check_time[1], 
												sizeof(cd->cling_reject_info.finger_info[t].repeat_touch_check_time[0]) * (PARADE_PRM_CLING_REJECT_REPEAT_TOUCH_THRESH_COUNT - 1) );
									}
								}
								else {
									cd->cling_reject_info.finger_info[t].repeat_touch_check_time_index++;
								}
							}
						}
					}
				}
			}
		}
#endif /* PARADE_CLING_REJECT_ENABLE */

#if defined(PARADE_DEGRADE_CLING_MEASURES_ENABLE)
		if ((PARADE_PRM_DEGRADE_CLING_MEASURES_FOS_ENABLE_PARAM_ENABLE == 0) || (cd->fos_enable_state == 0)) {
			if (PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_ENABLE != 0) {
				if (md->degrade_cling_measures_info.notch_finger_info != NULL) {
					if (md->degrade_cling_measures_info.notch_finger_info[t].check_status == 1) {
						if (time_after(jiffies, md->degrade_cling_measures_info.notch_finger_info[t].start_time + msecs_to_jiffies(PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_CHECK_TIME_MS)) == false) {
							int move_x = CALC_DIFF(tch->abs[PT_TCH_X], md->degrade_cling_measures_info.notch_finger_info[t].check_x);
							if ((tch->abs[PT_TCH_Y] <= PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_THRESH_Y) &&
								(move_x >= PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_THRESH_MOVE_X)) {
								pt_debug(dev, DL_INFO, "%s: [DEGRADE_CLING_MEASURES] t=%d x=%d y=%d notch cling by move_x(%d) >= %d. request initialize baselines.\n",
									__func__, t, tch->abs[PT_TCH_X], tch->abs[PT_TCH_Y], move_x, PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_THRESH_MOVE_X);
								pt_degrade_cling_measures_request_initialize_baselines(md);
								md->degrade_cling_measures_info.notch_finger_info[t].check_status = 0;
							}
						}
						else {
							md->degrade_cling_measures_info.notch_finger_info[t].check_status = 0;
						}

						md->degrade_cling_measures_info.notch_finger_info[t].last_x = tch->abs[PT_TCH_X];
						md->degrade_cling_measures_info.notch_finger_info[t].last_y = tch->abs[PT_TCH_Y];
					}
					if ((md->degrade_cling_measures_info.notch_finger_info[t].check_status == 0) &&
						(md->degrade_cling_measures_info.request_degrade_cling_measures_initialize_baselines_flg == false)) {

						unsigned char wx = tch->abs[PT_TCH_OR] ? tch->abs[PT_TCH_MAJ] : tch->abs[PT_TCH_MIN];
						unsigned char wy = tch->abs[PT_TCH_OR] ? tch->abs[PT_TCH_MIN] : tch->abs[PT_TCH_MAJ];
						if (pt_degrade_cling_measures_check_notch_cling_finger(md, 
																				tch->abs[PT_TCH_X],
																				tch->abs[PT_TCH_Y],
																				wx,
																				wy,
																				tch->abs[PT_TCH_P]) != 0) {
							md->degrade_cling_measures_info.notch_finger_info[t].check_status = 1;
							md->degrade_cling_measures_info.notch_finger_info[t].check_x = tch->abs[PT_TCH_X];
							md->degrade_cling_measures_info.notch_finger_info[t].check_y = tch->abs[PT_TCH_Y];
							md->degrade_cling_measures_info.notch_finger_info[t].last_x = tch->abs[PT_TCH_X];
							md->degrade_cling_measures_info.notch_finger_info[t].last_y = tch->abs[PT_TCH_Y];
							md->degrade_cling_measures_info.notch_finger_info[t].start_time = jiffies;
							md->degrade_cling_measures_info.notch_finger_info[t].tu_count = 0;
#if 0
							pt_debug(dev, DL_INFO, "%s: [DEGRADE_CLING_MEASURES] t=%d x=%d y=%d wx=%d wy=%d z=%d check start(%dms).\n",
								__func__, t, tch->abs[PT_TCH_X], tch->abs[PT_TCH_Y], wx, wy, tch->abs[PT_TCH_P], PARADE_PRM_DEGRADE_CLING_MEASURES_NOTCH_CHECK_TIME_MS);
#endif
						}
					}
				}
			}

			if (PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_LO_ENABLE != 0) {
				unsigned char wx = tch->abs[PT_TCH_OR] ? tch->abs[PT_TCH_MAJ] : tch->abs[PT_TCH_MIN];
				unsigned char wy = tch->abs[PT_TCH_OR] ? tch->abs[PT_TCH_MIN] : tch->abs[PT_TCH_MAJ];

				if (PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_LO_SKIP_JOIN_FINGER_ENABLE != 0) {
					if (md->pre_fw_finger_num >= 2) {
						if(((md->degrade_cling_measures_info.lo_info[t].last_wx < PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_FINGER_LO_THRESH_W) ||
						    (md->degrade_cling_measures_info.lo_info[t].last_wy < PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_FINGER_LO_THRESH_W)) &&
						   ((wx >= PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_FINGER_LO_THRESH_W) &&
						    (wy >= PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_FINGER_LO_THRESH_W))) {
							if (pt_get_touchup_count(md, num_cur_tch) > 0) {
								md->degrade_cling_measures_info.lo_info[t].check_skip = 1;
#if 0
								pt_debug(dev, DL_INFO, "%s: [DEGRADE_CLING_MEASURES] [%d] check skip FingerLO.\n",
									__func__, t);
#endif
							}
						}
					}
				}

				if (md->degrade_cling_measures_info.detect_lo_flg == true) {
					if ((md->degrade_cling_measures_info.check_lo_skip == 0) &&
					    (md->degrade_cling_measures_info.lo_info[t].check_skip == 0)) {
						pt_debug(dev, DL_INFO, "%s: [DEGRADE_CLING_MEASURES] [%d] detect LO and touch. request initialize baselines.\n",
							__func__, t);
						pt_degrade_cling_measures_request_initialize_baselines(md);
					}
				}

				if (PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_FINGER_LO_ENABLE != 0) {
					if ((tch->abs[PT_TCH_MAJ] >= PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_FINGER_LO_THRESH_W) &&
						(tch->abs[PT_TCH_MIN] >= PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_FINGER_LO_THRESH_W)) {
						if ((md->degrade_cling_measures_info.check_lo_skip == 0) &&
						    (md->degrade_cling_measures_info.lo_info[t].check_skip == 0)) {
							detect_finger_lo_count++;
						}
						else {
							detect_finger_count++;
						}
					}
					else {
						detect_finger_count++;
					}
				}

				md->degrade_cling_measures_info.lo_info[t].last_wx = wx;
				md->degrade_cling_measures_info.lo_info[t].last_wy = wy;
			}

			if (PARADE_PRM_DEGRADE_CLING_MEASURES_LARGE_TD_ENABLE != 0) {
				if (md->degrade_cling_measures_info.large_td_info != NULL) {
					if (md->degrade_cling_measures_info.large_td_info[t].check_status == 0) {
						if (pt_degrade_cling_measures_check_large_td_finger(md, tch) != 0) {
							md->degrade_cling_measures_info.large_td_info[t].check_status = 1;
							md->degrade_cling_measures_info.large_td_info[t].td_time = jiffies;
						}
						else {
							md->degrade_cling_measures_info.large_td_info[t].check_status = -1;
						}
					}
					else if (md->degrade_cling_measures_info.large_td_info[t].check_status == 1) {
						if (time_after(jiffies, md->degrade_cling_measures_info.large_td_info[t].td_time + msecs_to_jiffies(PARADE_PRM_DEGRADE_CLING_MEASURES_LARGE_TD_THRESH_TIME_MS)) == true) {
							pt_debug(dev, DL_INFO, "%s: [DEGRADE_CLING_MEASURES][LARGE_TD] [%d] detect Large TD (%dms). request initialize baselines.\n",
								__func__, t, PARADE_PRM_DEGRADE_CLING_MEASURES_LARGE_TD_THRESH_TIME_MS);
							pt_degrade_cling_measures_request_initialize_baselines(md);
							md->degrade_cling_measures_info.large_td_info[t].check_status = 0;
						}
					}
				}
			}

			if (PARADE_PRM_DEGRADE_CLING_MEASURES_NOSIZE_TD_ENABLE != 0) {
				if ((tch->abs[PT_TCH_E] == PT_EV_TOUCHDOWN) &&
					(tch->abs[PT_TCH_O] == PT_OBJ_STANDARD_FINGER) &&
					((tch->abs[PT_TCH_MAJ] == 0) && (tch->abs[PT_TCH_MIN] == 0))) {
					pt_debug(dev, DL_INFO, "%s: [DEGRADE_CLING_MEASURES][NOSIZE_TD] [%d] detect NoSize TD(wx=0 and wy=0). request initialize baselines.\n",
						__func__, t);
					pt_degrade_cling_measures_request_initialize_baselines(md);
				}
			}
		}
#endif /* PARADE_DEGRADE_CLING_MEASURES_ENABLE */

#if defined(PARADE_REJECT_TU_CHATTERING_ENABLE)
		if (PARADE_PRM_REJECT_TU_CHATTERING_ENABLE != 0) {
			if (cd->reject_tu_chattering_info.finger_info != NULL) {
				if (cd->reject_tu_chattering_info.finger_info[t].check_status == 1) {
					cd->reject_tu_chattering_info.finger_info[t].check_status = 0;

					if (time_after(jiffies, cd->reject_tu_chattering_info.finger_info[t].tu_time + msecs_to_jiffies(PARADE_PRM_REJECT_TU_CHATTERING_TU_CHECK_TIME_MS)) == false) {
						if ((((int)cd->reject_tu_chattering_info.finger_info[t].tu_x - PARADE_PRM_REJECT_TU_CHATTERING_TU_CHECK_THRESH_XY) <= tch->abs[PT_TCH_X]) &&
						    (((int)cd->reject_tu_chattering_info.finger_info[t].tu_x + PARADE_PRM_REJECT_TU_CHATTERING_TU_CHECK_THRESH_XY) >= tch->abs[PT_TCH_X]) &&
						    (((int)cd->reject_tu_chattering_info.finger_info[t].tu_y - PARADE_PRM_REJECT_TU_CHATTERING_TU_CHECK_THRESH_XY) <= tch->abs[PT_TCH_Y]) &&
						    (((int)cd->reject_tu_chattering_info.finger_info[t].tu_y + PARADE_PRM_REJECT_TU_CHATTERING_TU_CHECK_THRESH_XY) >= tch->abs[PT_TCH_Y])) {
							cd->reject_tu_chattering_info.finger_info[t].check_status = 2;
							// save td info;
							cd->reject_tu_chattering_info.finger_info[t].td_finger.x = tch->abs[PT_TCH_X];
							cd->reject_tu_chattering_info.finger_info[t].td_finger.y = tch->abs[PT_TCH_Y];
							cd->reject_tu_chattering_info.finger_info[t].td_finger.wx = tch->abs[PT_TCH_MAJ];
							cd->reject_tu_chattering_info.finger_info[t].td_finger.wy = tch->abs[PT_TCH_MIN];
							cd->reject_tu_chattering_info.finger_info[t].td_finger.z = tch->abs[PT_TCH_P];
							cd->reject_tu_chattering_info.finger_info[t].td_finger.obj = tch->abs[PT_TCH_O];
							cd->reject_tu_chattering_info.finger_info[t].td_finger.ori = tch->abs[PT_TCH_OR];

							cd->reject_tu_chattering_info.finger_info[t].td_time = jiffies;

							pt_reject_tu_chattering_update_tu_chattering_timer(md);

							pt_debug(dev, DL_INFO, "%s: [reject_tu_chattering] t=%d x=%d y=%d inhibit td. -> tu\n",
								__func__, t, tch->abs[PT_TCH_X], tch->abs[PT_TCH_Y]);
							tch->abs[PT_TCH_E] = PT_EV_LIFTOFF;
							goto pt_get_mt_touches_pr_tch;
						}
						else {
							pt_debug(dev, DL_INFO, "%s: [reject_tu_chattering] t=%d x=%d y=%d td is out of area(tu_x=%d tu_y=%d range=%d). check end.\n",
								__func__, t, tch->abs[PT_TCH_X], tch->abs[PT_TCH_Y],
								cd->reject_tu_chattering_info.finger_info[t].tu_x,
								cd->reject_tu_chattering_info.finger_info[t].tu_y,
								PARADE_PRM_REJECT_TU_CHATTERING_TU_CHECK_THRESH_XY);
						}
					}
				}
				else if (cd->reject_tu_chattering_info.finger_info[t].check_status == 2) {
					if ((((int)cd->reject_tu_chattering_info.finger_info[t].tu_x - PARADE_PRM_REJECT_TU_CHATTERING_TU_CHECK_THRESH_XY) <= tch->abs[PT_TCH_X]) &&
					    (((int)cd->reject_tu_chattering_info.finger_info[t].tu_x + PARADE_PRM_REJECT_TU_CHATTERING_TU_CHECK_THRESH_XY) >= tch->abs[PT_TCH_X]) &&
					    (((int)cd->reject_tu_chattering_info.finger_info[t].tu_y - PARADE_PRM_REJECT_TU_CHATTERING_TU_CHECK_THRESH_XY) <= tch->abs[PT_TCH_Y]) &&
					    (((int)cd->reject_tu_chattering_info.finger_info[t].tu_y + PARADE_PRM_REJECT_TU_CHATTERING_TU_CHECK_THRESH_XY) >= tch->abs[PT_TCH_Y])) {
						pt_debug(dev, DL_INFO, "%s: [reject_tu_chattering] t=%d x=%d y=%d inhibit drag. -> tu\n",
							__func__, t, tch->abs[PT_TCH_X], tch->abs[PT_TCH_Y]);
						tch->abs[PT_TCH_E] = PT_EV_LIFTOFF;
						goto pt_get_mt_touches_pr_tch;
					}
					else {
						pt_debug(dev, DL_INFO, "%s: [reject_tu_chattering] t=%d x=%d y=%d inhibit cancel by out of area(tu_x=%d tu_y=%d range=%d).\n",
							__func__, t, tch->abs[PT_TCH_X], tch->abs[PT_TCH_Y],
							cd->reject_tu_chattering_info.finger_info[t].tu_x,
							cd->reject_tu_chattering_info.finger_info[t].tu_y,
							PARADE_PRM_REJECT_TU_CHATTERING_TU_CHECK_THRESH_XY);
						cd->reject_tu_chattering_info.finger_info[t].check_status = 0;
						pt_reject_tu_chattering_notifier_td_finger(md, t);
						pt_reject_tu_chattering_update_tu_chattering_timer(md);
					}
				}

				if (cd->reject_tu_chattering_info.finger_info[t].check_status == 0) {
					cd->reject_tu_chattering_info.finger_info[t].tu_x = tch->abs[PT_TCH_X];
					cd->reject_tu_chattering_info.finger_info[t].tu_y = tch->abs[PT_TCH_Y];
				}
			}
		}
#endif /* PARADE_REJECT_TU_CHATTERING_ENABLE */

		/* use 0 based track id's */
		t -= md->t_min;

		sig = MT_PARAM_SIGNAL(md, PT_ABS_ID_OST);
		if (sig != PT_IGNORE_VALUE) {
			if (md->mt_function.input_report)
				md->mt_function.input_report(md->input, sig,
						t, tch->abs[PT_TCH_O]);
			__set_bit(t, ids);
		}

		pt_report_event(md, PT_ABS_D_OST, 0);

		/* all devices: position and pressure fields */
		for (j = 0; j <= PT_ABS_W_OST; j++) {
			if (!si->tch_abs[j].report)
				continue;
#if defined(PARADE_GLOVE_ENABLE)
			if ((PT_TCH_X + j) == PT_TCH_P && tch->abs[PT_TCH_O] == PT_OBJ_GLOVE) {
				pt_report_event(md, PT_ABS_X_OST + j,
						tch->abs[PT_TCH_X + j] += 0x0100);
			}
			else {
				pt_report_event(md, PT_ABS_X_OST + j,
						tch->abs[PT_TCH_X + j]);
			}
#else
			pt_report_event(md, PT_ABS_X_OST + j,
					tch->abs[PT_TCH_X + j]);
#endif /* PARADE_GLOVE_ENABLE */
		}

		/* Get the extended touch fields */
		for (j = 0; j < PT_NUM_EXT_TCH_FIELDS; j++) {
			if (!si->tch_abs[PT_ABS_MAJ_OST + j].report)
				continue;
			pt_report_event(md, PT_ABS_MAJ_OST + j,
					tch->abs[PT_TCH_MAJ + j]);
		}
		if (md->mt_function.input_sync)
			md->mt_function.input_sync(md->input);
		mt_sync_count++;

pt_get_mt_touches_pr_tch:
		pt_debug(dev, DL_INFO,
			"%s: t=%d x=%d y=%d z=%d M=%d m=%d o=%d e=%d obj=%d tip=%d\n",
			__func__, t,
			tch->abs[PT_TCH_X],
			tch->abs[PT_TCH_Y],
			tch->abs[PT_TCH_P],
			tch->abs[PT_TCH_MAJ],
			tch->abs[PT_TCH_MIN],
			tch->abs[PT_TCH_OR],
			tch->abs[PT_TCH_E],
			tch->abs[PT_TCH_O],
			tch->abs[PT_TCH_TIP]);

#if defined(PT_MT_NOTIFIER_DIAG_TOUCH_ENABLE)
		if (pt_mt_diag.fingers) {
			int max_tch = si->sensing_conf_data.max_tch;
			if (t < max_tch) {
				if(tch->abs[PT_TCH_E] == PT_EV_LIFTOFF) {
					pt_mt_diag.fingers[t].state = 0;
					pt_mt_diag.fingers[t].x = 0;
					pt_mt_diag.fingers[t].y = 0;
					pt_mt_diag.fingers[t].wx = 0;
					pt_mt_diag.fingers[t].wy = 0;
					pt_mt_diag.fingers[t].z = 0;
				}
				else {
					pt_mt_diag.fingers[t].state = tch->abs[PT_TCH_E] == PT_EV_NO_EVENT ? PT_EV_MOVE : tch->abs[PT_TCH_E];
					pt_mt_diag.fingers[t].x = tch->abs[PT_TCH_X];
					pt_mt_diag.fingers[t].y = tch->abs[PT_TCH_Y];
					pt_mt_diag.fingers[t].wx = tch->abs[PT_TCH_MAJ];
					pt_mt_diag.fingers[t].wy = tch->abs[PT_TCH_MIN];
					pt_mt_diag.fingers[t].z = tch->abs[PT_TCH_P];
				}
			}
		}
#endif
	}

#if defined(PARADE_DEGRADE_CLING_MEASURES_ENABLE)
	if ((PARADE_PRM_DEGRADE_CLING_MEASURES_FOS_ENABLE_PARAM_ENABLE == 0) || (cd->fos_enable_state == 0)) {
		if (PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_FINGER_LO_ENABLE != 0) {
			if (detect_finger_lo_count > 0) {
				if (detect_finger_count > 0) {
					pt_debug(dev, DL_INFO, "%s: [DEGRADE_CLING_MEASURES] detect fingerLO and touch. request initialize baselines.\n",
						__func__);
					pt_degrade_cling_measures_request_initialize_baselines(md);
				}
				else {
					if (md->degrade_cling_measures_info.detect_finger_lo_flg == false) {
						md->degrade_cling_measures_info.detect_finger_lo_flg = true;
						md->degrade_cling_measures_info.detect_finger_lo_start_time_jiffies = jiffies;
					}
					else {
						if (time_after(jiffies, md->degrade_cling_measures_info.detect_finger_lo_start_time_jiffies + msecs_to_jiffies(PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_LO_TIME_MS)) == true) {
							pt_debug(dev, DL_INFO, "%s: [DEGRADE_CLING_MEASURES] detect fingerLO %dms. request initialize baselines.\n",
								__func__, PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_LO_TIME_MS);
							pt_degrade_cling_measures_request_initialize_baselines(md);
						}
					}
				}
			}
			else {
				md->degrade_cling_measures_info.detect_finger_lo_flg = false;
			}
		}
	}
#endif /* PARADE_DEGRADE_CLING_MEASURES_ENABLE */

	if (md->mt_function.final_sync)
		md->mt_function.final_sync(md->input,
				si->tch_abs[PT_TCH_T].max, mt_sync_count, ids);

	if(md->num_prv_rec == 0 && num_cur_tch > 0){
		touchevent_notifier_call_chain(1, NULL);
	}

	md->num_prv_rec = num_cur_tch;
	md->pre_fw_finger_num = fw_finger_num;

#if defined(PT_MT_NOTIFIER_DIAG_TOUCH_ENABLE)
	pt_mt_diag.event = 1;
	wake_up_interruptible(&pt_mt_diag.wait);
#endif

#if 1
	kfree(ids);
#endif
}

/*******************************************************************************
 * FUNCTION: pt_xy_worker
 *
 * SUMMARY: Read xy_data for all current touches
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *     *md - pointer to touch data structure
 ******************************************************************************/
static int pt_xy_worker(struct pt_mt_data *md)
{
	struct device *dev = md->dev;
	struct pt_sysinfo *si = md->si;
	int max_tch = si->sensing_conf_data.max_tch;
	struct pt_touch tch;
	u8 num_cur_tch;
	int rc = 0;
	u8 fw_num_cur_tch;
#if defined(PARADE_DEGRADE_CLING_MEASURES_ENABLE)
	struct pt_core_data *cd = dev_get_drvdata(dev);
#endif /* PARADE_DEGRADE_CLING_MEASURES_ENABLE */

	pt_get_touch_hdr(md, &tch, si->xy_mode + 3);

	num_cur_tch = tch.hdr[PT_TCH_NUM];
	if (num_cur_tch > max_tch) {
		pt_debug(dev, DL_ERROR, "%s: Num touch err detected (n=%d)\n",
			__func__, num_cur_tch);
		num_cur_tch = max_tch;
	}

	if (tch.hdr[PT_TCH_LO]) {
		pt_debug(dev, DL_INFO, "%s: Large area detected\n",
			__func__);
		if (md->pdata->flags & PT_MT_FLAG_NO_TOUCH_ON_LO)
			num_cur_tch = 0;
	}

	fw_num_cur_tch = num_cur_tch;

#if 1
	if (md->is_suspended == true) {
		pt_debug(dev, DL_INFO, "%s: Discard the touch because suspended.\n",
			__func__);
		num_cur_tch = 0;
	}
#endif

#if defined(PARADE_SUPPORT_DEGRADE_FLG_ENABLE)
	if (PARADE_PRM_SUPPORT_DEGRADE_FLG_ENABLE != 0) {
		if (md->degrade_flg_info.request_touchup_initialize_baselines_flg == true) {
			if ((md->fw_num_prv_rec > 0) && (fw_num_cur_tch == 0)) {
				pt_debug(dev, DL_INFO, "%s: request_initialize_baselines by degrade_flg all touchup.\n",
					__func__);

				md->degrade_flg_info.request_degrade_flg_initialize_baselines_flg = true;
				schedule_delayed_work(&md->degrade_flg_info.degrade_flg_initialize_baselines_delayed_work, 0);

				md->degrade_flg_info.request_touchup_initialize_baselines_flg = false;

				num_cur_tch = 0;
			}
		}
	}
	if (PARADE_PRM_SUPPORT_BL_INIT_FLG_ENABLE != 0) {
		if (tch.hdr[PT_TCH_BL_INIT]) {
			if (md->degrade_flg_info.request_degrade_flg_initialize_baselines_flg == false) {
				pt_debug(dev, DL_INFO, "%s: request_initialize_baselines by BL_Init.\n",
					__func__);

				md->degrade_flg_info.request_degrade_flg_initialize_baselines_flg = true;
				schedule_delayed_work(&md->degrade_flg_info.degrade_flg_initialize_baselines_delayed_work, 0);
			}
			else {
				pt_debug(dev, DL_INFO, "%s: Discard the touch because BL_Init.\n",
					__func__);
			}

			num_cur_tch = 0;
		}
	}
	if (PARADE_PRM_SUPPORT_RNF_FLG_ENABLE != 0) {
		if (tch.hdr[PT_TCH_RNF] && pt_get_touchdown_count(md, num_cur_tch) == 1) {
			if (md->degrade_flg_info.request_degrade_flg_initialize_baselines_flg == false) {
				pt_debug(dev, DL_INFO, "%s: request_initialize_baselines by RNF and Touch_num=1.\n",
					__func__);

				md->degrade_flg_info.request_degrade_flg_initialize_baselines_flg = true;
				schedule_delayed_work(&md->degrade_flg_info.degrade_flg_initialize_baselines_delayed_work, 0);
			}
			num_cur_tch = 0;
		}
	}
#endif /* PARADE_SUPPORT_DEGRADE_FLG_ENABLE */

#if defined(PARADE_DEGRADE_CLING_MEASURES_ENABLE)
	if ((PARADE_PRM_DEGRADE_CLING_MEASURES_FOS_ENABLE_PARAM_ENABLE == 0) || (cd->fos_enable_state == 0)) {
		if (PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_LO_ENABLE != 0) {
			if (tch.hdr[PT_TCH_LO]) {
				if (md->degrade_cling_measures_info.detect_lo_flg == false) {
					md->degrade_cling_measures_info.detect_lo_flg = true;
					md->degrade_cling_measures_info.detect_lo_start_time_jiffies = jiffies;
				}
				else {
					if (time_after(jiffies, md->degrade_cling_measures_info.detect_lo_start_time_jiffies + msecs_to_jiffies(PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_LO_TIME_MS)) == true) {
						pt_debug(dev, DL_INFO, "%s: [DEGRADE_CLING_MEASURES] detect LO %dms. request initialize baselines.\n",
							__func__, PARADE_PRM_DEGRADE_CLING_MEASURES_CHECK_LO_TIME_MS);
						pt_degrade_cling_measures_request_initialize_baselines(md);
					}
				}
			}
			else {
				md->degrade_cling_measures_info.detect_lo_flg = false;
				md->degrade_cling_measures_info.check_lo_skip = 0;
			}
		}
	}
#endif /* PARADE_DEGRADE_CLING_MEASURES_ENABLE */

	if (num_cur_tch == 0 && md->num_prv_rec == 0)
		goto pt_xy_worker_exit;

	/* extract xy_data for all currently reported touches */
	pt_debug(dev, DL_DEBUG, "%s: extract data num_cur_tch=%d\n",
		__func__, num_cur_tch);
	if (num_cur_tch)
		pt_get_mt_touches(md, &tch, num_cur_tch);
	else
		pt_mt_lift_all(md);

#if defined(PARADE_DEGRADE_CLING_MEASURES_ENABLE)
	if ((PARADE_PRM_DEGRADE_CLING_MEASURES_FOS_ENABLE_PARAM_ENABLE == 0) || (cd->fos_enable_state == 0)) {
		if (tch.hdr[PT_TCH_LO]) {
			md->degrade_cling_measures_info.pre_fw_lo = true;
		}
		else {
			md->degrade_cling_measures_info.pre_fw_lo = false;
		}
	}
#endif /* PARADE_DEGRADE_CLING_MEASURES_ENABLE */

	rc = 0;

pt_xy_worker_exit:
	md->fw_num_prv_rec = fw_num_cur_tch;
	return rc;
}

#if defined(PARADE_PROXIMITY_SUPPORT_ENABLE)
static int pt_mt_proximity_check(struct pt_mt_data *md)
{
	struct device *dev = md->dev;
	int data = -1;

	pt_debug(dev, DL_DEBUG, "%s: [proximity] check start\n",
		__func__);
	PROX_dataread_func(&data);
	pt_debug(dev, DL_DEBUG, "%s: [proximity] check end(data:%d)\n",
		__func__, data);

	if(data == PARADE_PROXIMITY_NEAR){
		return PARADE_PROXIMITY_NEAR;
	}

	return PARADE_PROXIMITY_FAR;
}
#endif /* PARADE_PROXIMITY_SUPPORT_ENABLE */

/*******************************************************************************
 * FUNCTION: pt_mt_send_dummy_event
 *
 * SUMMARY: Send dummy/key event to wakeup upper layer of system
 *
 * PARAMETERS:
 *     *cd  - pointer to core data structure
 *     *md  - pointer to touch data structure
 ******************************************************************************/
static void pt_mt_send_dummy_event(struct pt_core_data *cd,
		struct pt_mt_data *md)
{
#if defined(PARADE_LPWG_ENABLE)
	struct device *dev = md->dev;

	if (cd->gesture_id == GESTURE_DOUBLE_TAP && (cd->sleep_state == SS_SLEEP_ON)) {
		md->lpwg.double_tap_x = cd->gesture_data[1] << 8 | cd->gesture_data[0];
		md->lpwg.double_tap_y = cd->gesture_data[3] << 8 | cd->gesture_data[2];

#if defined(PARADE_PROXIMITY_SUPPORT_ENABLE)
		if (pt_mt_proximity_check(md) != PARADE_PROXIMITY_NEAR) {
			sysfs_notify(&dev->kobj, NULL, pt_mt_sysfs_wake_event.attr.name);

			mutex_lock(&cd->lpwg_wake_lock_on_mutex);

			if (cd->lpwg_wake_lock_on == false) {
				cd->lpwg_wake_lock_on = true;
				pt_debug(dev, DL_DEBUG, "%s: wake_lock(lpwg_wake_lock)\n", __func__);
				wake_lock(&cd->lpwg_wake_lock);
			}

			mutex_unlock(&cd->lpwg_wake_lock_on_mutex);

			cancel_delayed_work(&cd->lpwg_wake_lock_timer);
			schedule_delayed_work(&cd->lpwg_wake_lock_timer,
									msecs_to_jiffies(PARADE_LPWG_WAKE_LOCK_TIMEOUT_MS));
		}
		else{
			pt_debug(dev, DL_DEBUG, "%s: [LPWG] proximity near\n",
				__func__);
		}
#else /* PARADE_PROXIMITY_SUPPORT_ENABLE */
		sysfs_notify(&dev->kobj, NULL, pt_mt_sysfs_wake_event.attr.name);
#endif /* PARADE_PROXIMITY_SUPPORT_ENABLE */
	}
	else if (cd->gesture_id == GESTURE_SINGLE_FINGER_LONG_TOUCH && (cd->sleep_state == SS_SLEEP_ON)) {
		unsigned long ids = 0;
		int long_touch_x = cd->gesture_data[1] << 8 | cd->gesture_data[0];
		int long_touch_y = cd->gesture_data[3] << 8 | cd->gesture_data[2];
		int long_touch_ma = cd->gesture_data[4];
		int long_touch_mi = cd->gesture_data[5];
		int long_touch_ori = cd->gesture_data[6];

		mutex_lock(&cd->lpwg_wake_lock_on_mutex);

		if (cd->lpwg_wake_lock_on == false) {
			cd->lpwg_wake_lock_on = true;
			pt_debug(dev, DL_DEBUG, "%s: wake_lock(lpwg_wake_lock)\n", __func__);
			wake_lock(&cd->lpwg_wake_lock);
		}

		mutex_unlock(&cd->lpwg_wake_lock_on_mutex);

		cancel_delayed_work(&cd->lpwg_wake_lock_timer);
		schedule_delayed_work(&cd->lpwg_wake_lock_timer,
								msecs_to_jiffies(PARADE_LPWG_WAKE_LOCK_TIMEOUT_MS));

		#if defined( PARADE_FINGERPRINT_NOTIFY_SUPPRESS_TOUCHDOWN_ENABLE )
			if (PARADE_PRM_FINGERPRINT_NOTIFY_SUPPRESS_TOUCHDOWN_ENABLE) {
				cd->fingerprint_notify_suppress_touchdown_enable = true;
				cd->fingerprint_notify_suppress_touchdown_notifytime = jiffies;
			}
		#endif /* PARADE_FINGERPRINT_NOTIFY_SUPPRESS_TOUCHDOWN_ENABLE */

		long_touch_ori = pt_mt_process_orientation(long_touch_ori);

		if (md->mt_function.input_report)
			md->mt_function.input_report(md->input, ABS_MT_TRACKING_ID,
				0, PT_OBJ_STANDARD_FINGER);
		pt_report_event(md, PT_ABS_X_OST, long_touch_x);
		pt_report_event(md, PT_ABS_Y_OST, long_touch_y);
		pt_report_event(md, PT_ABS_P_OST, 10);
		pt_report_event(md, PT_ABS_MAJ_OST, long_touch_ma);
		pt_report_event(md, PT_ABS_MIN_OST, long_touch_mi);
		pt_report_event(md, PT_ABS_OR_OST, long_touch_ori);
		if (md->mt_function.final_sync)
			md->mt_function.final_sync(md->input, 0, 1, &ids);
		if (md->mt_function.report_slot_liftoff)
			md->mt_function.report_slot_liftoff(md, 1);
		if (md->mt_function.final_sync)
			md->mt_function.final_sync(md->input, 1, 1, &ids);
	}
	else if (cd->gesture_id == GESTURE_2_FINGERS_LONG_TOUCH && (cd->sleep_state == SS_SLEEP_ON)) {
		unsigned long ids = 0;
		int long_touch_x1 = cd->gesture_data[1] << 8 | cd->gesture_data[0];
		int long_touch_y1 = cd->gesture_data[3] << 8 | cd->gesture_data[2];
		int long_touch_ma1 = cd->gesture_data[4];
		int long_touch_mi1 = cd->gesture_data[5];
		int long_touch_ori1 = cd->gesture_data[6];
		int long_touch_x2 = cd->gesture_data[10] << 8 | cd->gesture_data[9];
		int long_touch_y2 = cd->gesture_data[12] << 8 | cd->gesture_data[11];
		int long_touch_ma2 = cd->gesture_data[13];
		int long_touch_mi2 = cd->gesture_data[14];
		int long_touch_ori2 = cd->gesture_data[15];

		mutex_lock(&cd->lpwg_wake_lock_on_mutex);

		if (cd->lpwg_wake_lock_on == false) {
			cd->lpwg_wake_lock_on = true;
			pt_debug(dev, DL_DEBUG, "%s: wake_lock(lpwg_wake_lock)\n", __func__);
			wake_lock(&cd->lpwg_wake_lock);
		}

		mutex_unlock(&cd->lpwg_wake_lock_on_mutex);

		cancel_delayed_work(&cd->lpwg_wake_lock_timer);
		schedule_delayed_work(&cd->lpwg_wake_lock_timer,
								msecs_to_jiffies(PARADE_LPWG_WAKE_LOCK_TIMEOUT_MS));

		long_touch_ori1 = pt_mt_process_orientation(long_touch_ori1);
		long_touch_ori2 = pt_mt_process_orientation(long_touch_ori2);

		if (md->mt_function.input_report)
			md->mt_function.input_report(md->input, ABS_MT_TRACKING_ID,
				0, PT_OBJ_STANDARD_FINGER);
		pt_report_event(md, PT_ABS_X_OST, long_touch_x1);
		pt_report_event(md, PT_ABS_Y_OST, long_touch_y1);
		pt_report_event(md, PT_ABS_P_OST, 10);
		pt_report_event(md, PT_ABS_MAJ_OST, long_touch_ma1);
		pt_report_event(md, PT_ABS_MIN_OST, long_touch_mi1);
		pt_report_event(md, PT_ABS_OR_OST, long_touch_ori1);
		if (md->mt_function.input_report)
			md->mt_function.input_report(md->input, ABS_MT_TRACKING_ID,
				1, PT_OBJ_STANDARD_FINGER);
		pt_report_event(md, PT_ABS_X_OST, long_touch_x2);
		pt_report_event(md, PT_ABS_Y_OST, long_touch_y2);
		pt_report_event(md, PT_ABS_P_OST, 10);
		pt_report_event(md, PT_ABS_MAJ_OST, long_touch_ma2);
		pt_report_event(md, PT_ABS_MIN_OST, long_touch_mi2);
		pt_report_event(md, PT_ABS_OR_OST, long_touch_ori2);
		if (md->mt_function.final_sync)
			md->mt_function.final_sync(md->input, 0, 2, &ids);
		if (md->mt_function.report_slot_liftoff)
			md->mt_function.report_slot_liftoff(md, 2);
		if (md->mt_function.final_sync)
			md->mt_function.final_sync(md->input, 2, 2, &ids);
	}
#else
#ifndef EASYWAKE_TSG6
	/* TSG5 EasyWake */
	unsigned long ids = 0;

	/* for easy wakeup */
	if (md->mt_function.input_report)
		md->mt_function.input_report(md->input, ABS_MT_TRACKING_ID,
			0, PT_OBJ_STANDARD_FINGER);
	if (md->mt_function.input_sync)
		md->mt_function.input_sync(md->input);
	if (md->mt_function.final_sync)
		md->mt_function.final_sync(md->input, 0, 1, &ids);
	if (md->mt_function.report_slot_liftoff)
		md->mt_function.report_slot_liftoff(md, 1);
	if (md->mt_function.final_sync)
		md->mt_function.final_sync(md->input, 1, 1, &ids);
#else
	/* TSG6 FW1.3 and above only. TSG6 FW1.0 - 1.2 does not */
	/*  support EasyWake, and this function will not be called */
	u8 key_value = 0;

	switch (cd->gesture_id) {
	case GESTURE_DOUBLE_TAP:
		key_value = KEY_F1;
		break;
	case GESTURE_TWO_FINGERS_SLIDE:
		key_value = KEY_F2;
		break;
	case GESTURE_TOUCH_DETECTED:
		key_value = KEY_F3;
		break;
	case GESTURE_PUSH_BUTTON:
		key_value = KEY_F4;
		break;
	case GESTURE_SINGLE_SLIDE_DE_TX:
		key_value = KEY_F5;
		break;
	case GESTURE_SINGLE_SLIDE_IN_TX:
		key_value = KEY_F6;
		break;
	case GESTURE_SINGLE_SLIDE_DE_RX:
		key_value = KEY_F7;
		break;
	case GESTURE_SINGLE_SLIDE_IN_RX:
		key_value = KEY_F8;
		break;
	default:
		break;
	}

	if (key_value > 0) {
		input_report_key(md->input, key_value, 1);
		mdelay(10);
		input_report_key(md->input, key_value, 0);
		input_sync(md->input);
	}

	/*
	 * Caution - this debug print is needed by the TTDL automated
	 *           regression test suite
	 */
	pt_debug(md->dev, DL_INFO, "%s: report key: %d\n",
		__func__, key_value);
#endif
#endif /* PARADE_LPWG_ENABLE */
}

/*******************************************************************************
 * FUNCTION: pt_mt_attention
 *
 * SUMMARY: Wrapper function for pt_xy_worker() that subscribe into the TTDL
 *  attention list.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *     *dev - pointer to device structure
 ******************************************************************************/
static int pt_mt_attention(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_mt_data *md = &cd->md;
	int rc;

	if (md->si->xy_mode[2] !=  md->si->desc.tch_report_id)
		return 0;

	/* core handles handshake */
	mutex_lock(&md->mt_lock);
	rc = pt_xy_worker(md);
	mutex_unlock(&md->mt_lock);
	if (rc < 0)
		pt_debug(dev, DL_ERROR,
			"%s: xy_worker error r=%d\n", __func__, rc);

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_mt_wake_attention
 *
 * SUMMARY: Wrapper function for pt_mt_send_dummy_event() that register to
 *  attention list.
 *
 * RETURN:
 *	 0 = success
 *
 * PARAMETERS:
 *     *dev - pointer to device structure
 ******************************************************************************/
static int pt_mt_wake_attention(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_mt_data *md = &cd->md;

	mutex_lock(&md->mt_lock);
	pt_mt_send_dummy_event(cd, md);
	mutex_unlock(&md->mt_lock);
	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_startup_attention
 *
 * SUMMARY: Wrapper function for pt_mt_lift_all() that subcribe into the TTDL
 *  attention list.
 *
 * RETURN:
 *	 0 = success
 *
 * PARAMETERS:
 *     *dev - pointer to device structure
 ******************************************************************************/
static int pt_startup_attention(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_mt_data *md = &cd->md;

	mutex_lock(&md->mt_lock);
	pt_mt_lift_all(md);
	mutex_unlock(&md->mt_lock);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_mt_suspend_attention
 *
 * SUMMARY: Function for touch to enter suspend state that as following steps:
 *          1) Lift all touch
 *          2) Set flag with suspend state
 *          3) Decrese pm system count
 *
 * RETURN:
 *	 0 = success
 *
 * PARAMETERS:
 *     *dev - pointer to device structure
 ******************************************************************************/
static int pt_mt_suspend_attention(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_mt_data *md = &cd->md;

	mutex_lock(&md->mt_lock);
#if defined(PARADE_CLING_REJECT_ENABLE)
	mutex_lock(&md->mt_initialize_baselines_lock);
#endif /* PARADE_CLING_REJECT_ENABLE */
	pt_mt_lift_all(md);
#if defined( PARADE_FINGERPRINT_NOTIFY_SUPPRESS_TOUCHDOWN_ENABLE )
	cd->fingerprint_notify_suppress_touchdown_enable = false;
#endif /* PARADE_FINGERPRINT_NOTIFY_SUPPRESS_TOUCHDOWN_ENABLE */
#if defined(PARADE_CLING_REJECT_ENABLE)
	if (cd->cling_reject_info.finger_info != NULL) {
		int t;
		for (t = 0; t < cd->cling_reject_info.finger_info_max; t++) {
			cd->cling_reject_info.finger_info[t].touch_check_enable = 0;
			cd->cling_reject_info.finger_info[t].repeat_touch_check_enable = 0;
			cd->cling_reject_info.finger_info[t].repeat_touch_check_time_index = 0;
		}

		cd->cling_reject_info.is_running = false;

		cancel_delayed_work(&cd->cling_reject_info.initialize_baselines_delayed_work);
		cd->cling_reject_info.request_initialize_baselines_flg = false;
	}
#endif /* PARADE_CLING_REJECT_ENABLE */
#if defined(PARADE_REJECT_TU_CHATTERING_ENABLE)
	if (cd->reject_tu_chattering_info.finger_info != NULL) {
		int t;
		for (t = 0; t < cd->reject_tu_chattering_info.finger_info_max; t++) {
			cd->reject_tu_chattering_info.finger_info[t].check_status = 0;
		}

		cancel_delayed_work(&cd->reject_tu_chattering_info.timer_delayed_work);
	}
#endif /* PARADE_REJECT_TU_CHATTERING_ENABLE */
#if defined(PARADE_SUPPORT_DEGRADE_FLG_ENABLE)
	md->degrade_flg_info.request_touchup_initialize_baselines_flg = false;
	md->degrade_flg_info.request_degrade_flg_initialize_baselines_flg = false;
#endif /* PARADE_SUPPORT_DEGRADE_FLG_ENABLE */
#if defined(PARADE_DEGRADE_CLING_MEASURES_ENABLE)
	if (md->degrade_cling_measures_info.notch_finger_info != NULL) {
		int t;
		for (t = 0; t < md->degrade_cling_measures_info.finger_info_max; t++) {
			md->degrade_cling_measures_info.notch_finger_info[t].check_status = 0;
			md->degrade_cling_measures_info.lo_info[t].check_skip = 0;
			md->degrade_cling_measures_info.large_td_info[t].check_status = 0;
		}
		md->degrade_cling_measures_info.check_lo_skip = 0;
		md->degrade_cling_measures_info.detect_lo_flg = false;
		md->degrade_cling_measures_info.detect_finger_lo_flg = false;

		cancel_delayed_work(&md->degrade_cling_measures_info.degrade_cling_measures_initialize_baselines_delayed_work);
		md->degrade_cling_measures_info.request_degrade_cling_measures_initialize_baselines_flg = false;
	}
#endif /* PARADE_DEGRADE_CLING_MEASURES_ENABLE */

	md->fw_num_prv_rec = 0;

	md->is_suspended = true;

#if defined(PARADE_CLING_REJECT_ENABLE)
	mutex_unlock(&md->mt_initialize_baselines_lock);
#endif /* PARADE_CLING_REJECT_ENABLE */
	mutex_unlock(&md->mt_lock);

	pm_runtime_put(dev);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_mt_resume_attention
 *
 * SUMMARY: Function for touch to leave suspend state that as following steps:
 *          1) Increse pm system count
 *          2) Clear suspend state flag
 *
 * RETURN:
 *	 0 = success
 *
 * PARAMETERS:
 *     *dev - pointer to device structure
 ******************************************************************************/
static int pt_mt_resume_attention(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_mt_data *md = &cd->md;

	pm_runtime_get(dev);

	mutex_lock(&md->mt_lock);
#if defined(PARADE_CLING_REJECT_ENABLE)
	mutex_lock(&md->mt_initialize_baselines_lock);
	cd->cling_reject_info.is_running = true;
	cd->cling_reject_info.resume_time = jiffies;
#endif /* PARADE_CLING_REJECT_ENABLE */
	md->is_suspended = false;
#if defined(PARADE_CLING_REJECT_ENABLE)
	mutex_unlock(&md->mt_initialize_baselines_lock);
#endif /* PARADE_CLING_REJECT_ENABLE */
	mutex_unlock(&md->mt_lock);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_mt_open
 *
 * SUMMARY: Open method for input device(touch) that sets up call back
 *  functions to TTDL attention list
 *
 * RETURN:
 *	 0 = success
 *
 * PARAMETERS:
 *     *input - pointer to input_dev structure
 ******************************************************************************/
static int pt_mt_open(struct input_dev *input)
{
	struct device *dev = input->dev.parent;
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_mt_data *md = &cd->md;

	pm_runtime_get_sync(dev);

	mutex_lock(&md->mt_lock);
	md->is_suspended = false;
	mutex_unlock(&md->mt_lock);

	pt_debug(dev, DL_INFO, "%s: setup subscriptions\n", __func__);

	/* set up touch call back */
	_pt_subscribe_attention(dev, PT_ATTEN_IRQ, PT_MT_NAME,
		pt_mt_attention, PT_MODE_OPERATIONAL);

	/* set up startup call back */
	_pt_subscribe_attention(dev, PT_ATTEN_STARTUP, PT_MT_NAME,
		pt_startup_attention, 0);

	/* set up wakeup call back */
	_pt_subscribe_attention(dev, PT_ATTEN_WAKE, PT_MT_NAME,
		pt_mt_wake_attention, 0);

	/* set up suspend call back */
	_pt_subscribe_attention(dev, PT_ATTEN_SUSPEND, PT_MT_NAME,
		pt_mt_suspend_attention, 0);

	/* set up resume call back */
	_pt_subscribe_attention(dev, PT_ATTEN_RESUME, PT_MT_NAME,
		pt_mt_resume_attention, 0);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_mt_close
 *
 * SUMMARY: Close method for input device(touch) that clears call back
 *  functions from TTDL attention list.
 *
 * PARAMETERS:
 *     *input - pointer to input_dev structure
 ******************************************************************************/
static void pt_mt_close(struct input_dev *input)
{
	struct device *dev = input->dev.parent;
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_mt_data *md = &cd->md;

	_pt_unsubscribe_attention(dev, PT_ATTEN_IRQ, PT_MT_NAME,
		pt_mt_attention, PT_MODE_OPERATIONAL);

	_pt_unsubscribe_attention(dev, PT_ATTEN_STARTUP, PT_MT_NAME,
		pt_startup_attention, 0);

	_pt_unsubscribe_attention(dev, PT_ATTEN_WAKE, PT_MT_NAME,
		pt_mt_wake_attention, 0);

	_pt_unsubscribe_attention(dev, PT_ATTEN_SUSPEND, PT_MT_NAME,
		pt_mt_suspend_attention, 0);

	_pt_unsubscribe_attention(dev, PT_ATTEN_RESUME, PT_MT_NAME,
		pt_mt_resume_attention, 0);

	mutex_lock(&md->mt_lock);
	if (!md->is_suspended) {
		pm_runtime_put(dev);
		md->is_suspended = true;
	}
	mutex_unlock(&md->mt_lock);
}

/*******************************************************************************
 * FUNCTION: pt_setup_input_device
 *
 * SUMMARY: Set up resolution, event signal capabilities and
 *  register input device for touch.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *     *dev - pointer to device structure
 ******************************************************************************/
static int pt_setup_input_device(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_mt_data *md = &cd->md;
	int signal = PT_IGNORE_VALUE;
	int max_x, max_y, max_p, min, max;
	int max_x_tmp, max_y_tmp;
	int i;
	int rc;

	pt_debug(dev, DL_INFO, "%s: Initialize event signals\n",
		__func__);
	__set_bit(EV_ABS, md->input->evbit);
	__set_bit(EV_REL, md->input->evbit);
	__set_bit(EV_KEY, md->input->evbit);
#ifdef INPUT_PROP_DIRECT
	__set_bit(INPUT_PROP_DIRECT, md->input->propbit);
#endif

	/* If virtualkeys enabled, don't use all screen */
	if (md->pdata->flags & PT_MT_FLAG_VKEYS) {
		max_x_tmp = md->pdata->vkeys_x;
		max_y_tmp = md->pdata->vkeys_y;
	} else {
		max_x_tmp = md->si->sensing_conf_data.res_x;
		max_y_tmp = md->si->sensing_conf_data.res_y;
	}

	/* get maximum values from the sysinfo data */
	if (md->pdata->flags & PT_MT_FLAG_FLIP) {
		max_x = max_y_tmp - 1;
		max_y = max_x_tmp - 1;
	} else {
		max_x = max_x_tmp - 1;
		max_y = max_y_tmp - 1;
	}
#if defined(PARADE_GLOVE_ENABLE)
	max_p = 511;
#else
	max_p = md->si->sensing_conf_data.max_z;
#endif /* PARADE_GLOVE_ENABLE */

	/* set event signal capabilities */
	for (i = 0; i < NUM_SIGNALS(md->pdata->frmwrk); i++) {
		signal = MT_PARAM_SIGNAL(md, i);
		if (signal != PT_IGNORE_VALUE) {
			__set_bit(signal, md->input->absbit);

			min = MT_PARAM_MIN(md, i);
			max = MT_PARAM_MAX(md, i);
			if (i == PT_ABS_ID_OST) {
				/* shift track ids down to start at 0 */
				max = max - min;
				min = min - min;
			}
#if defined(PT_MT_SIMPLE_ORIENTATION_ENABLE)
			else if (i == PT_ABS_OR_OST) {
				min = 0;
				max = 1;
			}
#endif /* PT_MT_SIMPLE_ORIENTATION_ENABLE */
			else if (i == PT_ABS_X_OST)
				max = max_x;
			else if (i == PT_ABS_Y_OST)
				max = max_y;
			else if (i == PT_ABS_P_OST)
				max = max_p;
#if 1
			else if (i == PT_ABS_MAJ_OST)
				max = 50;
			else if (i == PT_ABS_MIN_OST)
				max = 50;
#endif

			input_set_abs_params(md->input, signal, min, max,
				MT_PARAM_FUZZ(md, i), MT_PARAM_FLAT(md, i));
			pt_debug(dev, DL_INFO,
				"%s: register signal=%02X min=%d max=%d\n",
				__func__, signal, min, max);
		}
	}

#if defined(PT_MT_SIMPLE_ORIENTATION_ENABLE)
	md->or_min = 0;
	md->or_max = 1;
#else
	md->or_min = MT_PARAM_MIN(md, PT_ABS_OR_OST);
	md->or_max = MT_PARAM_MAX(md, PT_ABS_OR_OST);
#endif /* PT_MT_SIMPLE_ORIENTATION_ENABLE */

	md->t_min = MT_PARAM_MIN(md, PT_ABS_ID_OST);
	md->t_max = MT_PARAM_MAX(md, PT_ABS_ID_OST);

	rc = md->mt_function.input_register_device(md->input,
			md->si->tch_abs[PT_TCH_T].max);
	if (rc < 0)
		pt_debug(dev, DL_ERROR, "%s: Error, failed register input device r=%d\n",
			__func__, rc);
	else
		md->input_device_registered = true;

#if defined(PARADE_LPWG_ENABLE)
#else
#ifdef EASYWAKE_TSG6
	input_set_capability(md->input, EV_KEY, KEY_F1);
	input_set_capability(md->input, EV_KEY, KEY_F2);
	input_set_capability(md->input, EV_KEY, KEY_F3);
	input_set_capability(md->input, EV_KEY, KEY_F4);
	input_set_capability(md->input, EV_KEY, KEY_F5);
	input_set_capability(md->input, EV_KEY, KEY_F6);
	input_set_capability(md->input, EV_KEY, KEY_F7);
	input_set_capability(md->input, EV_KEY, KEY_F8);
#endif
#endif /* PARADE_LPWG_ENABLE */
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_setup_input_attention
 *
 * SUMMARY: Wrapper function for pt_setup_input_device() register to TTDL
 *  attention list.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *     *dev - pointer to device structure
 ******************************************************************************/
static int pt_setup_input_attention(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_mt_data *md = &cd->md;
	int rc;

	md->si = _pt_request_sysinfo(dev);
	if (!md->si)
		return -EINVAL;

	rc = pt_setup_input_device(dev);

#if defined(PT_MT_NOTIFIER_DIAG_TOUCH_ENABLE)
	pt_mt_diag.fingers = kzalloc(md->si->sensing_conf_data.max_tch * sizeof(struct touch_info), GFP_KERNEL);
	if (!pt_mt_diag.fingers) {
		pt_debug(dev, DL_ERROR, "%s: Failed to allocate memory for pt_mt_diag.fingers\n",
				__func__);
	}
#endif
#if defined(PARADE_CLING_REJECT_ENABLE)
	if (cd->cling_reject_info.finger_info) {
		kfree(cd->cling_reject_info.finger_info);
	}
	cd->cling_reject_info.finger_info_max = md->t_max + 1;
	cd->cling_reject_info.finger_info = kzalloc(cd->cling_reject_info.finger_info_max * sizeof(struct pt_cling_reject_finger_info), GFP_KERNEL);
	if (!cd->cling_reject_info.finger_info) {
		pt_debug(dev, DL_ERROR, "%s: Failed to allocate memory for cd->cling_reject_info.finger_info\n",
				__func__);
	}
#endif /* PARADE_CLING_REJECT_ENABLE */
#if defined(PARADE_REJECT_TU_CHATTERING_ENABLE)
	if (cd->reject_tu_chattering_info.finger_info) {
		kfree(cd->reject_tu_chattering_info.finger_info);
	}
	cd->reject_tu_chattering_info.finger_info_max = md->t_max + 1;
	cd->reject_tu_chattering_info.finger_info = kzalloc(cd->reject_tu_chattering_info.finger_info_max * sizeof(struct pt_reject_tu_chattering_finger_info), GFP_KERNEL);
	if (!cd->reject_tu_chattering_info.finger_info) {
		pt_debug(dev, DL_ERROR, "%s: Failed to allocate memory for cd->reject_tu_chattering_info.finger_info\n",
				__func__);
	}
#endif /* PARADE_REJECT_TU_CHATTERING_ENABLE */
#if defined(PARADE_DEGRADE_CLING_MEASURES_ENABLE)
	if (md->degrade_cling_measures_info.notch_finger_info) {
		kfree(md->degrade_cling_measures_info.notch_finger_info);
	}
	md->degrade_cling_measures_info.finger_info_max = md->t_max + 1;
	md->degrade_cling_measures_info.notch_finger_info = kzalloc(md->degrade_cling_measures_info.finger_info_max * sizeof(struct pt_mt_degrade_cling_measures_notch_finger_info), GFP_KERNEL);
	if (!md->degrade_cling_measures_info.notch_finger_info) {
		pt_debug(dev, DL_ERROR, "%s: Failed to allocate memory for md->degrade_cling_measures_info.notch_finger_info\n",
				__func__);
	}
	md->degrade_cling_measures_info.lo_info = kzalloc(md->degrade_cling_measures_info.finger_info_max * sizeof(struct pt_mt_degrade_cling_measures_lo_info), GFP_KERNEL);
	if (!md->degrade_cling_measures_info.lo_info) {
		pt_debug(dev, DL_ERROR, "%s: Failed to allocate memory for md->degrade_cling_measures_info.lo_info\n",
				__func__);
	}
	md->degrade_cling_measures_info.large_td_info = kzalloc(md->degrade_cling_measures_info.finger_info_max * sizeof(struct pt_mt_degrade_cling_measures_large_td_info), GFP_KERNEL);
	if (!md->degrade_cling_measures_info.large_td_info) {
		pt_debug(dev, DL_ERROR, "%s: Failed to allocate memory for md->degrade_cling_measures_info.large_td_info\n",
				__func__);
	}
#endif /* PARADE_DEGRADE_CLING_MEASURES_ENABLE */

	_pt_unsubscribe_attention(dev, PT_ATTEN_STARTUP, PT_MT_NAME,
		pt_setup_input_attention, 0);

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_mt_probe
 *
 * SUMMARY: The probe function for touch input device
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev   - pointer to device structure
 ******************************************************************************/
int pt_mt_probe(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_mt_data *md = &cd->md;
	struct pt_platform_data *pdata = dev_get_platdata(dev);
	struct pt_mt_platform_data *mt_pdata;
	int rc = 0;

	pt_debug(dev, DL_INFO,
		"%s: >>>>>> Register MT <<<<<<\n", __func__);
	if (!pdata || !pdata->mt_pdata) {
		pt_debug(dev, DL_ERROR,
			"%s: Missing platform data\n", __func__);
		rc = -ENODEV;
		goto error_no_pdata;
	}
	mt_pdata = pdata->mt_pdata;

	pt_init_function_ptrs(md);

	mutex_init(&md->mt_lock);
#if defined(PARADE_CLING_REJECT_ENABLE)
	mutex_init(&md->mt_initialize_baselines_lock);
#endif /* PARADE_CLING_REJECT_ENABLE */
#if defined(PARADE_SUPPORT_DEGRADE_FLG_ENABLE)
	INIT_DELAYED_WORK(&md->degrade_flg_info.degrade_flg_initialize_baselines_delayed_work, pt_degrade_flg_initialize_baselines_delayed_work_function);
	md->degrade_flg_info.initialize_baselines_end_time_jiffies = jiffies;
#endif /* PARADE_SUPPORT_DEGRADE_FLG_ENABLE */
	md->dev = dev;
	md->pdata = mt_pdata;
#if defined(PT_MT_NOTIFIER_DIAG_TOUCH_ENABLE)
	memset(&pt_mt_diag, 0, sizeof(pt_mt_diag));
#endif

	/* Create the input device and register it. */
	pt_debug(dev, DL_INFO,
		"%s: Create the input device and register it\n", __func__);
	md->input = input_allocate_device();
	if (!md->input) {
		pt_debug(dev, DL_ERROR, "%s: Error, failed to allocate input device\n",
			__func__);
		rc = -ENODEV;
		goto error_alloc_failed;
	} else
		md->input_device_allocated = true;

	if (md->pdata->inp_dev_name)
		md->input->name = md->pdata->inp_dev_name;
	else
		md->input->name = PT_MT_NAME;
	scnprintf(md->phys, sizeof(md->phys), "%s/input%d", dev_name(dev),
			cd->phys_num++);
	md->input->phys = md->phys;
	md->input->dev.parent = md->dev;
	md->input->open = pt_mt_open;
	md->input->close = pt_mt_close;
	input_set_drvdata(md->input, md);

	/* get sysinfo */
	md->si = _pt_request_sysinfo(dev);

	if (md->si) {
		rc = pt_setup_input_device(dev);
		if (rc)
			goto error_init_input;
	} else {
		pt_debug(dev, DL_ERROR, "%s: Fail get sysinfo pointer from core p=%p\n",
			__func__, md->si);
		_pt_subscribe_attention(dev, PT_ATTEN_STARTUP,
			PT_MT_NAME, pt_setup_input_attention, 0);
	}

#if defined(PT_MT_NOTIFIER_DIAG_TOUCH_ENABLE)
	init_waitqueue_head(&pt_mt_diag.wait);

	if (md->si) {
		pt_mt_diag.fingers = kzalloc(md->si->sensing_conf_data.max_tch * sizeof(struct touch_info), GFP_KERNEL);
		if (!pt_mt_diag.fingers) {
			pt_debug(dev, DL_ERROR, "%s: Failed to allocate memory for pt_mt_diag.fingers\n",
					__func__);
			goto err_pt_mt_diag_fingers;
		}
	}
#endif

	{
		int idx;
		for (idx = 0; idx < ARRAY_SIZE(pt_mt_attrs); idx++) {
			rc = device_create_file(dev,
					pt_mt_attrs[idx]);
			if (rc < 0) {
				pt_debug(dev, DL_ERROR, "%s: Failed to create sysfs file\n",
					__func__);
				goto err_sysfs_create_file;
			}
		}
	}

#if defined(PARADE_CLING_REJECT_ENABLE)
	INIT_DELAYED_WORK(&cd->cling_reject_info.initialize_baselines_delayed_work, pt_cling_reject_initialize_baselines_delayed_work_function);

	if (md->si) {
		cd->cling_reject_info.finger_info_max = md->t_max + 1;
		cd->cling_reject_info.finger_info = kzalloc(cd->cling_reject_info.finger_info_max * sizeof(struct pt_cling_reject_finger_info), GFP_KERNEL);
		if (!cd->cling_reject_info.finger_info) {
			pt_debug(dev, DL_ERROR, "%s: Failed to allocate memory for cd->cling_reject_info.finger_info\n",
					__func__);
			goto err_cling_reject_info_finger_info;
		}
	}
#endif /* PARADE_CLING_REJECT_ENABLE */

#if defined(PARADE_REJECT_TU_CHATTERING_ENABLE)
	INIT_DELAYED_WORK(&cd->reject_tu_chattering_info.timer_delayed_work, pt_reject_tu_chattering_delayed_work_function);

	if (md->si) {
		cd->reject_tu_chattering_info.finger_info_max = md->t_max + 1;
		cd->reject_tu_chattering_info.finger_info = kzalloc(cd->reject_tu_chattering_info.finger_info_max * sizeof(struct pt_reject_tu_chattering_finger_info), GFP_KERNEL);
		if (!cd->reject_tu_chattering_info.finger_info) {
			pt_debug(dev, DL_ERROR, "%s: Failed to allocate memory for cd->reject_tu_chattering_info.finger_info\n",
					__func__);
			goto err_reject_tu_chattering_info_finger_info;
		}
	}
#endif /* PARADE_REJECT_TU_CHATTERING_ENABLE */

#if defined(PARADE_DEGRADE_CLING_MEASURES_ENABLE)
	INIT_DELAYED_WORK(&md->degrade_cling_measures_info.degrade_cling_measures_initialize_baselines_delayed_work, pt_degrade_cling_measures_delayed_work_function);

	if (md->si) {
		md->degrade_cling_measures_info.finger_info_max = md->t_max + 1;
		md->degrade_cling_measures_info.notch_finger_info = kzalloc(md->degrade_cling_measures_info.finger_info_max * sizeof(struct pt_mt_degrade_cling_measures_notch_finger_info), GFP_KERNEL);
		if (!md->degrade_cling_measures_info.notch_finger_info) {
			pt_debug(dev, DL_ERROR, "%s: Failed to allocate memory for md->degrade_cling_measures_info.notch_finger_info\n",
					__func__);
			goto err_degrade_cling_measures_info_notch_finger_info;
		}
		md->degrade_cling_measures_info.lo_info = kzalloc(md->degrade_cling_measures_info.finger_info_max * sizeof(struct pt_mt_degrade_cling_measures_lo_info), GFP_KERNEL);
		if (!md->degrade_cling_measures_info.lo_info) {
			pt_debug(dev, DL_ERROR, "%s: Failed to allocate memory for md->degrade_cling_measures_info.lo_info\n",
					__func__);
			goto err_degrade_cling_measures_info_lo_info;
		}
		md->degrade_cling_measures_info.large_td_info = kzalloc(md->degrade_cling_measures_info.finger_info_max * sizeof(struct pt_mt_degrade_cling_measures_large_td_info), GFP_KERNEL);
		if (!md->degrade_cling_measures_info.large_td_info) {
			pt_debug(dev, DL_ERROR, "%s: Failed to allocate memory for md->degrade_cling_measures_info.large_td_info\n",
					__func__);
			goto err_degrade_cling_measures_info_large_td_info;
		}
	}
#endif /* PARADE_DEGRADE_CLING_MEASURES_ENABLE */

	return 0;

#if defined(PARADE_DEGRADE_CLING_MEASURES_ENABLE)
	if (md->degrade_cling_measures_info.large_td_info) {
		kfree(md->degrade_cling_measures_info.large_td_info);
	}
err_degrade_cling_measures_info_large_td_info:
	if (md->degrade_cling_measures_info.lo_info) {
		kfree(md->degrade_cling_measures_info.lo_info);
	}
err_degrade_cling_measures_info_lo_info:
	if (md->degrade_cling_measures_info.notch_finger_info) {
		kfree(md->degrade_cling_measures_info.notch_finger_info);
	}
err_degrade_cling_measures_info_notch_finger_info:
#endif /* PARADE_DEGRADE_CLING_MEASURES_ENABLE */
#if defined(PARADE_REJECT_TU_CHATTERING_ENABLE)
	if (cd->reject_tu_chattering_info.finger_info) {
		kfree(cd->reject_tu_chattering_info.finger_info);
	}
err_reject_tu_chattering_info_finger_info:
#endif /* PARADE_REJECT_TU_CHATTERING_ENABLE */
#if defined(PARADE_CLING_REJECT_ENABLE)
	if (cd->cling_reject_info.finger_info) {
		kfree(cd->cling_reject_info.finger_info);
	}
err_cling_reject_info_finger_info:
#endif /* PARADE_CLING_REJECT_ENABLE */
#if defined(PT_MT_NOTIFIER_DIAG_TOUCH_ENABLE)
	{
		int idx;
		for (idx = 0; idx < ARRAY_SIZE(pt_mt_attrs); idx++) {
			device_remove_file(dev, pt_mt_attrs[idx]);
		}
	}
err_sysfs_create_file:
	if (pt_mt_diag.fingers) {
		kfree(pt_mt_diag.fingers);
	}
err_pt_mt_diag_fingers:
#endif
error_init_input:
	input_free_device(md->input);
	md->input_device_allocated = false;
error_alloc_failed:
error_no_pdata:
	pt_debug(dev, DL_ERROR, "%s failed.\n", __func__);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_mt_release
 *
 * SUMMARY: The release function for touch input device
 *
 * RETURN:
 *	 0 = success
 *
 * PARAMETERS:
 *	*dev   - pointer to device structure
 ******************************************************************************/
int pt_mt_release(struct device *dev)
{
	struct pt_core_data *cd;
	struct pt_mt_data *md;

	/* Ensure valid pointers before de-referencing them */
	if (dev) {
		cd = dev_get_drvdata(dev);
		if (cd)
			md = &cd->md;
		else
			return 0;
	} else {
		return 0;
	}

#if defined(PARADE_DEGRADE_CLING_MEASURES_ENABLE)
	if (md->degrade_cling_measures_info.large_td_info) {
		kfree(md->degrade_cling_measures_info.large_td_info);
	}
	if (md->degrade_cling_measures_info.lo_info) {
		kfree(md->degrade_cling_measures_info.lo_info);
	}
	if (md->degrade_cling_measures_info.notch_finger_info) {
		kfree(md->degrade_cling_measures_info.notch_finger_info);
	}
#endif /* PARADE_DEGRADE_CLING_MEASURES_ENABLE */
#if defined(PARADE_REJECT_TU_CHATTERING_ENABLE)
	if (cd->reject_tu_chattering_info.finger_info) {
		kfree(cd->reject_tu_chattering_info.finger_info);
	}
#endif /* PARADE_REJECT_TU_CHATTERING_ENABLE */
#if defined(PARADE_CLING_REJECT_ENABLE)
	if (cd->cling_reject_info.finger_info) {
		kfree(cd->cling_reject_info.finger_info);
	}
#endif /* PARADE_CLING_REJECT_ENABLE */
#if defined(PT_MT_NOTIFIER_DIAG_TOUCH_ENABLE)
	{
		int idx;
		for (idx = 0; idx < ARRAY_SIZE(pt_mt_attrs); idx++) {
			device_remove_file(dev, pt_mt_attrs[idx]);
		}
	}
	if (pt_mt_diag.fingers) {
		kfree(pt_mt_diag.fingers);
	}
#endif

	/*
	 * Second call this function may cause kernel panic if probe fail.
	 * Use input_device_registered & input_device_allocated variable to
	 * avoid unregister or free unavailable devive.
	 */
	if (md && md->input_device_registered) {
		md->input_device_registered = false;
		input_unregister_device(md->input);
		/* Unregistering device will free the device too */
		md->input_device_allocated = false;
	} else if (md && md->input_device_allocated) {
		md->input_device_allocated = false;
		input_free_device(md->input);
		_pt_unsubscribe_attention(dev, PT_ATTEN_STARTUP,
			PT_MT_NAME, pt_setup_input_attention, 0);
	}

	return 0;
}
