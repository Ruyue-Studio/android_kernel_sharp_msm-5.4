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
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/iopoll.h>
#include <video/mipi_display.h>
#include "../sde/sde_hw_intf.h"
#include "../dsi/dsi_ctrl_reg.h"
#include "../dsi/dsi_hw.h"
#include "../msm_drv.h"
#include "drm_cmn.h"
#ifdef CONFIG_SHARP_DRM_HR_VID
#include "drm_mfr.h"
#endif /* CONFIG_SHARP_DRM_HR_VID */
#ifdef CONFIG_SHARP_BOOT
#include <soc/qcom/sharp/sh_boot_manager.h>
#endif /* CONFIG_SHARP_BOOT */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* STRUCTURE                                                                 */
/* ------------------------------------------------------------------------- */
static struct {
	int		boot_mode;
	int		hw_revision;
	unsigned char	hw_handset;
	unsigned char	upperunit;
	unsigned char	disp_on_status;
	unsigned char	lcd_switch;
	unsigned char	otp_bias;
	unsigned char	otp_vgsp;
	unsigned char	otp_gamma_shift;
} drm_cmn_ctx = {
#ifdef CONFIG_SHARP_BOOT
	.boot_mode = SH_BOOT_NORMAL,
#endif /* CONFIG_SHARP_BOOT */
	.hw_revision = 7,
	.hw_handset = 1,
	.upperunit = 1,
	.disp_on_status = 0,
	.lcd_switch = 3,
	.otp_bias = 0,
	.otp_vgsp = 0,
	.otp_gamma_shift = 1,
};
/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static char str_boot_param[MAX_CMDLINE_PARAM_LEN];
module_param_string(boot_param, str_boot_param, MAX_CMDLINE_PARAM_LEN,
								0600);
/* ------------------------------------------------------------------------- */
/* FUNCTION                                                                  */
/* ------------------------------------------------------------------------- */
static int drm_cmn_ctrl_timing_generator(u8 onoff, u32 ctrl_count);
static int drm_cmn_ctrl_video_engine(struct dsi_display *pdisp, u8 onoff);
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
void drm_cmn_init(void)
{
	int val1, val2, val3, val4, val5, val6, val7, val8, val9, val10;

	if (sscanf(str_boot_param, "%d:%d:%d:%d:%d:%d:%d:%d:panel_gamma_shift=%d:panel_revision=%d:", &val1, &val2, &val3, &val4, &val5, &val6, &val7, &val8, &val9, &val10) == 10) {
		drm_cmn_ctx.boot_mode = val1;
		drm_cmn_ctx.hw_handset = val2;
		drm_cmn_ctx.hw_revision = val3;
		drm_cmn_ctx.upperunit = val4;
		drm_debug = val5;
		drm_cmn_ctx.disp_on_status = val6;
		drm_cmn_ctx.otp_bias = val7;
		drm_cmn_ctx.otp_vgsp = val8;
		drm_cmn_ctx.otp_gamma_shift = val9;
		drm_cmn_ctx.lcd_switch = val10;
	}
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_video_transfer_ctrl(struct dsi_display *pdisp, u8 onoff)
{
	int ret = 0;

	pr_debug("%s: in\n", __func__);

	if (!pdisp) {
		pr_err("%s: no display\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: display= %p\n", __func__, pdisp);

	if (onoff) {
		ret = drm_cmn_start_video(pdisp);
		if (ret) {
			goto error;
		}
#ifdef CONFIG_SHARP_DRM_HR_VID
		drm_mfr_suspend_ctrl(false);
#endif /* CONFIG_SHARP_DRM_HR_VID */
	} else {
#ifdef CONFIG_SHARP_DRM_HR_VID
		drm_mfr_suspend_ctrl(true);
#endif /* CONFIG_SHARP_DRM_HR_VID */
		ret = drm_cmn_stop_video(pdisp);
		if (ret) {
			goto error;
		}
	}

	pr_debug("%s: succeed %s video\n", __func__,
					(onoff ? "starting" : "stopping"));
	return 0;

error:
	pr_err("%s: failed to %s video\n", __func__,
					(onoff ? "starting" : "stopping"));
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_stop_video(struct dsi_display *pdisp)
{
	int ret = 0;
	int wait_ms = 20;

	pr_debug("%s: in\n", __func__);

	if (!pdisp) {
		pr_err("%s: no display\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: display= %p\n", __func__, pdisp);

	ret = drm_cmn_ctrl_timing_generator(false, pdisp->ctrl_count);
	if (ret) {
		pr_err("%s: failed drm_cmn_ctrl_timing_generator\n",
								 __func__);
		return ret;
	}

	if (pdisp->panel && pdisp->panel->cur_mode) {
		if (pdisp->panel->cur_mode->timing.refresh_rate < 60)
			wait_ms = 50;
	}

	pr_debug("%s: wait %dmsec\n", __func__, wait_ms);
	usleep_range(wait_ms*1000, wait_ms*1000+10);

	ret = drm_cmn_ctrl_video_engine(pdisp, false);
	if (ret) {
		pr_err("%s: failed drm_cmn_ctrl_video_engine\n", __func__);
		return ret;
	}

	pr_debug("%s: out\n", __func__);
	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_start_video(struct dsi_display *pdisp)
{
	int ret = 0;

	pr_debug("%s: in\n", __func__);

	if (!pdisp) {
		pr_err("%s: no display\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: display= %p\n", __func__, pdisp);

	ret = drm_cmn_ctrl_video_engine(pdisp, true);
	if (ret) {
		pr_err("%s: failed drm_cmn_ctrl_video_engine\n", __func__);
		return ret;
	}

	ret = drm_cmn_ctrl_timing_generator(true, pdisp->ctrl_count);
	if (ret) {
		pr_err("%s: failed drm_cmn_ctrl_timing_generator\n",
								__func__);
		return ret;
	}

	pr_debug("%s: out\n", __func__);
	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int drm_cmn_timing_generator(int index, u8 onoff)
{
	struct sde_hw_intf *intf;

	intf = get_sde_hw_intf(index);
	if (!intf) {
		pr_warn("%s: null sde_hw_intf\n", __func__);
		return 0;
	}
	pr_debug("%s: intf = %p\n", __func__, intf);
	if (!intf->ops.enable_timing) {
		pr_err("%s: enable_timing function is Null\n", __func__);
		return -EINVAL;
	}

	intf->ops.enable_timing(intf, onoff);

	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int drm_cmn_ctrl_timing_generator(u8 onoff, u32 ctrl_count)
{
	int ret = 0;
	pr_debug("%s: %s timing generator\n", __func__,
						(onoff ? "start" : "stop"));

	ret = drm_cmn_timing_generator(1, onoff);
	if (ret) {
		return ret;
	}

	if (ctrl_count > 1) {
		ret = drm_cmn_timing_generator(2, onoff);
		if (ret) {
			return ret;
		}
	}

	pr_debug("%s: out\n", __func__);

	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int drm_cmn_ctrl_check_vid_engine_state(struct dsi_ctrl *dsi_ctrl, u32 op_state)
{
	int rc = 0;
	struct dsi_ctrl_state_info *state = &dsi_ctrl->current_state;

	if (state->vid_engine_state == op_state) {
		pr_warn("[%d] No change in state, cmd_state=%d\n",
			   dsi_ctrl->cell_index, op_state);
		rc = -1;
	}

	return rc;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int drm_cmn_video_engine(struct dsi_ctrl *pctrl, u8 onoff)
{
	int ret = 0;
	int ctrl_engine = onoff ? DSI_CTRL_ENGINE_ON : DSI_CTRL_ENGINE_OFF;

	if (pctrl) {
		pr_debug("%s: dsi_ctrl = %p\n", __func__, pctrl);

		ret = drm_cmn_ctrl_check_vid_engine_state(pctrl, ctrl_engine);
		if (!ret) {
			ret = dsi_ctrl_set_vid_engine_state(pctrl, ctrl_engine, true);
			if (ret) {
				pr_err("%s: failed dsi_ctrl_set_vid_engine_state\n", __func__);
			}
		}
	} else {
		pr_warn("%s: no dsi_ctrl\n", __func__);
	}

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int drm_cmn_ctrl_video_engine(struct dsi_display *pdisp, u8 onoff)
{
	struct dsi_display *display = pdisp;
	int ret = 0;

	pr_debug("%s: %s video engine\n", __func__,
						(onoff ? "start" : "stop"));

	if (!display) {
		pr_err("%s: no display\n", __func__);
		ret = -EINVAL;
		goto error;
	}
	pr_debug("%s: display= %p\n", __func__, display);

	ret = drm_cmn_video_engine(display->ctrl[0].ctrl, onoff);
	if (!ret) {
		goto error;
	}

	if (display->ctrl_count > 1) {
		ret = drm_cmn_video_engine(display->ctrl[1].ctrl, onoff);
		if (!ret) {
			goto error;
		}
	}

	pr_debug("%s: out\n", __func__);

error:
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_dsi_cmds_transfer(struct dsi_display *pdisp, struct dsi_cmd_desc cmds[], int cmd_cnt)
{
	int ret = 0;
	int ret2 = 0;
	int i;

	if (!pdisp || !pdisp->panel || !pdisp->panel->host ||
	    !pdisp->panel->host->ops || !cmds) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	/* Avoid sending DCS commands when ESD recovery is pending */
	if (atomic_read(&pdisp->panel->esd_recovery_pending)) {
		pr_debug("ESD recovery pending\n");
		return 0;
	}

	ret = dsi_display_clk_ctrl(pdisp->dsi_clk_handle,
			DSI_ALL_CLKS, DSI_CLK_ON);
	if (ret) {
		pr_err("%s: failed to enable all DSI clocks, rc=%d\n",
		       pdisp->name, ret);
		goto error_enable_dsi_clocks;
	}

	ret = dsi_display_cmd_engine_ctrl(pdisp, true);
	if (ret) {
		pr_err("%s: set cmd engine enable err ret=%d\n",
							__func__, ret);
		goto error_disable_dsi_clocks;
	}

	for (i = 0; i < cmd_cnt; i++) {
		if (cmds[i].last_command) {
			cmds[i].msg.flags |=  MIPI_DSI_MSG_LASTCOMMAND;
		}

		ret = pdisp->panel->host->ops->transfer(pdisp->panel->host,
				&cmds[i].msg);
		if (ret < 0) {
			pr_err("%s: cmd transfer failed ret=%d\n",
								__func__, ret);
			break;
		}
		if ((cmds[i].msg.type == MIPI_DSI_DCS_READ) &&
						(cmds[i].msg.rx_len != ret)) {
			pr_err("%s: cmd transfer failed "
						"read size req=%ld read=%d\n",
					__func__, cmds[i].msg.rx_len, ret);
			break;
		}
		if (cmds[i].post_wait_ms) {
			usleep_range(cmds[i].post_wait_ms * 1000,
					((cmds[i].post_wait_ms * 1000) + 10));
		}
	}

	if (ret > 0) {
		ret = 0;
	}

	ret2 = dsi_display_cmd_engine_ctrl(pdisp, false);
	if (ret2) {
		pr_err("%s: set cmd engine disable err ret=%d\n",
							__func__, ret2);
		ret = ret2;
	}

error_disable_dsi_clocks:

	ret2 = dsi_display_clk_ctrl(pdisp->dsi_clk_handle,
			DSI_ALL_CLKS, DSI_CLK_OFF);
	if (ret2) {
		pr_err("%s: failed to disable all DSI clocks, rc=%d\n",
		       pdisp->name, ret2);
		ret = ret2;
	}

error_enable_dsi_clocks:
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_panel_cmds_transfer(struct dsi_display *pdisp, struct dsi_cmd_desc cmds[], int cmd_cnt)
{
	int ret = 0;

	if (!pdisp || !pdisp->panel || !cmds) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	/* acquire panel_lock to make sure no commands are in progress */
	dsi_panel_acquire_panel_lock(pdisp->panel);

	ret = drm_cmn_dsi_cmds_transfer(pdisp, cmds, cmd_cnt);

	/* release panel_lock */
	dsi_panel_release_panel_lock(pdisp->panel);

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_panel_cmds_transfer_videoctrl(struct dsi_display *pdisp, struct dsi_cmd_desc cmds[], int cmd_cnt)
{
	int ret = 0;
	int ret2 = 0;


	if (!pdisp || !pdisp->panel) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	ret = drm_cmn_video_transfer_ctrl(pdisp, false);
	if (ret) {
		pr_err("%s: failed stop_video\n", __func__);
		return ret;
	}


	ret = drm_cmn_panel_cmds_transfer(pdisp, cmds, cmd_cnt);

	ret2 = drm_cmn_video_transfer_ctrl(pdisp, true);
	if (ret2) {
		pr_err("%s: failed start_video\n", __func__);
		ret = ret2;
	}
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_panel_dcs_write0(struct dsi_display *pdisp, char addr)
{
	int ret = 0;
	int msg_flags;
	char payload[2];
	struct dsi_cmd_desc drm_cmds;

//	msg_flags = MIPI_DSI_MSG_UNICAST;
	msg_flags = MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST;

	memset(&payload, 0, sizeof(payload));
	payload[0] = addr;
//	payload[1] = data;

	memset(&drm_cmds, 0, sizeof(drm_cmds));
	drm_cmds.msg.channel  = 0;
	drm_cmds.msg.type = MIPI_DSI_DCS_SHORT_WRITE;
	drm_cmds.msg.flags    = msg_flags;
	drm_cmds.msg.ctrl     = 0;	/* 0 = dsi-master */
	drm_cmds.msg.tx_len   = 1;
	drm_cmds.msg.tx_buf   = payload;
	drm_cmds.last_command = 1;
	drm_cmds.post_wait_ms = 0;

	ret = drm_cmn_panel_cmds_transfer(pdisp, &drm_cmds, 1);

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_panel_dcs_write1(struct dsi_display *pdisp,
				char addr,
				char data)
{
	int ret = 0;
	int msg_flags;
	char payload[2];
	struct dsi_cmd_desc drm_cmds;

//	msg_flags = MIPI_DSI_MSG_UNICAST;
	msg_flags = MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST;

	memset(&payload, 0, sizeof(payload));
	payload[0] = addr;
	payload[1] = data;

	memset(&drm_cmds, 0, sizeof(drm_cmds));
	drm_cmds.msg.channel  = 0;
#ifdef CONFIG_SHARP_PANEL_ROSETTA
	drm_cmds.msg.type = MIPI_DSI_DCS_LONG_WRITE;
#else /* CONFIG_SHARP_PANEL_XXX */
	drm_cmds.msg.type = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
#endif /* CONFIG_SHARP_PANEL_XXX */
	drm_cmds.msg.flags    = msg_flags;
	drm_cmds.msg.ctrl     = 0;	/* 0 = dsi-master */
	drm_cmds.msg.tx_len   = 2;
	drm_cmds.msg.tx_buf   = payload;
	drm_cmds.last_command = 1;
	drm_cmds.post_wait_ms = 0;

	ret = drm_cmn_panel_cmds_transfer(pdisp, &drm_cmds, 1);

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_panel_dcs_read(struct dsi_display *pdisp, char addr,
				int rlen, char *rbuf)
{
	int ret = 0;
	unsigned char addr_value[2] = {addr, 0x00};
	struct dsi_cmd_desc read_cmd[] = {
		{{0, MIPI_DSI_DCS_READ,
			MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST,
			0, 0, 1, addr_value, rlen, rbuf}, 1, 0},
	};

	if (!pdisp) {
		return -EINVAL;
	}

	ret = drm_cmn_panel_cmds_transfer(pdisp, read_cmd, ARRAY_SIZE(read_cmd));
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_dsi_dcs_read(struct dsi_display *pdisp, char addr,
				int rlen, char *rbuf)
{
	int ret = 0;
	unsigned char addr_value[2] = {addr, 0x00};
	struct dsi_cmd_desc read_cmd[] = {
		{{0, MIPI_DSI_DCS_READ,
			MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST,
			0, 0, 1, addr_value, rlen, rbuf}, 1, 0},
	};

	if (!pdisp) {
		return -EINVAL;
	}

	ret = drm_cmn_dsi_cmds_transfer(pdisp, read_cmd, ARRAY_SIZE(read_cmd));
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_get_panel_type(void)
{
	return drm_cmn_ctx.lcd_switch;
}

bool drm_cmn_is_diag_mode(void)
{
	bool ret = false;

#ifdef CONFIG_SHARP_BOOT
	if ((drm_cmn_ctx.boot_mode == SH_BOOT_D) || (drm_cmn_ctx.boot_mode == SH_BOOT_F_F)) {
		ret = true;
	}
#endif /* CONFIG_SHARP_BOOT */

	return ret;
}

unsigned char drm_cmn_get_hw_handset(void)
{
	return drm_cmn_ctx.hw_handset;
}

int drm_cmn_get_hw_revision(void)
{
	return drm_cmn_ctx.hw_revision;
}

unsigned char drm_cmn_get_upperunit(void)
{
	return drm_cmn_ctx.upperunit;
}

unsigned char drm_cmn_get_disp_on_status(void)
{
	return drm_cmn_ctx.disp_on_status;
}

unsigned char drm_cmn_get_otp_bias(void)
{
	return drm_cmn_ctx.otp_bias;
}

unsigned char drm_cmn_get_otp_vgsp(void)
{
	return drm_cmn_ctx.otp_vgsp;
}

unsigned char drm_cmn_get_otp_gamma_shift(void)
{
	return drm_cmn_ctx.otp_gamma_shift;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
void drm_cmn_set_wakelock(struct wake_lock *wakelock, bool en)
{
	pr_debug("%s:start. caller=%pS\n",
			__func__, __builtin_return_address(0));

	if (en) {
		if (!drm_cmn_wake_lock_active(wakelock)) {
			drm_cmn_wake_lock(wakelock);
			pr_debug("%s: wake_lock\n", __func__);
		}
	} else {
		if (drm_cmn_wake_lock_active(wakelock)) {
			drm_cmn_wake_unlock(wakelock);
			pr_debug("%s: wake_unlock\n", __func__);
		}
	}
}
