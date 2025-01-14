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

#include <linux/types.h>
#include <linux/ctype.h>
#include <linux/fb.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/fb.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/iopoll.h>
#ifdef CONFIG_SHARP_SHTERM
#include "misc/shterm_k.h"
#endif /* CONFIG_SHARP_SHTERM */
#include "../dsi/dsi_display.h"
#ifdef PLL_CALC_DATA
#undef PLL_CALC_DATA
#endif
#include "../dsi/dsi_hw.h"
#include "../dsi/dsi_ctrl_reg.h"
#include "drm_cmn.h"
#include "../sde/sde_connector.h"
#include "../msm_drv.h"
#include "../sde/sde_encoder.h"

#define DETIN_NAME	"detin"
#define MIPIERR_NAME	"mipierr"
#define DISPONCHK_NAME	"disponchk"

#define DETIN_ERR_COUNT	5
#define DETIN_ERR_ARRAY	(DETIN_ERR_COUNT-1)
#define DETIN_ERR_TIME	10000

#if defined(CONFIG_ARCH_DOZLE) || defined(CONFIG_ARCH_JUDAU)
#define RETRY_RECOVERY_CNT 5
#else /* CONFIG_ARCH_DOZLE || CONFIG_ARCH_JUDAU */
#define RETRY_RECOVERY_CNT 3
#endif /* CONFIG_ARCH_DOZLE || CONFIG_ARCH_JUDAU */

struct drm_det_irq {
	bool enable;
	char *name;
	int irq;
	int gpio;
	int trigger;
	ktime_t err_time[DETIN_ERR_ARRAY];
	struct wake_lock wakelock;
	struct workqueue_struct *workque;
	struct work_struct work;
	void (*workqueue_handler)(struct work_struct *work);
};

static struct {
	bool panel_on;
	int recovery_cnt;
	bool retry_over;
	struct drm_det_irq detin;
	struct drm_det_irq mipierr;
	struct dsi_display *display;
	struct workqueue_struct *delayedwkq;
	struct delayed_work delayedwk;
	struct wake_lock dispon_wakelock;
	unsigned char disp_on_status;
	struct mutex drm_det_lock;
} drm_det_ctx = {0};

static int drm_det_irq_init
	(struct platform_device *pdev, struct drm_det_irq *ptr,
					struct device_node *irq_node);
static irqreturn_t drm_det_isr(int irq_num, void *data);
static int drm_det_set_irq(struct drm_det_irq *ptr, bool enable);
static void drm_det_resume(void);
static void drm_det_suspend(void);
static void drm_det_request_recovery(void);
static int drm_det_power_mode_chk(struct dsi_display *pdisp);
static int drm_det_mipierr_probe(struct platform_device *pdev,
					struct device_node *panel_np);
static void drm_det_mipierr_workqueue_handler(struct work_struct *work);
static int drm_det_mipierr_port_chk(void);
static int drm_det_detin_probe(struct platform_device *pdev,
					struct device_node *panel_np);
static void drm_det_detin_workqueue_handler(struct work_struct *work);
static int drm_det_detin_port_chk(void);
static void drm_det_dispon_recovery_work(struct work_struct *work);
#ifdef CONFIG_SHARP_SHTERM
static void drm_det_shterm_send_event(int event);
#endif /* CONFIG_SHARP_SHTERM */
static int drm_det_panel_dead(struct dsi_display *display);
static int drm_det_event_notify(struct drm_encoder  *drm_enc);
static int drm_det_dispon_recovery(void);
static int drm_det_disponchk_sub(struct dsi_display *display);
static int drm_det_chk_panel_on_sub(void);

static bool disable_det_irq = false;
static uint test_det_disponchk = 0;
static uint test_det_detlow = 0;
static uint test_det_mipierr = 0;
#ifdef CONFIG_DEBUG_FS
module_param(disable_det_irq, bool, 0600);
module_param(test_det_disponchk, uint, 0600);
module_param(test_det_detlow, uint, 0600);
module_param(test_det_mipierr, uint, 0600);
#endif /* CONFIG_DEBUG_FS */

static unsigned char drm_det_test_disponchk(unsigned char disp_on_status)
{
	unsigned char status = disp_on_status;
	int err_kind = test_det_disponchk >> 4;
	int test_num  = test_det_disponchk & 0x0F;

	if (test_num > 0) {
		switch (err_kind) {
		case DRM_PANEL_DISPONCHK_STATERR:
			status = DRM_PANEL_DISPONCHK_STATERR;
			break;
		case DRM_PANEL_DISPONCHK_READERR:
			status = DRM_PANEL_DISPONCHK_READERR;
			break;
		default:
			status = DRM_PANEL_DISPONCHK_SUCCESS;
			test_det_disponchk = 0;
			return status;
		}

		test_num--;
		pr_info("%s[TEST]force DispOnChk NG(%d)\n", __func__, test_num);
		if (test_num <= 0) {
			test_num = 0;
			err_kind = 0;
		}
		test_det_disponchk = (err_kind << 4) + test_num;
	} else {
		test_det_disponchk = 0;
	}
	return status;
}

int drm_det_init(struct dsi_display *display)
{
	int ret = 0;

	if (!display || !display->pdev) {
		pr_err("%s: Invalid input data\n", __func__);
		return ret;
	}

	if (drm_det_ctx.display) {
		pr_warn("%s:already initialized", __func__);
		return 0;
	}

	drm_det_detin_probe(display->pdev, display->pdev->dev.of_node);
	drm_det_mipierr_probe(display->pdev, display->pdev->dev.of_node);

	drm_det_ctx.display = display;
	drm_det_ctx.recovery_cnt = 0;
	drm_det_ctx.retry_over = false;
	drm_det_ctx.delayedwkq
		= create_singlethread_workqueue(DISPONCHK_NAME);
	INIT_DELAYED_WORK(&drm_det_ctx.delayedwk,
		drm_det_dispon_recovery_work);

	drm_cmn_wake_lock_init(&drm_det_ctx.dispon_wakelock,
		&display->pdev->dev, DISPONCHK_NAME);

	drm_det_ctx.disp_on_status =
		drm_cmn_get_disp_on_status();
	pr_info("%s: disp_on_status(from XBL)=%d\n",
			__func__, drm_det_ctx.disp_on_status);

	if (drm_det_ctx.disp_on_status == DRM_PANEL_DISPONCHK_SUCCESS) {
		drm_det_ctx.panel_on = true;
	}

	if (drm_cmn_get_hw_handset() == 1 && drm_cmn_get_hw_revision() < 3) {
		disable_det_irq = true;
	}

	mutex_init(&drm_det_ctx.drm_det_lock);
	pr_debug("%s:out. ret=%d, disp_on_status=%d\n",
		__func__, ret, drm_det_ctx.disp_on_status);
	return ret;
}

int drm_det_post_panel_on(void)
{
	drm_det_resume();
	return 0;
}

int drm_det_disponchk(struct dsi_display *display)
{
	if (drm_det_ctx.panel_on) {
		pr_debug("%s: Not disponchk because check result in uefi.\n",
								__func__);
		return 0;
	}

	return drm_det_disponchk_sub(display);
}

int drm_det_pre_panel_off(void)
{
	struct dsi_display *dsi_display = NULL;
	struct dsi_panel *panel = NULL;

	dsi_display = drm_det_ctx.display;
	if (!dsi_display) {
		pr_err("%s: Invalid dsi_display is NULL\n", __func__);
		return -EINVAL;
	}
	panel = dsi_display->panel;
	if (!panel) {
		pr_err("%s: Invalid dsi_panel is NULL\n", __func__);
		return -EINVAL;
	}

	drm_det_suspend();
	if (drm_det_ctx.retry_over) {
		mutex_lock(&panel->panel_lock);
		dsi_panel_set_backlight(panel, 0);
		mutex_unlock(&panel->panel_lock);
	}
	return 0;
}

bool drm_det_is_retry_over(void)
{
	bool flg = drm_det_ctx.retry_over;
	return (flg);
}

int drm_det_chk_panel_on(void)
{
	return drm_det_chk_panel_on_sub();
}

static int drm_det_disponchk_sub(struct dsi_display *display)
{
	int ret = 0;
	pr_debug("%s:in \n", __func__);

	if (!display) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	// wait 17ms
	usleep_range(17000, 17010);

	ret = drm_det_power_mode_chk(display);
	switch (ret) {
	case -EIO:
		drm_det_ctx.disp_on_status = DRM_PANEL_DISPONCHK_READERR;
		break;
	case -EINVAL:
		drm_det_ctx.disp_on_status = DRM_PANEL_DISPONCHK_STATERR;
		break;
	default:
		drm_det_ctx.disp_on_status = DRM_PANEL_DISPONCHK_SUCCESS;
		break;
	}

	pr_debug("%s:out. ret=%d, disp_on_status=%d\n",
		__func__, ret, drm_det_ctx.disp_on_status);

	return ret;
}


static int drm_det_chk_panel_on_sub(void)
{
	int ret = 0;
	unsigned char disp_on_status = drm_det_ctx.disp_on_status;

	pr_debug("%s:in , disp_on_status=%d\n", __func__, disp_on_status);

	if (test_det_disponchk > 0) {
		disp_on_status = drm_det_test_disponchk(disp_on_status);
	}

	if (disp_on_status == DRM_PANEL_DISPONCHK_READERR) {
		ret = -EIO;
#ifdef CONFIG_SHARP_SHTERM
		drm_det_shterm_send_event(SHBATTLOG_EVENT_DISP_ERR_READ_ERROR);
#endif /* CONFIG_SHARP_SHTERM */
	} else if (disp_on_status == DRM_PANEL_DISPONCHK_STATERR) {
		ret = -EINVAL;
#ifdef CONFIG_SHARP_SHTERM
		drm_det_shterm_send_event(SHBATTLOG_EVENT_DISP_ERR_DISPOFF);
#endif /* CONFIG_SHARP_SHTERM */
	} else {
		ret = 0;
	}

	if (drm_det_detin_port_chk()) {
#ifdef CONFIG_SHARP_SHTERM
		drm_det_shterm_send_event(SHBATTLOG_EVENT_DISP_ERR_SLPIN);
#endif /* CONFIG_SHARP_SHTERM */
		pr_err("%s:det port is low.\n", __func__);
		ret = -EIO;
	}
	if (drm_det_mipierr_port_chk()) {
#ifdef CONFIG_SHARP_SHTERM
		drm_det_shterm_send_event(SHBATTLOG_EVENT_DISP_ERR_MIPI_ERROR);
#endif /* CONFIG_SHARP_SHTERM */
		pr_err("%s:mipi error port is high.\n", __func__);
		ret = -EIO;
	}

	if (ret){
		if (drm_det_ctx.recovery_cnt >= RETRY_RECOVERY_CNT) {
			pr_err("%s:recovery retry over. recovery_cnt=%d\n",
				__func__, drm_det_ctx.recovery_cnt);
			drm_det_ctx.retry_over = true;
		}
		drm_det_ctx.recovery_cnt++;
		drm_det_dispon_recovery();
	} else {
		drm_det_ctx.recovery_cnt = 0;
		drm_det_ctx.retry_over = false;
	}

	pr_debug("%s:out. ret=%d, recovery_cnt=%d\n",
		__func__, ret, drm_det_ctx.recovery_cnt);

	return ret;
}

static int drm_det_dispon_recovery(void)
{
	int ret = 0;
	pr_debug("%s: in\n", __func__);
	if (!drm_det_ctx.delayedwkq) {
		pr_err("%s:failed to create_singlethread_workqueue().\n",
			__func__);
		return -EFAULT;
	}
	drm_cmn_set_wakelock(&drm_det_ctx.dispon_wakelock, true);
	ret = queue_delayed_work(
		drm_det_ctx.delayedwkq,
		&drm_det_ctx.delayedwk,
		msecs_to_jiffies(100));
	if (ret == 0) {
		drm_cmn_set_wakelock(&drm_det_ctx.dispon_wakelock, false);
		pr_err("%s:failed to queue_work(). ret=%d\n", __func__, ret);
	}
	pr_debug("%s: out ret=%d\n", __func__, ret);
	return 0;
}

static void drm_det_dispon_recovery_work(struct work_struct *work)
{
	pr_debug("%s: in\n", __func__);
	mutex_lock(&drm_det_ctx.drm_det_lock);
	if (!drm_det_ctx.panel_on) {
		pr_warn("%s: display OFF, will be exited.\n", __func__);
		goto exit;
	}
	if (atomic_read(&drm_det_ctx.display->panel->esd_recovery_pending)) {
		pr_warn("%s: ESD recovery pending\n", __func__);
		goto exit;
	}

	drm_det_request_recovery();
exit:
	mutex_unlock(&drm_det_ctx.drm_det_lock);
	pr_debug("%s: out\n", __func__);
	drm_cmn_set_wakelock(&drm_det_ctx.dispon_wakelock, false);
}

#ifdef CONFIG_SHARP_SHTERM
static void drm_det_shterm_send_event(int event)
{
	shbattlog_info_t info;
	memset(&info, 0, sizeof(info));

	info.event_num = event;
	shterm_k_set_event(&info);
}
#endif /* CONFIG_SHARP_SHTERM */

static int drm_det_power_mode_chk(struct dsi_display *pdisp)
{
	int rc = 0;
	char expect_val[] = {0x14};
	unsigned char rbuf[] = {0x00};
	char pwrmode_addr[] = {0x0A};

	rc = drm_cmn_panel_dcs_read(pdisp, pwrmode_addr[0],
					sizeof(pwrmode_addr), rbuf);
	pr_info("%s: read rc = %d, rbuf=0x%02X\n",
		__func__, rc, rbuf[0]);
	if (rc) {
		pr_err("%s: failed to read "
			"power-mode rc=%d\n", __func__,rc);
		rc = -EIO;
	// } else if (memcmp(expect_val, rbuf, sizeof(rbuf))) {
	} else if ((rbuf[0] & expect_val[0]) != expect_val[0]) {
		pr_err("%s: disp on chk ng. "
			"rbuf[0x%02X], expect_val[0x%02X]\n",
			__func__, rbuf[0], expect_val[0]);
		rc = -EINVAL;
	}
	return rc;
}

static void drm_det_request_recovery(void)
{
	struct dsi_display *display = NULL;
	struct drm_connector *conn = NULL;
	struct sde_connector *c_conn = NULL;

	display = drm_det_ctx.display;
	if (display) {
		conn = display->drm_conn;
		if (conn) {
			c_conn = to_sde_connector(conn);
			if (c_conn)
				c_conn->panel_dead = true;
		}
	}
	drm_det_panel_dead(drm_det_ctx.display);
	pr_warn("%s: recovery_cnt=%d\n", __func__, drm_det_ctx.recovery_cnt);
}

void drm_det_esd_chk_ng(void)
{
#ifdef CONFIG_SHARP_SHTERM
	drm_det_shterm_send_event(SHBATTLOG_EVENT_DISP_ERR_CHK_NG);
#endif /* CONFIG_SHARP_SHTERM */
	drm_det_request_recovery();
}

static int drm_det_panel_dead(struct dsi_display *display)
{
	int rc = 0;
	struct drm_device *dev;
	struct drm_encoder *encoder;

	if (!display) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	dev = display->drm_dev;
	if (!dev) {
		pr_err("%s: no drm_dev\n", __func__);
		return -EINVAL;
	}

	if (display->panel) {
		atomic_set(&display->panel->esd_recovery_pending, 1);
	}
	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		if (!encoder || !encoder->dev || !encoder->dev->dev_private ||
		!encoder->crtc || (encoder->encoder_type != DRM_MODE_ENCODER_DSI)) {
			continue;
		}
		drm_det_event_notify(encoder);
		break;
	}

	return  rc;
}

static int drm_det_event_notify(struct drm_encoder *drm_enc)
{
	int rc = 0;
	bool panel_dead = true;
	struct drm_event event;

	if (!drm_enc) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	/* panel dead event notify */
	if (drm_det_ctx.retry_over) {
		event.type = DRM_EVENT_PANEL_OFF;
	} else {
		event.type = DRM_EVENT_PANEL_DEAD;
	}
	event.length = sizeof(u32);
	msm_mode_object_event_notify(&drm_enc->base,
		drm_enc->dev, &event, (u8*)&panel_dead);

	sde_encoder_display_failure_notification(drm_enc, false);

	return  rc;
}

static int drm_det_irq_init(struct platform_device *pdev, struct drm_det_irq *ptr,
	struct device_node *irq_node)
{
	struct resource *pr = NULL;

	if (!pdev || !ptr || !irq_node) {
		pr_err("%s:parameter error.", __func__);
		return -EPERM;
	}

	ptr->gpio = of_get_named_gpio(irq_node, ptr->name, 0);
	if (!gpio_is_valid(ptr->gpio)) {
		pr_err("%s:failed to get named gpio().\n", ptr->name);
		return -EFAULT;
	}
	pr = platform_get_resource_byname(pdev, IORESOURCE_IRQ, ptr->name);
	if (pr != NULL) {
		ptr->irq = pr->start;
	} else if (0 <= gpio_to_irq(ptr->gpio)) {
		ptr->irq = gpio_to_irq(ptr->gpio);
	} else {
		pr_err("%s:failed to get irq().\n", ptr->name);
		return -EFAULT;
	}

	ptr->workque = create_singlethread_workqueue(ptr->name);
	if (!ptr->workque) {
		pr_err("%s:failed to create_singlethread_workqueue().\n", ptr->name);
		return -EFAULT;
	}
	if (!ptr->workqueue_handler) {
		pr_err("%s:workqueue_handler parameter error.\n", ptr->name);
		return -EPERM;
	}
	INIT_WORK(&ptr->work, ptr->workqueue_handler);
	drm_cmn_wake_lock_init(&ptr->wakelock, &pdev->dev, ptr->name);
	pr_debug("%s:gpio=%d irq=%d\n", __func__, ptr->gpio, ptr->irq);
	memset(ptr->err_time, 0, sizeof(ptr->err_time));
	return 0;
}

static irqreturn_t drm_det_isr(int irq_num, void *data)
{
	int ret;
	struct drm_det_irq *ptr;

	pr_debug("%s: irq_num=%d data=0x%p\n", __func__, irq_num, data);
	disable_irq_nosync(irq_num);

	if (!data) {
		pr_err("invalid isr parameter.\n");
		goto exit;
	}
	ptr = (struct drm_det_irq *)data;

	if (!ptr->workque) {
		pr_err("invalid work queue.\n");
		goto exit;
	}
	if (!drm_det_ctx.panel_on) {
		pr_debug("%s: display OFF, will be exited.\n", __func__);
		goto exit;
	}

	if (atomic_read(&drm_det_ctx.display->panel->esd_recovery_pending)) {
		pr_err("%s: ESD recovery sequence underway\n", __func__);
		goto exit;
	}

	drm_cmn_set_wakelock(&ptr->wakelock, true);
	ret = queue_work(ptr->workque, &ptr->work);
	if (ret == 0) {
		drm_cmn_set_wakelock(&ptr->wakelock, false);
		pr_err("%s:failed to queue_work(). ret=%d\n",
			__func__, ret);
	}
	pr_debug("%s: out\n", __func__);
exit:
	return IRQ_HANDLED;
}

static int drm_det_set_irq(struct drm_det_irq *ptr, bool enable)
{
	int ret = 0;

	if (disable_det_irq) {
		pr_debug("%s:disabe det irq.\n", __func__);
		return ret;
	}

	pr_debug("%s: data=0x%p, enable=%d\n", __func__, ptr, enable);
	if (ptr == NULL) {
		pr_err("%s:parameter error.\n", __func__);
		return -EPERM;
	}
	if (!ptr->enable) {
		pr_debug("%s:not irq configuration.\n", __func__);
		return -ENODEV;
	}
	if (enable) {
		ret = request_irq(ptr->irq, drm_det_isr,
					ptr->trigger, ptr->name, ptr);
		if (ret) {
			pr_err("%s:failed to request_irq().(ret=%d)\n"
				, ptr->name, ret);
		}
	} else {
		disable_irq(ptr->irq);
		free_irq(ptr->irq, ptr);
	}
	return ret;
}

static void drm_det_resume(void)
{
	pr_debug("%s: in\n", __func__);

	drm_det_ctx.panel_on = true;
	drm_det_set_irq(&drm_det_ctx.mipierr, true);
	drm_det_set_irq(&drm_det_ctx.detin, true);
}

static void drm_det_suspend(void)
{
	pr_debug("%s: in\n", __func__);
	if (cancel_delayed_work_sync(&drm_det_ctx.delayedwk) == true) {
		pr_debug("%s: cancel_delayed_work done.\n", __func__);
		drm_cmn_set_wakelock(&drm_det_ctx.dispon_wakelock, false);
	}
	if (drm_det_ctx.panel_on) {
		drm_det_ctx.panel_on = false;
		drm_det_set_irq(&drm_det_ctx.detin, false);
		drm_det_set_irq(&drm_det_ctx.mipierr, false);
	}
}

static int drm_det_mipierr_probe(struct platform_device *pdev,
	struct device_node *panel_np)
{
	int ret = 0;
	struct device_node *irq_node;

	pr_debug("%s: in\n", __func__);
	if (pdev == NULL || panel_np == NULL) {
		pr_err("%s:pdev parameter error.\n", __func__);
		return -EPERM;
	}

	irq_node = of_get_child_by_name(panel_np,
			"sharp,drm_panel_mipierr");

	drm_det_ctx.mipierr.name = MIPIERR_NAME;
	drm_det_ctx.mipierr.trigger = IRQF_TRIGGER_HIGH;
	drm_det_ctx.mipierr.workqueue_handler
		= drm_det_mipierr_workqueue_handler;

	if (irq_node) {
		ret = drm_det_irq_init(pdev, &drm_det_ctx.mipierr, irq_node);
	} else {
		ret = -ENODEV;
	}

	if (!ret) {
		drm_det_ctx.mipierr.enable = true;
	} else {
		pr_warn("%s:mipi_err port no config.\n", __func__);
		drm_det_ctx.mipierr.enable = false;
	}
	pr_debug("%s: out\n", __func__);
	return 0;
}

static int drm_det_mipierr_port_chk(void)
{
	if (!drm_det_ctx.mipierr.enable) {
		pr_debug("%s:not gpio configuration.\n", __func__);
		return 0;
	}
	pr_debug("%s: mipierr.gpio(%d)\n", __func__,
				drm_det_ctx.mipierr.gpio);
	if (gpio_get_value(drm_det_ctx.mipierr.gpio)) {
		pr_debug("%s: Chk NG(GPIO is High)\n", __func__);
		return -EIO;
	}
	if (test_det_mipierr > 0) {
		test_det_mipierr--;
		pr_info("%s[TEST]force MIPI error(%d)\n", __func__, test_det_mipierr);
		return -EIO;
	}
	pr_debug("%s: Chk OK(GPIO is Low)\n", __func__);
	return 0;
}

static int drm_det_dsi_cmd_bta_sw_trigger(struct dsi_ctrl *dsi_ctrl)
{
	u32 status;
	int timeout_us = 35000;
	struct dsi_ctrl_hw *ctrl = NULL;

	pr_debug("%s: in\n", __func__);

	if (dsi_ctrl == NULL) {
		pr_err("%s: dsi_ctrl is NULL.\n", __func__);
		return -EINVAL;
	}

	ctrl = &dsi_ctrl->hw;
	/* CMD_MODE_BTA_SW_TRIGGER */
	DSI_W32(ctrl, DSI_CMD_MODE_BTA_SW_TRIGGER, 0x00000001);
	wmb();

	/* Check for CMD_MODE_DMA_BUSY */
	if (readl_poll_timeout(((ctrl->base) + DSI_STATUS),
				status, ((status & 0x0010) == 0),
				0, timeout_us)) {
		pr_info("%s: timeout. status=0x%08x\n", __func__, status);
		return -EIO;
	}

	pr_debug("%s: out status=0x%08x\n", __func__, status);

	return 0;
}

static int drm_det_mipierr_clear_bta(struct dsi_display *display)
{
	int ret = 0;

	pr_debug("%s: in\n", __func__);

	if (!display) {
		pr_err("%s: Invalid input data\n", __func__);
		return ret;
	}

	drm_det_dsi_cmd_bta_sw_trigger(display->ctrl[0].ctrl);
	if (display->ctrl[1].ctrl) {
		drm_det_dsi_cmd_bta_sw_trigger(display->ctrl[1].ctrl);
	}

	pr_debug("%s:end ret=%d\n", __func__, ret);
	return 0;
}

int drm_det_mipierr_clear(struct dsi_display *display)
{
	int ret = 0;

	if (!display) {
		pr_err("%s: Invalid input data\n", __func__);
		return ret;
	}

	dsi_display_clk_ctrl(drm_det_ctx.display->dsi_clk_handle, DSI_ALL_CLKS, DSI_CLK_ON);
	dsi_display_cmd_engine_ctrl(drm_det_ctx.display, true);
	ret = drm_det_mipierr_clear_bta(display);
	dsi_display_cmd_engine_ctrl(drm_det_ctx.display, false);
	dsi_display_clk_ctrl(drm_det_ctx.display->dsi_clk_handle, DSI_ALL_CLKS, DSI_CLK_OFF);

	return ret;
}

void drm_det_mipierr_irq_clear(void)
{
	cancel_work_sync(&drm_det_ctx.mipierr.work);
	drm_det_mipierr_clear(drm_det_ctx.display);
}

int drm_det_check_mipierr_gpio(void)
{
	int ret = 0;
	ret = gpio_get_value(drm_det_ctx.mipierr.gpio);
	pr_debug("%s: mipierr.gpio(%d)=%d\n", __func__,
				drm_det_ctx.mipierr.gpio, ret);
	if (ret) {
		return -EIO;
	}
	return 0;
}

static void drm_det_mipierr_workqueue_handler(struct work_struct *work)
{
	pr_debug("%s: in\n", __func__);
	mutex_lock(&drm_det_ctx.drm_det_lock);
	if (!drm_det_ctx.panel_on) {
		pr_warn("%s: display OFF, will be exited.\n", __func__);
		goto exit;
	}
	if (atomic_read(&drm_det_ctx.display->panel->esd_recovery_pending)) {
		pr_warn("%s: ESD recovery pending\n", __func__);
		goto exit;
	}

	if (drm_det_mipierr_port_chk()) {
		pr_err("%s:mipi error port is high.\n", __func__);
#ifdef CONFIG_SHARP_SHTERM
		drm_det_shterm_send_event(SHBATTLOG_EVENT_DISP_ERR_MIPI_ERROR);
#endif /* CONFIG_SHARP_SHTERM */
		drm_det_mipierr_clear(drm_det_ctx.display);
		drm_det_request_recovery();
	} else {
		pr_debug("mipi error port is low.\n");
		enable_irq(drm_det_ctx.mipierr.irq);
	}
exit:
	mutex_unlock(&drm_det_ctx.drm_det_lock);
	pr_debug("%s: out\n", __func__);
	drm_cmn_set_wakelock(&drm_det_ctx.mipierr.wakelock, false);
	return;
}

static int drm_det_detin_probe(struct platform_device *pdev,
	struct device_node *panel_np)
{
	int ret = 0;
	struct device_node *irq_node;

	pr_debug("%s: in\n", __func__);
	if (pdev == NULL || panel_np == NULL) {
		pr_err("%s:pdev parameter error.\n", __func__);
		return -EPERM;
	}

	irq_node = of_get_child_by_name(panel_np,
			"sharp,drm_panel_det");

	drm_det_ctx.detin.name = DETIN_NAME;
#if defined(CONFIG_ARCH_DOZLE) || defined(CONFIG_ARCH_JUDAU)
	drm_det_ctx.detin.trigger = IRQF_TRIGGER_LOW;
#else
	drm_det_ctx.detin.trigger = IRQF_TRIGGER_FALLING;
#endif /* CONFIG_ARCH_DOZLE || CONFIG_ARCH_JUDAU */
	drm_det_ctx.detin.workqueue_handler
		= drm_det_detin_workqueue_handler;

	if (irq_node) {
		ret = drm_det_irq_init(pdev, &drm_det_ctx.detin, irq_node);
	} else {
		ret = -ENODEV;
	}

	if (!ret) {
		drm_det_ctx.detin.enable = true;
	} else {
		pr_warn("%s:detin port no config.\n", __func__);
		drm_det_ctx.detin.enable = false;
	}
	pr_debug("%s: out\n", __func__);
	return 0;
}

static int drm_det_detin_port_chk(void)
{
	if (!drm_det_ctx.detin.enable) {
		pr_debug("%s:not gpio configuration.\n", __func__);
		return 0;
	}
	if (!gpio_get_value(drm_det_ctx.detin.gpio)) {
		return -EIO;
	}
	if (test_det_detlow > 0) {
		test_det_detlow--;
		pr_info("%s[TEST]force Detin error(%d)\n", __func__, test_det_detlow);
		return -EIO;
	}
	return 0;
}

static void drm_det_detin_set_time(ktime_t cur_time) {
	int i = 0;

	for (i=(DETIN_ERR_ARRAY-1); i>0; i--) {
		drm_det_ctx.detin.err_time[i] =
				drm_det_ctx.detin.err_time[i-1];
	}

	drm_det_ctx.detin.err_time[0] = cur_time;
}

static void drm_det_detin_chk_cnt(void)
{
	ktime_t cur_time;
	s64 diff_ms;

	cur_time = ktime_get_boottime();
	if (drm_det_ctx.detin.err_time[DETIN_ERR_ARRAY-1] != 0) {
		diff_ms = ktime_ms_delta(cur_time,
			drm_det_ctx.detin.err_time[DETIN_ERR_ARRAY-1]);
		if (diff_ms <= DETIN_ERR_TIME) {
			goto error;
		}
	}

	drm_det_detin_set_time(cur_time);
	return;

error:
#if !defined(CONFIG_ARCH_DOZLE) && !defined(CONFIG_ARCH_JUDAU)
	drm_det_ctx.retry_over = true;
#endif /* not CONFIG_ARCH_DOZLE && CONFIG_ARCH_JUDAU */
	memset(drm_det_ctx.detin.err_time, 0,
					sizeof(drm_det_ctx.detin.err_time));
	return;
}

static void drm_det_detin_workqueue_handler(struct work_struct *work)
{
	pr_debug("%s: in\n", __func__);
	mutex_lock(&drm_det_ctx.drm_det_lock);
	if (!drm_det_ctx.panel_on) {
		pr_warn("%s: display OFF, will be exited.\n", __func__);
		goto exit;
	}
	if (atomic_read(&drm_det_ctx.display->panel->esd_recovery_pending)) {
		pr_warn("%s: ESD recovery pending\n", __func__);
		goto exit;
	}

	if (drm_det_detin_port_chk()) {
		pr_err("%s:det port is low.\n", __func__);
#ifdef CONFIG_SHARP_SHTERM
		drm_det_shterm_send_event(SHBATTLOG_EVENT_DISP_ERR_DRVOFF);
#endif /* CONFIG_SHARP_SHTERM */
		drm_det_detin_chk_cnt();
		drm_det_request_recovery();
	} else {
		pr_debug("det port is high.\n");
		enable_irq(drm_det_ctx.detin.irq);
	}
exit:
	mutex_unlock(&drm_det_ctx.drm_det_lock);
	pr_debug("%s: out\n", __func__);
	drm_cmn_set_wakelock(&drm_det_ctx.detin.wakelock, false);
	return;
}
