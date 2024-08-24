// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt)	"BATTERY_CHG: %s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/rpmsg.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/soc/qcom/battery_charger.h>
#ifdef CONFIG_BATTERY_SHARP
#include <linux/reboot.h>
#endif /* CONFIG_BATTERY_SHARP */

#define MSG_OWNER_BC			32778
#define MSG_TYPE_REQ_RESP		1
#define MSG_TYPE_NOTIFY			2

/* opcode for battery charger */
#define BC_SET_NOTIFY_REQ		0x04
#define BC_NOTIFY_IND			0x07
#define BC_BATTERY_STATUS_GET		0x30
#define BC_BATTERY_STATUS_SET		0x31
#define BC_USB_STATUS_GET		0x32
#define BC_USB_STATUS_SET		0x33
#define BC_WLS_STATUS_GET		0x34
#define BC_WLS_STATUS_SET		0x35
#define BC_SHIP_MODE_REQ_SET		0x36
#define BC_WLS_FW_CHECK_UPDATE		0x40
#define BC_WLS_FW_PUSH_BUF_REQ		0x41
#define BC_WLS_FW_UPDATE_STATUS_RESP	0x42
#define BC_WLS_FW_PUSH_BUF_RESP		0x43
#define BC_WLS_FW_GET_VERSION		0x44
#define BC_SHUTDOWN_NOTIFY		0x47
#ifdef CONFIG_BATTERY_SHARP
#define BC_BATTERY_DEBUG_PARAM_GET	0x45
#define BC_BATTERY_DEBUG_PARAM_SET	0x46

#define BC_DIAG_MODE_REQ_SET		0xA0
#define BC_CHG_MODE_REQ_SET			0xA1
#define BC_AFP_MODE_REQ_SET			0xA2
#define BC_DIRECT_CHARGE_MODE_REQ_SET	0xA3
#endif /* CONFIG_BATTERY_SHARP */
#define BC_GENERIC_NOTIFY		0x80

/* Generic definitions */
#define MAX_STR_LEN			128
#define BC_WAIT_TIME_MS			1000
#define WLS_FW_PREPARE_TIME_MS		300
#define WLS_FW_WAIT_TIME_MS		500
#define WLS_FW_UPDATE_TIME_MS		1000
#define WLS_FW_BUF_SIZE			128
#define DEFAULT_RESTRICT_FCC_UA		1000000

enum psy_type {
	PSY_TYPE_BATTERY,
	PSY_TYPE_USB,
	PSY_TYPE_WLS,
	PSY_TYPE_MAX,
};

enum ship_mode_type {
	SHIP_MODE_PMIC,
	SHIP_MODE_PACK_SIDE,
};

/* property ids */
enum battery_property_id {
	BATT_STATUS,
	BATT_HEALTH,
	BATT_PRESENT,
	BATT_CHG_TYPE,
	BATT_CAPACITY,
	BATT_SOH,
	BATT_VOLT_OCV,
	BATT_VOLT_NOW,
	BATT_VOLT_MAX,
	BATT_CURR_NOW,
	BATT_CHG_CTRL_LIM,
	BATT_CHG_CTRL_LIM_MAX,
	BATT_TEMP,
	BATT_TECHNOLOGY,
	BATT_CHG_COUNTER,
	BATT_CYCLE_COUNT,
	BATT_CHG_FULL_DESIGN,
	BATT_CHG_FULL,
	BATT_MODEL_NAME,
	BATT_TTF_AVG,
	BATT_TTE_AVG,
	BATT_RESISTANCE,
	BATT_POWER_NOW,
	BATT_POWER_AVG,
#ifdef CONFIG_BATTERY_SHARP
	BATT_CURR_AVG,
	BATT_CC_SAFETY_LEVEL,
	BATT_INPUT_SUSPEND,
	BATT_DEPLETED_INIT,
	BATT_DEPLETED_VER,
	BATT_DEPLETED_AVE,
	BATT_DEPLETED_POS,
	BATT_DEPLETED_RESULT,
	BATT_AGE_LEVEL,
	BATT_CHARGING_CYCLE_INIT,
	BATT_CHARGING_CYCLE_CC,
	BATT_CHARGING_CYCLE_RTC,
	BATT_OFFCHG_MODE,
	BATT_CHARGER_ERROR_STATUS,
	BATT_MSOC,
	BATT_DIRECT_CHARGE_MODE,
#endif /* CONFIG_BATTERY_SHARP */
	BATT_PROP_MAX,
};

enum usb_property_id {
	USB_ONLINE,
	USB_VOLT_NOW,
	USB_VOLT_MAX,
	USB_CURR_NOW,
	USB_CURR_MAX,
	USB_INPUT_CURR_LIMIT,
	USB_TYPE,
	USB_ADAP_TYPE,
	USB_MOISTURE_DET_EN,
	USB_MOISTURE_DET_STS,
	USB_TEMP,
	USB_REAL_TYPE,
	USB_TYPEC_COMPLIANT,
#ifdef CONFIG_BATTERY_SHARP
	USB_PRESENT,
	USB_PARTNER_TYPE,
#endif /* CONFIG_BATTERY_SHARP */
	USB_PROP_MAX,
};

enum wireless_property_id {
	WLS_ONLINE,
	WLS_VOLT_NOW,
	WLS_VOLT_MAX,
	WLS_CURR_NOW,
	WLS_CURR_MAX,
	WLS_TYPE,
	WLS_BOOST_EN,
	WLS_PROP_MAX,
};

enum {
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP = 0x80,
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3,
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5,
};

#ifdef CONFIG_BATTERY_SHARP
enum debug_property_id {
  DEBUG_PARAM_ICL = 0,
  DEBUG_PARAM_SUSPEND_USB,
  DEBUG_PARAM_DISABLE_MCP,
  DEBUG_PARAM_CHG_TEMP_OVR,
  DEBUG_PARAM_SET_MAIN_TEMP,
  DEBUG_PARAM_SET_SMB_TEMP,
  DEBUG_PARAM_WLS_TX_SOURCE,
  DEBUG_PARAM_EN_OPT_MTC_CHG,
  DEBUG_PARAM_FCC,
  DEBUG_PARAM_FV,
  DEBUG_PARAM_CHG_ENABLED,
  DEBUG_PARAM_MSOC,
  DEBUG_PARAM_USB_THERM,
  DEBUG_PARAM_TYPEC_SAFETY_DISABLED,
  DEBUG_PARAM_MAX,
};

enum bc_chg_mode {
  BC_CHG_MODE_ENABLED = 0,
  BC_CHG_MODE_FEEDING,
  BC_CHG_MODE_DISABLED,
  BC_CHG_MODE_MAX
};
#endif /* CONFIG_BATTERY_SHARP */

struct battery_charger_set_notify_msg {
	struct pmic_glink_hdr	hdr;
	u32			battery_id;
	u32			power_state;
	u32			low_capacity;
	u32			high_capacity;
};

struct battery_charger_notify_msg {
	struct pmic_glink_hdr	hdr;
	u32			notification;
};

struct battery_charger_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			battery_id;
	u32			property_id;
	u32			value;
};

struct battery_charger_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	u32			value;
	u32			ret_code;
};

struct battery_model_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	char			model[MAX_STR_LEN];
};

struct wireless_fw_check_req {
	struct pmic_glink_hdr	hdr;
	u32			fw_version;
	u32			fw_size;
	u32			fw_crc;
};

struct wireless_fw_check_resp {
	struct pmic_glink_hdr	hdr;
	u32			ret_code;
};

struct wireless_fw_push_buf_req {
	struct pmic_glink_hdr	hdr;
	u8			buf[WLS_FW_BUF_SIZE];
	u32			fw_chunk_id;
};

struct wireless_fw_push_buf_resp {
	struct pmic_glink_hdr	hdr;
	u32			fw_update_status;
};

struct wireless_fw_update_status {
	struct pmic_glink_hdr	hdr;
	u32			fw_update_done;
};

struct wireless_fw_get_version_req {
	struct pmic_glink_hdr	hdr;
};

struct wireless_fw_get_version_resp {
	struct pmic_glink_hdr	hdr;
	u32			fw_version;
};

struct battery_charger_ship_mode_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			ship_mode_type;
};

#ifdef CONFIG_BATTERY_SHARP
struct battery_charger_diag_mode_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			diag_mode;
	u32			ret_code;
};

struct battery_charger_get_debug_param_req_msg {
	struct pmic_glink_hdr	hdr;
	u32 debug_param_property_id;
	u32 data;
	u32 status;
};

struct battery_charger_set_debug_param_req_msg {
	struct pmic_glink_hdr	hdr;
	u32 debug_param_property_id;
	u32 data;
};

struct battery_generic_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			ret_code;
};

struct battery_charger_chg_mode_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			chg_mode;
	u32			ret_code;
};

struct battery_charger_afp_mode_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			afp_mode;
	u32			ret_code;
};

struct battery_charger_direct_charge_mode_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			direct_charge_mode;
	u32			ret_code;
};
#endif /* CONFIG_BATTERY_SHARP */

struct psy_state {
	struct power_supply	*psy;
	char			*model;
	const int		*map;
	u32			*prop;
	u32			prop_count;
	u32			opcode_get;
	u32			opcode_set;
};

struct battery_chg_dev {
	struct device			*dev;
	struct class			battery_class;
	struct pmic_glink_client	*client;
	struct mutex			rw_lock;
	struct completion		ack;
	struct completion		fw_buf_ack;
	struct completion		fw_update_ack;
	struct psy_state		psy_list[PSY_TYPE_MAX];
	struct dentry			*debugfs_dir;
	u32				*thermal_levels;
	const char			*wls_fw_name;
	int				curr_thermal_level;
	int				num_thermal_levels;
	atomic_t			state;
	struct work_struct		subsys_up_work;
	struct work_struct		usb_type_work;
#ifdef CONFIG_BATTERY_SHARP
	struct work_struct		chg_ctrl_limit_work;
	struct work_struct		restart_work;
#endif /* CONFIG_BATTERY_SHARP */
	int				fake_soc;
	bool				block_tx;
	bool				ship_mode_en;
#ifdef CONFIG_BATTERY_SHARP
	int					diag_mode;
	int					chg_mode;
	int					fake_cc_safety_level;
	int					debug_prop[DEBUG_PARAM_MAX];
	bool				afp_mode_en;
	bool				direct_charge_mode_en;
	bool				evt_battery_detected;
	bool				offcharge_mode;
#endif /* CONFIG_BATTERY_SHARP */
	bool				debug_battery_detected;
	bool				wls_fw_update_reqd;
	u32				wls_fw_version;
	u16				wls_fw_crc;
	struct notifier_block		reboot_notifier;
	u32				thermal_fcc_ua;
	u32				restrict_fcc_ua;
	u32				last_fcc_ua;
	u32				usb_icl_ua;
	bool				restrict_chg_en;
	/* To track the driver initialization status */
	bool				initialized;
};

static const int battery_prop_map[BATT_PROP_MAX] = {
	[BATT_STATUS]		= POWER_SUPPLY_PROP_STATUS,
	[BATT_HEALTH]		= POWER_SUPPLY_PROP_HEALTH,
	[BATT_PRESENT]		= POWER_SUPPLY_PROP_PRESENT,
	[BATT_CHG_TYPE]		= POWER_SUPPLY_PROP_CHARGE_TYPE,
	[BATT_CAPACITY]		= POWER_SUPPLY_PROP_CAPACITY,
	[BATT_VOLT_OCV]		= POWER_SUPPLY_PROP_VOLTAGE_OCV,
	[BATT_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[BATT_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[BATT_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[BATT_CHG_CTRL_LIM]	= POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	[BATT_CHG_CTRL_LIM_MAX]	= POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	[BATT_TEMP]		= POWER_SUPPLY_PROP_TEMP,
	[BATT_TECHNOLOGY]	= POWER_SUPPLY_PROP_TECHNOLOGY,
	[BATT_CHG_COUNTER]	= POWER_SUPPLY_PROP_CHARGE_COUNTER,
	[BATT_CYCLE_COUNT]	= POWER_SUPPLY_PROP_CYCLE_COUNT,
	[BATT_CHG_FULL_DESIGN]	= POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	[BATT_CHG_FULL]		= POWER_SUPPLY_PROP_CHARGE_FULL,
	[BATT_MODEL_NAME]	= POWER_SUPPLY_PROP_MODEL_NAME,
	[BATT_TTF_AVG]		= POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	[BATT_TTE_AVG]		= POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	[BATT_POWER_NOW]	= POWER_SUPPLY_PROP_POWER_NOW,
	[BATT_POWER_AVG]	= POWER_SUPPLY_PROP_POWER_AVG,
#ifdef CONFIG_BATTERY_SHARP
	[BATT_CURR_AVG]		= POWER_SUPPLY_PROP_CURRENT_AVG,
	[BATT_CC_SAFETY_LEVEL] = POWER_SUPPLY_PROP_CC_SAFETY_LEVEL,
	[BATT_INPUT_SUSPEND]= POWER_SUPPLY_PROP_INPUT_SUSPEND,
	[BATT_DEPLETED_INIT]   = POWER_SUPPLY_PROP_DEPLETED_INIT,
	[BATT_DEPLETED_VER]    = POWER_SUPPLY_PROP_DEPLETED_VER,
	[BATT_DEPLETED_AVE]    = POWER_SUPPLY_PROP_DEPLETED_AVE,
	[BATT_DEPLETED_POS]    = POWER_SUPPLY_PROP_DEPLETED_POS,
	[BATT_DEPLETED_RESULT] = POWER_SUPPLY_PROP_DEPLETED_RESULT,
	[BATT_AGE_LEVEL]       = POWER_SUPPLY_PROP_BATT_AGE_LEVEL,
	[BATT_CHARGING_CYCLE_INIT] = POWER_SUPPLY_PROP_CHARGING_CYCLE_INIT,
	[BATT_CHARGING_CYCLE_CC]   = POWER_SUPPLY_PROP_CHARGING_CYCLE_CC,
	[BATT_CHARGING_CYCLE_RTC]  = POWER_SUPPLY_PROP_CHARGING_CYCLE_RTC,
	[BATT_OFFCHG_MODE]          = POWER_SUPPLY_PROP_OFFCHG_MODE,
	[BATT_CHARGER_ERROR_STATUS] = POWER_SUPPLY_PROP_CHARGER_ERROR_STATUS,
	[BATT_MSOC]                 = POWER_SUPPLY_PROP_MSOC,
	[BATT_DIRECT_CHARGE_MODE]   = POWER_SUPPLY_PROP_DIRECT_CHARGE_MODE,
#endif /* CONFIG_BATTERY_SHARP */
};

static const int usb_prop_map[USB_PROP_MAX] = {
	[USB_ONLINE]		= POWER_SUPPLY_PROP_ONLINE,
	[USB_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[USB_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[USB_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[USB_CURR_MAX]		= POWER_SUPPLY_PROP_CURRENT_MAX,
	[USB_INPUT_CURR_LIMIT]	= POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	[USB_ADAP_TYPE]		= POWER_SUPPLY_PROP_USB_TYPE,
	[USB_TEMP]		= POWER_SUPPLY_PROP_TEMP,
#ifdef CONFIG_BATTERY_SHARP
	[USB_PRESENT]		= POWER_SUPPLY_PROP_PRESENT,
	[USB_PARTNER_TYPE]  = POWER_SUPPLY_PROP_PARTNER_TYPE,
#endif /* CONFIG_BATTERY_SHARP */
};

static const int wls_prop_map[WLS_PROP_MAX] = {
	[WLS_ONLINE]		= POWER_SUPPLY_PROP_ONLINE,
	[WLS_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[WLS_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[WLS_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[WLS_CURR_MAX]		= POWER_SUPPLY_PROP_CURRENT_MAX,
};

/* Standard usb_type definitions similar to power_supply_sysfs.c */
static const char * const power_supply_usb_type_text[] = {
	"Unknown", "SDP", "DCP", "CDP", "ACA", "C",
	"PD", "PD_DRP", "PD_PPS", "BrickID"
};

/* Custom usb_type definitions */
static const char * const qc_power_supply_usb_type_text[] = {
	"HVDCP", "HVDCP_3", "HVDCP_3P5"
};

#ifdef CONFIG_BATTERY_SHARP
static struct battery_chg_dev *the_bcdev = NULL;

static int battery_chg_get_debug_praram(
    struct battery_chg_dev *bcdev,
    enum debug_property_id debug_param_id);

static bool need_req_msg( enum power_supply_property prop );
#endif /* CONFIG_BATTERY_SHARP */

static int battery_chg_fw_write(struct battery_chg_dev *bcdev, void *data,
				int len)
{
	int rc;

	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		pr_debug("glink state is down\n");
		return -ENOTCONN;
	}

	reinit_completion(&bcdev->fw_buf_ack);
	rc = pmic_glink_write(bcdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->fw_buf_ack,
					msecs_to_jiffies(WLS_FW_WAIT_TIME_MS));
		if (!rc) {
			pr_err("Error, timed out sending message\n");
			return -ETIMEDOUT;
		}

		rc = 0;
	}

	return rc;
}

static int battery_chg_write(struct battery_chg_dev *bcdev, void *data,
				int len)
{
	int rc;

	/*
	 * When the subsystem goes down, it's better to return the last
	 * known values until it comes back up. Hence, return 0 so that
	 * pmic_glink_write() is not attempted until pmic glink is up.
	 */
	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		pr_debug("glink state is down\n");
		return 0;
	}

	if (bcdev->debug_battery_detected && bcdev->block_tx)
		return 0;

	mutex_lock(&bcdev->rw_lock);
	reinit_completion(&bcdev->ack);
	rc = pmic_glink_write(bcdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->ack,
					msecs_to_jiffies(BC_WAIT_TIME_MS));
		if (!rc) {
			pr_err("Error, timed out sending message\n");
			mutex_unlock(&bcdev->rw_lock);
			return -ETIMEDOUT;
		}

		rc = 0;
	}
	mutex_unlock(&bcdev->rw_lock);

	return rc;
}

static int write_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id, u32 val)
{
	struct battery_charger_req_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.battery_id = 0;
	req_msg.value = val;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_set;

	pr_debug("psy: %s prop_id: %u val: %u\n", pst->psy->desc->name,
		req_msg.property_id, val);

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static int read_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id)
{
	struct battery_charger_req_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.battery_id = 0;
	req_msg.value = 0;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;

	pr_debug("psy: %s prop_id: %u\n", pst->psy->desc->name,
		req_msg.property_id);

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static int get_property_id(struct psy_state *pst,
			enum power_supply_property prop)
{
	u32 i;

	for (i = 0; i < pst->prop_count; i++)
		if (pst->map[i] == prop)
			return i;

	pr_err("No property id for property %d in psy %s\n", prop,
		pst->psy->desc->name);

	return -ENOENT;
}

static void battery_chg_notify_enable(struct battery_chg_dev *bcdev)
{
	struct battery_charger_set_notify_msg req_msg = { { 0 } };
	int rc;

	/* Send request to enable notification */
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_NOTIFY;
	req_msg.hdr.opcode = BC_SET_NOTIFY_REQ;

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0)
		pr_err("Failed to enable notification rc=%d\n", rc);
}

static void battery_chg_state_cb(void *priv, enum pmic_glink_state state)
{
	struct battery_chg_dev *bcdev = priv;

	pr_debug("state: %d\n", state);

	atomic_set(&bcdev->state, state);
	if (state == PMIC_GLINK_STATE_UP)
		schedule_work(&bcdev->subsys_up_work);
}

/**
 * qti_battery_charger_get_prop() - Gets the property being requested
 *
 * @name: Power supply name
 * @prop_id: Property id to be read
 * @val: Pointer to value that needs to be updated
 *
 * Return: 0 if success, negative on error.
 */
int qti_battery_charger_get_prop(const char *name,
				enum battery_charger_prop prop_id, int *val)
{
	struct power_supply *psy;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst;
	int rc = 0;

	if (prop_id >= BATTERY_CHARGER_PROP_MAX)
		return -EINVAL;

	if (strcmp(name, "battery") && strcmp(name, "usb") &&
	    strcmp(name, "wireless"))
		return -EINVAL;

	psy = power_supply_get_by_name(name);
	if (!psy)
		return -ENODEV;

	bcdev = power_supply_get_drvdata(psy);
	if (!bcdev)
		return -ENODEV;

	power_supply_put(psy);

	switch (prop_id) {
	case BATTERY_RESISTANCE:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		rc = read_property_id(bcdev, pst, BATT_RESISTANCE);
		if (!rc)
			*val = pst->prop[BATT_RESISTANCE];
		break;
	default:
		break;
	}

	return rc;
}
EXPORT_SYMBOL(qti_battery_charger_get_prop);

static bool validate_message(struct battery_charger_resp_msg *resp_msg,
				size_t len)
{
	if (len != sizeof(*resp_msg)) {
		pr_err("Incorrect response length %zu for opcode %#x\n", len,
			resp_msg->hdr.opcode);
		return false;
	}

	if (resp_msg->ret_code) {
		pr_err("Error in response for opcode %#x prop_id %u, rc=%d\n",
			resp_msg->hdr.opcode, resp_msg->property_id,
			(int)resp_msg->ret_code);
		return false;
	}

	return true;
}

#define MODEL_DEBUG_BOARD	"Debug_Board"
#ifdef CONFIG_BATTERY_SHARP
#define MODEL_EVT_BATTERY	"SHARP_8350_1ST_EVT"
#endif /* CONFIG_BATTERY_SHARP */
static void handle_message(struct battery_chg_dev *bcdev, void *data,
				size_t len)
{
	struct battery_charger_resp_msg *resp_msg = data;
	struct battery_model_resp_msg *model_resp_msg = data;
	struct wireless_fw_check_resp *fw_check_msg;
	struct wireless_fw_push_buf_resp *fw_resp_msg;
	struct wireless_fw_update_status *fw_update_msg;
	struct wireless_fw_get_version_resp *fw_ver_msg;
	struct psy_state *pst;
#ifdef CONFIG_BATTERY_SHARP
	struct battery_charger_diag_mode_req_msg *set_diag_mode_msg;
	struct battery_charger_get_debug_param_req_msg *get_debug_param_msg;
	struct battery_generic_resp_msg *generic_resp_msg;
	struct battery_charger_chg_mode_req_msg *set_chg_mode_msg;
	struct battery_charger_afp_mode_req_msg *set_afp_mode_msg;
	struct battery_charger_direct_charge_mode_req_msg *set_direct_charge_mode_msg;
#endif /* CONFIG_BATTERY_SHARP */
	bool ack_set = false;

#ifdef CONFIG_BATTERY_SHARP
	pr_debug("message opcode:%u\n", resp_msg->hdr.opcode);
#endif /* CONFIG_BATTERY_SHARP */

	switch (resp_msg->hdr.opcode) {
	case BC_BATTERY_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

		/* Handle model response uniquely as it's a string */
		if (pst->model && len == sizeof(*model_resp_msg)) {
			memcpy(pst->model, model_resp_msg->model, MAX_STR_LEN);
			ack_set = true;
			bcdev->debug_battery_detected = !strcmp(pst->model,
					MODEL_DEBUG_BOARD);
#ifdef CONFIG_BATTERY_SHARP
			bcdev->evt_battery_detected = !strcmp(pst->model,
					MODEL_EVT_BATTERY);
#endif /* CONFIG_BATTERY_SHARP */
			break;
		}

		/* Other response should be of same type as they've u32 value */
		if (validate_message(resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
#ifdef CONFIG_BATTERY_SHARP
			if (resp_msg->property_id == BATT_CC_SAFETY_LEVEL) {
				if ((resp_msg->value == POWER_SUPPLY_CC_SAFETY_LEVEL3)
					&& (bcdev->offcharge_mode)) {
					schedule_work(&bcdev->restart_work);
				}

				if ((bcdev->fake_cc_safety_level >= POWER_SUPPLY_CC_SAFETY_LEVEL0)
					&& (bcdev->fake_cc_safety_level <= POWER_SUPPLY_CC_SAFETY_LEVEL3)){
					pst->prop[resp_msg->property_id] = bcdev->fake_cc_safety_level;
				}
			}
#endif /* CONFIG_BATTERY_SHARP */
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case BC_USB_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		if (validate_message(resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case BC_WLS_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_WLS];
		if (validate_message(resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case BC_BATTERY_STATUS_SET:
	case BC_USB_STATUS_SET:
	case BC_WLS_STATUS_SET:
		if (validate_message(data, len))
			ack_set = true;

		break;
	case BC_SET_NOTIFY_REQ:
		/* Always ACK response for notify request */
		ack_set = true;
		break;
	case BC_WLS_FW_CHECK_UPDATE:
		if (len == sizeof(*fw_check_msg)) {
			fw_check_msg = data;
			if (fw_check_msg->ret_code == 1)
				bcdev->wls_fw_update_reqd = true;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for wls_fw_check_update\n",
				len);
		}
		break;
	case BC_WLS_FW_PUSH_BUF_RESP:
		if (len == sizeof(*fw_resp_msg)) {
			fw_resp_msg = data;
			if (fw_resp_msg->fw_update_status == 1)
				complete(&bcdev->fw_buf_ack);
		} else {
			pr_err("Incorrect response length %zu for wls_fw_push_buf_resp\n",
				len);
		}
		break;
	case BC_WLS_FW_UPDATE_STATUS_RESP:
		if (len == sizeof(*fw_update_msg)) {
			fw_update_msg = data;
			if (fw_update_msg->fw_update_done == 1)
				complete(&bcdev->fw_update_ack);
			else
				pr_err("Wireless FW update not done %d\n",
					(int)fw_update_msg->fw_update_done);
		} else {
			pr_err("Incorrect response length %zu for wls_fw_update_status_resp\n",
				len);
		}
		break;
	case BC_WLS_FW_GET_VERSION:
		if (len == sizeof(*fw_ver_msg)) {
			fw_ver_msg = data;
			bcdev->wls_fw_version = fw_ver_msg->fw_version;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for wls_fw_get_version\n",
				len);
		}
		break;
#ifdef CONFIG_BATTERY_SHARP
	case BC_DIAG_MODE_REQ_SET:
		if (len == sizeof(*set_diag_mode_msg)) {
			set_diag_mode_msg = data;
			if (!set_diag_mode_msg->ret_code) {
				pr_info("BC_DIAG_MODE_REQ_SET ret_code:%d\n", set_diag_mode_msg->ret_code);
			} else {
				pr_err("Failed to set diag mode ret_code:%d\n", set_diag_mode_msg->ret_code);
			}
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for set_diag_mode_msg\n",
				len);
		}
		break;
	case BC_BATTERY_DEBUG_PARAM_GET:
		if (len == sizeof(*get_debug_param_msg)) {
			get_debug_param_msg = data;
			if (!get_debug_param_msg->status) {
				bcdev->debug_prop[get_debug_param_msg->debug_param_property_id] = get_debug_param_msg->data;
				pr_info("DEBUG_PARAM_GET property_id:%d, data:%d\n", get_debug_param_msg->debug_param_property_id, get_debug_param_msg->data);
			} else {
				if (get_debug_param_msg->debug_param_property_id == DEBUG_PARAM_MSOC) {
					bcdev->debug_prop[get_debug_param_msg->debug_param_property_id] = -EINVAL;
				}
				pr_err("Failed to get debug param status:%d\n", get_debug_param_msg->status);
			}
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for get_debug_param_msg\n",
				len);
		}
		break;
	case BC_BATTERY_DEBUG_PARAM_SET:
		if (len == sizeof(*generic_resp_msg)) {
			generic_resp_msg = data;
			pr_info("DEBUG_PARAM_SET ret_code:%d\n", generic_resp_msg->ret_code);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for set_debug_param_msg\n",
				len);
		}
		break;
	case BC_CHG_MODE_REQ_SET:
		if (len == sizeof(*set_chg_mode_msg)) {
			set_chg_mode_msg = data;
			if (!set_chg_mode_msg->ret_code) {
				pr_info("BC_CHG_MODE_REQ_SET ret_code:%d\n", set_chg_mode_msg->ret_code);
			} else {
				pr_err("Failed to set charge mode ret_code:%d\n", set_chg_mode_msg->ret_code);
			}
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for set_chg_mode_msg\n",
				len);
		}
		break;
	case BC_AFP_MODE_REQ_SET:
		if (len == sizeof(*set_afp_mode_msg)) {
			set_afp_mode_msg = data;
			if (!set_afp_mode_msg->ret_code) {
				pr_info("BC_AFP_MODE_REQ_SET ret_code:%d\n", set_afp_mode_msg->ret_code);
			} else {
				pr_err("Failed to set afp mode ret_code:%d\n", set_afp_mode_msg->ret_code);
			}
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for set_afp_mode_msg\n",
				len);
		}
		break;
	case BC_DIRECT_CHARGE_MODE_REQ_SET:
		if (len == sizeof(*set_direct_charge_mode_msg)) {
			set_direct_charge_mode_msg = data;
			if (!set_direct_charge_mode_msg->ret_code) {
				pr_info("BC_DIRECT_CHARGE_MODE_REQ_SET ret_code:%d\n", set_direct_charge_mode_msg->ret_code);
			} else {
				pr_err("Failed to set direct charge mode ret_code:%d\n", set_direct_charge_mode_msg->ret_code);
			}
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for set_direct_charge_mode_msg\n",
				len);
		}
		break;
#endif /* CONFIG_BATTERY_SHARP */
	default:
		pr_err("Unknown opcode: %u\n", resp_msg->hdr.opcode);
		break;
	}

	if (ack_set)
		complete(&bcdev->ack);
}

static struct power_supply_desc usb_psy_desc;

static void battery_chg_update_usb_type_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev, usb_type_work);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_ADAP_TYPE);
	if (rc < 0) {
		pr_err("Failed to read USB_ADAP_TYPE rc=%d\n", rc);
		return;
	}

	/* Reset usb_icl_ua whenever USB adapter type changes */
	if (pst->prop[USB_ADAP_TYPE] != POWER_SUPPLY_USB_TYPE_SDP &&
	    pst->prop[USB_ADAP_TYPE] != POWER_SUPPLY_USB_TYPE_PD)
		bcdev->usb_icl_ua = 0;

	pr_debug("usb_adap_type: %u\n", pst->prop[USB_ADAP_TYPE]);

	switch (pst->prop[USB_ADAP_TYPE]) {
	case POWER_SUPPLY_USB_TYPE_SDP:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB;
		break;
	case POWER_SUPPLY_USB_TYPE_DCP:
	case POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID:
	case QTI_POWER_SUPPLY_USB_TYPE_HVDCP:
	case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3:
	case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case POWER_SUPPLY_USB_TYPE_CDP:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case POWER_SUPPLY_USB_TYPE_ACA:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case POWER_SUPPLY_USB_TYPE_C:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_TYPE_C;
		break;
	case POWER_SUPPLY_USB_TYPE_PD:
	case POWER_SUPPLY_USB_TYPE_PD_DRP:
	case POWER_SUPPLY_USB_TYPE_PD_PPS:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_PD;
		break;
	default:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB;
		break;
	}
}

#ifdef CONFIG_BATTERY_SHARP
static int battery_psy_set_charge_current(struct battery_chg_dev *bcdev, int val);
static void battery_chg_update_chg_ctrl_limit_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev, chg_ctrl_limit_work);
	int rc;

	rc = battery_psy_set_charge_current(bcdev, bcdev->curr_thermal_level);
	pr_debug("charge_control_limit update:%d, rc:%d\n", bcdev->curr_thermal_level, rc);
}

static void battery_restart_work(struct work_struct *work)
{
	pr_info("battery_restart_work\n");
	kernel_restart(NULL);
}
#endif /* CONFIG_BATTERY_SHARP */

static void handle_notification(struct battery_chg_dev *bcdev, void *data,
				size_t len)
{
	struct battery_charger_notify_msg *notify_msg = data;
	struct psy_state *pst = NULL;

	if (len != sizeof(*notify_msg)) {
		pr_err("Incorrect response length %zu\n", len);
		return;
	}

	pr_debug("notification: %#x\n", notify_msg->notification);

	switch (notify_msg->notification) {
	case BC_BATTERY_STATUS_GET:
	case BC_GENERIC_NOTIFY:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
#ifdef CONFIG_BATTERY_SHARP
		if (pst) {
			if (pst->prop[BATT_STATUS] != POWER_SUPPLY_STATUS_FULL) {
				schedule_work(&bcdev->chg_ctrl_limit_work);
			}
		}
#endif /* CONFIG_BATTERY_SHARP */
		break;
	case BC_USB_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		schedule_work(&bcdev->usb_type_work);
		break;
	case BC_WLS_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_WLS];
		break;
	default:
		break;
	}

	if (pst && pst->psy) {
		/*
		 * For charger mode, keep the device awake at least for 50 ms
		 * so that device won't enter suspend when a non-SDP charger
		 * is removed. This would allow the userspace process like
		 * "charger" to be able to read power supply uevents to take
		 * appropriate actions (e.g. shutting down when the charger is
		 * unplugged).
		 */
		power_supply_changed(pst->psy);
		pm_wakeup_dev_event(bcdev->dev, 50, true);
	}
}

static int battery_chg_callback(void *priv, void *data, size_t len)
{
	struct pmic_glink_hdr *hdr = data;
	struct battery_chg_dev *bcdev = priv;

	pr_debug("owner: %u type: %u opcode: %#x len: %zu\n", hdr->owner,
		hdr->type, hdr->opcode, len);

	if (!bcdev->initialized) {
		pr_debug("Driver initialization failed: Dropping glink callback message: state %d\n",
			 bcdev->state);
		return 0;
	}

	if (hdr->opcode == BC_NOTIFY_IND)
		handle_notification(bcdev, data, len);
	else
		handle_message(bcdev, data, len);

	return 0;
}

static int wls_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_WLS];
	int prop_id, rc;

	pval->intval = -ENODATA;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	pval->intval = pst->prop[prop_id];

	return 0;
}

static int wls_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	return 0;
}

static int wls_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	return 0;
}

static enum power_supply_property wls_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const struct power_supply_desc wls_psy_desc = {
	.name			= "wireless",
	.type			= POWER_SUPPLY_TYPE_WIRELESS,
	.properties		= wls_props,
	.num_properties		= ARRAY_SIZE(wls_props),
	.get_property		= wls_psy_get_prop,
	.set_property		= wls_psy_set_prop,
	.property_is_writeable	= wls_psy_prop_is_writeable,
};

static const char *get_usb_type_name(u32 usb_type)
{
	u32 i;

	if (usb_type >= QTI_POWER_SUPPLY_USB_TYPE_HVDCP &&
	    usb_type <= QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5) {
		for (i = 0; i < ARRAY_SIZE(qc_power_supply_usb_type_text);
		     i++) {
			if (i == (usb_type - QTI_POWER_SUPPLY_USB_TYPE_HVDCP))
				return qc_power_supply_usb_type_text[i];
		}
		return "Unknown";
	}

	for (i = 0; i < ARRAY_SIZE(power_supply_usb_type_text); i++) {
		if (i == usb_type)
			return power_supply_usb_type_text[i];
	}

	return "Unknown";
}

static int usb_psy_set_icl(struct battery_chg_dev *bcdev, u32 prop_id, int val)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	u32 temp;
	int rc;

	rc = read_property_id(bcdev, pst, USB_ADAP_TYPE);
	if (rc < 0) {
		pr_err("Failed to read prop USB_ADAP_TYPE, rc=%d\n", rc);
		return rc;
	}

	/* Allow this only for SDP or USB_PD and not for other charger types */
	if (pst->prop[USB_ADAP_TYPE] != POWER_SUPPLY_USB_TYPE_SDP &&
	    pst->prop[USB_ADAP_TYPE] != POWER_SUPPLY_USB_TYPE_PD)
		return -EINVAL;

	/*
	 * Input current limit (ICL) can be set by different clients. E.g. USB
	 * driver can request for a current of 500/900 mA depending on the
	 * port type. Also, clients like EUD driver can pass 0 or -22 to
	 * suspend or unsuspend the input for its use case.
	 */

	temp = val;
	if (val < 0)
		temp = UINT_MAX;

	rc = write_property_id(bcdev, pst, prop_id, temp);
	if (rc < 0) {
		pr_err("Failed to set ICL (%u uA) rc=%d\n", temp, rc);
	} else {
		pr_debug("Set ICL to %u\n", temp);
		bcdev->usb_icl_ua = temp;
	}

	return rc;
}

static int usb_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int prop_id, rc;

	pval->intval = -ENODATA;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	pval->intval = pst->prop[prop_id];
	if (prop == POWER_SUPPLY_PROP_TEMP)
		pval->intval = DIV_ROUND_CLOSEST((int)pval->intval, 10);

	return 0;
}

static int usb_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int prop_id, rc = 0;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		rc = usb_psy_set_icl(bcdev, prop_id, pval->intval);
		break;
	default:
		break;
	}

	return rc;
}

static int usb_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_TEMP,
#ifdef CONFIG_BATTERY_SHARP
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_PARTNER_TYPE,
#endif /* CONFIG_BATTERY_SHARP */
};

static enum power_supply_usb_type usb_psy_supported_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_ACA,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_PD_PPS,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID,
};

static struct power_supply_desc usb_psy_desc = {
	.name			= "usb",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= usb_props,
	.num_properties		= ARRAY_SIZE(usb_props),
	.get_property		= usb_psy_get_prop,
	.set_property		= usb_psy_set_prop,
	.usb_types		= usb_psy_supported_types,
	.num_usb_types		= ARRAY_SIZE(usb_psy_supported_types),
	.property_is_writeable	= usb_psy_prop_is_writeable,
};

static int __battery_psy_set_charge_current(struct battery_chg_dev *bcdev,
					u32 fcc_ua)
{
	int rc;

	if (bcdev->restrict_chg_en) {
		fcc_ua = min_t(u32, fcc_ua, bcdev->restrict_fcc_ua);
		fcc_ua = min_t(u32, fcc_ua, bcdev->thermal_fcc_ua);
	}

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_BATTERY],
				BATT_CHG_CTRL_LIM, fcc_ua);
	if (rc < 0) {
		pr_err("Failed to set FCC %u, rc=%d\n", fcc_ua, rc);
	} else {
		pr_debug("Set FCC to %u uA\n", fcc_ua);
		bcdev->last_fcc_ua = fcc_ua;
	}

	return rc;
}

#ifdef CONFIG_BATTERY_SHARP
static int battery_psy_set_batt_age_level(struct battery_chg_dev *bcdev,
					int batt_age_level)
{
	int rc;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_BATTERY],
				BATT_AGE_LEVEL, batt_age_level);
	if (rc < 0) {
		pr_err("Failed to set batt_age_level %u, rc=%d\n", batt_age_level, rc);
	} else {
		pr_debug("Set batt_age_level to %d", batt_age_level);
	}

	return rc;
}

static int battery_chg_set_chg_mode(struct battery_chg_dev *bcdev);
#endif /* CONFIG_BATTERY_SHARP */
static int battery_psy_set_charge_current(struct battery_chg_dev *bcdev,
					int val)
{
	int rc;
	u32 fcc_ua, prev_fcc_ua;
#ifdef CONFIG_BATTERY_SHARP
	struct psy_state *pst;
#endif /* CONFIG_BATTERY_SHARP */

	if (!bcdev->num_thermal_levels)
		return 0;

	if (bcdev->num_thermal_levels < 0) {
		pr_err("Incorrect num_thermal_levels\n");
		return -EINVAL;
	}

#ifdef CONFIG_BATTERY_SHARP
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	if (pst == NULL) {
		pr_err("Battery power_supply is null\n");
		return -EINVAL;
	}

	if ((pst->prop[BATT_STATUS] == POWER_SUPPLY_STATUS_FULL)
		&& ((val == bcdev->num_thermal_levels) || (val == bcdev->num_thermal_levels + 1))
		)
	{
		bcdev->curr_thermal_level = val;
		pr_debug("Ignore system_temp_level=%d in FULL(%d)\n", val, pst->prop[BATT_STATUS]);
		return 0;
	}

	if (val == bcdev->num_thermal_levels + 1) {
		/* stop feeding power */
		if (bcdev->chg_mode != BC_CHG_MODE_DISABLED) {
			bcdev->chg_mode = BC_CHG_MODE_DISABLED;
			battery_chg_set_chg_mode(bcdev);
		}
		bcdev->curr_thermal_level = val;
		pr_info("charge_control_limit=%d. Stop feeding power.\n", bcdev->curr_thermal_level);
		return 0;
	} else if (val == bcdev->num_thermal_levels) {
		/* feed power without charge */
		if (bcdev->chg_mode != BC_CHG_MODE_FEEDING) {
			bcdev->chg_mode = BC_CHG_MODE_FEEDING;
			battery_chg_set_chg_mode(bcdev);
		}
		bcdev->curr_thermal_level = val;
		pr_info("charge_control_limit=%d. Feed power without charge.\n", bcdev->curr_thermal_level);
		return 0;
	} else {
		if (bcdev->chg_mode != BC_CHG_MODE_ENABLED) {
		bcdev->chg_mode = BC_CHG_MODE_ENABLED;
			battery_chg_set_chg_mode(bcdev);
		}
	}
#endif /* CONFIG_BATTERY_SHARP */

	if (val < 0 || val > bcdev->num_thermal_levels)
		return -EINVAL;

	fcc_ua = bcdev->thermal_levels[val];
	prev_fcc_ua = bcdev->thermal_fcc_ua;
	bcdev->thermal_fcc_ua = fcc_ua;

	rc = __battery_psy_set_charge_current(bcdev, fcc_ua);
	if (!rc)
		bcdev->curr_thermal_level = val;
	else
		bcdev->thermal_fcc_ua = prev_fcc_ua;

	return rc;
}

static int battery_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int prop_id, rc;
#ifdef CONFIG_BATTERY_SHARP
	bool is_rc;
	rc = 0;
#endif /* CONFIG_BATTERY_SHARP */

	pval->intval = -ENODATA;

	/*
	 * The prop id of TIME_TO_FULL_NOW and TIME_TO_FULL_AVG is same.
	 * So, map the prop id of TIME_TO_FULL_AVG for TIME_TO_FULL_NOW.
	 */
	if (prop == POWER_SUPPLY_PROP_TIME_TO_FULL_NOW)
#ifdef CONFIG_BATTERY_SHARP
	{
		pval->intval = -1;
		return rc;
	}
#else
		prop = POWER_SUPPLY_PROP_TIME_TO_FULL_AVG;
#endif /* CONFIG_BATTERY_SHARP */

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

#ifdef CONFIG_BATTERY_SHARP
	is_rc = need_req_msg( prop );
	if ( !is_rc )
		goto skip_req_msg;
#endif /* CONFIG_BATTERY_SHARP */

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

#ifdef CONFIG_BATTERY_SHARP
skip_req_msg:
#endif /* CONFIG_BATTERY_SHARP */

	switch (prop) {
	case POWER_SUPPLY_PROP_MODEL_NAME:
		pval->strval = pst->model;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		pval->intval = DIV_ROUND_CLOSEST(pst->prop[prop_id], 100);
		if (IS_ENABLED(CONFIG_QTI_PMIC_GLINK_CLIENT_DEBUG) &&
		   (bcdev->fake_soc >= 0 && bcdev->fake_soc <= 100))
			pval->intval = bcdev->fake_soc;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		pval->intval = DIV_ROUND_CLOSEST((int)pst->prop[prop_id], 10);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		pval->intval = bcdev->curr_thermal_level;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		pval->intval = bcdev->num_thermal_levels;
		break;
#ifdef CONFIG_BATTERY_SHARP
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		battery_chg_get_debug_praram(bcdev, DEBUG_PARAM_SUSPEND_USB);
		pval->intval = bcdev->debug_prop[DEBUG_PARAM_SUSPEND_USB];
		break;
	case POWER_SUPPLY_PROP_OFFCHG_MODE:
		pval->intval = bcdev->offcharge_mode;
		break;
	case POWER_SUPPLY_PROP_DIRECT_CHARGE_MODE:
		pval->intval = bcdev->direct_charge_mode_en;
		break;
#endif /* CONFIG_BATTERY_SHARP */
	default:
		pval->intval = pst->prop[prop_id];
		break;
	}

	return rc;
}

static int battery_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
#ifdef CONFIG_BATTERY_SHARP
	struct psy_state *pst = NULL;
	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
#endif /* CONFIG_BATTERY_SHARP */

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		return battery_psy_set_charge_current(bcdev, pval->intval);
#ifdef CONFIG_BATTERY_SHARP
	case POWER_SUPPLY_PROP_DEPLETED_INIT:
		pst->prop[BATT_DEPLETED_INIT]   = pval->intval;
		return 0;
	case POWER_SUPPLY_PROP_DEPLETED_VER:
		pst->prop[BATT_DEPLETED_VER]    = pval->intval;
		return 0;
	case POWER_SUPPLY_PROP_DEPLETED_AVE:
		pst->prop[BATT_DEPLETED_AVE]    = pval->intval;
		return 0;
	case POWER_SUPPLY_PROP_DEPLETED_POS:
		pst->prop[BATT_DEPLETED_POS]    = pval->intval;
		return 0;
	case POWER_SUPPLY_PROP_DEPLETED_RESULT:
		pst->prop[BATT_DEPLETED_RESULT] = pval->intval;
		return 0;
	case POWER_SUPPLY_PROP_BATT_AGE_LEVEL:
		pst->prop[BATT_AGE_LEVEL] = pval->intval;
		return battery_psy_set_batt_age_level(bcdev, pval->intval);
	case POWER_SUPPLY_PROP_CHARGING_CYCLE_INIT:
		pst->prop[BATT_CHARGING_CYCLE_INIT] = pval->intval;
		return 0;
	case POWER_SUPPLY_PROP_CHARGING_CYCLE_CC:
		pst->prop[BATT_CHARGING_CYCLE_CC]   = pval->intval;
		return 0;
	case POWER_SUPPLY_PROP_CHARGING_CYCLE_RTC:
		pst->prop[BATT_CHARGING_CYCLE_RTC]  = pval->intval;
		return 0;
#endif /* CONFIG_BATTERY_SHARP */
	default:
		return -EINVAL;
	}

	return 0;
}

static int battery_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	switch (prop) {
#ifdef CONFIG_BATTERY_SHARP
	case POWER_SUPPLY_PROP_DEPLETED_INIT:
	case POWER_SUPPLY_PROP_DEPLETED_VER:
	case POWER_SUPPLY_PROP_DEPLETED_AVE:
	case POWER_SUPPLY_PROP_DEPLETED_POS:
	case POWER_SUPPLY_PROP_DEPLETED_RESULT:
	case POWER_SUPPLY_PROP_BATT_AGE_LEVEL:
	case POWER_SUPPLY_PROP_CHARGING_CYCLE_INIT:
	case POWER_SUPPLY_PROP_CHARGING_CYCLE_CC:
	case POWER_SUPPLY_PROP_CHARGING_CYCLE_RTC:
#endif /* CONFIG_BATTERY_SHARP */
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
#ifdef CONFIG_BATTERY_SHARP
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CC_SAFETY_LEVEL,
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
	POWER_SUPPLY_PROP_DEPLETED_INIT,
	POWER_SUPPLY_PROP_DEPLETED_VER,
	POWER_SUPPLY_PROP_DEPLETED_AVE,
	POWER_SUPPLY_PROP_DEPLETED_POS,
	POWER_SUPPLY_PROP_DEPLETED_RESULT,
	POWER_SUPPLY_PROP_BATT_AGE_LEVEL,
	POWER_SUPPLY_PROP_CHARGING_CYCLE_INIT,
	POWER_SUPPLY_PROP_CHARGING_CYCLE_CC,
	POWER_SUPPLY_PROP_CHARGING_CYCLE_RTC,
	POWER_SUPPLY_PROP_OFFCHG_MODE,
	POWER_SUPPLY_PROP_CHARGER_ERROR_STATUS,
	POWER_SUPPLY_PROP_MSOC,
	POWER_SUPPLY_PROP_DIRECT_CHARGE_MODE,
#endif /* CONFIG_BATTERY_SHARP */
};

static const struct power_supply_desc batt_psy_desc = {
	.name			= "battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= battery_props,
	.num_properties		= ARRAY_SIZE(battery_props),
	.get_property		= battery_psy_get_prop,
	.set_property		= battery_psy_set_prop,
	.property_is_writeable	= battery_psy_prop_is_writeable,
};

static int battery_chg_init_psy(struct battery_chg_dev *bcdev)
{
	struct power_supply_config psy_cfg = {};
	int rc;

	psy_cfg.drv_data = bcdev;
	psy_cfg.of_node = bcdev->dev->of_node;
	bcdev->psy_list[PSY_TYPE_BATTERY].psy =
		devm_power_supply_register(bcdev->dev, &batt_psy_desc,
						&psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_BATTERY].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_BATTERY].psy);
		pr_err("Failed to register battery power supply, rc=%d\n", rc);
		return rc;
	}

	bcdev->psy_list[PSY_TYPE_USB].psy =
		devm_power_supply_register(bcdev->dev, &usb_psy_desc, &psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_USB].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_USB].psy);
		pr_err("Failed to register USB power supply, rc=%d\n", rc);
		return rc;
	}

	bcdev->psy_list[PSY_TYPE_WLS].psy =
		devm_power_supply_register(bcdev->dev, &wls_psy_desc, &psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_WLS].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_WLS].psy);
		pr_err("Failed to register wireless power supply, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static void battery_chg_subsys_up_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev, subsys_up_work);
	int rc;

	battery_chg_notify_enable(bcdev);

	/*
	 * Give some time after enabling notification so that USB adapter type
	 * information can be obtained properly which is essential for setting
	 * USB ICL.
	 */
	msleep(200);

	if (bcdev->last_fcc_ua) {
		rc = __battery_psy_set_charge_current(bcdev,
				bcdev->last_fcc_ua);
		if (rc < 0)
			pr_err("Failed to set FCC (%u uA), rc=%d\n",
				bcdev->last_fcc_ua, rc);
	}

	if (bcdev->usb_icl_ua) {
		rc = usb_psy_set_icl(bcdev, USB_INPUT_CURR_LIMIT,
				bcdev->usb_icl_ua);
		if (rc < 0)
			pr_err("Failed to set ICL(%u uA), rc=%d\n",
				bcdev->usb_icl_ua, rc);
	}
}

static int wireless_fw_send_firmware(struct battery_chg_dev *bcdev,
					const struct firmware *fw)
{
	struct wireless_fw_push_buf_req msg = {};
	const u8 *ptr;
	u32 i, num_chunks, partial_chunk_size;
	int rc;

	num_chunks = fw->size / WLS_FW_BUF_SIZE;
	partial_chunk_size = fw->size % WLS_FW_BUF_SIZE;

	if (!num_chunks)
		return -EINVAL;

	pr_debug("Updating FW...\n");

	ptr = fw->data;
	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_WLS_FW_PUSH_BUF_REQ;

	for (i = 0; i < num_chunks; i++, ptr += WLS_FW_BUF_SIZE) {
		msg.fw_chunk_id = i + 1;
		memcpy(msg.buf, ptr, WLS_FW_BUF_SIZE);

		pr_debug("sending FW chunk %u\n", i + 1);
		rc = battery_chg_fw_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			return rc;
	}

	if (partial_chunk_size) {
		msg.fw_chunk_id = i + 1;
		memset(msg.buf, 0, WLS_FW_BUF_SIZE);
		memcpy(msg.buf, ptr, partial_chunk_size);

		pr_debug("sending partial FW chunk %u\n", i + 1);
		rc = battery_chg_fw_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			return rc;
	}

	return 0;
}

static int wireless_fw_check_for_update(struct battery_chg_dev *bcdev,
					u32 version, size_t size)
{
	struct wireless_fw_check_req req_msg = {};

	bcdev->wls_fw_update_reqd = false;

	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = BC_WLS_FW_CHECK_UPDATE;
	req_msg.fw_version = version;
	req_msg.fw_size = size;
	req_msg.fw_crc = bcdev->wls_fw_crc;

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

#define IDT_FW_MAJOR_VER_OFFSET		0x94
#define IDT_FW_MINOR_VER_OFFSET		0x96
static int wireless_fw_update(struct battery_chg_dev *bcdev, bool force)
{
	const struct firmware *fw;
	struct psy_state *pst;
	u32 version;
	u16 maj_ver, min_ver;
	int rc;

	pm_stay_awake(bcdev->dev);

	/*
	 * Check for USB presence. If nothing is connected, check whether
	 * battery SOC is at least 50% before allowing FW update.
	 */
	pst = &bcdev->psy_list[PSY_TYPE_USB];
	rc = read_property_id(bcdev, pst, USB_ONLINE);
	if (rc < 0)
		goto out;

	if (!pst->prop[USB_ONLINE]) {
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		rc = read_property_id(bcdev, pst, BATT_CAPACITY);
		if (rc < 0)
			goto out;

		if ((pst->prop[BATT_CAPACITY] / 100) < 50) {
			pr_err("Battery SOC should be at least 50%% or connect charger\n");
			rc = -EINVAL;
			goto out;
		}
	}

	rc = firmware_request_nowarn(&fw, bcdev->wls_fw_name, bcdev->dev);
	if (rc) {
		pr_err("Couldn't get firmware rc=%d\n", rc);
		goto out;
	}

	if (!fw || !fw->data || !fw->size) {
		pr_err("Invalid firmware\n");
		rc = -EINVAL;
		goto release_fw;
	}

	if (fw->size < SZ_16K) {
		pr_err("Invalid firmware size %zu\n", fw->size);
		rc = -EINVAL;
		goto release_fw;
	}

	maj_ver = le16_to_cpu(*(__le16 *)(fw->data + IDT_FW_MAJOR_VER_OFFSET));
	min_ver = le16_to_cpu(*(__le16 *)(fw->data + IDT_FW_MINOR_VER_OFFSET));
	version = maj_ver << 16 | min_ver;

	if (force)
		version = UINT_MAX;

	pr_debug("FW size: %zu version: %#x\n", fw->size, version);

	rc = wireless_fw_check_for_update(bcdev, version, fw->size);
	if (rc < 0) {
		pr_err("Wireless FW update not needed, rc=%d\n", rc);
		goto release_fw;
	}

	if (!bcdev->wls_fw_update_reqd) {
		pr_warn("Wireless FW update not required\n");
		goto release_fw;
	}

	/* Wait for IDT to be setup by charger firmware */
	msleep(WLS_FW_PREPARE_TIME_MS);

	reinit_completion(&bcdev->fw_update_ack);
	rc = wireless_fw_send_firmware(bcdev, fw);
	if (rc < 0) {
		pr_err("Failed to send FW chunk, rc=%d\n", rc);
		goto release_fw;
	}

	rc = wait_for_completion_timeout(&bcdev->fw_update_ack,
				msecs_to_jiffies(WLS_FW_UPDATE_TIME_MS));
	if (!rc) {
		pr_err("Error, timed out updating firmware\n");
		rc = -ETIMEDOUT;
		goto release_fw;
	} else {
		rc = 0;
	}

	pr_info("Wireless FW update done\n");

release_fw:
	bcdev->wls_fw_crc = 0;
	release_firmware(fw);
out:
	pm_relax(bcdev->dev);

	return rc;
}

static ssize_t wireless_fw_crc_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	u16 val;

	if (kstrtou16(buf, 0, &val) || !val)
		return -EINVAL;

	bcdev->wls_fw_crc = val;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_crc);

static ssize_t wireless_fw_version_show(struct class *c,
					struct class_attribute *attr,
					char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct wireless_fw_get_version_req req_msg = {};
	int rc;

	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = BC_WLS_FW_GET_VERSION;

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get FW version rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%#x\n", bcdev->wls_fw_version);
}
static CLASS_ATTR_RO(wireless_fw_version);

static ssize_t wireless_fw_force_update_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	bool val;
	int rc;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	rc = wireless_fw_update(bcdev, true);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_force_update);

static ssize_t wireless_fw_update_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	bool val;
	int rc;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	rc = wireless_fw_update(bcdev, false);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_update);

static ssize_t usb_typec_compliant_show(struct class *c,
				struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_TYPEC_COMPLIANT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			(int)pst->prop[USB_TYPEC_COMPLIANT]);
}
static CLASS_ATTR_RO(usb_typec_compliant);

static ssize_t usb_real_type_show(struct class *c,
				struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_REAL_TYPE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			get_usb_type_name(pst->prop[USB_REAL_TYPE]));
}
static CLASS_ATTR_RO(usb_real_type);

static ssize_t restrict_cur_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	u32 fcc_ua, prev_fcc_ua;

	if (kstrtou32(buf, 0, &fcc_ua) || fcc_ua > bcdev->thermal_fcc_ua)
		return -EINVAL;

	prev_fcc_ua = bcdev->restrict_fcc_ua;
	bcdev->restrict_fcc_ua = fcc_ua;
	if (bcdev->restrict_chg_en) {
		rc = __battery_psy_set_charge_current(bcdev, fcc_ua);
		if (rc < 0) {
			bcdev->restrict_fcc_ua = prev_fcc_ua;
			return rc;
		}
	}

	return count;
}

static ssize_t restrict_cur_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%u\n", bcdev->restrict_fcc_ua);
}
static CLASS_ATTR_RW(restrict_cur);

static ssize_t restrict_chg_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	bcdev->restrict_chg_en = val;
	rc = __battery_psy_set_charge_current(bcdev, bcdev->restrict_chg_en ?
			bcdev->restrict_fcc_ua : bcdev->thermal_fcc_ua);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t restrict_chg_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->restrict_chg_en);
}
static CLASS_ATTR_RW(restrict_chg);

static ssize_t fake_soc_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	bcdev->fake_soc = val;
	pr_debug("Set fake soc to %d\n", val);

	if (IS_ENABLED(CONFIG_QTI_PMIC_GLINK_CLIENT_DEBUG) && pst->psy)
		power_supply_changed(pst->psy);

	return count;
}

static ssize_t fake_soc_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->fake_soc);
}
static CLASS_ATTR_RW(fake_soc);

static ssize_t wireless_boost_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_WLS],
				WLS_BOOST_EN, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t wireless_boost_en_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_WLS];
	int rc;

	rc = read_property_id(bcdev, pst, WLS_BOOST_EN);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[WLS_BOOST_EN]);
}
static CLASS_ATTR_RW(wireless_boost_en);

static ssize_t moisture_detection_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_USB],
				USB_MOISTURE_DET_EN, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t moisture_detection_en_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_MOISTURE_DET_EN);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			pst->prop[USB_MOISTURE_DET_EN]);
}
static CLASS_ATTR_RW(moisture_detection_en);

static ssize_t moisture_detection_status_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_MOISTURE_DET_STS);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			pst->prop[USB_MOISTURE_DET_STS]);
}
static CLASS_ATTR_RO(moisture_detection_status);

static ssize_t resistance_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc;

	rc = read_property_id(bcdev, pst, BATT_RESISTANCE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[BATT_RESISTANCE]);
}
static CLASS_ATTR_RO(resistance);

static ssize_t soh_show(struct class *c, struct class_attribute *attr,
			char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc;

	rc = read_property_id(bcdev, pst, BATT_SOH);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[BATT_SOH]);
}
static CLASS_ATTR_RO(soh);

static ssize_t ship_mode_en_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtobool(buf, &bcdev->ship_mode_en))
		return -EINVAL;

	return count;
}

static ssize_t ship_mode_en_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->ship_mode_en);
}
static CLASS_ATTR_RW(ship_mode_en);

#ifdef CONFIG_BATTERY_SHARP
static int battery_chg_set_diag_mode(struct battery_chg_dev *bcdev)
{
	struct battery_charger_diag_mode_req_msg msg = { { 0 } };
	int rc;

	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_DIAG_MODE_REQ_SET;
	msg.diag_mode = bcdev->diag_mode;

	rc = battery_chg_write(bcdev, &msg, sizeof(msg));
	if (rc < 0)
		pr_emerg("Failed to write diag mode: %d\n", rc);

	return rc;
}

static ssize_t diag_mode_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtoint(buf, 0, &bcdev->diag_mode))
		return -EINVAL;

	battery_chg_set_diag_mode(bcdev);

	return count;
}

static ssize_t diag_mode_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->diag_mode);
}
static CLASS_ATTR_RW(diag_mode);

static int battery_chg_set_debug_param(struct battery_chg_dev *bcdev, enum debug_property_id debug_param_id)
{
	struct battery_charger_set_debug_param_req_msg msg = { { 0 } };
	int rc;

	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_BATTERY_DEBUG_PARAM_SET;
	msg.debug_param_property_id = debug_param_id;
	msg.data = bcdev->debug_prop[debug_param_id];

	rc = battery_chg_write(bcdev, &msg, sizeof(msg));
	if (rc < 0)
		pr_emerg("Failed to write debug param: %d\n", rc);

	return rc;
}

static int battery_chg_get_debug_praram(struct battery_chg_dev *bcdev, enum debug_property_id debug_param_id)
{
	struct battery_charger_get_debug_param_req_msg msg = { { 0 } };
	int rc;

	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_BATTERY_DEBUG_PARAM_GET;
	msg.debug_param_property_id = debug_param_id;

	rc = battery_chg_write(bcdev, &msg, sizeof(msg));
	if (rc < 0)
		pr_emerg("Failed to write debug param: %d\n", rc);

	return rc;
}

static ssize_t constant_charge_current_max_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtoint(buf, 0, &bcdev->debug_prop[DEBUG_PARAM_FCC]))
		return -EINVAL;

	battery_chg_set_debug_param(bcdev, DEBUG_PARAM_FCC);

	return count;
}

static ssize_t constant_charge_current_max_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	battery_chg_get_debug_praram(bcdev, DEBUG_PARAM_FCC);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->debug_prop[DEBUG_PARAM_FCC]);
}
static CLASS_ATTR_RW(constant_charge_current_max);

static ssize_t input_suspend_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtoint(buf, 0, &bcdev->debug_prop[DEBUG_PARAM_SUSPEND_USB]))
		return -EINVAL;

	battery_chg_set_debug_param(bcdev, DEBUG_PARAM_SUSPEND_USB);

	return count;
}

static ssize_t input_suspend_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	battery_chg_get_debug_praram(bcdev, DEBUG_PARAM_SUSPEND_USB);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->debug_prop[DEBUG_PARAM_SUSPEND_USB]);
}
static CLASS_ATTR_RW(input_suspend);

static ssize_t input_current_max_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtoint(buf, 0, &bcdev->debug_prop[DEBUG_PARAM_ICL]))
		return -EINVAL;

	battery_chg_set_debug_param(bcdev, DEBUG_PARAM_ICL);

	return count;
}

static ssize_t input_current_max_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	battery_chg_get_debug_praram(bcdev, DEBUG_PARAM_ICL);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->debug_prop[DEBUG_PARAM_ICL]);
}
static CLASS_ATTR_RW(input_current_max);

static ssize_t voltage_max_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtoint(buf, 0, &bcdev->debug_prop[DEBUG_PARAM_FV]))
		return -EINVAL;

	battery_chg_set_debug_param(bcdev, DEBUG_PARAM_FV);

	return count;
}

static ssize_t voltage_max_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	battery_chg_get_debug_praram(bcdev, DEBUG_PARAM_FV);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->debug_prop[DEBUG_PARAM_FV]);
}
static CLASS_ATTR_RW(voltage_max);

static ssize_t battery_charging_enabled_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtoint(buf, 0, &bcdev->debug_prop[DEBUG_PARAM_CHG_ENABLED]))
		return -EINVAL;

	battery_chg_set_debug_param(bcdev, DEBUG_PARAM_CHG_ENABLED);

	return count;
}

static ssize_t battery_charging_enabled_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	battery_chg_get_debug_praram(bcdev, DEBUG_PARAM_CHG_ENABLED);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->debug_prop[DEBUG_PARAM_CHG_ENABLED]);
}
static CLASS_ATTR_RW(battery_charging_enabled);

static ssize_t capacity_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	battery_chg_get_debug_praram(bcdev, DEBUG_PARAM_MSOC);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->debug_prop[DEBUG_PARAM_MSOC]);
}
static CLASS_ATTR_RO(capacity);

static ssize_t debug_usb_therm_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtoint(buf, 0, &bcdev->debug_prop[DEBUG_PARAM_USB_THERM]))
		return -EINVAL;

	battery_chg_set_debug_param(bcdev, DEBUG_PARAM_USB_THERM);

	return count;
}

static ssize_t debug_usb_therm_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	battery_chg_get_debug_praram(bcdev, DEBUG_PARAM_USB_THERM);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->debug_prop[DEBUG_PARAM_USB_THERM]);
}
static CLASS_ATTR_RW(debug_usb_therm);

static ssize_t typec_safety_disabled_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtoint(buf, 0, &bcdev->debug_prop[DEBUG_PARAM_TYPEC_SAFETY_DISABLED]))
		return -EINVAL;

	battery_chg_set_debug_param(bcdev, DEBUG_PARAM_TYPEC_SAFETY_DISABLED);

	return count;
}

static ssize_t typec_safety_disabled_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	battery_chg_get_debug_praram(bcdev, DEBUG_PARAM_TYPEC_SAFETY_DISABLED);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->debug_prop[DEBUG_PARAM_TYPEC_SAFETY_DISABLED]);
}
static CLASS_ATTR_RW(typec_safety_disabled);

static int battery_chg_set_chg_mode(struct battery_chg_dev *bcdev)
{
	struct battery_charger_chg_mode_req_msg msg = { { 0 } };
	int rc;

	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_CHG_MODE_REQ_SET;
	msg.chg_mode = bcdev->chg_mode;

	rc = battery_chg_write(bcdev, &msg, sizeof(msg));
	if (rc < 0)
		pr_emerg("Failed to write chg mode: %d\n", rc);

	return rc;
}

static ssize_t chg_mode_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtoint(buf, 0, &bcdev->chg_mode))
		return -EINVAL;

	battery_chg_set_chg_mode(bcdev);

	return count;
}

static ssize_t chg_mode_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->chg_mode);
}
static CLASS_ATTR_RW(chg_mode);

static int battery_chg_set_afp_mode(struct battery_chg_dev *bcdev)
{
	struct battery_charger_afp_mode_req_msg msg = { { 0 } };
	int rc;

	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_AFP_MODE_REQ_SET;
	msg.afp_mode = bcdev->afp_mode_en;

	rc = battery_chg_write(bcdev, &msg, sizeof(msg));
	if (rc < 0)
		pr_emerg("Failed to write afp mode: %d\n", rc);

	return rc;
}

static ssize_t afp_mode_en_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtobool(buf, &bcdev->afp_mode_en))
		return -EINVAL;

	battery_chg_set_afp_mode(bcdev);

	return count;
}

static ssize_t afp_mode_en_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->afp_mode_en);
}
static CLASS_ATTR_RW(afp_mode_en);

static int battery_chg_set_direct_charge_mode(struct battery_chg_dev *bcdev)
{
	struct battery_charger_direct_charge_mode_req_msg msg = { { 0 } };
	int rc;

	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_DIRECT_CHARGE_MODE_REQ_SET;
	msg.direct_charge_mode = bcdev->direct_charge_mode_en;

	rc = battery_chg_write(bcdev, &msg, sizeof(msg));
	if (rc < 0)
		pr_emerg("Failed to write direct charge mode: %d\n", rc);

	return rc;
}

static ssize_t direct_charge_mode_en_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtobool(buf, &bcdev->direct_charge_mode_en))
		return -EINVAL;

	battery_chg_set_direct_charge_mode(bcdev);

	return count;
}

static ssize_t direct_charge_mode_en_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->direct_charge_mode_en);
}
static CLASS_ATTR_RW(direct_charge_mode_en);

static ssize_t fake_cc_safety_level_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtoint(buf, 0, &bcdev->fake_cc_safety_level))
		return -EINVAL;

	return count;
}

static ssize_t fake_cc_safety_level_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->fake_cc_safety_level);
}
static CLASS_ATTR_RW(fake_cc_safety_level);

static bool need_req_msg(enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_DEPLETED_INIT:
	case POWER_SUPPLY_PROP_DEPLETED_VER:
	case POWER_SUPPLY_PROP_DEPLETED_AVE:
	case POWER_SUPPLY_PROP_DEPLETED_POS:
	case POWER_SUPPLY_PROP_DEPLETED_RESULT:
	case POWER_SUPPLY_PROP_CHARGING_CYCLE_INIT:
	case POWER_SUPPLY_PROP_CHARGING_CYCLE_CC:
	case POWER_SUPPLY_PROP_CHARGING_CYCLE_RTC:
		return false;
	default:
		break;
	}

	return true;
}

static int offcharge_mode_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;
	struct battery_chg_dev *bcdev = the_bcdev;

	if (bcdev == NULL) {
		pr_err("Battery power_supply is null\n");
		return -EINVAL;
	}

	ret = kstrtoint(val, 0, &value);

	if(!ret) {
		if(value == 1)
			bcdev->offcharge_mode = true;
		else if(value == 0)
			bcdev->offcharge_mode = false;
	}

	pr_info("%s offcharge mode is %d \n", __func__, value);

	return ret;
}

static int offcharge_mode_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	struct battery_chg_dev *bcdev = the_bcdev;

	if (bcdev == NULL) {
		pr_err("Battery power_supply is null\n");
		return -EINVAL;
	}

	ret = sprintf(buf, "%d", bcdev->offcharge_mode);

	return ret;
}
module_param_call(offcharge_mode, offcharge_mode_set, offcharge_mode_get, NULL, 0644);
#endif /* CONFIG_BATTERY_SHARP */

static struct attribute *battery_class_attrs[] = {
	&class_attr_soh.attr,
	&class_attr_resistance.attr,
	&class_attr_moisture_detection_status.attr,
	&class_attr_moisture_detection_en.attr,
	&class_attr_wireless_boost_en.attr,
	&class_attr_fake_soc.attr,
	&class_attr_wireless_fw_update.attr,
	&class_attr_wireless_fw_force_update.attr,
	&class_attr_wireless_fw_version.attr,
	&class_attr_wireless_fw_crc.attr,
	&class_attr_ship_mode_en.attr,
	&class_attr_restrict_chg.attr,
	&class_attr_restrict_cur.attr,
	&class_attr_usb_real_type.attr,
	&class_attr_usb_typec_compliant.attr,
#ifdef CONFIG_BATTERY_SHARP
	&class_attr_diag_mode.attr,
	&class_attr_constant_charge_current_max.attr,
	&class_attr_input_suspend.attr,
	&class_attr_input_current_max.attr,
	&class_attr_voltage_max.attr,
	&class_attr_battery_charging_enabled.attr,
	&class_attr_chg_mode.attr,
	&class_attr_afp_mode_en.attr,
	&class_attr_direct_charge_mode_en.attr,
	&class_attr_capacity.attr,
	&class_attr_fake_cc_safety_level.attr,
	&class_attr_debug_usb_therm.attr,
	&class_attr_typec_safety_disabled.attr,
#endif /* CONFIG_BATTERY_SHARP */
	NULL,
};
ATTRIBUTE_GROUPS(battery_class);

#ifdef CONFIG_DEBUG_FS
static void battery_chg_add_debugfs(struct battery_chg_dev *bcdev)
{
	int rc;
	struct dentry *dir, *file;

	dir = debugfs_create_dir("battery_charger", NULL);
	if (IS_ERR(dir)) {
		rc = PTR_ERR(dir);
		pr_err("Failed to create charger debugfs directory, rc=%d\n",
			rc);
		return;
	}

	file = debugfs_create_bool("block_tx", 0600, dir, &bcdev->block_tx);
	if (IS_ERR(file)) {
		rc = PTR_ERR(file);
		pr_err("Failed to create block_tx debugfs file, rc=%d\n",
			rc);
		goto error;
	}

	bcdev->debugfs_dir = dir;

	return;
error:
	debugfs_remove_recursive(dir);
}
#else
static void battery_chg_add_debugfs(struct battery_chg_dev *bcdev) { }
#endif

static int battery_chg_parse_dt(struct battery_chg_dev *bcdev)
{
	struct device_node *node = bcdev->dev->of_node;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int i, rc, len;
	u32 prev, val;

	of_property_read_string(node, "qcom,wireless-fw-name",
				&bcdev->wls_fw_name);

#ifdef CONFIG_BATTERY_SHARP
	rc = read_property_id(bcdev, pst, BATT_MODEL_NAME);
	if (rc < 0) {
		pr_err("Failed to read prop BATT_MODEL_NAME, rc=%d\n", rc);
		return 0;
	}
	if (bcdev->evt_battery_detected) {
		rc = of_property_count_elems_of_size(node, "sharp,thermal-mitigation-evt",
							sizeof(u32));
	} else {
		rc = of_property_count_elems_of_size(node, "qcom,thermal-mitigation",
							sizeof(u32));
	}
#else
	rc = of_property_count_elems_of_size(node, "qcom,thermal-mitigation",
						sizeof(u32));
#endif /* CONFIG_BATTERY_SHARP */
	if (rc <= 0)
		return 0;

	len = rc;

	rc = read_property_id(bcdev, pst, BATT_CHG_CTRL_LIM_MAX);
	if (rc < 0) {
		pr_err("Failed to read prop BATT_CHG_CTRL_LIM_MAX, rc=%d\n",
			rc);
		return rc;
	}

	prev = pst->prop[BATT_CHG_CTRL_LIM_MAX];

	for (i = 0; i < len; i++) {
#ifdef CONFIG_BATTERY_SHARP
		if (bcdev->evt_battery_detected) {
			rc = of_property_read_u32_index(node, "sharp,thermal-mitigation-evt",
							i, &val);
		} else {
			rc = of_property_read_u32_index(node, "qcom,thermal-mitigation",
							i, &val);
		}
#else
		rc = of_property_read_u32_index(node, "qcom,thermal-mitigation",
						i, &val);
#endif /* CONFIG_BATTERY_SHARP */
		if (rc < 0)
			return rc;

		if (val > prev) {
			pr_err("Thermal levels should be in descending order\n");
			bcdev->num_thermal_levels = -EINVAL;
			return 0;
		}

		prev = val;
	}

	bcdev->thermal_levels = devm_kcalloc(bcdev->dev, len + 1,
					sizeof(*bcdev->thermal_levels),
					GFP_KERNEL);
	if (!bcdev->thermal_levels)
		return -ENOMEM;

	/*
	 * Element 0 is for normal charging current. Elements from index 1
	 * onwards is for thermal mitigation charging currents.
	 */

	bcdev->thermal_levels[0] = pst->prop[BATT_CHG_CTRL_LIM_MAX];

#ifdef CONFIG_BATTERY_SHARP
	if (bcdev->evt_battery_detected) {
		rc = of_property_read_u32_array(node, "sharp,thermal-mitigation-evt",
						&bcdev->thermal_levels[1], len);
	} else {
		rc = of_property_read_u32_array(node, "qcom,thermal-mitigation",
						&bcdev->thermal_levels[1], len);
	}
#else
	rc = of_property_read_u32_array(node, "qcom,thermal-mitigation",
					&bcdev->thermal_levels[1], len);
#endif /* CONFIG_BATTERY_SHARP */
	if (rc < 0) {
		pr_err("Error in reading qcom,thermal-mitigation, rc=%d\n", rc);
		return rc;
	}

	bcdev->num_thermal_levels = len;
	bcdev->thermal_fcc_ua = pst->prop[BATT_CHG_CTRL_LIM_MAX];

	return 0;
}

static int battery_chg_ship_mode(struct notifier_block *nb, unsigned long code,
		void *unused)
{
	struct battery_charger_notify_msg msg_notify = { { 0 } };
	struct battery_charger_ship_mode_req_msg msg = { { 0 } };
	struct battery_chg_dev *bcdev = container_of(nb, struct battery_chg_dev,
						     reboot_notifier);
	int rc;

	msg_notify.hdr.owner = MSG_OWNER_BC;
	msg_notify.hdr.type = MSG_TYPE_NOTIFY;
	msg_notify.hdr.opcode = BC_SHUTDOWN_NOTIFY;

	rc = battery_chg_write(bcdev, &msg_notify, sizeof(msg_notify));
	if (rc < 0)
		pr_err("Failed to send shutdown notification rc=%d\n", rc);

	if (!bcdev->ship_mode_en)
		return NOTIFY_DONE;

	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_SHIP_MODE_REQ_SET;
	msg.ship_mode_type = SHIP_MODE_PMIC;

	if (code == SYS_POWER_OFF) {
		rc = battery_chg_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			pr_emerg("Failed to write ship mode: %d\n", rc);
	}

	return NOTIFY_DONE;
}

static int battery_chg_probe(struct platform_device *pdev)
{
	struct battery_chg_dev *bcdev;
	struct device *dev = &pdev->dev;
	struct pmic_glink_client_data client_data = { };
	int rc, i;
#ifdef CONFIG_BATTERY_SHARP
	struct psy_state *pst_bat = NULL;
	const int INIT_VAL        = -1;
#endif /* CONFIG_BATTERY_SHARP */

	bcdev = devm_kzalloc(&pdev->dev, sizeof(*bcdev), GFP_KERNEL);
	if (!bcdev)
		return -ENOMEM;

	bcdev->psy_list[PSY_TYPE_BATTERY].map = battery_prop_map;
	bcdev->psy_list[PSY_TYPE_BATTERY].prop_count = BATT_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_BATTERY].opcode_get = BC_BATTERY_STATUS_GET;
	bcdev->psy_list[PSY_TYPE_BATTERY].opcode_set = BC_BATTERY_STATUS_SET;
	bcdev->psy_list[PSY_TYPE_USB].map = usb_prop_map;
	bcdev->psy_list[PSY_TYPE_USB].prop_count = USB_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_USB].opcode_get = BC_USB_STATUS_GET;
	bcdev->psy_list[PSY_TYPE_USB].opcode_set = BC_USB_STATUS_SET;
	bcdev->psy_list[PSY_TYPE_WLS].map = wls_prop_map;
	bcdev->psy_list[PSY_TYPE_WLS].prop_count = WLS_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_WLS].opcode_get = BC_WLS_STATUS_GET;
	bcdev->psy_list[PSY_TYPE_WLS].opcode_set = BC_WLS_STATUS_SET;

	for (i = 0; i < PSY_TYPE_MAX; i++) {
		bcdev->psy_list[i].prop =
			devm_kcalloc(&pdev->dev, bcdev->psy_list[i].prop_count,
					sizeof(u32), GFP_KERNEL);
		if (!bcdev->psy_list[i].prop)
			return -ENOMEM;
	}

	bcdev->psy_list[PSY_TYPE_BATTERY].model =
		devm_kzalloc(&pdev->dev, MAX_STR_LEN, GFP_KERNEL);
	if (!bcdev->psy_list[PSY_TYPE_BATTERY].model)
		return -ENOMEM;

#ifdef CONFIG_BATTERY_SHARP
	the_bcdev = bcdev;
#endif /* CONFIG_BATTERY_SHARP */

	mutex_init(&bcdev->rw_lock);
	init_completion(&bcdev->ack);
	init_completion(&bcdev->fw_buf_ack);
	init_completion(&bcdev->fw_update_ack);
	INIT_WORK(&bcdev->subsys_up_work, battery_chg_subsys_up_work);
	INIT_WORK(&bcdev->usb_type_work, battery_chg_update_usb_type_work);
#ifdef CONFIG_BATTERY_SHARP
	INIT_WORK(&bcdev->chg_ctrl_limit_work, battery_chg_update_chg_ctrl_limit_work);
	INIT_WORK(&bcdev->restart_work, battery_restart_work);
#endif /* CONFIG_BATTERY_SHARP */
	atomic_set(&bcdev->state, PMIC_GLINK_STATE_UP);
	bcdev->dev = dev;

	client_data.id = MSG_OWNER_BC;
	client_data.name = "battery_charger";
	client_data.msg_cb = battery_chg_callback;
	client_data.priv = bcdev;
	client_data.state_cb = battery_chg_state_cb;

	bcdev->client = pmic_glink_register_client(dev, &client_data);
	if (IS_ERR(bcdev->client)) {
		rc = PTR_ERR(bcdev->client);
		if (rc != -EPROBE_DEFER)
			dev_err(dev, "Error in registering with pmic_glink %d\n",
				rc);
		return rc;
	}

	bcdev->initialized = true;
	bcdev->reboot_notifier.notifier_call = battery_chg_ship_mode;
	bcdev->reboot_notifier.priority = 255;
	register_reboot_notifier(&bcdev->reboot_notifier);

	rc = battery_chg_parse_dt(bcdev);
	if (rc < 0) {
		dev_err(dev, "Failed to parse dt rc=%d\n", rc);
		goto error;
	}

#ifdef CONFIG_BATTERY_SHARP
	bcdev->afp_mode_en = false;
	bcdev->direct_charge_mode_en = false;
	bcdev->fake_cc_safety_level = -1;
	bcdev->offcharge_mode = false;

	pst_bat = &bcdev->psy_list[PSY_TYPE_BATTERY];
	if (pst_bat) {
		pst_bat->prop[BATT_DEPLETED_INIT]       = INIT_VAL;
		pst_bat->prop[BATT_DEPLETED_VER]        = INIT_VAL;
		pst_bat->prop[BATT_DEPLETED_AVE]        = INIT_VAL;
		pst_bat->prop[BATT_DEPLETED_POS]        = INIT_VAL;
		pst_bat->prop[BATT_DEPLETED_RESULT]     = INIT_VAL;
		pst_bat->prop[BATT_CHARGING_CYCLE_INIT] = INIT_VAL;
		pst_bat->prop[BATT_CHARGING_CYCLE_CC]   = INIT_VAL;
		pst_bat->prop[BATT_CHARGING_CYCLE_RTC]  = INIT_VAL;
	}
#endif /* CONFIG_BATTERY_SHARP */
	bcdev->restrict_fcc_ua = DEFAULT_RESTRICT_FCC_UA;
	platform_set_drvdata(pdev, bcdev);
	bcdev->fake_soc = -EINVAL;
	rc = battery_chg_init_psy(bcdev);
	if (rc < 0)
		goto error;

	bcdev->battery_class.name = "qcom-battery";
	bcdev->battery_class.class_groups = battery_class_groups;
	rc = class_register(&bcdev->battery_class);
	if (rc < 0) {
		dev_err(dev, "Failed to create battery_class rc=%d\n", rc);
		goto error;
	}

	battery_chg_add_debugfs(bcdev);
	battery_chg_notify_enable(bcdev);
	device_init_wakeup(bcdev->dev, true);
	schedule_work(&bcdev->usb_type_work);

	return 0;
error:
	bcdev->initialized = false;
	complete(&bcdev->ack);
	pmic_glink_unregister_client(bcdev->client);
	unregister_reboot_notifier(&bcdev->reboot_notifier);
	return rc;
}

static int battery_chg_remove(struct platform_device *pdev)
{
	struct battery_chg_dev *bcdev = platform_get_drvdata(pdev);
	int rc;

	device_init_wakeup(bcdev->dev, false);
	debugfs_remove_recursive(bcdev->debugfs_dir);
	class_unregister(&bcdev->battery_class);
	unregister_reboot_notifier(&bcdev->reboot_notifier);
	rc = pmic_glink_unregister_client(bcdev->client);
	if (rc < 0) {
		pr_err("Error unregistering from pmic_glink, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static const struct of_device_id battery_chg_match_table[] = {
	{ .compatible = "qcom,battery-charger" },
	{},
};

static struct platform_driver battery_chg_driver = {
	.driver = {
		.name = "qti_battery_charger",
		.of_match_table = battery_chg_match_table,
	},
	.probe = battery_chg_probe,
	.remove = battery_chg_remove,
};
module_platform_driver(battery_chg_driver);

MODULE_DESCRIPTION("QTI Glink battery charger driver");
MODULE_LICENSE("GPL v2");
