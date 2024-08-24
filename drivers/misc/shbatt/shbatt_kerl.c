/* drivers/misc/shbatt/shbatt_kerl.c
 *
 * Copyright (C) 2021 SHARP CORPORATION All rights reserved.
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

/*+-----------------------------------------------------------------------------+*/
/*| @ DEFINE COMPILE SWITCH :                                                   |*/
/*+-----------------------------------------------------------------------------+*/
/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :                                                            |*/
/*+-----------------------------------------------------------------------------+*/

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/alarmtimer.h>	/* Timer */
#include <linux/time.h>			/* Timer */
#include <linux/namei.h>
#include <linux/of.h>
#include <linux/rtc.h>
#include <linux/bitops.h>
#include <linux/qti_power_supply.h>

#include "misc/shbatt_kerl.h"
#include "shbatt_type.h"
#include "misc/shpwr_log.h"
#include "soc/qcom/sh_smem.h"
#include "soc/qcom/sharp/shdiag_smd.h"
#ifdef CONFIG_SHARP_SHTERM
#include "misc/shterm_k.h"
#endif /* CONFIG_SHARP_SHTERM */
#include <linux/notifier.h>
#include <drm/drm_panel.h>

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL MACRO DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/
#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "SHBATT:%s: " fmt, __func__

#define SHBATT_ERROR(x...)	SHPWR_LOG(SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,x)
#define SHBATT_INFO(x...)	SHPWR_LOG(SHPWR_LOG_LEVEL_INFO,SHPWR_LOG_TYPE_BATT,x)
#define SHBATT_TRACE(x...)	SHPWR_LOG(SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,x)

#define SHBATT_DEV_NAME						"shbatt"
#define SHBATT_OBJ_NAME						"shbatt_kerl"
#define SHBATT_ATTR_ARRAY_END_NAME			"END_NULL"
#define SHBATT_DEPLETED_JUDGE_AVERAGE_COUNT 5

#define SHBATT_WAKE_CTL(x)										\
{																\
	do															\
	{															\
		if(x == 0)												\
		{														\
			SHBATT_TRACE("[P] %s SHBATT_WAKE_CTL(0) call shbatt_wakeup_source_num=%d \n",__FUNCTION__, atomic_read(&shbatt_wakeup_source_num));	\
			if(atomic_dec_return(&shbatt_wakeup_source_num) == 0)	\
			{													\
				if(shbatt_wakeup_source->active) 				\
				{												\
					__pm_relax(shbatt_wakeup_source); 				\
					SHBATT_TRACE("[P] %s SHBATT_WAKE_CTL(0) done shbatt_wakeup_source_num=%d \n",__FUNCTION__, atomic_read(&shbatt_wakeup_source_num));	\
				}												\
			}													\
		}														\
		else													\
		{														\
			SHBATT_TRACE("[P] %s SHBATT_WAKE_CTL(1) call shbatt_wakeup_source_num=%d \n",__FUNCTION__, atomic_read(&shbatt_wakeup_source_num));	\
			if(atomic_inc_return(&shbatt_wakeup_source_num) == 1)	\
			{													\
				SHBATT_TRACE("[P] %s SHBATT_WAKE_CTL(1) done shbatt_wakeup_source_num=%d \n",__FUNCTION__, atomic_read(&shbatt_wakeup_source_num));	\
				__pm_stay_awake(shbatt_wakeup_source);					\
			}													\
		}														\
	} while(0); 												\
}

#define SHBATT_ATTR_END											\
{																\
	.attr  =													\
	{															\
		.name = SHBATT_ATTR_ARRAY_END_NAME,						\
		.mode = S_IRUGO | S_IWUSR | S_IWGRP						\
	},															\
	.show  = NULL,												\
	.store = NULL,												\
}

#define SHPWR_LOG_INFO(fmt, ...) { \
     pr_info(fmt, ##__VA_ARGS__); \
}

/*+-----------------------------------------------------------------------------+*/
/*| @ VALUE DEFINE DECLARE :                                                    |*/
/*+-----------------------------------------------------------------------------+*/
#define UNINITIALIZED						-1
#define SHBATT_ENODATA						0

/* wake_lock */
#define SHBATT_LOCK_FUNC_LEN				64	/* SH_PWR_DEBUG T.B.D */
#define DEFAULT_CL_HIGH_THRESH				75
#define DEFAULT_CL_LOW_THRESH				45

#define CAP_LEARNING_INIT_WORK_DELAY        10000         /* 10 sec */
#define CAP_LEARNING_INIT_RETRY_MAX_COUNT   10
#define CAP_LEARNING_UNINITIALIZED_VAL      -1

/* for depleted capacity decision */
#define DEFAULT_CL_INFO_CC_UPDATE_THRESH	1			/* cc_soc_sw threshold for save[%] while aged level1*/
#define DEFAULT_CL_INFO_BATT_TEMP_THRESH	400			/* batt temp threshold[0.1degc] */
#define DEFAULT_CL_INFO_SOC_THRESH			90			/* soc threshold[%] */
#define DEFAULT_CL_INFO_RTC_UPDATE_THRESH	60			/* since epoch threshold for save[sec] while aged level1 */
/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/
typedef enum
{
	SHBATT_TASK_COMMAND_LOCK,
	SHBATT_MUTEX_TYPE_NUM
}shbatt_kernel_mutex_type;

typedef enum {
	SHBATT_FV_AGED_LEVEL0,
	SHBATT_FV_AGED_LEVEL1,
	SHBATT_FV_AGED_LEVEL2,
	SHBATT_FV_AGED_LEVEL3,
	SHBATT_FV_AGED_LEVEL_NUM
} shbatt_fv_aged_level_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :                                                  |*/
/*+-----------------------------------------------------------------------------+*/
typedef struct {
	int						cc_now;
	bool					is_cc_start;
	int						cc_prev;
	int						cc_accumulated;
	int						cc_stored;
	int						cc_thresh[SHBATT_FV_AGED_LEVEL_NUM];
	int						cc_update_thresh;
	int						batt_temp;
	int						batt_temp_thresh;
	int						soc;
	int						soc_thresh;
	unsigned long			rtc_now;
	bool					is_rtc_start;
	unsigned long			rtc_prev;
	unsigned long			rtc_accumulated;
	unsigned long			rtc_stored;
	unsigned long			rtc_thresh[SHBATT_FV_AGED_LEVEL_NUM];
	unsigned long			rtc_update_thresh;
	shbatt_fv_aged_level_t	fv_aged_level;
	shbatt_fv_aged_level_t	fv_aged_level_cc;
	shbatt_fv_aged_level_t	fv_aged_level_rtc;
	shbatt_fv_aged_level_t	fv_aged_level_max;
	bool					durable_update;
	bool					load_completed;
	struct mutex			cl_info_lock;
	bool					cl_info_enbale;
} cap_learning_info_t;

typedef struct {
    const char       *name;
    struct kobject   *kobj;
    struct kobj_type *ktype;
} shbatt_data;

/*+-----------------------------------------------------------------------------+*/
/*| @ EXTERN VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ STATIC VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

static bool						shbatt_task_is_initialized = false;
static struct wakeup_source		*shbatt_wakeup_source;
static atomic_t					shbatt_wakeup_source_num;

static dev_t					shbatt_dev;
static int						shbatt_major;
static int						shbatt_minor;
static struct cdev				shbatt_cdev;
static struct class*			shbatt_dev_class;
static struct kset             *shbatt_kset;
static struct kobject           shbatt_kobj_child_1;

/* ---------------- */
/* sysfs            */
/* ---------------- */
/* Battery deterioration judgment */
static int shbatt_sysfs_depleted_init = UNINITIALIZED;
static struct attribute depleted_init = { "depleted_init", 0666, };

static int shbatt_sysfs_depleted_ver = UNINITIALIZED;
static struct attribute depleted_ver = { "depleted_ver", 0666, };

static int shbatt_sysfs_depleted_ave = UNINITIALIZED;
static struct attribute depleted_ave = { "depleted_ave", 0666, };

static int shbatt_sysfs_depleted_pos = UNINITIALIZED;
static struct attribute depleted_pos = { "depleted_pos", 0666, };

static int shbatt_sysfs_depleted_result = UNINITIALIZED;
static struct attribute depleted_result = { "depleted_result", 0666, };

/* Charging cycle */
static int shbatt_sysfs_charging_cycle_init = UNINITIALIZED;
static struct attribute charging_cycle_init = { "charging_cycle_init", 0666, };

static int shbatt_sysfs_charging_cycle_cc = UNINITIALIZED;
static struct attribute charging_cycle_cc = { "charging_cycle_cc", 0666, };

static int shbatt_sysfs_charging_cycle_rtc = UNINITIALIZED;
static struct attribute charging_cycle_rtc = { "charging_cycle_rtc", 0666, };


static struct attribute *shbatt_default_attrs[] = {
    &depleted_init,
    &depleted_ver,
    &depleted_ave,
    &depleted_pos,
    &depleted_result,
    &charging_cycle_init,
    &charging_cycle_cc,
    &charging_cycle_rtc,
    NULL,
};

/* Timer */
static spinlock_t				shbatt_pkt_lock;
static struct mutex				shbatt_task_lock;
static struct workqueue_struct*	shbatt_task_workqueue_p;
#ifdef CONFIG_SHARP_SHTERM
static shbatt_packet_t			shbatt_pkt[16];
#endif /* CONFIG_SHARP_SHTERM */

/* wake_lock */
static struct timespec64		shbatt_lock_time[SHBATT_MUTEX_TYPE_NUM];
static struct timespec64		shbatt_unlock_time[SHBATT_MUTEX_TYPE_NUM];
static char						shbatt_lock_func[SHBATT_MUTEX_TYPE_NUM][SHBATT_LOCK_FUNC_LEN];

struct shbatt_chip {
	struct device                *dev;
	struct notifier_block        nb;
	struct notifier_block        drm_nb; //for drm(direct rendering manager)
	struct work_struct           batt_psy_changed_work;
	struct work_struct           usb_psy_changed_work;
	struct work_struct           dc_psy_changed_work;
	struct delayed_work          cap_learning_initialize_work;
	int                          cl_high_thresh;
	int                          cl_low_thresh;
	int                          *thermal_mitigation;
	int                          thermal_levels;
	cap_learning_info_t          cl_info;
	struct kobject               *kobj;
	struct drm_panel             *active_panel;
};

static struct shbatt_chip *the_chip = NULL;
static int shbatt_cur_depleted_val = 100;
module_param_named(cur_depleted_val, shbatt_cur_depleted_val, int, 0644);

static int shbatt_depleted_capacity_pos = 0;
static int shbatt_depleted_capacity_array[5] = {0,};
static char shbatt_depleted_calc_ver = 2;
static int shbatt_avr_depleted_val = 0;
module_param_named(avr_depleted_val, shbatt_avr_depleted_val, int, 0644);

static int restored_dep_ver = CAP_LEARNING_UNINITIALIZED_VAL;
static int restored_dep_ave = CAP_LEARNING_UNINITIALIZED_VAL;
static int restored_dep_pos = CAP_LEARNING_UNINITIALIZED_VAL;
static int restored_dep_res = CAP_LEARNING_UNINITIALIZED_VAL;
static bool isInitialized_dep  = false;

static bool isInitialized_cycle  = false;

/* Debug */
static int shbatt_fake_learned_cc = UNINITIALIZED;
module_param_named(fake_learned_cc, shbatt_fake_learned_cc, int, 0644);

/*+-----------------------------------------------------------------------------+*/
/*| @ EXTERN FUNCTION PROTO TYPE DECLARE :                                      |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION PROTO TYPE DECLARE :                                       |*/
/*+-----------------------------------------------------------------------------+*/
static int shbatt_drv_create_device( void );

static void shbatt_release( struct kobject *kobj );

static ssize_t shbatt_info_show(
    struct kobject *kobj,
    struct attribute *attr,
    char *buf
);

static ssize_t shbatt_info_store(
    struct kobject *kobj,
    struct attribute *attr,
    const char *buf,
    size_t len
);

/* task */
#ifdef CONFIG_SHARP_SHTERM
static void shbatt_task(
	struct work_struct*			work_p );

static void shbatt_task_cmd_invalid(
	shbatt_packet_t*			pkt_p );

static void shbatt_task_cmd_battlog_event(
	shbatt_packet_t*			pkt_p );

static shbatt_result_t shbatt_seq_battlog_event(
	int							evt );

/* Timer */
static shbatt_packet_t* shbatt_task_get_packet( void );

static void shbatt_task_free_packet(
	shbatt_packet_t*			pkt );

/* wake_lock */
static void shbatt_seq_lock_task_mutex(
	shbatt_kernel_mutex_type	type,
	const char*					func );

static void shbatt_seq_unlock_task_mutex(
	shbatt_kernel_mutex_type	type,
	const char*					func );
#endif /* CONFIG_SHARP_SHTERM */

/*+-----------------------------------------------------------------------------+*/
/*| @ PLATFORM DRIVER MODULE PROTO TYPE DECLARE :                               |*/
/*+-----------------------------------------------------------------------------+*/
/* attribute store */

/* driver I/F */
static int shbatt_drv_open(
	struct inode*				in_p,
	struct file*				fi_p );

static int shbatt_drv_release(
	struct inode*				in_p,
	struct file*				fi_p );

static unsigned int shbatt_drv_poll(
	struct file*				fi_p,
	poll_table*					wait_p );

static long shbatt_drv_ioctl(
	struct file*				fi_p,
	unsigned int				cmd,
	unsigned long				arg );

static int shbatt_drv_probe(
	struct platform_device*		dev_p );

static int shbatt_drv_remove(
	struct platform_device*		dev_p );

static void shbatt_drv_shutdown(
	struct platform_device*		dev_p );

static int shbatt_drv_resume(
	struct platform_device*		dev_p);

static int shbatt_drv_suspend(
	struct platform_device*		dev_p,
	pm_message_t				state );

static int shbatt_depleted_backup_params(void);
static int shbatt_depleted_restore_params(void);

static int __init shbatt_drv_module_init( void );
static void __exit shbatt_drv_module_exit( void );

bool is_shbatt_task_initialized( void );

/*+-----------------------------------------------------------------------------+*/
/*| @ FUNCTION TABLE PROTO TYPE DECLARE :                                       |*/
/*+-----------------------------------------------------------------------------+*/
static struct sysfs_ops shbatt_sysfs_ops = {
    .show  = shbatt_info_show,
    .store = shbatt_info_store,
};

static struct kobj_type shbatt_ktype = {
    .release = shbatt_release,
    .sysfs_ops = &shbatt_sysfs_ops,
    .default_attrs = shbatt_default_attrs,
};

static shbatt_data kobj_data = {
    .name  = "notice",
    .kobj  = &shbatt_kobj_child_1,
    .ktype = &shbatt_ktype,
};

static struct file_operations shbatt_fops =
{
	.owner			= THIS_MODULE,
	.open			= shbatt_drv_open,
	.release		= shbatt_drv_release,
	.poll			= shbatt_drv_poll,
	.unlocked_ioctl	= shbatt_drv_ioctl,
	.compat_ioctl	= shbatt_drv_ioctl,
};

#ifdef CONFIG_OF
static struct of_device_id shbatt_match_table[] = {
	{
		.compatible = "sharp,shbatt",
	},
	{}
};
#else  /* CONFIG_OF */
#define shbatt_match_table NULL;
#endif /* CONFIG_OF */

static struct platform_driver shbatt_platform_driver = {
	.probe		= shbatt_drv_probe,
	.remove		= shbatt_drv_remove,
	.shutdown	= shbatt_drv_shutdown,
	.driver		= {
		.name	= SHBATT_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = shbatt_match_table,
	},
	.resume		= shbatt_drv_resume,
	.suspend	= shbatt_drv_suspend,
};

/* Timer */
#ifdef CONFIG_SHARP_SHTERM
static void (*const shbatt_task_cmd_func[])( shbatt_packet_t* pkt_p ) =
{
	shbatt_task_cmd_invalid,									/* SHBATT_TASK_CMD_INVALID */
	shbatt_task_cmd_battlog_event,								/* SHBATT_TASK_CMD_BATTLOG_EVENT */
};
#endif /* CONFIG_SHARP_SHTERM */

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC FUNCTION'S CODE AREA :                                             |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION'S CODE AREA :                                              |*/
/*+-----------------------------------------------------------------------------+*/
static shbatt_result_t shbatt_send_uevent( shbatt_uevent_data d ) {
    shbatt_result_t result = SHBATT_RESULT_SUCCESS;
    int i, ret;
    char *envp[5];

    SHBATT_TRACE("[S] %s : uevent = %d\n", __FUNCTION__, d.cmd);

    if ( !kobj_data.kobj ) {
        SHBATT_ERROR("[E] %s kernel_kobj is not generated.\n", __FUNCTION__);
        return SHBATT_RESULT_FAIL;
    }

    envp[0] = kasprintf(GFP_KERNEL, "EVENT=%d", d.cmd);
    envp[1] = kasprintf(GFP_KERNEL, "DATA_0=%d", d.data_0);
    envp[2] = kasprintf(GFP_KERNEL, "DATA_1=%d", d.data_1);
    envp[3] = kasprintf(GFP_KERNEL, "DATA_2=%d", d.data_2);
    envp[4] = NULL;

    ret = kobject_uevent_env( kobj_data.kobj, KOBJ_CHANGE, envp );
    if ( ret ) {
        SHBATT_ERROR("[E] %s : Failed to send uevent. ret=%d\n", __FUNCTION__, ret);
        result = SHBATT_RESULT_FAIL;
    }

    for (i = 0; i < 1; ++i)
        kfree(envp[i]);

    SHBATT_TRACE("[E] %s \n", __FUNCTION__);

    return result;
}

static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

static int shbatt_drv_create_device( void )
{
	struct device*				dev_p;
	int							ret;

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	ret = alloc_chrdev_region(&shbatt_dev,0,1,SHBATT_DEV_NAME);

	if(ret < 0)
	{
		SHBATT_ERROR("%s : alloc_chrdev_region failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_0;
	}

	shbatt_major = MAJOR(shbatt_dev);
	shbatt_minor = MINOR(shbatt_dev);

	cdev_init(&shbatt_cdev,&shbatt_fops);

	shbatt_cdev.owner = THIS_MODULE;

	ret = cdev_add(&shbatt_cdev,shbatt_dev,1);

	if(ret < 0)
	{
		SHBATT_ERROR("%s : cdev_add failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_1;
	}

	shbatt_dev_class = class_create(THIS_MODULE,SHBATT_DEV_NAME);

	if(IS_ERR(shbatt_dev_class))
	{
		ret = PTR_ERR(shbatt_dev_class);
		SHBATT_ERROR("%s : class_create failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_2;
	}

	dev_p = device_create(shbatt_dev_class,NULL,shbatt_dev,&shbatt_cdev,SHBATT_DEV_NAME);

	if(IS_ERR(dev_p))
	{
		ret = PTR_ERR(dev_p);
		SHBATT_ERROR("%s : device_create failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_3;
	}


	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return ret;

create_device_exit_3:
	class_destroy(shbatt_dev_class);

create_device_exit_2:
	cdev_del(&shbatt_cdev);

create_device_exit_1:
	unregister_chrdev_region(shbatt_dev,1);

create_device_exit_0:

	return ret;
}


#define BATTLOG_EVENT_BUF_LEN    64
static int get_battlog_event_name(
	shbattlog_event_num event,
	char *buf
){
	switch (event) {
	case SHBATTLOG_EVENT_BATT_FV_AGED_LEVEL1:
		return sprintf(buf, "%s", "BATT_FV_AGED_LEVEL1");
	case SHBATTLOG_EVENT_BATT_FV_AGED_LEVEL2:
		return sprintf(buf, "%s", "BATT_FV_AGED_LEVEL2");
	case SHBATTLOG_EVENT_BATT_FV_AGED_LEVEL3:
		return sprintf(buf, "%s", "BATT_FV_AGED_LEVEL3");
	case SHBATTLOG_EVENT_CHG_COLD_FAST_ST:
		return sprintf(buf, "%s", "CHG_COLD_FAST_ST");
	case SHBATTLOG_EVENT_CHG_COLD_STOP_ST:
		return sprintf(buf, "%s", "CHG_COLD_STOP_ST");
	case SHBATTLOG_EVENT_CHG_COMP:
		return sprintf(buf, "%s", "CHG_COMP");
	case SHBATTLOG_EVENT_CHG_COUNT_OVER_STOP_ST:
		return sprintf(buf, "%s", "CHG_COUNT_OVER_STOP_ST");
	case SHBATTLOG_EVENT_CHG_END:
		return sprintf(buf, "%s", "CHG_END");
	case SHBATTLOG_EVENT_CHG_ERR_BD_BAT_UNUSUAL_ST:
		return sprintf(buf, "%s", "CHG_ERR_BD_BAT_UNUSUAL_ST");
	case SHBATTLOG_EVENT_CHG_ERR_BD_CHG_UNUSUAL_ST:
		return sprintf(buf, "%s", "CHG_ERR_BD_CHG_UNUSUAL_ST");
	case SHBATTLOG_EVENT_CHG_ERR_CHG_POWER_SHORTAGE_ST:
		return sprintf(buf, "%s", "CHG_ERR_CHG_POWER_SHORTAGE_ST");
	case SHBATTLOG_EVENT_CHG_ERROR:
		return sprintf(buf, "%s", "CHG_ERROR");
	case SHBATTLOG_EVENT_CHG_FAST_ST:
		return sprintf(buf, "%s", "CHG_FAST_ST");
	case SHBATTLOG_EVENT_CHG_HOT_FAST_ST:
		return sprintf(buf, "%s", "CHG_HOT_FAST_ST");
	case SHBATTLOG_EVENT_CHG_HOT_STOP_ST:
		return sprintf(buf, "%s", "CHG_HOT_STOP_ST");
	case SHBATTLOG_EVENT_CHG_INPUT_SUSPEND_OFF:
		return sprintf(buf, "%s", "CHG_INPUT_SUSPEND_OFF");
	case SHBATTLOG_EVENT_CHG_INPUT_SUSPEND_ON:
		return sprintf(buf, "%s", "CHG_INPUT_SUSPEND_ON");
	case SHBATTLOG_EVENT_CHG_INSERT_USB:
		return sprintf(buf, "%s", "CHG_INSERT_USB");
	case SHBATTLOG_EVENT_CHG_PUT_CRADLE:
		return sprintf(buf, "%s", "CHG_PUT_CRADLE");
	case SHBATTLOG_EVENT_CHG_PUT_WIRELESS:
		return sprintf(buf, "%s", "CHG_PUT_WIRELESS");
	case SHBATTLOG_EVENT_CHG_REMOVE_CRADLE:
		return sprintf(buf, "%s", "CHG_REMOVE_CRADLE");
	case SHBATTLOG_EVENT_CHG_REMOVE_USB:
		return sprintf(buf, "%s", "CHG_REMOVE_USB");
	case SHBATTLOG_EVENT_CHG_REMOVE_WIRELESS:
		return sprintf(buf, "%s", "CHG_REMOVE_WIRELESS");
	case SHBATTLOG_EVENT_CHG_START:
		return sprintf(buf, "%s", "CHG_START");
	case SHBATTLOG_EVENT_CHG_FEED:
		return sprintf(buf, "%s", "CHG_FEED");
	case SHBATTLOG_EVENT_CHG_UNFEED:
		return sprintf(buf, "%s", "CHG_UNFEED");
	case SHBATTLOG_EVENT_CHG_FEED_DCHG:
		return sprintf(buf, "%s", "CHG_FEED_DCHG");
	case SHBATTLOG_EVENT_CHG_UNFEED_DCHG:
		return sprintf(buf, "%s", "CHG_UNFEED_DCHG");
	case SHBATTLOG_EVENT_CHG_TYPE_CDP:
		return sprintf(buf, "%s", "CHG_TYPE_CDP");
	case SHBATTLOG_EVENT_CHG_TYPE_DCP:
		return sprintf(buf, "%s", "CHG_TYPE_DCP");
	case SHBATTLOG_EVENT_CHG_TYPE_HVDCP:
		return sprintf(buf, "%s", "CHG_TYPE_HVDCP");
	case SHBATTLOG_EVENT_CHG_TYPE_OTHER:
		return sprintf(buf, "%s", "CHG_TYPE_OTHER");
	case SHBATTLOG_EVENT_CHG_TYPE_PD:
		return sprintf(buf, "%s", "CHG_TYPE_PD");
	case SHBATTLOG_EVENT_CHG_TYPE_SDP:
		return sprintf(buf, "%s", "CHG_TYPE_SDP");
	case SHBATTLOG_EVENT_CHG_TYPE_C:
		return sprintf(buf, "%s", "CHG_TYPE_C");
	case SHBATTLOG_EVENT_CHG_TYPE_FLOAT:
		return sprintf(buf, "%s", "CHG_TYPE_FLOAT");
	case SHBATTLOG_EVENT_CHG_TYPE_PD_PPS:
		return sprintf(buf, "%s", "CHG_TYPE_PD_PPS");
	case SHBATTLOG_EVENT_DETECT_USB_HIGH_TEMP:
		return sprintf(buf, "%s", "DETECT_USB_HIGH_TEMP");
	case SHBATTLOG_EVENT_FGIC_EX0:
		return sprintf(buf, "%s", "FGIC_EX0");
	case SHBATTLOG_EVENT_FGIC_EX10:
		return sprintf(buf, "%s", "FGIC_EX10");
	case SHBATTLOG_EVENT_FGIC_EX100:
		return sprintf(buf, "%s", "FGIC_EX100");
	case SHBATTLOG_EVENT_FGIC_EX20:
		return sprintf(buf, "%s", "FGIC_EX20");
	case SHBATTLOG_EVENT_FGIC_EX30:
		return sprintf(buf, "%s", "FGIC_EX30");
	case SHBATTLOG_EVENT_FGIC_EX40:
		return sprintf(buf, "%s", "FGIC_EX40");
	case SHBATTLOG_EVENT_FGIC_EX50:
		return sprintf(buf, "%s", "FGIC_EX50");
	case SHBATTLOG_EVENT_FGIC_EX60:
		return sprintf(buf, "%s", "FGIC_EX60");
	case SHBATTLOG_EVENT_FGIC_EX70:
		return sprintf(buf, "%s", "FGIC_EX70");
	case SHBATTLOG_EVENT_FGIC_EX80:
		return sprintf(buf, "%s", "FGIC_EX80");
	case SHBATTLOG_EVENT_FGIC_EX90:
		return sprintf(buf, "%s", "FGIC_EX90");
	case SHBATTLOG_EVENT_NONE:
		return sprintf(buf, "%s", "NONE");
	case SHBATTLOG_EVENT_QI_CHARGING:
		return sprintf(buf, "%s", "QI_CHARGING");
	case SHBATTLOG_EVENT_QI_ERROR:
		return sprintf(buf, "%s", "QI_ERROR");
	case SHBATTLOG_EVENT_QI_FULL:
		return sprintf(buf, "%s", "QI_FULL");
	case SHBATTLOG_EVENT_QI_GUIDING:
		return sprintf(buf, "%s", "QI_GUIDING");
	case SHBATTLOG_EVENT_QI_IDLE:
		return sprintf(buf, "%s", "QI_IDLE");
	case SHBATTLOG_EVENT_QI_INHIBIT:
		return sprintf(buf, "%s", "QI_INHIBIT");
	case SHBATTLOG_EVENT_QI_SUSPEND:
		return sprintf(buf, "%s", "QI_SUSPEND");
	case SHBATTLOG_EVENT_RELEASE_USB_HIGH_TEMP:
		return sprintf(buf, "%s", "RELEASE_USB_HIGH_TEMP");
	case SHBATTLOG_EVENT_TYPEC_MODE_NON_COMPLIANT:
		return sprintf(buf, "%s", "TYPEC_MODE_NON_COMPLIANT");
	case SHBATTLOG_EVENT_TYPEC_MODE_NONE:
		return sprintf(buf, "%s", "TYPEC_MODE_NONE");
	case SHBATTLOG_EVENT_TYPEC_MODE_POWERED_CABLE_ONLY:
		return sprintf(buf, "%s", "TYPEC_MODE_POWERED_CABLE_ONLY");
	case SHBATTLOG_EVENT_TYPEC_MODE_SINK:
		return sprintf(buf, "%s", "TYPEC_MODE_SINK");
	case SHBATTLOG_EVENT_TYPEC_MODE_SINK_AUDIO_ADAPTER:
		return sprintf(buf, "%s", "TYPEC_MODE_SINK_AUDIO_ADAPTER");
	case SHBATTLOG_EVENT_TYPEC_MODE_SINK_DEBUG_ACCESSORY:
		return sprintf(buf, "%s", "TYPEC_MODE_SINK_DEBUG_ACCESSORY");
	case SHBATTLOG_EVENT_TYPEC_MODE_SINK_POWERED_CABLE:
		return sprintf(buf, "%s", "TYPEC_MODE_SINK_POWERED_CABLE");
	case SHBATTLOG_EVENT_TYPEC_MODE_SOURCE_DEFAULT:
		return sprintf(buf, "%s", "TYPEC_MODE_SOURCE_DEFAULT");
	case SHBATTLOG_EVENT_TYPEC_MODE_SOURCE_HIGH:
		return sprintf(buf, "%s", "TYPEC_MODE_SOURCE_HIGH");
	case SHBATTLOG_EVENT_TYPEC_MODE_SOURCE_MEDIUM:
		return sprintf(buf, "%s", "TYPEC_MODE_SOURCE_MEDIUM");
	default:
		return sprintf(buf, "%s(event No.%d)", "UNKNOWN", event);
	}
}

static void shbatt_task(
	struct work_struct*			work_p
){
	shbatt_packet_t*			pkt_p;

	shbatt_seq_lock_task_mutex( SHBATT_TASK_COMMAND_LOCK,
								__FUNCTION__ );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	pkt_p = (shbatt_packet_t*)work_p;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"shbatt_task %d \n",pkt_p->hdr.cmd );

	if(pkt_p->hdr.cmd < NUM_SHBATT_TASK_CMD)
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
					"shbatt_task OK \n");
		shbatt_task_cmd_func[pkt_p->hdr.cmd](pkt_p);
	}
	else
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
					"shbatt_task NG \n");
		shbatt_task_cmd_invalid(pkt_p);
	}

	SHBATT_WAKE_CTL(0);

	shbatt_task_free_packet(pkt_p);

	shbatt_seq_unlock_task_mutex( SHBATT_TASK_COMMAND_LOCK,
								__FUNCTION__ );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static void shbatt_task_cmd_invalid(
	shbatt_packet_t*			pkt_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static void shbatt_task_cmd_battlog_event(
	shbatt_packet_t*			pkt_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	shbatt_seq_battlog_event(pkt_p->prm.evt);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static shbatt_packet_t* shbatt_task_get_packet( void )
{
	int							idx;
	unsigned long				flags;
	shbatt_packet_t*			ret = NULL;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	spin_lock_irqsave( &shbatt_pkt_lock, flags );

	for( idx = 0; idx < 16; idx++ )
	{
		if( shbatt_pkt[idx].is_used == false )
		{
			shbatt_pkt[idx].is_used = true;

			ret = &shbatt_pkt[idx];

			break;
		}
	}

	spin_unlock_irqrestore( &shbatt_pkt_lock, flags );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );

	return ret;
}

static void shbatt_task_free_packet(
	shbatt_packet_t*			pkt
){
	unsigned long				flags;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	spin_lock_irqsave( &shbatt_pkt_lock, flags );

	pkt->is_used = false;

	spin_unlock_irqrestore( &shbatt_pkt_lock, flags );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );

	return;
}

/* wake_lock */
static void shbatt_seq_lock_task_mutex(
	shbatt_kernel_mutex_type	type,
	const char*					func
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	strncpy( &shbatt_lock_func[type][0], func, SHBATT_LOCK_FUNC_LEN - 1 );
	shbatt_lock_time[type] = ktime_to_timespec64(ktime_get_boottime());

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG, SHPWR_LOG_TYPE_BATT,
				"[P] %s() lock start\n", &shbatt_lock_func[type][0] );

	mutex_lock(&shbatt_task_lock);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);
}

static void shbatt_seq_unlock_task_mutex(
	shbatt_kernel_mutex_type	type,
	const char*					func
){
	struct timespec64			diff;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	mutex_unlock(&shbatt_task_lock);
	shbatt_unlock_time[type] = ktime_to_timespec64(ktime_get_boottime());

	memset(&diff, 0x00, sizeof( diff ) );
	diff = timespec64_sub( shbatt_unlock_time[type], shbatt_lock_time[type] );

	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE, SHPWR_LOG_TYPE_BATT,
				"[P] %s() locktime:%lu.%09lu\n", &shbatt_lock_func[type][0], diff.tv_sec, diff.tv_nsec );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);
}

shbatt_result_t shbatt_api_battlog_event(
	shbattlog_event_num			evt
){
	shbatt_packet_t*			pkt_p;
	char *event_name_buf;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	event_name_buf = kmalloc(BATTLOG_EVENT_BUF_LEN, GFP_KERNEL);
	if (event_name_buf) {
		get_battlog_event_name(evt, event_name_buf);
		SHPWR_LOG_INFO("%s\n", event_name_buf);
		kfree(event_name_buf);
	}

	if(shbatt_task_is_initialized == false)
	{
		return SHBATT_RESULT_REJECTED;
	}

	pkt_p = shbatt_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHBATT_RESULT_REJECTED;
	}

	SHBATT_WAKE_CTL(1);

	pkt_p->hdr.cmd		= SHBATT_TASK_CMD_BATTLOG_EVENT;
	pkt_p->hdr.cmp_p	= NULL;
	pkt_p->hdr.ret_p	= NULL;
	pkt_p->prm.evt		= evt;

	INIT_WORK((struct work_struct*)pkt_p,shbatt_task);

	queue_work(shbatt_task_workqueue_p,(struct work_struct*)pkt_p);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_charge_status(
	int			status
){
#ifdef CONFIG_SHARP_SHTERM
	int charge_status_event = SHBATTLOG_EVENT_NONE;
	static int pre_charge_status_event = SHBATTLOG_EVENT_NONE;

	switch (status) {
	case POWER_SUPPLY_STATUS_CHARGING:
		charge_status_event = SHBATTLOG_EVENT_CHG_START;
		break;
	case POWER_SUPPLY_STATUS_DISCHARGING:
		charge_status_event = SHBATTLOG_EVENT_CHG_END;
		break;
	case POWER_SUPPLY_STATUS_FULL:
		charge_status_event = SHBATTLOG_EVENT_CHG_COMP;
		break;
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		charge_status_event = SHBATTLOG_EVENT_CHG_ERROR;
		break;
	default:
		charge_status_event = SHBATTLOG_EVENT_NONE;
	}

	if(charge_status_event != SHBATTLOG_EVENT_NONE && charge_status_event != pre_charge_status_event) {
		SHPWR_LOG_INFO("charge_status_event = %d -> %d\n", pre_charge_status_event, charge_status_event);

		shbatt_api_battlog_event(charge_status_event);
		pre_charge_status_event = charge_status_event;
	}
#endif /* CONFIG_SHARP_SHTERM */

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_charge_error(
	int			charge_error_status
){
#ifdef CONFIG_SHARP_SHTERM
	static int pre_charge_error_event = SHBATTLOG_EVENT_NONE;
	int charge_error_event = SHBATTLOG_EVENT_NONE;

	if (charge_error_status == POWER_SUPPLY_HEALTH_OVERVOLTAGE)
		charge_error_event = SHBATTLOG_EVENT_CHG_ERR_BD_CHG_UNUSUAL_ST;
	else if (charge_error_status == POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE)
		charge_error_event = SHBATTLOG_EVENT_CHG_ERR_BD_BAT_UNUSUAL_ST;

	if(charge_error_event != SHBATTLOG_EVENT_NONE && charge_error_event != pre_charge_error_event) {
		SHPWR_LOG_INFO("charge_error_event = %d -> %d\n", pre_charge_error_event, charge_error_event);

		shbatt_api_battlog_event(charge_error_event);
		pre_charge_error_event = charge_error_event;
	}
#endif /* CONFIG_SHARP_SHTERM */

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_jeita_status(
	int			jeita_cur_status
){
#ifdef CONFIG_SHARP_SHTERM
	static int jeita_pre_status = POWER_SUPPLY_HEALTH_UNKNOWN;

	if (jeita_cur_status  != jeita_pre_status) {
		SHPWR_LOG_INFO("jeita_pre_status = %d,jeita_cur_status  = %d\n", jeita_pre_status , jeita_cur_status );

		switch (jeita_cur_status){
		case POWER_SUPPLY_HEALTH_OVERHEAT:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_HOT_STOP_ST);
			break;
		case POWER_SUPPLY_HEALTH_COLD:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_COLD_STOP_ST);
			break;
		case POWER_SUPPLY_HEALTH_WARM:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_HOT_FAST_ST);
			break;
		case POWER_SUPPLY_HEALTH_COOL:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_COLD_FAST_ST);
			break;
		case POWER_SUPPLY_HEALTH_GOOD:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_FAST_ST);
			break;
		default:
			break;
		}
	}
	jeita_pre_status = jeita_cur_status;
#endif /* CONFIG_SHARP_SHTERM */

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_capacity(
	int			cur_capacity
){
#ifdef CONFIG_SHARP_SHTERM
	static int pre_capacity = -1;
	static const shbattlog_event_num event_tbl[11] =
	{
		SHBATTLOG_EVENT_FGIC_EX0,
		SHBATTLOG_EVENT_FGIC_EX10,
		SHBATTLOG_EVENT_FGIC_EX20,
		SHBATTLOG_EVENT_FGIC_EX30,
		SHBATTLOG_EVENT_FGIC_EX40,
		SHBATTLOG_EVENT_FGIC_EX50,
		SHBATTLOG_EVENT_FGIC_EX60,
		SHBATTLOG_EVENT_FGIC_EX70,
		SHBATTLOG_EVENT_FGIC_EX80,
		SHBATTLOG_EVENT_FGIC_EX90,
		SHBATTLOG_EVENT_FGIC_EX100
	};

	if( cur_capacity < 0 || cur_capacity > 100 ){
		return SHBATT_RESULT_REJECTED;
	}

	if( pre_capacity != cur_capacity ){
		SHBATT_TRACE("[P] %s capacity = %d -> %d\n",__FUNCTION__, pre_capacity, cur_capacity);

		if(cur_capacity % 10 == 0)
		{
			shbatt_api_battlog_event(event_tbl[cur_capacity/10]);
		}
		pre_capacity = cur_capacity;
	}
#endif /* CONFIG_SHARP_SHTERM */

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_usb_type(
    int usb_type
){
#ifdef CONFIG_SHARP_SHTERM
    static int pre_event = SHBATTLOG_EVENT_NONE;
    int event = SHBATTLOG_EVENT_NONE;

    switch (usb_type) {
    case POWER_SUPPLY_USB_TYPE_SDP:
        event = SHBATTLOG_EVENT_CHG_TYPE_SDP;
        break;
    case POWER_SUPPLY_USB_TYPE_DCP:
        event = SHBATTLOG_EVENT_CHG_TYPE_DCP;
        break;
    case POWER_SUPPLY_USB_TYPE_CDP:
        event = SHBATTLOG_EVENT_CHG_TYPE_CDP;
        break;
    case POWER_SUPPLY_USB_TYPE_C:
        event = SHBATTLOG_EVENT_CHG_TYPE_C;
        break;
    case POWER_SUPPLY_USB_TYPE_PD:
    case POWER_SUPPLY_USB_TYPE_PD_DRP:
        event = SHBATTLOG_EVENT_CHG_TYPE_PD;
        break;
    case POWER_SUPPLY_USB_TYPE_PD_PPS:
        event = SHBATTLOG_EVENT_CHG_TYPE_PD_PPS;
        break;
    case POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID:
        event = SHBATTLOG_EVENT_CHG_TYPE_FLOAT;
        break;
    case POWER_SUPPLY_USB_TYPE_ACA:
    case POWER_SUPPLY_USB_TYPE_UNKNOWN:
        event = SHBATTLOG_EVENT_CHG_TYPE_OTHER;
        break;
    default:
        SHBATT_ERROR("%s : unexpected usb-type was specified. usb_type = %d\n",__FUNCTION__, usb_type);
        break;
    }

    // Notify if the event is different from the previous one.
    if (event != pre_event) {
        SHPWR_LOG_INFO("change usb_type = %d, event = %d -> %d\n", usb_type, pre_event, event);
        shbatt_api_battlog_event(event);
        pre_event = event;
    }
#endif /* CONFIG_SHARP_SHTERM */

    return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_partner_type(
    int partner_type
){
#ifdef CONFIG_SHARP_SHTERM
    static int pre_event = UNINITIALIZED;
           int     event = UNINITIALIZED;

    if ( (partner_type < SSDEV_PARTNER_NONE) ||
         (partner_type > SSDEV_PARTNER_INVALID) ) {
        SHBATT_ERROR("Unsupported partner_type = %d\n",__FUNCTION__, partner_type);
        return SHBATT_RESULT_FAIL;
    }

    switch (partner_type) {
    case SSDEV_PARTNER_NONE:
        event = SHBATTLOG_EVENT_TYPEC_MODE_NONE;
        break;
    case SSDEV_PARTNER_SNK_TYPEC_DEFAULT:
        event = SHBATTLOG_EVENT_TYPEC_MODE_SOURCE_DEFAULT;
        break;
    case SSDEV_PARTNER_SNK_TYPEC_RP_MEDIUM_1P5A:
        event = SHBATTLOG_EVENT_TYPEC_MODE_SOURCE_MEDIUM;
        break;
    case SSDEV_PARTNER_SNK_TYPEC_RP_HIGH_3A:
        event = SHBATTLOG_EVENT_TYPEC_MODE_SOURCE_HIGH;
        break;
    case SSDEV_PARTNER_SNK_DEBUG_ACCESS:
    case SSDEV_PARTNER_SRC_TYPEC_UNORIENTED_DEBUG_ACCESS:
        event = SHBATTLOG_EVENT_TYPEC_MODE_SINK_DEBUG_ACCESSORY;
        break;
    case SSDEV_PARTNER_SRC_TYPEC_POWERCABLE:
        event = SHBATTLOG_EVENT_TYPEC_MODE_SINK_POWERED_CABLE;
        break;
    case SSDEV_PARTNER_SRC_TYPEC_AUDIO_ACCESS:
        event = SHBATTLOG_EVENT_TYPEC_MODE_SINK_AUDIO_ADAPTER;
        break;
    case SSDEV_PARTNER_UNKNOWN:
        event = SHBATTLOG_EVENT_TYPEC_MODE_NON_COMPLIANT;
        break;
    /* Event is not supported  */
    case SSDEV_PARTNER_SNK_USB_SDP:
    case SSDEV_PARTNER_SNK_USB_OCP:
    case SSDEV_PARTNER_SNK_USB_CDP:
    case SSDEV_PARTNER_SNK_USB_DCP:
    case SSDEV_PARTNER_SNK_USB_FLOAT:
    case SSDEV_PARTNER_SNK_USB_QC_2P0:
    case SSDEV_PARTNER_SNK_USB_QC_3P0:
    case SSDEV_PARTNER_SNK_USB_QC_3P5:
    case SSDEV_PARTNER_SNK_PD:
    case SSDEV_PARTNER_SNK_PPS:
    case SSDEV_PARTNER_WLS_SRC_BPP:
    case SSDEV_PARTNER_WLS_SNK_BPP:
    case SSDEV_PARTNER_WLS_SNK_EPP:
    case SSDEV_PARTNER_WLS_SNK_PDDE:
    case SSDEV_PARTNER_INVALID:
    default:
        SHPWR_LOG_INFO("Skip event notifications. partner_type = %d\n", __FUNCTION__, partner_type);
        pre_event = event;
        return SHBATT_RESULT_SUCCESS;
    }

    // Notify if the event is different from the previous one.
    if ( event != pre_event ) {
        SHPWR_LOG_INFO("change partner_type = %d, event = %d -> %d\n", partner_type, pre_event, event);
        shbatt_api_battlog_event(event);
        pre_event = event;
    }
#endif /* CONFIG_SHARP_SHTERM */

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_usb_present(
	int			usb_present
){
#ifdef CONFIG_SHARP_SHTERM
	static int pre_usb_present = 0;

	// Notify if the event is different from the previous one.
	if (pre_usb_present != usb_present) {
		SHPWR_LOG_INFO("change usb_present : %d -> %d\n",
						pre_usb_present, usb_present);

		if (usb_present)
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_INSERT_USB);
		else
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_REMOVE_USB);

		pre_usb_present = usb_present;
	}
#endif /* CONFIG_SHARP_SHTERM */

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_dc_present(
	int			dc_present
){
#ifdef CONFIG_SHARP_SHTERM
	static int pre_dc_present = 0;

	// Notify if the event is different from the previous one.
	if (pre_dc_present != dc_present) {
		SHPWR_LOG_INFO("change dc_present : %d -> %d\n",
						pre_dc_present, dc_present);

//		if (dc_present)
//			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_PUT_CRADLE);
//		else
//			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_REMOVE_CRADLE);

		pre_dc_present = dc_present;
	}
#endif /* CONFIG_SHARP_SHTERM */

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_input_suspend(
	int			input_suspend
){
#ifdef CONFIG_SHARP_SHTERM
	static int pre_input_suspend = 0;

	// Notify if the event is different from the previous one.
	if (pre_input_suspend != input_suspend) {
		SHPWR_LOG_INFO("change input_suspend : %d -> %d\n",
						pre_input_suspend, input_suspend);

		if (input_suspend)
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_INPUT_SUSPEND_ON);
		else
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_INPUT_SUSPEND_OFF);

		pre_input_suspend = input_suspend;
	}
#endif /* CONFIG_SHARP_SHTERM */

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_system_temp_level(
    int level
){
#ifdef CONFIG_SHARP_SHTERM
    static int pre_level = 0;
    int level_feed = the_chip->thermal_levels;
    int level_stop = level_feed + 1;

    // Notify if the event is different from the previous one.
    if ( level == pre_level )
        return SHBATT_RESULT_SUCCESS;

    if ( level == level_feed ) {
        shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_FEED);
    } else if ( (pre_level == level_feed) && (level < level_feed) ) {
        shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_UNFEED);
    } else if ( level == level_stop ) {
        shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_COUNT_OVER_STOP_ST);
    }

    SHPWR_LOG_INFO("thermal_level = %d / %d\n", level, level_stop);
    pre_level = level;
#endif /* CONFIG_SHARP_SHTERM */

    return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_direct_charge_mode(
    int status
){
#ifdef CONFIG_SHARP_SHTERM
    static int pre_status = 0;

    // Notify if the event is different from the previous one.
    if ( status == pre_status )
        return SHBATT_RESULT_SUCCESS;

    if ( status == 1 ) {
        shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_FEED_DCHG);
    } else if ( status == 0 ) {
        shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_UNFEED_DCHG);
    }

    SHPWR_LOG_INFO("direct_charge_mode = %d -> %d\n", pre_status, status);
    pre_status = status;
#endif /* CONFIG_SHARP_SHTERM */

    return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_shortage_st(
	int			storming_status
){
#ifdef CONFIG_SHARP_SHTERM
	static int pre_storming_status = 0;

	// Notify if the event is different from the previous one.
	if (pre_storming_status != storming_status) {
		SHPWR_LOG_INFO("change storming status : %d -> %d\n",
						pre_storming_status, storming_status);

		if (storming_status)
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_ERR_CHG_POWER_SHORTAGE_ST);

		pre_storming_status = storming_status;
	}
#endif /* CONFIG_SHARP_SHTERM */

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_typec_overheat_st(
    int lev
){
#ifdef CONFIG_SHARP_SHTERM
    static enum power_supply_cc_safety_level pre_lev = POWER_SUPPLY_CC_SAFETY_LEVEL0;

    // Notify if the event is different from the previous one.
    if (pre_lev == lev)
        return SHBATT_RESULT_SUCCESS;

    switch (lev) {
    case POWER_SUPPLY_CC_SAFETY_LEVEL0:
        shbatt_api_battlog_event(SHBATTLOG_EVENT_RELEASE_USB_HIGH_TEMP);
        break;
    case POWER_SUPPLY_CC_SAFETY_LEVEL3:
        shbatt_api_battlog_event(SHBATTLOG_EVENT_DETECT_USB_HIGH_TEMP);
        break;
    default:
        return SHBATT_RESULT_SUCCESS;
    }

    SHPWR_LOG_INFO("change typec overheat status : %d -> %d\n", pre_lev, lev);
    pre_lev = lev;
#endif /* CONFIG_SHARP_SHTERM */

    return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_fv_aged_level(
	int			aged_level
){
#ifdef CONFIG_SHARP_SHTERM
	static int pre_aged_level = SHBATT_FV_AGED_LEVEL0;
	int aged_level_event = -1;

	switch (aged_level) {
	case SHBATT_FV_AGED_LEVEL1:
		aged_level_event = SHBATTLOG_EVENT_BATT_FV_AGED_LEVEL1;
		break;
	case SHBATT_FV_AGED_LEVEL2:
		aged_level_event = SHBATTLOG_EVENT_BATT_FV_AGED_LEVEL2;
		break;
	case SHBATT_FV_AGED_LEVEL3:
		aged_level_event = SHBATTLOG_EVENT_BATT_FV_AGED_LEVEL3;
		break;
	default:
		aged_level_event = -1;
	}

	// Notify if the event is different from the previous one.
	if ((aged_level_event != -1) && (aged_level != pre_aged_level)) {
		SHPWR_LOG_INFO("change fv_aged_level = %d -> %d\n", pre_aged_level, aged_level);
		shbatt_api_battlog_event(aged_level_event);

		pre_aged_level = aged_level;
	}
#endif /* CONFIG_SHARP_SHTERM */

	return SHBATT_RESULT_SUCCESS;
}

static int shbatt_depleted_backup_params(void)
{
	int count = 0;
	int average_depleted_val = 0;
	shbatt_result_t result = SHBATT_RESULT_SUCCESS;
    shbatt_uevent_data event_data;

	SHBATT_TRACE("[S] %s\n", __FUNCTION__);

	for(count = 0; count < shbatt_depleted_capacity_pos; count++)
	{
		average_depleted_val += shbatt_depleted_capacity_array[count];
	}
	average_depleted_val = average_depleted_val / shbatt_depleted_capacity_pos;
	shbatt_avr_depleted_val = average_depleted_val;

    event_data.cmd  = SHBATT_REQ_CMD_CAP_LEARNING_WRITE_CALCULATED_DATA;
    event_data.data_0 = shbatt_depleted_calc_ver;
    event_data.data_1 = average_depleted_val;
    event_data.data_2 = shbatt_depleted_capacity_pos;
    result = shbatt_send_uevent( event_data );

	SHBATT_TRACE("[E] %s : result=%d\n", __FUNCTION__, result);

	return result;
}

static int shbatt_depleted_restore_params(void)
{
    static bool executed = false;
    shbatt_result_t result = SHBATT_RESULT_SUCCESS;
    int count = 0;

    SHBATT_TRACE("[S] %s\n", __FUNCTION__);

    if (executed)
        goto out;

    if ( !isInitialized_dep ) {
        SHBATT_ERROR("%s : Initialization is not finished.\n", __FUNCTION__);
        result = SHBATT_RESULT_FAIL;
        goto out;
    }

    for(count = 0; count < restored_dep_pos; count++)
        shbatt_depleted_capacity_array[count] = restored_dep_ave;
    shbatt_depleted_calc_ver        = restored_dep_ver;
    shbatt_depleted_capacity_pos    = restored_dep_pos;

    executed = true;

    SHBATT_TRACE("[P] %s : completed [%d, %d, %d, %d]\n", __FUNCTION__,
                                shbatt_depleted_calc_ver, restored_dep_ave,
                                shbatt_depleted_capacity_pos, restored_dep_res );

out:
    SHBATT_TRACE("[E] %s\n", __FUNCTION__);

    return result;
}

shbatt_result_t shbatt_api_store_fg_cap_learning_result(
	int64_t learned_cc_uah,
	int nom_uah,
	int high_thresh,
	int low_thresh
) {
	int dep_per;
	int dep_result;
	static int prev_dep_result = CAP_LEARNING_UNINITIALIZED_VAL;
	int res = -EINVAL;
	shbatt_result_t result = SHBATT_RESULT_SUCCESS;
	int count;
    shbatt_uevent_data event_data;
    char dep_result_char[2];

	SHBATT_TRACE("[S] %s\n", __FUNCTION__);

	res = shbatt_depleted_restore_params();
	if ( (res == SHBATT_RESULT_SUCCESS)
			&& (prev_dep_result == CAP_LEARNING_UNINITIALIZED_VAL) ) {
		prev_dep_result = restored_dep_res;
	}

	dep_per = div64_s64(learned_cc_uah * 100, nom_uah);
	if(shbatt_depleted_capacity_pos < SHBATT_DEPLETED_JUDGE_AVERAGE_COUNT)
	{
		shbatt_depleted_capacity_array[shbatt_depleted_capacity_pos] = dep_per;
		shbatt_depleted_capacity_pos++;
	} else {
		for(count = 1; count < shbatt_depleted_capacity_pos; count++)
		{
			shbatt_depleted_capacity_array[count-1] = shbatt_depleted_capacity_array[count];
		}
		shbatt_depleted_capacity_array[count-1] = dep_per;
	}
	shbatt_depleted_backup_params();

	shbatt_cur_depleted_val = dep_per;

	/* Good: 0, Depleted: 1, Acceptable: 2 */
	if(shbatt_avr_depleted_val >= high_thresh)
		dep_result = 0;
	else if(shbatt_avr_depleted_val <= low_thresh)
		dep_result = 1;
	else /* low_thresh < dep_per < high_thresh*/
		dep_result = 2;

	SHBATT_TRACE("[P] %s : prev_dep_result=%d dep_result=%d\n", __FUNCTION__, prev_dep_result, dep_result);
	SHPWR_LOG_INFO("shbatt_cur_depleted_val=%d shbatt_avr_depleted_val=%d\n", shbatt_cur_depleted_val, shbatt_avr_depleted_val);

    if ( prev_dep_result != dep_result ) {
        sprintf(dep_result_char, "%d", dep_result);
        event_data.cmd  = SHBATT_REQ_CMD_CAP_LEARNING_WRITE_RESULT;
        event_data.data_0 = dep_result;
        event_data.data_1 = 0;
        event_data.data_2 = 0;
        res = shbatt_send_uevent( event_data );

        if ( res == SHBATT_RESULT_SUCCESS ) {
            prev_dep_result = dep_result;
        } else if ( prev_dep_result != CAP_LEARNING_UNINITIALIZED_VAL ) {
            prev_dep_result = dep_result;
        }
    }

	SHBATT_TRACE("[E] %s\n", __FUNCTION__);
	return result;
}

shbatt_result_t cap_learning_update(struct shbatt_chip *chip, struct power_supply *psy)
{
    static int pre_learned_cc_uah = 0;
    shbatt_result_t result = SHBATT_RESULT_SUCCESS;
    union power_supply_propval val = {0,};
    int learned_cc_uah = 0;
    int nom_batt_cap_uah = 0;
    int ret;

    SHBATT_TRACE("[P] %s start.\n", __FUNCTION__);

    if (!psy) {
        SHBATT_ERROR("%s : psy is null.\n",__FUNCTION__);
        goto out;
    }

    ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CHARGE_FULL, &val);
    if (!ret) learned_cc_uah = val.intval;

    if ( shbatt_fake_learned_cc > UNINITIALIZED ) {
        pr_info("[P] %s : Warning!! use fake_learned_cc %d->%d\n", __FUNCTION__, learned_cc_uah, shbatt_fake_learned_cc);
        learned_cc_uah = shbatt_fake_learned_cc;
    }

    if ( (pre_learned_cc_uah == 0) || (pre_learned_cc_uah == learned_cc_uah) )
        goto out;

    ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &val);
    if (!ret) nom_batt_cap_uah = val.intval;

    if ( (learned_cc_uah > 0) && (nom_batt_cap_uah > 0) ) {
        SHBATT_TRACE("[P] %s cap learning result store. learned_cc_uah:%d, nom_batt_cap_uah:%d\n",
            __FUNCTION__, learned_cc_uah, nom_batt_cap_uah);

        shbatt_api_store_fg_cap_learning_result(
            learned_cc_uah, nom_batt_cap_uah,
            chip->cl_high_thresh, chip->cl_low_thresh);
    }

out:
    pre_learned_cc_uah = learned_cc_uah;
    SHBATT_TRACE("[P] %s end.\n", __FUNCTION__);

    return result;
}


static int cl_info_cc_stored_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;

	ret = kstrtoint(val, 0, &value);
	if(!ret) {
		mutex_lock(&the_chip->cl_info.cl_info_lock);
		the_chip->cl_info.cc_stored = value;
		mutex_unlock(&the_chip->cl_info.cl_info_lock);
	}

	return ret;
}
static int cl_info_cc_stored_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	ret = sprintf(buf, "%d", the_chip->cl_info.cc_stored);

	return ret;
}
module_param_call(cc_stored, cl_info_cc_stored_set, cl_info_cc_stored_get, NULL, 0644);

static int cl_info_cc_thresh_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;

	ret = kstrtoint(val, 0, &value);
	if(!ret) {
		mutex_lock(&the_chip->cl_info.cl_info_lock);
		the_chip->cl_info.cc_thresh[SHBATT_FV_AGED_LEVEL0] = value;
		mutex_unlock(&the_chip->cl_info.cl_info_lock);
	}

	return ret;
}
static int cl_info_cc_thresh_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	ret = sprintf(buf, "%d", the_chip->cl_info.cc_thresh[SHBATT_FV_AGED_LEVEL0]);

	return ret;
}
module_param_call(cc_thresh, cl_info_cc_thresh_set, cl_info_cc_thresh_get, NULL, 0644);

static int cl_info_cc_update_thresh_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;

	ret = kstrtoint(val, 0, &value);
	if(!ret) {
		mutex_lock(&the_chip->cl_info.cl_info_lock);
		the_chip->cl_info.cc_update_thresh = value;
		mutex_unlock(&the_chip->cl_info.cl_info_lock);
	}

	return ret;
}
static int cl_info_cc_update_thresh_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	ret = sprintf(buf, "%d", the_chip->cl_info.cc_update_thresh);

	return ret;
}
module_param_call(cc_update_thresh, cl_info_cc_update_thresh_set, cl_info_cc_update_thresh_get, NULL, 0644);

static int cl_info_batt_temp_thresh_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;

	ret = kstrtoint(val, 0, &value);
	if(!ret) {
		mutex_lock(&the_chip->cl_info.cl_info_lock);
		the_chip->cl_info.batt_temp_thresh = value;
		mutex_unlock(&the_chip->cl_info.cl_info_lock);
	}

	return ret;
}
static int cl_info_batt_temp_thresh_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	ret = sprintf(buf, "%d", the_chip->cl_info.batt_temp_thresh);

	return ret;
}
module_param_call(batt_temp_thresh, cl_info_batt_temp_thresh_set, cl_info_batt_temp_thresh_get, NULL, 0644);

static int cl_info_soc_thresh_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;

	ret = kstrtoint(val, 0, &value);
	if(!ret) {
		mutex_lock(&the_chip->cl_info.cl_info_lock);
		the_chip->cl_info.soc_thresh = value;
		mutex_unlock(&the_chip->cl_info.cl_info_lock);
	}

	return ret;
}
static int cl_info_soc_thresh_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	ret = sprintf(buf, "%d", the_chip->cl_info.soc_thresh);

	return ret;
}
module_param_call(soc_thresh, cl_info_soc_thresh_set, cl_info_soc_thresh_get, NULL, 0644);

static int cl_info_rtc_stored_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;

	ret = kstrtoint(val, 0, &value);
	if(!ret) {
		mutex_lock(&the_chip->cl_info.cl_info_lock);
		the_chip->cl_info.rtc_stored = (unsigned long)value;
		mutex_unlock(&the_chip->cl_info.cl_info_lock);
	}

	return ret;
}

static int cl_info_rtc_stored_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	ret = sprintf(buf, "%d", (int)the_chip->cl_info.rtc_stored);

	return ret;
}
module_param_call(rtc_stored, cl_info_rtc_stored_set, cl_info_rtc_stored_get, NULL, 0644);

static int cl_info_rtc_thresh_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;

	ret = kstrtoint(val, 0, &value);
	if(!ret) {
		mutex_lock(&the_chip->cl_info.cl_info_lock);
		the_chip->cl_info.rtc_thresh[SHBATT_FV_AGED_LEVEL0] = (unsigned long)value;
		mutex_unlock(&the_chip->cl_info.cl_info_lock);
	}

	return ret;
}

static int cl_info_rtc_thresh_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	ret = sprintf(buf, "%d", (int)the_chip->cl_info.rtc_thresh[SHBATT_FV_AGED_LEVEL0]);

	return ret;
}
module_param_call(rtc_thresh, cl_info_rtc_thresh_set, cl_info_rtc_thresh_get, NULL, 0644);

static int cl_info_rtc_update_thresh_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;

	ret = kstrtoint(val, 0, &value);
	if(!ret) {
		mutex_lock(&the_chip->cl_info.cl_info_lock);
		the_chip->cl_info.rtc_update_thresh = (unsigned long)value;
		mutex_unlock(&the_chip->cl_info.cl_info_lock);
	}

	return ret;
}

static int cl_info_rtc_update_thresh_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	ret = sprintf(buf, "%d", (int)the_chip->cl_info.rtc_update_thresh);

	return ret;
}
module_param_call(rtc_update_thresh, cl_info_rtc_update_thresh_set, cl_info_rtc_update_thresh_get, NULL, 0644);

void shbatt_api_cl_aged_level_decision(struct shbatt_chip *chip)
{
	int ret;
	cap_learning_info_t *cl_info = &chip->cl_info;
	struct power_supply*		batt_psy = NULL;
	union power_supply_propval	val = {0,};

	SHBATT_TRACE("[S] %s \n", __FUNCTION__);

	if (!cl_info->cl_info_enbale) {
		SHBATT_ERROR("%s : cl_info_enbale is disabled\n", __FUNCTION__);
		goto out;
	}

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		SHBATT_ERROR("%s : batt_psy is null\n", __FUNCTION__);
		goto out;
	}

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_BATT_AGE_LEVEL, &val);
	if (ret) {
		SHBATT_ERROR("%s : can't get property batt_age_level\n", __FUNCTION__);
		goto out;
	}

	if (cl_info->fv_aged_level != val.intval)
		SHPWR_LOG_INFO("cl_info->fv_aged_level(%d) does't match current batt_psy batt_age_level(%d)\n", cl_info->fv_aged_level, val.intval);

	if (cl_info->fv_aged_level == val.intval)
		goto out;

	SHPWR_LOG_INFO("change fv_aged_level = %d -> %d\n", val.intval, cl_info->fv_aged_level);

	val.intval = cl_info->fv_aged_level;

	ret = batt_psy->desc->set_property(batt_psy, POWER_SUPPLY_PROP_BATT_AGE_LEVEL, &val);
	if (ret) SHBATT_ERROR("%s : can't set property batt_age_level\n", __FUNCTION__);

out:
	SHBATT_TRACE("[E] %s \n", __FUNCTION__);

	return;
}

void shbatt_api_cl_update_acc_soc(struct shbatt_chip *chip)
{
	int ret;
	cap_learning_info_t *cl_info = &chip->cl_info;
	struct power_supply*		batt_psy = NULL;
	union power_supply_propval	val = {0,};
	int charge_status = -1;
	const char * const power_supply_status_text[] = {
		"Unknown", "Charging", "Discharging", "Not charging", "Full"
	};

	SHBATT_TRACE("[S] %s \n", __FUNCTION__);

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		SHBATT_ERROR("%s : batt_psy is null\n", __FUNCTION__);
		goto out;
	}

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_MSOC, &val);
	if (!ret) cl_info->cc_now = val.intval;

	if (cl_info->is_cc_start) {
		cl_info->cc_accumulated += cl_info->cc_now - cl_info->cc_prev;

		SHBATT_TRACE("[P] %s cc_now:%d, cc_prev:%d, cc_thresh:%d, cc_accumulated:%d, cc_stored:%d\n",
						__FUNCTION__, cl_info->cc_now, cl_info->cc_prev, cl_info->cc_thresh[cl_info->fv_aged_level],
						cl_info->cc_accumulated, cl_info->cc_stored);

		cl_info->cc_prev = cl_info->cc_now;

		if ((cl_info->cc_stored + cl_info->cc_accumulated) < cl_info->cc_thresh[cl_info->fv_aged_level]) {
			cl_info->fv_aged_level_cc = cl_info->fv_aged_level;
			if (cl_info->cc_accumulated >= cl_info->cc_update_thresh) {
				cl_info->cc_stored += cl_info->cc_accumulated;
				cl_info->cc_accumulated = 0;
				cl_info->durable_update = true;
			}
		} else {
			cl_info->fv_aged_level_cc = min(cl_info->fv_aged_level + 1, cl_info->fv_aged_level_max);
			if (cl_info->cc_accumulated >= cl_info->cc_update_thresh) {
				cl_info->cc_stored += cl_info->cc_accumulated;
				cl_info->cc_accumulated = 0;
				cl_info->durable_update = true;
			}
		}
	}

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_STATUS, &val);
	if (!ret) charge_status = val.intval;

	if (charge_status == POWER_SUPPLY_STATUS_CHARGING) {
		if (!cl_info->is_cc_start) {
			cl_info->is_cc_start = true;
			cl_info->cc_prev = cl_info->cc_now;
			SHPWR_LOG_INFO("cc_soc accumulating start cc_now:%d, charge_status:%s\n",
							cl_info->cc_now, power_supply_status_text[charge_status]);
		}
	} else {
		if (cl_info->is_cc_start) {
			cl_info->is_cc_start = false;
			cl_info->cc_prev = 0;
			if ((charge_status >= POWER_SUPPLY_STATUS_UNKNOWN) && (charge_status <= POWER_SUPPLY_STATUS_FULL)) {
				SHPWR_LOG_INFO("cc_soc accumulating end cc_now:%d, charge_status:%s\n",
								cl_info->cc_now, power_supply_status_text[charge_status]);
			} else {
				SHPWR_LOG_INFO("Warning!! cc_soc accumulating illegal end cc_now:%d, charge_status:%d\n",
								cl_info->cc_now, charge_status);
			}
		}
	}

out:
	SHBATT_TRACE("[E] %s \n", __FUNCTION__);

	return;
}

void shbatt_api_cl_update_acc_rtc(struct shbatt_chip *chip)
{
	int ret;
	cap_learning_info_t *cl_info = &chip->cl_info;
	struct power_supply*		batt_psy = NULL;
	union power_supply_propval	val = {0,};
	unsigned long rtc_now = 0;

	SHBATT_TRACE("[S] %s \n", __FUNCTION__);

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		SHBATT_ERROR("%s : batt_psy is null\n", __FUNCTION__);
		goto out;
	}

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (!ret) cl_info->batt_temp = val.intval;

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (!ret) cl_info->soc = val.intval;

	ret = get_current_time(&rtc_now);
	if (!ret) cl_info->rtc_now = rtc_now;

	if (cl_info->is_rtc_start) {
		cl_info->rtc_accumulated += cl_info->rtc_now - cl_info->rtc_prev;

		SHBATT_TRACE("[P] %s rtc_now:%d, rtc_prev:%d, rtc_thresh:%d, rtc_accumulated:%d, rtc_stored:%d\n",
						__FUNCTION__, cl_info->rtc_now, cl_info->rtc_prev, cl_info->rtc_thresh[cl_info->fv_aged_level],
						cl_info->rtc_accumulated, cl_info->rtc_stored);

		cl_info->rtc_prev = cl_info->rtc_now;

		if ((cl_info->rtc_stored + cl_info->rtc_accumulated) < cl_info->rtc_thresh[cl_info->fv_aged_level]) {
			cl_info->fv_aged_level_rtc = cl_info->fv_aged_level;
			if (cl_info->rtc_accumulated >= cl_info->rtc_update_thresh) {
				cl_info->rtc_stored += cl_info->rtc_accumulated;
				cl_info->rtc_accumulated = 0;
				cl_info->durable_update = true;
			}
		} else {
			cl_info->fv_aged_level_rtc = min(cl_info->fv_aged_level + 1, cl_info->fv_aged_level_max);
			cl_info->rtc_stored += cl_info->rtc_accumulated;
			cl_info->rtc_accumulated = 0;
			cl_info->durable_update = true;
		}
	}

	if ((cl_info->batt_temp >= cl_info->batt_temp_thresh) && (cl_info->soc >= cl_info->soc_thresh)) {
		if (!cl_info->is_rtc_start) {
			cl_info->is_rtc_start = true;
			cl_info->rtc_prev = cl_info->rtc_now;
			SHPWR_LOG_INFO("rtc accumulating start rtc_now:%d, batt_temp:%d soc:%d\n",
							cl_info->rtc_now, cl_info->batt_temp, cl_info->soc);
		}
	} else {
		if (cl_info->is_rtc_start) {
			cl_info->is_rtc_start = false;
			cl_info->rtc_prev = 0;
			SHPWR_LOG_INFO("rtc accumulating end rtc_now:%d, batt_temp:%d soc:%d\n",
							cl_info->rtc_now, cl_info->batt_temp, cl_info->soc);
		}
	}

out:
	SHBATT_TRACE("[E] %s \n", __FUNCTION__);
	return;
}

shbatt_result_t shbatt_api_cl_load_info(struct shbatt_chip *chip)
{
    cap_learning_info_t *cl_info      = &chip->cl_info;
    shbatt_result_t result            = SHBATT_RESULT_SUCCESS;
    shbatt_fv_aged_level_t aged_level = SHBATT_FV_AGED_LEVEL0;

    SHBATT_TRACE("[S] %s\n", __FUNCTION__);

    // aged level decision
    for (aged_level = SHBATT_FV_AGED_LEVEL0; aged_level < cl_info->fv_aged_level_max; aged_level++) {
        if ( (cl_info->cc_stored < cl_info->cc_thresh[aged_level])
                && (cl_info->rtc_stored < cl_info->rtc_thresh[aged_level]) ) {
            break;
        }
    }
    cl_info->fv_aged_level = aged_level;

    SHPWR_LOG_INFO("cl_info loaded cc_stored:%d, rtc_stored:%d aged_level:%d\n",
                        cl_info->cc_stored, cl_info->rtc_stored, cl_info->fv_aged_level);

    shbatt_api_cl_aged_level_decision(chip);

    cl_info->load_completed = true;

    SHBATT_TRACE("[E] %s\n", __FUNCTION__);

    return result;
}

shbatt_result_t shbatt_api_cl_store_info(struct shbatt_chip *chip)
{
	cap_learning_info_t *cl_info = &chip->cl_info;
	shbatt_result_t result = SHBATT_RESULT_SUCCESS;
    shbatt_uevent_data event_data;

	SHBATT_TRACE("[S] %s\n", __FUNCTION__);

	SHPWR_LOG_INFO("cl_info store cc_stored:%d, rtc_stored:%d\n", cl_info->cc_stored, cl_info->rtc_stored);

    event_data.cmd  = SHBATT_REQ_CMD_CAP_LEARNING_WRITE_CYCLE_INFO;
    event_data.data_0 = cl_info->cc_stored;
    event_data.data_1 = cl_info->rtc_stored;
    event_data.data_2 = 0;
    result = shbatt_send_uevent( event_data );

	SHBATT_TRACE("[E] %s\n", __FUNCTION__);
	return result;
}

void shbatt_api_cl_update_fv_aged_level(struct shbatt_chip *chip)
{
    cap_learning_info_t *cl_info = &chip->cl_info;

    SHBATT_TRACE("[S] %s \n", __FUNCTION__);

    mutex_lock(&cl_info->cl_info_lock);

    if ( !shpwr_is_initialized() || !isInitialized_cycle ) {
        SHBATT_TRACE("[P] %s initialize waiting..", __FUNCTION__);
        goto out;
    }

    if ( !cl_info->load_completed )
        shbatt_api_cl_load_info(chip);

    cl_info->durable_update = false;

    shbatt_api_cl_update_acc_soc(chip);
    shbatt_api_cl_update_acc_rtc(chip);

    cl_info->fv_aged_level = max( max( cl_info->fv_aged_level, cl_info->fv_aged_level_cc ),
                                  cl_info->fv_aged_level_rtc );

    SHBATT_TRACE("[P] %s cl_info update cc_stored:%d, rtc_stored:%d, aged_level:%d\n",
                    __FUNCTION__, cl_info->cc_stored,
                    cl_info->rtc_stored, cl_info->fv_aged_level);

    if ((cl_info->fv_aged_level > SHBATT_FV_AGED_LEVEL0)
        && (cl_info->fv_aged_level <= cl_info->fv_aged_level_max))
    {
        shbatt_api_cl_aged_level_decision(chip);
    }

    if (cl_info->durable_update) {
        /* store cc_stored and rtc_stored to durable */
        shbatt_api_cl_store_info(chip);
    }

out:
    shbatt_api_battlog_fv_aged_level(cl_info->fv_aged_level);

    mutex_unlock(&cl_info->cl_info_lock);

    SHBATT_TRACE("[E] %s \n", __FUNCTION__);

    return;
}

#ifdef CONFIG_SHARP_SHTERM
static shbatt_result_t shbatt_seq_battlog_event(
	int		evt
) {
	shbattlog_info_t	shterm_bli;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	memset(&shterm_bli, 0, sizeof(shterm_bli));

	shterm_bli.event_num = evt;

	shterm_k_set_event(&shterm_bli);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return SHBATT_RESULT_SUCCESS;
}
#endif /* CONFIG_SHARP_SHTERM */

/*+-----------------------------------------------------------------------------+*/
/*| @ PLATFORM DRIVER MODULE CODE AREA :                                        |*/
/*+-----------------------------------------------------------------------------+*/
static void shbatt_release( struct kobject *kobj )
{
	kfree( kobj );
}

static ssize_t shbatt_info_show( struct kobject *kobj, struct attribute *attr, char *buf )
{
    if ( !strcmp(attr->name, "depleted_init") ) {
        sprintf( buf, "%d", shbatt_sysfs_depleted_init );
    } else if ( !strcmp(attr->name, "depleted_ver") ) {
        sprintf( buf, "%d", shbatt_sysfs_depleted_ver );
    } else if ( !strcmp(attr->name, "depleted_ave") ) {
        sprintf( buf, "%d", shbatt_sysfs_depleted_ave );
    } else if ( !strcmp(attr->name, "depleted_pos") ) {
        sprintf( buf, "%d", shbatt_sysfs_depleted_pos );
    } else if ( !strcmp(attr->name, "depleted_result") ) {
        sprintf( buf, "%d", shbatt_sysfs_depleted_result );
    } else if ( !strcmp(attr->name, "charging_cycle_init") ) {
        sprintf( buf, "%d", shbatt_sysfs_charging_cycle_init );
    } else if ( !strcmp(attr->name, "charging_cycle_cc") ) {
        sprintf( buf, "%d", shbatt_sysfs_charging_cycle_cc );
    } else if ( !strcmp(attr->name, "charging_cycle_rtc") ) {
        sprintf( buf, "%d", shbatt_sysfs_charging_cycle_rtc );
    }

    return strlen( buf );
}

static ssize_t shbatt_info_store( struct kobject *kobj, struct attribute *attr, const char *buf, size_t len )
{

    if ( !strcmp(attr->name, "depleted_init") ) {
        shbatt_sysfs_depleted_init = (int)simple_strtol( buf, (char **)NULL, 10 );
    } else if ( !strcmp(attr->name, "depleted_ver") ) {
        shbatt_sysfs_depleted_ver = (int)simple_strtol( buf, (char **)NULL, 10 );
    } else if ( !strcmp(attr->name, "depleted_ave") ) {
        shbatt_sysfs_depleted_ave = (int)simple_strtol( buf, (char **)NULL, 10 );
    } else if ( !strcmp(attr->name, "depleted_pos") ) {
        shbatt_sysfs_depleted_pos = (int)simple_strtol( buf, (char **)NULL, 10 );
    } else if ( !strcmp(attr->name, "depleted_result") ) {
        shbatt_sysfs_depleted_result = (int)simple_strtol( buf, (char **)NULL, 10 );
    } else if ( !strcmp(attr->name, "charging_cycle_init") ) {
        shbatt_sysfs_charging_cycle_init = (int)simple_strtol( buf, (char **)NULL, 10 );
    } else if ( !strcmp(attr->name, "charging_cycle_cc") ) {
        shbatt_sysfs_charging_cycle_cc = (int)simple_strtol( buf, (char **)NULL, 10 );
    } else if ( !strcmp(attr->name, "charging_cycle_rtc") ) {
        shbatt_sysfs_charging_cycle_rtc = (int)simple_strtol( buf, (char **)NULL, 10 );
    }

    return len;
}

static int shbatt_drv_open(
	struct inode*				in_p,
	struct file*				fi_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
	return 0;
}

static int shbatt_drv_release(
	struct inode*				in_p,
	struct file*				fi_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
	return 0;
}

static unsigned int shbatt_drv_poll(
	struct file*				fi_p,
	poll_table*					wait_p
){
	unsigned					int mask = 0;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );

	return mask;
}

static long shbatt_drv_ioctl(
	struct file*				fi_p,
	unsigned int				cmd,
	unsigned long				arg
){
	int							ret = -EPERM;

	SHBATT_TRACE("[S] %s() cmd=0x%x\n",__FUNCTION__,cmd);

	SHBATT_TRACE("[E] %s() ret=%d \n",__FUNCTION__,ret);

	return ret;
}

static bool cap_learning_init_battery_depleted()
{
    bool isResult = false;

    SHBATT_TRACE("[S] %s() \n",__FUNCTION__);

    /* Check if initialization is complete. */
    if ( shbatt_sysfs_depleted_init == UNINITIALIZED )
        goto failed;

    /* Get the restored value. */
    if ( shbatt_sysfs_depleted_ver == UNINITIALIZED ) {
        /* File does not exist. value is the initial value. */
        restored_dep_ver = shbatt_depleted_calc_ver;
    } else {
        restored_dep_ver = shbatt_sysfs_depleted_ver;
    }

    if ( shbatt_sysfs_depleted_ave == UNINITIALIZED ) {
        /* File does not exist. value is the initial value. */
        restored_dep_ave = 0;
    } else {
        restored_dep_ave = shbatt_sysfs_depleted_ave;
    }

    if ( shbatt_sysfs_depleted_pos == UNINITIALIZED ) {
        /* File does not exist. value is the initial value. */
        restored_dep_pos = 0;
    } else {
        restored_dep_pos = shbatt_sysfs_depleted_pos;
    }

    if ( shbatt_sysfs_depleted_result == UNINITIALIZED ) {
        /* File does not exist. value is the initial value. */
        restored_dep_res = CAP_LEARNING_UNINITIALIZED_VAL;
    } else {
        restored_dep_res = shbatt_sysfs_depleted_result;
    }

    SHBATT_TRACE("[P] %s : restore_params : completed [%d, %d, %d, %d]\n", __FUNCTION__,
                                restored_dep_ver, restored_dep_ave,
                                restored_dep_pos, restored_dep_res );

    isInitialized_dep = true;
    isResult = true;

failed:
    SHBATT_TRACE("[E] %s \n", __FUNCTION__);

    return isResult;
}

static bool cap_learning_init_charging_cycle(
    struct shbatt_chip  *chip
){
    cap_learning_info_t *cl_info;
    bool isResult = false;

    SHBATT_TRACE("[S] %s() \n",__FUNCTION__);

    if ( chip == NULL ) {
        SHBATT_ERROR("%s : shbatt_chip is null\n", __FUNCTION__);
        goto failed;
    }

    cl_info = &chip->cl_info;

    if ( !cl_info->cl_info_enbale ) {
        isInitialized_cycle = true;
        isResult = true;
        goto failed;
    }

    /* Check if initialization is complete. */
    if ( shbatt_sysfs_charging_cycle_init == UNINITIALIZED )
        goto failed;

    /* Get the restored value. */
    if ( shbatt_sysfs_charging_cycle_cc == UNINITIALIZED ) {
        /* File does not exist. value is the initial value. */
        cl_info->cc_stored = 0;
    } else {
        cl_info->cc_stored = shbatt_sysfs_charging_cycle_cc;
    }

    if ( shbatt_sysfs_charging_cycle_rtc == UNINITIALIZED ) {
        /* File does not exist. value is the initial value. */
        cl_info->rtc_stored = 0;
    } else {
        cl_info->rtc_stored = shbatt_sysfs_charging_cycle_rtc;
    }

    SHBATT_TRACE("[P] %s : restore_params : completed [%d, %d]\n", __FUNCTION__,
                                cl_info->cc_stored, cl_info->rtc_stored );

    isInitialized_cycle = true;
    isResult = true;

    /* Change the setting to the previous AGE LEVEL */
    shbatt_api_cl_update_fv_aged_level(chip);

failed:
    SHBATT_TRACE("[E] %s \n", __FUNCTION__);

    return isResult;
}

static void cap_learning_initialize_work(struct work_struct *work)
{
    struct shbatt_chip *chip = container_of(
                        work,
                        struct shbatt_chip,
                        cap_learning_initialize_work.work );
    static int retry_countt = 1;

    SHBATT_TRACE("[S] %s() \n",__FUNCTION__);

    if ( chip == NULL ) {
        SHBATT_ERROR("%s : shbatt_chip is null\n", __FUNCTION__);
        goto retry;
    }

    if ( !isInitialized_dep )
        cap_learning_init_battery_depleted();

    if ( !isInitialized_cycle )
        cap_learning_init_charging_cycle( chip );

    if ( isInitialized_dep && isInitialized_cycle )
        goto completed;

retry:
    if ( retry_countt <= CAP_LEARNING_INIT_RETRY_MAX_COUNT ) {
        SHBATT_TRACE("[P] %s : cap_learning_initialize_work rescheduled(%d / %d). dep=%d, cycle=%d\n",
                                __FUNCTION__, retry_countt, CAP_LEARNING_INIT_RETRY_MAX_COUNT,
                                isInitialized_dep, isInitialized_cycle);
        schedule_delayed_work( &chip->cap_learning_initialize_work,
                                msecs_to_jiffies(CAP_LEARNING_INIT_WORK_DELAY) );
        retry_countt++;
    } else {
        SHBATT_ERROR("%s : depleted_restore_params : failed (dep=%d, cycle=%d)\n", __FUNCTION__,
                                isInitialized_dep, isInitialized_cycle);
    }


completed:
    SHBATT_TRACE("[E] %s \n", __FUNCTION__);
}


static void batt_psy_changed_work(struct work_struct *work)
{
	int ret;
	struct shbatt_chip *chip = container_of(work,
			struct shbatt_chip, batt_psy_changed_work);
	union power_supply_propval	val = {0,};
	struct power_supply*		batt_psy = NULL;
	struct power_supply*		usb_psy = NULL;
	struct power_supply*		dc_psy = NULL;
	bool usb_present = false, dc_present = false;
	int charge_status = -1;
	int capacity;

	dev_dbg(chip->dev, "batt_psy_changed_work start\n");

	if (!is_shbatt_task_initialized()) {
		dev_err(chip->dev, "shbatt_task is not initialize\n");
		goto out;
	}

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		dev_err(chip->dev, "batt_psy is null\n");
		goto out;
	}

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_err(chip->dev, "usb_psy is null\n");
		goto out;
	}

	dc_psy = power_supply_get_by_name("dc");
	if (!dc_psy) {
		dev_dbg(chip->dev, "dc_psy is null\n");
	}

	ret = usb_psy->desc->get_property(usb_psy, POWER_SUPPLY_PROP_PRESENT, &val);
	if (!ret)
		usb_present = (bool)val.intval;

	if ( dc_psy ) {
		ret = dc_psy->desc->get_property(dc_psy, POWER_SUPPLY_PROP_PRESENT, &val);
		if (!ret)
			dc_present = (bool)val.intval;
	}

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &val);
	if (!ret)
		shbatt_api_battlog_system_temp_level(val.intval);

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_DIRECT_CHARGE_MODE, &val);
	if (!ret)
		shbatt_api_battlog_direct_charge_mode(val.intval);

# if 0
	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_STORMING_STATUS, &val);
	if (!ret)
		shbatt_api_battlog_shortage_st(val.intval);
#endif

	charge_status = -1;
	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_STATUS, &val);
	if (!ret) {
		charge_status = val.intval;
		shbatt_api_battlog_charge_status(charge_status);
		if (charge_status == POWER_SUPPLY_STATUS_NOT_CHARGING) {
			//OVP and Safty Timer Expired check
			ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_CHARGER_ERROR_STATUS, &val);
			if (!ret)
				shbatt_api_battlog_charge_error(val.intval);
		}
	}

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_HEALTH, &val);
	if ((!ret) && (usb_present || dc_present))
			shbatt_api_battlog_jeita_status(val.intval);

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (!ret) {
		capacity = val.intval;
		shbatt_api_battlog_capacity(capacity);
	}

	/* capacity learning status check */
	cap_learning_update(chip, batt_psy);

	if(chip->cl_info.cl_info_enbale)
		shbatt_api_cl_update_fv_aged_level(chip);

out:
	pm_relax(chip->dev);
}

static void usb_psy_changed_work(struct work_struct *work)
{
	int ret;
	bool present = false;
	struct shbatt_chip *chip = container_of(work,
			struct shbatt_chip, usb_psy_changed_work);
	union power_supply_propval	val = {0,};
	struct power_supply*		usb_psy = NULL;
	struct power_supply*		batt_psy = NULL;

	dev_dbg(chip->dev, "usb_psy_changed_work start\n");

	if(!is_shbatt_task_initialized()) {
		dev_err(chip->dev, "shbatt_task is not initialize\n");
		goto out;
	}

	usb_psy = power_supply_get_by_name("usb");
	if(!usb_psy) {
		dev_err(chip->dev, "usb_psy is null\n");
		goto out;
	}

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		dev_err(chip->dev, "batt_psy is null\n");
		goto out;
	}

	ret = usb_psy->desc->get_property(usb_psy, POWER_SUPPLY_PROP_PRESENT, &val);
	if (!ret) {
		shbatt_api_battlog_usb_present(val.intval);
		present = (bool)val.intval;
	}

	ret = usb_psy->desc->get_property(usb_psy, POWER_SUPPLY_PROP_USB_TYPE, &val);
	if (!ret)
		shbatt_api_battlog_usb_type(val.intval);

	ret = usb_psy->desc->get_property(usb_psy, POWER_SUPPLY_PROP_PARTNER_TYPE, &val);
	if (!ret)
		shbatt_api_battlog_partner_type(val.intval);

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_INPUT_SUSPEND, &val);
	if (!ret)
		shbatt_api_battlog_input_suspend(val.intval);

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_CC_SAFETY_LEVEL, &val);
	if (!ret)
		shbatt_api_battlog_typec_overheat_st(val.intval);


out:
	pm_relax(chip->dev);
}

static void dc_psy_changed_work(struct work_struct *work)
{
	int ret;
	struct shbatt_chip *chip = container_of(work,
			struct shbatt_chip, dc_psy_changed_work);
	union power_supply_propval	val = {0,};
	struct power_supply*		dc_psy = NULL;

	dev_dbg(chip->dev, "dc_psy_changed_work start\n");

	if(!is_shbatt_task_initialized()) {
		dev_err(chip->dev, "shbatt_task is not initialize\n");
		goto out;
	}

	dc_psy = power_supply_get_by_name("dc");
	if(!dc_psy) {
		dev_err(chip->dev, "dc_psy is null\n");
		goto out;
	}

	ret = dc_psy->desc->get_property(dc_psy, POWER_SUPPLY_PROP_PRESENT, &val);
	if (!ret)
		shbatt_api_battlog_dc_present(val.intval);

out:
	pm_relax(chip->dev);
}

static int shbatt_notifier_cb(struct notifier_block *nb, unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct shbatt_chip *chip = container_of(nb, struct shbatt_chip, nb);

	dev_dbg(chip->dev, "notifier call back :%s\n", psy->desc->name);

	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (strcmp(psy->desc->name, "battery") == 0) {
		if (work_pending(&chip->batt_psy_changed_work))
			return NOTIFY_OK;

		pm_stay_awake(chip->dev);
		schedule_work(&chip->batt_psy_changed_work);
	}
	else if (strcmp(psy->desc->name, "usb") == 0) {
		if (work_pending(&chip->usb_psy_changed_work))
			return NOTIFY_OK;

		pm_stay_awake(chip->dev);
		schedule_work(&chip->usb_psy_changed_work);
	}
	else if (strcmp(psy->desc->name, "dc") == 0) {
		if (work_pending(&chip->dc_psy_changed_work))
			return NOTIFY_OK;

		pm_stay_awake(chip->dev);
		schedule_work(&chip->dc_psy_changed_work);
	}

	return NOTIFY_OK;
}

static bool disp_on_input = true;
static ssize_t show_disp_on_input(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", disp_on_input);
}

struct kobject *shbatt_kerl_kobj;
static struct kobj_attribute disp_on_input_attr = __ATTR(disp_on_input, S_IRUGO, show_disp_on_input, NULL);

static int shbatt_drm_notifier_cb(struct notifier_block *self, unsigned long event, void *data)
{
	int blank;
	struct drm_panel_notifier *evdata = data;
	bool prev_disp_on_input;

	if ( !shbatt_kerl_kobj ) {
		SHBATT_ERROR("[E] %s kobj is NULL\n", __FUNCTION__);
		return 0;
	}

	if (data) {
		blank = *(int *)(evdata->data);
		prev_disp_on_input = disp_on_input;
		switch (event) {
		case DRM_PANEL_EARLY_EVENT_BLANK:
			if (blank == DRM_PANEL_BLANK_POWERDOWN || blank == DRM_PANEL_BLANK_LP)
				disp_on_input = false;
			break;
		case DRM_PANEL_EVENT_BLANK:
			if (blank == DRM_PANEL_BLANK_UNBLANK)
				disp_on_input = true;
			break;
		default:
			break;
		}
		if (prev_disp_on_input != disp_on_input)
			sysfs_notify(shbatt_kerl_kobj, NULL, "disp_on_input");

		SHBATT_TRACE("[P] %s() disp_on_input=0x%x\n",__FUNCTION__,disp_on_input);
	} else {
		SHBATT_TRACE("[P] %s() data is null, disp_on_input=0x%x\n",__FUNCTION__,disp_on_input);
	}

	return 0;
}

static int create_kobject( void )
{
    int rc = 0;

    /* parent kobject */
    shbatt_kerl_kobj = kobject_create_and_add( SHBATT_OBJ_NAME, kernel_kobj );
    if ( !shbatt_kerl_kobj ) {
        SHBATT_ERROR("[E] %s Failed to create kernel_kobj\n", __FUNCTION__);
        return -1;
    }
    rc = sysfs_create_file( shbatt_kerl_kobj, &disp_on_input_attr.attr );
    if ( rc )
        SHBATT_ERROR("[E] %s Failed to create disp_on_input sysfs rc=%d\n", __FUNCTION__, rc);


    /* child kobject */
    shbatt_kset = kset_create_and_add( "info", NULL, shbatt_kerl_kobj );
    if ( !shbatt_kset ) {
        SHBATT_ERROR("[E] %s Failed to create kset\n", __FUNCTION__);
        return -1;
    }
    kobj_data.kobj->kset = shbatt_kset;

    rc = kobject_init_and_add( kobj_data.kobj, kobj_data.ktype, NULL, "%s", kobj_data.name );
    if ( rc ) {
        SHBATT_ERROR("[E] %s Failed to kobject init\n", __FUNCTION__);
        kobject_put( kobj_data.kobj );
        return -1;
    }

    return rc;
}

static int shbatt_drv_probe(
	struct platform_device*		dev_p
){
	static struct shbatt_chip *chip;
	struct device_node *charger_node = NULL;

	int rc = 0;
	int byte_len, i;
	u32 temp, temp_array[SHBATT_FV_AGED_LEVEL_NUM];
	int index=0;
	int count;
	struct device_node *node;
	struct drm_panel *panel;


	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	chip = devm_kzalloc(&dev_p->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&dev_p->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	memset(&chip->cl_info, 0 , sizeof(cap_learning_info_t));
	mutex_init(&chip->cl_info.cl_info_lock);

	rc = of_property_read_u32(dev_p->dev.of_node, "sharp,cl-high-thresh", &temp);
	if (rc < 0)
		chip->cl_high_thresh = DEFAULT_CL_HIGH_THRESH;
	else
		chip->cl_high_thresh = temp;

	dev_dbg(&dev_p->dev, "%s: qcom,cl-high-thresh = %d\n", __func__, chip->cl_high_thresh);

	rc = of_property_read_u32(dev_p->dev.of_node, "sharp,cl-low-thresh", &temp);
	if (rc < 0)
		chip->cl_low_thresh = DEFAULT_CL_LOW_THRESH;
	else
		chip->cl_low_thresh = temp;

	dev_dbg(&dev_p->dev, "%s: qcom,cl-low-thresh = %d\n", __func__, chip->cl_low_thresh);

	chip->cl_info.cl_info_enbale = of_property_read_bool(dev_p->dev.of_node, "sharp,cl-info-enbale");
	pr_info("[P] %s : cl_info_enbale: %d\n", __FUNCTION__, chip->cl_info.cl_info_enbale);

	if (of_find_property(dev_p->dev.of_node, "sharp,cl-cc-thresh", &byte_len)) {
		chip->cl_info.fv_aged_level_max = byte_len / sizeof(u32);
		if (chip->cl_info.fv_aged_level_max > SHBATT_FV_AGED_LEVEL_NUM - 1) {
			dev_err(&dev_p->dev, "fv_aged_level_max over!! %d-->%d\n", byte_len, SHBATT_FV_AGED_LEVEL_NUM - 1);
			chip->cl_info.fv_aged_level_max = SHBATT_FV_AGED_LEVEL_NUM - 1;
		}

		rc = of_property_read_u32_array(dev_p->dev.of_node,
											"sharp,cl-cc-thresh",
											chip->cl_info.cc_thresh,
											chip->cl_info.fv_aged_level_max);
		if (rc < 0)
			dev_err(&dev_p->dev, "Couldn't read sharp,cl-cc-thresh rc = %d\n", rc);

		for (index = 0; index < chip->cl_info.fv_aged_level_max; index++)
			dev_info(&dev_p->dev, "cc_thresh[%d]=%d\n", index, chip->cl_info.cc_thresh[index]);
	}

	rc = of_property_read_u32(dev_p->dev.of_node, "sharp,cl-cc-update-thresh", &temp);
	if (rc < 0)
		chip->cl_info.cc_update_thresh = DEFAULT_CL_INFO_CC_UPDATE_THRESH;
	else
		chip->cl_info.cc_update_thresh = temp;

	rc = of_property_read_u32(dev_p->dev.of_node, "sharp,cl-batt-temp-thresh", &temp);
	if (rc < 0)
		chip->cl_info.batt_temp_thresh = DEFAULT_CL_INFO_BATT_TEMP_THRESH;
	else
		chip->cl_info.batt_temp_thresh = temp;

	rc = of_property_read_u32(dev_p->dev.of_node, "sharp,cl-soc-thresh", &temp);
	if (rc < 0)
		chip->cl_info.soc_thresh = DEFAULT_CL_INFO_SOC_THRESH;
	else
		chip->cl_info.soc_thresh = temp;

	if (of_find_property(dev_p->dev.of_node, "sharp,cl-rtc-thresh", &byte_len)) {
		if (chip->cl_info.fv_aged_level_max != byte_len / sizeof(u32)) {
			dev_warn(&dev_p->dev, "cc_thresh levels doesn't match rtc_thresh levels %d-->%d\n",
										chip->cl_info.fv_aged_level_max, byte_len / sizeof(u32));
		}

		memset(&temp_array, 0x0, sizeof(u32) * SHBATT_FV_AGED_LEVEL_NUM);
		rc = of_property_read_u32_array(dev_p->dev.of_node, 
											"sharp,cl-rtc-thresh",
											temp_array,
											chip->cl_info.fv_aged_level_max);
		if (rc < 0) {
			dev_err(&dev_p->dev, "Couldn't read sharp,cl-rtc-thresh rc = %d\n", rc);
		} else {
			for (i = 0; i < chip->cl_info.fv_aged_level_max; i++)
			chip->cl_info.rtc_thresh[i] = (unsigned long)temp_array[i];
		}

		for (index = 0; index < chip->cl_info.fv_aged_level_max; index++)
			dev_info(&dev_p->dev, "rtc_thresh[%d]=%d\n", index, chip->cl_info.rtc_thresh[index]);
	}

	rc = of_property_read_u32(dev_p->dev.of_node, "sharp,cl-rtc-update-thresh", &temp);
	if (rc < 0)
		chip->cl_info.rtc_update_thresh = DEFAULT_CL_INFO_RTC_UPDATE_THRESH;
	else
		chip->cl_info.rtc_update_thresh = temp;

	//check drm panel
	chip->active_panel = NULL;
	count = of_count_phandle_with_args(dev_p->dev.of_node, "panel", NULL);
	if (count > 0) {
		for (i = 0; i < count; i++) {
			node = of_parse_phandle(dev_p->dev.of_node, "panel", i);
			panel = of_drm_find_panel(node);
			of_node_put(node);
			if (!IS_ERR(panel) && (PTR_ERR(panel) != -EPROBE_DEFER)) {
				chip->active_panel = panel;
				break;
			}
		}
	}

	charger_node = of_find_compatible_node(NULL, NULL, "qcom,battery-charger");
	if (charger_node) {
		if (of_find_property(charger_node, "qcom,thermal-mitigation", &byte_len)) {
			chip->thermal_mitigation = devm_kzalloc(&dev_p->dev, byte_len, GFP_KERNEL);

			if (chip->thermal_mitigation != NULL) {
				chip->thermal_levels = byte_len / sizeof(u32);
				rc = of_property_read_u32_array(charger_node,
						"qcom,thermal-mitigation",
						chip->thermal_mitigation,
						chip->thermal_levels);
				if (rc < 0) {
					dev_err(&dev_p->dev, "Couldn't read threm limits rc = %d\n", rc);
				}
			}
		}
	}
	else {
		dev_err(&dev_p->dev, "%s: qcom,battery-charger not found, initialize to default value\n", __func__);
		chip->thermal_mitigation = NULL;
		chip->thermal_levels = 0;
	}
	dev_dbg(&dev_p->dev, "%s: qcom,thermal-mitigation num = %d\n", __func__, chip->thermal_levels);

	chip->dev = &dev_p->dev;
	the_chip = chip;

	shbatt_wakeup_source = wakeup_source_register(the_chip->dev, "shbatt_wake");
	if (!shbatt_wakeup_source) {
		SHBATT_ERROR("[E] %s Failed to register wakeup_source\n", __FUNCTION__);
	}
	atomic_set( &shbatt_wakeup_source_num, 0 );

	INIT_WORK(&chip->batt_psy_changed_work, batt_psy_changed_work);
	INIT_WORK(&chip->usb_psy_changed_work, usb_psy_changed_work);
	INIT_WORK(&chip->dc_psy_changed_work, dc_psy_changed_work);

	INIT_DELAYED_WORK(&chip->cap_learning_initialize_work, cap_learning_initialize_work);
	schedule_delayed_work( &chip->cap_learning_initialize_work, msecs_to_jiffies(CAP_LEARNING_INIT_WORK_DELAY) );

	chip->nb.notifier_call = shbatt_notifier_cb;
	rc = power_supply_reg_notifier(&chip->nb);
	if (rc < 0) {
		dev_err(chip->dev, "Failed register notifier_block rc:%d\n", rc);
		return rc;
	}

	//drm notifier cb regiter
	if (chip->active_panel) {
		chip->drm_nb.notifier_call = shbatt_drm_notifier_cb;
		drm_panel_notifier_register(chip->active_panel, &chip->drm_nb);
	}

	if(shbatt_drv_create_device() < 0)
	{
		SHBATT_ERROR("%s : create device failed.\n",__FUNCTION__);
		return -EPERM;
	}

/*
 * If this is not comment out, the debugboard can
 * not boot normally. This is seems related
 * to share memeory
 */
	if (1) {
		shbatt_task_is_initialized = true;
	}

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static int shbatt_drv_remove(
	struct platform_device*		dev_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	if (the_chip->active_panel)
		drm_panel_notifier_unregister(the_chip->active_panel, &the_chip->drm_nb);

	the_chip = NULL;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
	return 0;
}

static void shbatt_drv_shutdown(
	struct platform_device*		dev_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	shbatt_task_is_initialized = false;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static int shbatt_drv_resume(
	struct platform_device*		dev_p
){
	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static int shbatt_drv_suspend(
	struct platform_device*		dev_p,
	pm_message_t				state
){
	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);
	return 0;

}

static int __init shbatt_drv_module_init( void )
{
	SHBATT_TRACE( "[S] %s \n", __FUNCTION__ );

	shbatt_task_workqueue_p = create_singlethread_workqueue("shbatt_task");
	if (!shbatt_task_workqueue_p) {
		SHBATT_ERROR("[E] %s can't create workqueue\n",__FUNCTION__);
		return -ENOMEM;
	}

	mutex_init(&shbatt_task_lock);

	spin_lock_init(&shbatt_pkt_lock);
	create_kobject();
	platform_driver_register( &shbatt_platform_driver );

	/* wake_lock */
	memset( &shbatt_lock_time, 0x00, sizeof(shbatt_lock_time) );
	memset( &shbatt_unlock_time, 0x00, sizeof(shbatt_unlock_time) );
	memset( &shbatt_lock_func, 0x00, sizeof(shbatt_lock_func) );

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static void __exit shbatt_drv_module_exit( void )
{

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	cancel_delayed_work_sync(&the_chip->cap_learning_initialize_work);
	kobject_put( kobj_data.kobj );
	kset_unregister( shbatt_kset );
	platform_driver_unregister(&shbatt_platform_driver);

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);
}

bool is_shbatt_task_initialized( void ) {
	return shbatt_task_is_initialized;
}
EXPORT_SYMBOL(is_shbatt_task_initialized);

module_init(shbatt_drv_module_init);
module_exit(shbatt_drv_module_exit);

MODULE_DESCRIPTION("SH Battery Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :                                                           |*/
/*+-----------------------------------------------------------------------------+*/
