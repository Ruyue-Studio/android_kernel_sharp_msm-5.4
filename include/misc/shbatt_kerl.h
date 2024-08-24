/*
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

#ifndef SHBATT_KERL_H
#define SHBATT_KERL_H

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :                                                            |*/
/*+-----------------------------------------------------------------------------+*/
#ifdef CONFIG_SHARP_SHTERM
#include <misc/shterm_k.h>
#endif /* CONFIG_SHARP_SHTERM */

//#include <misc/shub_driver.h>

/*+-----------------------------------------------------------------------------+*/
/*| @ VALUE DEFINE DECLARE :                                                    |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/

typedef enum shbatt_result_tag
{
	SHBATT_RESULT_SUCCESS,
	SHBATT_RESULT_FAIL,
	SHBATT_RESULT_REJECTED,
	NUM_SHBATT_RESULT

} shbatt_result_t;

typedef enum
{
    SHBATT_REQ_CMD_CAP_LEARNING_WRITE_RESULT,
    SHBATT_REQ_CMD_CAP_LEARNING_WRITE_CALCULATED_DATA,
    SHBATT_REQ_CMD_CAP_LEARNING_WRITE_CYCLE_INFO,
    SHBATT_REQ_CMD_INVALID
} SHBATT_REQ_CMD;

/* Must match the content of ssdev.h */
typedef enum
{
    SSDEV_PARTNER_NONE,
    SSDEV_PARTNER_UNKNOWN,
    SSDEV_PARTNER_SNK_USB_SDP,
    SSDEV_PARTNER_SNK_USB_OCP,
    SSDEV_PARTNER_SNK_USB_CDP,
    SSDEV_PARTNER_SNK_USB_DCP,
    SSDEV_PARTNER_SNK_USB_FLOAT,
    SSDEV_PARTNER_SNK_TYPEC_DEFAULT,
    SSDEV_PARTNER_SNK_TYPEC_RP_MEDIUM_1P5A,
    SSDEV_PARTNER_SNK_TYPEC_RP_HIGH_3A,
    SSDEV_PARTNER_SNK_DEBUG_ACCESS,
    SSDEV_PARTNER_SNK_USB_QC_2P0,
    SSDEV_PARTNER_SNK_USB_QC_3P0,
    SSDEV_PARTNER_SNK_USB_QC_3P5,
    SSDEV_PARTNER_SNK_PD,
    SSDEV_PARTNER_SNK_PPS,
    SSDEV_PARTNER_SRC_TYPEC_POWERCABLE,                 //RD-RA
    SSDEV_PARTNER_SRC_TYPEC_UNORIENTED_DEBUG_ACCESS,    //RD/RD
    SSDEV_PARTNER_SRC_TYPEC_AUDIO_ACCESS,               //RA/RA
    SSDEV_PARTNER_WLS_SRC_BPP,
    SSDEV_PARTNER_WLS_SNK_BPP,
    SSDEV_PARTNER_WLS_SNK_EPP,
    SSDEV_PARTNER_WLS_SNK_PDDE,
    SSDEV_PARTNER_INVALID,
} SSDEV_PARTNER_TYPE;

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :                                                  |*/
/*+-----------------------------------------------------------------------------+*/

typedef struct {
    SHBATT_REQ_CMD  cmd;
    int             data_0;
    int             data_1;
    int             data_2;
} shbatt_uevent_data;

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC FUNCTION PROTO TYPE DECLARE :                                      |*/
/*+-----------------------------------------------------------------------------+*/
#ifdef CONFIG_SHARP_SHTERM
shbatt_result_t shbatt_api_battlog_event(
	shbattlog_event_num			evt);

shbatt_result_t shbatt_api_battlog_charge_status(
	int			status);

shbatt_result_t shbatt_api_battlog_charge_error(
	int			charge_error_event);

shbatt_result_t shbatt_api_battlog_jeita_status(
	int			jeita_cur_status);

shbatt_result_t shbatt_api_battlog_capacity(
	int			cur_capacity);

shbatt_result_t shbatt_api_battlog_usb_type(
	int			usb_type);

shbatt_result_t shbatt_api_battlog_partner_type(
	int			partner_type);

shbatt_result_t shbatt_api_battlog_fv_aged_level(
	int			aged_level);
#endif /* CONFIG_SHARP_SHTERM */

bool is_shbatt_task_initialized( void );

shbatt_result_t shbatt_api_store_fg_cap_learning_result(
	int64_t learned_cc_uah,
	int nom_uah,
	int high_thresh,
	int low_thresh
);
/*+-----------------------------------------------------------------------------+*/
/*| @ PRIVATE FUNCTION PROTO TYPE DECLARE :                                     |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :                                                           |*/
/*+-----------------------------------------------------------------------------+*/

#endif /* SHBATT_KERL_H */
