/* drivers/misc/shbatt/shpwr_log.c
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
/*| @ DEFINE COMPILE SWITCH :													|*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :                                                            |*/
/*+-----------------------------------------------------------------------------+*/

#include <linux/module.h>
#include <linux/kernel.h>
#include "misc/shpwr_log.h"

#include <linux/power_supply.h>


/*+-----------------------------------------------------------------------------+*/
/*| @ VALUE DEFINE DECLARE :                                                    |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ STATIC VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

static int      shpwr_batt_level = SHPWR_LOG_LEVEL_ERR;
module_param( shpwr_batt_level, int, 0664 );

static bool     shpwr_log_is_initialized = false;

static int      durable_shpwr_initialized = -1;
module_param(durable_shpwr_initialized, int, 0644);

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL MACRO DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/

#define SHPWR_ERROR(x...)   SHPWR_LOG(SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,x)
#define SHPWR_INFO(x...)    SHPWR_LOG(SHPWR_LOG_LEVEL_INFO,SHPWR_LOG_TYPE_BATT,x)
#define SHPWR_TRACE(x...)   SHPWR_LOG(SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,x)

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :                                                  |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ EXTERN FUNCTION PROTO TYPE DECLARE :                                      |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION PROTO TYPE DECLARE :                                       |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION'S CODE AREA :                                              |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC FUNCTION'S CODE AREA :                                             |*/
/*+-----------------------------------------------------------------------------+*/

shpwr_log_level shpwr_log_current_level(
    shpwr_log_type  type
) {
    shpwr_log_level ret;

    switch( type )
    {
        case SHPWR_LOG_TYPE_BATT:
            ret = shpwr_batt_level;
            break;
        default:
            ret = SHPWR_LOG_LEVEL_ERR;
            break;
    }
    if( ( ret < SHPWR_LOG_LEVEL_EMERG ) || ( ret > SHPWR_LOG_LEVEL_DEBUG ) )
    {
       ret = SHPWR_LOG_LEVEL_ERR;
    }
    return ret;
}
EXPORT_SYMBOL(shpwr_log_current_level);


bool shpwr_is_initialized(void)
{
    int ret;
    static offchg_mode = 0;
    struct power_supply*    batt_psy = NULL;
    union power_supply_propval  val = {0,};

    batt_psy = power_supply_get_by_name("battery");
    if (!batt_psy) {
        SHPWR_ERROR("%s : batt_psy is null\n", __FUNCTION__);
        goto out;
    }

    ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_OFFCHG_MODE, &val);
    if (!ret) offchg_mode = val.intval;

    if (offchg_mode == 1)
        durable_shpwr_initialized = 1;

out:
    return ((shpwr_log_is_initialized == true) && (durable_shpwr_initialized == 1));
}
EXPORT_SYMBOL(shpwr_is_initialized);


/*+-----------------------------------------------------------------------------+*/
/*| @ PLATFORM DRIVER MODULE CODE AREA :                                        |*/
/*+-----------------------------------------------------------------------------+*/

static int __init shpwr_drv_module_init( void )
{
    SHPWR_TRACE("[%s]in\n", __func__);

    shpwr_log_is_initialized = true;

    SHPWR_TRACE("[%s]out\n", __func__);

    return 0;
}

static void __exit shpwr_drv_module_exit( void )
{
    SHPWR_TRACE("[%s]in\n", __func__);

    SHPWR_TRACE("[%s]out\n", __func__);
}

module_init(shpwr_drv_module_init);
module_exit(shpwr_drv_module_exit);

MODULE_DESCRIPTION("SH Power Log Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
