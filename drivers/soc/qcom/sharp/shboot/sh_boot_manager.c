/* drivers/soc/qcom/sharp/shboot/sh_boot_manager.c
 *
 * Copyright (C) 2017 Sharp Corporation
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

/*===========================================================================
INCLUDE
===========================================================================*/
#include <linux/module.h>
#include <soc/qcom/sharp/sh_boot_manager.h>

/*===========================================================================
DEFINE
===========================================================================*/

/*===========================================================================
PROTOTYPES
===========================================================================*/
static int sh_boot_get_bootmode_to_user(char *buffer, const struct kernel_param *kp);
static int sh_boot_set_bootmode_from_user(const char *buffer, const struct kernel_param *kp);
static int sh_boot_get_handset_to_user(char *buffer, const struct kernel_param *kp);

int sh_hw_revision = 0;
int sh_boot_mode   = 0;
int sh_hw_handset  = 0;

/*===========================================================================
GLOBAL VARIABLES
===========================================================================*/
static unsigned long boot_mode = 0;
static unsigned char handset = 0;
static struct kernel_param_ops param_ops_bootmode = {
	.get = sh_boot_get_bootmode_to_user,
	.set = sh_boot_set_bootmode_from_user,
};
static struct kernel_param_ops param_ops_handset = {
	.get = sh_boot_get_handset_to_user,
};

/*=============================================================================
FUNCTION
=============================================================================*/
static int __init get_sh_hw_revision(char *str)
{
	get_option(&str, &sh_hw_revision);
	return 1;
}
__setup("androidboot.hardware.revision.number=", get_sh_hw_revision);

static int __init get_sh_boot_mode(char *str)
{
	get_option(&str, &sh_boot_mode);
	return 1;
}
__setup("androidboot.wakeinfo=", get_sh_boot_mode);

static int __init get_sh_hw_handset(char *str)
{
	get_option(&str, &sh_hw_handset);
	return 1;
}
__setup("androidboot.hardware.handset=", get_sh_hw_handset);

unsigned short sh_boot_get_hw_revision(void)
{
	return sh_hw_revision;
}
EXPORT_SYMBOL(sh_boot_get_hw_revision);

unsigned long sh_boot_get_bootmode(void)
{
	return sh_boot_mode;
}
EXPORT_SYMBOL(sh_boot_get_bootmode);

unsigned char sh_boot_get_handset(void)
{
	return sh_hw_handset;
}
EXPORT_SYMBOL(sh_boot_get_handset);

static void sh_boot_set_bootmode(unsigned short mode)
{

}

MODULE_DESCRIPTION("SH Boot Manager");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");

/*=============================================================================

FUNCTION sh_boot_get_bootmode_to_user

DESCRIPTION

DEPENDENCIES
  None

RETURN VALUE

SIDE EFFECTS
  None

NOTE
  None

=============================================================================*/
static int sh_boot_get_bootmode_to_user(char *buffer, const struct kernel_param *kp)
{
	int ret = 0;

	boot_mode = sh_boot_get_bootmode();
	ret = param_get_int(buffer, kp);

	return ret;
}

/*=============================================================================

FUNCTION sh_boot_set_bootmode_from_user

DESCRIPTION

DEPENDENCIES
  None

RETURN VALUE

SIDE EFFECTS
  None

NOTE
  None

=============================================================================*/
static int sh_boot_set_bootmode_from_user(const char *buffer, const struct kernel_param *kp)
{
	int ret = -EINVAL;
	unsigned long mode;

	if (kstrtoul(buffer, 0, &mode) == 0) {
		switch (mode) {
		case SH_BOOT_NORMAL:
			boot_mode = mode;
			sh_boot_set_bootmode(boot_mode);
			ret = 0;
			break;
		default:
			break;
		}
	}

	return ret;
}
module_param_cb(boot_mode, &param_ops_bootmode, &boot_mode, 0644);

/*=============================================================================

FUNCTION sh_boot_get_handset_to_user

DESCRIPTION

DEPENDENCIES
  None

RETURN VALUE

SIDE EFFECTS
  None

NOTE
  None

=============================================================================*/
static int sh_boot_get_handset_to_user(char *buffer, const struct kernel_param *kp)
{
	int ret = 0;

	handset = sh_boot_get_handset();
	ret = param_get_int(buffer, kp);

	return ret;
}
module_param_cb(handset, &param_ops_handset, &handset, 0644);
