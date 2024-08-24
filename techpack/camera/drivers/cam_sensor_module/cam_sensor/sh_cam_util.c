/* drivers/sharp/shcamled/sh_cam_util.c
 *
 * Copyright (C) 2020 SHARP CORPORATION
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
#include "cam_sensor_core.h"
#include "cam_sensor_util.h"
#include "cam_soc_util.h"
#include "cam_trace.h"

#include <soc/qcom/sh_smem.h>
//#include <soc/qcom/sharp/sh_boot_manager.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
uint8_t *shcam_CalibData = NULL;
int32_t camCalibData_size = 0;

/* ------------------------------------------------------------------------- */
/* CODE                                                                      */
/* ------------------------------------------------------------------------- */
static int back_mult_camera_caldata_open(struct inode *inode, struct file *filp)
{
	CAM_ERR(CAM_SENSOR, "%s:%d start\n", __func__, __LINE__);
	return 0;
}

static int back_mult_camera_caldata_release(struct inode *inode, struct file *filp)
{
	CAM_ERR(CAM_SENSOR, "%s:%d start\n", __func__, __LINE__);
	return 0;
}

static ssize_t back_mult_camera_caldata_compat_read(struct file *filp, char __user *buf, size_t len, loff_t *ppos)
{
#if 1
	sharp_smem_common_type *p_sh_smem_common_type = NULL;
	
	CAM_ERR(CAM_SENSOR, "%s:%d start\n", __func__, __LINE__);
	
	if(shcam_CalibData == NULL){
		CAM_ERR(CAM_SENSOR, "%s:%d SMEM get", __func__, __LINE__);
		p_sh_smem_common_type = sh_smem_get_common_address();
		if(p_sh_smem_common_type != NULL){
			camCalibData_size = sizeof(p_sh_smem_common_type->sh_camCalibData);
			
			CAM_ERR(CAM_SENSOR, "%s camCalibData_size = %d\n", __func__, camCalibData_size);
			
			shcam_CalibData = kmalloc(camCalibData_size, GFP_KERNEL);
			
			if(shcam_CalibData != NULL){
				memcpy(shcam_CalibData, &p_sh_smem_common_type->sh_camCalibData[0], camCalibData_size);
			} else {
				return 0;
			}
		}
	}
	
	return simple_read_from_buffer(buf, len, ppos, shcam_CalibData, camCalibData_size);
#else
	CAM_ERR(CAM_SENSOR, "%s:%d start, stub\n", __func__, __LINE__);
	return 0;
#endif
}

static ssize_t back_mult_camera_caldata_write(struct file *filp, const char __user *buf, size_t len, loff_t *ppos)
{
#if 1
	CAM_ERR(CAM_SENSOR, "%s:%d start\n", __func__, __LINE__);
	
	CAM_ERR(CAM_SENSOR, "%s:%d len=%d camCalibData_size=%d\n", __func__, __LINE__, len, camCalibData_size);
	
	if(shcam_CalibData == NULL){
		if(len == 4096){
			camCalibData_size = len;
				
			CAM_ERR(CAM_SENSOR, "%s camCalibData_size = %d\n", __func__, camCalibData_size);
			shcam_CalibData = kmalloc(camCalibData_size, GFP_KERNEL);
			
			if(shcam_CalibData != NULL){
				return simple_write_to_buffer(shcam_CalibData, camCalibData_size, ppos, buf, len);
			} else {
				return 0;
			}
		} else {
			return 0;
		}
	} else if(len == camCalibData_size) {
		return simple_write_to_buffer(shcam_CalibData, camCalibData_size, ppos, buf, len);
	} 
	
	return 0;
#else
	CAM_ERR(CAM_SENSOR, "%s:%d start, stub\n", __func__, __LINE__);
	return 0;
#endif
}


static struct file_operations back_mult_camera_caldata_fops = {
	.owner = THIS_MODULE,
	.open = back_mult_camera_caldata_open,
	.release = back_mult_camera_caldata_release,
	.write = back_mult_camera_caldata_write,
	.read = back_mult_camera_caldata_compat_read,
};

dev_t dev = 0;
struct cdev c_dev;
struct class *my_class = NULL;
#if 0
static int32_t tof_hw_revision;
module_param(tof_hw_revision, int, 0444);

static int get_hw_revision()
{
	sharp_smem_common_type *p_sharp_smem_common_type;

	p_sharp_smem_common_type = sh_smem_get_common_address();
	if( p_sharp_smem_common_type != 0 )
	{
		return p_sharp_smem_common_type->sh_hw_revision;
	}else{
		return 0xFF;
	}
	return 0;
}
#endif
int32_t back_mult_camera_caldata_register(void)
{
	int32_t rc = 0;

	CAM_ERR(CAM_SENSOR, "%s:%d start\n", __func__, __LINE__);

	rc = alloc_chrdev_region(&dev, 0, 1, "back_mult_camera_caldata.bin");
	if (rc != 0) {
		pr_err("alloc_chrdev_region = %d\n", rc);
		return -EFAULT;
	}
	cdev_init(&c_dev, &back_mult_camera_caldata_fops);
	c_dev.owner = THIS_MODULE;
	
	rc = cdev_add(&c_dev, dev, 1);
	if (rc != 0) {
		pr_err("cdev_add = %d\n", rc);
		return -EFAULT;
	}
	
	my_class = class_create(THIS_MODULE, "caldata");
	
	if (IS_ERR(my_class)) {
		pr_err("class_create error\n");
		cdev_del(&c_dev);
		unregister_chrdev_region(dev, 1);
		return -EFAULT;
	}
	device_create(my_class, NULL, dev, NULL, "back_mult_camera_caldata.bin");

#if 0
	tof_hw_revision = get_hw_revision();
	CAM_DBG(CAM_SENSOR, "%s:%d hw_rev:%d\n", __func__, __LINE__, tof_hw_revision);
#endif

	CAM_ERR(CAM_SENSOR, "%s:%d end rc:%d\n", __func__, __LINE__, rc);
	return rc;
}

EXPORT_SYMBOL(back_mult_camera_caldata_register);
