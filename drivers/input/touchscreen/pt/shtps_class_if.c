/* 
 * Copyright (c) 2020, Sharp. All rights reserved.
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
#include <linux/module.h>
#include <linux/cdev.h>
#include "pt_regs.h"

#define SHTPS_CLASS_ATTR_RW(_name) \
	static struct class_attribute shtps_class_attr_##_name = __ATTR(_name, 0660, _name##_show, _name##_store)
#define SHTPS_CLASS_ATTR_RO(_name) \
	static struct class_attribute shtps_class_attr_##_name = __ATTR(_name, 0440, _name##_show, NULL)
#define SHTPS_CLASS_ATTR_WO(_name) \
	static struct class_attribute shtps_class_attr_##_name = __ATTR(_name, 0220, NULL, _name##_store)

struct device *core_dev = NULL;

static int shtps_get_arguments(
	char	*argStr,		/* [I/O] arguments strings (processed in function) */
	char	**argList,		/* [I/O] arguments pointer output buffer */
	int		argListMaxSize	/* [I/ ] arguments list size */
)
{
	int i;
	int argListNum = 0;
	int isParam;

	if((argStr == NULL) || (argList == NULL) || (argListMaxSize < 1)){
		return 0;
	}

	isParam = 0;

	for(i = 0; i < PAGE_SIZE; i++){
		if( argStr[i] == '\0' ){
			if(isParam == 1){
				argListNum++;
			}
			break;
		}
		else if( (argStr[i] == '\n') || (argStr[i] == ',') || (argStr[i] == ' ') ){
			argStr[i] = '\0';
			if(isParam == 1){
				argListNum++;
			}
			isParam = 0;
			if(argListNum >= argListMaxSize){
				break;
			}
			continue;
		}
		else{
			if(isParam == 0){
				isParam = 1;
				argList[argListNum] = &argStr[i];
			}
		}
	}

	return argListNum;
}

static ssize_t intr2_enable_store(struct class *class, struct class_attribute *attr, const char *buf, size_t size)
{
	long		input;
	int			status;

	status = kstrtol(buf, 0, &input);

	if(input == 0) {
		pt_fingerprint_notify_set_enable(core_dev, 0);
	}
	else {
		pt_fingerprint_notify_set_enable(core_dev, 1);
	}

	return status ? : size;
}
SHTPS_CLASS_ATTR_WO(intr2_enable);

static ssize_t intr2_test_node_store(struct class *class, struct class_attribute *attr, const char *buf, size_t size)
{
	long		input;
	int			status;

	status = kstrtol(buf, 0, &input);

	if(input == 0) {
		pt_fingerprint_notify_test(core_dev, 0);
	}
	else {
		pt_fingerprint_notify_test(core_dev, 1);
	}

	return status ? : size;
}
SHTPS_CLASS_ATTR_WO(intr2_test_node);

static ssize_t aoi_enable_show(struct class *class, struct class_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", pt_fingerprint_notify_aoi_get_enable(core_dev));
}
static ssize_t aoi_enable_store(struct class *class, struct class_attribute *attr, const char *buf, size_t size)
{
	long		input;
	int			status;

	status = kstrtol(buf, 0, &input);

	if(input == 0) {
		pt_fingerprint_notify_aoi_set_enable(core_dev, 0);
	}
	else {
		pt_fingerprint_notify_aoi_set_enable(core_dev, 1);
	}

	return status ? : size;
}
SHTPS_CLASS_ATTR_RW(aoi_enable);

static ssize_t aoi_set_show(struct class *class, struct class_attribute *attr, char *buf)
{
	struct pt_fingerprint_aoi_area area;

	pt_fingerprint_notify_aoi_get(core_dev, &area);

	return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n", area.left, area.top, area.right, area.bottom);
}
static ssize_t aoi_set_store(struct class *class, struct class_attribute *attr, const char *buf, size_t size)
{
	struct pt_fingerprint_aoi_area area;
	int ret = 0;
	char *argbuf;
	int argc;
	char *argv[10];
	long input;

	argbuf = (char*)kzalloc(size + 1, GFP_KERNEL);
	if(argbuf == NULL){
		return -ENOMEM;
	}
	memcpy(argbuf, buf, size);

	argc = shtps_get_arguments( argbuf, argv, sizeof(argv)/sizeof(char *) );

	if(argc >= 4) {
		ret = 0;
		ret |= kstrtol(argv[0], 0, &input);		area.left = input;
		ret |= kstrtol(argv[1], 0, &input);		area.top = input;
		ret |= kstrtol(argv[2], 0, &input);		area.right = input;
		ret |= kstrtol(argv[3], 0, &input);		area.bottom = input;
		if(ret == 0) {
			pt_fingerprint_notify_aoi_set(core_dev, &area);
		}
		else {
			pr_err("%s: input parameter error\n", __func__);
		}
	}

	kfree(argbuf);
	if(ret < 0){
		return ret;
	}
	return size;
}
SHTPS_CLASS_ATTR_RW(aoi_set);

/* ---------------------------------------------------------------------------*/
static struct attribute *shtps_class_attrs[] = {
	&shtps_class_attr_intr2_enable.attr,
	&shtps_class_attr_intr2_test_node.attr,
	&shtps_class_attr_aoi_set.attr,
	&shtps_class_attr_aoi_enable.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};
ATTRIBUTE_GROUPS(shtps_class);

static struct class shtps_class = {
	.name =		"shtps",
	.owner =	THIS_MODULE,

	.class_groups = shtps_class_groups,
};

int shtps_class_if_init(struct device *dev)
{
	int		status;

	core_dev = dev;
	status = class_register(&shtps_class);

	return status;
}

void shtps_class_if_exit(void)
{
	core_dev = NULL;
	class_unregister(&shtps_class);
}
