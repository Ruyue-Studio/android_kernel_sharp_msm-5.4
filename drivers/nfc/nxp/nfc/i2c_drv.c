/******************************************************************************
 * Copyright (C) 2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013-2022 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ******************************************************************************/
/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include "common_ese.h"

#include <linux/poll.h>
/* SHARP EXTENDED ADD-Start CUST-ID:0103 */
#define SH_CUST_WAKE_LOCK
#ifdef SH_CUST_WAKE_LOCK
#include "nfc_wakelock.h"
#endif /* SH_CUST_WAKE_LOCK */
/* SHARP EXTENDED ADD-End CUST-ID:0103 */

/* SHARP EXTENDED ADD-Start CUST-ID:0103 */
#ifdef SH_CUST_WAKE_LOCK
struct wake_lock sh_nfc_wake_lock;
#define PN553_WAKE_LOCK_NAME	   "pn553-i2c"
/* SHARP EXTENDED MOD-Start CUST-ID:0161 */
/* #define PN553_WAKE_LOCK_TIMEOUT	  1 */	  /* sec */
#define PN553_WAKE_LOCK_TIMEOUT_POLL	1	 /* sec */
#define PN553_WAKE_LOCK_TIMEOUT_IRQ 	2	 /* sec */
/* SHARP EXTENDED MOD-End CUST-ID:0161 */
#endif /* SH_CUST_WAKE_LOCK */
/* SHARP EXTENDED ADD-End CUST-ID:0103 */

/* SHARP EXTENDED MOD-Start CUST-ID:0274 */
#ifdef CONFIG_ANDROID_ENGINEERING
static unsigned char debug_i2c_log = 0;
module_param(debug_i2c_log, byte, 0644);

static void i2c_write_log(const uint8_t *data, uint16_t size)
{
    int len1,len2,ofs;
    char work1[256];
    char work2[8];
    
    if(debug_i2c_log == 0) {
        return;
    }
    
    printk(KERN_INFO "[nfc-i2c]write size=%d\n", size);
    for( len1=0 ; len1<(size/16) ; len1++ ){
        printk(KERN_INFO "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
               *(data+len1*16+0),
               *(data+len1*16+1),
               *(data+len1*16+2),
               *(data+len1*16+3),
               *(data+len1*16+4),
               *(data+len1*16+5),
               *(data+len1*16+6),
               *(data+len1*16+7),
               *(data+len1*16+8),
               *(data+len1*16+9),
               *(data+len1*16+10),
               *(data+len1*16+11),
               *(data+len1*16+12),
               *(data+len1*16+13),
               *(data+len1*16+14),
               *(data+len1*16+15)
               );
    }
    if( size%16 ){
        memset(work1, 0, sizeof(work1));
        ofs=0;
        for(len2=0;len2<(size%16);len2++){
            memset(work2, 0, sizeof(work2));
            sprintf(work2, "%02x ", *(data+len2));
            strcat(work1, work2);
        }
        printk(KERN_INFO "%s", work1);
    }
}

static void i2c_read_log(uint8_t index, uint8_t *data, uint16_t size)
{
    int len1,len2,ofs;
    char work1[512];
    char work2[8];
    
    if(debug_i2c_log == 0) {
        return;
    }
    
    printk(KERN_INFO "[nfc-i2c]read index=%d, size=%d\n", index, size);
    for( len1=0 ; len1<(size/16) ; len1++ ){
        printk(KERN_INFO "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
               *(data+len1*16+0),
               *(data+len1*16+1),
               *(data+len1*16+2),
               *(data+len1*16+3),
               *(data+len1*16+4),
               *(data+len1*16+5),
               *(data+len1*16+6),
               *(data+len1*16+7),
               *(data+len1*16+8),
               *(data+len1*16+9),
               *(data+len1*16+10),
               *(data+len1*16+11),
               *(data+len1*16+12),
               *(data+len1*16+13),
               *(data+len1*16+14),
               *(data+len1*16+15)
               );
    }
    if( size%16 ){
        memset(work1, 0, sizeof(work1));
        ofs=0;
        for(len2=0;len2<(size%16);len2++){
            memset(work2, 0, sizeof(work2));
            sprintf(work2, "%02x ", *(data+len2));
            strcat(work1, work2);
        }
        printk(KERN_INFO "%s", work1);
    }
}
#endif /* CONFIG_ANDROID_ENGINEERING */
/* SHARP EXTENDED ADD-End CUST-ID:0274 */

/**
 * i2c_disable_irq()
 *
 * Check if interrupt is disabled or not
 * and disable interrupt
 *
 * Return: int
 */
int i2c_disable_irq(struct nfc_dev *dev)
{
/* SHARP EXTENDED ADD-Start CUST-ID:0004 */
#ifndef SH_CUST_POLL
/* SHARP EXTENDED ADD-End CUST-ID:0004 */
	unsigned long flags;

	spin_lock_irqsave(&dev->i2c_dev.irq_enabled_lock, flags);
/* SHARP EXTENDED ADD-Start CUST-ID:0004 */
#endif /* SH_CUST_POLL */
/* SHARP EXTENDED ADD-End CUST-ID:0004 */
	if (dev->i2c_dev.irq_enabled) {
		disable_irq_nosync(dev->i2c_dev.client->irq);
		dev->i2c_dev.irq_enabled = false;
	}
/* SHARP EXTENDED ADD-Start CUST-ID:0004 */
#ifndef SH_CUST_POLL
/* SHARP EXTENDED ADD-End CUST-ID:0004 */
	spin_unlock_irqrestore(&dev->i2c_dev.irq_enabled_lock, flags);
/* SHARP EXTENDED ADD-Start CUST-ID:0004 */
#endif /* SH_CUST_POLL */
/* SHARP EXTENDED ADD-End CUST-ID:0004 */

	return 0;
}

/**
 * i2c_enable_irq()
 *
 * Check if interrupt is enabled or not
 * and enable interrupt
 *
 * Return: int
 */
int i2c_enable_irq(struct nfc_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->i2c_dev.irq_enabled_lock, flags);
	if (!dev->i2c_dev.irq_enabled) {
		dev->i2c_dev.irq_enabled = true;
		enable_irq(dev->i2c_dev.client->irq);
	}
	spin_unlock_irqrestore(&dev->i2c_dev.irq_enabled_lock, flags);

	return 0;
}

static irqreturn_t i2c_irq_handler(int irq, void *dev_id)
{
	struct nfc_dev *nfc_dev = dev_id;
/* SHARP EXTENDED DEL-Start CUST-ID:0161 */
//	struct i2c_dev *i2c_dev = &nfc_dev->i2c_dev;
/* SHARP EXTENDED DEL-End CUST-ID:0161 */

/* SHARP EXTENDED ADD-Start CUST-ID:0004 */
#ifdef SH_CUST_POLL
	unsigned long flags;
	spin_lock_irqsave(&nfc_dev->i2c_dev.irq_enabled_lock, flags);
#endif /* SH_CUST_POLL */
/* SHARP EXTENDED ADD-End CUST-ID:0004 */
/* SHARP EXTENDED DEL-Start CUST-ID:0161 */
//	if (device_may_wakeup(&i2c_dev->client->dev))
//		pm_wakeup_event(&i2c_dev->client->dev, WAKEUP_SRC_TIMEOUT);
/* SHARP EXTENDED DEL-End CUST-ID:0161 */

	i2c_disable_irq(nfc_dev);

/* SHARP EXTENDED ADD-Start CUST-ID:0161 */
	if(wake_lock_active(&sh_nfc_wake_lock)) {
		pr_debug("%s: wake_unlock", __func__);
		wake_unlock(&sh_nfc_wake_lock);
	}
	pr_debug("%s: wake_lock_timeout(%d)", __func__, PN553_WAKE_LOCK_TIMEOUT_IRQ);
	wake_lock_timeout(&sh_nfc_wake_lock, PN553_WAKE_LOCK_TIMEOUT_IRQ*HZ);
/* SHARP EXTENDED ADD-End CUST-ID:0161 */

/* SHARP EXTENDED ADD-Start CUST-ID:0004 */
#ifdef SH_CUST_POLL
/* SHARP EXTENDED DEL-Start CUST-ID:0210 */
//	nfc_dev->count_irq++;
/* SHARP EXTENDED DEL-End CUST-ID:0210 */
	spin_unlock_irqrestore(&nfc_dev->i2c_dev.irq_enabled_lock, flags);
#endif /* SH_CUST_POLL */
/* SHARP EXTENDED ADD-End CUST-ID:0004 */
	wake_up(&nfc_dev->read_wq);

	return IRQ_HANDLED;
}

/* SHARP EXTENDED ADD-Start CUST-ID:0004 */
#ifdef SH_CUST_POLL
static unsigned int nfc_i2c_dev_poll(struct file *filp, poll_table *wait)
{
	struct nfc_dev *nfc_dev = (struct nfc_dev *)filp->private_data;
	unsigned int mask = 0;
	unsigned long flags;

/* SHARP EXTENDED MOD-Start CUST-ID:0210 */
	struct platform_gpio *nfc_gpio = &nfc_dev->configs.gpio;
#if 0
	if (nfc_dev->count_irq == 0) {
#else
	if (!gpio_get_value(nfc_gpio->irq)) {
		pr_debug("%s:poll() waiting", __func__);
#endif
/* SHARP EXTENDED MOD-End CUST-ID:0210 */
		i2c_enable_irq(nfc_dev);
		poll_wait(filp, &nfc_dev->read_wq, wait);
	}
/* SHARP EXTENDED DEL-Start CUST-ID:0210 */
//	pr_debug("%s:poll() cnt_irq:%d", __func__ ,nfc_dev->count_irq);
/* SHARP EXTENDED DEL-End CUST-ID:0210 */

	spin_lock_irqsave(&nfc_dev->i2c_dev.irq_enabled_lock, flags);
/* SHARP EXTENDED MOD-Start CUST-ID:0210 */
#if 0
	if (nfc_dev->count_irq > 0)
	{
		nfc_dev->count_irq--;
#else
	if (gpio_get_value(nfc_gpio->irq))
	{
#endif
/* SHARP EXTENDED MOD-End CUST-ID:0210 */
		mask |= POLLIN | POLLRDNORM;
		pr_debug("%s:poll() irq", __func__);
	}

	spin_unlock_irqrestore(&nfc_dev->i2c_dev.irq_enabled_lock, flags);
/* SHARP EXTENDED ADD-Start CUST-ID:0103 */
#ifdef SH_CUST_WAKE_LOCK
	if(mask) {
		if(wake_lock_active(&sh_nfc_wake_lock)) {
			pr_debug("%s: wake_unlock", __func__);
			wake_unlock(&sh_nfc_wake_lock);
		}
/* SHARP EXTENDED MOD-Start CUST-ID:0161 */
/*		  pr_debug("%s: wake_lock_timeout(%d)", __func__, PN553_WAKE_LOCK_TIMEOUT);
		wake_lock_timeout(&sh_nfc_wake_lock, PN553_WAKE_LOCK_TIMEOUT*HZ); */
		pr_debug("%s: wake_lock_timeout(%d)", __func__, PN553_WAKE_LOCK_TIMEOUT_POLL);
		wake_lock_timeout(&sh_nfc_wake_lock, PN553_WAKE_LOCK_TIMEOUT_POLL*HZ);
/* SHARP EXTENDED MOD-End CUST-ID:0161 */
	}
#endif /* SH_CUST_WAKE_LOCK */
/* SHARP EXTENDED ADD-End CUST-ID:0103 */
/* SHARP EXTENDED MOD-Start CUST-ID:0210 */
//	pr_debug("%s:poll() cnt_irq:%d ret:((0x%x)", __func__ ,nfc_dev->count_irq, mask);
	pr_debug("%s:poll() ret:((0x%x)", __func__, mask);
/* SHARP EXTENDED MOD-End CUST-ID:0210 */
	return mask;
}
#endif /* SH_CUST_POLL */
/* SHARP EXTENDED ADD-End CUST-ID:0004 */

int i2c_read(struct nfc_dev *nfc_dev, char *buf, size_t count, int timeout)
{
	int ret;
/* SHARP EXTENDED ADD-Start CUST-ID:0004 */
#ifdef SH_CUST_POLL
	unsigned long flags;
#endif /* SH_CUST_POLL */
/* SHARP EXTENDED ADD-End CUST-ID:0004 */
	struct i2c_dev *i2c_dev = &nfc_dev->i2c_dev;
	struct platform_gpio *nfc_gpio = &nfc_dev->configs.gpio;

	pr_debug("%s: reading %zu bytes.\n", __func__, count);

	if (timeout > NCI_CMD_RSP_TIMEOUT_MS)
		timeout = NCI_CMD_RSP_TIMEOUT_MS;

	if (count > MAX_NCI_BUFFER_SIZE)
		count = MAX_NCI_BUFFER_SIZE;

	if (!gpio_get_value(nfc_gpio->irq)) {
		while (1) {
			ret = 0;
			if (!i2c_dev->irq_enabled) {
				i2c_dev->irq_enabled = true;
				enable_irq(i2c_dev->client->irq);
			}
			if (!gpio_get_value(nfc_gpio->irq)) {
				if (timeout) {
					ret = wait_event_interruptible_timeout(
						nfc_dev->read_wq,
						!i2c_dev->irq_enabled,
						msecs_to_jiffies(timeout));
					if (ret <= 0) {
						pr_err("%s: timeout error\n",
						       __func__);
						goto err;
					}
				} else {
					ret = wait_event_interruptible(
						nfc_dev->read_wq,
						!i2c_dev->irq_enabled);
					if (ret) {
						pr_err("%s: err wakeup of wq\n",
						       __func__);
						goto err;
					}
				}
			}
/* SHARP EXTENDED ADD-Start CUST-ID:0004 */
#ifdef SH_CUST_POLL
			spin_lock_irqsave(&nfc_dev->i2c_dev.irq_enabled_lock, flags);
#endif /* SH_CUST_POLL */
/* SHARP EXTENDED ADD-End CUST-ID:0004 */
			i2c_disable_irq(nfc_dev);

/* SHARP EXTENDED ADD-Start CUST-ID:0004 */
#ifdef SH_CUST_POLL
			spin_unlock_irqrestore(&nfc_dev->i2c_dev.irq_enabled_lock, flags);
#endif /* SH_CUST_POLL */
/* SHARP EXTENDED ADD-End CUST-ID:0004 */
			if (gpio_get_value(nfc_gpio->irq))
				break;
			if (!gpio_get_value(nfc_gpio->ven)) {
				pr_info("%s: releasing read\n", __func__);
				ret = -EIO;
				goto err;
			}
			/*
			 * NFC service wanted to close the driver so,
			 * release the calling reader thread asap.
			 *
			 * This can happen in case of nfc node close call from
			 * eSE HAL in that case the NFC HAL reader thread
			 * will again call read system call
			 */
			if (nfc_dev->release_read) {
				pr_debug("%s: releasing read\n", __func__);
				return 0;
			}
			pr_warn("%s: spurious interrupt detected\n", __func__);
		}
	}

	memset(buf, 0x00, count);
	/* Read data */
	ret = i2c_master_recv(nfc_dev->i2c_dev.client, buf, count);
	if (ret <= 0) {
		pr_err("%s: returned %d\n", __func__, ret);
		goto err;
	}
/* SHARP EXTENDED MOD-Start CUST-ID:0274 */
#ifdef CONFIG_ANDROID_ENGINEERING
	i2c_read_log(1, buf, count);
#endif
/* SHARP EXTENDED ADD-End CUST-ID:0274 */
/* SHARP EXTENDED ADD-Start CUST-ID:0103 */
#ifdef SH_CUST_WAKE_LOCK
	if(wake_lock_active(&sh_nfc_wake_lock)) {
		pr_debug("%s: wake_unlock", __func__);
		wake_unlock(&sh_nfc_wake_lock);
	}
#endif /* SH_CUST_WAKE_LOCK */
/* SHARP EXTENDED ADD-End CUST-ID:0103 */
	/* check if it's response of cold reset command
	 * NFC HAL process shouldn't receive this data as
	 * command was sent by driver
	 */
	if (nfc_dev->cold_reset.rsp_pending) {
		if (IS_PROP_CMD_RSP(buf)) {
			/* Read data */
			ret = i2c_master_recv(nfc_dev->i2c_dev.client,
					      &buf[NCI_PAYLOAD_IDX],
					      buf[NCI_PAYLOAD_LEN_IDX]);
			if (ret <= 0) {
				pr_err("%s: error reading cold rst/prot rsp\n",
				       __func__);
				goto err;
			}
/* SHARP EXTENDED MOD-Start CUST-ID:0274 */
#ifdef CONFIG_ANDROID_ENGINEERING
			i2c_read_log(2, &buf[NCI_PAYLOAD_IDX], buf[NCI_PAYLOAD_LEN_IDX]);
#endif
/* SHARP EXTENDED ADD-End CUST-ID:0274 */
			wakeup_on_prop_rsp(nfc_dev, buf);
			/*
			 * NFC process doesn't know about cold reset command
			 * being sent as it was initiated by eSE process
			 * we shouldn't return any data to NFC process
			 */
			return 0;
		}
	}
err:
	return ret;
}

int i2c_write(struct nfc_dev *nfc_dev, const char *buf, size_t count,
	      int max_retry_cnt)
{
	int ret = -EINVAL;
	int retry_cnt;
	struct platform_gpio *nfc_gpio = &nfc_dev->configs.gpio;

	if (count > MAX_DL_BUFFER_SIZE)
		count = MAX_DL_BUFFER_SIZE;

	pr_debug("%s: writing %zu bytes.\n", __func__, count);
	/*
	 * Wait for any pending read for max 15ms before write
	 * This is to avoid any packet corruption during read, when
	 * the host cmds resets NFCC during any parallel read operation
	 */
	for (retry_cnt = 1; retry_cnt <= MAX_WRITE_IRQ_COUNT; retry_cnt++) {
		if (gpio_get_value(nfc_gpio->irq)) {
			pr_warn("%s: irq high during write, wait\n", __func__);
			usleep_range(WRITE_RETRY_WAIT_TIME_US,
				     WRITE_RETRY_WAIT_TIME_US + 100);
		} else {
			break;
		}
		if (retry_cnt == MAX_WRITE_IRQ_COUNT &&
		    gpio_get_value(nfc_gpio->irq)) {
			pr_warn("%s: allow after maximum wait\n", __func__);
		}
	}

	for (retry_cnt = 1; retry_cnt <= max_retry_cnt; retry_cnt++) {
/* SHARP EXTENDED MOD-Start CUST-ID:0274 */
#ifdef CONFIG_ANDROID_ENGINEERING
		i2c_write_log(buf, count);
#endif
/* SHARP EXTENDED ADD-End CUST-ID:0274 */
		ret = i2c_master_send(nfc_dev->i2c_dev.client, buf, count);
		if (ret <= 0) {
			pr_warn("%s: write failed ret(%d), maybe in standby\n",
				__func__, ret);
			usleep_range(WRITE_RETRY_WAIT_TIME_US,
				     WRITE_RETRY_WAIT_TIME_US + 100);
		} else if (ret != count) {
			pr_err("%s: failed to write %d\n", __func__, ret);
			ret = -EIO;
		} else if (ret == count)
			break;
	}
	return ret;
}

ssize_t nfc_i2c_dev_read(struct file *filp, char __user *buf, size_t count,
			 loff_t *offset)
{
	int ret;
	struct nfc_dev *nfc_dev = (struct nfc_dev *)filp->private_data;

	if (!nfc_dev) {
		pr_err("%s: device doesn't exist anymore\n", __func__);
		return -ENODEV;
	}
	mutex_lock(&nfc_dev->read_mutex);
	if (filp->f_flags & O_NONBLOCK) {
		ret = i2c_master_recv(nfc_dev->i2c_dev.client, nfc_dev->read_kbuf, count);
		pr_debug("%s: NONBLOCK read ret = %d\n", __func__, ret);
/* SHARP EXTENDED MOD-Start CUST-ID:0274 */
#ifdef CONFIG_ANDROID_ENGINEERING
		i2c_read_log(3, nfc_dev->read_kbuf, count);
#endif
/* SHARP EXTENDED ADD-End CUST-ID:0274 */
	} else {
		ret = i2c_read(nfc_dev, nfc_dev->read_kbuf, count, 0);
	}
	if (ret > 0) {
		if (copy_to_user(buf, nfc_dev->read_kbuf, ret)) {
			pr_warn("%s: failed to copy to user space\n", __func__);
			ret = -EFAULT;
		}
	}
	mutex_unlock(&nfc_dev->read_mutex);
	return ret;
}

ssize_t nfc_i2c_dev_write(struct file *filp, const char __user *buf,
			  size_t count, loff_t *offset)
{
	int ret;
	struct nfc_dev *nfc_dev = (struct nfc_dev *)filp->private_data;

	if (count > MAX_DL_BUFFER_SIZE)
		count = MAX_DL_BUFFER_SIZE;

	if (!nfc_dev) {
		pr_err("%s: device doesn't exist anymore\n", __func__);
		return -ENODEV;
	}

	mutex_lock(&nfc_dev->write_mutex);
	if (copy_from_user(nfc_dev->write_kbuf, buf, count)) {
		pr_err("%s: failed to copy from user space\n", __func__);
		mutex_unlock(&nfc_dev->write_mutex);
		return -EFAULT;
	}
/* SHARP EXTENDED Mod-Start CUST-ID:0010 */
/*	ret = i2c_write(nfc_dev, nfc_dev->write_kbuf, count, NO_RETRY); */
	ret = i2c_write(nfc_dev, nfc_dev->write_kbuf, count, MAX_RETRY_COUNT);
/* SHARP EXTENDED Mod-End CUST-ID:0010 */
	mutex_unlock(&nfc_dev->write_mutex);
	return ret;
}

static const struct file_operations nfc_i2c_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = nfc_i2c_dev_read,
	.write = nfc_i2c_dev_write,
	.open = nfc_dev_open,
	.flush = nfc_dev_flush,
	.release = nfc_dev_close,
	.unlocked_ioctl = nfc_dev_ioctl,
/* SHARP EXTENDED ADD-Start CUST-ID:0004 */
#ifdef SH_CUST_POLL
	.poll = nfc_i2c_dev_poll,
#endif /* SH_CUST_POLL */
/* SHARP EXTENDED ADD-End CUST-ID:0004 */
#ifdef CONFIG_COMPAT
	.compat_ioctl = nfc_dev_compat_ioctl,
#endif
};

int nfc_i2c_dev_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct nfc_dev *nfc_dev = NULL;
	struct i2c_dev *i2c_dev = NULL;
	struct platform_configs *nfc_configs = NULL;
	struct platform_gpio *nfc_gpio = NULL;

/* SHARP EXTENDED ADD-Start CUST-ID:0004 */
#ifdef SH_CUST_POLL
	unsigned long flags;
#endif /* SH_CUST_POLL */
/* SHARP EXTENDED ADD-End CUST-ID:0004 */
	pr_debug("%s: enter\n", __func__);
	nfc_dev = kzalloc(sizeof(struct nfc_dev), GFP_KERNEL);
	if (nfc_dev == NULL) {
		ret = -ENOMEM;
		goto err;
	}
	nfc_configs = &nfc_dev->configs;
	nfc_gpio = &nfc_configs->gpio;
	/* retrieve details of gpios from dt */
	ret = nfc_parse_dt(&client->dev,nfc_configs, PLATFORM_IF_I2C);
	if (ret) {
		pr_err("%s: failed to parse dt\n", __func__);
		goto err_free_nfc_dev;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_free_nfc_dev;
	}
	nfc_dev->read_kbuf = kzalloc(MAX_NCI_BUFFER_SIZE, GFP_DMA | GFP_KERNEL);
	if (!nfc_dev->read_kbuf) {
		ret = -ENOMEM;
		goto err_free_nfc_dev;
	}
	nfc_dev->write_kbuf = kzalloc(MAX_DL_BUFFER_SIZE, GFP_DMA | GFP_KERNEL);
	if (!nfc_dev->write_kbuf) {
		ret = -ENOMEM;
		goto err_free_read_kbuf;
	}
	nfc_dev->interface = PLATFORM_IF_I2C;
	nfc_dev->nfc_state = NFC_STATE_NCI;
	nfc_dev->i2c_dev.client = client;
	i2c_dev = &nfc_dev->i2c_dev;
	nfc_dev->nfc_read = i2c_read;
	nfc_dev->nfc_write = i2c_write;
	nfc_dev->nfc_enable_intr = i2c_enable_irq;
	nfc_dev->nfc_disable_intr = i2c_disable_irq;
	ret = configure_gpio(nfc_gpio->ven, GPIO_OUTPUT);
	if (ret) {
		pr_err("%s: unable to request nfc reset gpio [%d]\n", __func__,
		       nfc_gpio->ven);
		goto err_free_write_kbuf;
	}
	ret = configure_gpio(nfc_gpio->irq, GPIO_IRQ);
	if (ret <= 0) {
		pr_err("%s: unable to request nfc irq gpio [%d]\n", __func__,
		       nfc_gpio->irq);
		goto err_free_gpio;
	}
	client->irq = ret;
	ret = configure_gpio(nfc_gpio->dwl_req, GPIO_OUTPUT);
	if (ret) {
		pr_err("%s: unable to request nfc firm downl gpio [%d]\n",
		       __func__, nfc_gpio->dwl_req);
	}
	/* init mutex and queues */
	init_waitqueue_head(&nfc_dev->read_wq);
	mutex_init(&nfc_dev->read_mutex);
	mutex_init(&nfc_dev->write_mutex);
	mutex_init(&nfc_dev->dev_ref_mutex);
	spin_lock_init(&i2c_dev->irq_enabled_lock);
	common_ese_init(nfc_dev);
	ret = nfc_misc_register(nfc_dev, &nfc_i2c_dev_fops, DEV_COUNT,
				NFC_CHAR_DEV_NAME, CLASS_NAME);
	if (ret) {
		pr_err("%s: nfc_misc_register failed\n", __func__);
		goto err_mutex_destroy;
	}
/* SHARP EXTENDED ADD-Start CUST-ID:0103 */
#ifdef SH_CUST_WAKE_LOCK
	wake_lock_init(&sh_nfc_wake_lock, WAKE_LOCK_SUSPEND, PN553_WAKE_LOCK_NAME);
#endif /* SH_CUST_WAKE_LOCK */
/* SHARP EXTENDED ADD-End CUST-ID:0103 */
	/* interrupt initializations */
	pr_info("%s: requesting IRQ %d\n", __func__, client->irq);
	i2c_dev->irq_enabled = true;
	ret = request_irq(client->irq, i2c_irq_handler, IRQF_TRIGGER_HIGH,
			  client->name, nfc_dev);
	if (ret) {
		pr_err("%s: request_irq failed\n", __func__);
		goto err_nfc_misc_unregister;
	}
/* SHARP EXTENDED MOD-Start CUST-ID:0004 */
#ifdef SH_CUST_POLL
	spin_lock_irqsave(&nfc_dev->i2c_dev.irq_enabled_lock, flags);
	i2c_disable_irq(nfc_dev);
	spin_unlock_irqrestore(&nfc_dev->i2c_dev.irq_enabled_lock, flags);
#else /* SH_CUST_POLL */
	i2c_disable_irq(nfc_dev);
#endif /* SH_CUST_POLL */
/* SHARP EXTENDED MOD-End CUST-ID:0004 */
	gpio_set_ven(nfc_dev, 1);
	gpio_set_ven(nfc_dev, 0);
	gpio_set_ven(nfc_dev, 1);
	device_init_wakeup(&client->dev, true);
	i2c_set_clientdata(client, nfc_dev);
	i2c_dev->irq_wake_up = false;

	pr_info("%s: probing nfc i2c successfully\n", __func__);
	return 0;
err_nfc_misc_unregister:
	nfc_misc_unregister(nfc_dev, DEV_COUNT);
err_mutex_destroy:
	mutex_destroy(&nfc_dev->dev_ref_mutex);
	mutex_destroy(&nfc_dev->read_mutex);
	mutex_destroy(&nfc_dev->write_mutex);
err_free_gpio:
	gpio_free_all(nfc_dev);
err_free_write_kbuf:
	kfree(nfc_dev->write_kbuf);
err_free_read_kbuf:
	kfree(nfc_dev->read_kbuf);
err_free_nfc_dev:
	kfree(nfc_dev);
err:
	pr_err("%s: probing not successful, check hardware\n", __func__);
	return ret;
}

int nfc_i2c_dev_remove(struct i2c_client *client)
{
	int ret = 0;
	struct nfc_dev *nfc_dev = NULL;

	pr_info("%s: remove device\n", __func__);
	nfc_dev = i2c_get_clientdata(client);
	if (!nfc_dev) {
		pr_err("%s: device doesn't exist anymore\n", __func__);
		ret = -ENODEV;
		return ret;
	}
	if (nfc_dev->dev_ref_count > 0) {
		pr_err("%s: device already in use\n", __func__);
		return -EBUSY;
	}
/* SHARP EXTENDED ADD-Start CUST-ID:0103 */
#ifdef SH_CUST_WAKE_LOCK
	wake_lock_destroy(&sh_nfc_wake_lock);
#endif /* SH_CUST_WAKE_LOCK */
/* SHARP EXTENDED ADD-End CUST-ID:0103 */
	device_init_wakeup(&client->dev, false);
	free_irq(client->irq, nfc_dev);
	nfc_misc_unregister(nfc_dev, DEV_COUNT);
	mutex_destroy(&nfc_dev->read_mutex);
	mutex_destroy(&nfc_dev->write_mutex);
	gpio_free_all(nfc_dev);
	kfree(nfc_dev->read_kbuf);
	kfree(nfc_dev->write_kbuf);
	kfree(nfc_dev);
	return ret;
}

int nfc_i2c_dev_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct nfc_dev *nfc_dev = i2c_get_clientdata(client);
	struct i2c_dev *i2c_dev = NULL;
	if (!nfc_dev) {
		pr_err("%s: device doesn't exist anymore\n", __func__);
		return -ENODEV;
	}
	i2c_dev = &nfc_dev->i2c_dev;

	if (device_may_wakeup(&client->dev) && i2c_dev->irq_enabled) {
		if (!enable_irq_wake(client->irq))
			i2c_dev->irq_wake_up = true;
	}
	pr_debug("%s: irq_wake_up = %d", __func__, i2c_dev->irq_wake_up);
	return 0;
}

int nfc_i2c_dev_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct nfc_dev *nfc_dev = i2c_get_clientdata(client);
	struct i2c_dev *i2c_dev = NULL;
	if (!nfc_dev) {
		pr_err("%s: device doesn't exist anymore\n", __func__);
		return -ENODEV;
	}
	i2c_dev = &nfc_dev->i2c_dev;

	if (device_may_wakeup(&client->dev) && i2c_dev->irq_wake_up) {
		if (!disable_irq_wake(client->irq))
			i2c_dev->irq_wake_up = false;
	}
	pr_debug("%s: irq_wake_up = %d", __func__, i2c_dev->irq_wake_up);
	return 0;
}

static const struct i2c_device_id nfc_i2c_dev_id[] = { { NFC_I2C_DEV_ID, 0 },
						       {} };

static const struct of_device_id nfc_i2c_dev_match_table[] = {
	{
		.compatible = NFC_I2C_DRV_STR,
	},
	{}
};

static const struct dev_pm_ops nfc_i2c_dev_pm_ops = { SET_SYSTEM_SLEEP_PM_OPS(
	nfc_i2c_dev_suspend, nfc_i2c_dev_resume) };

static struct i2c_driver nfc_i2c_dev_driver = {
	.id_table = nfc_i2c_dev_id,
	.probe = nfc_i2c_dev_probe,
	.remove = nfc_i2c_dev_remove,
	.driver = {
		.name = NFC_I2C_DRV_STR,
		.pm = &nfc_i2c_dev_pm_ops,
		.of_match_table = nfc_i2c_dev_match_table,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};

MODULE_DEVICE_TABLE(of, nfc_i2c_dev_match_table);

static int __init nfc_i2c_dev_init(void)
{
	int ret = 0;

	pr_info("%s: Loading NXP NFC I2C driver\n", __func__);
	ret = i2c_add_driver(&nfc_i2c_dev_driver);
	if (ret != 0)
		pr_err("%s: NFC I2C add driver error ret %d\n", __func__, ret);
	return ret;
}

module_init(nfc_i2c_dev_init);

static void __exit nfc_i2c_dev_exit(void)
{
	pr_info("%s: Unloading NXP NFC I2C driver\n", __func__);
	i2c_del_driver(&nfc_i2c_dev_driver);
}

module_exit(nfc_i2c_dev_exit);

MODULE_DESCRIPTION("NXP NFC I2C driver");
MODULE_LICENSE("GPL");
