/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

#include <linux/vmalloc.h>
#include "inc/gyroscope.h"
#include "sensor_performance.h"

#define GYRO_MIN_POLL_DELAY_NS	5000000LL
#define GYRO_MAX_POLL_DELAY_NS	160000000LL

struct gyro_context *gyro_context_obj/* = NULL*/;
static struct platform_device *pltfm_dev;

static struct gyro_init_info *gyroscope_init_list[MAX_CHOOSE_GYRO_NUM] = {0};

static int64_t getCurNS(void)
{
	int64_t ns;
	struct timespec time;

	time.tv_sec = time.tv_nsec = 0;
	get_monotonic_boottime(&time);
	ns = time.tv_sec * 1000000000LL + time.tv_nsec;

	return ns;
}

static void initTimer(struct hrtimer *timer, enum hrtimer_restart (*callback)(struct hrtimer *))
{
	hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	timer->function = callback;
}

static void startTimer(struct hrtimer *timer, int delay_ms, bool first)
{
	struct gyro_context *obj = (struct gyro_context *)container_of(timer, struct gyro_context, hrTimer);
	static int count;

	if (obj == NULL) {
		GYRO_PR_ERR("NULL pointer\n");
		return;
	}

	if (first) {
		obj->target_ktime = ktime_add_ns(ktime_get(), (int64_t)delay_ms*1000000);
#if 0
		GYRO_LOG("%d, cur_nt = %lld, delay_ms = %d, target_nt = %lld\n", count,
			getCurNT(), delay_ms, ktime_to_us(obj->target_ktime));
#endif
		count = 0;
	} else {
		do {
			obj->target_ktime = ktime_add_ns(obj->target_ktime, (int64_t)delay_ms*1000000);
		} while (ktime_to_ns(obj->target_ktime) < ktime_to_ns(ktime_get()));
#if 0
		GYRO_LOG("%d, cur_nt = %lld, delay_ms = %d, target_nt = %lld\n", count,
			getCurNT(), delay_ms, ktime_to_us(obj->target_ktime));
#endif
		count++;
	}

	hrtimer_start(timer, obj->target_ktime, HRTIMER_MODE_ABS);
}

#ifndef CONFIG_NANOHUB
static void stopTimer(struct hrtimer *timer)
{
	hrtimer_cancel(timer);
}
#endif
static void gyro_work_func(struct work_struct *work)
{

	struct gyro_context *cxt = NULL;
	int x = 0, y = 0, z = 0, status = 0;
	int err = 0;

	cxt  = gyro_context_obj;

    /* add wake lock to make sure data can be read before system suspend */
	if (cxt->gyro_data.get_data != NULL)
		err = cxt->gyro_data.get_data(&x, &y, &z, &status);
	else {
		GYRO_PR_ERR("gyro driver not register data path\n");
		err = -ENODEV;
	}

	if (err) {
		GYRO_PR_ERR("get gyro data fails!!\n");
		goto gyro_loop;
	} else {
		cxt->drv_data.x = x;
		cxt->drv_data.y = y;
		cxt->drv_data.z = z;
		cxt->drv_data.status = status;
		cxt->drv_data.timestamp = getCurNS();
	 }

	if (true ==  cxt->is_first_data_after_enable) {
		cxt->is_first_data_after_enable = false;
		/* filter -1 value */
		if (cxt->drv_data.x == GYRO_INVALID_VALUE ||
		    cxt->drv_data.y == GYRO_INVALID_VALUE ||
		    cxt->drv_data.z == GYRO_INVALID_VALUE) {
			GYRO_LOG(" read invalid data\n");
			goto gyro_loop;
		}
	}

	/* GYRO_LOG("gyro data[%d,%d,%d]\n" ,cxt->drv_data.gyro_data.values[0], */
	/* cxt->drv_data.gyro_data.values[1],cxt->drv_data.gyro_data.values[2]); */

	/* One physical acquisition produces exactly one boottime-stamped event. */
	gyro_data_report(&cxt->drv_data);

gyro_loop:
	if (READ_ONCE(cxt->is_polling_run))
		startTimer(&cxt->hrTimer, atomic_read(&cxt->delay), false);
}

enum hrtimer_restart gyro_poll(struct hrtimer *timer)
{
	struct gyro_context *obj = (struct gyro_context *)container_of(timer, struct gyro_context, hrTimer);

	if (READ_ONCE(obj->is_polling_run))
		queue_work(obj->gyro_workqueue, &obj->report);

	/* GYRO_LOG("cur_nt = %lld\n", getCurNT()); */

	return HRTIMER_NORESTART;
}

static struct gyro_context *gyro_context_alloc_object(void)
{

	struct gyro_context *obj = kzalloc(sizeof(*obj), GFP_KERNEL);

	GYRO_LOG("gyro_context_alloc_object++++\n");
	if (!obj) {
		GYRO_PR_ERR("Alloc gyro object error!\n");
		return NULL;
	}
	atomic_set(&obj->delay, 200); /*5Hz,  set work queue delay time 200ms */
	atomic_set(&obj->wake, 0);
	INIT_WORK(&obj->report, gyro_work_func);
	obj->gyro_workqueue = NULL;
	obj->gyro_workqueue = create_workqueue("gyro_polling");
	if (!obj->gyro_workqueue) {
		kfree(obj);
		return NULL;
	}
	initTimer(&obj->hrTimer, gyro_poll);
	obj->is_active_nodata = false;
	obj->is_active_data = false;
	obj->is_first_data_after_enable = false;
	obj->is_polling_run = false;
	obj->is_batch_enable = false;
	obj->cali_sw[GYRO_AXIS_X] = 0;
	obj->cali_sw[GYRO_AXIS_Y] = 0;
	obj->cali_sw[GYRO_AXIS_Z] = 0;
	obj->power = 0;
	obj->enable = 0;
	obj->delay_ns = -1;
	obj->latency_ns = -1;
	mutex_init(&obj->gyro_op_mutex);
	GYRO_LOG("gyro_context_alloc_object----\n");
	return obj;
}

#ifndef CONFIG_NANOHUB
static int gyro_enable_and_batch(void)
{
	struct gyro_context *cxt = gyro_context_obj;
	int err;

	/* Data reporting may stop while a no-data client keeps the chip powered. */
	if (!cxt->is_active_data &&
	    !cxt->gyro_ctl.is_report_input_direct &&
	    READ_ONCE(cxt->is_polling_run)) {
		WRITE_ONCE(cxt->is_polling_run, false);
		stopTimer(&cxt->hrTimer);
		cancel_work_sync(&cxt->report);
		stopTimer(&cxt->hrTimer);
		cxt->drv_data.x = GYRO_INVALID_VALUE;
		cxt->drv_data.y = GYRO_INVALID_VALUE;
		cxt->drv_data.z = GYRO_INVALID_VALUE;
		GYRO_LOG("gyro stop polling done\n");
	}

	/* power on -> power off */
	if (cxt->power == 1 && cxt->enable == 0) {
		GYRO_LOG("GYRO disable\n");
		/* turn off the power */
		if (cxt->is_active_data == false && cxt->is_active_nodata == false) {
			err = cxt->gyro_ctl.enable_nodata(0);
			if (err) {
				GYRO_PR_ERR("gyro turn off power err = %d\n", err);
				return err;
			}
			GYRO_LOG("gyro turn off power done\n");
		}

		cxt->power = 0;
		cxt->delay_ns = -1;
		GYRO_LOG("GYRO disable done\n");
		return 0;
	}
	/* power off -> power on */
	if (cxt->power == 0 && cxt->enable == 1) {
		GYRO_LOG("GYRO enable\n");
		if (true == cxt->is_active_data || true == cxt->is_active_nodata) {
			err = cxt->gyro_ctl.enable_nodata(1);
			if (err) {
				GYRO_PR_ERR("gyro turn on power err = %d\n", err);
				return err;
			}
			GYRO_LOG("gyro turn on power done\n");
		}
		cxt->power = 1;
		GYRO_LOG("GYRO enable done\n");
	}
	/* rate change */
	if (cxt->power == 1 && cxt->delay_ns >= 0) {
		GYRO_LOG("GYRO set batch\n");
		/* set ODR, fifo timeout latency */
		if (cxt->gyro_ctl.is_support_batch)
			err = cxt->gyro_ctl.batch(0, cxt->delay_ns, cxt->latency_ns);
		else
			err = cxt->gyro_ctl.batch(0, cxt->delay_ns, 0);
		if (err) {
			GYRO_PR_ERR("gyro set batch(ODR) err %d\n", err);
			return err;
		}
		GYRO_LOG("gyro set ODR, fifo latency done\n");
		/* start polling, if needed */
		if (cxt->is_active_data == true
			&& cxt->gyro_ctl.is_report_input_direct == false) {
			int64_t mdelay = cxt->delay_ns / 1000000LL;

			if (mdelay < 1)
				mdelay = 1;
			if (mdelay > INT_MAX)
				mdelay = INT_MAX;
			atomic_set(&cxt->delay, (int)mdelay);
			/* the first sensor start polling timer */
			if (!READ_ONCE(cxt->is_polling_run)) {
				cxt->is_first_data_after_enable = true;
				WRITE_ONCE(cxt->is_polling_run, true);
				startTimer(&cxt->hrTimer, atomic_read(&cxt->delay), true);
			}
			GYRO_LOG("gyro set polling delay %d ms\n", atomic_read(&cxt->delay));
		}
		GYRO_LOG("GYRO batch done\n");
	}
	/* just for debug, remove it when everything is ok */
	if (cxt->power == 0 && cxt->delay_ns >= 0)
		GYRO_INFO("batch will call firstly in API1.3, do nothing\n");

	return 0;
}
#endif
static ssize_t gyro_show_enable_nodata(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int len = 0;

	GYRO_LOG(" not support now\n");
	return len;
}

static ssize_t gyro_store_enable_nodata(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct gyro_context *cxt = gyro_context_obj;
	int err = 0;
	int old_enable;
	bool old_active_nodata;
#ifndef CONFIG_NANOHUB
	int old_power;
	int64_t old_delay_ns;
	bool old_polling;
#endif

	GYRO_LOG("gyro_store_enable nodata buf=%s\n", buf);
	mutex_lock(&gyro_context_obj->gyro_op_mutex);
	old_enable = cxt->enable;
	old_active_nodata = cxt->is_active_nodata;
#ifndef CONFIG_NANOHUB
	old_power = cxt->power;
	old_delay_ns = cxt->delay_ns;
	old_polling = READ_ONCE(cxt->is_polling_run);
#endif
	if (!strncmp(buf, "1", 1)) {
		cxt->is_active_nodata = true;
	} else if (!strncmp(buf, "0", 1)) {
		cxt->is_active_nodata = false;
	} else {
		GYRO_INFO(" gyro_store enable nodata cmd error !!\n");
		err = -EINVAL;
		goto err_out;
	}
	cxt->enable = cxt->is_active_data || cxt->is_active_nodata;
#ifdef CONFIG_NANOHUB
		if (true == cxt->is_active_data || true == cxt->is_active_nodata) {
			err = cxt->gyro_ctl.enable_nodata(1);
			if (err) {
				GYRO_PR_ERR("gyro turn on power err = %d\n", err);
				goto restore_state;
			}
			GYRO_LOG("gyro turn on power done\n");
		} else {
			err = cxt->gyro_ctl.enable_nodata(0);
			if (err) {
				GYRO_PR_ERR("gyro turn off power err = %d\n", err);
				goto restore_state;
			}
			GYRO_LOG("gyro turn off power done\n");
		}
#else
	err = gyro_enable_and_batch();
#endif
#ifdef CONFIG_NANOHUB
restore_state:
#endif
	if (err) {
		cxt->enable = old_enable;
		cxt->is_active_nodata = old_active_nodata;
#ifndef CONFIG_NANOHUB
		cxt->delay_ns = old_delay_ns;
		if (cxt->power != old_power ||
		    READ_ONCE(cxt->is_polling_run) != old_polling) {
			int rollback_err = gyro_enable_and_batch();

			/* A power-off rollback clears delay_ns; keep the staged batch. */
			cxt->delay_ns = old_delay_ns;
			if (rollback_err < 0)
				GYRO_PR_ERR("failed to restore gyro state: %d\n",
					    rollback_err);
		}
#endif
	}

err_out:
	mutex_unlock(&gyro_context_obj->gyro_op_mutex);
	return err ? (err < 0 ? err : -EIO) : count;
}

static ssize_t gyro_store_active(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct gyro_context *cxt = gyro_context_obj;
	int err = 0;
	int old_enable;
	bool old_active_data;
#ifndef CONFIG_NANOHUB
	int old_power;
	int64_t old_delay_ns;
	bool old_polling;
#endif

	GYRO_LOG("gyro_store_active buf=%s\n", buf);
	mutex_lock(&gyro_context_obj->gyro_op_mutex);
	old_enable = cxt->enable;
	old_active_data = cxt->is_active_data;
#ifndef CONFIG_NANOHUB
	old_power = cxt->power;
	old_delay_ns = cxt->delay_ns;
	old_polling = READ_ONCE(cxt->is_polling_run);
#endif
	if (!strncmp(buf, "1", 1)) {
		cxt->is_active_data = true;
	} else if (!strncmp(buf, "0", 1)) {
		cxt->is_active_data = false;
	} else {
		GYRO_PR_ERR(" gyro_store_active error !!\n");
		err = -EINVAL;
		goto err_out;
	}
	cxt->enable = cxt->is_active_data || cxt->is_active_nodata;
#ifdef CONFIG_NANOHUB
	if (true == cxt->is_active_data || true == cxt->is_active_nodata) {
		err = cxt->gyro_ctl.enable_nodata(1);
		if (err) {
			GYRO_PR_ERR("gyro turn on power err = %d\n", err);
			goto restore_state;
		}
		GYRO_LOG("gyro turn on power done\n");
	} else {
		err = cxt->gyro_ctl.enable_nodata(0);
		if (err) {
			GYRO_PR_ERR("gyro turn off power err = %d\n", err);
			goto restore_state;
		}
		GYRO_LOG("gyro turn off power done\n");
	}
#else
	err = gyro_enable_and_batch();
#endif
#ifdef CONFIG_NANOHUB
restore_state:
#endif
	if (err) {
		cxt->enable = old_enable;
		cxt->is_active_data = old_active_data;
#ifndef CONFIG_NANOHUB
		cxt->delay_ns = old_delay_ns;
		if (cxt->power != old_power ||
		    READ_ONCE(cxt->is_polling_run) != old_polling) {
			int rollback_err = gyro_enable_and_batch();

			/* A power-off rollback clears delay_ns; keep the staged batch. */
			cxt->delay_ns = old_delay_ns;
			if (rollback_err < 0)
				GYRO_PR_ERR("failed to restore gyro state: %d\n",
					    rollback_err);
		}
#endif
	}
err_out:
	mutex_unlock(&gyro_context_obj->gyro_op_mutex);
	GYRO_LOG(" gyro_store_active done\n");
	return err ? (err < 0 ? err : -EIO) : count;
}
/*----------------------------------------------------------------------------*/
static ssize_t gyro_show_active(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct gyro_context *cxt = NULL;
	int div = 0;

	cxt = gyro_context_obj;

	GYRO_LOG("gyro show active not support now\n");
	div = cxt->gyro_data.vender_div;
	GYRO_LOG("gyro vender_div value: %d\n", div);
	return snprintf(buf, PAGE_SIZE, "%d\n", div);
}


static ssize_t gyro_store_batch(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct gyro_context *cxt = gyro_context_obj;
	int handle = 0, flag = 0, err = 0;
	int64_t delay_ns, latency_ns;
	int64_t old_delay_ns, old_latency_ns;

	GYRO_LOG("gyro_store_batch %s\n", buf);
	err = sscanf(buf, "%d,%d,%lld,%lld", &handle, &flag,
			&delay_ns, &latency_ns);
	if (err != 4) {
		GYRO_INFO("gyro_store_batch param error: err = %d\n", err);
		return -EINVAL;
	}
	if (delay_ns < 0 || latency_ns < 0)
		return -EINVAL;
	if (delay_ns < GYRO_MIN_POLL_DELAY_NS)
		delay_ns = GYRO_MIN_POLL_DELAY_NS;
	if (delay_ns > GYRO_MAX_POLL_DELAY_NS)
		delay_ns = GYRO_MAX_POLL_DELAY_NS;

	mutex_lock(&gyro_context_obj->gyro_op_mutex);
	old_delay_ns = cxt->delay_ns;
	old_latency_ns = cxt->latency_ns;
	cxt->delay_ns = delay_ns;
	cxt->latency_ns = latency_ns;
#ifdef CONFIG_NANOHUB
	if (cxt->gyro_ctl.is_support_batch)
		err = cxt->gyro_ctl.batch(0, cxt->delay_ns, cxt->latency_ns);
	else
		err = cxt->gyro_ctl.batch(0, cxt->delay_ns, 0);
	if (err)
		GYRO_PR_ERR("gyro set batch(ODR) err %d\n", err);
#else
	err = gyro_enable_and_batch();
#endif
	if (err) {
		cxt->delay_ns = old_delay_ns;
		cxt->latency_ns = old_latency_ns;
		if (cxt->power == 1 && old_delay_ns >= 0) {
			int rollback_err;

#ifdef CONFIG_NANOHUB
			if (cxt->gyro_ctl.is_support_batch)
				rollback_err = cxt->gyro_ctl.batch(0, old_delay_ns,
							    old_latency_ns);
			else
				rollback_err = cxt->gyro_ctl.batch(0, old_delay_ns, 0);
#else
			rollback_err = gyro_enable_and_batch();
#endif
			if (rollback_err < 0)
				GYRO_PR_ERR("failed to restore gyro rate: %d\n",
					    rollback_err);
		}
	}
	mutex_unlock(&gyro_context_obj->gyro_op_mutex);
	return err ? (err < 0 ? err : -EIO) : count;
}

static ssize_t gyro_show_batch(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t gyro_store_flush(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct gyro_context *cxt = NULL;
	int handle = 0, err = 0;

	err = kstrtoint(buf, 10, &handle);
	if (err != 0) {
		GYRO_INFO("gyro_store_flush param error: err = %d\n", err);
		return err;
	}

	GYRO_LOG("gyro_store_flush param: handle %d\n", handle);

	mutex_lock(&gyro_context_obj->gyro_op_mutex);
	cxt = gyro_context_obj;
	if (cxt->gyro_ctl.flush != NULL)
		err = cxt->gyro_ctl.flush();
	else {
		GYRO_INFO("GYRO DRIVER OLD ARCHITECTURE DON'T SUPPORT GYRO COMMON VERSION FLUSH\n");
		err = -EOPNOTSUPP;
	}
	if (err < 0)
		GYRO_INFO("gyro enable flush err %d\n", err);
	mutex_unlock(&gyro_context_obj->gyro_op_mutex);
	return err ? (err < 0 ? err : -EIO) : count;
}

static ssize_t gyro_show_flush(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}
static ssize_t gyro_show_cali(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t gyro_store_cali(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct gyro_context *cxt = NULL;
	int err = 0;
	uint8_t *cali_buf = NULL;

	cali_buf = vzalloc(count);
	if (cali_buf == NULL) {
		GYRO_INFO("kzalloc fail\n");
		return -EFAULT;
	}
	memcpy(cali_buf, buf, count);

	mutex_lock(&gyro_context_obj->gyro_op_mutex);
	cxt = gyro_context_obj;
	if (cxt->gyro_ctl.set_cali != NULL)
		err = cxt->gyro_ctl.set_cali(cali_buf, count);
	else {
		GYRO_INFO("GYRO DRIVER OLD ARCHITECTURE DON'T SUPPORT GYRO COMMON VERSION FLUSH\n");
		err = -EOPNOTSUPP;
	}
	if (err < 0)
		GYRO_INFO("gyro set cali err %d\n", err);
	mutex_unlock(&gyro_context_obj->gyro_op_mutex);
	vfree(cali_buf);
	return err ? (err < 0 ? err : -EIO) : count;
}

/* need work around again */
static ssize_t gyro_show_devnum(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}
static int gyroscope_remove(struct platform_device *pdev)
{
	GYRO_LOG("gyroscope_remove\n");
	return 0;
}

static int gyroscope_probe(struct platform_device *pdev)
{
	GYRO_LOG("gyroscope_probe\n");
	pltfm_dev = pdev;
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id gyroscope_of_match[] = {
	{ .compatible = "mediatek,gyroscope", },
	{},
};
#endif

static struct platform_driver gyroscope_driver = {
	.probe      = gyroscope_probe,
	.remove     = gyroscope_remove,
	.driver = {
		.name  = "gyroscope",
		#ifdef CONFIG_OF
		.of_match_table = gyroscope_of_match,
		#endif
	}
};

static int gyro_real_driver_init(struct platform_device *pdev)
{
	int i = 0;
	int err = 0;

	GYRO_LOG("gyro_real_driver_init +\n");
	for (i = 0; i < MAX_CHOOSE_GYRO_NUM; i++) {
		GYRO_LOG("i=%d\n", i);
		if (gyroscope_init_list[i] != 0) {
			GYRO_LOG("gyro try to init driver %s\n", gyroscope_init_list[i]->name);
			err = gyroscope_init_list[i]->init(pdev);
			if (err == 0) {
				GYRO_LOG("gyro real driver %s probe ok\n", gyroscope_init_list[i]->name);
				break;
			}
		}
	}

	if (i == MAX_CHOOSE_GYRO_NUM) {
		GYRO_LOG(" gyro_real_driver_init fail\n");
		err =  -1;
	}
	return err;
}

int gyro_driver_add(struct gyro_init_info *obj)
{
	int err = 0;
	int i = 0;

	if (!obj) {
		GYRO_PR_ERR("gyro driver add fail, gyro_init_info is NULL\n");
		return -1;
	}

	for (i = 0; i < MAX_CHOOSE_GYRO_NUM; i++) {
		if ((i == 0) && (gyroscope_init_list[0] == NULL)) {
			GYRO_LOG("register gyro driver for the first time\n");
			if (platform_driver_register(&gyroscope_driver))
				GYRO_PR_ERR("failed to register gyro driver already exist\n");
		}

		if (gyroscope_init_list[i] == NULL) {
			obj->platform_diver_addr = &gyroscope_driver;
			gyroscope_init_list[i] = obj;
			break;
		}
	}

	if (i >= MAX_CHOOSE_GYRO_NUM) {
		GYRO_PR_ERR("gyro driver add err\n");
		err =  -1;
	}

	return err;
}
EXPORT_SYMBOL_GPL(gyro_driver_add);
static int gyroscope_open(struct inode *inode, struct file *file)
{
	nonseekable_open(inode, file);
	return 0;
}

static ssize_t gyroscope_read(struct file *file, char __user *buffer,
			  size_t count, loff_t *ppos)
{
	ssize_t read_cnt = 0;

	read_cnt = sensor_event_read(gyro_context_obj->mdev.minor, file, buffer, count, ppos);

	return read_cnt;
}

static unsigned int gyroscope_poll(struct file *file, poll_table *wait)
{
	return sensor_event_poll(gyro_context_obj->mdev.minor, file, wait);
}

static const struct file_operations gyroscope_fops = {
	.owner = THIS_MODULE,
	.open = gyroscope_open,
	.read = gyroscope_read,
	.poll = gyroscope_poll,
};

static int gyro_misc_init(struct gyro_context *cxt)
{
	int err = 0;

	cxt->mdev.minor = ID_GYROSCOPE;
	cxt->mdev.name  = GYRO_MISC_DEV_NAME;
	cxt->mdev.fops = &gyroscope_fops;
	err = sensor_attr_register(&cxt->mdev);
	if (err)
		GYRO_PR_ERR("unable to register gyro misc device!!\n");

	return err;
}
DEVICE_ATTR(gyroenablenodata,     S_IWUSR | S_IRUGO, gyro_show_enable_nodata, gyro_store_enable_nodata);
DEVICE_ATTR(gyroactive,     S_IWUSR | S_IRUGO, gyro_show_active, gyro_store_active);
DEVICE_ATTR(gyrobatch,     S_IWUSR | S_IRUGO, gyro_show_batch, gyro_store_batch);
DEVICE_ATTR(gyroflush,      S_IWUSR | S_IRUGO, gyro_show_flush,  gyro_store_flush);
DEVICE_ATTR(gyrocali,      S_IWUSR | S_IRUGO, gyro_show_cali,  gyro_store_cali);
DEVICE_ATTR(gyrodevnum,      S_IWUSR | S_IRUGO, gyro_show_devnum,  NULL);

static struct attribute *gyro_attributes[] = {
	&dev_attr_gyroenablenodata.attr,
	&dev_attr_gyroactive.attr,
	&dev_attr_gyrobatch.attr,
	&dev_attr_gyroflush.attr,
	&dev_attr_gyrocali.attr,
	&dev_attr_gyrodevnum.attr,
	NULL
};

static struct attribute_group gyro_attribute_group = {
	.attrs = gyro_attributes
};

int gyro_register_data_path(struct gyro_data_path *data)
{
	struct gyro_context *cxt = NULL;

	cxt = gyro_context_obj;
	cxt->gyro_data.get_data = data->get_data;
	cxt->gyro_data.vender_div = data->vender_div;
	cxt->gyro_data.get_raw_data = data->get_raw_data;
	GYRO_LOG("gyro register data path vender_div: %d\n", cxt->gyro_data.vender_div);
	if (cxt->gyro_data.get_data == NULL) {
		GYRO_LOG("gyro register data path fail\n");
		return -1;
	}
	return 0;
}

int gyro_register_control_path(struct gyro_control_path *ctl)
{
	struct gyro_context *cxt = NULL;
	int err = 0;

	cxt = gyro_context_obj;
	cxt->gyro_ctl.set_delay = ctl->set_delay;
	cxt->gyro_ctl.open_report_data = ctl->open_report_data;
	cxt->gyro_ctl.enable_nodata = ctl->enable_nodata;
	cxt->gyro_ctl.batch = ctl->batch;
	cxt->gyro_ctl.flush = ctl->flush;
	cxt->gyro_ctl.set_cali = ctl->set_cali;
	cxt->gyro_ctl.is_support_batch = ctl->is_support_batch;
	cxt->gyro_ctl.is_use_common_factory = ctl->is_use_common_factory;
	cxt->gyro_ctl.is_report_input_direct = ctl->is_report_input_direct;
	if (NULL == cxt->gyro_ctl.batch || NULL == cxt->gyro_ctl.open_report_data
		|| NULL == cxt->gyro_ctl.enable_nodata) {
		GYRO_LOG("gyro register control path fail\n");
		return -1;
	}

	/* add misc dev for sensor hal control cmd */
	err = gyro_misc_init(gyro_context_obj);
	if (err) {
		GYRO_INFO("unable to register gyro misc device!!\n");
		return -2;
	}
	err = sysfs_create_group(&gyro_context_obj->mdev.this_device->kobj,
			&gyro_attribute_group);
	if (err < 0) {
		GYRO_INFO("unable to create gyro attribute file\n");
		return -3;
	}

	kobject_uevent(&gyro_context_obj->mdev.this_device->kobj, KOBJ_ADD);

	return 0;
}

int x_t/* = 0*/;
int y_t/* = 0*/;
int z_t/* = 0*/;
long pc/* = 0*/;

static int check_repeat_data(int x, int y, int z)
{
	if ((x_t == x) && (y_t == y) && (z_t == z))
		pc++;
	else
		pc = 0;

	x_t = x; y_t = y; z_t = z;

	if (pc > 100) {
		GYRO_INFO("Gyro sensor output repeat data\n");
		pc = 0;
	}

	return 0;
}

int gyro_data_report(struct gyro_data *data)
{
	struct sensor_event event;
	int err = 0;
	memset(&event, 0, sizeof(struct sensor_event));

	check_repeat_data(data->x, data->y, data->z);
	event.time_stamp = data->timestamp;
	event.flush_action = DATA_ACTION;
	event.status = data->status;
	event.word[0] = data->x;
	event.word[1] = data->y;
	event.word[2] = data->z;
	event.reserved = data->reserved[0];

	if (event.reserved == 1)
		mark_timestamp(ID_GYROSCOPE, DATA_REPORT, ktime_get_boot_ns(), event.time_stamp);
	err = sensor_input_event(gyro_context_obj->mdev.minor, &event);
	if (err < 0)
		pr_err_ratelimited("gyro_data_report failed due to event buffer full\n");
	return err;
}

int gyro_bias_report(struct gyro_data *data)
{
	struct sensor_event event;
	int err = 0;
	memset(&event, 0, sizeof(struct sensor_event));

	event.flush_action = BIAS_ACTION;
	event.word[0] = data->x;
	event.word[1] = data->y;
	event.word[2] = data->z;

	err = sensor_input_event(gyro_context_obj->mdev.minor, &event);
	if (err < 0)
		pr_err_ratelimited("gyro_bias_report failed due to event buffer full\n");
	return err;
}

int gyro_cali_report(struct gyro_data *data)
{
	struct sensor_event event;
	int err = 0;

	memset(&event, 0, sizeof(struct sensor_event));

	event.flush_action = CALI_ACTION;
	event.word[0] = data->x;
	event.word[1] = data->y;
	event.word[2] = data->z;

	err = sensor_input_event(gyro_context_obj->mdev.minor, &event);
	if (err < 0)
		GYRO_PR_ERR("gyro_bias_report failed due to event buffer full\n");
	return err;
}

int gyro_temp_report(int32_t *temp)
{
	struct sensor_event event;
	int err = 0;

	memset(&event, 0, sizeof(struct sensor_event));

	event.flush_action = TEMP_ACTION;
	event.word[0] = temp[0];
	event.word[1] = temp[1];
	event.word[2] = temp[2];
	event.word[3] = temp[3];
	event.word[4] = temp[4];
	event.word[5] = temp[5];

	err = sensor_input_event(gyro_context_obj->mdev.minor, &event);
	if (err < 0)
		GYRO_PR_ERR("gyro_bias_report failed due to event buffer full\n");
	return err;
}

int gyro_flush_report(void)
{
	struct sensor_event event;
	int err = 0;
	memset(&event, 0, sizeof(struct sensor_event));

	GYRO_LOG("flush\n");
	event.flush_action = FLUSH_ACTION;
	err = sensor_input_event(gyro_context_obj->mdev.minor, &event);
	if (err < 0)
		pr_err_ratelimited("gyro_flush_report failed due to event buffer full\n");
	return err;
}

static int gyro_probe(void)
{

	int err;

	GYRO_LOG("+++++++++++++gyro_probe!!\n");

	gyro_context_obj = gyro_context_alloc_object();
	if (!gyro_context_obj) {
		err = -ENOMEM;
		GYRO_PR_ERR("unable to allocate devobj!\n");
		goto exit_alloc_data_failed;
	}

	/* init real gyroeleration driver */
	err = gyro_real_driver_init(pltfm_dev);
	if (err) {
		GYRO_PR_ERR("gyro real driver init fail\n");
		goto real_driver_init_fail;
	}
	GYRO_LOG("----gyro_probe OK !!\n");
	return 0;

real_driver_init_fail:
	kfree(gyro_context_obj);

exit_alloc_data_failed:
	GYRO_PR_ERR("----gyro_probe fail !!!\n");
	return err;
}

static int gyro_remove(void)
{
	int err = 0;

	sysfs_remove_group(&gyro_context_obj->mdev.this_device->kobj,
				&gyro_attribute_group);
	err = sensor_attr_deregister(&gyro_context_obj->mdev);
	if (err)
		GYRO_PR_ERR("misc_deregister fail: %d\n", err);

	kfree(gyro_context_obj);

	return 0;
}

static int __init gyro_init(void)
{
	GYRO_LOG("gyro_init\n");

	if (gyro_probe()) {
		GYRO_PR_ERR("failed to register gyro driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit gyro_exit(void)
{
	gyro_remove();
	platform_driver_unregister(&gyroscope_driver);
}

late_initcall(gyro_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GYROSCOPE device driver");
MODULE_AUTHOR("Mediatek");
