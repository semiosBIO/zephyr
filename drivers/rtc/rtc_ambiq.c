/*
 * Copyright (c) 2024 Semios Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#define DT_DRV_COMPAT ambiq_rtc

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/rtc.h>

#include <am_mcu_apollo.h>

LOG_MODULE_REGISTER(rtc_ambiq, CONFIG_RTC_LOG_LEVEL);

#define AMBIQ_RTC_ALARM_MASK (RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | \
			RTC_ALARM_TIME_MASK_HOUR | RTC_ALARM_TIME_MASK_MONTHDAY | \
			RTC_ALARM_TIME_MASK_MONTH | RTC_ALARM_TIME_MASK_WEEKDAY)

#ifdef CONFIG_RTC_ALARM
void ambiq_rtc_alarm_work_handler(struct k_work *work);
K_WORK_DEFINE(alarm_work, ambiq_rtc_alarm_work_handler);
#endif

struct ambiq_rtc_data {
	struct k_mutex lock;
	int set_century;
#ifdef CONFIG_RTC_ALARM
	const struct device *dev;
	volatile bool alarm_pending;
	rtc_alarm_callback alarm_callback;
	void *alarm_user_data;
	struct k_work alarm_work;
#endif
};

static int rtc_time_to_ambiq(const struct rtc_time *tm, am_hal_rtc_time_t *am,
				int *prev_century)
{
	am->ui32CenturyEnable = 1;
	am->ui32Hour = tm->tm_hour;
	am->ui32Minute = tm->tm_min;
	am->ui32Second = tm->tm_sec;
	am->ui32Hundredths = tm->tm_nsec / 10000000;
	am->ui32Weekday = tm->tm_wday;
	am->ui32DayOfMonth = tm->tm_mday;
	am->ui32Month = tm->tm_mon + 1;
	/* tm_year = year - 1900 */
	am->ui32Year = tm->tm_year % 100;
	/* 0 = 2000s; 1 = 1900s/2100s */
	if ((tm->tm_year >= 100) && (tm->tm_year < 200)) {
		*prev_century = 0;
	} else {
		*prev_century = 1;
	}
	am->ui32Century = *prev_century;
	return 0;
}

static int ambiq_to_rtc_time(const am_hal_rtc_time_t *am, struct rtc_time *tm,
				int prev_century)
{
	tm->tm_hour = am->ui32Hour;
	tm->tm_min = am->ui32Minute;
	tm->tm_sec = am->ui32Second + am->ui32Hundredths / 100;
	tm->tm_nsec = (am->ui32Hundredths % 100) * 10000000;
	tm->tm_wday = am->ui32Weekday;
	tm->tm_mday = am->ui32DayOfMonth;
	tm->tm_mon = am->ui32Month - 1;
	/* 0 = 2000s; 1 = 1900s/2100s */
	tm->tm_year = am->ui32Year;
	if (am->ui32CenturyEnable) {
		if (am->ui32Century == 0) {
			tm->tm_year += 100;
		} else if ((prev_century == 0) && (am->ui32Century == 1)) {
			tm->tm_year += 200;
		}
	}
	tm->tm_yday = -1;
	tm->tm_isdst = -1;
	return 0;
}

static int rtc_ambiq_set_time(const struct device *dev, const struct rtc_time *tm)
{
	struct ambiq_rtc_data *data = dev->data;
	am_hal_rtc_time_t ambiq_time = {0};
	int err;

	if ((tm == NULL) || (tm->tm_year > 299)) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	rtc_time_to_ambiq(tm, &ambiq_time, &data->set_century);

	err = am_hal_rtc_time_set(&ambiq_time);
	if (err) {
		printk("Failed to set ambiq rtc: %d\n", err);
		err = -EIO;
	}

	k_mutex_unlock(&data->lock);

	return err;
}

static int rtc_ambiq_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	struct ambiq_rtc_data *data = dev->data;
	am_hal_rtc_time_t ambiq_time = {0};
	int err;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	err = am_hal_rtc_time_get(&ambiq_time);
	if (err) {
		printk("Failed to get ambiq rtc time: %d\n", err);
		err = -EIO;
		goto ambiq_rtc_unlock;
	}

	ambiq_to_rtc_time(&ambiq_time, timeptr, data->set_century);

ambiq_rtc_unlock:
	k_mutex_unlock(&data->lock);
	return err;
}

#if CONFIG_RTC_ALARM
void ambiq_rtc_alarm_work_handler(struct k_work *work)
{
	struct ambiq_rtc_data *data = CONTAINER_OF(work, struct ambiq_rtc_data, alarm_work);

	if (data->alarm_callback) {
		data->alarm_callback(data->dev, 0, data->alarm_user_data);
		data->alarm_pending = false;
	}
}

static int rtc_ambiq_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask,
		struct rtc_time *timeptr)
{
	int err = 0;
	struct ambiq_rtc_data *data = dev->data;
	am_hal_rtc_time_t ambiq_time = {0};
	int temp_century = 0;

	if (id != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	am_hal_rtc_alarm_get(&ambiq_time);
	ambiq_to_rtc_time(&ambiq_time, timeptr, temp_century);

	*mask = 0;

	if (ambiq_time.ui32Second) {
		*mask |= RTC_ALARM_TIME_MASK_SECOND;
	}

	if (ambiq_time.ui32Minute) {
		*mask |= RTC_ALARM_TIME_MASK_MINUTE;
	}

	if (ambiq_time.ui32Hour) {
		*mask |= RTC_ALARM_TIME_MASK_HOUR;
	}

	if (ambiq_time.ui32Weekday) {
		*mask |= RTC_ALARM_TIME_MASK_WEEKDAY;
	}

	if (ambiq_time.ui32DayOfMonth) {
		*mask |= RTC_ALARM_TIME_MASK_MONTHDAY;
	}

	if (ambiq_time.ui32Month) {
		*mask |= RTC_ALARM_TIME_MASK_MONTH;
	}

	LOG_DBG("get alarm: wday = %d, mon = %d, mday = %d, hour = %d, min = %d, sec = %d, "
		"mask = 0x%04x", timeptr->tm_wday, timeptr->tm_mon, timeptr->tm_mday,
		timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec, *mask);

	k_mutex_unlock(&data->lock);
	return err;
}

static int rtc_ambiq_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
		const struct rtc_time *timeptr)
{
	struct ambiq_rtc_data *data = dev->data;
	uint32_t rpt_interval = AM_HAL_RTC_ALM_RPT_DIS;
	am_hal_rtc_time_t ambiq_time;
	int temp_century = 0;

	if (id != 0) {
		LOG_ERR("invalid alarm id: %u", id);
		return -EINVAL;
	}

	if ((mask & ~(AMBIQ_RTC_ALARM_MASK)) != 0) {
		LOG_ERR("unsupported alarm mask 0x%04x", mask);
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (mask == 0) {
		LOG_DBG("alarm disabled");
		goto unlock;
	}

	rtc_time_to_ambiq(timeptr, &ambiq_time, &temp_century);

	if (mask & RTC_ALARM_TIME_MASK_SECOND) {
		rpt_interval = AM_HAL_RTC_ALM_RPT_MIN;
	} else {
		ambiq_time.ui32Second = 0;
	}

	if (mask & RTC_ALARM_TIME_MASK_MINUTE) {
		rpt_interval = AM_HAL_RTC_ALM_RPT_HR;
	} else {
		ambiq_time.ui32Minute = 0;
	}

	if (mask & RTC_ALARM_TIME_MASK_HOUR) {
		rpt_interval = AM_HAL_RTC_ALM_RPT_DAY;
	} else {
		ambiq_time.ui32Hour = 0;
	}

	if (mask & RTC_ALARM_TIME_MASK_WEEKDAY) {
		rpt_interval = AM_HAL_RTC_ALM_RPT_WK;
	} else {
		ambiq_time.ui32Weekday = 0;
	}

	if (mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		rpt_interval = AM_HAL_RTC_ALM_RPT_MTH;
	} else {
		ambiq_time.ui32DayOfMonth = 0;
	}

	if (mask & RTC_ALARM_TIME_MASK_MONTH) {
		rpt_interval = AM_HAL_RTC_ALM_RPT_YR;
	} else {
		ambiq_time.ui32Month = 0;
	}

	am_hal_rtc_alarm_set(&ambiq_time, rpt_interval);

	LOG_DBG("set alarm: second = %d, min = %d, hour = %d, mday = %d, month = %d,"
		"wday = %d,  mask = 0x%04x",
		timeptr->tm_sec, timeptr->tm_min, timeptr->tm_hour, timeptr->tm_mday,
		timeptr->tm_mon, timeptr->tm_wday, mask);

unlock:
	k_mutex_unlock(&data->lock);
	return 0;
}

static int rtc_ambiq_alarm_get_supported_fields(const struct device *dev,
		uint16_t id, uint16_t *mask)
{
	ARG_UNUSED(dev);

	if ((mask == NULL) || (id != 0)) {
		return -EINVAL;
	}

	*mask = AMBIQ_RTC_ALARM_MASK;

	return 0;
}

static int rtc_ambiq_alarm_is_pending(const struct device *dev, uint16_t id)
{
	struct ambiq_rtc_data *data = dev->data;
	int ret;

	if (id != 0) {
		return -EINVAL;
	}


	k_mutex_lock(&data->lock, K_FOREVER);

	am_hal_rtc_int_disable(AM_HAL_RTC_INT_ALM);
	ret = data->alarm_pending;
	data->alarm_pending = false;
	am_hal_rtc_int_enable(AM_HAL_RTC_INT_ALM);

	k_mutex_lock(&data->lock, K_FOREVER);


	return ret;
}

static int rtc_ambiq_alarm_set_callback(const struct device *dev, uint16_t id,
		rtc_alarm_callback callback, void *user_data)
{
	struct ambiq_rtc_data *data = dev->data;

	if (id != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	data->alarm_callback = callback;
	data->alarm_user_data = user_data;

	k_mutex_lock(&data->lock, K_FOREVER);

	return 0;
}

static void rtc_ambiq_alarm_isr(const struct device *dev)
{
	struct ambiq_rtc_data *data = dev->data;

	data->alarm_pending = true;
	am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);
	k_work_submit(&(data->alarm_work));
}
#endif

static int rtc_ambiq_init(const struct device *dev)
{
	struct ambiq_rtc_data *data = dev->data;

	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);
	am_hal_rtc_osc_select(AM_HAL_RTC_OSC_XT);
	am_hal_rtc_osc_enable();

	am_hal_rtc_time_12hour(false);

	k_mutex_init(&data->lock);

	/* Indicates that starting at 1900 */
	data->set_century = -1;
#ifdef CONFIG_RTC_ALARM
	data->alarm_work = alarm_work;
	data->dev = dev;

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), rtc_ambiq_alarm_isr,
				DEVICE_DT_INST_GET(0), 0);

	am_hal_rtc_alarm_interval_set(AM_HAL_RTC_ALM_RPT_DIS);
	am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);
	am_hal_rtc_int_enable(AM_HAL_RTC_INT_ALM);

	irq_enable(DT_INST_IRQN(0));
#endif

	return 0;
}

static const struct rtc_driver_api rtc_ambiq_driver_api = {
	.set_time = rtc_ambiq_set_time,
	.get_time = rtc_ambiq_get_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = rtc_ambiq_alarm_get_supported_fields,
	.alarm_set_time = rtc_ambiq_alarm_set_time,
	.alarm_get_time = rtc_ambiq_alarm_get_time,
	.alarm_is_pending = rtc_ambiq_alarm_is_pending,
	.alarm_set_callback = rtc_ambiq_alarm_set_callback,
#endif
};

static struct ambiq_rtc_data rtc_data;

DEVICE_DT_INST_DEFINE(0, &rtc_ambiq_init, NULL, &rtc_data, NULL, PRE_KERNEL_1,
		      CONFIG_RTC_INIT_PRIORITY, &rtc_ambiq_driver_api);
