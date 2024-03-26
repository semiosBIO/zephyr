/*
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_counter

#include <zephyr/drivers/counter.h>
#include <zephyr/spinlock.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
/* ambiq-sdk includes */
#include <am_mcu_apollo.h>

LOG_MODULE_REGISTER(ambiq_counter, CONFIG_COUNTER_LOG_LEVEL);

#define TIMER_IRQ (DT_INST_IRQN(0))

#define DEVICE_DT_GET_AND_COMMA(node_id) DEVICE_DT_GET(node_id),

static void counter_ambiq_isr(void *arg);

struct counter_ambiq_config {
	struct counter_config_info counter_info;
	uint32_t timer;
	uint32_t clk;
	uint32_t function;
};

struct counter_ambiq_data {
	counter_alarm_callback_t callback;
	void *user_data;
	uint32_t ambiq_interrupt_status_bit;
	uint32_t freq;
	uint32_t guard;
	bool started;
};

static const struct device *const devices[] = {
	DT_FOREACH_STATUS_OKAY(ambiq_counter, DEVICE_DT_GET_AND_COMMA)};

static struct k_spinlock lock;
static bool irq_init = true;

#ifdef CONFIG_SOC_APOLLO3P_BLUE
static uint32_t get_ambiq_function(uint32_t function)
{
	switch (function) {
	case 0:
		return AM_HAL_CTIMER_FN_ONCE;
	case 1:
		return AM_HAL_CTIMER_FN_REPEAT;
	case 2:
		return AM_HAL_CTIMER_FN_PWM_ONCE;
	case 3:
		return AM_HAL_CTIMER_FN_PWM_REPEAT;
	case 4:
		return AM_HAL_CTIMER_FN_PTN_ONCE;
	case 5:
		return AM_HAL_CTIMER_FN_PTN_REPEAT;
	case 6:
		return AM_HAL_CTIMER_FN_CONTINUOUS;
	case 7:
		return AM_HAL_CTIMER_FN_PWM_ALTERNATE;
	default:
		return AM_HAL_CTIMER_FN_ONCE;
	}
}

static uint32_t get_ambiq_interrupt_bit(uint32_t timer)
{
	switch (timer) {
	case 0:
		return AM_HAL_CTIMER_INT_TIMERA0C0;
	case 1:
		return AM_HAL_CTIMER_INT_TIMERA1C0;
	case 2:
		return AM_HAL_CTIMER_INT_TIMERA2C0;
	case 3:
		return AM_HAL_CTIMER_INT_TIMERA3C0;
	case 4:
		return AM_HAL_CTIMER_INT_TIMERA4C0;
	case 5:
		return AM_HAL_CTIMER_INT_TIMERA5C0;
	case 6:
		return AM_HAL_CTIMER_INT_TIMERA6C0;
	case 7:
		return AM_HAL_CTIMER_INT_TIMERA7C0;
	default:
		return AM_HAL_CTIMER_INT_TIMERA0C0;
	}
}

static uint32_t get_ambiq_clk(uint32_t clk)
{
	switch (clk) {
	case 0:
		return AM_HAL_CTIMER_CLK_PIN;
	case 1:
		return AM_HAL_CTIMER_HFRC_12MHZ;
	case 2:
		return AM_HAL_CTIMER_HFRC_3MHZ;
	case 3:
		return AM_HAL_CTIMER_HFRC_187_5KHZ;
	case 4:
		return AM_HAL_CTIMER_HFRC_47KHZ;
	case 5:
		return AM_HAL_CTIMER_HFRC_12KHZ;
	case 6:
		return AM_HAL_CTIMER_XT_32_768KHZ;
	case 7:
		return AM_HAL_CTIMER_XT_16_384KHZ;
	case 8:
		return AM_HAL_CTIMER_XT_2_048KHZ;
	case 9:
		return AM_HAL_CTIMER_XT_256HZ;
	case 10:
		return AM_HAL_CTIMER_LFRC_512HZ;
	case 11:
		return AM_HAL_CTIMER_LFRC_32HZ;
	case 12:
		return AM_HAL_CTIMER_LFRC_1HZ;
	case 13:
		return AM_HAL_CTIMER_LFRC_1_16HZ;
	case 14:
		return AM_HAL_CTIMER_RTC_100HZ;
	default:
		return AM_HAL_CTIMER_CLK_PIN;
	}
}

static uint32_t get_ambiq_clk_ctrl(uint32_t clk)
{
	switch (clk) {
	case AM_HAL_CTIMER_CLK_PIN:
		return -ENOTSUP;
	case AM_HAL_CTIMER_HFRC_12MHZ:
	case AM_HAL_CTIMER_HFRC_3MHZ:
	case AM_HAL_CTIMER_HFRC_187_5KHZ:
	case AM_HAL_CTIMER_HFRC_47KHZ:
	case AM_HAL_CTIMER_HFRC_12KHZ:
		return AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX;
	case AM_HAL_CTIMER_XT_32_768KHZ:
	case AM_HAL_CTIMER_XT_16_384KHZ:
	case AM_HAL_CTIMER_XT_2_048KHZ:
	case AM_HAL_CTIMER_XT_256HZ:
		return AM_HAL_CLKGEN_CONTROL_XTAL_START;
	case AM_HAL_CTIMER_LFRC_512HZ:
	case AM_HAL_CTIMER_LFRC_32HZ:
	case AM_HAL_CTIMER_LFRC_1HZ:
	case AM_HAL_CTIMER_LFRC_1_16HZ:
		return AM_HAL_CLKGEN_CONTROL_LFRC_START;
	case AM_HAL_CTIMER_RTC_100HZ:
		return -ENOTSUP;
	default:
		return -ENOTSUP;
	}
}

static uint32_t get_freq(uint32_t clk)
{
	switch (clk) {
	case AM_HAL_CTIMER_CLK_PIN:
		return 0;
	case AM_HAL_CTIMER_HFRC_12MHZ:
		return 12000000;
	case AM_HAL_CTIMER_HFRC_3MHZ:
		return 3000000;
	case AM_HAL_CTIMER_HFRC_187_5KHZ:
		return 1875000;
	case AM_HAL_CTIMER_HFRC_47KHZ:
		return 47000;
	case AM_HAL_CTIMER_HFRC_12KHZ:
		return 12000;
	case AM_HAL_CTIMER_XT_32_768KHZ:
		return 32768;
	case AM_HAL_CTIMER_XT_16_384KHZ:
		return 16384;
	case AM_HAL_CTIMER_XT_2_048KHZ:
		return 2048;
	case AM_HAL_CTIMER_XT_256HZ:
		return 256;
	case AM_HAL_CTIMER_LFRC_512HZ:
		return 512;
	case AM_HAL_CTIMER_LFRC_32HZ:
		return 32;
	case AM_HAL_CTIMER_LFRC_1HZ:
		return 1;
	case AM_HAL_CTIMER_LFRC_1_16HZ:
		return 1;
	case AM_HAL_CTIMER_RTC_100HZ:
		return 100;
	default:
		return 32768;
	}
};
#endif

static int counter_ambiq_init(const struct device *dev)
{
	const struct counter_ambiq_config *config = dev->config;
	struct counter_ambiq_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&lock);

#ifdef CONFIG_SOC_APOLLO3P_BLUE
	data->ambiq_interrupt_status_bit = get_ambiq_interrupt_bit(config->timer);
	data->freq = get_freq(get_ambiq_clk(config->clk));
	am_hal_clkgen_control(get_ambiq_clk_ctrl(get_ambiq_clk(config->clk)), 0);
	am_hal_ctimer_config_single(config->timer, AM_HAL_CTIMER_BOTH,
				    (get_ambiq_function(config->function) |
				     get_ambiq_clk(config->clk) | AM_HAL_CTIMER_INT_ENABLE));
#else
	am_hal_timer_config_t tc;
	am_hal_timer_default_config_set(&tc);
	tc.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV16;
	tc.eFunction = AM_HAL_TIMER_FN_UPCOUNT;
	tc.ui32PatternLimit = 0;
	data->freq = 6000000;

	am_hal_timer_config(0, &tc);
#endif
	k_spin_unlock(&lock, key);

	if (irq_init) {
		irq_init = false;
		NVIC_ClearPendingIRQ(TIMER_IRQ);
		IRQ_CONNECT(TIMER_IRQ, DT_INST_IRQ(0, priority), counter_ambiq_isr,
			    DEVICE_DT_INST_GET(0), 0);

		irq_enable(TIMER_IRQ);
	}

	return 0;
}

static int counter_ambiq_start(const struct device *dev)
{
	const struct counter_ambiq_config *config = dev->config;
	struct counter_ambiq_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&lock);
#ifdef CONFIG_SOC_APOLLO3P_BLUE
	am_hal_ctimer_start(config->timer, AM_HAL_CTIMER_BOTH);
	data->started = true;
#else
	am_hal_timer_start(0);
#endif
	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ambiq_stop(const struct device *dev)
{
	const struct counter_ambiq_config *config = dev->config;
	struct counter_ambiq_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&lock);
#ifdef CONFIG_SOC_APOLLO3P_BLUE
	am_hal_ctimer_stop(config->timer, AM_HAL_CTIMER_BOTH);
	data->started = false;
#else
	am_hal_timer_stop(0);
#endif

	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ambiq_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_ambiq_config *config = dev->config;
	k_spinlock_key_t key = k_spin_lock(&lock);
#ifdef CONFIG_SOC_APOLLO3P_BLUE
	*ticks = am_hal_ctimer_read(config->timer, AM_HAL_CTIMER_BOTH);
#else
	*ticks = am_hal_timer_read(0);
#endif

	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ambiq_get_value_64(const struct device *dev, uint64_t *ticks)
{
	const struct counter_ambiq_config *config = dev->config;
	k_spinlock_key_t key = k_spin_lock(&lock);
#ifdef CONFIG_SOC_APOLLO3P_BLUE
	*ticks = (uint64_t)am_hal_ctimer_read(config->timer, AM_HAL_CTIMER_BOTH);
#else
	*ticks = (uint64_t)am_hal_timer_read(0);
#endif

	k_spin_unlock(&lock, key);

	return 0;
}

static uint32_t counter_ambiq_get_pending_int(const struct device *dev)
{
	const struct counter_ambiq_data *data = dev->data;
#ifdef CONFIG_SOC_APOLLO3P_BLUE
	if ((data->ambiq_interrupt_status_bit & am_hal_ctimer_int_status_get(true)) != 0) {
		return 1;
	} else {
		return 0;
	}
#endif
	return 0;
}

static int counter_ambiq_set_top_value(const struct device *dev, const struct counter_top_cfg *cfg)
{
	const struct counter_ambiq_config *config = dev->config;

	if (cfg->ticks != config->counter_info.max_top_value) {
		return -ENOTSUP;
	} else {
		return 0;
	}
}

static uint32_t counter_ambiq_get_top_value(const struct device *dev)
{
	const struct counter_ambiq_config *config = dev->config;

	return config->counter_info.max_top_value;
}

static int counter_ambiq_set_alarm(const struct device *dev, uint8_t chan_id,
				   const struct counter_alarm_cfg *alarm_cfg)
{
	ARG_UNUSED(chan_id);
	struct counter_ambiq_data *data = dev->data;
	const struct counter_ambiq_config *config = dev->config;
	uint32_t now;
	uint32_t compare_val = 0;
	uint32_t relative_guard = data->started ? data->guard : 0;
	bool expire_now = false;
	int ret = 0;

	if (alarm_cfg->callback == NULL) {
		return -EINVAL;
	}
	if (counter_ambiq_get_pending_int(dev)) {
		return -EBUSY;
	}

	data->user_data = alarm_cfg->user_data;
	data->callback = alarm_cfg->callback;

	uint32_t top = counter_ambiq_get_top_value(dev);

	counter_ambiq_get_value(dev, &now);

	k_spinlock_key_t key = k_spin_lock(&lock);

	if ((alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) == 0) {
		if (alarm_cfg->ticks < relative_guard) {
			expire_now = true;
		} else {
			if ((int64_t)top - ((int64_t)now + (int64_t)alarm_cfg->ticks +
					    (int64_t)relative_guard) >
			    0) {
				compare_val = alarm_cfg->ticks + now;
			} else {
				compare_val = (now + alarm_cfg->ticks + data->guard) - top;
			}
		}
	} else {
		if (alarm_cfg->ticks >= (now + data->guard) % top) {
			compare_val = alarm_cfg->ticks;
		} else {
			expire_now =
				(alarm_cfg->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE) ? 1 : 0;
			ret = -ETIME;
		}
	}

	/* Enable interrupt, due to counter_ambiq_cancel_alarm() disables it*/
#ifdef CONFIG_SOC_APOLLO3P_BLUE
	am_hal_ctimer_int_clear(data->ambiq_interrupt_status_bit);
	am_hal_ctimer_int_enable(data->ambiq_interrupt_status_bit);
	if (expire_now) {
		am_hal_ctimer_int_set(data->ambiq_interrupt_status_bit);
	} else {
		if (ret == 0) {
			am_hal_ctimer_compare_set(config->timer,
				AM_HAL_CTIMER_BOTH, 0, compare_val);
		}
	}
#else
	am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(0, AM_HAL_TIMER_COMPARE1));
	am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(0, AM_HAL_TIMER_COMPARE1));

	am_hal_timer_compare1_set(0, compare_val);
#endif

	k_spin_unlock(&lock, key);

	return ret;
}

static int counter_ambiq_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	ARG_UNUSED(chan_id);

	const struct counter_ambiq_config *config = dev->config;
	struct counter_ambiq_data *data = dev->data;

	if (data->started == false) {
		return -ENOTSUP;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
#ifdef CONFIG_SOC_APOLLO3P_BLUE
	am_hal_ctimer_int_disable(data->ambiq_interrupt_status_bit);
	/* Reset the compare register */
	am_hal_ctimer_compare_set(config->timer, AM_HAL_CTIMER_BOTH, 0, 0);
#else
	am_hal_timer_interrupt_disable(AM_HAL_TIMER_MASK(0, AM_HAL_TIMER_COMPARE1));
	/* Reset the compare register */
	am_hal_timer_compare1_set(0, 0);
#endif
	k_spin_unlock(&lock, key);

	return 0;
}

static uint32_t counter_ambiq_get_freq(const struct device *dev)
{
	const struct counter_ambiq_data *data = dev->data;

	return data->freq;
}

static int counter_ambiq_set_guard_period(const struct device *dev, uint32_t ticks, uint32_t flags)
{
	ARG_UNUSED(flags);
	struct counter_ambiq_data *data = dev->data;

	/* max guard period half of max counter value */
	if (ticks >= (counter_ambiq_get_top_value(dev) / 2)) {
		return -EINVAL;
	}
	data->guard = ticks;
	return 0;
}

static uint32_t counter_ambiq_get_guard_period(const struct device *dev, uint32_t flags)
{
	ARG_UNUSED(flags);
	const struct counter_ambiq_data *data = dev->data;

	return data->guard;
}

static const struct counter_driver_api counter_api = {
	.start = counter_ambiq_start,
	.stop = counter_ambiq_stop,
	.get_value = counter_ambiq_get_value,
	.get_value_64 = counter_ambiq_get_value_64,
	.set_alarm = counter_ambiq_set_alarm,
	.cancel_alarm = counter_ambiq_cancel_alarm,
	.set_top_value = counter_ambiq_set_top_value,
	.get_pending_int = counter_ambiq_get_pending_int,
	.get_top_value = counter_ambiq_get_top_value,
	.get_freq = counter_ambiq_get_freq,
	.set_guard_period = counter_ambiq_set_guard_period,
	.get_guard_period = counter_ambiq_get_guard_period,
};

static const struct device *counter_ambiq_get_device(uint32_t interrupt)
{
	for (int i = 0; i < ARRAY_SIZE(devices); i++) {
		struct counter_ambiq_data *data = devices[i]->data;

		if (data->ambiq_interrupt_status_bit == interrupt) {
			return devices[i];
		}
	}
	return NULL;
}

static void counter_ambiq_isr(void *arg)
{
	uint32_t now = 0;
#ifdef CONFIG_SOC_APOLLO3P_BLUE
	const struct device *dev = counter_ambiq_get_device(am_hal_ctimer_int_status_get(true));

	if (dev == NULL) {
		return;
	}
	const struct counter_ambiq_data *data = dev->data;

	am_hal_ctimer_int_disable(data->ambiq_interrupt_status_bit);
	am_hal_ctimer_int_clear(data->ambiq_interrupt_status_bit);

#else
	const struct device *dev = (const struct device *)arg;
	struct counter_ambiq_data *data = dev->data;
	am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(0, AM_HAL_TIMER_COMPARE1));
#endif
	counter_ambiq_get_value(dev, &now);

	if (data->callback) {
		data->callback(dev, 0, now, data->user_data);
	}
}

#define AMBIQ_COUNTER_INIT(idx)                                                                    \
	static struct counter_ambiq_data counter_data_##idx = {                                    \
		.started = false,                                                                  \
		.guard = 0,                                                                        \
	};                                                                                         \
                                                                                                   \
	static const struct counter_ambiq_config counter_config_##idx = {                          \
		.counter_info = {.max_top_value = UINT32_MAX,                                      \
				 .flags = COUNTER_CONFIG_INFO_COUNT_UP,                            \
				 .channels = 1},                                                   \
		.timer = DT_PROP(DT_INST(idx, ambiq_counter), timer),                              \
		.clk = DT_PROP(DT_INST(idx, ambiq_counter), counter_clock),                        \
		.function = DT_PROP(DT_INST(idx, ambiq_counter), counter_function),                \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, counter_ambiq_init, NULL, &counter_data_##idx,                  \
			      &counter_config_##idx, PRE_KERNEL_1, CONFIG_COUNTER_INIT_PRIORITY,   \
			      &counter_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_COUNTER_INIT);
