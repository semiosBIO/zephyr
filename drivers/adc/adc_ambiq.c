/*
 * Copyright (c) 2023 Semios Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT ambiq_adc

#include <zephyr/drivers/adc.h>
#include <zephyr/dt-bindings/adc/ambiq_adc.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#define ADC_CONTEXT_ENABLE_ON_COMPLETE

#include "adc_context.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>

#include <am_mcu_apollo.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_ambiq, CONFIG_ADC_LOG_LEVEL);

#define AMBIQ_ADC_NUM_SE_CHANNELS 10
#define AMBIQ_ADC_NUM_DF_CHANNELS 2
#define AMBIQ_ADC_NUM_SLOTS 8

#define AMBIQ_ADC_REF_INT  0
#define AMBIQ_ADC_REF_EXT0 1
#define AMBIQ_ADC_REF_EXT1 2

struct adc_ambiq_config {
	/* Base register address */
	uint32_t base;
	/* Total size of register space */
	int size;
	const struct pinctrl_dev_config *pcfg;
	/* ADC votage reference source */
	unsigned int ref_source;
	/* ADC reference voltage in millivolts */
	unsigned int ref_mv;
};

struct adc_ambiq_data {
	/* ADC context when reading */
	struct adc_context ctx;
	/* Internal handle for the Ambiq API */
	void *adc_handle;
	/* Bit mask of outstanding channels to be read */
	uint32_t read_channel_mask;
	/* Bit mask of channels to be read */
	uint32_t channel_mask;
	/* Total used ADC slots during conversion/read */
	unsigned int used_slots;
	/* User provided buffer for ADC data */
	uint16_t *buffer;
	/* User provided buffer for repeat ADC read data */
	uint16_t *repeat_buffer;
	/* Current slot configuration common to all channels */
	am_hal_adc_slot_config_t slot_config;
};

static int adc_ambiq_channel_setup(const struct device *dev,
				const struct adc_channel_cfg *channel_cfg)
{
	const struct adc_ambiq_config *cfg = dev->config;
	struct adc_ambiq_data *data = dev->data;
	uint8_t chan_id = channel_cfg->channel_id;

	am_hal_adc_config_t adc_config;
	int err;

	if (chan_id >= (AMBIQ_ADC_NUM_SE_CHANNELS + AMBIQ_ADC_NUM_DF_CHANNELS)) {
		return -EINVAL;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Select ADC acquisition time is invalid");
		return -EINVAL;
	};

	if (channel_cfg->differential) {
		uint8_t ipos = channel_cfg->input_positive;
		uint8_t ineg = channel_cfg->input_negative;

		if (chan_id < AMBIQ_ADC_NUM_SE_CHANNELS) {
			LOG_ERR("Invalid differential channel");
			return -EINVAL;
		}

		if ((ipos != AMBIQ_ADC_P_P13) && (ipos != AMBIQ_ADC_P_P14) &&
			(ineg != AMBIQ_ADC_N_P12) && (ineg != AMBIQ_ADC_N_P15)) {
			LOG_ERR("Differential input pins invalid");
			return -EINVAL;
		}

		if ((chan_id == 10) && ((ipos != AMBIQ_ADC_P_P13) && (ineg != AMBIQ_ADC_N_P12))) {
			LOG_ERR("Channel 10 may only use pins 12 and 13 as differential inputs");
			return -EINVAL;
		}

		if ((chan_id == 11) && ((ipos != AMBIQ_ADC_P_P14) && (ineg != AMBIQ_ADC_N_P15))) {
			LOG_ERR("Channel 11 may only use pins 14 and 15 as differential inputs");
			return -EINVAL;
		}
	}

	if (channel_cfg->reference != (cfg->ref_source + ADC_REF_INTERNAL)) {
		LOG_ERR("Channel reference source must be the same as the ADC configuration");
		return -EINVAL;
	}

	adc_config.eClock = AM_HAL_ADC_CLKSEL_HFRC;
	adc_config.ePolarity = AM_HAL_ADC_TRIGPOL_RISING;
	adc_config.eTrigger = AM_HAL_ADC_TRIGSEL_SOFTWARE;

	if (cfg->ref_source == AMBIQ_ADC_REF_INT) {
		if (cfg->ref_mv == 2200) {
			adc_config.eReference = AM_HAL_ADC_REFSEL_INT_2P0;
		} else {
			adc_config.eReference = AM_HAL_ADC_REFSEL_INT_1P5;
		}
	} else {
		if (cfg->ref_mv == 2200) {
			adc_config.eReference = AM_HAL_ADC_REFSEL_EXT_2P0;
		} else {
			adc_config.eReference = AM_HAL_ADC_REFSEL_EXT_1P5;
		}
	}

	adc_config.eClockMode = AM_HAL_ADC_CLKMODE_LOW_LATENCY;
	adc_config.ePowerMode = AM_HAL_ADC_LPMODE0;
	adc_config.eRepeat = AM_HAL_ADC_SINGLE_SCAN;

	err = am_hal_adc_configure(data->adc_handle, &adc_config);
	if (err) {
		LOG_ERR("Failed to configure ADC %d", err);
	}

	return err;
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_ambiq_data *data = CONTAINER_OF(ctx, struct adc_ambiq_data, ctx);
	int err = 0;

	if (data->read_channel_mask == 0) {
		data->read_channel_mask = data->channel_mask;
	}

	data->used_slots = MIN(__builtin_popcount(data->read_channel_mask), AMBIQ_ADC_NUM_SLOTS);

	/*
	 * Since there are only ADC_NUM_SLOT_CHANNELS available it's possible
	 * that not all the channels can be configured to be read right now
	 */
	for (unsigned int ii = 0; ii < data->used_slots; ii++) {
		unsigned int chan = __builtin_ctz(data->read_channel_mask);

		data->read_channel_mask ^= 1 << chan;
		data->slot_config.eChannel = (am_hal_adc_slot_chan_e)chan;

		am_hal_adc_configure_slot(data->adc_handle, ii, &data->slot_config);
		if (err) {
			LOG_ERR("Failed to configure ADC slot %d", err);
		}
	}

	am_hal_adc_sw_trigger(data->adc_handle);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat)
{
	struct adc_ambiq_data *data = CONTAINER_OF(ctx, struct adc_ambiq_data, ctx);

	if (repeat) {
		data->buffer = data->repeat_buffer;
	}
}

static void adc_context_on_complete(struct adc_context *ctx, int status)
{
	struct adc_ambiq_data *data = CONTAINER_OF(ctx, struct adc_ambiq_data, ctx);

	am_hal_adc_interrupt_disable(data->adc_handle, AM_HAL_ADC_INT_CNVCMP);
	am_hal_adc_disable(data->adc_handle);
}

/**
 * Validate that the given read sequence has provided enough storage space
 *
 * @param sequence Reference to the sequence for read operation
 *
 * @return 0 success
 * @return -ENOMEM no enough memory provided
 */
static int check_buffer_size(const struct adc_sequence *sequence)
{
	size_t required_buffer_size;
	unsigned int total_channels = __builtin_popcount(sequence->channels);

	required_buffer_size = total_channels * sizeof(uint16_t);
	if (sequence->options) {
		required_buffer_size *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < required_buffer_size) {
		return -ENOMEM;
	}
	return 0;
}

static int adc_ambiq_start_read(const struct device *dev,
				const struct adc_sequence *sequence)
{
	struct adc_ambiq_data *data = dev->data;
	int err = 0;
	am_hal_adc_slot_config_t slot_config;

	err = check_buffer_size(sequence);
	if (err) {
		return err;
	}

	if (sequence->oversampling > 7) {
		LOG_ERR("Invalid oversampling");
		return -EINVAL;
	}

	if (sequence->channels == 0) {
		LOG_ERR("At least one channel must be selected");
		return -EINVAL;
	}

	if (sequence->options && sequence->options->interval_us) {
		return -ENOTSUP;
	}

	switch (sequence->resolution) {
	case 8:
		slot_config.ePrecisionMode = AM_HAL_ADC_SLOT_8BIT;
		break;
	case 10:
		slot_config.ePrecisionMode = AM_HAL_ADC_SLOT_10BIT;
		break;
	case 12:
		slot_config.ePrecisionMode = AM_HAL_ADC_SLOT_12BIT;
		break;
	case 14:
		slot_config.ePrecisionMode = AM_HAL_ADC_SLOT_14BIT;
		break;
	default:
		LOG_ERR("Invalid sampling resolution %d", sequence->resolution);
		return -EINVAL;
	}

	switch (sequence->oversampling) {
	case 2:
		slot_config.eMeasToAvg = AM_HAL_ADC_SLOT_AVG_2;
		break;
	case 4:
		slot_config.eMeasToAvg = AM_HAL_ADC_SLOT_AVG_4;
		break;
	case 8:
		slot_config.eMeasToAvg = AM_HAL_ADC_SLOT_AVG_8;
		break;
	case 16:
		slot_config.eMeasToAvg = AM_HAL_ADC_SLOT_AVG_16;
		break;
	case 32:
		slot_config.eMeasToAvg = AM_HAL_ADC_SLOT_AVG_32;
		break;
	case 64:
		slot_config.eMeasToAvg = AM_HAL_ADC_SLOT_AVG_64;
		break;
	case 128:
		slot_config.eMeasToAvg = AM_HAL_ADC_SLOT_AVG_128;
		break;
	case 1:
	default:
		slot_config.eMeasToAvg = AM_HAL_ADC_SLOT_AVG_1;
		break;
	}

	slot_config.bWindowCompare = false;
	slot_config.bEnabled = true;

	data->slot_config = slot_config;
	data->read_channel_mask = sequence->channels;
	data->channel_mask = sequence->channels;
	data->buffer = sequence->buffer;
	data->repeat_buffer = sequence->buffer;

	am_hal_adc_interrupt_enable(data->adc_handle, AM_HAL_ADC_INT_CNVCMP);

	err = am_hal_adc_enable(data->adc_handle);
	if (err) {
		LOG_ERR("Failed to read ADC %d", err);
	}

	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int adc_ambiq_read(const struct device *dev,
				const struct adc_sequence *sequence)
{
	struct adc_ambiq_data *data = dev->data;
	int err = 0;

	adc_context_lock(&data->ctx, false, NULL);
	err = adc_ambiq_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_ambiq_read_async(const struct device *dev,
				const struct adc_sequence *sequence,
				struct k_poll_signal *async)
{
	struct adc_ambiq_data *data = dev->data;
	int err = 0;

	adc_context_lock(&data->ctx, true, async);
	err = adc_ambiq_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}
#endif /* CONFIG_ADC_ASYNC */

static void adc_ambiq_isr(const struct device *dev)
{
	struct adc_ambiq_data *data = dev->data;
	am_hal_adc_sample_t sample;
	uint32_t num_samples = 1;
	unsigned int read_cnt = 0;
	uint32_t mask;

	am_hal_adc_interrupt_status(data->adc_handle, &mask, false);
	am_hal_adc_interrupt_clear(data->adc_handle, mask);

	if (mask & AM_HAL_ADC_INT_CNVCMP) {
		for (read_cnt = 0; read_cnt < data->used_slots; read_cnt++) {
			if (am_hal_adc_samples_read(data->adc_handle, false, NULL,
							&num_samples, &sample)) {
				LOG_ERR("Failed to read ADC samples FIFO");
				sample.ui32Sample = 0;
			}
			*data->buffer++ = (uint16_t)sample.ui32Sample;
		}

		data->used_slots = 0;

		/*
		 * When reading more channels than the maximum number of slots, another
		 * round of sampling has to be scheduled
		 */
		if (data->read_channel_mask == 0) {
			adc_context_on_sampling_done(&data->ctx, dev);
		} else {
			adc_context_start_sampling(&data->ctx);
		}
	}
}

static int adc_ambiq_init(const struct device *dev)
{
	struct adc_ambiq_data *data = dev->data;
	const struct adc_ambiq_config *cfg = dev->config;
	int ret = 0;
	uint32_t pwr_addr = DT_REG_ADDR(DT_INST_PHANDLE(0, ambiq_pwrcfg)) +
						DT_INST_PHA(0, ambiq_pwrcfg, offset);

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("ADC pinctrl setup failed (%d)", ret);
		return ret;
	}

	sys_write32((sys_read32(pwr_addr) | DT_INST_PHA(0, ambiq_pwrcfg, mask)), pwr_addr);
	k_busy_wait(5);

	ret = am_hal_adc_initialize((cfg->base - REG_ADC_BASEADDR) / cfg->size, &data->adc_handle);

	/* There is only a single ADC interrupt for all channels */
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), adc_ambiq_isr,
			DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	adc_context_unlock_unconditionally(&data->ctx);

	return ret;
}

static const struct adc_driver_api adc_ambiq_api = {
	.channel_setup = adc_ambiq_channel_setup,
	.read = adc_ambiq_read,
#if CONFIG_ADC_ASYNC
	.read_async = adc_ambiq_read_async,
#endif
	/* There is only one ADC instance available so instance 0 works here */
	.ref_internal = DT_INST_PROP(0, ambiq_vref_mv)
};

#define AMBIQ_ADC_INIT(inst) \
	BUILD_ASSERT((inst) == 0, "only a single instance supported"); \
	PINCTRL_DT_INST_DEFINE(inst); \
	static const struct adc_ambiq_config adc_config_##inst = { \
		.base = DT_INST_REG_ADDR(inst), \
		.size = DT_INST_REG_SIZE(inst), \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst), \
		.ref_source = DT_INST_PROP(inst, ambiq_adc_reference), \
		.ref_mv = DT_INST_PROP(inst, ambiq_vref_mv) \
	}; \
	static struct adc_ambiq_data adc_data_##inst = { \
		ADC_CONTEXT_INIT_TIMER(adc_data_##inst, ctx), \
		ADC_CONTEXT_INIT_LOCK(adc_data_##inst, ctx), \
		ADC_CONTEXT_INIT_SYNC(adc_data_##inst, ctx), \
	}; \
	DEVICE_DT_INST_DEFINE(0, \
					adc_ambiq_init, \
					NULL, \
					&adc_data_##inst, \
					&adc_config_##inst, \
					POST_KERNEL, \
					CONFIG_ADC_INIT_PRIORITY, \
					&adc_ambiq_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_ADC_INIT);
