/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "emg_sampler.h"

#include "emg_notch.h"

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <hal/nrf_saadc.h>

/* ADC (P0.04 = AIN2 on nRF52) */
#define ADC_NODE            DT_INST(0, nordic_nrf_saadc)
#define ADC_CHANNEL_ID      2
#define ADC_RESOLUTION      12

static const struct device *const adc_dev = DEVICE_DT_GET(ADC_NODE);
static int16_t adc_sample_buf;

static struct emg_notch_biquad_q30 notch1;
#if IS_ENABLED(CONFIG_EYE_NOTCH_DOUBLE)
static struct emg_notch_biquad_q30 notch2;
#endif

static atomic_t latest_sample_u16;
static atomic_t latest_seq;
static atomic_t latest_raw_u16;
static atomic_t latest_raw_seq;
static atomic_t stream_drop;
static atomic_t raw_clip_hi;
static atomic_t raw_clip_lo;

struct stream_item {
	uint32_t seq;
	uint16_t sample;
};

K_MSGQ_DEFINE(stream_q, sizeof(struct stream_item), 256, 4);
static atomic_t stream_seq;

K_THREAD_STACK_DEFINE(emg_thread_stack, 1024);
static struct k_thread emg_thread;

static int adc_setup(void)
{
	if (!device_is_ready(adc_dev)) {
		return -ENODEV;
	}

	struct adc_channel_cfg cfg = {
		.gain = ADC_GAIN_1_6,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
		.channel_id = ADC_CHANNEL_ID,
	};

#ifdef CONFIG_ADC_CONFIGURABLE_INPUTS
	cfg.input_positive = NRF_SAADC_INPUT_AIN2;
#endif

	return adc_channel_setup(adc_dev, &cfg);
}

static int adc_read_once(uint16_t *out_raw_u16)
{
	struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_ID),
		.buffer = &adc_sample_buf,
		.buffer_size = sizeof(adc_sample_buf),
		.resolution = ADC_RESOLUTION,
	};

	int err = adc_read(adc_dev, &sequence);
	if (err) {
		return err;
	}

	int32_t raw = adc_sample_buf;
	if (raw < 0) {
		raw = 0;
		atomic_inc(&raw_clip_lo);
	} else if (raw > (1 << ADC_RESOLUTION) - 1) {
		raw = (1 << ADC_RESOLUTION) - 1;
		atomic_inc(&raw_clip_hi);
	}

	*out_raw_u16 = (uint16_t)raw;
	return 0;
}

static void emg_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	BUILD_ASSERT(CONFIG_EYE_ADC_SAMPLE_RATE_HZ % CONFIG_EYE_BLE_STREAM_HZ == 0,
		     "EYE_ADC_SAMPLE_RATE_HZ must be a multiple of EYE_BLE_STREAM_HZ");

	const uint32_t fs_hz = CONFIG_EYE_ADC_SAMPLE_RATE_HZ;
	const uint32_t out_hz = CONFIG_EYE_BLE_STREAM_HZ;
	const uint32_t period_us = 1000000U / fs_hz;
	const uint32_t decim = fs_hz / out_hz;
	const bool use_envelope = IS_ENABLED(CONFIG_EYE_ENVELOPE_ENABLED);

	uint32_t sample_count = 0;
	uint16_t raw_u16 = 0;
	int32_t env_iir = 0;

	for (;;) {
		if (adc_read_once(&raw_u16) == 0) {
			atomic_set(&latest_raw_u16, raw_u16);
			atomic_inc(&latest_raw_seq);

			int32_t centered = (int32_t)raw_u16 - (1 << (ADC_RESOLUTION - 1));
			int32_t filtered = centered;
			if (IS_ENABLED(CONFIG_EYE_NOTCH_ENABLED)) {
				filtered = emg_notch_process(&notch1, filtered);
#if IS_ENABLED(CONFIG_EYE_NOTCH_DOUBLE)
				filtered = emg_notch_process(&notch2, filtered);
#endif
			}

			int32_t env = filtered;
			if (use_envelope) {
				int32_t rect = (filtered < 0) ? -filtered : filtered;
				env_iir = env_iir + ((rect - env_iir) >> CONFIG_EYE_ENVELOPE_TAU_SHIFT);
				env = env_iir;
			}

			if ((++sample_count % decim) == 0) {
				int32_t uncentered = use_envelope
							 ? env
							 : (filtered + (1 << (ADC_RESOLUTION - 1)));
				uint16_t out_u16 = (uint16_t)CLAMP(uncentered, 0, (1 << ADC_RESOLUTION) - 1);

				atomic_set(&latest_sample_u16, out_u16);
				uint32_t seq = (uint32_t)atomic_inc(&stream_seq);
				atomic_set(&latest_seq, (atomic_val_t)seq);

				struct stream_item item = {
					.seq = seq,
					.sample = out_u16,
				};
				if (k_msgq_put(&stream_q, &item, K_NO_WAIT) != 0) {
					struct stream_item dropped;
					(void)k_msgq_get(&stream_q, &dropped, K_NO_WAIT);
					atomic_inc(&stream_drop);
					(void)k_msgq_put(&stream_q, &item, K_NO_WAIT);
				}
			}
		}

		k_usleep(period_us);
	}
}

int emg_sampler_init(void)
{
	int err = adc_setup();
	if (err) {
		return err;
	}

	if (IS_ENABLED(CONFIG_EYE_NOTCH_ENABLED)) {
		err = emg_notch_configure(&notch1, CONFIG_EYE_ADC_SAMPLE_RATE_HZ,
					  CONFIG_EYE_NOTCH_FREQ_HZ, CONFIG_EYE_NOTCH_Q);
		if (err) {
			return -EINVAL;
		}
#if IS_ENABLED(CONFIG_EYE_NOTCH_DOUBLE)
		err = emg_notch_configure(&notch2, CONFIG_EYE_ADC_SAMPLE_RATE_HZ,
					  CONFIG_EYE_NOTCH_FREQ_HZ, CONFIG_EYE_NOTCH_Q);
		if (err) {
			return -EINVAL;
		}
#endif
	}

	atomic_set(&latest_sample_u16, 0);
	atomic_set(&latest_seq, 0);
	atomic_set(&latest_raw_u16, 0);
	atomic_set(&latest_raw_seq, 0);
	atomic_set(&stream_seq, 0);
	atomic_set(&stream_drop, 0);
	atomic_set(&raw_clip_hi, 0);
	atomic_set(&raw_clip_lo, 0);

	k_thread_create(&emg_thread, emg_thread_stack, K_THREAD_STACK_SIZEOF(emg_thread_stack),
			emg_thread_fn, NULL, NULL, NULL,
			K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	k_thread_name_set(&emg_thread, "emg_sampler");

	return 0;
}

uint32_t emg_sampler_get_latest_seq(void)
{
	return (uint32_t)atomic_get(&latest_seq);
}

uint16_t emg_sampler_get_latest_sample_u16(void)
{
	return (uint16_t)atomic_get(&latest_sample_u16);
}

uint32_t emg_sampler_get_latest_raw_seq(void)
{
	return (uint32_t)atomic_get(&latest_raw_seq);
}

uint16_t emg_sampler_get_latest_raw_u16(void)
{
	return (uint16_t)atomic_get(&latest_raw_u16);
}

uint32_t emg_sampler_pop_stream(uint32_t *first_seq, uint16_t *dst, uint32_t max_samples)
{
	if (!first_seq || !dst || max_samples == 0) {
		return 0;
	}

	struct stream_item item;
	uint32_t count = 0;

	while (count < max_samples) {
		if (k_msgq_get(&stream_q, &item, K_NO_WAIT) != 0) {
			break;
		}

		if (count == 0) {
			*first_seq = item.seq;
		}
		dst[count++] = item.sample;
	}

	return count;
}

uint32_t emg_sampler_get_stream_drop_count(void)
{
	return (uint32_t)atomic_get(&stream_drop);
}

uint32_t emg_sampler_get_clip_hi(void)
{
	return (uint32_t)atomic_get(&raw_clip_hi);
}

uint32_t emg_sampler_get_clip_lo(void)
{
	return (uint32_t)atomic_get(&raw_clip_lo);
}
