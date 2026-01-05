/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "emg_sampler.h"

#include "emg_notch.h"

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <nrfx_timer.h>
#include <nrfx_ppi.h>
#include <hal/nrf_saadc.h>
#include <nrfx_saadc.h>

#ifndef NRFX_SAADC_CONFIG_IRQ_PRIORITY
#define NRFX_SAADC_CONFIG_IRQ_PRIORITY 6
#endif

/*
 * SAADC sampler pipeline.
 *
 * Data flow:
 * - TIMER2 compares at fs -> PPI -> SAADC SAMPLE
 * - SAADC EasyDMA fills double buffers
 * - SAADC DONE ISR processes samples:
 *   raw -> notch -> high-pass (DC block) -> optional envelope
 * - Outputs:
 *   - Latest values (atomics) for BLE read/notify
 *   - Stream queue (decimated) for BLE streaming
 *   - UART queue (full-rate) for binary streaming
 */

/* SAADC (P0.04 = AIN2 on nRF52) */
#define ADC_CHANNEL_ID      0
#define ADC_INPUT           NRF_SAADC_INPUT_AIN2
#define ADC_RESOLUTION_BITS 12
/* Larger DMA buffer: 200 samples = 100 ms @ 2 kHz (double-buffered). */
#define SAADC_BUF_SIZE      200

static const nrfx_timer_t emg_timer = NRFX_TIMER_INSTANCE(2);
static nrf_ppi_channel_t timer_saadc_ppi;
static nrf_ppi_channel_t saadc_start_ppi;
static int16_t saadc_buf[2][SAADC_BUF_SIZE] __aligned(4);

static struct emg_notch_biquad_q30 notch1;
#if IS_ENABLED(CONFIG_EYE_NOTCH_DOUBLE)
static struct emg_notch_biquad_q30 notch2;
#endif

static atomic_t latest_sample_u16;
static atomic_t latest_seq;
static atomic_t latest_raw_u16;
static atomic_t latest_raw_seq;
static atomic_t latest_notch_u16;
static atomic_t latest_env_u16;
static atomic_t stream_drop;
static atomic_t raw_clip_hi;
static atomic_t raw_clip_lo;
/* Queue for UART to fetch every sample (raw + filtered + notch). */
/* UART queue depth trimmed for RAM headroom. */
K_MSGQ_DEFINE(uart_sample_q, sizeof(struct emg_sample_triple), 192, 4);
static atomic_t uart_q_drop;
static atomic_t uart_q_hwm;

struct stream_item {
	uint32_t seq;
	uint16_t filtered;
	uint16_t envelope;
};

/* Decimated stream queue: keep 64 entries (~512 bytes) to reduce drops. */
K_MSGQ_DEFINE(stream_q, sizeof(struct stream_item), 64, 4);
static atomic_t stream_seq;

static const uint32_t decim = CONFIG_EYE_ADC_SAMPLE_RATE_HZ / CONFIG_EYE_BLE_STREAM_HZ;
static const bool use_envelope = IS_ENABLED(CONFIG_EYE_ENVELOPE_ENABLED);
static const int32_t adc_max = (1 << ADC_RESOLUTION_BITS) - 1;
static uint32_t sample_count;
static int32_t env_iir;
static uint16_t env_last_u16;
static int evt_logged;
static uint16_t active_buf_size = SAADC_BUF_SIZE;
static int drop_log_budget = 8;
/* Simple high-pass (DC-block) on centered filtered signal: y = x - x1 + a*y1 (Q15). */
#define HP_A_Q15 32112 /* ~0.98 */
static int32_t hp_x1;
static int32_t hp_y1;

/*
 * Process one raw sample:
 * - Clip to ADC range, track clip counters.
 * - Notch filter at mains frequency (single or double stage).
 * - DC-block high-pass to remove baseline wander.
 * - Envelope extraction (abs + IIR).
 * - Publish to atomics + queues.
 */
static void process_sample(int16_t raw)
{
	if (raw < 0) {
		raw = 0;
		atomic_inc(&raw_clip_lo);
	} else if (raw > adc_max) {
		raw = adc_max;
		atomic_inc(&raw_clip_hi);
	}

	atomic_set(&latest_raw_u16, (uint16_t)raw);
	atomic_inc(&latest_raw_seq);

	int32_t centered = (int32_t)raw - (1 << (ADC_RESOLUTION_BITS - 1));
	int32_t filtered = emg_notch_process(&notch1, centered);
#if IS_ENABLED(CONFIG_EYE_NOTCH_DOUBLE)
	filtered = emg_notch_process(&notch2, filtered);
#endif
	/* DC-block to keep the baseline near mid-scale. */
	int32_t hp_y = (int32_t)filtered - hp_x1 + ((hp_y1 * HP_A_Q15) >> 15);
	hp_x1 = filtered;
	hp_y1 = hp_y;

	uint16_t notch_u16_fullrate =
		(uint16_t)CLAMP(hp_y + (1 << (ADC_RESOLUTION_BITS - 1)), 0, adc_max);
	atomic_set(&latest_notch_u16, notch_u16_fullrate);

	int32_t env = filtered;
	if (use_envelope) {
		/* Rectify and smooth to approximate signal envelope. */
		int32_t rect = (hp_y < 0) ? -hp_y : hp_y;
		env_iir = env_iir + ((rect - env_iir) >> CONFIG_EYE_ENVELOPE_TAU_SHIFT);
		env = env_iir;
	}

	uint16_t env_u16 = (uint16_t)CLAMP(env, 0, adc_max);
	env_last_u16 = env_u16;

	int32_t uncentered_fullrate = hp_y + (1 << (ADC_RESOLUTION_BITS - 1));
	uint16_t out_fullrate_u16 = (uint16_t)CLAMP(uncentered_fullrate, 0, adc_max);
	atomic_set(&latest_sample_u16, out_fullrate_u16);
	atomic_set(&latest_env_u16, env_u16);

	if (!IS_ENABLED(CONFIG_EYE_BLE_STREAM_DISABLED)) {
		/* Decimate to the BLE stream rate to keep queue size manageable. */
		if ((++sample_count % decim) == 0) {
			uint32_t seq = (uint32_t)atomic_inc(&stream_seq);
			atomic_set(&latest_seq, (atomic_val_t)seq);

			struct stream_item item = {
				.seq = seq,
				.filtered = out_fullrate_u16,
				.envelope = env_last_u16,
			};
			if (k_msgq_put(&stream_q, &item, K_NO_WAIT) != 0) {
				struct stream_item dropped;
				(void)k_msgq_get(&stream_q, &dropped, K_NO_WAIT);
				uint32_t drop_new = (uint32_t)atomic_inc(&stream_drop) + 1U;
				(void)k_msgq_put(&stream_q, &item, K_NO_WAIT);
				if (drop_log_budget > 0) {
					uint32_t used = (uint32_t)k_msgq_num_used_get(&stream_q);
					printk("Stream drop #%u (used=%u seq=%u)\n",
					       drop_new, used, seq);
					drop_log_budget--;
				}
			}
		}
	}

	/* Enqueue full-rate triple for UART; drop oldest if queue is full. */
	struct emg_sample_triple triple = {
		.raw = (uint16_t)raw,
		.filtered = out_fullrate_u16,
		.notch = (uint16_t)atomic_get(&latest_notch_u16),
		.envelope = env_last_u16,
	};
	uint32_t used = (uint32_t)k_msgq_num_used_get(&uart_sample_q);
	if (used > (uint32_t)atomic_get(&uart_q_hwm)) {
		atomic_set(&uart_q_hwm, (atomic_val_t)used);
	}
	if (k_msgq_put(&uart_sample_q, &triple, K_NO_WAIT) != 0) {
		struct emg_sample_triple dropped;
		(void)k_msgq_get(&uart_sample_q, &dropped, K_NO_WAIT);
		(void)k_msgq_put(&uart_sample_q, &triple, K_NO_WAIT);
		atomic_inc(&uart_q_drop);
	}
}

/*
 * SAADC event handler:
 * - Iterate completed buffer and process each sample.
 * - Re-queue the buffer for continuous sampling.
 */
static void saadc_evt_handler(nrfx_saadc_evt_t const *p_event)
{
	switch (p_event->type) {
	case NRFX_SAADC_EVT_DONE:
	{
		int16_t *buf = (int16_t *)p_event->data.done.p_buffer;
		for (uint16_t i = 0; i < p_event->data.done.size; i++) {
			process_sample(buf[i]);
		}
		/* Re-queue the processed buffer. */
		nrfx_err_t berr = nrfx_saadc_buffer_set(p_event->data.done.p_buffer, active_buf_size);
		if (berr != NRFX_SUCCESS && evt_logged < 3) {
			printk("SAADC buffer_set err=0x%x\n", berr);
		}
		if (evt_logged < 3) {
			printk("SAADC evt done size=%u first=%d\n",
			       p_event->data.done.size, buf[0]);
			evt_logged++;
		}
	}
		break;
	default:
		break;
	}
}

int emg_sampler_init(void)
{
	/* Reset state and counters first to avoid stale stats. */
	atomic_set(&latest_sample_u16, 0);
	atomic_set(&latest_seq, 0);
	atomic_set(&latest_raw_u16, 0);
	atomic_set(&latest_raw_seq, 0);
	atomic_set(&stream_seq, 0);
	atomic_set(&latest_notch_u16, 0);
	atomic_set(&latest_env_u16, 0);
	atomic_set(&stream_drop, 0);
	atomic_set(&raw_clip_hi, 0);
	atomic_set(&raw_clip_lo, 0);
	sample_count = 0;
	env_iir = 0;
	env_last_u16 = 0;
	hp_x1 = 0;
	hp_y1 = 0;
	evt_logged = 0;
	drop_log_budget = 8;

	/* Configure notch biquad(s) for the current sample rate. */
	int err = emg_notch_configure(&notch1, CONFIG_EYE_ADC_SAMPLE_RATE_HZ,
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

	err = nrfx_saadc_init(NRFX_SAADC_CONFIG_IRQ_PRIORITY);
	if (err != NRFX_SUCCESS) {
		printk("SAADC init err=0x%x\n", err);
		return -EIO;
	}

	/* Explicitly hook SAADC IRQ to avoid spurious interrupt (IRQ 7).
	 * Use a valid Zephyr priority (0-3 on nRF52); nrfx defaults are out of range here.
	 */
	IRQ_CONNECT(SAADC_IRQn, 1, nrfx_saadc_irq_handler, NULL, 0);
	irq_enable(SAADC_IRQn);

	/* Single-ended channel on AIN2 with internal reference. */
	nrfx_saadc_channel_t ch_cfg = NRFX_SAADC_DEFAULT_CHANNEL_SE(ADC_INPUT, ADC_CHANNEL_ID);
	ch_cfg.channel_config.gain = NRF_SAADC_GAIN1_5;
	ch_cfg.channel_config.acq_time = NRF_SAADC_ACQTIME_20US;
	ch_cfg.channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
	err = nrfx_saadc_channels_config(&ch_cfg, 1);
	if (err != NRFX_SUCCESS) {
		printk("SAADC channel cfg err=0x%x\n", err);
		return -EIO;
	}

	printk("SAADC cfg ok: gain=1/6 acq=20us bufsize=%u\n", SAADC_BUF_SIZE);
	printk("Sampler init: fs=%u decim=%u queue=%u\n",
	       CONFIG_EYE_ADC_SAMPLE_RATE_HZ, (unsigned int)decim, 40U);

	/* Advanced mode lets us use EasyDMA buffers + event callback. */
	nrfx_saadc_adv_config_t adv_cfg = NRFX_SAADC_DEFAULT_ADV_CONFIG;
	adv_cfg.oversampling = NRF_SAADC_OVERSAMPLE_DISABLED;
	adv_cfg.burst = NRF_SAADC_BURST_DISABLED;
	adv_cfg.internal_timer_cc = 0;
	adv_cfg.start_on_end = false;
	err = nrfx_saadc_advanced_mode_set(BIT(ADC_CHANNEL_ID),
					   NRF_SAADC_RESOLUTION_12BIT,
					   &adv_cfg,
					   saadc_evt_handler);
	if (err != NRFX_SUCCESS) {
		printk("SAADC adv_mode_set err=0x%x\n", err);
		return -EIO;
	}

	/* Double-buffering to keep sampling continuous. */
	err = nrfx_saadc_buffer_set(saadc_buf[0], SAADC_BUF_SIZE);
	if (err != NRFX_SUCCESS) {
		printk("SAADC buf0 set err=0x%x (size=%u)\n", err, SAADC_BUF_SIZE);
		if (err == NRFX_ERROR_INVALID_LENGTH) {
			active_buf_size = 1;
			printk("SAADC retry buf0 with size=%u\n", active_buf_size);
			err = nrfx_saadc_buffer_set(saadc_buf[0], active_buf_size);
		}
		if (err != NRFX_SUCCESS) {
			printk("SAADC buf0 retry failed err=0x%x\n", err);
			return -EIO;
		}
	}
	err = nrfx_saadc_buffer_set(saadc_buf[1], active_buf_size);
	if (err != NRFX_SUCCESS) {
		printk("SAADC buf1 set err=0x%x (size=%u)\n", err, active_buf_size);
		return -EIO;
	}
	printk("SAADC buffers armed size=%u\n", active_buf_size);

	/* Timer drives SAADC sampling (PPI from TIMER2 compare[0] -> SAADC SAMPLE). */
	const uint32_t timer_base_hz = NRFX_TIMER_BASE_FREQUENCY_GET(emg_timer.p_reg);
	const uint32_t timer_target_hz = timer_base_hz; /* always valid; prescaler=0 */
	printk("TIMER base=%u Hz target=%u Hz\n", (unsigned int)timer_base_hz,
	       (unsigned int)timer_target_hz);

	nrfx_timer_config_t tcfg = NRFX_TIMER_DEFAULT_CONFIG(timer_target_hz);
	tcfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
	printk("TIMER cfg: mode=%u bit_width=%u irq_prio=%u\n",
	       (unsigned int)tcfg.mode, (unsigned int)tcfg.bit_width,
	       (unsigned int)tcfg.interrupt_priority);
	err = nrfx_timer_init(&emg_timer, &tcfg, NULL);
	if (err != NRFX_SUCCESS) {
		printk("TIMER init err=0x%x\n", err);
		return -EIO;
	}

	/* Compare interval in microseconds for the requested fs. */
	uint32_t ticks = nrfx_timer_us_to_ticks(&emg_timer, 1000000U / CONFIG_EYE_ADC_SAMPLE_RATE_HZ);
	printk("TIMER ticks per sample=%u (fs=%u)\n", (unsigned int)ticks,
	       CONFIG_EYE_ADC_SAMPLE_RATE_HZ);
	nrfx_timer_clear(&emg_timer);
	nrfx_timer_extended_compare(&emg_timer, NRF_TIMER_CC_CHANNEL0, ticks,
				    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
	nrfx_timer_enable(&emg_timer);

	/* PPI: TIMER COMPARE -> SAADC SAMPLE. */
	err = nrfx_ppi_channel_alloc(&timer_saadc_ppi);
	if (err != NRFX_SUCCESS) {
		printk("PPI alloc timer->saadc err=0x%x\n", err);
		return -EIO;
	}
	err = nrfx_ppi_channel_assign(timer_saadc_ppi,
		nrfx_timer_compare_event_address_get(&emg_timer, NRF_TIMER_CC_CHANNEL0),
		nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));
	if (err != NRFX_SUCCESS) {
		printk("PPI assign timer->saadc err=0x%x\n", err);
		return -EIO;
	}
	err = nrfx_ppi_channel_enable(timer_saadc_ppi);
	if (err != NRFX_SUCCESS) {
		printk("PPI enable timer->saadc err=0x%x\n", err);
		return -EIO;
	}

	/* PPI: SAADC END -> START to keep sampling continuously with EasyDMA buffers. */
	/* PPI: SAADC END -> SAADC START for seamless DMA buffer cycling. */
	err = nrfx_ppi_channel_alloc(&saadc_start_ppi);
	if (err != NRFX_SUCCESS) {
		printk("PPI alloc end->start err=0x%x\n", err);
		return -EIO;
	}
	err = nrfx_ppi_channel_assign(saadc_start_ppi,
		nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
		nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));
	if (err != NRFX_SUCCESS) {
		printk("PPI assign end->start err=0x%x\n", err);
		return -EIO;
	}
	err = nrfx_ppi_channel_enable(saadc_start_ppi);
	if (err != NRFX_SUCCESS) {
		printk("PPI enable end->start err=0x%x\n", err);
		return -EIO;
	}

	/* Kick off first conversion; TIMER+PPI and END->START keep it running. */
	err = nrfx_saadc_mode_trigger();
	if (err != NRFX_SUCCESS) {
		printk("SAADC mode_trigger err=0x%x\n", err);
		return -EIO;
	}

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

int emg_sampler_get_uart_triple(struct emg_sample_triple *out, k_timeout_t timeout)
{
	if (!out) {
		return -EINVAL;
	}
	return k_msgq_get(&uart_sample_q, out, timeout);
}

uint16_t emg_sampler_get_latest_notch_u16(void)
{
	return (uint16_t)atomic_get(&latest_notch_u16);
}

uint32_t emg_sampler_get_uart_drop(void)
{
	return (uint32_t)atomic_get(&uart_q_drop);
}

uint32_t emg_sampler_get_uart_hwm(void)
{
	return (uint32_t)atomic_get(&uart_q_hwm);
}

uint32_t emg_sampler_pop_stream(uint32_t *first_seq, struct emg_stream_pair *dst,
				uint32_t max_samples)
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
		dst[count].filtered = item.filtered;
		dst[count].envelope = item.envelope;
		count++;
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

/* End of file */
