/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>

struct emg_sample_triple {
	/* Raw ADC sample after clipping to [0..4095]. */
	uint16_t raw;
	/* Filtered sample (notch + DC-block), re-centered to mid-scale. */
	uint16_t filtered;
	/* Notch-filter output before DC-block re-centering (exported as u16). */
	uint16_t notch;
	/* Envelope of the filtered signal (rectified + smoothed). */
	uint16_t envelope;
} __packed;

/* Initialize SAADC sampling pipeline and filters. */
int emg_sampler_init(void);
/* Most recent decimated stream sequence number. */
uint32_t emg_sampler_get_latest_seq(void);
/* Most recent filtered (full-rate) sample in u16. */
uint16_t emg_sampler_get_latest_sample_u16(void);
/* Most recent raw sample sequence number. */
uint32_t emg_sampler_get_latest_raw_seq(void);
/* Most recent raw sample in u16. */
uint16_t emg_sampler_get_latest_raw_u16(void);
/* Pop one triple from the UART queue (full-rate). */
int emg_sampler_get_uart_triple(struct emg_sample_triple *out, k_timeout_t timeout);
/* Pop up to max_samples from the BLE stream queue. */
uint32_t emg_sampler_pop_stream(uint32_t *first_seq, uint16_t *dst, uint32_t max_samples);
/* Stream queue drop counter (decimated BLE stream). */
uint32_t emg_sampler_get_stream_drop_count(void);
/* Raw clip counters for low/high saturation. */
uint32_t emg_sampler_get_clip_hi(void);
uint32_t emg_sampler_get_clip_lo(void);
/* Most recent notch output in u16. */
uint16_t emg_sampler_get_latest_notch_u16(void);
/* UART queue drop/high-water counters. */
uint32_t emg_sampler_get_uart_drop(void);
uint32_t emg_sampler_get_uart_hwm(void);
