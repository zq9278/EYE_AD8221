/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <stdint.h>

int emg_sampler_init(void);
uint32_t emg_sampler_get_latest_seq(void);
uint16_t emg_sampler_get_latest_sample_u16(void);
uint32_t emg_sampler_get_latest_raw_seq(void);
uint16_t emg_sampler_get_latest_raw_u16(void);
uint32_t emg_sampler_pop_stream(uint32_t *first_seq, uint16_t *dst, uint32_t max_samples);
uint32_t emg_sampler_get_stream_drop_count(void);
uint32_t emg_sampler_get_clip_hi(void);
uint32_t emg_sampler_get_clip_lo(void);
uint16_t emg_sampler_get_latest_notch_u16(void);
