/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "emg_ble_stream.h"

#include <zephyr/sys/atomic.h>

static atomic_t last_sent_seq;
static atomic_t last_sent_sample_u16;
static atomic_t notify_ok;
static atomic_t notify_err;

void emg_ble_stream_note_sent(uint32_t seq, uint16_t sample)
{
	atomic_set(&last_sent_seq, (atomic_val_t)seq);
	atomic_set(&last_sent_sample_u16, (atomic_val_t)sample);
}

void emg_ble_stream_note_notify_ok(uint32_t samples)
{
	for (uint32_t i = 0; i < samples; i++) {
		atomic_inc(&notify_ok);
	}
}

void emg_ble_stream_note_notify_err(void)
{
	atomic_inc(&notify_err);
}

uint32_t emg_ble_stream_get_last_sent_seq(void)
{
	return (uint32_t)atomic_get(&last_sent_seq);
}

uint16_t emg_ble_stream_get_last_sent_sample(void)
{
	return (uint16_t)atomic_get(&last_sent_sample_u16);
}

uint32_t emg_ble_stream_get_notify_ok(void)
{
	return (uint32_t)atomic_get(&notify_ok);
}

uint32_t emg_ble_stream_get_notify_err(void)
{
	return (uint32_t)atomic_get(&notify_err);
}

