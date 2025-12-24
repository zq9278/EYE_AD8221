/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <stdint.h>

void emg_ble_stream_note_sent(uint32_t seq, uint16_t sample);
void emg_ble_stream_note_notify_ok(uint32_t samples);
void emg_ble_stream_note_notify_err(void);

uint32_t emg_ble_stream_get_last_sent_seq(void);
uint16_t emg_ble_stream_get_last_sent_sample(void);
uint32_t emg_ble_stream_get_notify_ok(void);
uint32_t emg_ble_stream_get_notify_err(void);

