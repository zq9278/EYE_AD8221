/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <stdint.h>

/* Record the last sequence/sample that made it onto the BLE stream. */
void emg_ble_stream_note_sent(uint32_t seq, uint16_t sample);
/* Count the number of samples successfully notified. */
void emg_ble_stream_note_notify_ok(uint32_t samples);
/* Count a notify error (link congestion/MTU/backpressure). */
void emg_ble_stream_note_notify_err(void);

/* Diagnostics: last sent sequence/sample and notify stats. */
uint32_t emg_ble_stream_get_last_sent_seq(void);
uint16_t emg_ble_stream_get_last_sent_sample(void);
uint32_t emg_ble_stream_get_notify_ok(void);
uint32_t emg_ble_stream_get_notify_err(void);
