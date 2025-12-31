/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <stdint.h>

struct emg_notch_biquad_q30 {
	/* Q30 biquad coefficients (normalized). */
	int32_t b0;
	int32_t b1;
	int32_t b2;
	int32_t a1;
	int32_t a2;

	/* Filter state (Direct Form II Transposed). */
	int64_t s1;
	int64_t s2;
};

/* Reset filter state to zero. */
void emg_notch_reset(struct emg_notch_biquad_q30 *state);
/* Configure coefficients for fs/f0/q; uses table or computed fallback. */
int emg_notch_configure(struct emg_notch_biquad_q30 *state, uint32_t fs_hz,
			uint32_t f0_hz, uint32_t q);
/* Process one sample (integer input, integer output). */
int32_t emg_notch_process(struct emg_notch_biquad_q30 *state, int32_t x);
