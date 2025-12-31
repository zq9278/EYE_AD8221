/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "emg_notch.h"

#include <stddef.h>
#include <math.h>
#include <zephyr/sys/printk.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct coeff_entry {
	uint16_t fs_hz;
	uint8_t f0_hz;
	uint8_t q;
	int32_t b0;
	int32_t b1;
	int32_t b2;
	int32_t a1;
	int32_t a2;
};

/*
 * Precomputed biquad notch coefficients in Q30 for:
 * - fs: 1000/2000 Hz
 * - f0: 50/60 Hz
 * - Q: 20/25/30/35
 *
 * Coeffs follow the normalized form:
 *   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 */
static const struct coeff_entry coeff_table[] = {
	/* fs=1000, f0=50 */
	{ 1000, 50, 20, 1065510304, -2026721036, 1065510304, -2026721036, 1057278784 },
	{ 1000, 50, 25, 1067146496, -2029833258, 1067146496, -2029833258, 1060551168 },
	{ 1000, 50, 30, 1068240085, -2031913388, 1068240085, -2031913388, 1062738346 },
	{ 1000, 50, 35, 1069022593, -2033401807, 1069022593, -2033401807, 1064303363 },

	/* fs=1000, f0=60 */
	{ 1000, 60, 20, 1063950169, -1978471700, 1063950169, -1978471700, 1054158515 },
	{ 1000, 60, 25, 1065894188, -1982086704, 1065894188, -1982086704, 1058046551 },
	{ 1000, 60, 30, 1067194151, -1984504055, 1067194151, -1984504055, 1060646478 },
	{ 1000, 60, 35, 1068124640, -1986234348, 1068124640, -1986234348, 1062507455 },

	/* fs=2000, f0=50 */
	{ 2000, 50, 20, 1069558927, -2112781764, 1069558927, -2112781764, 1065376030 },
	{ 2000, 50, 25, 1070392897, -2114429169, 1070392897, -2114429169, 1067043970 },
	{ 2000, 50, 30, 1070949600, -2115528867, 1070949600, -2115528867, 1068157376 },
	{ 2000, 50, 35, 1071347600, -2116315066, 1071347600, -2116315066, 1068953376 },

	/* fs=2000, f0=60 */
	{ 2000, 60, 20, 1068735298, -2099610116, 1068735298, -2099610116, 1063728773 },
	{ 2000, 60, 25, 1069732865, -2101569910, 1069732865, -2101569910, 1065723906 },
	{ 2000, 60, 30, 1070398945, -2102878473, 1070398945, -2102878473, 1067056065 },
	{ 2000, 60, 35, 1070875224, -2103814159, 1070875224, -2103814159, 1068008624 },
};

/*
 * Pick the closest Q entry for a given fs/f0.
 * The table only supports the fixed fs/f0 pairs above.
 */
static const struct coeff_entry *find_best_coeff(uint32_t fs_hz, uint32_t f0_hz, uint32_t q)
{
	const struct coeff_entry *best = NULL;
	uint32_t best_dist = UINT32_MAX;

	for (size_t i = 0; i < (sizeof(coeff_table) / sizeof(coeff_table[0])); i++) {
		const struct coeff_entry *e = &coeff_table[i];

		if (e->fs_hz != fs_hz || e->f0_hz != f0_hz) {
			continue;
		}

		uint32_t dist = (e->q > q) ? (e->q - q) : (q - e->q);
		if (dist < best_dist) {
			best = e;
			best_dist = dist;
		}
	}

	return best;
}

/*
 * Fallback coefficient compute for arbitrary fs/f0/q (floating-point).
 * The resulting coefficients are converted to Q30 and used in the
 * integer biquad process function.
 */
static int compute_fallback_coeff(struct emg_notch_biquad_q30 *state,
				  uint32_t fs_hz, uint32_t f0_hz, uint32_t q)
{
	if (f0_hz == 0 || fs_hz == 0 || q == 0) {
		return -1;
	}

	double w0 = 2.0 * M_PI * ((double)f0_hz / (double)fs_hz);
	double cos_w0 = cos(w0);
	double alpha = sin(w0) / (2.0 * (double)q);

	double b0 = 1.0;
	double b1 = -2.0 * cos_w0;
	double b2 = 1.0;
	double a0 = 1.0 + alpha;
	double a1 = -2.0 * cos_w0;
	double a2 = 1.0 - alpha;

	/* Normalize by a0 */
	b0 /= a0;
	b1 /= a0;
	b2 /= a0;
	a1 /= a0;
	a2 /= a0;

	const double q30 = (double)(1ULL << 30);
	state->b0 = (int32_t)lrint(b0 * q30);
	state->b1 = (int32_t)lrint(b1 * q30);
	state->b2 = (int32_t)lrint(b2 * q30);
	state->a1 = (int32_t)lrint(a1 * q30);
	state->a2 = (int32_t)lrint(a2 * q30);
	emg_notch_reset(state);

	printk("Notch coeff computed (fs=%u f0=%u q=%u)\n", fs_hz, f0_hz, q);
	return 0;
}

void emg_notch_reset(struct emg_notch_biquad_q30 *state)
{
	if (!state) {
		return;
	}

	state->s1 = 0;
	state->s2 = 0;
}

int emg_notch_configure(struct emg_notch_biquad_q30 *state, uint32_t fs_hz,
			uint32_t f0_hz, uint32_t q)
{
	if (!state) {
		return -1;
	}

	/* Prefer precomputed coefficients when available; fall back otherwise. */
	const struct coeff_entry *e = find_best_coeff(fs_hz, f0_hz, q);
	if (!e) {
		return compute_fallback_coeff(state, fs_hz, f0_hz, q);
	}

	state->b0 = e->b0;
	state->b1 = e->b1;
	state->b2 = e->b2;
	state->a1 = e->a1;
	state->a2 = e->a2;
	emg_notch_reset(state);

	return 0;
}

int32_t emg_notch_process(struct emg_notch_biquad_q30 *state, int32_t x)
{
	/* Direct Form II Transposed, Q30 coefficients, integer samples (Q0). */
	int64_t acc = (int64_t)state->b0 * x + state->s1;
	int32_t y = (int32_t)(acc >> 30);

	state->s1 = (int64_t)state->b1 * x + state->s2 - (int64_t)state->a1 * y;
	state->s2 = (int64_t)state->b2 * x - (int64_t)state->a2 * y;

	return y;
}
