/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "emg_notch.h"

#include <stddef.h>
#include <zephyr/sys/printk.h>

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

	const struct coeff_entry *e = find_best_coeff(fs_hz, f0_hz, q);
	if (!e) {
		/* Fallback: bypass filter if unsupported fs/f0/Q (e.g. fs=4000). */
		state->b0 = (1 << 30);
		state->b1 = 0;
		state->b2 = 0;
		state->a1 = 0;
		state->a2 = 0;
		emg_notch_reset(state);
		printk("Notch coeff missing for fs=%u f0=%u q=%u, bypassing\n",
		       fs_hz, f0_hz, q);
		return 0;
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
