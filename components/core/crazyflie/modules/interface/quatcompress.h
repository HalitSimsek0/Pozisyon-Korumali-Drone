#ifndef QUATCOMPRESS_H
#define QUATCOMPRESS_H
#include <stdint.h>
#include <math.h>
static inline uint32_t quatcompress(float const q[4])
{
	unsigned i_largest = 0;
	for (unsigned i = 1; i < 4; ++i) {
		if (fabsf(q[i]) > fabsf(q[i_largest])) {
			i_largest = i;
		}
	}
	unsigned negate = q[i_largest] < 0;
	uint32_t comp = i_largest;
	for (unsigned i = 0; i < 4; ++i) {
		if (i != i_largest) {
			unsigned negbit = (q[i] < 0) ^ negate;
			unsigned mag = ((1 << 9) - 1) * (fabsf(q[i]) / (float)M_SQRT1_2) + 0.5f;
			comp = (comp << 10) | (negbit << 9) | mag;
		}
	}
	return comp;
}
static inline void quatdecompress(uint32_t comp, float q[4])
{
	unsigned const mask = (1 << 9) - 1;
	int const i_largest = comp >> 30;
	float sum_squares = 0;
	for (int i = 3; i >= 0; --i) {
		if (i != i_largest) {
			unsigned mag = comp & mask;
			unsigned negbit = (comp >> 9) & 0x1;
			comp = comp >> 10;
			q[i] = ((float)M_SQRT1_2) * ((float)mag) / mask;
			if (negbit == 1) {
				q[i] = -q[i];
			}
			sum_squares += q[i] * q[i];
		}
	}
	q[i_largest] = sqrtf(1.0f - sum_squares);
}
#endif 