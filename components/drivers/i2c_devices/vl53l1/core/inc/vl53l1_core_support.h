#ifndef _VL53L1_CORE_SUPPORT_H_
#define _VL53L1_CORE_SUPPORT_H_
#include "vl53l1_types.h"
#ifdef __cplusplus
extern "C" {
#endif
uint32_t VL53L1_calc_pll_period_us(
	uint16_t fast_osc_frequency);
#ifdef PAL_EXTENDED
uint32_t VL53L1_duration_maths(
	uint32_t  pll_period_us,
	uint32_t  vcsel_parm_pclks,
	uint32_t  window_vclks,
	uint32_t  periods_elapsed_mclks);
uint32_t VL53L1_isqrt(
	uint32_t  num);
uint16_t VL53L1_rate_maths(
	int32_t   events,
	uint32_t  time_us);
uint16_t VL53L1_rate_per_spad_maths(
	uint32_t  frac_bits,
	uint32_t  peak_count_rate,
	uint16_t  num_spads,
	uint32_t  max_output_value);
int32_t VL53L1_range_maths(
	uint16_t  fast_osc_frequency,
	uint16_t  phase,
	uint16_t  zero_distance_phase,
	uint8_t   fractional_bits,
	int32_t   gain_factor,
	int32_t   range_offset_mm);
#endif
uint8_t VL53L1_decode_vcsel_period(
	uint8_t vcsel_period_reg);
void VL53L1_decode_row_col(
	uint8_t   spad_number,
	uint8_t  *prow,
	uint8_t  *pcol);
#ifdef __cplusplus
}
#endif
#endif 