#include "vl53l1_ll_def.h"
#include "vl53l1_ll_device.h"
#include "vl53l1_platform_log.h"
#include "vl53l1_core_support.h"
#include "vl53l1_platform_user_data.h"
#include "vl53l1_platform_user_defines.h"
#ifdef VL53L1_LOGGING
#include "vl53l1_debug.h"
#include "vl53l1_register_debug.h"
#endif
#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(VL53L1_TRACE_MODULE_CORE, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(VL53L1_TRACE_MODULE_CORE, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...) \
	_LOG_FUNCTION_END_FMT(VL53L1_TRACE_MODULE_CORE, \
		status, fmt, ##__VA_ARGS__)
#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_CORE, \
	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)
uint32_t VL53L1_calc_pll_period_us(
	uint16_t  fast_osc_frequency)
{
	uint32_t  pll_period_us        = 0;
	LOG_FUNCTION_START("");
	pll_period_us = (0x01 << 30) / fast_osc_frequency;
#ifdef VL53L1_LOGGING
	trace_print(VL53L1_TRACE_LEVEL_DEBUG,
			"    %-48s : %10u\n", "pll_period_us",
			pll_period_us);
#endif
	LOG_FUNCTION_END(0);
	return pll_period_us;
}
#ifdef PAL_EXTENDED
uint32_t  VL53L1_duration_maths(
	uint32_t  pll_period_us,
	uint32_t  vcsel_parm_pclks,
	uint32_t  window_vclks,
	uint32_t  elapsed_mclks)
{
	uint64_t  tmp_long_int = 0;
	uint32_t  duration_us  = 0;
	duration_us = window_vclks * pll_period_us;
	duration_us = duration_us >> 12;
	tmp_long_int = (uint64_t)duration_us;
	duration_us = elapsed_mclks * vcsel_parm_pclks;
	duration_us = duration_us >> 4;
	tmp_long_int = tmp_long_int * (uint64_t)duration_us;
	tmp_long_int = tmp_long_int >> 12;
	if (tmp_long_int > 0xFFFFFFFF) {
		tmp_long_int = 0xFFFFFFFF;
	}
	duration_us  = (uint32_t)tmp_long_int;
	return duration_us;
}
uint32_t VL53L1_isqrt(uint32_t num)
{
	uint32_t  res = 0;
	uint32_t  bit = 1 << 30; 
	while (bit > num) {
		bit >>= 2;
	}
	while (bit != 0) {
		if (num >= res + bit)  {
			num -= res + bit;
			res = (res >> 1) + bit;
		} else {
			res >>= 1;
		}
		bit >>= 2;
	}
	return res;
}
uint16_t VL53L1_rate_maths(
	int32_t   events,
	uint32_t  time_us)
{
	uint32_t  tmp_int   = 0;
	uint32_t  frac_bits = 7;
	uint16_t  rate_mcps = 0; 
	if (events > VL53L1_SPAD_TOTAL_COUNT_MAX) {
		tmp_int = VL53L1_SPAD_TOTAL_COUNT_MAX;
	} else if (events > 0) {
		tmp_int = (uint32_t)events;
	}
	if (events > VL53L1_SPAD_TOTAL_COUNT_RES_THRES) {
		frac_bits = 3;
	} else {
		frac_bits = 7;
	}
	if (time_us > 0) {
		tmp_int = ((tmp_int << frac_bits) + (time_us / 2)) / time_us;
	}
	if (events > VL53L1_SPAD_TOTAL_COUNT_RES_THRES) {
		tmp_int = tmp_int << 4;
	}
	if (tmp_int > 0xFFFF) {
		tmp_int = 0xFFFF;
	}
	rate_mcps =  (uint16_t)tmp_int;
	return rate_mcps;
}
uint16_t VL53L1_rate_per_spad_maths(
	uint32_t  frac_bits,
	uint32_t  peak_count_rate,
	uint16_t  num_spads,
	uint32_t  max_output_value)
{
	uint32_t  tmp_int   = 0;
	uint16_t  rate_per_spad = 0;
	if (num_spads > 0) {
		tmp_int = (peak_count_rate << 8) << frac_bits;
		tmp_int = (tmp_int + ((uint32_t)num_spads / 2)) / (uint32_t)num_spads;
	} else {
		tmp_int = ((peak_count_rate) << frac_bits);
	}
	if (tmp_int > max_output_value) {
		tmp_int = max_output_value;
	}
	rate_per_spad = (uint16_t)tmp_int;
	return rate_per_spad;
}
int32_t VL53L1_range_maths(
	uint16_t  fast_osc_frequency,
	uint16_t  phase,
	uint16_t  zero_distance_phase,
	uint8_t   fractional_bits,
	int32_t   gain_factor,
	int32_t   range_offset_mm)
{
	uint32_t    pll_period_us = 0; 
	int64_t     tmp_long_int  = 0;
	int32_t     range_mm      = 0;
	pll_period_us  = VL53L1_calc_pll_period_us(fast_osc_frequency);
	tmp_long_int = (int64_t)phase - (int64_t)zero_distance_phase;
	tmp_long_int =  tmp_long_int * (int64_t)pll_period_us;
	tmp_long_int =  tmp_long_int / (0x01 << 9);
	tmp_long_int =  tmp_long_int * VL53L1_SPEED_OF_LIGHT_IN_AIR_DIV_8;
	tmp_long_int =  tmp_long_int / (0x01 << 22);
	range_mm  = (int32_t)tmp_long_int + range_offset_mm;
	range_mm *= gain_factor;
	range_mm += 0x0400;
	range_mm /= 0x0800;
	if (fractional_bits == 0)
		range_mm = range_mm / (0x01 << 2);
	else if (fractional_bits == 1)
		range_mm = range_mm / (0x01 << 1);
	return range_mm;
}
#endif
uint8_t VL53L1_decode_vcsel_period(uint8_t vcsel_period_reg)
{
	uint8_t vcsel_period_pclks = 0;
	vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
	return vcsel_period_pclks;
}
void VL53L1_decode_row_col(
	uint8_t  spad_number,
	uint8_t  *prow,
	uint8_t  *pcol)
{
	if (spad_number > 127) {
		*prow = 8 + ((255-spad_number) & 0x07);
		*pcol = (spad_number-128) >> 3;
	} else {
		*prow = spad_number & 0x07;
		*pcol = (127-spad_number) >> 3;
	}
}