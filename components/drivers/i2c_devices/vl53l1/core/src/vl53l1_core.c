#include "vl53l1_ll_def.h"
#include "vl53l1_ll_device.h"
#include "vl53l1_platform.h"
#include "vl53l1_register_map.h"
#include "vl53l1_register_funcs.h"
#include "vl53l1_register_settings.h"
#include "vl53l1_api_preset_modes.h"
#include "vl53l1_core.h"
#include "vl53l1_tuning_parm_defaults.h"
#ifdef VL53L1_LOGGING
#include "vl53l1_api_debug.h"
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
void  VL53L1_init_version(
	VL53L1_DEV        Dev)
{
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	pdev->version.ll_major    = VL53L1_LL_API_IMPLEMENTATION_VER_MAJOR;
	pdev->version.ll_minor    = VL53L1_LL_API_IMPLEMENTATION_VER_MINOR;
	pdev->version.ll_build    = VL53L1_LL_API_IMPLEMENTATION_VER_SUB;
	pdev->version.ll_revision = VL53L1_LL_API_IMPLEMENTATION_VER_REVISION;
}
void  VL53L1_init_ll_driver_state(
	VL53L1_DEV         Dev,
	VL53L1_DeviceState device_state)
{
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	VL53L1_ll_driver_state_t *pstate = &(pdev->ll_state);
	pstate->cfg_device_state  = device_state;
	pstate->cfg_stream_count  = 0;
	pstate->cfg_gph_id        = VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;
	pstate->cfg_timing_status = 0;
	pstate->rd_device_state   = device_state;
	pstate->rd_stream_count   = 0;
	pstate->rd_gph_id         = VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;
	pstate->rd_timing_status  = 0;
}
VL53L1_Error  VL53L1_update_ll_driver_rd_state(
	VL53L1_DEV         Dev)
{
	VL53L1_Error        status  = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	VL53L1_ll_driver_state_t *pstate = &(pdev->ll_state);
	LOG_FUNCTION_START("");
#ifdef VL53L1_LOGGING
	VL53L1_print_ll_driver_state(pstate);
#endif
	if ((pdev->sys_ctrl.system__mode_start &
		VL53L1_DEVICEMEASUREMENTMODE_MODE_MASK) == 0x00) {
		pstate->rd_device_state  = VL53L1_DEVICESTATE_SW_STANDBY;
		pstate->rd_stream_count  = 0;
		pstate->rd_gph_id = VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;
		pstate->rd_timing_status = 0;
	} else {
		if (pstate->rd_stream_count == 0xFF) {
			pstate->rd_stream_count = 0x80;
		} else {
			pstate->rd_stream_count++;
		}
		pstate->rd_gph_id ^= VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;
		switch (pstate->rd_device_state) {
		case VL53L1_DEVICESTATE_SW_STANDBY:
			if ((pdev->dyn_cfg.system__grouped_parameter_hold &
				VL53L1_GROUPEDPARAMETERHOLD_ID_MASK) > 0) {
				pstate->rd_device_state =
					VL53L1_DEVICESTATE_RANGING_WAIT_GPH_SYNC;
			} else {
				pstate->rd_device_state =
					VL53L1_DEVICESTATE_RANGING_OUTPUT_DATA;
			}
			pstate->rd_stream_count  = 0;
			pstate->rd_timing_status = 0;
		break;
		case VL53L1_DEVICESTATE_RANGING_WAIT_GPH_SYNC:
			pstate->rd_stream_count = 0;
			pstate->rd_device_state =
				VL53L1_DEVICESTATE_RANGING_OUTPUT_DATA;
		break;
		case VL53L1_DEVICESTATE_RANGING_GATHER_DATA:
			pstate->rd_device_state =
				VL53L1_DEVICESTATE_RANGING_OUTPUT_DATA;
		break;
		case VL53L1_DEVICESTATE_RANGING_OUTPUT_DATA:
			pstate->rd_timing_status ^= 0x01;
			pstate->rd_device_state =
				VL53L1_DEVICESTATE_RANGING_OUTPUT_DATA;
		break;
		default:
			pstate->rd_device_state  =
				VL53L1_DEVICESTATE_SW_STANDBY;
			pstate->rd_stream_count  = 0;
			pstate->rd_gph_id = VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;
			pstate->rd_timing_status = 0;
		break;
		}
	}
#ifdef VL53L1_LOGGING
	VL53L1_print_ll_driver_state(pstate);
#endif
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_check_ll_driver_rd_state(
	VL53L1_DEV         Dev)
{
	VL53L1_Error         status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t  *pdev =
			VL53L1DevStructGetLLDriverHandle(Dev);
	VL53L1_ll_driver_state_t  *pstate       = &(pdev->ll_state);
	VL53L1_system_results_t   *psys_results = &(pdev->sys_results);
	uint8_t   device_range_status   = 0;
	uint8_t   device_stream_count   = 0;
	uint8_t   device_gph_id         = 0;
	LOG_FUNCTION_START("");
#ifdef VL53L1_LOGGING
	VL53L1_print_ll_driver_state(pstate);
#endif
	device_range_status =
			psys_results->result__range_status &
			VL53L1_RANGE_STATUS__RANGE_STATUS_MASK;
	device_stream_count = psys_results->result__stream_count;
	device_gph_id = (psys_results->result__interrupt_status &
		VL53L1_INTERRUPT_STATUS__GPH_ID_INT_STATUS_MASK) >> 4;
	if ((pdev->sys_ctrl.system__mode_start &
		VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK) ==
		VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK) {
		if (pstate->rd_device_state ==
			VL53L1_DEVICESTATE_RANGING_WAIT_GPH_SYNC) {
			if (device_range_status !=
				VL53L1_DEVICEERROR_GPHSTREAMCOUNT0READY) {
				status = VL53L1_ERROR_GPH_SYNC_CHECK_FAIL;
			}
		} else {
			if (pstate->rd_stream_count != device_stream_count) {
				status = VL53L1_ERROR_STREAM_COUNT_CHECK_FAIL;
			}
		if (pstate->rd_gph_id != device_gph_id) {
			status = VL53L1_ERROR_GPH_ID_CHECK_FAIL;
#ifdef VL53L1_LOGGING
				trace_print(VL53L1_TRACE_LEVEL_ALL,
					"    RDSTATECHECK: Check failed: rd_gph_id: %d, device_gph_id: %d\n",
					pstate->rd_gph_id,
					device_gph_id);
#endif
			} else {
#ifdef VL53L1_LOGGING
				trace_print(VL53L1_TRACE_LEVEL_ALL,
					"    RDSTATECHECK: Check passed: rd_gph_id: %d, device_gph_id: %d\n",
					pstate->rd_gph_id,
					device_gph_id);
#endif
			}
		} 
	} 
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error  VL53L1_update_ll_driver_cfg_state(
	VL53L1_DEV         Dev)
{
	VL53L1_Error         status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t  *pdev =
			VL53L1DevStructGetLLDriverHandle(Dev);
	VL53L1_ll_driver_state_t *pstate = &(pdev->ll_state);
	LOG_FUNCTION_START("");
#ifdef VL53L1_LOGGING
	VL53L1_print_ll_driver_state(pstate);
#endif
	if ((pdev->sys_ctrl.system__mode_start &
		VL53L1_DEVICEMEASUREMENTMODE_MODE_MASK) == 0x00) {
		pstate->cfg_device_state  = VL53L1_DEVICESTATE_SW_STANDBY;
		pstate->cfg_stream_count  = 0;
		pstate->cfg_gph_id = VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;
		pstate->cfg_timing_status = 0;
	} else {
		if (pstate->cfg_stream_count == 0xFF) {
			pstate->cfg_stream_count = 0x80;
		} else {
			pstate->cfg_stream_count++;
		}
		pstate->cfg_gph_id ^= VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;
		switch (pstate->cfg_device_state) {
		case VL53L1_DEVICESTATE_SW_STANDBY:
			pstate->cfg_timing_status ^= 0x01;
			pstate->cfg_stream_count = 1;
			pstate->cfg_device_state = VL53L1_DEVICESTATE_RANGING_DSS_AUTO;
		break;
		case VL53L1_DEVICESTATE_RANGING_DSS_AUTO:
			pstate->cfg_timing_status ^= 0x01;
		break;
		default:
			pstate->cfg_device_state = VL53L1_DEVICESTATE_SW_STANDBY;
			pstate->cfg_stream_count = 0;
			pstate->cfg_gph_id = VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;
			pstate->cfg_timing_status = 0;
		break;
		}
	}
#ifdef VL53L1_LOGGING
	VL53L1_print_ll_driver_state(pstate);
#endif
	LOG_FUNCTION_END(status);
	return status;
}
void VL53L1_copy_rtn_good_spads_to_buffer(
	VL53L1_nvm_copy_data_t  *pdata,
	uint8_t                 *pbuffer)
{
	*(pbuffer +  0) = pdata->global_config__spad_enables_rtn_0;
	*(pbuffer +  1) = pdata->global_config__spad_enables_rtn_1;
	*(pbuffer +  2) = pdata->global_config__spad_enables_rtn_2;
	*(pbuffer +  3) = pdata->global_config__spad_enables_rtn_3;
	*(pbuffer +  4) = pdata->global_config__spad_enables_rtn_4;
	*(pbuffer +  5) = pdata->global_config__spad_enables_rtn_5;
	*(pbuffer +  6) = pdata->global_config__spad_enables_rtn_6;
	*(pbuffer +  7) = pdata->global_config__spad_enables_rtn_7;
	*(pbuffer +  8) = pdata->global_config__spad_enables_rtn_8;
	*(pbuffer +  9) = pdata->global_config__spad_enables_rtn_9;
	*(pbuffer + 10) = pdata->global_config__spad_enables_rtn_10;
	*(pbuffer + 11) = pdata->global_config__spad_enables_rtn_11;
	*(pbuffer + 12) = pdata->global_config__spad_enables_rtn_12;
	*(pbuffer + 13) = pdata->global_config__spad_enables_rtn_13;
	*(pbuffer + 14) = pdata->global_config__spad_enables_rtn_14;
	*(pbuffer + 15) = pdata->global_config__spad_enables_rtn_15;
	*(pbuffer + 16) = pdata->global_config__spad_enables_rtn_16;
	*(pbuffer + 17) = pdata->global_config__spad_enables_rtn_17;
	*(pbuffer + 18) = pdata->global_config__spad_enables_rtn_18;
	*(pbuffer + 19) = pdata->global_config__spad_enables_rtn_19;
	*(pbuffer + 20) = pdata->global_config__spad_enables_rtn_20;
	*(pbuffer + 21) = pdata->global_config__spad_enables_rtn_21;
	*(pbuffer + 22) = pdata->global_config__spad_enables_rtn_22;
	*(pbuffer + 23) = pdata->global_config__spad_enables_rtn_23;
	*(pbuffer + 24) = pdata->global_config__spad_enables_rtn_24;
	*(pbuffer + 25) = pdata->global_config__spad_enables_rtn_25;
	*(pbuffer + 26) = pdata->global_config__spad_enables_rtn_26;
	*(pbuffer + 27) = pdata->global_config__spad_enables_rtn_27;
	*(pbuffer + 28) = pdata->global_config__spad_enables_rtn_28;
	*(pbuffer + 29) = pdata->global_config__spad_enables_rtn_29;
	*(pbuffer + 30) = pdata->global_config__spad_enables_rtn_30;
	*(pbuffer + 31) = pdata->global_config__spad_enables_rtn_31;
}
void VL53L1_init_system_results(
		VL53L1_system_results_t  *pdata)
{
	pdata->result__interrupt_status                       = 0xFF;
	pdata->result__range_status                           = 0xFF;
	pdata->result__report_status                          = 0xFF;
	pdata->result__stream_count                           = 0xFF;
	pdata->result__dss_actual_effective_spads_sd0         = 0xFFFF;
	pdata->result__peak_signal_count_rate_mcps_sd0        = 0xFFFF;
	pdata->result__ambient_count_rate_mcps_sd0            = 0xFFFF;
	pdata->result__sigma_sd0                              = 0xFFFF;
	pdata->result__phase_sd0                              = 0xFFFF;
	pdata->result__final_crosstalk_corrected_range_mm_sd0 = 0xFFFF;
	pdata->result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 =
			0xFFFF;
	pdata->result__mm_inner_actual_effective_spads_sd0    = 0xFFFF;
	pdata->result__mm_outer_actual_effective_spads_sd0    = 0xFFFF;
	pdata->result__avg_signal_count_rate_mcps_sd0         = 0xFFFF;
	pdata->result__dss_actual_effective_spads_sd1         = 0xFFFF;
	pdata->result__peak_signal_count_rate_mcps_sd1        = 0xFFFF;
	pdata->result__ambient_count_rate_mcps_sd1            = 0xFFFF;
	pdata->result__sigma_sd1                              = 0xFFFF;
	pdata->result__phase_sd1                              = 0xFFFF;
	pdata->result__final_crosstalk_corrected_range_mm_sd1 = 0xFFFF;
	pdata->result__spare_0_sd1                            = 0xFFFF;
	pdata->result__spare_1_sd1                            = 0xFFFF;
	pdata->result__spare_2_sd1                            = 0xFFFF;
	pdata->result__spare_3_sd1                            = 0xFF;
}
void VL53L1_i2c_encode_uint16_t(
	uint16_t    ip_value,
	uint16_t    count,
	uint8_t    *pbuffer)
{
	uint16_t   i    = 0;
	uint16_t   data = 0;
	data =  ip_value;
	for (i = 0; i < count ; i++) {
		pbuffer[count-i-1] = (uint8_t)(data & 0x00FF);
		data = data >> 8;
	}
}
uint16_t VL53L1_i2c_decode_uint16_t(
	uint16_t    count,
	uint8_t    *pbuffer)
{
	uint16_t   value = 0x00;
	while (count-- > 0) {
		value = (value << 8) | (uint16_t)*pbuffer++;
	}
	return value;
}
void VL53L1_i2c_encode_int16_t(
	int16_t     ip_value,
	uint16_t    count,
	uint8_t    *pbuffer)
{
	uint16_t   i    = 0;
	int16_t    data = 0;
	data =  ip_value;
	for (i = 0; i < count ; i++) {
		pbuffer[count-i-1] = (uint8_t)(data & 0x00FF);
		data = data >> 8;
	}
}
int16_t VL53L1_i2c_decode_int16_t(
	uint16_t    count,
	uint8_t    *pbuffer)
{
	int16_t    value = 0x00;
	if (*pbuffer >= 0x80) {
		value = 0xFFFF;
	}
	while (count-- > 0) {
		value = (value << 8) | (int16_t)*pbuffer++;
	}
	return value;
}
void VL53L1_i2c_encode_uint32_t(
	uint32_t    ip_value,
	uint16_t    count,
	uint8_t    *pbuffer)
{
	uint16_t   i    = 0;
	uint32_t   data = 0;
	data =  ip_value;
	for (i = 0; i < count ; i++) {
		pbuffer[count-i-1] = (uint8_t)(data & 0x00FF);
		data = data >> 8;
	}
}
uint32_t VL53L1_i2c_decode_uint32_t(
	uint16_t    count,
	uint8_t    *pbuffer)
{
	uint32_t   value = 0x00;
	while (count-- > 0) {
		value = (value << 8) | (uint32_t)*pbuffer++;
	}
	return value;
}
uint32_t VL53L1_i2c_decode_with_mask(
	uint16_t    count,
	uint8_t    *pbuffer,
	uint32_t    bit_mask,
	uint32_t    down_shift,
	uint32_t    offset)
{
	uint32_t   value = 0x00;
	while (count-- > 0) {
		value = (value << 8) | (uint32_t)*pbuffer++;
	}
	value =  value & bit_mask;
	if (down_shift > 0) {
		value = value >> down_shift;
	}
	value = value + offset;
	return value;
}
void VL53L1_i2c_encode_int32_t(
	int32_t     ip_value,
	uint16_t    count,
	uint8_t    *pbuffer)
{
	uint16_t   i    = 0;
	int32_t    data = 0;
	data =  ip_value;
	for (i = 0; i < count ; i++) {
		pbuffer[count-i-1] = (uint8_t)(data & 0x00FF);
		data = data >> 8;
	}
}
int32_t VL53L1_i2c_decode_int32_t(
	uint16_t    count,
	uint8_t    *pbuffer)
{
	int32_t    value = 0x00;
	if (*pbuffer >= 0x80) {
		value = 0xFFFFFFFF;
	}
	while (count-- > 0) {
		value = (value << 8) | (int32_t)*pbuffer++;
	}
	return value;
}
#ifndef VL53L1_NOCALIB
VL53L1_Error VL53L1_start_test(
	VL53L1_DEV    Dev,
	uint8_t       test_mode__ctrl)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	LOG_FUNCTION_START("");
	if (status == VL53L1_ERROR_NONE) { 
		status = VL53L1_WrByte(
					Dev,
					VL53L1_TEST_MODE__CTRL,
					test_mode__ctrl);
	}
	LOG_FUNCTION_END(status);
	return status;
}
#endif
VL53L1_Error VL53L1_set_firmware_enable_register(
	VL53L1_DEV    Dev,
	uint8_t       value)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	pdev->sys_ctrl.firmware__enable = value;
	status = VL53L1_WrByte(
				Dev,
				VL53L1_FIRMWARE__ENABLE,
				pdev->sys_ctrl.firmware__enable);
	return status;
}
VL53L1_Error VL53L1_enable_firmware(
	VL53L1_DEV    Dev)
{
	VL53L1_Error status       = VL53L1_ERROR_NONE;
	LOG_FUNCTION_START("");
	status = VL53L1_set_firmware_enable_register(Dev, 0x01);
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_disable_firmware(
	VL53L1_DEV    Dev)
{
	VL53L1_Error status       = VL53L1_ERROR_NONE;
	LOG_FUNCTION_START("");
	status = VL53L1_set_firmware_enable_register(Dev, 0x00);
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_powerforce_register(
	VL53L1_DEV    Dev,
	uint8_t       value)
{
	VL53L1_Error status       = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	pdev->sys_ctrl.power_management__go1_power_force = value;
	status = VL53L1_WrByte(
			Dev,
			VL53L1_POWER_MANAGEMENT__GO1_POWER_FORCE,
			pdev->sys_ctrl.power_management__go1_power_force);
	return status;
}
VL53L1_Error VL53L1_enable_powerforce(
	VL53L1_DEV    Dev)
{
	VL53L1_Error status       = VL53L1_ERROR_NONE;
	LOG_FUNCTION_START("");
	status = VL53L1_set_powerforce_register(Dev, 0x01);
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_disable_powerforce(
	VL53L1_DEV    Dev)
{
	VL53L1_Error status       = VL53L1_ERROR_NONE;
	LOG_FUNCTION_START("");
	status = VL53L1_set_powerforce_register(Dev, 0x00);
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_clear_interrupt(
	VL53L1_DEV    Dev)
{
	VL53L1_Error status       = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdev->sys_ctrl.system__interrupt_clear = VL53L1_CLEAR_RANGE_INT;
	status = VL53L1_WrByte(
					Dev,
					VL53L1_SYSTEM__INTERRUPT_CLEAR,
					pdev->sys_ctrl.system__interrupt_clear);
	LOG_FUNCTION_END(status);
	return status;
}
#ifdef VL53L1_DEBUG
VL53L1_Error VL53L1_force_shadow_stream_count_to_zero(
	VL53L1_DEV    Dev)
{
	VL53L1_Error status       = VL53L1_ERROR_NONE;
	if (status == VL53L1_ERROR_NONE) { 
		status = VL53L1_disable_firmware(Dev);
	}
	if (status == VL53L1_ERROR_NONE) {
		status = VL53L1_WrByte(
				Dev,
				VL53L1_SHADOW_RESULT__STREAM_COUNT,
				0x00);
	}
	if (status == VL53L1_ERROR_NONE) {
		status = VL53L1_enable_firmware(Dev);
	}
	return status;
}
#endif
uint32_t VL53L1_calc_macro_period_us(
	uint16_t  fast_osc_frequency,
	uint8_t   vcsel_period)
{
	uint32_t  pll_period_us        = 0;
	uint8_t   vcsel_period_pclks   = 0;
	uint32_t  macro_period_us      = 0;
	LOG_FUNCTION_START("");
	pll_period_us = VL53L1_calc_pll_period_us(fast_osc_frequency);
	vcsel_period_pclks = VL53L1_decode_vcsel_period(vcsel_period);
	macro_period_us =
			(uint32_t)VL53L1_MACRO_PERIOD_VCSEL_PERIODS *
			pll_period_us;
	macro_period_us = macro_period_us >> 6;
	macro_period_us = macro_period_us * (uint32_t)vcsel_period_pclks;
	macro_period_us = macro_period_us >> 6;
#ifdef VL53L1_LOGGING
	trace_print(VL53L1_TRACE_LEVEL_DEBUG,
			"    %-48s : %10u\n", "pll_period_us",
			pll_period_us);
	trace_print(VL53L1_TRACE_LEVEL_DEBUG,
			"    %-48s : %10u\n", "vcsel_period_pclks",
			vcsel_period_pclks);
	trace_print(VL53L1_TRACE_LEVEL_DEBUG,
			"    %-48s : %10u\n", "macro_period_us",
			macro_period_us);
#endif
	LOG_FUNCTION_END(0);
	return macro_period_us;
}
uint16_t VL53L1_calc_range_ignore_threshold(
	uint32_t central_rate,
	int16_t  x_gradient,
	int16_t  y_gradient,
	uint8_t  rate_mult)
{
	int32_t    range_ignore_thresh_int  = 0;
	uint16_t   range_ignore_thresh_kcps = 0;
	int32_t    central_rate_int         = 0;
	int16_t    x_gradient_int           = 0;
	int16_t    y_gradient_int           = 0;
	LOG_FUNCTION_START("");
	central_rate_int = ((int32_t)central_rate * (1 << 4)) / (1000);
	if (x_gradient < 0) {
		x_gradient_int = x_gradient * -1;
	}
	if (y_gradient < 0) {
		y_gradient_int = y_gradient * -1;
	}
	range_ignore_thresh_int = (8 * x_gradient_int * 4) + (8 * y_gradient_int * 4);
	range_ignore_thresh_int = range_ignore_thresh_int / 1000;
	range_ignore_thresh_int = range_ignore_thresh_int + central_rate_int;
	range_ignore_thresh_int = (int32_t)rate_mult * range_ignore_thresh_int;
	range_ignore_thresh_int = (range_ignore_thresh_int + (1<<4)) / (1<<5);
	if (range_ignore_thresh_int > 0xFFFF) {
		range_ignore_thresh_kcps = 0xFFFF;
	} else {
		range_ignore_thresh_kcps = (uint16_t)range_ignore_thresh_int;
	}
#ifdef VL53L1_LOGGING
	trace_print(VL53L1_TRACE_LEVEL_DEBUG,
			"    %-48s : %10u\n", "range_ignore_thresh_kcps",
			range_ignore_thresh_kcps);
#endif
	LOG_FUNCTION_END(0);
	return range_ignore_thresh_kcps;
}
uint32_t VL53L1_calc_timeout_mclks(
	uint32_t timeout_us,
	uint32_t macro_period_us)
{
	uint32_t timeout_mclks   = 0;
	LOG_FUNCTION_START("");
	timeout_mclks   =
			((timeout_us << 12) + (macro_period_us>>1)) /
			macro_period_us;
	LOG_FUNCTION_END(0);
	return timeout_mclks;
}
uint16_t VL53L1_calc_encoded_timeout(
	uint32_t timeout_us,
	uint32_t macro_period_us)
{
	uint32_t timeout_mclks   = 0;
	uint16_t timeout_encoded = 0;
	LOG_FUNCTION_START("");
	timeout_mclks   =
		VL53L1_calc_timeout_mclks(timeout_us, macro_period_us);
	timeout_encoded =
		VL53L1_encode_timeout(timeout_mclks);
#ifdef VL53L1_LOGGING
	trace_print(VL53L1_TRACE_LEVEL_DEBUG,
			"    %-48s : %10u  (0x%04X)\n", "timeout_mclks",
			timeout_mclks, timeout_mclks);
	trace_print(VL53L1_TRACE_LEVEL_DEBUG,
			"    %-48s : %10u  (0x%04X)\n", "timeout_encoded",
			timeout_encoded, timeout_encoded);
#endif
	LOG_FUNCTION_END(0);
	return timeout_encoded;
}
uint32_t VL53L1_calc_timeout_us(
	uint32_t timeout_mclks,
	uint32_t macro_period_us)
{
	uint32_t timeout_us     = 0;
	uint64_t tmp            = 0;
	LOG_FUNCTION_START("");
	tmp  = (uint64_t)timeout_mclks * (uint64_t)macro_period_us;
	tmp += 0x00800;
	tmp  = tmp >> 12;
	timeout_us = (uint32_t)tmp;
#ifdef VL53L1_LOGGING
	trace_print(VL53L1_TRACE_LEVEL_DEBUG,
			"    %-48s : %10u  (0x%04X)\n", "timeout_mclks",
			timeout_mclks, timeout_mclks);
	trace_print(VL53L1_TRACE_LEVEL_DEBUG,
			"    %-48s : %10u us\n", "timeout_us",
			timeout_us, timeout_us);
#endif
	LOG_FUNCTION_END(0);
	return timeout_us;
}
uint32_t VL53L1_calc_crosstalk_plane_offset_with_margin(
		uint32_t     plane_offset_kcps,
		int16_t      margin_offset_kcps)
{
	uint32_t plane_offset_with_margin = 0;
	int32_t  plane_offset_kcps_temp   = 0;
	LOG_FUNCTION_START("");
	plane_offset_kcps_temp =
		(int32_t)plane_offset_kcps +
		(int32_t)margin_offset_kcps;
	if (plane_offset_kcps_temp < 0) {
		plane_offset_kcps_temp = 0;
	} else {
		if (plane_offset_kcps_temp > 0x3FFFF) {
			plane_offset_kcps_temp = 0x3FFFF;
		}
	}
	plane_offset_with_margin = (uint32_t) plane_offset_kcps_temp;
	LOG_FUNCTION_END(0);
	return plane_offset_with_margin;
}
uint32_t VL53L1_calc_decoded_timeout_us(
	uint16_t timeout_encoded,
	uint32_t macro_period_us)
{
	uint32_t timeout_mclks  = 0;
	uint32_t timeout_us     = 0;
	LOG_FUNCTION_START("");
	timeout_mclks =
		VL53L1_decode_timeout(timeout_encoded);
	timeout_us    =
		VL53L1_calc_timeout_us(timeout_mclks, macro_period_us);
	LOG_FUNCTION_END(0);
	return timeout_us;
}
uint16_t VL53L1_encode_timeout(uint32_t timeout_mclks)
{
	uint16_t encoded_timeout = 0;
	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;
	if (timeout_mclks > 0) {
		ls_byte = timeout_mclks - 1;
		while ((ls_byte & 0xFFFFFF00) > 0) {
			ls_byte = ls_byte >> 1;
			ms_byte++;
		}
		encoded_timeout = (ms_byte << 8)
				+ (uint16_t) (ls_byte & 0x000000FF);
	}
	return encoded_timeout;
}
uint32_t VL53L1_decode_timeout(uint16_t encoded_timeout)
{
	uint32_t timeout_macro_clks = 0;
	timeout_macro_clks = ((uint32_t) (encoded_timeout & 0x00FF)
			<< (uint32_t) ((encoded_timeout & 0xFF00) >> 8)) + 1;
	return timeout_macro_clks;
}
VL53L1_Error VL53L1_calc_timeout_register_values(
	uint32_t                 phasecal_config_timeout_us,
	uint32_t                 mm_config_timeout_us,
	uint32_t                 range_config_timeout_us,
	uint16_t                 fast_osc_frequency,
	VL53L1_general_config_t *pgeneral,
	VL53L1_timing_config_t  *ptiming)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint32_t macro_period_us    = 0;
	uint32_t timeout_mclks      = 0;
	uint16_t timeout_encoded    = 0;
	LOG_FUNCTION_START("");
	if (fast_osc_frequency == 0) {
		status = VL53L1_ERROR_DIVISION_BY_ZERO;
	} else {
		macro_period_us =
				VL53L1_calc_macro_period_us(
					fast_osc_frequency,
					ptiming->range_config__vcsel_period_a);
		timeout_mclks =
			VL53L1_calc_timeout_mclks(
				phasecal_config_timeout_us,
				macro_period_us);
		if (timeout_mclks > 0xFF)
			timeout_mclks = 0xFF;
		pgeneral->phasecal_config__timeout_macrop =
				(uint8_t)timeout_mclks;
		timeout_encoded =
			VL53L1_calc_encoded_timeout(
				mm_config_timeout_us,
				macro_period_us);
		ptiming->mm_config__timeout_macrop_a_hi =
				(uint8_t)((timeout_encoded & 0xFF00) >> 8);
		ptiming->mm_config__timeout_macrop_a_lo =
				(uint8_t) (timeout_encoded & 0x00FF);
		timeout_encoded =
			VL53L1_calc_encoded_timeout(
				range_config_timeout_us,
				macro_period_us);
		ptiming->range_config__timeout_macrop_a_hi =
				(uint8_t)((timeout_encoded & 0xFF00) >> 8);
		ptiming->range_config__timeout_macrop_a_lo =
				(uint8_t) (timeout_encoded & 0x00FF);
		macro_period_us =
				VL53L1_calc_macro_period_us(
					fast_osc_frequency,
					ptiming->range_config__vcsel_period_b);
		timeout_encoded =
				VL53L1_calc_encoded_timeout(
					mm_config_timeout_us,
					macro_period_us);
		ptiming->mm_config__timeout_macrop_b_hi =
				(uint8_t)((timeout_encoded & 0xFF00) >> 8);
		ptiming->mm_config__timeout_macrop_b_lo =
				(uint8_t) (timeout_encoded & 0x00FF);
		timeout_encoded = VL53L1_calc_encoded_timeout(
							range_config_timeout_us,
							macro_period_us);
		ptiming->range_config__timeout_macrop_b_hi =
				(uint8_t)((timeout_encoded & 0xFF00) >> 8);
		ptiming->range_config__timeout_macrop_b_lo =
				(uint8_t) (timeout_encoded & 0x00FF);
	}
	LOG_FUNCTION_END(0);
	return status;
}
uint8_t VL53L1_encode_vcsel_period(uint8_t vcsel_period_pclks)
{
	uint8_t vcsel_period_reg = 0;
	vcsel_period_reg = (vcsel_period_pclks >> 1) - 1;
	return vcsel_period_reg;
}
uint32_t VL53L1_decode_unsigned_integer(
	uint8_t  *pbuffer,
	uint8_t   no_of_bytes)
{
	uint8_t   i = 0;
	uint32_t  decoded_value = 0;
	for (i = 0 ; i < no_of_bytes ; i++) {
		decoded_value = (decoded_value << 8) + (uint32_t)pbuffer[i];
	}
	return decoded_value;
}
void VL53L1_encode_unsigned_integer(
	uint32_t  ip_value,
	uint8_t   no_of_bytes,
	uint8_t  *pbuffer)
{
	uint8_t   i    = 0;
	uint32_t  data = 0;
	data = ip_value;
	for (i = 0; i < no_of_bytes ; i++) {
		pbuffer[no_of_bytes-i-1] = data & 0x00FF;
		data = data >> 8;
	}
}
void VL53L1_spad_number_to_byte_bit_index(
	uint8_t  spad_number,
	uint8_t *pbyte_index,
	uint8_t *pbit_index,
	uint8_t *pbit_mask)
{
    *pbyte_index  = spad_number >> 3;
    *pbit_index   = spad_number & 0x07;
    *pbit_mask    = 0x01 << *pbit_index;
}
void VL53L1_encode_row_col(
	uint8_t  row,
	uint8_t  col,
	uint8_t *pspad_number)
{
	if (row > 7) {
		*pspad_number = 128 + (col << 3) + (15-row);
	} else {
		*pspad_number = ((15-col) << 3) + row;
	}
}
void VL53L1_decode_zone_size(
	uint8_t  encoded_xy_size,
	uint8_t  *pwidth,
	uint8_t  *pheight)
{
	*pheight = encoded_xy_size >> 4;
	*pwidth  = encoded_xy_size & 0x0F;
}
void VL53L1_encode_zone_size(
	uint8_t  width,
	uint8_t  height,
	uint8_t *pencoded_xy_size)
{
	*pencoded_xy_size = (height << 4) + width;
}
void VL53L1_decode_zone_limits(
	uint8_t   encoded_xy_centre,
	uint8_t   encoded_xy_size,
	int16_t  *px_ll,
	int16_t  *py_ll,
	int16_t  *px_ur,
	int16_t  *py_ur)
{
	uint8_t x_centre = 0;
	uint8_t y_centre = 0;
	uint8_t width    = 0;
	uint8_t height   = 0;
	VL53L1_decode_row_col(
		encoded_xy_centre,
		&y_centre,
		&x_centre);
	VL53L1_decode_zone_size(
		encoded_xy_size,
		&width,
		&height);
	*px_ll = (int16_t)x_centre - ((int16_t)width + 1) / 2;
	if (*px_ll < 0)
		*px_ll = 0;
	*px_ur = *px_ll + (int16_t)width;
	if (*px_ur > (VL53L1_SPAD_ARRAY_WIDTH-1))
		*px_ur = VL53L1_SPAD_ARRAY_WIDTH-1;
	*py_ll = (int16_t)y_centre - ((int16_t)height + 1) / 2;
	if (*py_ll < 0)
		*py_ll = 0;
	*py_ur = *py_ll + (int16_t)height;
	if (*py_ur > (VL53L1_SPAD_ARRAY_HEIGHT-1))
		*py_ur = VL53L1_SPAD_ARRAY_HEIGHT-1;
}
uint8_t VL53L1_is_aperture_location(
	uint8_t row,
	uint8_t col)
{
	uint8_t is_aperture = 0;
	uint8_t mod_row     = row % 4;
	uint8_t mod_col     = col % 4;
	if (mod_row == 0 && mod_col == 2)
		is_aperture = 1;
	if (mod_row == 2 && mod_col == 0)
		is_aperture = 1;
	return is_aperture;
}
void VL53L1_calc_mm_effective_spads(
	uint8_t     encoded_mm_roi_centre,
	uint8_t     encoded_mm_roi_size,
	uint8_t     encoded_zone_centre,
	uint8_t     encoded_zone_size,
	uint8_t    *pgood_spads,
	uint16_t    aperture_attenuation,
	uint16_t   *pmm_inner_effective_spads,
	uint16_t   *pmm_outer_effective_spads)
{
	int16_t   x         = 0;
	int16_t   y         = 0;
	int16_t   mm_x_ll   = 0;
	int16_t   mm_y_ll   = 0;
	int16_t   mm_x_ur   = 0;
	int16_t   mm_y_ur   = 0;
	int16_t   zone_x_ll = 0;
	int16_t   zone_y_ll = 0;
	int16_t   zone_x_ur = 0;
	int16_t   zone_y_ur = 0;
	uint8_t   spad_number = 0;
	uint8_t   byte_index  = 0;
	uint8_t   bit_index   = 0;
	uint8_t   bit_mask    = 0;
	uint8_t   is_aperture = 0;
	uint16_t  spad_attenuation = 0;
	VL53L1_decode_zone_limits(
		encoded_mm_roi_centre,
		encoded_mm_roi_size,
		&mm_x_ll,
		&mm_y_ll,
		&mm_x_ur,
		&mm_y_ur);
	VL53L1_decode_zone_limits(
		encoded_zone_centre,
		encoded_zone_size,
		&zone_x_ll,
		&zone_y_ll,
		&zone_x_ur,
		&zone_y_ur);
	*pmm_inner_effective_spads = 0;
	*pmm_outer_effective_spads = 0;
	for (y = zone_y_ll ; y <= zone_y_ur ; y++) {
		for (x = zone_x_ll ; x <= zone_x_ur ; x++) {
			VL53L1_encode_row_col(
				(uint8_t)y,
				(uint8_t)x,
				&spad_number);
			VL53L1_spad_number_to_byte_bit_index(
				spad_number,
				&byte_index,
				&bit_index,
				&bit_mask);
			if ((pgood_spads[byte_index] & bit_mask) > 0) {
				is_aperture = VL53L1_is_aperture_location(
					(uint8_t)y,
					(uint8_t)x);
				if (is_aperture > 0)
					spad_attenuation = aperture_attenuation;
				else
					spad_attenuation = 0x0100;
				if (x >= mm_x_ll && x <= mm_x_ur &&
					y >= mm_y_ll && y <= mm_y_ur)
					*pmm_inner_effective_spads +=
						spad_attenuation;
				else
					*pmm_outer_effective_spads +=
						spad_attenuation;
			}
		}
	}
}
uint8_t	VL53L1_encode_GPIO_interrupt_config(
	VL53L1_GPIO_interrupt_config_t	*pintconf)
{
	uint8_t system__interrupt_config;
	system__interrupt_config = pintconf->intr_mode_distance;
	system__interrupt_config |= ((pintconf->intr_mode_rate) << 2);
	system__interrupt_config |= ((pintconf->intr_new_measure_ready) << 5);
	system__interrupt_config |= ((pintconf->intr_no_target) << 6);
	system__interrupt_config |= ((pintconf->intr_combined_mode) << 7);
	return system__interrupt_config;
}
VL53L1_GPIO_interrupt_config_t VL53L1_decode_GPIO_interrupt_config(
	uint8_t		system__interrupt_config)
{
	VL53L1_GPIO_interrupt_config_t	intconf;
	intconf.intr_mode_distance = system__interrupt_config & 0x03;
	intconf.intr_mode_rate = (system__interrupt_config >> 2) & 0x03;
	intconf.intr_new_measure_ready = (system__interrupt_config >> 5) & 0x01;
	intconf.intr_no_target = (system__interrupt_config >> 6) & 0x01;
	intconf.intr_combined_mode = (system__interrupt_config >> 7) & 0x01;
	intconf.threshold_rate_low = 0;
	intconf.threshold_rate_high = 0;
	intconf.threshold_distance_low = 0;
	intconf.threshold_distance_high = 0;
	return intconf;
}
VL53L1_Error VL53L1_set_GPIO_distance_threshold(
	VL53L1_DEV                      Dev,
	uint16_t			threshold_high,
	uint16_t			threshold_low)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdev->dyn_cfg.system__thresh_high = threshold_high;
	pdev->dyn_cfg.system__thresh_low = threshold_low;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_GPIO_rate_threshold(
	VL53L1_DEV                      Dev,
	uint16_t			threshold_high,
	uint16_t			threshold_low)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdev->gen_cfg.system__thresh_rate_high = threshold_high;
	pdev->gen_cfg.system__thresh_rate_low = threshold_low;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_GPIO_thresholds_from_struct(
	VL53L1_DEV                      Dev,
	VL53L1_GPIO_interrupt_config_t *pintconf)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	LOG_FUNCTION_START("");
	status = VL53L1_set_GPIO_distance_threshold(
			Dev,
			pintconf->threshold_distance_high,
			pintconf->threshold_distance_low);
	if (status == VL53L1_ERROR_NONE) {
		status =
			VL53L1_set_GPIO_rate_threshold(
				Dev,
				pintconf->threshold_rate_high,
				pintconf->threshold_rate_low);
	}
	LOG_FUNCTION_END(status);
	return status;
}
#ifndef VL53L1_NOCALIB
VL53L1_Error VL53L1_set_ref_spad_char_config(
	VL53L1_DEV    Dev,
	uint8_t       vcsel_period_a,
	uint32_t      phasecal_timeout_us,
	uint16_t      total_rate_target_mcps,
	uint16_t      max_count_rate_rtn_limit_mcps,
	uint16_t      min_count_rate_rtn_limit_mcps,
	uint16_t      fast_osc_frequency)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	uint8_t buffer[2];
	uint32_t macro_period_us = 0;
	uint32_t timeout_mclks   = 0;
	LOG_FUNCTION_START("");
	macro_period_us =
		VL53L1_calc_macro_period_us(
			fast_osc_frequency,
			vcsel_period_a);
	timeout_mclks = phasecal_timeout_us << 12;
	timeout_mclks = timeout_mclks + (macro_period_us>>1);
	timeout_mclks = timeout_mclks / macro_period_us;
	if (timeout_mclks > 0xFF)
		pdev->gen_cfg.phasecal_config__timeout_macrop = 0xFF;
	else
		pdev->gen_cfg.phasecal_config__timeout_macrop =
				(uint8_t)timeout_mclks;
	pdev->tim_cfg.range_config__vcsel_period_a = vcsel_period_a;
	if (status == VL53L1_ERROR_NONE) 
		status =
			VL53L1_WrByte(
				Dev,
				VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP,
				pdev->gen_cfg.phasecal_config__timeout_macrop);
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_WrByte(
				Dev,
				VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A,
				pdev->tim_cfg.range_config__vcsel_period_a);
	buffer[0] = pdev->tim_cfg.range_config__vcsel_period_a;
	buffer[1] = pdev->tim_cfg.range_config__vcsel_period_a;
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_WriteMulti(
				Dev,
				VL53L1_SD_CONFIG__WOI_SD0,
				buffer,
				2); 
	pdev->customer.ref_spad_char__total_rate_target_mcps =
			total_rate_target_mcps;
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_WrWord(
				Dev,
				VL53L1_REF_SPAD_CHAR__TOTAL_RATE_TARGET_MCPS,
				total_rate_target_mcps);  
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_WrWord(
				Dev,
				VL53L1_RANGE_CONFIG__SIGMA_THRESH,
				max_count_rate_rtn_limit_mcps);
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_WrWord(
				Dev,
				VL53L1_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,
				min_count_rate_rtn_limit_mcps);
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_ssc_config(
	VL53L1_DEV            Dev,
	VL53L1_ssc_config_t  *pssc_cfg,
	uint16_t              fast_osc_frequency)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t buffer[5];
	uint32_t macro_period_us = 0;
	uint16_t timeout_encoded = 0;
	LOG_FUNCTION_START("");
	macro_period_us =
		VL53L1_calc_macro_period_us(
			fast_osc_frequency,
			pssc_cfg->vcsel_period);
	timeout_encoded =
		VL53L1_calc_encoded_timeout(
			pssc_cfg->timeout_us,
			macro_period_us);
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_WrByte(
				Dev,
				VL53L1_CAL_CONFIG__VCSEL_START,
				pssc_cfg->vcsel_start);
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_WrByte(
				Dev,
				VL53L1_GLOBAL_CONFIG__VCSEL_WIDTH,
				pssc_cfg->vcsel_width);
    buffer[0] = (uint8_t)((timeout_encoded &  0x0000FF00) >> 8);
    buffer[1] = (uint8_t) (timeout_encoded &  0x000000FF);
    buffer[2] = pssc_cfg->vcsel_period;
    buffer[3] = (uint8_t)((pssc_cfg->rate_limit_mcps &  0x0000FF00) >> 8);
    buffer[4] = (uint8_t) (pssc_cfg->rate_limit_mcps &  0x000000FF);
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_WriteMulti(
				Dev,
				VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
				buffer,
				5);
    buffer[0] = pssc_cfg->vcsel_period;
    buffer[1] = pssc_cfg->vcsel_period;
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_WriteMulti(
				Dev,
				VL53L1_SD_CONFIG__WOI_SD0,
				buffer,
				2);
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_WrByte(
				Dev,
				VL53L1_NVM_BIST__CTRL,
				pssc_cfg->array_select);
	LOG_FUNCTION_END(status);
	return status;
}
#endif
#ifndef VL53L1_NOCALIB
VL53L1_Error VL53L1_get_spad_rate_data(
	VL53L1_DEV                Dev,
	VL53L1_spad_rate_data_t  *pspad_rates)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
    int               i = 0;
    uint8_t  data[512];
    uint8_t *pdata = &data[0];
	LOG_FUNCTION_START("");
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_disable_firmware(Dev);
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_ReadMulti(
				Dev,
				VL53L1_PRIVATE__PATCH_BASE_ADDR_RSLV,
				pdata,
				512);
    pdata = &data[0];
    for (i = 0 ; i < VL53L1_NO_OF_SPAD_ENABLES ; i++) {
		pspad_rates->rate_data[i] =
			(uint16_t)VL53L1_decode_unsigned_integer(pdata, 2);
		pdata += 2;
    }
    pspad_rates->buffer_size     = VL53L1_NO_OF_SPAD_ENABLES;
    pspad_rates->no_of_values    = VL53L1_NO_OF_SPAD_ENABLES;
    pspad_rates->fractional_bits = 15;
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_enable_firmware(Dev);
	LOG_FUNCTION_END(status);
	return status;
}
#endif
VL53L1_Error VL53L1_low_power_auto_data_init(
	VL53L1_DEV                          Dev
	)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdev->low_power_auto_data.vhv_loop_bound =
		VL53L1_TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT;
	pdev->low_power_auto_data.is_low_power_auto_mode = 0;
	pdev->low_power_auto_data.low_power_auto_range_count = 0;
	pdev->low_power_auto_data.saved_interrupt_config = 0;
	pdev->low_power_auto_data.saved_vhv_init = 0;
	pdev->low_power_auto_data.saved_vhv_timeout = 0;
	pdev->low_power_auto_data.first_run_phasecal_result = 0;
	pdev->low_power_auto_data.dss__total_rate_per_spad_mcps = 0;
	pdev->low_power_auto_data.dss__required_spads = 0;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_low_power_auto_data_stop_range(
	VL53L1_DEV                          Dev
	)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdev->low_power_auto_data.low_power_auto_range_count = 0xFF;
	pdev->low_power_auto_data.first_run_phasecal_result = 0;
	pdev->low_power_auto_data.dss__total_rate_per_spad_mcps = 0;
	pdev->low_power_auto_data.dss__required_spads = 0;
	if (pdev->low_power_auto_data.saved_vhv_init != 0)
		pdev->stat_nvm.vhv_config__init =
			pdev->low_power_auto_data.saved_vhv_init;
	if (pdev->low_power_auto_data.saved_vhv_timeout != 0)
		pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound =
			pdev->low_power_auto_data.saved_vhv_timeout;
	pdev->gen_cfg.phasecal_config__override = 0x00;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_config_low_power_auto_mode(
	VL53L1_general_config_t   *pgeneral,
	VL53L1_dynamic_config_t   *pdynamic,
	VL53L1_low_power_auto_data_t *plpadata
	)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	LOG_FUNCTION_START("");
	plpadata->is_low_power_auto_mode = 1;
	plpadata->low_power_auto_range_count = 0;
	pdynamic->system__sequence_config = \
			VL53L1_SEQUENCE_VHV_EN | \
			VL53L1_SEQUENCE_PHASECAL_EN | \
			VL53L1_SEQUENCE_DSS1_EN | \
			VL53L1_SEQUENCE_RANGE_EN;
	pgeneral->dss_config__manual_effective_spads_select = 200 << 8;
	pgeneral->dss_config__roi_mode_control =
		VL53L1_DEVICEDSSMODE__REQUESTED_EFFFECTIVE_SPADS;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_low_power_auto_setup_manual_calibration(
	VL53L1_DEV        Dev)
{
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	LOG_FUNCTION_START("");
	pdev->low_power_auto_data.saved_vhv_init =
		pdev->stat_nvm.vhv_config__init;
	pdev->low_power_auto_data.saved_vhv_timeout =
		pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound;
	pdev->stat_nvm.vhv_config__init &= 0x7F;
	pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound =
		(pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound & 0x03) +
		(pdev->low_power_auto_data.vhv_loop_bound << 2);
	pdev->gen_cfg.phasecal_config__override = 0x01;
	pdev->low_power_auto_data.first_run_phasecal_result =
		pdev->dbg_results.phasecal_result__vcsel_start;
	pdev->gen_cfg.cal_config__vcsel_start =
		pdev->low_power_auto_data.first_run_phasecal_result;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_low_power_auto_update_DSS(
	VL53L1_DEV        Dev)
{
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	uint32_t utemp32a;
	LOG_FUNCTION_START("");
	utemp32a = pdev->sys_results.result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
		pdev->sys_results.result__ambient_count_rate_mcps_sd0;
	if (utemp32a > 0xFFFF)
		utemp32a = 0xFFFF;
	utemp32a = utemp32a << 16;
	if (pdev->sys_results.result__dss_actual_effective_spads_sd0 == 0)
		status = VL53L1_ERROR_DIVISION_BY_ZERO;
	else {
		utemp32a = utemp32a /
			pdev->sys_results.result__dss_actual_effective_spads_sd0;
		pdev->low_power_auto_data.dss__total_rate_per_spad_mcps =
			utemp32a;
		utemp32a = pdev->stat_cfg.dss_config__target_total_rate_mcps <<
			16;
		if (pdev->low_power_auto_data.dss__total_rate_per_spad_mcps == 0)
			status = VL53L1_ERROR_DIVISION_BY_ZERO;
		else {
			utemp32a = utemp32a /
				pdev->low_power_auto_data.dss__total_rate_per_spad_mcps;
			if (utemp32a > 0xFFFF)
				utemp32a = 0xFFFF;
			pdev->low_power_auto_data.dss__required_spads =
				(uint16_t)utemp32a;
			pdev->gen_cfg.dss_config__manual_effective_spads_select =
				pdev->low_power_auto_data.dss__required_spads;
			pdev->gen_cfg.dss_config__roi_mode_control =
				VL53L1_DEVICEDSSMODE__REQUESTED_EFFFECTIVE_SPADS;
		}
	}
	if (status == VL53L1_ERROR_DIVISION_BY_ZERO) {
		pdev->low_power_auto_data.dss__required_spads = 0x8000;
		pdev->gen_cfg.dss_config__manual_effective_spads_select =
		pdev->low_power_auto_data.dss__required_spads;
		pdev->gen_cfg.dss_config__roi_mode_control =
		VL53L1_DEVICEDSSMODE__REQUESTED_EFFFECTIVE_SPADS;
		status = VL53L1_ERROR_NONE;
	}
	LOG_FUNCTION_END(status);
	return status;
}