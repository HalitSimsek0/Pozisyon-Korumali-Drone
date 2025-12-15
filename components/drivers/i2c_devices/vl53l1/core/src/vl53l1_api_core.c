#include "vl53l1_ll_def.h"
#include "vl53l1_ll_device.h"
#include "vl53l1_platform.h"
#include "vl53l1_register_map.h"
#include "vl53l1_register_settings.h"
#include "vl53l1_register_funcs.h"
#include "vl53l1_nvm_map.h"
#include "vl53l1_core.h"
#include "vl53l1_wait.h"
#include "vl53l1_api_preset_modes.h"
#include "vl53l1_silicon_core.h"
#include "vl53l1_api_core.h"
#include "vl53l1_tuning_parm_defaults.h"
#ifdef VL53L1_LOG_ENABLE
#include "vl53l1_api_debug.h"
#endif
#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(VL53L1_TRACE_MODULE_CORE, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(VL53L1_TRACE_MODULE_CORE, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...) \
	_LOG_FUNCTION_END_FMT(VL53L1_TRACE_MODULE_CORE, status, \
	fmt, ##__VA_ARGS__)
#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_CORE, \
	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)
#define VL53L1_MAX_I2C_XFER_SIZE 256
#ifdef VL53L1_DEBUG
VL53L1_Error VL53L1_get_version(
	VL53L1_DEV           Dev,
	VL53L1_ll_version_t *pdata)
{
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	VL53L1_init_version(Dev);
	memcpy(pdata, &(pdev->version), sizeof(VL53L1_ll_version_t));
	return VL53L1_ERROR_NONE;
}
VL53L1_Error VL53L1_get_device_firmware_version(
	VL53L1_DEV        Dev,
	uint16_t         *pfw_version)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	LOG_FUNCTION_START("");
	if (status == VL53L1_ERROR_NONE) 
		status = VL53L1_disable_firmware(Dev);
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_RdWord(
				Dev,
				VL53L1_MCU_GENERAL_PURPOSE__GP_0,
				pfw_version);
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_enable_firmware(Dev);
	LOG_FUNCTION_END(status);
	return status;
}
#endif
VL53L1_Error VL53L1_data_init(
	VL53L1_DEV        Dev,
	uint8_t           read_p2p_data)
{
	VL53L1_Error status       = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t    *pdev =
			VL53L1DevStructGetLLDriverHandle(Dev);
	VL53L1_init_ll_driver_state(
			Dev,
			VL53L1_DEVICESTATE_UNKNOWN);
	pdev->wait_method             = VL53L1_WAIT_METHOD_BLOCKING;
	pdev->preset_mode             = VL53L1_DEVICEPRESETMODE_STANDARD_RANGING;
	pdev->measurement_mode        = VL53L1_DEVICEMEASUREMENTMODE_STOP;
	pdev->offset_calibration_mode =
		VL53L1_OFFSETCALIBRATIONMODE__MM1_MM2__STANDARD;
	pdev->offset_correction_mode  =
		VL53L1_OFFSETCORRECTIONMODE__MM1_MM2_OFFSETS;
	pdev->phasecal_config_timeout_us  =  1000;
	pdev->mm_config_timeout_us        =  2000;
	pdev->range_config_timeout_us     = 13000;
	pdev->inter_measurement_period_ms =   100;
	pdev->dss_config__target_total_rate_mcps = 0x0A00;
	pdev->debug_mode                  =  0x00;
	pdev->gain_cal.standard_ranging_gain_factor =
			VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT;
	VL53L1_init_version(Dev);
	if (read_p2p_data > 0 && status == VL53L1_ERROR_NONE) 
			status = VL53L1_read_p2p_data(Dev);
#ifndef VL53L1_NOCALIB
	status =
		VL53L1_init_refspadchar_config_struct(
			&(pdev->refspadchar));
#endif
#ifndef VL53L1_NOCALIB
	status =
		VL53L1_init_ssc_config_struct(
			&(pdev->ssc_cfg));
#endif
	status =
		VL53L1_init_xtalk_config_struct(
			&(pdev->customer),
			&(pdev->xtalk_cfg));
#ifndef VL53L1_NOCALIB
	status =
		VL53L1_init_offset_cal_config_struct(
		    &(pdev->offsetcal_cfg));
#endif
	status =
		VL53L1_init_tuning_parm_storage_struct(
			&(pdev->tuning_parms));
	status = VL53L1_set_vhv_loopbound(Dev,
		VL53L1_TUNINGPARM_VHV_LOOPBOUND_DEFAULT);
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_set_preset_mode(
						Dev,
						pdev->preset_mode,
						pdev->dss_config__target_total_rate_mcps,  
						pdev->phasecal_config_timeout_us,
						pdev->mm_config_timeout_us,
						pdev->range_config_timeout_us,
						pdev->inter_measurement_period_ms);
	VL53L1_low_power_auto_data_init(
			Dev
			);
#ifdef VL53L1_LOG_ENABLE
	VL53L1_print_static_nvm_managed(
		&(pdev->stat_nvm),
		"data_init():pdev->lldata.stat_nvm.",
		VL53L1_TRACE_MODULE_DATA_INIT);
	VL53L1_print_customer_nvm_managed(
		&(pdev->customer),
		"data_init():pdev->lldata.customer.",
		VL53L1_TRACE_MODULE_DATA_INIT);
	VL53L1_print_nvm_copy_data(
		&(pdev->nvm_copy_data),
		"data_init():pdev->lldata.nvm_copy_data.",
		VL53L1_TRACE_MODULE_DATA_INIT);
	VL53L1_print_additional_offset_cal_data(
		&(pdev->add_off_cal_data),
		"data_init():pdev->lldata.add_off_cal_data.",
		VL53L1_TRACE_MODULE_DATA_INIT);
	VL53L1_print_user_zone(
		&(pdev->mm_roi),
		"data_init():pdev->lldata.mm_roi.",
		VL53L1_TRACE_MODULE_DATA_INIT);
	VL53L1_print_optical_centre(
		&(pdev->optical_centre),
		"data_init():pdev->lldata.optical_centre.",
		VL53L1_TRACE_MODULE_DATA_INIT);
	VL53L1_print_cal_peak_rate_map(
		&(pdev->cal_peak_rate_map),
		"data_init():pdev->lldata.cal_peak_rate_map.",
		VL53L1_TRACE_MODULE_DATA_INIT);
#endif
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_read_p2p_data(
	VL53L1_DEV        Dev)
{
	VL53L1_Error status       = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_get_static_nvm_managed(
						Dev,
						&(pdev->stat_nvm));
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_get_customer_nvm_managed(
						Dev,
						&(pdev->customer));
	if (status == VL53L1_ERROR_NONE) {
		status = VL53L1_get_nvm_copy_data(
						Dev,
						&(pdev->nvm_copy_data));
		if (status == VL53L1_ERROR_NONE)
			VL53L1_copy_rtn_good_spads_to_buffer(
					&(pdev->nvm_copy_data),
					&(pdev->rtn_good_spads[0]));
	}
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_RdWord(
				Dev,
				VL53L1_RESULT__OSC_CALIBRATE_VAL,
				&(pdev->dbg_results.result__osc_calibrate_val));
	if (pdev->stat_nvm.osc_measured__fast_osc__frequency < 0x1000) {
		trace_print(
			VL53L1_TRACE_LEVEL_WARNING,
			"\nInvalid %s value (0x%04X) - forcing to 0x%04X\n\n",
			"pdev->stat_nvm.osc_measured__fast_osc__frequency",
			pdev->stat_nvm.osc_measured__fast_osc__frequency,
			0xBCCC);
		pdev->stat_nvm.osc_measured__fast_osc__frequency = 0xBCCC;
	}
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_get_mode_mitigation_roi(
				Dev,
				&(pdev->mm_roi));
	if (pdev->optical_centre.x_centre == 0 &&
		pdev->optical_centre.y_centre == 0) {
		pdev->optical_centre.x_centre =
				pdev->mm_roi.x_centre << 4;
		pdev->optical_centre.y_centre =
				pdev->mm_roi.y_centre << 4;
	}
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_software_reset(
	VL53L1_DEV    Dev)
{
	VL53L1_Error status       = VL53L1_ERROR_NONE;
	LOG_FUNCTION_START("");
	if (status == VL53L1_ERROR_NONE) 
		status = VL53L1_WrByte(
						Dev,
						VL53L1_SOFT_RESET,
						0x00);
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_WaitUs(
				Dev,
				VL53L1_SOFTWARE_RESET_DURATION_US);
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_WrByte(
						Dev,
						VL53L1_SOFT_RESET,
						0x01);
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_wait_for_boot_completion(Dev);
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_part_to_part_data(
	VL53L1_DEV                            Dev,
	VL53L1_calibration_data_t            *pcal_data)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	uint32_t tempu32;
	LOG_FUNCTION_START("");
	if (pcal_data->struct_version !=
		VL53L1_LL_CALIBRATION_DATA_STRUCT_VERSION) {
		status = VL53L1_ERROR_INVALID_PARAMS;
	}
	if (status == VL53L1_ERROR_NONE) {
		memcpy(
			&(pdev->customer),
			&(pcal_data->customer),
			sizeof(VL53L1_customer_nvm_managed_t));
		memcpy(
			&(pdev->add_off_cal_data),
			&(pcal_data->add_off_cal_data),
			sizeof(VL53L1_additional_offset_cal_data_t));
		memcpy(
			&(pdev->gain_cal),
			&(pcal_data->gain_cal),
			sizeof(VL53L1_gain_calibration_data_t));
		memcpy(
			&(pdev->cal_peak_rate_map),
			&(pcal_data->cal_peak_rate_map),
			sizeof(VL53L1_cal_peak_rate_map_t));
		pdev->xtalk_cfg.algo__crosstalk_compensation_plane_offset_kcps =
			pdev->customer.algo__crosstalk_compensation_plane_offset_kcps;
		pdev->xtalk_cfg.algo__crosstalk_compensation_x_plane_gradient_kcps =
			pdev->customer.algo__crosstalk_compensation_x_plane_gradient_kcps;
		pdev->xtalk_cfg.algo__crosstalk_compensation_y_plane_gradient_kcps =
			pdev->customer.algo__crosstalk_compensation_y_plane_gradient_kcps;
		if (pdev->xtalk_cfg.global_crosstalk_compensation_enable == 0x00) {
			pdev->customer.algo__crosstalk_compensation_plane_offset_kcps =
				0x00;
			pdev->customer.algo__crosstalk_compensation_x_plane_gradient_kcps =
				0x00;
			pdev->customer.algo__crosstalk_compensation_y_plane_gradient_kcps =
				0x00;
		} else {
			tempu32 = VL53L1_calc_crosstalk_plane_offset_with_margin(
				pdev->xtalk_cfg.algo__crosstalk_compensation_plane_offset_kcps,
				pdev->xtalk_cfg.lite_mode_crosstalk_margin_kcps);
			if (tempu32 > 0xFFFF) {	
				tempu32 = 0xFFFF;
			}
			pdev->customer.algo__crosstalk_compensation_plane_offset_kcps =
				(uint16_t)tempu32;
		}
	}
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_get_part_to_part_data(
	VL53L1_DEV                      Dev,
	VL53L1_calibration_data_t      *pcal_data)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pcal_data->struct_version =
			VL53L1_LL_CALIBRATION_DATA_STRUCT_VERSION;
	memcpy(
		&(pcal_data->customer),
		&(pdev->customer),
		sizeof(VL53L1_customer_nvm_managed_t));
	if (pdev->xtalk_cfg.algo__crosstalk_compensation_plane_offset_kcps > 0xFFFF) {
		pcal_data->customer.algo__crosstalk_compensation_plane_offset_kcps =
			0xFFFF;
	} else {
		pcal_data->customer.algo__crosstalk_compensation_plane_offset_kcps =
			(uint16_t)pdev->xtalk_cfg.algo__crosstalk_compensation_plane_offset_kcps;
	}
	pcal_data->customer.algo__crosstalk_compensation_x_plane_gradient_kcps =
		pdev->xtalk_cfg.algo__crosstalk_compensation_x_plane_gradient_kcps;
	pcal_data->customer.algo__crosstalk_compensation_y_plane_gradient_kcps =
		pdev->xtalk_cfg.algo__crosstalk_compensation_y_plane_gradient_kcps;
	memcpy(
		&(pcal_data->add_off_cal_data),
		&(pdev->add_off_cal_data),
		sizeof(VL53L1_additional_offset_cal_data_t));
	memcpy(
		&(pcal_data->optical_centre),
		&(pdev->optical_centre),
		sizeof(VL53L1_optical_centre_t));
	memcpy(
		&(pcal_data->gain_cal),
		&(pdev->gain_cal),
		sizeof(VL53L1_gain_calibration_data_t));
	memcpy(
		&(pcal_data->cal_peak_rate_map),
		&(pdev->cal_peak_rate_map),
		sizeof(VL53L1_cal_peak_rate_map_t));
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_inter_measurement_period_ms(
	VL53L1_DEV              Dev,
	uint32_t                inter_measurement_period_ms)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	if (pdev->dbg_results.result__osc_calibrate_val == 0)
		status = VL53L1_ERROR_DIVISION_BY_ZERO;
	if (status == VL53L1_ERROR_NONE) {
		pdev->inter_measurement_period_ms = inter_measurement_period_ms;
		pdev->tim_cfg.system__intermeasurement_period = \
			inter_measurement_period_ms *
			(uint32_t)pdev->dbg_results.result__osc_calibrate_val;
	}
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_get_inter_measurement_period_ms(
	VL53L1_DEV              Dev,
	uint32_t               *pinter_measurement_period_ms)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	if (pdev->dbg_results.result__osc_calibrate_val == 0)
		status = VL53L1_ERROR_DIVISION_BY_ZERO;
	if (status == VL53L1_ERROR_NONE)
		*pinter_measurement_period_ms = \
			pdev->tim_cfg.system__intermeasurement_period /
			(uint32_t)pdev->dbg_results.result__osc_calibrate_val;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_timeouts_us(
	VL53L1_DEV          Dev,
	uint32_t            phasecal_config_timeout_us,
	uint32_t            mm_config_timeout_us,
	uint32_t            range_config_timeout_us)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
			VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	if (pdev->stat_nvm.osc_measured__fast_osc__frequency == 0)
		status = VL53L1_ERROR_DIVISION_BY_ZERO;
	if (status == VL53L1_ERROR_NONE) {
		pdev->phasecal_config_timeout_us = phasecal_config_timeout_us;
		pdev->mm_config_timeout_us       = mm_config_timeout_us;
		pdev->range_config_timeout_us    = range_config_timeout_us;
		status =
			VL53L1_calc_timeout_register_values(
				phasecal_config_timeout_us,
				mm_config_timeout_us,
				range_config_timeout_us,
				pdev->stat_nvm.osc_measured__fast_osc__frequency,
				&(pdev->gen_cfg),
				&(pdev->tim_cfg));
	}
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_get_timeouts_us(
	VL53L1_DEV           Dev,
	uint32_t            *pphasecal_config_timeout_us,
	uint32_t            *pmm_config_timeout_us,
	uint32_t			*prange_config_timeout_us)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
			VL53L1DevStructGetLLDriverHandle(Dev);
	uint32_t  macro_period_us = 0;
	uint16_t  timeout_encoded = 0;
	LOG_FUNCTION_START("");
	if (pdev->stat_nvm.osc_measured__fast_osc__frequency == 0)
		status = VL53L1_ERROR_DIVISION_BY_ZERO;
	if (status == VL53L1_ERROR_NONE) {
		macro_period_us =
			VL53L1_calc_macro_period_us(
				pdev->stat_nvm.osc_measured__fast_osc__frequency,
				pdev->tim_cfg.range_config__vcsel_period_a);
		*pphasecal_config_timeout_us =
			VL53L1_calc_timeout_us(
				(uint32_t)pdev->gen_cfg.phasecal_config__timeout_macrop,
				macro_period_us);
		timeout_encoded =
			(uint16_t)pdev->tim_cfg.mm_config__timeout_macrop_a_hi;
		timeout_encoded = (timeout_encoded << 8) +
			(uint16_t)pdev->tim_cfg.mm_config__timeout_macrop_a_lo;
		*pmm_config_timeout_us =
			VL53L1_calc_decoded_timeout_us(
				timeout_encoded,
				macro_period_us);
		timeout_encoded =
			(uint16_t)pdev->tim_cfg.range_config__timeout_macrop_a_hi;
		timeout_encoded = (timeout_encoded << 8) +
			(uint16_t)pdev->tim_cfg.range_config__timeout_macrop_a_lo;
		*prange_config_timeout_us =
			VL53L1_calc_decoded_timeout_us(
				timeout_encoded,
				macro_period_us);
		pdev->phasecal_config_timeout_us = *pphasecal_config_timeout_us;
		pdev->mm_config_timeout_us       = *pmm_config_timeout_us;
		pdev->range_config_timeout_us    = *prange_config_timeout_us;
	}
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_calibration_repeat_period(
	VL53L1_DEV          Dev,
	uint16_t            cal_config__repeat_period)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
		VL53L1DevStructGetLLDriverHandle(Dev);
	pdev->gen_cfg.cal_config__repeat_rate = cal_config__repeat_period;
	return status;
}
VL53L1_Error VL53L1_get_calibration_repeat_period(
	VL53L1_DEV          Dev,
	uint16_t           *pcal_config__repeat_period)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
		VL53L1DevStructGetLLDriverHandle(Dev);
	*pcal_config__repeat_period = pdev->gen_cfg.cal_config__repeat_rate;
	return status;
}
VL53L1_Error VL53L1_set_sequence_config_bit(
	VL53L1_DEV                    Dev,
	VL53L1_DeviceSequenceConfig   bit_id,
	uint8_t                       value)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
		VL53L1DevStructGetLLDriverHandle(Dev);
	uint8_t  bit_mask        = 0x01;
	uint8_t  clr_mask        = 0xFF  - bit_mask;
	uint8_t  bit_value       = value & bit_mask;
	if (bit_id <= VL53L1_DEVICESEQUENCECONFIG_RANGE) {
		if (bit_id > 0) {
			bit_mask  = 0x01 << bit_id;
			bit_value = bit_value << bit_id;
			clr_mask  = 0xFF  - bit_mask;
		}
		pdev->dyn_cfg.system__sequence_config = \
			(pdev->dyn_cfg.system__sequence_config & clr_mask) | \
			bit_value;
	} else {
		status = VL53L1_ERROR_INVALID_PARAMS;
	}
	return status;
}
VL53L1_Error VL53L1_get_sequence_config_bit(
	VL53L1_DEV                    Dev,
	VL53L1_DeviceSequenceConfig   bit_id,
	uint8_t                      *pvalue)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
		VL53L1DevStructGetLLDriverHandle(Dev);
	uint8_t  bit_mask        = 0x01;
	if (bit_id <= VL53L1_DEVICESEQUENCECONFIG_RANGE) {
		if (bit_id > 0) {
			bit_mask  = 0x01 << bit_id;
		}
		*pvalue =
			pdev->dyn_cfg.system__sequence_config & bit_mask;
		if (bit_id > 0) {
			*pvalue  = *pvalue >> bit_id;
		}
	} else {
		status = VL53L1_ERROR_INVALID_PARAMS;
	}
	return status;
}
VL53L1_Error VL53L1_set_interrupt_polarity(
	VL53L1_DEV                      Dev,
	VL53L1_DeviceInterruptPolarity  interrupt_polarity)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
		VL53L1DevStructGetLLDriverHandle(Dev);
	pdev->stat_cfg.gpio_hv_mux__ctrl = \
			(pdev->stat_cfg.gpio_hv_mux__ctrl & \
			 VL53L1_DEVICEINTERRUPTPOLARITY_CLEAR_MASK) | \
			(interrupt_polarity & \
			 VL53L1_DEVICEINTERRUPTPOLARITY_BIT_MASK);
	return status;
}
#ifndef VL53L1_NOCALIB
VL53L1_Error VL53L1_set_refspadchar_config_struct(
	VL53L1_DEV                     Dev,
	VL53L1_refspadchar_config_t   *pdata)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
		VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdev->refspadchar.device_test_mode = pdata->device_test_mode;
	pdev->refspadchar.vcsel_period     = pdata->vcsel_period;
	pdev->refspadchar.timeout_us       = pdata->timeout_us;
	pdev->refspadchar.target_count_rate_mcps    =
			pdata->target_count_rate_mcps;
	pdev->refspadchar.min_count_rate_limit_mcps =
			pdata->min_count_rate_limit_mcps;
	pdev->refspadchar.max_count_rate_limit_mcps =
			pdata->max_count_rate_limit_mcps;
	LOG_FUNCTION_END(status);
	return status;
}
#endif
#ifndef VL53L1_NOCALIB
VL53L1_Error VL53L1_get_refspadchar_config_struct(
	VL53L1_DEV                     Dev,
	VL53L1_refspadchar_config_t   *pdata)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
		VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdata->device_test_mode       = pdev->refspadchar.device_test_mode;
	pdata->vcsel_period           = pdev->refspadchar.vcsel_period;
	pdata->timeout_us             = pdev->refspadchar.timeout_us;
	pdata->target_count_rate_mcps = pdev->refspadchar.target_count_rate_mcps;
	pdata->min_count_rate_limit_mcps =
			pdev->refspadchar.min_count_rate_limit_mcps;
	pdata->max_count_rate_limit_mcps =
			pdev->refspadchar.max_count_rate_limit_mcps;
	LOG_FUNCTION_END(status);
	return status;
}
#endif
VL53L1_Error VL53L1_set_range_ignore_threshold(
	VL53L1_DEV              Dev,
	uint8_t                 range_ignore_thresh_mult,
	uint16_t                range_ignore_threshold_mcps)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
		VL53L1DevStructGetLLDriverHandle(Dev);
	pdev->xtalk_cfg.crosstalk_range_ignore_threshold_rate_mcps =
		range_ignore_threshold_mcps;
	pdev->xtalk_cfg.crosstalk_range_ignore_threshold_mult =
		range_ignore_thresh_mult;
	return status;
}
VL53L1_Error VL53L1_get_range_ignore_threshold(
	VL53L1_DEV              Dev,
	uint8_t                *prange_ignore_thresh_mult,
	uint16_t               *prange_ignore_threshold_mcps_internal,
	uint16_t               *prange_ignore_threshold_mcps_current)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
		VL53L1DevStructGetLLDriverHandle(Dev);
	*prange_ignore_thresh_mult =
	    pdev->xtalk_cfg.crosstalk_range_ignore_threshold_mult;
	*prange_ignore_threshold_mcps_current =
		pdev->stat_cfg.algo__range_ignore_threshold_mcps;
	*prange_ignore_threshold_mcps_internal =
		pdev->xtalk_cfg.crosstalk_range_ignore_threshold_rate_mcps;
	return status;
}
VL53L1_Error VL53L1_get_interrupt_polarity(
	VL53L1_DEV                       Dev,
	VL53L1_DeviceInterruptPolarity  *pinterrupt_polarity)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
		VL53L1DevStructGetLLDriverHandle(Dev);
	*pinterrupt_polarity = \
		pdev->stat_cfg.gpio_hv_mux__ctrl & \
		VL53L1_DEVICEINTERRUPTPOLARITY_BIT_MASK ;
	return status;
}
VL53L1_Error VL53L1_set_user_zone(
	VL53L1_DEV              Dev,
	VL53L1_user_zone_t     *puser_zone)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	VL53L1_encode_row_col(
		puser_zone->y_centre,
		puser_zone->x_centre,
		&(pdev->dyn_cfg.roi_config__user_roi_centre_spad));
	VL53L1_encode_zone_size(
		puser_zone->width,
		puser_zone->height,
		&(pdev->dyn_cfg.roi_config__user_roi_requested_global_xy_size));
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_get_user_zone(
	VL53L1_DEV              Dev,
	VL53L1_user_zone_t     *puser_zone)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	VL53L1_decode_row_col(
			pdev->dyn_cfg.roi_config__user_roi_centre_spad,
			&(puser_zone->y_centre),
			&(puser_zone->x_centre));
	VL53L1_decode_zone_size(
		pdev->dyn_cfg.roi_config__user_roi_requested_global_xy_size,
		&(puser_zone->width),
		&(puser_zone->height));
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_get_mode_mitigation_roi(
	VL53L1_DEV              Dev,
	VL53L1_user_zone_t     *pmm_roi)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	uint8_t  x       = 0;
	uint8_t  y       = 0;
	uint8_t  xy_size = 0;
	LOG_FUNCTION_START("");
	VL53L1_decode_row_col(
			pdev->nvm_copy_data.roi_config__mode_roi_centre_spad,
			&y,
			&x);
	pmm_roi->x_centre = x;
	pmm_roi->y_centre = y;
	xy_size = pdev->nvm_copy_data.roi_config__mode_roi_xy_size;
	pmm_roi->height = xy_size >> 4;
	pmm_roi->width  = xy_size & 0x0F;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_get_preset_mode_timing_cfg(
	VL53L1_DEV                   Dev,
	VL53L1_DevicePresetModes     device_preset_mode,
	uint16_t                    *pdss_config__target_total_rate_mcps,
	uint32_t                    *pphasecal_config_timeout_us,
	uint32_t                    *pmm_config_timeout_us,
	uint32_t                    *prange_config_timeout_us)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	switch (device_preset_mode) {
	case VL53L1_DEVICEPRESETMODE_STANDARD_RANGING:
	case VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_SHORT_RANGE:
	case VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_LONG_RANGE:
	case VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_MM1_CAL:
	case VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_MM2_CAL:
	case VL53L1_DEVICEPRESETMODE_OLT:
		*pdss_config__target_total_rate_mcps =
				pdev->tuning_parms.tp_dss_target_lite_mcps;
		*pphasecal_config_timeout_us =
				pdev->tuning_parms.tp_phasecal_timeout_lite_us;
		*pmm_config_timeout_us =
				pdev->tuning_parms.tp_mm_timeout_lite_us;
		*prange_config_timeout_us =
				pdev->tuning_parms.tp_range_timeout_lite_us;
	break;
	case VL53L1_DEVICEPRESETMODE_TIMED_RANGING:
	case VL53L1_DEVICEPRESETMODE_TIMED_RANGING_SHORT_RANGE:
	case VL53L1_DEVICEPRESETMODE_TIMED_RANGING_LONG_RANGE:
	case VL53L1_DEVICEPRESETMODE_SINGLESHOT_RANGING:
		*pdss_config__target_total_rate_mcps =
				pdev->tuning_parms.tp_dss_target_timed_mcps;
		*pphasecal_config_timeout_us =
				pdev->tuning_parms.tp_phasecal_timeout_timed_us;
		*pmm_config_timeout_us =
				pdev->tuning_parms.tp_mm_timeout_timed_us;
		*prange_config_timeout_us =
				pdev->tuning_parms.tp_range_timeout_timed_us;
	break;
	case VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_SHORT_RANGE:
	case VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_MEDIUM_RANGE:
	case VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_LONG_RANGE:
		*pdss_config__target_total_rate_mcps =
				pdev->tuning_parms.tp_dss_target_timed_mcps;
		*pphasecal_config_timeout_us =
				pdev->tuning_parms.tp_phasecal_timeout_timed_us;
		*pmm_config_timeout_us =
				pdev->tuning_parms.tp_mm_timeout_lpa_us;
		*prange_config_timeout_us =
				pdev->tuning_parms.tp_range_timeout_lpa_us;
	break;
	default:
		status = VL53L1_ERROR_INVALID_PARAMS;
		break;
	}
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_preset_mode(
	VL53L1_DEV                   Dev,
	VL53L1_DevicePresetModes     device_preset_mode,
	uint16_t                     dss_config__target_total_rate_mcps,
	uint32_t                     phasecal_config_timeout_us,
	uint32_t                     mm_config_timeout_us,
	uint32_t                     range_config_timeout_us,
	uint32_t                     inter_measurement_period_ms)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
			VL53L1DevStructGetLLDriverHandle(Dev);
	VL53L1_static_config_t        *pstatic       = &(pdev->stat_cfg);
	VL53L1_general_config_t       *pgeneral      = &(pdev->gen_cfg);
	VL53L1_timing_config_t        *ptiming       = &(pdev->tim_cfg);
	VL53L1_dynamic_config_t       *pdynamic      = &(pdev->dyn_cfg);
	VL53L1_system_control_t       *psystem       = &(pdev->sys_ctrl);
	VL53L1_tuning_parm_storage_t  *ptuning_parms = &(pdev->tuning_parms);
	VL53L1_low_power_auto_data_t  *plpadata      =
					&(pdev->low_power_auto_data);
	LOG_FUNCTION_START("");
	pdev->preset_mode                 = device_preset_mode;
	pdev->mm_config_timeout_us        = mm_config_timeout_us;
	pdev->range_config_timeout_us     = range_config_timeout_us;
	pdev->inter_measurement_period_ms = inter_measurement_period_ms;
	VL53L1_init_ll_driver_state(
			Dev,
			VL53L1_DEVICESTATE_SW_STANDBY);
	switch (device_preset_mode) {
	case VL53L1_DEVICEPRESETMODE_STANDARD_RANGING:
		status = VL53L1_preset_mode_standard_ranging(
					pstatic,
					pgeneral,
					ptiming,
					pdynamic,
					psystem,
					ptuning_parms);
		break;
	case VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_SHORT_RANGE:
		status = VL53L1_preset_mode_standard_ranging_short_range(
					pstatic,
					pgeneral,
					ptiming,
					pdynamic,
					psystem,
					ptuning_parms);
		break;
	case VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_LONG_RANGE:
		status = VL53L1_preset_mode_standard_ranging_long_range(
					pstatic,
					pgeneral,
					ptiming,
					pdynamic,
					psystem,
					ptuning_parms);
		break;
#ifndef VL53L1_NOCALIB
	case VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_MM1_CAL:
		status = VL53L1_preset_mode_standard_ranging_mm1_cal(
					pstatic,
					pgeneral,
					ptiming,
					pdynamic,
					psystem,
					ptuning_parms);
		break;
	case VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_MM2_CAL:
		status = VL53L1_preset_mode_standard_ranging_mm2_cal(
					pstatic,
					pgeneral,
					ptiming,
					pdynamic,
					psystem,
					ptuning_parms);
		break;
#endif
	case VL53L1_DEVICEPRESETMODE_TIMED_RANGING:
		status = VL53L1_preset_mode_timed_ranging(
					pstatic,
					pgeneral,
					ptiming,
					pdynamic,
					psystem,
					ptuning_parms);
		break;
	case VL53L1_DEVICEPRESETMODE_TIMED_RANGING_SHORT_RANGE:
		status = VL53L1_preset_mode_timed_ranging_short_range(
					pstatic,
					pgeneral,
					ptiming,
					pdynamic,
					psystem,
					ptuning_parms);
		break;
	case VL53L1_DEVICEPRESETMODE_TIMED_RANGING_LONG_RANGE:
		status = VL53L1_preset_mode_timed_ranging_long_range(
					pstatic,
					pgeneral,
					ptiming,
					pdynamic,
					psystem,
					ptuning_parms);
		break;
	case VL53L1_DEVICEPRESETMODE_OLT:
		status = VL53L1_preset_mode_olt(
					pstatic,
					pgeneral,
					ptiming,
					pdynamic,
					psystem,
					ptuning_parms);
		break;
	case VL53L1_DEVICEPRESETMODE_SINGLESHOT_RANGING:
		status = VL53L1_preset_mode_singleshot_ranging(
					pstatic,
					pgeneral,
					ptiming,
					pdynamic,
					psystem,
					ptuning_parms);
		break;
	case VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_SHORT_RANGE:
		status = VL53L1_preset_mode_low_power_auto_short_ranging(
					pstatic,
					pgeneral,
					ptiming,
					pdynamic,
					psystem,
					ptuning_parms,
					plpadata);
		break;
	case VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_MEDIUM_RANGE:
		status = VL53L1_preset_mode_low_power_auto_ranging(
					pstatic,
					pgeneral,
					ptiming,
					pdynamic,
					psystem,
					ptuning_parms,
					plpadata);
		break;
	case VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_LONG_RANGE:
		status = VL53L1_preset_mode_low_power_auto_long_ranging(
					pstatic,
					pgeneral,
					ptiming,
					pdynamic,
					psystem,
					ptuning_parms,
					plpadata);
		break;
	default:
		status = VL53L1_ERROR_INVALID_PARAMS;
		break;
	}
	if (status == VL53L1_ERROR_NONE) {
		pstatic->dss_config__target_total_rate_mcps =
				dss_config__target_total_rate_mcps;
		pdev->dss_config__target_total_rate_mcps    =
				dss_config__target_total_rate_mcps;
	}
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_set_timeouts_us(
				Dev,
				phasecal_config_timeout_us,
				mm_config_timeout_us,
				range_config_timeout_us);
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_set_inter_measurement_period_ms(
				Dev,
				inter_measurement_period_ms);
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error  VL53L1_enable_xtalk_compensation(
	VL53L1_DEV                 Dev)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint32_t tempu32;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	tempu32 = VL53L1_calc_crosstalk_plane_offset_with_margin(
		pdev->xtalk_cfg.algo__crosstalk_compensation_plane_offset_kcps,
		pdev->xtalk_cfg.lite_mode_crosstalk_margin_kcps);
	if (tempu32 > 0xFFFF) {
		tempu32 = 0xFFFF;
	}
	pdev->customer.algo__crosstalk_compensation_plane_offset_kcps =
		(uint16_t)tempu32;
	pdev->customer.algo__crosstalk_compensation_x_plane_gradient_kcps =
		pdev->xtalk_cfg.algo__crosstalk_compensation_x_plane_gradient_kcps;
	pdev->customer.algo__crosstalk_compensation_y_plane_gradient_kcps =
		pdev->xtalk_cfg.algo__crosstalk_compensation_y_plane_gradient_kcps;
	pdev->xtalk_cfg.global_crosstalk_compensation_enable = 0x01;
	if (status == VL53L1_ERROR_NONE) {
		pdev->xtalk_cfg.crosstalk_range_ignore_threshold_rate_mcps =
			VL53L1_calc_range_ignore_threshold(
				pdev->xtalk_cfg.algo__crosstalk_compensation_plane_offset_kcps,
				pdev->xtalk_cfg.algo__crosstalk_compensation_x_plane_gradient_kcps,
				pdev->xtalk_cfg.algo__crosstalk_compensation_y_plane_gradient_kcps,
				pdev->xtalk_cfg.crosstalk_range_ignore_threshold_mult);
	}
	if (status == VL53L1_ERROR_NONE) 
		status =
			VL53L1_set_customer_nvm_managed(
				Dev,
				&(pdev->customer));
	LOG_FUNCTION_END(status);
	return status;
}
void VL53L1_get_xtalk_compensation_enable(
	VL53L1_DEV    Dev,
	uint8_t       *pcrosstalk_compensation_enable)
{
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	*pcrosstalk_compensation_enable =
		pdev->xtalk_cfg.global_crosstalk_compensation_enable;
}
VL53L1_Error VL53L1_get_lite_xtalk_margin_kcps(
	VL53L1_DEV                          Dev,
	int16_t                           *pxtalk_margin)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	*pxtalk_margin = pdev->xtalk_cfg.lite_mode_crosstalk_margin_kcps;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_lite_xtalk_margin_kcps(
	VL53L1_DEV                     Dev,
	int16_t                        xtalk_margin)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdev->xtalk_cfg.lite_mode_crosstalk_margin_kcps = xtalk_margin;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_restore_xtalk_nvm_default(
	VL53L1_DEV                     Dev)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdev->xtalk_cfg.algo__crosstalk_compensation_plane_offset_kcps =
		pdev->xtalk_cfg.nvm_default__crosstalk_compensation_plane_offset_kcps;
	pdev->xtalk_cfg.algo__crosstalk_compensation_x_plane_gradient_kcps =
		pdev->xtalk_cfg.nvm_default__crosstalk_compensation_x_plane_gradient_kcps;
	pdev->xtalk_cfg.algo__crosstalk_compensation_y_plane_gradient_kcps =
		pdev->xtalk_cfg.nvm_default__crosstalk_compensation_y_plane_gradient_kcps;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error  VL53L1_disable_xtalk_compensation(
	VL53L1_DEV                 Dev)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdev->customer.algo__crosstalk_compensation_plane_offset_kcps =
		0x00;
	pdev->customer.algo__crosstalk_compensation_x_plane_gradient_kcps =
		0x00;
	pdev->customer.algo__crosstalk_compensation_y_plane_gradient_kcps =
		0x00;
	pdev->xtalk_cfg.global_crosstalk_compensation_enable = 0x00;
	if (status == VL53L1_ERROR_NONE) {
		pdev->xtalk_cfg.crosstalk_range_ignore_threshold_rate_mcps =
			0x0000;
	}
	if (status == VL53L1_ERROR_NONE) { 
		status =
			VL53L1_set_customer_nvm_managed(
				Dev,
				&(pdev->customer));
	}
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_get_lite_sigma_threshold(
	VL53L1_DEV                          Dev,
	uint16_t                           *plite_sigma)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	*plite_sigma =
			pdev->tim_cfg.range_config__sigma_thresh;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_lite_sigma_threshold(
	VL53L1_DEV                          Dev,
	uint16_t                           lite_sigma)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdev->tim_cfg.range_config__sigma_thresh = lite_sigma;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_get_lite_min_count_rate(
	VL53L1_DEV                          Dev,
	uint16_t                           *plite_mincountrate)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	*plite_mincountrate =
			pdev->tim_cfg.range_config__min_count_rate_rtn_limit_mcps;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_lite_min_count_rate(
	VL53L1_DEV                          Dev,
	uint16_t                            lite_mincountrate)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdev->tim_cfg.range_config__min_count_rate_rtn_limit_mcps =
		lite_mincountrate;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_get_vhv_loopbound(
	VL53L1_DEV                   Dev,
	uint8_t                     *pvhv_loopbound)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	*pvhv_loopbound = pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound / 4 ;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_vhv_loopbound(
	VL53L1_DEV                   Dev,
	uint8_t                      vhv_loopbound)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound =
			(pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound & 0x03) +
			(vhv_loopbound * 4);
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_get_vhv_config(
	VL53L1_DEV                   Dev,
	uint8_t                     *pvhv_init_en,
	uint8_t                     *pvhv_init_value)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	*pvhv_init_en    = (pdev->stat_nvm.vhv_config__init & 0x80) >> 7;
	*pvhv_init_value =
			(pdev->stat_nvm.vhv_config__init & 0x7F);
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_vhv_config(
	VL53L1_DEV                   Dev,
	uint8_t                      vhv_init_en,
	uint8_t                      vhv_init_value)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdev->stat_nvm.vhv_config__init =
		((vhv_init_en   & 0x01) << 7) +
		(vhv_init_value & 0x7F);
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_init_and_start_range(
	VL53L1_DEV                     Dev,
	uint8_t                        measurement_mode,
	VL53L1_DeviceConfigLevel       device_config_level)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	uint8_t buffer[VL53L1_MAX_I2C_XFER_SIZE];
	VL53L1_static_nvm_managed_t   *pstatic_nvm   = &(pdev->stat_nvm);
	VL53L1_customer_nvm_managed_t *pcustomer_nvm = &(pdev->customer);
	VL53L1_static_config_t        *pstatic       = &(pdev->stat_cfg);
	VL53L1_general_config_t       *pgeneral      = &(pdev->gen_cfg);
	VL53L1_timing_config_t        *ptiming       = &(pdev->tim_cfg);
	VL53L1_dynamic_config_t       *pdynamic      = &(pdev->dyn_cfg);
	VL53L1_system_control_t       *psystem       = &(pdev->sys_ctrl);
	VL53L1_ll_driver_state_t  *pstate   = &(pdev->ll_state);
	uint8_t  *pbuffer                   = &buffer[0];
	uint16_t i                          = 0;
	uint16_t i2c_index                  = 0;
	uint16_t i2c_buffer_offset_bytes    = 0;
	uint16_t i2c_buffer_size_bytes      = 0;
	LOG_FUNCTION_START("");
	pdev->measurement_mode = measurement_mode;
	psystem->system__mode_start =
		(psystem->system__mode_start &
		VL53L1_DEVICEMEASUREMENTMODE_STOP_MASK) |
		measurement_mode;
	pdev->stat_cfg.algo__range_ignore_threshold_mcps =
		pdev->xtalk_cfg.crosstalk_range_ignore_threshold_rate_mcps;
	if (pdev->low_power_auto_data.low_power_auto_range_count == 0xFF) {
		pdev->low_power_auto_data.low_power_auto_range_count = 0x0;
	}
	if ((pdev->low_power_auto_data.is_low_power_auto_mode == 1) &&
		(pdev->low_power_auto_data.low_power_auto_range_count == 0)) {
		pdev->low_power_auto_data.saved_interrupt_config =
			pdev->gen_cfg.system__interrupt_config_gpio;
		pdev->gen_cfg.system__interrupt_config_gpio = 1 << 5;
		if ((pdev->dyn_cfg.system__sequence_config & (
			VL53L1_SEQUENCE_MM1_EN | VL53L1_SEQUENCE_MM2_EN)) ==
				0x0) {
			pdev->customer.algo__part_to_part_range_offset_mm =
				pdev->customer.mm_config__outer_offset_mm * 4;
		} else {
			pdev->customer.algo__part_to_part_range_offset_mm = 0x0;
		}
		if (device_config_level <
				VL53L1_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS) {
			device_config_level =
				VL53L1_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS;
		}
	}
	if ((pdev->low_power_auto_data.is_low_power_auto_mode == 1) &&
		(pdev->low_power_auto_data.low_power_auto_range_count == 1)) {
		pdev->gen_cfg.system__interrupt_config_gpio =
			pdev->low_power_auto_data.saved_interrupt_config;
		device_config_level = VL53L1_DEVICECONFIGLEVEL_FULL;
	}
	switch (device_config_level) {
	case VL53L1_DEVICECONFIGLEVEL_FULL:
		i2c_index = VL53L1_STATIC_NVM_MANAGED_I2C_INDEX;
		break;
	case VL53L1_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS:
		i2c_index = VL53L1_CUSTOMER_NVM_MANAGED_I2C_INDEX;
		break;
	case VL53L1_DEVICECONFIGLEVEL_STATIC_ONWARDS:
		i2c_index = VL53L1_STATIC_CONFIG_I2C_INDEX;
		break;
	case VL53L1_DEVICECONFIGLEVEL_GENERAL_ONWARDS:
		i2c_index = VL53L1_GENERAL_CONFIG_I2C_INDEX;
		break;
	case VL53L1_DEVICECONFIGLEVEL_TIMING_ONWARDS:
		i2c_index = VL53L1_TIMING_CONFIG_I2C_INDEX;
		break;
	case VL53L1_DEVICECONFIGLEVEL_DYNAMIC_ONWARDS:
		i2c_index = VL53L1_DYNAMIC_CONFIG_I2C_INDEX;
		break;
	default:
		i2c_index = VL53L1_SYSTEM_CONTROL_I2C_INDEX;
		break;
	}
	i2c_buffer_size_bytes = \
			(VL53L1_SYSTEM_CONTROL_I2C_INDEX +
			 VL53L1_SYSTEM_CONTROL_I2C_SIZE_BYTES) -
			 i2c_index;
	pbuffer = &buffer[0];
	for (i = 0 ; i < i2c_buffer_size_bytes ; i++) {
		*pbuffer++ = 0;
	}
	if (device_config_level >= VL53L1_DEVICECONFIGLEVEL_FULL &&
		status == VL53L1_ERROR_NONE) {
		i2c_buffer_offset_bytes = \
			VL53L1_STATIC_NVM_MANAGED_I2C_INDEX - i2c_index;
		status =
			VL53L1_i2c_encode_static_nvm_managed(
				pstatic_nvm,
				VL53L1_STATIC_NVM_MANAGED_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes]);
	}
	if (device_config_level >= VL53L1_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS &&
		status == VL53L1_ERROR_NONE) {
		i2c_buffer_offset_bytes = \
			VL53L1_CUSTOMER_NVM_MANAGED_I2C_INDEX - i2c_index;
		status =
			VL53L1_i2c_encode_customer_nvm_managed(
				pcustomer_nvm,
				VL53L1_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes]);
	}
	if (device_config_level >= VL53L1_DEVICECONFIGLEVEL_STATIC_ONWARDS &&
		status == VL53L1_ERROR_NONE) {
		i2c_buffer_offset_bytes = \
			VL53L1_STATIC_CONFIG_I2C_INDEX - i2c_index;
		status =
			VL53L1_i2c_encode_static_config(
				pstatic,
				VL53L1_STATIC_CONFIG_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes]);
	}
	if (device_config_level >= VL53L1_DEVICECONFIGLEVEL_GENERAL_ONWARDS &&
		status == VL53L1_ERROR_NONE) {
		i2c_buffer_offset_bytes =
				VL53L1_GENERAL_CONFIG_I2C_INDEX - i2c_index;
		status =
			VL53L1_i2c_encode_general_config(
				pgeneral,
				VL53L1_GENERAL_CONFIG_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes]);
	}
	if (device_config_level >= VL53L1_DEVICECONFIGLEVEL_TIMING_ONWARDS &&
		status == VL53L1_ERROR_NONE) {
		i2c_buffer_offset_bytes = \
				VL53L1_TIMING_CONFIG_I2C_INDEX - i2c_index;
		status =
			VL53L1_i2c_encode_timing_config(
				ptiming,
				VL53L1_TIMING_CONFIG_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes]);
	}
	if (device_config_level >= VL53L1_DEVICECONFIGLEVEL_DYNAMIC_ONWARDS &&
		status == VL53L1_ERROR_NONE) {
		i2c_buffer_offset_bytes = \
			VL53L1_DYNAMIC_CONFIG_I2C_INDEX - i2c_index;
		if ((psystem->system__mode_start &
			VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK) ==
			VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK) {
			pdynamic->system__grouped_parameter_hold_0 = pstate->cfg_gph_id | 0x01;
			pdynamic->system__grouped_parameter_hold_1 = pstate->cfg_gph_id | 0x01;
			pdynamic->system__grouped_parameter_hold   = pstate->cfg_gph_id;
		}
		status =
			VL53L1_i2c_encode_dynamic_config(
				pdynamic,
				VL53L1_DYNAMIC_CONFIG_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes]);
	}
	if (status == VL53L1_ERROR_NONE) {
		i2c_buffer_offset_bytes = \
				VL53L1_SYSTEM_CONTROL_I2C_INDEX - i2c_index;
		status =
			VL53L1_i2c_encode_system_control(
				psystem,
				VL53L1_SYSTEM_CONTROL_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes]);
	}
	if (status == VL53L1_ERROR_NONE) {
		status =
			VL53L1_WriteMulti(
				Dev,
				i2c_index,
				buffer,
				(uint32_t)i2c_buffer_size_bytes);
	}
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_update_ll_driver_rd_state(Dev);
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_update_ll_driver_cfg_state(Dev);
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_stop_range(
	VL53L1_DEV     Dev)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
			VL53L1DevStructGetLLDriverHandle(Dev);
	pdev->sys_ctrl.system__mode_start =
			(pdev->sys_ctrl.system__mode_start & VL53L1_DEVICEMEASUREMENTMODE_STOP_MASK) |
			 VL53L1_DEVICEMEASUREMENTMODE_ABORT;
	status = VL53L1_set_system_control(
				Dev,
				&pdev->sys_ctrl);
	pdev->sys_ctrl.system__mode_start =
			(pdev->sys_ctrl.system__mode_start & VL53L1_DEVICEMEASUREMENTMODE_STOP_MASK);
	VL53L1_init_ll_driver_state(
			Dev,
			VL53L1_DEVICESTATE_SW_STANDBY);
	if (pdev->low_power_auto_data.is_low_power_auto_mode == 1)
		VL53L1_low_power_auto_data_stop_range(Dev);
	return status;
}
VL53L1_Error VL53L1_get_measurement_results(
	VL53L1_DEV                     Dev,
	VL53L1_DeviceResultsLevel      device_results_level)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	uint8_t buffer[VL53L1_MAX_I2C_XFER_SIZE];
	VL53L1_system_results_t   *psystem_results = &(pdev->sys_results);
	VL53L1_core_results_t     *pcore_results   = &(pdev->core_results);
	VL53L1_debug_results_t    *pdebug_results  = &(pdev->dbg_results);
	uint16_t i2c_index               = VL53L1_SYSTEM_RESULTS_I2C_INDEX;
	uint16_t i2c_buffer_offset_bytes = 0;
	uint16_t i2c_buffer_size_bytes   = 0;
	LOG_FUNCTION_START("");
	switch (device_results_level) {
	case VL53L1_DEVICERESULTSLEVEL_FULL:
		i2c_buffer_size_bytes =
				(VL53L1_DEBUG_RESULTS_I2C_INDEX +
				VL53L1_DEBUG_RESULTS_I2C_SIZE_BYTES) -
				i2c_index;
		break;
	case VL53L1_DEVICERESULTSLEVEL_UPTO_CORE:
		i2c_buffer_size_bytes =
				(VL53L1_CORE_RESULTS_I2C_INDEX +
				VL53L1_CORE_RESULTS_I2C_SIZE_BYTES) -
				i2c_index;
		break;
	default:
		i2c_buffer_size_bytes =
				VL53L1_SYSTEM_RESULTS_I2C_SIZE_BYTES;
		break;
	}
	if (status == VL53L1_ERROR_NONE) 
		status =
			VL53L1_ReadMulti(
				Dev,
				i2c_index,
				buffer,
				(uint32_t)i2c_buffer_size_bytes);
	if (device_results_level >= VL53L1_DEVICERESULTSLEVEL_FULL &&
		status == VL53L1_ERROR_NONE) {
		i2c_buffer_offset_bytes =
				VL53L1_DEBUG_RESULTS_I2C_INDEX - i2c_index;
		status =
			VL53L1_i2c_decode_debug_results(
				VL53L1_DEBUG_RESULTS_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes],
				pdebug_results);
	}
	if (device_results_level >= VL53L1_DEVICERESULTSLEVEL_UPTO_CORE &&
		status == VL53L1_ERROR_NONE) {
		i2c_buffer_offset_bytes =
				VL53L1_CORE_RESULTS_I2C_INDEX - i2c_index;
		status =
			VL53L1_i2c_decode_core_results(
				VL53L1_CORE_RESULTS_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes],
				pcore_results);
	}
	if (status == VL53L1_ERROR_NONE) {
		i2c_buffer_offset_bytes = 0;
		status =
			VL53L1_i2c_decode_system_results(
				VL53L1_SYSTEM_RESULTS_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes],
				psystem_results);
	}
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_get_device_results(
	VL53L1_DEV                    Dev,
	VL53L1_DeviceResultsLevel     device_results_level,
	VL53L1_range_results_t       *prange_results)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
			VL53L1DevStructGetLLDriverHandle(Dev);
	VL53L1_LLDriverResults_t *pres =
			VL53L1DevStructGetLLResultsHandle(Dev);
	VL53L1_range_results_t   *presults = &(pres->range_results);
	LOG_FUNCTION_START("");
	if (status == VL53L1_ERROR_NONE) 
		status = VL53L1_get_measurement_results(
						Dev,
						device_results_level);
	if (status == VL53L1_ERROR_NONE)
		VL53L1_copy_sys_and_core_results_to_range_results(
				(int32_t)pdev->gain_cal.standard_ranging_gain_factor,
				&(pdev->sys_results),
				&(pdev->core_results),
				presults);
	if (pdev->low_power_auto_data.is_low_power_auto_mode == 1) {
		if ((status == VL53L1_ERROR_NONE) &&
			(pdev->low_power_auto_data.low_power_auto_range_count == 0)) {
			status = VL53L1_low_power_auto_setup_manual_calibration(
					Dev);
			pdev->low_power_auto_data.low_power_auto_range_count = 1;
		} else if ((status == VL53L1_ERROR_NONE) &&
			(pdev->low_power_auto_data.low_power_auto_range_count == 1)) {
			pdev->low_power_auto_data.low_power_auto_range_count = 2;
		}
		if ((pdev->low_power_auto_data.low_power_auto_range_count != 0xFF) &&
			(status == VL53L1_ERROR_NONE)) {
			status = VL53L1_low_power_auto_update_DSS(
					Dev);
		}
	}
	presults->cfg_device_state = pdev->ll_state.cfg_device_state;
	presults->rd_device_state  = pdev->ll_state.rd_device_state;
	memcpy(
		prange_results,
		presults,
		sizeof(VL53L1_range_results_t));
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_check_ll_driver_rd_state(Dev);
#ifdef VL53L1_LOG_ENABLE
	if (status == VL53L1_ERROR_NONE)
		VL53L1_print_range_results(
			presults,
			"get_device_results():pdev->llresults.range_results.",
			VL53L1_TRACE_MODULE_RANGE_RESULTS_DATA);
#endif
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_clear_interrupt_and_enable_next_range(
	VL53L1_DEV        Dev,
	uint8_t           measurement_mode)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	LOG_FUNCTION_START("");
	if (status == VL53L1_ERROR_NONE) 
		status = VL53L1_init_and_start_range(
					Dev,
					measurement_mode,
					VL53L1_DEVICECONFIGLEVEL_GENERAL_ONWARDS);
	LOG_FUNCTION_END(status);
	return status;
}
void VL53L1_copy_sys_and_core_results_to_range_results(
	int32_t                           gain_factor,
	VL53L1_system_results_t          *psys,
	VL53L1_core_results_t            *pcore,
	VL53L1_range_results_t           *presults)
{
	uint8_t  i = 0;
	VL53L1_range_data_t *pdata;
	int32_t range_mm = 0;
	uint32_t tmpu32 = 0;
	LOG_FUNCTION_START("");
	presults->stream_count    = psys->result__stream_count;
	pdata = &(presults->data[0]);
	for (i = 0 ; i < 2 ; i++) {
		pdata->range_id     = i;
		pdata->time_stamp   = 0;
		if ((psys->result__stream_count == 0) &&
			((psys->result__range_status & VL53L1_RANGE_STATUS__RANGE_STATUS_MASK) ==
			VL53L1_DEVICEERROR_RANGECOMPLETE)) {
			pdata->range_status = VL53L1_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK;
		} else {
			pdata->range_status =
					psys->result__range_status & VL53L1_RANGE_STATUS__RANGE_STATUS_MASK;
		}
		switch (i) {
		case 0:
			if (psys->result__report_status == VL53L1_DEVICEREPORTSTATUS_MM1)
				pdata->actual_effective_spads =
					psys->result__mm_inner_actual_effective_spads_sd0;
			else if (psys->result__report_status == VL53L1_DEVICEREPORTSTATUS_MM2)
				pdata->actual_effective_spads =
						psys->result__mm_outer_actual_effective_spads_sd0;
			else
				pdata->actual_effective_spads =
					psys->result__dss_actual_effective_spads_sd0;
			pdata->peak_signal_count_rate_mcps =
				psys->result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
			pdata->avg_signal_count_rate_mcps =
				psys->result__avg_signal_count_rate_mcps_sd0;
			pdata->ambient_count_rate_mcps =
				psys->result__ambient_count_rate_mcps_sd0;
			tmpu32 = ((uint32_t)psys->result__sigma_sd0 << 5);
			if (tmpu32 > 0xFFFF) {
				tmpu32 = 0xFFFF;
			}
			pdata->sigma_mm = (uint16_t)tmpu32;
			pdata->median_phase =
				psys->result__phase_sd0;
			range_mm =
				(int32_t)psys->result__final_crosstalk_corrected_range_mm_sd0;
			range_mm *= gain_factor;
			range_mm += 0x0400;
			range_mm /= 0x0800;
			pdata->median_range_mm = (int16_t)range_mm;
			pdata->ranging_total_events =
				pcore->result_core__ranging_total_events_sd0;
			pdata->signal_total_events =
				pcore->result_core__signal_total_events_sd0;
			pdata->total_periods_elapsed =
				pcore->result_core__total_periods_elapsed_sd0;
			pdata->ambient_window_events =
				pcore->result_core__ambient_window_events_sd0;
			break;
		case 1:
			pdata->actual_effective_spads =
				psys->result__dss_actual_effective_spads_sd1;
			pdata->peak_signal_count_rate_mcps =
				psys->result__peak_signal_count_rate_mcps_sd1;
			pdata->avg_signal_count_rate_mcps =
				0xFFFF;
			pdata->ambient_count_rate_mcps =
				psys->result__ambient_count_rate_mcps_sd1;
			tmpu32 = ((uint32_t)psys->result__sigma_sd1 << 5);
			if (tmpu32 > 0xFFFF) {
				tmpu32 = 0xFFFF;
			}
			pdata->sigma_mm = (uint16_t)tmpu32;
			pdata->median_phase =
				psys->result__phase_sd1;
			range_mm =
				(int32_t)psys->result__final_crosstalk_corrected_range_mm_sd1;
			range_mm *= gain_factor;
			range_mm += 0x0400;
			range_mm /= 0x0800;
			pdata->median_range_mm = (int16_t)range_mm;
			pdata->ranging_total_events =
				pcore->result_core__ranging_total_events_sd1;
			pdata->signal_total_events =
				pcore->result_core__signal_total_events_sd1;
			pdata->total_periods_elapsed  =
				pcore->result_core__total_periods_elapsed_sd1;
			pdata->ambient_window_events =
				pcore->result_core__ambient_window_events_sd1;
			break;
		}
		pdata++;
	}
	presults->device_status = VL53L1_DEVICEERROR_NOUPDATE;
	switch (psys->result__range_status &
			VL53L1_RANGE_STATUS__RANGE_STATUS_MASK) {
	case VL53L1_DEVICEERROR_VCSELCONTINUITYTESTFAILURE:
	case VL53L1_DEVICEERROR_VCSELWATCHDOGTESTFAILURE:
	case VL53L1_DEVICEERROR_NOVHVVALUEFOUND:
	case VL53L1_DEVICEERROR_USERROICLIP:
	case VL53L1_DEVICEERROR_MULTCLIPFAIL:
		presults->device_status = (psys->result__range_status &
				VL53L1_RANGE_STATUS__RANGE_STATUS_MASK);
		presults->data[0].range_status = VL53L1_DEVICEERROR_NOUPDATE;
	break;
	}
	LOG_FUNCTION_END(0);
}
VL53L1_Error VL53L1_set_GPIO_interrupt_config(
	VL53L1_DEV                      Dev,
	VL53L1_GPIO_Interrupt_Mode	intr_mode_distance,
	VL53L1_GPIO_Interrupt_Mode	intr_mode_rate,
	uint8_t				intr_new_measure_ready,
	uint8_t				intr_no_target,
	uint8_t				intr_combined_mode,
	uint16_t			thresh_distance_high,
	uint16_t			thresh_distance_low,
	uint16_t			thresh_rate_high,
	uint16_t			thresh_rate_low
	)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	VL53L1_GPIO_interrupt_config_t *pintconf = &(pdev->gpio_interrupt_config);
	LOG_FUNCTION_START("");
	pintconf->intr_mode_distance = intr_mode_distance;
	pintconf->intr_mode_rate = intr_mode_rate;
	pintconf->intr_new_measure_ready = intr_new_measure_ready;
	pintconf->intr_no_target = intr_no_target;
	pintconf->intr_combined_mode = intr_combined_mode;
	pintconf->threshold_distance_high = thresh_distance_high;
	pintconf->threshold_distance_low = thresh_distance_low;
	pintconf->threshold_rate_high = thresh_rate_high;
	pintconf->threshold_rate_low = thresh_rate_low;
	pdev->gen_cfg.system__interrupt_config_gpio =
		VL53L1_encode_GPIO_interrupt_config(pintconf);
	status = VL53L1_set_GPIO_thresholds_from_struct(
			Dev,
			pintconf);
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_GPIO_interrupt_config_struct(
	VL53L1_DEV                      Dev,
	VL53L1_GPIO_interrupt_config_t	intconf)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	VL53L1_GPIO_interrupt_config_t *pintconf = &(pdev->gpio_interrupt_config);
	LOG_FUNCTION_START("");
	memcpy(pintconf, &(intconf), sizeof(VL53L1_GPIO_interrupt_config_t));
	pdev->gen_cfg.system__interrupt_config_gpio =
		VL53L1_encode_GPIO_interrupt_config(pintconf);
	status = VL53L1_set_GPIO_thresholds_from_struct(
			Dev,
			pintconf);
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_get_GPIO_interrupt_config(
	VL53L1_DEV                      Dev,
	VL53L1_GPIO_interrupt_config_t	*pintconf)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdev->gpio_interrupt_config = VL53L1_decode_GPIO_interrupt_config(
			pdev->gen_cfg.system__interrupt_config_gpio);
	pdev->gpio_interrupt_config.threshold_distance_high =
		pdev->dyn_cfg.system__thresh_high;
	pdev->gpio_interrupt_config.threshold_distance_low =
		pdev->dyn_cfg.system__thresh_low;
	pdev->gpio_interrupt_config.threshold_rate_high =
		pdev->gen_cfg.system__thresh_rate_high;
	pdev->gpio_interrupt_config.threshold_rate_low =
		pdev->gen_cfg.system__thresh_rate_low;
	if (pintconf == &(pdev->gpio_interrupt_config))	{
	} else {
		memcpy(pintconf, &(pdev->gpio_interrupt_config),
				sizeof(VL53L1_GPIO_interrupt_config_t));
	}
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_offset_calibration_mode(
	VL53L1_DEV                     Dev,
	VL53L1_OffsetCalibrationMode   offset_cal_mode)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdev->offset_calibration_mode = offset_cal_mode;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_get_offset_calibration_mode(
	VL53L1_DEV                     Dev,
	VL53L1_OffsetCalibrationMode  *poffset_cal_mode)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	*poffset_cal_mode = pdev->offset_calibration_mode;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_offset_correction_mode(
	VL53L1_DEV                     Dev,
	VL53L1_OffsetCorrectionMode    offset_cor_mode)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	pdev->offset_correction_mode = offset_cor_mode;
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_get_offset_correction_mode(
	VL53L1_DEV                     Dev,
	VL53L1_OffsetCorrectionMode   *poffset_cor_mode)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	*poffset_cor_mode = pdev->offset_correction_mode;
	LOG_FUNCTION_END(status);
	return status;
}
#ifdef VL53L1_DEBUG
VL53L1_Error VL53L1_get_tuning_debug_data(
	VL53L1_DEV                            Dev,
	VL53L1_tuning_parameters_t           *ptun_data)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	ptun_data->vl53l1_tuningparm_version =
		pdev->tuning_parms.tp_tuning_parm_version;
	ptun_data->vl53l1_tuningparm_key_table_version =
		pdev->tuning_parms.tp_tuning_parm_key_table_version;
	ptun_data->vl53l1_tuningparm_lld_version =
		pdev->tuning_parms.tp_tuning_parm_lld_version;
	ptun_data->vl53l1_tuningparm_lite_min_clip_mm =
		pdev->tuning_parms.tp_lite_min_clip;
	ptun_data->vl53l1_tuningparm_lite_long_sigma_thresh_mm =
		pdev->tuning_parms.tp_lite_long_sigma_thresh_mm;
	ptun_data->vl53l1_tuningparm_lite_med_sigma_thresh_mm =
		pdev->tuning_parms.tp_lite_med_sigma_thresh_mm;
	ptun_data->vl53l1_tuningparm_lite_short_sigma_thresh_mm =
		pdev->tuning_parms.tp_lite_short_sigma_thresh_mm;
	ptun_data->vl53l1_tuningparm_lite_long_min_count_rate_rtn_mcps =
		pdev->tuning_parms.tp_lite_long_min_count_rate_rtn_mcps;
	ptun_data->vl53l1_tuningparm_lite_med_min_count_rate_rtn_mcps =
		pdev->tuning_parms.tp_lite_med_min_count_rate_rtn_mcps;
	ptun_data->vl53l1_tuningparm_lite_short_min_count_rate_rtn_mcps =
		pdev->tuning_parms.tp_lite_short_min_count_rate_rtn_mcps;
	ptun_data->vl53l1_tuningparm_lite_sigma_est_pulse_width =
		pdev->tuning_parms.tp_lite_sigma_est_pulse_width_ns;
	ptun_data->vl53l1_tuningparm_lite_sigma_est_amb_width_ns =
		pdev->tuning_parms.tp_lite_sigma_est_amb_width_ns;
	ptun_data->vl53l1_tuningparm_lite_sigma_ref_mm =
		pdev->tuning_parms.tp_lite_sigma_ref_mm;
	ptun_data->vl53l1_tuningparm_lite_rit_mult =
		pdev->xtalk_cfg.crosstalk_range_ignore_threshold_mult;
	ptun_data->vl53l1_tuningparm_lite_seed_config =
		pdev->tuning_parms.tp_lite_seed_cfg ;
	ptun_data->vl53l1_tuningparm_lite_quantifier =
		pdev->tuning_parms.tp_lite_quantifier;
	ptun_data->vl53l1_tuningparm_lite_first_order_select =
		pdev->tuning_parms.tp_lite_first_order_select;
	ptun_data->vl53l1_tuningparm_lite_xtalk_margin_kcps =
		pdev->xtalk_cfg.lite_mode_crosstalk_margin_kcps;
	ptun_data->vl53l1_tuningparm_initial_phase_rtn_lite_long_range =
		pdev->tuning_parms.tp_init_phase_rtn_lite_long;
	ptun_data->vl53l1_tuningparm_initial_phase_rtn_lite_med_range =
		pdev->tuning_parms.tp_init_phase_rtn_lite_med;
	ptun_data->vl53l1_tuningparm_initial_phase_rtn_lite_short_range =
		pdev->tuning_parms.tp_init_phase_rtn_lite_short;
	ptun_data->vl53l1_tuningparm_initial_phase_ref_lite_long_range =
		pdev->tuning_parms.tp_init_phase_ref_lite_long;
	ptun_data->vl53l1_tuningparm_initial_phase_ref_lite_med_range =
		pdev->tuning_parms.tp_init_phase_ref_lite_med;
	ptun_data->vl53l1_tuningparm_initial_phase_ref_lite_short_range =
		pdev->tuning_parms.tp_init_phase_ref_lite_short;
	ptun_data->vl53l1_tuningparm_timed_seed_config =
		pdev->tuning_parms.tp_timed_seed_cfg;
	ptun_data->vl53l1_tuningparm_vhv_loopbound =
		pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound;
	ptun_data->vl53l1_tuningparm_refspadchar_device_test_mode =
		pdev->refspadchar.device_test_mode;
	ptun_data->vl53l1_tuningparm_refspadchar_vcsel_period =
		pdev->refspadchar.vcsel_period;
	ptun_data->vl53l1_tuningparm_refspadchar_phasecal_timeout_us =
		pdev->refspadchar.timeout_us;
	ptun_data->vl53l1_tuningparm_refspadchar_target_count_rate_mcps =
		pdev->refspadchar.target_count_rate_mcps;
	ptun_data->vl53l1_tuningparm_refspadchar_min_countrate_limit_mcps =
		pdev->refspadchar.min_count_rate_limit_mcps;
	ptun_data->vl53l1_tuningparm_refspadchar_max_countrate_limit_mcps =
		pdev->refspadchar.max_count_rate_limit_mcps;
	ptun_data->vl53l1_tuningparm_offset_cal_dss_rate_mcps =
		pdev->offsetcal_cfg.dss_config__target_total_rate_mcps;
	ptun_data->vl53l1_tuningparm_offset_cal_phasecal_timeout_us =
		pdev->offsetcal_cfg.phasecal_config_timeout_us;
	ptun_data->vl53l1_tuningparm_offset_cal_mm_timeout_us =
		pdev->offsetcal_cfg.mm_config_timeout_us;
	ptun_data->vl53l1_tuningparm_offset_cal_range_timeout_us =
		pdev->offsetcal_cfg.range_config_timeout_us;
	ptun_data->vl53l1_tuningparm_offset_cal_pre_samples =
		pdev->offsetcal_cfg.pre_num_of_samples;
	ptun_data->vl53l1_tuningparm_offset_cal_mm1_samples =
		pdev->offsetcal_cfg.mm1_num_of_samples;
	ptun_data->vl53l1_tuningparm_offset_cal_mm2_samples =
		pdev->offsetcal_cfg.mm2_num_of_samples;
	ptun_data->vl53l1_tuningparm_spadmap_vcsel_period =
		pdev->ssc_cfg.vcsel_period;
	ptun_data->vl53l1_tuningparm_spadmap_vcsel_start =
		pdev->ssc_cfg.vcsel_start;
	ptun_data->vl53l1_tuningparm_spadmap_rate_limit_mcps =
		pdev->ssc_cfg.rate_limit_mcps;
	ptun_data->vl53l1_tuningparm_lite_dss_config_target_total_rate_mcps =
		pdev->tuning_parms.tp_dss_target_lite_mcps;
	ptun_data->vl53l1_tuningparm_timed_dss_config_target_total_rate_mcps =
		pdev->tuning_parms.tp_dss_target_timed_mcps;
	ptun_data->vl53l1_tuningparm_lite_phasecal_config_timeout_us =
		pdev->tuning_parms.tp_phasecal_timeout_lite_us;
	ptun_data->vl53l1_tuningparm_timed_phasecal_config_timeout_us =
		pdev->tuning_parms.tp_phasecal_timeout_timed_us;
	ptun_data->vl53l1_tuningparm_lite_mm_config_timeout_us =
		pdev->tuning_parms.tp_mm_timeout_lite_us;
	ptun_data->vl53l1_tuningparm_timed_mm_config_timeout_us =
		pdev->tuning_parms.tp_mm_timeout_timed_us;
	ptun_data->vl53l1_tuningparm_lite_range_config_timeout_us =
		pdev->tuning_parms.tp_range_timeout_lite_us;
	ptun_data->vl53l1_tuningparm_timed_range_config_timeout_us =
		pdev->tuning_parms.tp_range_timeout_timed_us;
	ptun_data->vl53l1_tuningparm_lowpowerauto_vhv_loop_bound =
		pdev->low_power_auto_data.vhv_loop_bound;
	ptun_data->vl53l1_tuningparm_lowpowerauto_mm_config_timeout_us =
		pdev->tuning_parms.tp_mm_timeout_lpa_us;
	ptun_data->vl53l1_tuningparm_lowpowerauto_range_config_timeout_us =
		pdev->tuning_parms.tp_range_timeout_lpa_us;
	LOG_FUNCTION_END(status);
	return status;
}
#endif
VL53L1_Error VL53L1_get_tuning_parm(
	VL53L1_DEV                     Dev,
	VL53L1_TuningParms             tuning_parm_key,
	int32_t                       *ptuning_parm_value)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	switch (tuning_parm_key) {
	case VL53L1_TUNINGPARM_VERSION:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_tuning_parm_version;
	break;
	case VL53L1_TUNINGPARM_KEY_TABLE_VERSION:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_tuning_parm_key_table_version;
	break;
	case VL53L1_TUNINGPARM_LLD_VERSION:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_tuning_parm_lld_version;
	break;
	case VL53L1_TUNINGPARM_CONSISTENCY_LITE_PHASE_TOLERANCE:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_consistency_lite_phase_tolerance;
	break;
	case VL53L1_TUNINGPARM_PHASECAL_TARGET:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_phasecal_target;
	break;
	case VL53L1_TUNINGPARM_LITE_CAL_REPEAT_RATE:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_cal_repeat_rate;
	break;
	case VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR:
		*ptuning_parm_value =
				(int32_t)pdev->gain_cal.standard_ranging_gain_factor;
	break;
	case VL53L1_TUNINGPARM_LITE_MIN_CLIP_MM:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_lite_min_clip;
	break;
	case VL53L1_TUNINGPARM_LITE_LONG_SIGMA_THRESH_MM:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_lite_long_sigma_thresh_mm;
	break;
	case VL53L1_TUNINGPARM_LITE_MED_SIGMA_THRESH_MM:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_lite_med_sigma_thresh_mm;
	break;
	case VL53L1_TUNINGPARM_LITE_SHORT_SIGMA_THRESH_MM:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_lite_short_sigma_thresh_mm;
	break;
	case VL53L1_TUNINGPARM_LITE_LONG_MIN_COUNT_RATE_RTN_MCPS:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_lite_long_min_count_rate_rtn_mcps;
	break;
	case VL53L1_TUNINGPARM_LITE_MED_MIN_COUNT_RATE_RTN_MCPS:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_lite_med_min_count_rate_rtn_mcps;
	break;
	case VL53L1_TUNINGPARM_LITE_SHORT_MIN_COUNT_RATE_RTN_MCPS:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_lite_short_min_count_rate_rtn_mcps;
	break;
	case VL53L1_TUNINGPARM_LITE_SIGMA_EST_PULSE_WIDTH:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_lite_sigma_est_pulse_width_ns;
	break;
	case VL53L1_TUNINGPARM_LITE_SIGMA_EST_AMB_WIDTH_NS:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_lite_sigma_est_amb_width_ns;
	break;
	case VL53L1_TUNINGPARM_LITE_SIGMA_REF_MM:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_lite_sigma_ref_mm;
	break;
	case VL53L1_TUNINGPARM_LITE_RIT_MULT:
		*ptuning_parm_value =
				(int32_t)pdev->xtalk_cfg.crosstalk_range_ignore_threshold_mult;
	break;
	case VL53L1_TUNINGPARM_LITE_SEED_CONFIG:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_lite_seed_cfg ;
	break;
	case VL53L1_TUNINGPARM_LITE_QUANTIFIER:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_lite_quantifier;
	break;
	case VL53L1_TUNINGPARM_LITE_FIRST_ORDER_SELECT:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_lite_first_order_select;
	break;
	case VL53L1_TUNINGPARM_LITE_XTALK_MARGIN_KCPS:
		*ptuning_parm_value =
				(int32_t)pdev->xtalk_cfg.lite_mode_crosstalk_margin_kcps;
	break;
	case VL53L1_TUNINGPARM_INITIAL_PHASE_RTN_LITE_LONG_RANGE:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_init_phase_rtn_lite_long;
	break;
	case VL53L1_TUNINGPARM_INITIAL_PHASE_RTN_LITE_MED_RANGE:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_init_phase_rtn_lite_med;
	break;
	case VL53L1_TUNINGPARM_INITIAL_PHASE_RTN_LITE_SHORT_RANGE:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_init_phase_rtn_lite_short;
	break;
	case VL53L1_TUNINGPARM_INITIAL_PHASE_REF_LITE_LONG_RANGE:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_init_phase_ref_lite_long;
	break;
	case VL53L1_TUNINGPARM_INITIAL_PHASE_REF_LITE_MED_RANGE:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_init_phase_ref_lite_med;
	break;
	case VL53L1_TUNINGPARM_INITIAL_PHASE_REF_LITE_SHORT_RANGE:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_init_phase_ref_lite_short;
	break;
	case VL53L1_TUNINGPARM_TIMED_SEED_CONFIG:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_timed_seed_cfg;
	break;
	case VL53L1_TUNINGPARM_VHV_LOOPBOUND:
		*ptuning_parm_value =
				(int32_t)pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound;
	break;
	case VL53L1_TUNINGPARM_REFSPADCHAR_DEVICE_TEST_MODE:
		*ptuning_parm_value =
				(int32_t)pdev->refspadchar.device_test_mode;
	break;
	case VL53L1_TUNINGPARM_REFSPADCHAR_VCSEL_PERIOD:
		*ptuning_parm_value =
				(int32_t)pdev->refspadchar.vcsel_period;
	break;
	case VL53L1_TUNINGPARM_REFSPADCHAR_PHASECAL_TIMEOUT_US:
		*ptuning_parm_value =
				(int32_t)pdev->refspadchar.timeout_us;
	break;
	case VL53L1_TUNINGPARM_REFSPADCHAR_TARGET_COUNT_RATE_MCPS:
		*ptuning_parm_value =
				(int32_t)pdev->refspadchar.target_count_rate_mcps;
	break;
	case VL53L1_TUNINGPARM_REFSPADCHAR_MIN_COUNTRATE_LIMIT_MCPS:
		*ptuning_parm_value =
				(int32_t)pdev->refspadchar.min_count_rate_limit_mcps;
	break;
	case VL53L1_TUNINGPARM_REFSPADCHAR_MAX_COUNTRATE_LIMIT_MCPS:
		*ptuning_parm_value =
				(int32_t)pdev->refspadchar.max_count_rate_limit_mcps;
	break;
	case VL53L1_TUNINGPARM_OFFSET_CAL_DSS_RATE_MCPS:
		*ptuning_parm_value =
				(int32_t)pdev->offsetcal_cfg.dss_config__target_total_rate_mcps;;
	break;
	case VL53L1_TUNINGPARM_OFFSET_CAL_PHASECAL_TIMEOUT_US:
		*ptuning_parm_value =
				(int32_t)pdev->offsetcal_cfg.phasecal_config_timeout_us;
	break;
	case VL53L1_TUNINGPARM_OFFSET_CAL_MM_TIMEOUT_US:
		*ptuning_parm_value =
				(int32_t)pdev->offsetcal_cfg.mm_config_timeout_us;
	break;
	case VL53L1_TUNINGPARM_OFFSET_CAL_RANGE_TIMEOUT_US:
		*ptuning_parm_value =
				(int32_t)pdev->offsetcal_cfg.range_config_timeout_us;
	break;
	case VL53L1_TUNINGPARM_OFFSET_CAL_PRE_SAMPLES:
		*ptuning_parm_value =
				(int32_t)pdev->offsetcal_cfg.pre_num_of_samples;
	break;
	case VL53L1_TUNINGPARM_OFFSET_CAL_MM1_SAMPLES:
		*ptuning_parm_value =
			(int32_t)pdev->offsetcal_cfg.mm1_num_of_samples;
	break;
	case VL53L1_TUNINGPARM_OFFSET_CAL_MM2_SAMPLES:
		*ptuning_parm_value =
				(int32_t)pdev->offsetcal_cfg.mm2_num_of_samples;
	break;
	case VL53L1_TUNINGPARM_SPADMAP_VCSEL_PERIOD:
		*ptuning_parm_value =
				(int32_t)pdev->ssc_cfg.vcsel_period;
	break;
	case VL53L1_TUNINGPARM_SPADMAP_VCSEL_START:
		*ptuning_parm_value =
				(int32_t)pdev->ssc_cfg.vcsel_start;
	break;
	case VL53L1_TUNINGPARM_SPADMAP_RATE_LIMIT_MCPS:
		*ptuning_parm_value =
				(int32_t)pdev->ssc_cfg.rate_limit_mcps;
	break;
	case VL53L1_TUNINGPARM_LITE_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_dss_target_lite_mcps;
	break;
	case VL53L1_TUNINGPARM_TIMED_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_dss_target_timed_mcps;
	break;
	case VL53L1_TUNINGPARM_LITE_PHASECAL_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_phasecal_timeout_lite_us;
	break;
	case VL53L1_TUNINGPARM_TIMED_PHASECAL_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_phasecal_timeout_timed_us;
	break;
	case VL53L1_TUNINGPARM_LITE_MM_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_mm_timeout_lite_us;
	break;
	case VL53L1_TUNINGPARM_TIMED_MM_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_mm_timeout_timed_us;
	break;
	case VL53L1_TUNINGPARM_LITE_RANGE_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_range_timeout_lite_us;
	break;
	case VL53L1_TUNINGPARM_TIMED_RANGE_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_range_timeout_timed_us;
	break;
	case VL53L1_TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND:
		*ptuning_parm_value =
				(int32_t)pdev->low_power_auto_data.vhv_loop_bound;
	break;
	case VL53L1_TUNINGPARM_LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_mm_timeout_lpa_us;
	break;
	case VL53L1_TUNINGPARM_LOWPOWERAUTO_RANGE_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
				(int32_t)pdev->tuning_parms.tp_range_timeout_lpa_us;
	break;
	default:
		*ptuning_parm_value = 0x7FFFFFFF;
		status = VL53L1_ERROR_INVALID_PARAMS;
	break;
	}
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_set_tuning_parm(
	VL53L1_DEV            Dev,
	VL53L1_TuningParms    tuning_parm_key,
	int32_t               tuning_parm_value)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	switch (tuning_parm_key) {
	case VL53L1_TUNINGPARM_VERSION:
		pdev->tuning_parms.tp_tuning_parm_version =
				(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_KEY_TABLE_VERSION:
		pdev->tuning_parms.tp_tuning_parm_key_table_version =
					(uint16_t)tuning_parm_value;
		if ((uint16_t)tuning_parm_value
				!= VL53L1_TUNINGPARM_KEY_TABLE_VERSION_DEFAULT) {
			status = VL53L1_ERROR_TUNING_PARM_KEY_MISMATCH;
		}
	break;
	case VL53L1_TUNINGPARM_LLD_VERSION:
		pdev->tuning_parms.tp_tuning_parm_lld_version =
				(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_CONSISTENCY_LITE_PHASE_TOLERANCE:
		pdev->tuning_parms.tp_consistency_lite_phase_tolerance =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_PHASECAL_TARGET:
		pdev->tuning_parms.tp_phasecal_target =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_CAL_REPEAT_RATE:
		pdev->tuning_parms.tp_cal_repeat_rate =
				(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR:
		pdev->gain_cal.standard_ranging_gain_factor =
				(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_MIN_CLIP_MM:
		pdev->tuning_parms.tp_lite_min_clip =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_LONG_SIGMA_THRESH_MM:
		pdev->tuning_parms.tp_lite_long_sigma_thresh_mm =
				(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_MED_SIGMA_THRESH_MM:
		pdev->tuning_parms.tp_lite_med_sigma_thresh_mm =
				(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_SHORT_SIGMA_THRESH_MM:
		pdev->tuning_parms.tp_lite_short_sigma_thresh_mm =
				(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_LONG_MIN_COUNT_RATE_RTN_MCPS:
		pdev->tuning_parms.tp_lite_long_min_count_rate_rtn_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_MED_MIN_COUNT_RATE_RTN_MCPS:
		pdev->tuning_parms.tp_lite_med_min_count_rate_rtn_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_SHORT_MIN_COUNT_RATE_RTN_MCPS:
		pdev->tuning_parms.tp_lite_short_min_count_rate_rtn_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_SIGMA_EST_PULSE_WIDTH:
		pdev->tuning_parms.tp_lite_sigma_est_pulse_width_ns =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_SIGMA_EST_AMB_WIDTH_NS:
		pdev->tuning_parms.tp_lite_sigma_est_amb_width_ns =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_SIGMA_REF_MM:
		pdev->tuning_parms.tp_lite_sigma_ref_mm =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_RIT_MULT:
		pdev->xtalk_cfg.crosstalk_range_ignore_threshold_mult =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_SEED_CONFIG:
		pdev->tuning_parms.tp_lite_seed_cfg =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_QUANTIFIER:
		pdev->tuning_parms.tp_lite_quantifier =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_FIRST_ORDER_SELECT:
		pdev->tuning_parms.tp_lite_first_order_select =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_XTALK_MARGIN_KCPS:
		pdev->xtalk_cfg.lite_mode_crosstalk_margin_kcps =
				(int16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_INITIAL_PHASE_RTN_LITE_LONG_RANGE:
		pdev->tuning_parms.tp_init_phase_rtn_lite_long =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_INITIAL_PHASE_RTN_LITE_MED_RANGE:
		pdev->tuning_parms.tp_init_phase_rtn_lite_med =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_INITIAL_PHASE_RTN_LITE_SHORT_RANGE:
		pdev->tuning_parms.tp_init_phase_rtn_lite_short =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_INITIAL_PHASE_REF_LITE_LONG_RANGE:
		pdev->tuning_parms.tp_init_phase_ref_lite_long =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_INITIAL_PHASE_REF_LITE_MED_RANGE:
		pdev->tuning_parms.tp_init_phase_ref_lite_med =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_INITIAL_PHASE_REF_LITE_SHORT_RANGE:
		pdev->tuning_parms.tp_init_phase_ref_lite_short =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_TIMED_SEED_CONFIG:
		pdev->tuning_parms.tp_timed_seed_cfg =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_VHV_LOOPBOUND:
		pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_REFSPADCHAR_DEVICE_TEST_MODE:
		pdev->refspadchar.device_test_mode =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_REFSPADCHAR_VCSEL_PERIOD:
		pdev->refspadchar.vcsel_period =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_REFSPADCHAR_PHASECAL_TIMEOUT_US:
		pdev->refspadchar.timeout_us =
				(uint32_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_REFSPADCHAR_TARGET_COUNT_RATE_MCPS:
		pdev->refspadchar.target_count_rate_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_REFSPADCHAR_MIN_COUNTRATE_LIMIT_MCPS:
		pdev->refspadchar.min_count_rate_limit_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_REFSPADCHAR_MAX_COUNTRATE_LIMIT_MCPS:
		pdev->refspadchar.max_count_rate_limit_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_OFFSET_CAL_DSS_RATE_MCPS:
		pdev->offsetcal_cfg.dss_config__target_total_rate_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_OFFSET_CAL_PHASECAL_TIMEOUT_US:
		pdev->offsetcal_cfg.phasecal_config_timeout_us =
				(uint32_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_OFFSET_CAL_MM_TIMEOUT_US:
		pdev->offsetcal_cfg.mm_config_timeout_us =
				(uint32_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_OFFSET_CAL_RANGE_TIMEOUT_US:
		pdev->offsetcal_cfg.range_config_timeout_us =
				(uint32_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_OFFSET_CAL_PRE_SAMPLES:
		pdev->offsetcal_cfg.pre_num_of_samples =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_OFFSET_CAL_MM1_SAMPLES:
		pdev->offsetcal_cfg.mm1_num_of_samples =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_OFFSET_CAL_MM2_SAMPLES:
		pdev->offsetcal_cfg.mm2_num_of_samples =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_SPADMAP_VCSEL_PERIOD:
		pdev->ssc_cfg.vcsel_period =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_SPADMAP_VCSEL_START:
		pdev->ssc_cfg.vcsel_start =
				(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_SPADMAP_RATE_LIMIT_MCPS:
		pdev->ssc_cfg.rate_limit_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
		pdev->tuning_parms.tp_dss_target_lite_mcps =
			(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_TIMED_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
		pdev->tuning_parms.tp_dss_target_timed_mcps =
			(uint16_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_PHASECAL_CONFIG_TIMEOUT_US:
		pdev->tuning_parms.tp_phasecal_timeout_lite_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_TIMED_PHASECAL_CONFIG_TIMEOUT_US:
		pdev->tuning_parms.tp_phasecal_timeout_timed_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_MM_CONFIG_TIMEOUT_US:
		pdev->tuning_parms.tp_mm_timeout_lite_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_TIMED_MM_CONFIG_TIMEOUT_US:
		pdev->tuning_parms.tp_mm_timeout_timed_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LITE_RANGE_CONFIG_TIMEOUT_US:
		pdev->tuning_parms.tp_range_timeout_lite_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_TIMED_RANGE_CONFIG_TIMEOUT_US:
		pdev->tuning_parms.tp_range_timeout_timed_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND:
		pdev->low_power_auto_data.vhv_loop_bound =
			(uint8_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US:
		pdev->tuning_parms.tp_mm_timeout_lpa_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53L1_TUNINGPARM_LOWPOWERAUTO_RANGE_CONFIG_TIMEOUT_US:
		pdev->tuning_parms.tp_range_timeout_lpa_us =
			(uint32_t)tuning_parm_value;
	break;
	default:
		status = VL53L1_ERROR_INVALID_PARAMS;
	break;
	}
	LOG_FUNCTION_END(status);
	return status;
}