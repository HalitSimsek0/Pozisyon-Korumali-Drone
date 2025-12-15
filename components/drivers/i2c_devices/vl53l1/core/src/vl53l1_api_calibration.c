#include "vl53l1_ll_def.h"
#include "vl53l1_ll_device.h"
#include "vl53l1_platform.h"
#include "vl53l1_register_map.h"
#include "vl53l1_register_funcs.h"
#include "vl53l1_register_settings.h"
#include "vl53l1_core.h"
#include "vl53l1_wait.h"
#include "vl53l1_api_preset_modes.h"
#include "vl53l1_silicon_core.h"
#include "vl53l1_api_core.h"
#include "vl53l1_api_calibration.h"
#ifdef VL53L1_LOG_ENABLE
  #include "vl53l1_api_debug.h"
#endif
#ifdef VL53L1_LOGGING
  #include "vl53l1_debug.h"
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
#ifndef VL53L1_NOCALIB
VL53L1_Error VL53L1_run_ref_spad_char(
	VL53L1_DEV        Dev,
	VL53L1_Error     *pcal_status)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	uint8_t comms_buffer[6];
	VL53L1_refspadchar_config_t *prefspadchar  = &(pdev->refspadchar);
	LOG_FUNCTION_START("");
	if (status == VL53L1_ERROR_NONE) 
		status = VL53L1_enable_powerforce(Dev);
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_set_ref_spad_char_config(
				Dev,
				prefspadchar->vcsel_period,
				prefspadchar->timeout_us,
				prefspadchar->target_count_rate_mcps,
				prefspadchar->max_count_rate_limit_mcps,
				prefspadchar->min_count_rate_limit_mcps,
				pdev->stat_nvm.osc_measured__fast_osc__frequency);
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_run_device_test(
					Dev,
					prefspadchar->device_test_mode);
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_ReadMulti(
				Dev,
				VL53L1_REF_SPAD_CHAR_RESULT__NUM_ACTUAL_REF_SPADS,
				comms_buffer,
				2);
	if (status == VL53L1_ERROR_NONE) {
		pdev->dbg_results.ref_spad_char_result__num_actual_ref_spads =
				comms_buffer[0];
		pdev->dbg_results.ref_spad_char_result__ref_location =
				comms_buffer[1];
	}
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_WriteMulti(
				Dev,
				VL53L1_REF_SPAD_MAN__NUM_REQUESTED_REF_SPADS,
				comms_buffer,
				2);
	if (status == VL53L1_ERROR_NONE) {
		pdev->customer.ref_spad_man__num_requested_ref_spads =
				comms_buffer[0];
		pdev->customer.ref_spad_man__ref_location =
				comms_buffer[1];
	}
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_ReadMulti(
				Dev,
				VL53L1_RESULT__SPARE_0_SD1,
				comms_buffer,
				6);
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_WriteMulti(
				Dev,
				VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_REF_0,
				comms_buffer,
				6);
	if (status == VL53L1_ERROR_NONE) {
		pdev->customer.global_config__spad_enables_ref_0 =
				comms_buffer[0];
		pdev->customer.global_config__spad_enables_ref_1 =
				comms_buffer[1];
		pdev->customer.global_config__spad_enables_ref_2 =
				comms_buffer[2];
		pdev->customer.global_config__spad_enables_ref_3 =
				comms_buffer[3];
		pdev->customer.global_config__spad_enables_ref_4 =
				comms_buffer[4];
		pdev->customer.global_config__spad_enables_ref_5 =
				comms_buffer[5];
	}
#ifdef VL53L1_LOG_ENABLE
	if (status == VL53L1_ERROR_NONE)
		VL53L1_print_customer_nvm_managed(
			&(pdev->customer),
			"run_ref_spad_char():pdev->lldata.customer.",
			VL53L1_TRACE_MODULE_REF_SPAD_CHAR);
#endif
	if (status == VL53L1_ERROR_NONE) {
		switch (pdev->sys_results.result__range_status) {
		case VL53L1_DEVICEERROR_REFSPADCHARNOTENOUGHDPADS:
			status = VL53L1_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS;
			break;
		case VL53L1_DEVICEERROR_REFSPADCHARMORETHANTARGET:
			status = VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH;
			break;
		case VL53L1_DEVICEERROR_REFSPADCHARLESSTHANTARGET:
			status = VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW;
			break;
		}
	}
	*pcal_status = status;
	IGNORE_STATUS(
		IGNORE_REF_SPAD_CHAR_NOT_ENOUGH_SPADS,
		VL53L1_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS,
		status);
	IGNORE_STATUS(
		IGNORE_REF_SPAD_CHAR_RATE_TOO_HIGH,
		VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH,
		status);
	IGNORE_STATUS(
		IGNORE_REF_SPAD_CHAR_RATE_TOO_LOW,
		VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW,
		status);
	LOG_FUNCTION_END(status);
	return status;
}
VL53L1_Error VL53L1_run_offset_calibration(
	VL53L1_DEV	                  Dev,
	int16_t                       cal_distance_mm,
	VL53L1_Error                 *pcal_status)
{
	VL53L1_Error status        = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
		VL53L1DevStructGetLLDriverHandle(Dev);
	VL53L1_DevicePresetModes device_preset_modes[VL53L1_MAX_OFFSET_RANGE_RESULTS];
	VL53L1_range_results_t      range_results;
	VL53L1_range_results_t     *prange_results = &range_results;
	VL53L1_range_data_t        *prange_data = NULL;
	VL53L1_offset_range_data_t *poffset     = NULL;
	uint8_t  i                      = 0;
	uint8_t  m                      = 0;
	uint8_t  measurement_mode       =
		VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK;
	uint16_t manual_effective_spads =
		pdev->gen_cfg.dss_config__manual_effective_spads_select;
	uint8_t num_of_samples[VL53L1_MAX_OFFSET_RANGE_RESULTS];
	LOG_FUNCTION_START("");
	switch (pdev->offset_calibration_mode) {
	default:
		device_preset_modes[0] =
			VL53L1_DEVICEPRESETMODE_STANDARD_RANGING;
		device_preset_modes[1] =
			VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_MM1_CAL;
		device_preset_modes[2] =
			VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_MM2_CAL;
	break;
	}
	num_of_samples[0] = pdev->offsetcal_cfg.pre_num_of_samples;
	num_of_samples[1] = pdev->offsetcal_cfg.mm1_num_of_samples;
	num_of_samples[2] = pdev->offsetcal_cfg.mm2_num_of_samples;
	switch (pdev->offset_calibration_mode) {
	case VL53L1_OFFSETCALIBRATIONMODE__MM1_MM2__STANDARD_PRE_RANGE_ONLY:
		pdev->offset_results.active_results  = 1;
	break;
	default:
		pdev->customer.mm_config__inner_offset_mm  = 0;
		pdev->customer.mm_config__outer_offset_mm  = 0;
		pdev->offset_results.active_results  =
			VL53L1_MAX_OFFSET_RANGE_RESULTS;
	break;
	}
	pdev->customer.algo__part_to_part_range_offset_mm = 0;
	pdev->offset_results.max_results           = VL53L1_MAX_OFFSET_RANGE_RESULTS;
	pdev->offset_results.cal_distance_mm       = cal_distance_mm;
	for (m = 0 ; m <  VL53L1_MAX_OFFSET_RANGE_RESULTS; m++) {
		poffset = &(pdev->offset_results.data[m]);
		poffset->preset_mode         = 0;
		poffset->no_of_samples       = 0;
		poffset->effective_spads     = 0;
		poffset->peak_rate_mcps      = 0;
		poffset->sigma_mm            = 0;
		poffset->median_range_mm     = 0;
	}
	for (m = 0 ; m < pdev->offset_results.active_results ; m++) {
		poffset = &(pdev->offset_results.data[m]);
		poffset->preset_mode         = device_preset_modes[m];
		if (status == VL53L1_ERROR_NONE)
			status =
				VL53L1_set_preset_mode(
					Dev,
					device_preset_modes[m],
					pdev->offsetcal_cfg.dss_config__target_total_rate_mcps,
					pdev->offsetcal_cfg.phasecal_config_timeout_us,
					pdev->offsetcal_cfg.mm_config_timeout_us,
					pdev->offsetcal_cfg.range_config_timeout_us,
					100);
		pdev->gen_cfg.dss_config__manual_effective_spads_select =
				manual_effective_spads;
		if (status == VL53L1_ERROR_NONE)
			status =
				VL53L1_init_and_start_range(
					Dev,
					measurement_mode,
					VL53L1_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS);
		for (i = 0 ; i <= (num_of_samples[m]+2) ; i++) {
			if (status == VL53L1_ERROR_NONE)
				status =
					VL53L1_wait_for_range_completion(Dev);
			if (status == VL53L1_ERROR_NONE)
				status =
					VL53L1_get_device_results(
						Dev,
						VL53L1_DEVICERESULTSLEVEL_FULL,
						prange_results);
			prange_data  = &(prange_results->data[0]);
			if (prange_results->stream_count   > 1) {
				if (prange_data->range_status ==
						VL53L1_DEVICEERROR_RANGECOMPLETE) {
					poffset->no_of_samples++;
					poffset->effective_spads +=
						(uint32_t)prange_data->actual_effective_spads;
					poffset->peak_rate_mcps  +=
						(uint32_t)prange_data->peak_signal_count_rate_mcps;
					poffset->sigma_mm        +=
						(uint32_t)prange_data->sigma_mm;
					poffset->median_range_mm +=
						(int32_t)prange_data->median_range_mm;
					poffset->dss_config__roi_mode_control =
						pdev->gen_cfg.dss_config__roi_mode_control;
					poffset->dss_config__manual_effective_spads_select =
						pdev->gen_cfg.dss_config__manual_effective_spads_select;
				}
			}
			if (status == VL53L1_ERROR_NONE)
				status =
					VL53L1_wait_for_firmware_ready(Dev);
			if (status == VL53L1_ERROR_NONE)
				status =
					VL53L1_clear_interrupt_and_enable_next_range(
						Dev,
						measurement_mode);
		}
		if (status == VL53L1_ERROR_NONE)
			status = VL53L1_stop_range(Dev);
		if (status == VL53L1_ERROR_NONE)
			status = VL53L1_WaitUs(Dev, 1000);
		if (poffset->no_of_samples > 0) {
			poffset->effective_spads += (poffset->no_of_samples/2);
			poffset->effective_spads /= poffset->no_of_samples;
			poffset->peak_rate_mcps  += (poffset->no_of_samples/2);
			poffset->peak_rate_mcps  /= poffset->no_of_samples;
			poffset->sigma_mm        += (poffset->no_of_samples/2);
			poffset->sigma_mm        /= poffset->no_of_samples;
			poffset->median_range_mm += (poffset->no_of_samples/2);
			poffset->median_range_mm /= poffset->no_of_samples;
			poffset->range_mm_offset  =  (int32_t)cal_distance_mm;
			poffset->range_mm_offset -= poffset->median_range_mm;
			if (poffset->preset_mode ==
				VL53L1_DEVICEPRESETMODE_STANDARD_RANGING)
				manual_effective_spads =
					(uint16_t)poffset->effective_spads;
		}
	}
	switch (pdev->offset_calibration_mode) {
	case VL53L1_OFFSETCALIBRATIONMODE__MM1_MM2__STANDARD_PRE_RANGE_ONLY:
		pdev->customer.mm_config__inner_offset_mm +=
			(int16_t)pdev->offset_results.data[0].range_mm_offset;
		pdev->customer.mm_config__outer_offset_mm +=
			(int16_t)pdev->offset_results.data[0].range_mm_offset;
	break;
	default:
		pdev->customer.mm_config__inner_offset_mm =
			(int16_t)pdev->offset_results.data[1].range_mm_offset;
		pdev->customer.mm_config__outer_offset_mm =
			(int16_t)pdev->offset_results.data[2].range_mm_offset;
		pdev->customer.algo__part_to_part_range_offset_mm = 0;
		pdev->add_off_cal_data.result__mm_inner_actual_effective_spads =
			(uint16_t)pdev->offset_results.data[1].effective_spads;
		pdev->add_off_cal_data.result__mm_outer_actual_effective_spads =
			(uint16_t)pdev->offset_results.data[2].effective_spads;
		pdev->add_off_cal_data.result__mm_inner_peak_signal_count_rtn_mcps =
			(uint16_t)pdev->offset_results.data[1].peak_rate_mcps;
		pdev->add_off_cal_data.result__mm_outer_peak_signal_count_rtn_mcps =
			(uint16_t)pdev->offset_results.data[2].peak_rate_mcps;
		break;
	}
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_set_customer_nvm_managed(
				Dev,
				&(pdev->customer));
	for (m = 0 ; m < pdev->offset_results.active_results ; m++) {
		poffset = &(pdev->offset_results.data[m]);
		if (status == VL53L1_ERROR_NONE) {
			pdev->offset_results.cal_report = m;
			if (poffset->no_of_samples < num_of_samples[m])
				status = VL53L1_WARNING_OFFSET_CAL_MISSING_SAMPLES;
			if (m == 0 && poffset->sigma_mm >
				((uint32_t)VL53L1_OFFSET_CAL_MAX_SIGMA_MM<<5))
				status = VL53L1_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH;
			if (poffset->peak_rate_mcps >
				VL53L1_OFFSET_CAL_MAX_PRE_PEAK_RATE_MCPS)
				status = VL53L1_WARNING_OFFSET_CAL_RATE_TOO_HIGH;
			if (poffset->dss_config__manual_effective_spads_select <
				VL53L1_OFFSET_CAL_MIN_EFFECTIVE_SPADS)
				status = VL53L1_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW;
			if (poffset->dss_config__manual_effective_spads_select == 0)
				status = VL53L1_ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL;
			if (poffset->no_of_samples == 0)
				status = VL53L1_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL;
		}
	}
	pdev->offset_results.cal_status = status;
	*pcal_status = pdev->offset_results.cal_status;
	IGNORE_STATUS(
		IGNORE_OFFSET_CAL_MISSING_SAMPLES,
		VL53L1_WARNING_OFFSET_CAL_MISSING_SAMPLES,
		status);
	IGNORE_STATUS(
		IGNORE_OFFSET_CAL_SIGMA_TOO_HIGH,
		VL53L1_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH,
		status);
	IGNORE_STATUS(
		IGNORE_OFFSET_CAL_RATE_TOO_HIGH,
		VL53L1_WARNING_OFFSET_CAL_RATE_TOO_HIGH,
		status);
	IGNORE_STATUS(
		IGNORE_OFFSET_CAL_SPAD_COUNT_TOO_LOW,
		VL53L1_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW,
		status);
#ifdef VL53L1_LOG_ENABLE
	VL53L1_print_customer_nvm_managed(
		&(pdev->customer),
		"run_offset_calibration():pdev->lldata.customer.",
		VL53L1_TRACE_MODULE_OFFSET_DATA);
	VL53L1_print_additional_offset_cal_data(
		&(pdev->add_off_cal_data),
		"run_offset_calibration():pdev->lldata.add_off_cal_data.",
		VL53L1_TRACE_MODULE_OFFSET_DATA);
	VL53L1_print_offset_range_results(
		&(pdev->offset_results),
		"run_offset_calibration():pdev->lldata.offset_results.",
		VL53L1_TRACE_MODULE_OFFSET_DATA);
#endif
	LOG_FUNCTION_END(status);
	return status;
}
#endif
#ifndef VL53L1_NOCALIB
VL53L1_Error VL53L1_run_spad_rate_map(
	VL53L1_DEV                 Dev,
	VL53L1_DeviceTestMode      device_test_mode,
	VL53L1_DeviceSscArray      array_select,
	uint32_t                   ssc_config_timeout_us,
    VL53L1_spad_rate_data_t   *pspad_rate_data)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev =
		VL53L1DevStructGetLLDriverHandle(Dev);
	LOG_FUNCTION_START("");
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_enable_powerforce(Dev);
	if (status == VL53L1_ERROR_NONE) {
		pdev->ssc_cfg.array_select = array_select;
		pdev->ssc_cfg.timeout_us   = ssc_config_timeout_us;
		status =
			VL53L1_set_ssc_config(
				Dev,
				&(pdev->ssc_cfg),
				pdev->stat_nvm.osc_measured__fast_osc__frequency);
	}
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_run_device_test(
				Dev,
				device_test_mode);
    if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_get_spad_rate_data(
				Dev,
				pspad_rate_data);
	if (device_test_mode == VL53L1_DEVICETESTMODE_LCR_VCSEL_ON)
		pspad_rate_data->fractional_bits =  7;
	else
		pspad_rate_data->fractional_bits = 15;
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_disable_powerforce(Dev);
#ifdef VL53L1_LOG_ENABLE
    if (status == VL53L1_ERROR_NONE) {
		VL53L1_print_spad_rate_data(
			pspad_rate_data,
			"run_spad_rate_map():",
			VL53L1_TRACE_MODULE_SPAD_RATE_MAP);
		VL53L1_print_spad_rate_map(
			pspad_rate_data,
			"run_spad_rate_map():",
			VL53L1_TRACE_MODULE_SPAD_RATE_MAP);
    }
#endif
	LOG_FUNCTION_END(status);
	return status;
}
#endif
#ifndef VL53L1_NOCALIB
VL53L1_Error VL53L1_run_device_test(
	VL53L1_DEV             Dev,
	VL53L1_DeviceTestMode  device_test_mode)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
	uint8_t      comms_buffer[2];
	uint8_t      gpio_hv_mux__ctrl = 0;
	LOG_FUNCTION_START("");
	if (status == VL53L1_ERROR_NONE) 
		status =
			VL53L1_RdByte(
				Dev,
				VL53L1_GPIO_HV_MUX__CTRL,
				&gpio_hv_mux__ctrl);
	if (status == VL53L1_ERROR_NONE)
		pdev->stat_cfg.gpio_hv_mux__ctrl = gpio_hv_mux__ctrl;
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_start_test(
					Dev,
					device_test_mode);
	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_wait_for_test_completion(Dev);
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_ReadMulti(
				Dev,
				VL53L1_RESULT__RANGE_STATUS,
				comms_buffer,
				2);
	if (status == VL53L1_ERROR_NONE) {
		pdev->sys_results.result__range_status  = comms_buffer[0];
		pdev->sys_results.result__report_status = comms_buffer[1];
	}
	pdev->sys_results.result__range_status &=
		VL53L1_RANGE_STATUS__RANGE_STATUS_MASK;
	if (status == VL53L1_ERROR_NONE) {
		trace_print(
			VL53L1_TRACE_LEVEL_INFO,
			"    Device Test Complete:\n\t%-32s = %3u\n\t%-32s = %3u\n",
			"result__range_status",
			pdev->sys_results.result__range_status,
			"result__report_status",
			pdev->sys_results.result__report_status);
		if (status == VL53L1_ERROR_NONE)
			status = VL53L1_clear_interrupt(Dev);
	}
	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_start_test(
				Dev,
				0x00);
	LOG_FUNCTION_END(status);
	return status;
}
#endif