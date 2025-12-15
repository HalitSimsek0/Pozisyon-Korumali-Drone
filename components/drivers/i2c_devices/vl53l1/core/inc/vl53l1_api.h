#ifndef _VL53L1_API_H_
#define _VL53L1_API_H_
#include "vl53l1_api_strings.h"
#include "vl53l1_api_core.h"
#ifdef __cplusplus
extern "C"
{
#endif
#if !defined(VL53L1DevDataGet)
#warning "Usage of PALDevDataGet is deprecated define VL53L1DevDataGet instead\
	in your vl53l1_platform_user_data.h file"
#define VL53L1DevDataGet(Dev, field) (Dev->Data.field)
#endif
#if !defined(VL53L1DevDataSet)
#warning "Usage of PALDevDataSet is deprecated define VL53L1DevDataSet instead\
	in your vl53l1_platform_user_data.h file"
#define VL53L1DevDataSet(Dev, field, data) ((Dev->Data.field) = (data))
#endif
VL53L1_Error VL53L1_GetVersion(VL53L1_Version_t *pVersion);
VL53L1_Error VL53L1_GetProductRevision(VL53L1_DEV Dev,
	uint8_t *pProductRevisionMajor, uint8_t *pProductRevisionMinor);
VL53L1_Error VL53L1_GetDeviceInfo(VL53L1_DEV Dev,
	VL53L1_DeviceInfo_t *pVL53L1_DeviceInfo);
VL53L1_Error VL53L1_GetRangeStatusString(uint8_t RangeStatus,
	char *pRangeStatusString);
VL53L1_Error VL53L1_GetPalErrorString(VL53L1_Error PalErrorCode,
	char *pPalErrorString);
VL53L1_Error VL53L1_GetPalStateString(VL53L1_State PalStateCode,
	char *pPalStateString);
VL53L1_Error VL53L1_GetPalState(VL53L1_DEV Dev,
	VL53L1_State *pPalState);
VL53L1_Error VL53L1_SetDeviceAddress(VL53L1_DEV Dev,
	uint8_t DeviceAddress);
VL53L1_Error VL53L1_DataInit(VL53L1_DEV Dev);
VL53L1_Error VL53L1_StaticInit(VL53L1_DEV Dev);
VL53L1_Error VL53L1_WaitDeviceBooted(VL53L1_DEV Dev);
VL53L1_Error VL53L1_SetPresetMode(VL53L1_DEV Dev,
		VL53L1_PresetModes PresetMode);
VL53L1_Error VL53L1_GetPresetMode(VL53L1_DEV Dev,
		VL53L1_PresetModes *pPresetMode);
VL53L1_Error VL53L1_SetDistanceMode(VL53L1_DEV Dev,
		VL53L1_DistanceModes DistanceMode);
VL53L1_Error VL53L1_GetDistanceMode(VL53L1_DEV Dev,
		VL53L1_DistanceModes *pDistanceMode);
VL53L1_Error VL53L1_SetMeasurementTimingBudgetMicroSeconds(
	VL53L1_DEV Dev, uint32_t MeasurementTimingBudgetMicroSeconds);
VL53L1_Error VL53L1_GetMeasurementTimingBudgetMicroSeconds(
	VL53L1_DEV Dev, uint32_t *pMeasurementTimingBudgetMicroSeconds);
VL53L1_Error VL53L1_SetInterMeasurementPeriodMilliSeconds(
	VL53L1_DEV Dev, uint32_t InterMeasurementPeriodMilliSeconds);
VL53L1_Error VL53L1_GetInterMeasurementPeriodMilliSeconds(
	VL53L1_DEV Dev, uint32_t *pInterMeasurementPeriodMilliSeconds);
VL53L1_Error VL53L1_GetNumberOfLimitCheck(
	uint16_t *pNumberOfLimitCheck);
VL53L1_Error VL53L1_GetLimitCheckInfo(uint16_t LimitCheckId,
	char *pLimitCheckString);
VL53L1_Error VL53L1_GetLimitCheckStatus(VL53L1_DEV Dev,
	uint16_t LimitCheckId, uint8_t *pLimitCheckStatus);
VL53L1_Error VL53L1_SetLimitCheckEnable(VL53L1_DEV Dev,
	uint16_t LimitCheckId, uint8_t LimitCheckEnable);
VL53L1_Error VL53L1_GetLimitCheckEnable(VL53L1_DEV Dev,
	uint16_t LimitCheckId, uint8_t *pLimitCheckEnable);
VL53L1_Error VL53L1_SetLimitCheckValue(VL53L1_DEV Dev,
	uint16_t LimitCheckId, FixPoint1616_t LimitCheckValue);
VL53L1_Error VL53L1_GetLimitCheckValue(VL53L1_DEV Dev,
	uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckValue);
VL53L1_Error VL53L1_GetLimitCheckCurrent(VL53L1_DEV Dev,
	uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckCurrent);
VL53L1_Error VL53L1_SetUserROI(VL53L1_DEV Dev,
		VL53L1_UserRoi_t *pUserROi);
VL53L1_Error VL53L1_GetUserROI(VL53L1_DEV Dev,
		VL53L1_UserRoi_t *pUserROi);
VL53L1_Error VL53L1_GetNumberOfSequenceSteps(VL53L1_DEV Dev,
	uint8_t *pNumberOfSequenceSteps);
VL53L1_Error VL53L1_GetSequenceStepsInfo(
	VL53L1_SequenceStepId SequenceStepId, char *pSequenceStepsString);
VL53L1_Error VL53L1_SetSequenceStepEnable(VL53L1_DEV Dev,
	VL53L1_SequenceStepId SequenceStepId, uint8_t SequenceStepEnabled);
VL53L1_Error VL53L1_GetSequenceStepEnable(VL53L1_DEV Dev,
	VL53L1_SequenceStepId SequenceStepId, uint8_t *pSequenceStepEnabled);
VL53L1_Error VL53L1_StartMeasurement(VL53L1_DEV Dev);
VL53L1_Error VL53L1_StopMeasurement(VL53L1_DEV Dev);
VL53L1_Error VL53L1_ClearInterruptAndStartMeasurement(VL53L1_DEV Dev);
VL53L1_Error VL53L1_GetMeasurementDataReady(VL53L1_DEV Dev,
	uint8_t *pMeasurementDataReady);
VL53L1_Error VL53L1_WaitMeasurementDataReady(VL53L1_DEV Dev);
VL53L1_Error VL53L1_GetRangingMeasurementData(VL53L1_DEV Dev,
	VL53L1_RangingMeasurementData_t *pRangingMeasurementData);
VL53L1_Error VL53L1_SetTuningParameter(VL53L1_DEV Dev,
		uint16_t TuningParameterId, int32_t TuningParameterValue);
VL53L1_Error VL53L1_GetTuningParameter(VL53L1_DEV Dev,
		uint16_t TuningParameterId, int32_t *pTuningParameterValue);
VL53L1_Error VL53L1_PerformRefSpadManagement(VL53L1_DEV Dev);
VL53L1_Error VL53L1_SetXTalkCompensationEnable(VL53L1_DEV Dev,
uint8_t XTalkCompensationEnable);
VL53L1_Error VL53L1_GetXTalkCompensationEnable(VL53L1_DEV Dev,
	uint8_t *pXTalkCompensationEnable);
VL53L1_Error VL53L1_PerformSingleTargetXTalkCalibration(VL53L1_DEV Dev,
		int32_t CalDistanceMilliMeter);
VL53L1_Error VL53L1_SetOffsetCalibrationMode(VL53L1_DEV Dev,
		VL53L1_OffsetCalibrationModes OffsetCalibrationMode);
VL53L1_Error VL53L1_PerformOffsetCalibration(VL53L1_DEV Dev,
	int32_t CalDistanceMilliMeter);
VL53L1_Error VL53L1_PerformOffsetSimpleCalibration(VL53L1_DEV Dev,
		int32_t CalDistanceMilliMeter);
VL53L1_Error VL53L1_SetCalibrationData(VL53L1_DEV Dev,
		VL53L1_CalibrationData_t *pCalibrationData);
VL53L1_Error VL53L1_GetCalibrationData(VL53L1_DEV Dev,
		VL53L1_CalibrationData_t  *pCalibrationData);
VL53L1_Error VL53L1_GetOpticalCenter(VL53L1_DEV Dev,
		FixPoint1616_t *pOpticalCenterX,
		FixPoint1616_t *pOpticalCenterY);
VL53L1_Error VL53L1_SetThresholdConfig(VL53L1_DEV Dev,
		VL53L1_DetectionConfig_t *pConfig);
VL53L1_Error VL53L1_GetThresholdConfig(VL53L1_DEV Dev,
		VL53L1_DetectionConfig_t *pConfig);
#ifdef __cplusplus
}
#endif
#endif 