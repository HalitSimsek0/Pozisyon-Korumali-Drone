#ifndef _VL53L1_DEF_H_
#define _VL53L1_DEF_H_
#include "vl53l1_ll_def.h"
#ifdef __cplusplus
extern "C" {
#endif
#define VL53L1_IMPLEMENTATION_VER_MAJOR       2
#define VL53L1_IMPLEMENTATION_VER_MINOR       3
#define VL53L1_IMPLEMENTATION_VER_SUB         3
#define VL53L1_IMPLEMENTATION_VER_REVISION  1885
typedef struct {
	uint32_t     revision; 
	uint8_t      major;    
	uint8_t      minor;    
	uint8_t      build;    
} VL53L1_Version_t;
#define VL53L1_DEVINFO_STRLEN 32
typedef struct {
	char Name[VL53L1_DEVINFO_STRLEN];
	char Type[VL53L1_DEVINFO_STRLEN];
	char ProductId[VL53L1_DEVINFO_STRLEN];
	uint8_t ProductType;
	uint8_t ProductRevisionMajor;
	uint8_t ProductRevisionMinor;
} VL53L1_DeviceInfo_t;
typedef uint8_t VL53L1_PresetModes;
#define VL53L1_PRESETMODE_AUTONOMOUS                ((VL53L1_PresetModes)  3)
#define VL53L1_PRESETMODE_LITE_RANGING              ((VL53L1_PresetModes)  4)
#define VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS       ((VL53L1_PresetModes)  8)
typedef uint8_t VL53L1_DistanceModes;
#define VL53L1_DISTANCEMODE_SHORT             ((VL53L1_DistanceModes)  1)
#define VL53L1_DISTANCEMODE_MEDIUM            ((VL53L1_DistanceModes)  2)
#define VL53L1_DISTANCEMODE_LONG              ((VL53L1_DistanceModes)  3)
typedef uint8_t VL53L1_XtalkCalibrationModes;
#define VL53L1_XTALKCALIBRATIONMODE_NO_TARGET \
	((VL53L1_OffsetCalibrationModes) 0)
#define VL53L1_XTALKCALIBRATIONMODE_SINGLE_TARGET \
	((VL53L1_OffsetCalibrationModes)  1)
#define VL53L1_XTALKCALIBRATIONMODE_FULL_ROI \
	((VL53L1_OffsetCalibrationModes)  2)
typedef uint8_t VL53L1_OffsetCalibrationModes;
#define VL53L1_OFFSETCALIBRATIONMODE_STANDARD \
	((VL53L1_OffsetCalibrationModes)  1)
#define VL53L1_OFFSETCALIBRATIONMODE_PRERANGE_ONLY  \
	((VL53L1_OffsetCalibrationModes)  2)
#define VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE           0
#define VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE     1
#define VL53L1_CHECKENABLE_NUMBER_OF_CHECKS            2
typedef uint8_t VL53L1_ThresholdMode;
#define VL53L1_THRESHOLD_CROSSED_LOW   \
	((VL53L1_ThresholdMode)  0)
#define VL53L1_THRESHOLD_CROSSED_HIGH   \
	((VL53L1_ThresholdMode)  1)
#define VL53L1_THRESHOLD_OUT_OF_WINDOW    \
	((VL53L1_ThresholdMode)  2)
#define VL53L1_THRESHOLD_IN_WINDOW        \
	((VL53L1_ThresholdMode)  3)
typedef struct {
	VL53L1_ThresholdMode CrossMode;
	uint16_t High; 
	uint16_t Low;  
} VL53L1_DistanceThreshold_t;
typedef struct {
	VL53L1_ThresholdMode CrossMode;
	FixPoint1616_t High; 
	FixPoint1616_t Low;  
} VL53L1_RateThreshold_t;
typedef uint8_t VL53L1_DetectionMode;
#define VL53L1_DETECTION_NORMAL_RUN   \
	((VL53L1_DetectionMode)  0)
#define VL53L1_DETECTION_DISTANCE_ONLY   \
	((VL53L1_DetectionMode)  1)
#define VL53L1_DETECTION_RATE_ONLY   \
	((VL53L1_DetectionMode)  2)
#define VL53L1_DETECTION_DISTANCE_AND_RATE   \
	((VL53L1_DetectionMode)  3)
#define VL53L1_DETECTION_DISTANCE_OR_RATE   \
	((VL53L1_DetectionMode)  4)
typedef struct {
	VL53L1_DetectionMode DetectionMode;	
	uint8_t IntrNoTarget; 
	VL53L1_DistanceThreshold_t Distance; 
	VL53L1_RateThreshold_t Rate;
} VL53L1_DetectionConfig_t;
typedef struct {
	VL53L1_PresetModes PresetMode;
	VL53L1_DistanceModes DistanceMode;
	VL53L1_DistanceModes InternalDistanceMode;
	VL53L1_DistanceModes NewDistanceMode;
	uint32_t MeasurementTimingBudgetMicroSeconds;
	uint8_t LimitChecksEnable[VL53L1_CHECKENABLE_NUMBER_OF_CHECKS];
	uint8_t LimitChecksStatus[VL53L1_CHECKENABLE_NUMBER_OF_CHECKS];
	FixPoint1616_t LimitChecksValue[VL53L1_CHECKENABLE_NUMBER_OF_CHECKS];
	FixPoint1616_t LimitChecksCurrent[VL53L1_CHECKENABLE_NUMBER_OF_CHECKS];
} VL53L1_DeviceParameters_t;
typedef uint8_t VL53L1_State;
#define VL53L1_STATE_POWERDOWN       ((VL53L1_State)  0)
#define VL53L1_STATE_WAIT_STATICINIT ((VL53L1_State)  1)
#define VL53L1_STATE_STANDBY         ((VL53L1_State)  2)
#define VL53L1_STATE_IDLE            ((VL53L1_State)  3)
#define VL53L1_STATE_RUNNING         ((VL53L1_State)  4)
#define VL53L1_STATE_RESET           ((VL53L1_State)  5)
#define VL53L1_STATE_UNKNOWN         ((VL53L1_State)  98)
#define VL53L1_STATE_ERROR           ((VL53L1_State)  99)
typedef struct {
	uint32_t TimeStamp;
	uint8_t StreamCount;
	uint8_t RangeQualityLevel;
	FixPoint1616_t SignalRateRtnMegaCps;
	FixPoint1616_t AmbientRateRtnMegaCps;
	uint16_t EffectiveSpadRtnCount;
	FixPoint1616_t SigmaMilliMeter;
	int16_t RangeMilliMeter;
	uint8_t RangeFractionalPart;
	uint8_t RangeStatus;
} VL53L1_RangingMeasurementData_t;
typedef struct {
	uint8_t   TopLeftX;   
	uint8_t   TopLeftY;   
	uint8_t   BotRightX;  
	uint8_t   BotRightY;  
} VL53L1_UserRoi_t;
typedef struct {
	uint8_t   global_config__spad_enables_ref_0;
	uint8_t   global_config__spad_enables_ref_1;
	uint8_t   global_config__spad_enables_ref_2;
	uint8_t   global_config__spad_enables_ref_3;
	uint8_t   global_config__spad_enables_ref_4;
	uint8_t   global_config__spad_enables_ref_5;
	uint8_t   global_config__ref_en_start_select;
	uint8_t   ref_spad_man__num_requested_ref_spads;
	uint8_t   ref_spad_man__ref_location;
	uint32_t  algo__crosstalk_compensation_plane_offset_kcps;
	int16_t   algo__crosstalk_compensation_x_plane_gradient_kcps;
	int16_t   algo__crosstalk_compensation_y_plane_gradient_kcps;
	uint16_t  ref_spad_char__total_rate_target_mcps;
	int16_t   algo__part_to_part_range_offset_mm;
	int16_t   mm_config__inner_offset_mm;
	int16_t   mm_config__outer_offset_mm;
} VL53L1_CustomerNvmManaged_t;
typedef struct {
	uint32_t                             struct_version;
	VL53L1_CustomerNvmManaged_t          customer;
	VL53L1_additional_offset_cal_data_t  add_off_cal_data;
	VL53L1_optical_centre_t              optical_centre;
	VL53L1_gain_calibration_data_t       gain_cal;
	VL53L1_cal_peak_rate_map_t           cal_peak_rate_map;
} VL53L1_CalibrationData_t;
#define VL53L1_ADDITIONAL_CALIBRATION_DATA_STRUCT_VERSION  0x10
#define VL53L1_CALIBRATION_DATA_STRUCT_VERSION \
		(VL53L1_LL_CALIBRATION_DATA_STRUCT_VERSION + \
		VL53L1_ADDITIONAL_CALIBRATION_DATA_STRUCT_VERSION)
typedef VL53L1_additional_data_t VL53L1_AdditionalData_t;
typedef uint8_t VL53L1_SequenceStepId;
#define	 VL53L1_SEQUENCESTEP_VHV		 ((VL53L1_SequenceStepId) 0)
#define	 VL53L1_SEQUENCESTEP_PHASECAL		 ((VL53L1_SequenceStepId) 1)
#define	 VL53L1_SEQUENCESTEP_REFPHASE		 ((VL53L1_SequenceStepId) 2)
#define	 VL53L1_SEQUENCESTEP_DSS1		 ((VL53L1_SequenceStepId) 3)
#define	 VL53L1_SEQUENCESTEP_DSS2		 ((VL53L1_SequenceStepId) 4)
#define	 VL53L1_SEQUENCESTEP_MM1		 ((VL53L1_SequenceStepId) 5)
#define	 VL53L1_SEQUENCESTEP_MM2		 ((VL53L1_SequenceStepId) 6)
#define	 VL53L1_SEQUENCESTEP_RANGE		 ((VL53L1_SequenceStepId) 7)
#define	 VL53L1_SEQUENCESTEP_NUMBER_OF_ITEMS			 8
#define	 VL53L1_RANGESTATUS_RANGE_VALID				0
#define	 VL53L1_RANGESTATUS_SIGMA_FAIL				1
#define	 VL53L1_RANGESTATUS_SIGNAL_FAIL				2
#define	 VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED	3
#define	 VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL			4
#define	 VL53L1_RANGESTATUS_HARDWARE_FAIL			5
#define	 VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL	6
#define	VL53L1_RANGESTATUS_WRAP_TARGET_FAIL			7
#define	VL53L1_RANGESTATUS_PROCESSING_FAIL			8
#define	VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL			9
#define	VL53L1_RANGESTATUS_SYNCRONISATION_INT			10
#define	VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE		11
#define	VL53L1_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL	12
#define	VL53L1_RANGESTATUS_MIN_RANGE_FAIL			13
#define	VL53L1_RANGESTATUS_RANGE_INVALID			14
#define	 VL53L1_RANGESTATUS_NONE				255
typedef struct {
	VL53L1_LLDriverData_t   LLData;
	VL53L1_LLDriverResults_t llresults;
	VL53L1_State      PalState; 
	VL53L1_DeviceParameters_t CurrentParameters;
} VL53L1_DevData_t;
#define VL53L1_SETPARAMETERFIELD(Dev, field, value) \
	(VL53L1DevDataSet(Dev, CurrentParameters.field, value))
#define VL53L1_GETPARAMETERFIELD(Dev, field, variable) \
	(variable = VL53L1DevDataGet(Dev, CurrentParameters).field)
#define VL53L1_SETARRAYPARAMETERFIELD(Dev, field, index, value) \
	(VL53L1DevDataSet(Dev, CurrentParameters.field[index], value))
#define VL53L1_GETARRAYPARAMETERFIELD(Dev, field, index, variable) \
	(variable = VL53L1DevDataGet(Dev, CurrentParameters).field[index])
#define VL53L1_SETDEVICESPECIFICPARAMETER(Dev, field, value) \
	(VL53L1DevDataSet(Dev, DeviceSpecificParameters.field, value))
#define VL53L1_GETDEVICESPECIFICPARAMETER(Dev, field) \
	(VL53L1DevDataGet(Dev, DeviceSpecificParameters).field)
#define VL53L1_FIXPOINT1616TOFIXPOINT44(Value) \
	(uint16_t)((Value>>12)&0xFFFF)
#define VL53L1_FIXPOINT44TOFIXPOINT1616(Value) \
	(FixPoint1616_t)((uint32_t)Value<<12)
#define VL53L1_FIXPOINT1616TOFIXPOINT72(Value) \
	(uint16_t)((Value>>14)&0xFFFF)
#define VL53L1_FIXPOINT72TOFIXPOINT1616(Value) \
	(FixPoint1616_t)((uint32_t)Value<<14)
#define VL53L1_FIXPOINT1616TOFIXPOINT97(Value) \
	(uint16_t)((Value>>9)&0xFFFF)
#define VL53L1_FIXPOINT97TOFIXPOINT1616(Value) \
	(FixPoint1616_t)((uint32_t)Value<<9)
#define VL53L1_FIXPOINT1616TOFIXPOINT88(Value) \
	(uint16_t)((Value>>8)&0xFFFF)
#define VL53L1_FIXPOINT88TOFIXPOINT1616(Value) \
	(FixPoint1616_t)((uint32_t)Value<<8)
#define VL53L1_FIXPOINT1616TOFIXPOINT412(Value) \
	(uint16_t)((Value>>4)&0xFFFF)
#define VL53L1_FIXPOINT412TOFIXPOINT1616(Value) \
	(FixPoint1616_t)((uint32_t)Value<<4)
#define VL53L1_FIXPOINT1616TOFIXPOINT313(Value) \
	(uint16_t)((Value>>3)&0xFFFF)
#define VL53L1_FIXPOINT313TOFIXPOINT1616(Value) \
	(FixPoint1616_t)((uint32_t)Value<<3)
#define VL53L1_FIXPOINT1616TOFIXPOINT08(Value) \
	(uint8_t)((Value>>8)&0x00FF)
#define VL53L1_FIXPOINT08TOFIXPOINT1616(Value) \
	(FixPoint1616_t)((uint32_t)Value<<8)
#define VL53L1_FIXPOINT1616TOFIXPOINT53(Value) \
	(uint8_t)((Value>>13)&0x00FF)
#define VL53L1_FIXPOINT53TOFIXPOINT1616(Value) \
	(FixPoint1616_t)((uint32_t)Value<<13)
#define VL53L1_FIXPOINT1616TOFIXPOINT102(Value) \
	(uint16_t)((Value>>14)&0x0FFF)
#define VL53L1_FIXPOINT102TOFIXPOINT1616(Value) \
	(FixPoint1616_t)((uint32_t)Value<<14)
#define VL53L1_FIXPOINT1616TOFIXPOINT142(Value) \
	(uint16_t)((Value>>14)&0xFFFF)
#define VL53L1_FIXPOINT142TOFIXPOINT1616(Value) \
	(FixPoint1616_t)((uint32_t)Value<<14)
#define VL53L1_FIXPOINT1616TOFIXPOINT160(Value) \
	(uint16_t)((Value>>16)&0xFFFF)
#define VL53L1_FIXPOINT160TOFIXPOINT1616(Value) \
	(FixPoint1616_t)((uint32_t)Value<<16)
#define VL53L1_MAKEUINT16(lsb, msb) (uint16_t)((((uint16_t)msb)<<8) + \
		(uint16_t)lsb)
#ifndef SUPPRESS_UNUSED_WARNING
#define SUPPRESS_UNUSED_WARNING(x) ((void) (x))
#endif
#define CHECK_ERROR_GO_ENDFUNC do {\
		if (Status != VL53L1_ERROR_NONE) \
			goto ENDFUNC; \
	} while (0)
#ifdef __cplusplus
}
#endif
#endif 
