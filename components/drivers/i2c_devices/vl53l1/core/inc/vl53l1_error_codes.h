#ifndef _VL53L1_ERROR_CODES_H_
#define _VL53L1_ERROR_CODES_H_
#include "vl53l1_types.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef int8_t VL53L1_Error;
#define VL53L1_ERROR_NONE                              ((VL53L1_Error)  0)
#define VL53L1_ERROR_CALIBRATION_WARNING               ((VL53L1_Error) - 1)
#define VL53L1_ERROR_MIN_CLIPPED                       ((VL53L1_Error) - 2)
#define VL53L1_ERROR_UNDEFINED                         ((VL53L1_Error) - 3)
#define VL53L1_ERROR_INVALID_PARAMS                    ((VL53L1_Error) - 4)
#define VL53L1_ERROR_NOT_SUPPORTED                     ((VL53L1_Error) - 5)
#define VL53L1_ERROR_RANGE_ERROR                       ((VL53L1_Error) - 6)
#define VL53L1_ERROR_TIME_OUT                          ((VL53L1_Error) - 7)
#define VL53L1_ERROR_MODE_NOT_SUPPORTED                ((VL53L1_Error) - 8)
#define VL53L1_ERROR_BUFFER_TOO_SMALL                  ((VL53L1_Error) - 9)
#define VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL            ((VL53L1_Error) - 10)
#define VL53L1_ERROR_GPIO_NOT_EXISTING                 ((VL53L1_Error) - 11)
#define VL53L1_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED  ((VL53L1_Error) - 12)
#define VL53L1_ERROR_CONTROL_INTERFACE                 ((VL53L1_Error) - 13)
#define VL53L1_ERROR_INVALID_COMMAND                   ((VL53L1_Error) - 14)
#define VL53L1_ERROR_DIVISION_BY_ZERO                  ((VL53L1_Error) - 15)
#define VL53L1_ERROR_REF_SPAD_INIT                     ((VL53L1_Error) - 16)
#define VL53L1_ERROR_GPH_SYNC_CHECK_FAIL               ((VL53L1_Error) - 17)
#define VL53L1_ERROR_STREAM_COUNT_CHECK_FAIL           ((VL53L1_Error) - 18)
#define VL53L1_ERROR_GPH_ID_CHECK_FAIL                 ((VL53L1_Error) - 19)
#define VL53L1_ERROR_ZONE_STREAM_COUNT_CHECK_FAIL      ((VL53L1_Error) - 20)
#define VL53L1_ERROR_ZONE_GPH_ID_CHECK_FAIL            ((VL53L1_Error) - 21)
#define VL53L1_ERROR_XTALK_EXTRACTION_NO_SAMPLE_FAIL   ((VL53L1_Error) - 22)
#define VL53L1_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL ((VL53L1_Error) - 23)
#define VL53L1_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL           ((VL53L1_Error) - 24)
#define VL53L1_ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL    ((VL53L1_Error) - 25)
#define VL53L1_ERROR_ZONE_CAL_NO_SAMPLE_FAIL             ((VL53L1_Error) - 26)
#define VL53L1_ERROR_TUNING_PARM_KEY_MISMATCH             ((VL53L1_Error) - 27)
#define VL53L1_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS   ((VL53L1_Error) - 28)
#define VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH      ((VL53L1_Error) - 29)
#define VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW       ((VL53L1_Error) - 30)
#define VL53L1_WARNING_OFFSET_CAL_MISSING_SAMPLES       ((VL53L1_Error) - 31)
#define VL53L1_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH        ((VL53L1_Error) - 32)
#define VL53L1_WARNING_OFFSET_CAL_RATE_TOO_HIGH         ((VL53L1_Error) - 33)
#define VL53L1_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW    ((VL53L1_Error) - 34)
#define VL53L1_WARNING_ZONE_CAL_MISSING_SAMPLES       ((VL53L1_Error) - 35)
#define VL53L1_WARNING_ZONE_CAL_SIGMA_TOO_HIGH        ((VL53L1_Error) - 36)
#define VL53L1_WARNING_ZONE_CAL_RATE_TOO_HIGH         ((VL53L1_Error) - 37)
#define VL53L1_WARNING_XTALK_MISSING_SAMPLES             ((VL53L1_Error) - 38)
#define VL53L1_WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT     ((VL53L1_Error) - 39)
#define VL53L1_WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT    ((VL53L1_Error) - 40)
#define VL53L1_ERROR_NOT_IMPLEMENTED                   ((VL53L1_Error) - 41)
#define VL53L1_ERROR_PLATFORM_SPECIFIC_START           ((VL53L1_Error) - 60)
#ifdef __cplusplus
}
#endif
#endif 