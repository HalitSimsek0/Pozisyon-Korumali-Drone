#ifndef _VL53L1_SILICON_CORE_H_
#define _VL53L1_SILICON_CORE_H_
#include "vl53l1_platform.h"
#ifdef __cplusplus
extern "C" {
#endif
VL53L1_Error VL53L1_is_firmware_ready_silicon(
	VL53L1_DEV      Dev,
	uint8_t        *pready);
#ifdef __cplusplus
}
#endif
#endif 