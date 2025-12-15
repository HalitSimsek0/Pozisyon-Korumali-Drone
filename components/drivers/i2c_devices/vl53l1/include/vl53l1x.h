#ifndef _VL53L1X_H_
#define _VL53L1X_H_
#include "vl53l1_ll_def.h"
#include "vl53l1_platform_user_data.h"
#include "vl53l1_api.h"
#include "i2cdev.h"
#ifdef __cplusplus
extern "C"
{
#endif
#define VL53L1X_DEFAULT_ADDRESS 0b0101001
#define VL53L1X_ID			0xEACC
#define USE_I2C_2V8
bool vl53l1xInit(VL53L1_Dev_t *pdev, I2C_Dev *I2cHandle);
bool vl53l1xTestConnection(VL53L1_Dev_t* pdev);
VL53L1_Error vl53l1xSetI2CAddress(VL53L1_Dev_t* pdev, uint8_t address);
VL53L1_Error VL53L1_WriteMulti(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);
VL53L1_Error VL53L1_ReadMulti(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);
VL53L1_Error VL53L1_WrByte(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint8_t       data);
VL53L1_Error VL53L1_WrWord(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint16_t      data);
VL53L1_Error VL53L1_WrDWord(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint32_t      data);
VL53L1_Error VL53L1_RdByte(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata);
VL53L1_Error VL53L1_RdWord(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint16_t     *pdata);
VL53L1_Error VL53L1_RdDWord(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint32_t     *pdata);
VL53L1_Error VL53L1_WaitUs(
		VL53L1_Dev_t *pdev,
		int32_t       wait_us);
VL53L1_Error VL53L1_WaitMs(
		VL53L1_Dev_t *pdev,
		int32_t       wait_ms);
VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptime_ms);
VL53L1_Error VL53L1_WaitValueMaskEx(
		VL53L1_Dev_t *pdev,
		uint32_t      timeout_ms,
		uint16_t      index,
		uint8_t       value,
		uint8_t       mask,
		uint32_t      poll_delay_ms);
#ifdef __cplusplus
}
#endif
#endif