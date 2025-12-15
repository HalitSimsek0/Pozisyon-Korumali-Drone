#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#ifndef __CONFIGBLOCK_H__
#define __CONFIGBLOCK_H__
int configblockInit(void);
bool configblockTest(void);
// Removed: configblockGetRadioChannel, configblockGetRadioSpeed - Radio not used, WiFi used instead
uint64_t configblockGetRadioAddress(void); // Used only for device ID (address & 0xFF)
float configblockGetCalibPitch(void);
float configblockGetCalibRoll(void);
// New functions to save calibration values to NVS (replaces EEPROM)
esp_err_t configblockSetCalibPitch(float pitch);
esp_err_t configblockSetCalibRoll(float roll);
#endif 