#define DEBUG_MODULE "CFGBLK"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "config.h"
#include "debug_cf.h"
#include "configblock.h"
#include "nvs_flash.h"
#include "nvs.h"

#define NVS_NAMESPACE "config"
#define NVS_KEY_DEVICE_ID "dev_id"
#define NVS_KEY_CALIB_PITCH "cal_pitch"
#define NVS_KEY_CALIB_ROLL "cal_roll"

// Default device ID address (used only for device ID, not for radio communication)
#define DEFAULT_DEVICE_ID_ADDRESS 0xE7E7E7E7E7ULL

static bool isInit = false;
static uint64_t deviceId = DEFAULT_DEVICE_ID_ADDRESS;
static float calibPitch = 0.0f;
static float calibRoll = 0.0f;

int configblockInit(void)
{
  if (isInit) {
    return 0;
  }

  // Open NVS namespace
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
  
  if (err == ESP_OK) {
    // Read device ID
    size_t required_size = sizeof(uint64_t);
    err = nvs_get_blob(nvs_handle, NVS_KEY_DEVICE_ID, &deviceId, &required_size);
    if (err != ESP_OK || required_size != sizeof(uint64_t)) {
      deviceId = DEFAULT_DEVICE_ID_ADDRESS;
      DEBUG_PRINTD("Device ID not found in NVS, using default: 0x%llX\n", 
                   (unsigned long long)deviceId);
    } else {
      DEBUG_PRINTD("Device ID loaded from NVS: 0x%llX\n", (unsigned long long)deviceId);
    }
    
    // Read calibration pitch
    required_size = sizeof(float);
    err = nvs_get_blob(nvs_handle, NVS_KEY_CALIB_PITCH, &calibPitch, &required_size);
    if (err != ESP_OK || required_size != sizeof(float)) {
      calibPitch = 0.0f;
      DEBUG_PRINTD("Calibration pitch not found in NVS, using default: 0.0\n");
    } else {
      DEBUG_PRINTD("Calibration pitch loaded from NVS: %.2f degrees\n", (double)calibPitch);
    }
    
    // Read calibration roll
    required_size = sizeof(float);
    err = nvs_get_blob(nvs_handle, NVS_KEY_CALIB_ROLL, &calibRoll, &required_size);
    if (err != ESP_OK || required_size != sizeof(float)) {
      calibRoll = 0.0f;
      DEBUG_PRINTD("Calibration roll not found in NVS, using default: 0.0\n");
    } else {
      DEBUG_PRINTD("Calibration roll loaded from NVS: %.2f degrees\n", (double)calibRoll);
    }
    
    nvs_close(nvs_handle);
  } else {
    DEBUG_PRINTD("NVS namespace '%s' not found or not initialized, using defaults\n", NVS_NAMESPACE);
    DEBUG_PRINTD("Note: This is normal on first boot. Calibration values will be saved after first use.\n");
    deviceId = DEFAULT_DEVICE_ID_ADDRESS;
    calibPitch = 0.0f;
    calibRoll = 0.0f;
  }
  
  isInit = true;
  return 0;
}

bool configblockTest(void)
{
  return isInit;
}

uint64_t configblockGetRadioAddress(void)
{
  // This function is used only for device ID (address & 0xFF), not for radio communication
  return deviceId;
}

float configblockGetCalibPitch(void)
{
  return calibPitch;
}

float configblockGetCalibRoll(void)
{
  return calibRoll;
}

// New functions to save calibration values to NVS
esp_err_t configblockSetCalibPitch(float pitch)
{
  calibPitch = pitch;
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if (err == ESP_OK) {
    err = nvs_set_blob(nvs_handle, NVS_KEY_CALIB_PITCH, &calibPitch, sizeof(float));
    if (err == ESP_OK) {
      err = nvs_commit(nvs_handle);
      if (err == ESP_OK) {
        DEBUG_PRINTD("Calibration pitch saved to NVS: %.2f degrees\n", (double)calibPitch);
      } else {
        DEBUG_PRINTE("Failed to commit calibration pitch to NVS\n");
      }
    } else {
      DEBUG_PRINTE("Failed to set calibration pitch in NVS\n");
    }
    nvs_close(nvs_handle);
  } else {
    DEBUG_PRINTE("Failed to open NVS for calibration pitch save\n");
  }
  return err;
}

esp_err_t configblockSetCalibRoll(float roll)
{
  calibRoll = roll;
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if (err == ESP_OK) {
    err = nvs_set_blob(nvs_handle, NVS_KEY_CALIB_ROLL, &calibRoll, sizeof(float));
    if (err == ESP_OK) {
      err = nvs_commit(nvs_handle);
      if (err == ESP_OK) {
        DEBUG_PRINTD("Calibration roll saved to NVS: %.2f degrees\n", (double)calibRoll);
      } else {
        DEBUG_PRINTE("Failed to commit calibration roll to NVS\n");
      }
    } else {
      DEBUG_PRINTE("Failed to set calibration roll in NVS\n");
    }
    nvs_close(nvs_handle);
  } else {
    DEBUG_PRINTE("Failed to open NVS for calibration roll save\n");
  }
  return err;
}

