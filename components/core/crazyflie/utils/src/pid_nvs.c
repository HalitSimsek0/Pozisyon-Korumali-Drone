#define DEBUG_MODULE "PID_NVS"
#include "pid_nvs.h"
#include "pid.h"
#include "attitude_controller.h"
#include "debug_cf.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

#define NVS_NAMESPACE "pid"
#define NVS_KEY_CONFIG "config"

// External PID objects from attitude_pid_controller.c
extern PidObject pidRollRate;
extern PidObject pidPitchRate;
extern PidObject pidYawRate;
extern PidObject pidRoll;
extern PidObject pidPitch;
extern PidObject pidYaw;

static bool isInit = false;

esp_err_t pidNvsInit(void)
{
  if (isInit) {
    return ESP_OK;
  }
  isInit = true;
  return ESP_OK;
}

void pidNvsGetDefaults(pid_config_t* defaults)
{
  if (!defaults) return;
  
  // Attitude PIDs
  defaults->roll_kp = PID_ROLL_KP;
  defaults->roll_ki = PID_ROLL_KI;
  defaults->roll_kd = PID_ROLL_KD;
  defaults->roll_kf = 0.0f;
  
  defaults->pitch_kp = PID_PITCH_KP;
  defaults->pitch_ki = PID_PITCH_KI;
  defaults->pitch_kd = PID_PITCH_KD;
  defaults->pitch_kf = 0.0f;
  
  defaults->yaw_kp = PID_YAW_KP;
  defaults->yaw_ki = PID_YAW_KI;
  defaults->yaw_kd = PID_YAW_KD;
  defaults->yaw_kf = 0.0f;
  
  // Rate PIDs
  defaults->roll_rate_kp = PID_ROLL_RATE_KP;
  defaults->roll_rate_ki = PID_ROLL_RATE_KI;
  defaults->roll_rate_kd = PID_ROLL_RATE_KD;
  
  defaults->pitch_rate_kp = PID_PITCH_RATE_KP;
  defaults->pitch_rate_ki = PID_PITCH_RATE_KI;
  defaults->pitch_rate_kd = PID_PITCH_RATE_KD;
  
  defaults->yaw_rate_kp = PID_YAW_RATE_KP;
  defaults->yaw_rate_ki = PID_YAW_RATE_KI;
  defaults->yaw_rate_kd = PID_YAW_RATE_KD;
  
  // Integration limits
  defaults->roll_iLimit = PID_ROLL_INTEGRATION_LIMIT;
  defaults->pitch_iLimit = PID_PITCH_INTEGRATION_LIMIT;
  defaults->yaw_iLimit = PID_YAW_INTEGRATION_LIMIT;
  
  defaults->roll_rate_iLimit = PID_ROLL_RATE_INTEGRATION_LIMIT;
  defaults->pitch_rate_iLimit = PID_PITCH_RATE_INTEGRATION_LIMIT;
  defaults->yaw_rate_iLimit = PID_YAW_RATE_INTEGRATION_LIMIT;
}

esp_err_t pidNvsLoad(pid_config_t* out, const pid_config_t* defaults)
{
  if (!out || !defaults) {
    return ESP_ERR_INVALID_ARG;
  }
  
  // Start with defaults
  memcpy(out, defaults, sizeof(pid_config_t));
  
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
  
  if (err == ESP_OK) {
    size_t required_size = sizeof(pid_config_t);
    err = nvs_get_blob(nvs_handle, NVS_KEY_CONFIG, out, &required_size);
    
    if (err == ESP_OK && required_size == sizeof(pid_config_t)) {
      DEBUG_PRINTD("PID config loaded from NVS\n");
    } else {
      DEBUG_PRINTD("PID config not found in NVS, using defaults\n");
      memcpy(out, defaults, sizeof(pid_config_t));
      err = ESP_ERR_NOT_FOUND;
    }
    
    nvs_close(nvs_handle);
  } else {
    DEBUG_PRINTD("NVS open failed, using defaults\n");
    memcpy(out, defaults, sizeof(pid_config_t));
  }
  
  return err;
}

esp_err_t pidNvsSave(const pid_config_t* config)
{
  if (!config) {
    return ESP_ERR_INVALID_ARG;
  }
  
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
  
  if (err == ESP_OK) {
    err = nvs_set_blob(nvs_handle, NVS_KEY_CONFIG, config, sizeof(pid_config_t));
    if (err == ESP_OK) {
      err = nvs_commit(nvs_handle);
      if (err == ESP_OK) {
        DEBUG_PRINTD("PID config saved to NVS\n");
      } else {
        DEBUG_PRINTE("Failed to commit PID config to NVS\n");
      }
    } else {
      DEBUG_PRINTE("Failed to set PID config blob in NVS\n");
    }
    nvs_close(nvs_handle);
  } else {
    DEBUG_PRINTE("Failed to open NVS for PID config save\n");
  }
  
  return err;
}

esp_err_t pidNvsApply(const pid_config_t* config)
{
  if (!config) {
    return ESP_ERR_INVALID_ARG;
  }
  
  // Apply attitude PIDs
  pidSetKp(&pidRoll, config->roll_kp);
  pidSetKi(&pidRoll, config->roll_ki);
  pidSetKd(&pidRoll, config->roll_kd);
  pidSetKf(&pidRoll, config->roll_kf);
  pidSetIntegralLimit(&pidRoll, config->roll_iLimit);
  
  pidSetKp(&pidPitch, config->pitch_kp);
  pidSetKi(&pidPitch, config->pitch_ki);
  pidSetKd(&pidPitch, config->pitch_kd);
  pidSetKf(&pidPitch, config->pitch_kf);
  pidSetIntegralLimit(&pidPitch, config->pitch_iLimit);
  
  pidSetKp(&pidYaw, config->yaw_kp);
  pidSetKi(&pidYaw, config->yaw_ki);
  pidSetKd(&pidYaw, config->yaw_kd);
  pidSetKf(&pidYaw, config->yaw_kf);
  pidSetIntegralLimit(&pidYaw, config->yaw_iLimit);
  
  // Apply rate PIDs
  pidSetKp(&pidRollRate, config->roll_rate_kp);
  pidSetKi(&pidRollRate, config->roll_rate_ki);
  pidSetKd(&pidRollRate, config->roll_rate_kd);
  pidSetIntegralLimit(&pidRollRate, config->roll_rate_iLimit);
  
  pidSetKp(&pidPitchRate, config->pitch_rate_kp);
  pidSetKi(&pidPitchRate, config->pitch_rate_ki);
  pidSetKd(&pidPitchRate, config->pitch_rate_kd);
  pidSetIntegralLimit(&pidPitchRate, config->pitch_rate_iLimit);
  
  pidSetKp(&pidYawRate, config->yaw_rate_kp);
  pidSetKi(&pidYawRate, config->yaw_rate_ki);
  pidSetKd(&pidYawRate, config->yaw_rate_kd);
  pidSetIntegralLimit(&pidYawRate, config->yaw_rate_iLimit);
  
  DEBUG_PRINTD("PID configuration applied\n");
  return ESP_OK;
}

