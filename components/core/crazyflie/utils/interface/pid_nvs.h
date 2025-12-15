#ifndef PID_NVS_H_
#define PID_NVS_H_
#include <stdbool.h>
#include "esp_err.h"
#include "pid.h"

// PID configuration structure for NVS storage
typedef struct {
  // Attitude PIDs
  float roll_kp, roll_ki, roll_kd, roll_kf;
  float pitch_kp, pitch_ki, pitch_kd, pitch_kf;
  float yaw_kp, yaw_ki, yaw_kd, yaw_kf;
  
  // Rate PIDs
  float roll_rate_kp, roll_rate_ki, roll_rate_kd;
  float pitch_rate_kp, pitch_rate_ki, pitch_rate_kd;
  float yaw_rate_kp, yaw_rate_ki, yaw_rate_kd;
  
  // Integration limits
  float roll_iLimit, pitch_iLimit, yaw_iLimit;
  float roll_rate_iLimit, pitch_rate_iLimit, yaw_rate_iLimit;
} pid_config_t;

// Initialize PID NVS system
esp_err_t pidNvsInit(void);

// Load PID configuration from NVS
// If not found in NVS, uses default values from pid.h
esp_err_t pidNvsLoad(pid_config_t* out, const pid_config_t* defaults);

// Save PID configuration to NVS
// Only saves when user explicitly requests (to preserve flash lifetime)
esp_err_t pidNvsSave(const pid_config_t* config);

// Apply loaded PID configuration to PID controllers
// This should be called after attitudeControllerInit()
esp_err_t pidNvsApply(const pid_config_t* config);

// Get default PID configuration (from pid.h defines)
void pidNvsGetDefaults(pid_config_t* defaults);

#endif

