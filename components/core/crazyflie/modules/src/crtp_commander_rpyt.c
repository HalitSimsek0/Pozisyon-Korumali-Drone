#include <math.h>
#include <stdbool.h>
#include "crtp_commander.h"
#include "commander.h"
#include "estimator.h"
#include "crtp.h"
#include "param.h"
#include "FreeRTOS.h"
#include "num.h"
#include "stm32_legacy.h"
#include "stabilizer_types.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "flowdeck_v1v2.h"  // EKLE: Flow sensörü kontrolü için
#include "zranger2.h"  // EKLE: ToF sensörü kontrolü için
#define DEBUG_MODULE "MODE"
#include "debug_cf.h"
#define MIN_THRUST  1000
#define MAX_THRUST  60000
struct CommanderCrtpLegacyValues
{
  float roll;       
  float pitch;      
  float yaw;        
  uint16_t thrust;
} __attribute__((packed));
typedef enum
{
  RATE    = 0,
  ANGLE   = 1,
} RPYType;
typedef enum
{
  CAREFREE  = 0, 
  PLUSMODE  = 1, 
  XMODE     = 2, 
} YawModeType;
static RPYType stabilizationModeRoll  = ANGLE; 
static RPYType stabilizationModePitch = ANGLE; 
static RPYType stabilizationModeYaw   = RATE;  
static YawModeType yawMode = DEFAULT_YAW_MODE; 
static bool carefreeResetFront;             
static bool thrustLocked = true;
static bool altHoldMode = false;
static bool posHoldMode = false;
static bool posSetMode = false;
void setCommandermode(FlightMode mode){
#ifdef CONFIG_ENABLE_COMMAND_MODE_SET
  switch (mode) {
  case ALTHOLD_MODE:
    altHoldMode = true;
    posHoldMode = false;
    posSetMode = false;
    registerRequiredEstimator(complementaryEstimator);
    break;
  case POSHOLD_MODE:
    altHoldMode = true;
    posHoldMode = true;
    posSetMode = false;
    registerRequiredEstimator(kalmanEstimator); 
    break;
  case POSSET_MODE:
    altHoldMode = false;
    posHoldMode = false;
    posSetMode = true;
    registerRequiredEstimator(kalmanEstimator); 
    break;
  default:
    altHoldMode = false;
    posHoldMode = false;
    posSetMode = false;
    registerRequiredEstimator(complementaryEstimator);   
    break;
  }
  DEBUG_PRINTI("FlightMode = %d",mode);
#else
  DEBUG_PRINTI("set FlightMode disable");
#endif
}
static void rotateYaw(setpoint_t *setpoint, float yawRad)
{
  float cosy = cosf(yawRad);
  float siny = sinf(yawRad);
  float originalRoll = setpoint->attitude.roll;
  float originalPitch = setpoint->attitude.pitch;
  setpoint->attitude.roll = originalRoll * cosy - originalPitch * siny;
  setpoint->attitude.pitch = originalPitch * cosy + originalRoll * siny;
}
static void yawModeUpdate(setpoint_t *setpoint)
{
  switch (yawMode)
  {
    case CAREFREE:
      ASSERT(false);
      break;
    case PLUSMODE:
      rotateYaw(setpoint, 45 * M_PI / 180);
      break;
    case XMODE: 
    default:
      break;
  }
}
void crtpCommanderRpytDecodeSetpoint(setpoint_t *setpoint, CRTPPacket *pk)
{
  struct CommanderCrtpLegacyValues *values = (struct CommanderCrtpLegacyValues*)pk->data;
  if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
    thrustLocked = true;
  }
  if (values->thrust == 0) {
    thrustLocked = false;
  }
  uint16_t rawThrust = values->thrust;
  if (thrustLocked || (rawThrust < MIN_THRUST)) {
    setpoint->thrust = 0;
  } else {
    setpoint->thrust = fminf(rawThrust, MAX_THRUST);
  }
  if (altHoldMode) {
    // EKLE: ToF sensörü kontrolü - ToF sensörü yoksa altitude hold'i kapat
    if (!zRanger2Test()) {
      // ToF sensörü yoksa altitude hold'i kapat ve stabilize mode'a geç
      DEBUG_PRINTW("Altitude hold: ToF sensor not available, disabling altitude hold\n");
      altHoldMode = false;
      if (posHoldMode) {
        // Position hold da aktifse, onu da kapat
        posHoldMode = false;
        setCommandermode(STABILIZE_MODE);
      } else {
        setCommandermode(STABILIZE_MODE);
      }
      setpoint->mode.z = modeDisable;
    } else {
      setpoint->thrust = 0;
      setpoint->mode.z = modeVelocity;
      setpoint->velocity.z = ((float) rawThrust - 32767.f) / 32767.f;
    }
  } else {
    setpoint->mode.z = modeDisable;
  }
  if (posHoldMode) {
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->velocity_body = true;
    setpoint->velocity.x = -values->pitch/30.0f;
    setpoint->velocity.y = -values->roll/30.0f;
    setpoint->attitude.roll  = 0;
    setpoint->attitude.pitch = 0;
  } else if (posSetMode && values->thrust != 0) {
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeAbs;
    setpoint->position.x = -values->pitch;
    setpoint->position.y = values->roll;
    setpoint->position.z = values->thrust/1000.0f;
    setpoint->attitude.roll  = 0;
    setpoint->attitude.pitch = 0;
    setpoint->attitude.yaw = values->yaw;
    setpoint->thrust = 0;
  } else {
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    if (stabilizationModeRoll == RATE) {
      setpoint->mode.roll = modeVelocity;
      setpoint->attitudeRate.roll = values->roll;
      setpoint->attitude.roll = 0;
    } else {
      setpoint->mode.roll = modeAbs;
      setpoint->attitudeRate.roll = 0;
      setpoint->attitude.roll = values->roll;
    }
    if (stabilizationModePitch == RATE) {
      setpoint->mode.pitch = modeVelocity;
      setpoint->attitudeRate.pitch = values->pitch;
      setpoint->attitude.pitch = 0;
    } else {
      setpoint->mode.pitch = modeAbs;
      setpoint->attitudeRate.pitch = 0;
      setpoint->attitude.pitch = values->pitch;
    }
    setpoint->velocity.x = 0;
    setpoint->velocity.y = 0;
  }
  if (!posSetMode) {
    if (stabilizationModeYaw == RATE) {
      setpoint->attitudeRate.yaw = -values->yaw;
      yawModeUpdate(setpoint);
      setpoint->mode.yaw = modeVelocity;
    } else {
      setpoint->mode.yaw = modeAbs;
      setpoint->attitudeRate.yaw = 0;
      setpoint->attitude.yaw = values->yaw;
    }
  }
}
/**
 * Initialize RPYT commander
 * Position hold mode is enabled by default for continuous position hold
 * This allows position hold to work continuously while allowing manual control via joystick
 * When joystick is moved: drone moves (velocity command)
 * When joystick is released: drone holds position (position hold active)
 */
void crtpCommanderRpytInit(void)
{
  // Initialize position hold mode to TRUE by default for continuous position hold
  // This allows position hold to work continuously while allowing manual control via joystick
  // When joystick is moved: drone moves (velocity command)
  // When joystick is released: drone holds position (position hold active)
  posHoldMode = true;
  altHoldMode = true;
  posSetMode = false;
  registerRequiredEstimator(kalmanEstimator);  // Kalman estimator required for position hold
}

/**
 * Get position hold mode status
 * @return true if position hold mode is active
 */
bool crtpCommanderRpytGetPosHoldMode(void)
{
  return posHoldMode;
}

// Position hold state variables
static point_t savedPosition = {0};  // Saved position to hold
static bool positionSaved = false;  // Whether position has been saved
static bool wasFlying = false;      // Previous flying state
static uint32_t lastMovementTime = 0;  // Last time movement command was received
static float lastVelocityX = 0.0f;
static float lastVelocityY = 0.0f;
static float lastVelocityZ = 0.0f;
static point_t lastSavedPosition = {0};  // Last saved position for drift detection
static uint32_t lastPositionSaveTime = 0;  // Last time position was saved
static float lastPositionX = 0.0f;
static float lastPositionY = 0.0f;
static float lastPositionZ = 0.0f;

// Position hold configuration macros
#define POSITION_HOLD_MIN_ALTITUDE 0.1f  // Minimum altitude to enable position hold (10cm)
#define POSITION_HOLD_MOVEMENT_THRESHOLD 0.05f  // Velocity threshold to detect movement (5cm/s)
#define POSITION_HOLD_STILL_TIME_MS 500  // Time to wait before saving position after movement stops (500ms)
#define POSITION_HOLD_SAVE_DELAY_MS 1000  // Delay after takeoff before saving position (1 second)
// EKLE: Threshold'lar artırıldı - Normal uçuşta (rüzgar, titreşim) yanlış tetiklenmeyi önlemek için
// Önceki değerler çok düşüktü (0.05m, 0.2m/s) ve sürekli external force algılanıyordu
#define POSITION_HOLD_DRIFT_THRESHOLD 0.15f  // Position drift threshold (artırıldı: 0.05m -> 0.15m)
#define POSITION_HOLD_DRIFT_VELOCITY_THRESHOLD 0.5f  // Velocity threshold (artırıldı: 0.2m/s -> 0.5m/s)
#define POSITION_HOLD_UPDATE_INTERVAL_MS 2000  // Update saved position every 2 seconds during stable flight
#define POSITION_HOLD_NVS_NAMESPACE "poshold"
#define POSITION_HOLD_NVS_KEY_POSITION "saved_pos"

/**
 * Save position to NVS for persistence
 */
static void savePositionToNVS(const point_t *pos) {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open(POSITION_HOLD_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if (err == ESP_OK) {
    err = nvs_set_blob(nvs_handle, POSITION_HOLD_NVS_KEY_POSITION, pos, sizeof(point_t));
    if (err == ESP_OK) {
      nvs_commit(nvs_handle);
      DEBUG_PRINTD("Position hold: Saved to NVS x=%.2f y=%.2f z=%.2f\n",
                   (double)pos->x, (double)pos->y, (double)pos->z);
    }
    nvs_close(nvs_handle);
  }
}

/**
 * Load position from NVS
 */
static bool loadPositionFromNVS(point_t *pos) {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open(POSITION_HOLD_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
  if (err == ESP_OK) {
    size_t required_size = sizeof(point_t);
    err = nvs_get_blob(nvs_handle, POSITION_HOLD_NVS_KEY_POSITION, pos, &required_size);
    nvs_close(nvs_handle);
    if (err == ESP_OK && required_size == sizeof(point_t)) {
      DEBUG_PRINTD("Position hold: Loaded from NVS x=%.2f y=%.2f z=%.2f\n",
                   (double)pos->x, (double)pos->y, (double)pos->z);
      return true;
    }
  }
  return false;
}

/**
 * Detect external force (wind, push, etc.) by comparing actual position/velocity with expected
 * Returns true if external force is detected
 */
static bool detectExternalForce(const state_t *state, const setpoint_t *setpoint, uint32_t currentTime) {
  // Check if there's a significant position drift without user command
  float posDriftX = state->position.x - savedPosition.x;
  float posDriftY = state->position.y - savedPosition.y;
  float posDriftZ = state->position.z - savedPosition.z;
  float posDriftMag = sqrtf(posDriftX*posDriftX + posDriftY*posDriftY + posDriftZ*posDriftZ);
  
  // Check if there's unexpected velocity (drift velocity)
  float velMag = sqrtf(state->velocity.x*state->velocity.x + 
                       state->velocity.y*state->velocity.y + 
                       state->velocity.z*state->velocity.z);
  
  // User command velocity (should be near zero if no command)
  float cmdVelMag = sqrtf(setpoint->velocity.x*setpoint->velocity.x + 
                         setpoint->velocity.y*setpoint->velocity.y + 
                         setpoint->velocity.z*setpoint->velocity.z);
  
  // External force detected if:
  // 1. Position drift is significant AND
  // 2. Actual velocity is significant BUT
  // 3. Command velocity is small (no user command)
  bool externalForce = (posDriftMag > POSITION_HOLD_DRIFT_THRESHOLD) &&
                       (velMag > POSITION_HOLD_DRIFT_VELOCITY_THRESHOLD) &&
                       (cmdVelMag < POSITION_HOLD_MOVEMENT_THRESHOLD);
  
  if (externalForce) {
    DEBUG_PRINTD("Position hold: External force detected! Drift=%.2fm, Vel=%.2fm/s, CmdVel=%.2fm/s\n",
                 (double)posDriftMag, (double)velMag, (double)cmdVelMag);
  }
  
  return externalForce;
}

/**
 * Update position hold setpoint based on current state
 * This function implements intelligent position holding:
 * - Saves position after takeoff or when movement stops
 * - Holds saved position when no movement command is received
 * - Allows movement when joystick commands are received
 * - Distinguishes between user commands and external disturbances
 * - Detects drift from external forces (wind, push) and corrects
 * - Continuously updates saved position during stable flight
 * - Saves position to NVS for persistence
 * 
 * @param setpoint Setpoint to update
 * @param state Current state
 * @param currentTime Current time in ms
 */
void crtpCommanderRpytUpdatePositionHold(setpoint_t *setpoint, const state_t *state, uint32_t currentTime)
{
  if (!posHoldMode) {
    positionSaved = false;
    wasFlying = false;
    return;
  }

  // EKLE: Flow sensörü kontrolü - Flow sensörü yoksa position hold'i kapat
  if (!flowdeck2Test()) {
    // Flow sensörü yoksa position hold'i kapat ve stabilize mode'a geç
    DEBUG_PRINTW("Position hold: Flow sensor not available, disabling position hold\n");
    posHoldMode = false;
    positionSaved = false;
    setCommandermode(STABILIZE_MODE);  // Position hold'u kapat, stabilize mode'a geç
    return;
  }

  // Check if drone is flying (has altitude above minimum threshold)
  bool isFlying = (state->position.z > POSITION_HOLD_MIN_ALTITUDE);
  
  // Detect if this is first takeoff
  if (isFlying && !wasFlying) {
    // Just took off - wait a bit before saving position to allow stabilization
    lastMovementTime = currentTime;
    positionSaved = false;
    // Try to load last saved position from NVS (optional - for recovery)
    point_t nvsPos;
    if (loadPositionFromNVS(&nvsPos)) {
      // Optionally use NVS position as initial reference (commented out for now)
      // savedPosition = nvsPos;
    }
    DEBUG_PRINTD("Position hold: Takeoff detected at z=%.2f\n", (double)state->position.z);
  }
  wasFlying = isFlying;

  if (!isFlying) {
    // On ground - reset position hold
    positionSaved = false;
    return;
  }

  // Check if user is commanding movement (velocity setpoint from joystick)
  float velX = setpoint->velocity.x;
  float velY = setpoint->velocity.y;
  float velZ = setpoint->velocity.z;
  
  // Calculate velocity magnitude to detect movement commands
  float velMag = sqrtf(velX*velX + velY*velY + velZ*velZ);
  bool isMoving = (velMag > POSITION_HOLD_MOVEMENT_THRESHOLD);
  
  if (isMoving) {
    // User is commanding movement via joystick - this is intentional movement
    // Update last movement time and reset saved position
    lastMovementTime = currentTime;
    lastVelocityX = velX;
    lastVelocityY = velY;
    lastVelocityZ = velZ;
    positionSaved = false;  // Reset saved position - will save new position when movement stops
    DEBUG_PRINTD("Position hold: User movement detected vx=%.2f vy=%.2f vz=%.2f\n", 
                 (double)velX, (double)velY, (double)velZ);
  } else {
    // No movement command detected - check if we should save position or hold it
    uint32_t timeSinceMovement = currentTime - lastMovementTime;
    
    // EKLE: Joystick bırakıldığında velocity setpoint'lerini sıfırla
    if (velMag < POSITION_HOLD_MOVEMENT_THRESHOLD) {
      setpoint->velocity.x = 0.0f;
      setpoint->velocity.y = 0.0f;
      // velocity.z sıfırlanmamalı (altitude hold için throttle kontrolü gerekebilir)
    }
    
    if (!positionSaved) {
      // Position not saved yet - check if we should save it
      // Wait for movement to stop before saving to avoid saving during active flight
      if (timeSinceMovement > M2T(POSITION_HOLD_STILL_TIME_MS)) {
        // Movement stopped - save current position as hold target
        savedPosition.x = state->position.x;
        savedPosition.y = state->position.y;
        savedPosition.z = state->position.z;
        lastSavedPosition = savedPosition;
        lastPositionSaveTime = currentTime;
        lastPositionX = state->position.x;
        lastPositionY = state->position.y;
        lastPositionZ = state->position.z;
        positionSaved = true;
        savePositionToNVS(&savedPosition);  // Save to NVS for persistence
        DEBUG_PRINTD("Position hold: Position saved x=%.2f y=%.2f z=%.2f\n",
                     (double)savedPosition.x, (double)savedPosition.y, (double)savedPosition.z);
      }
    } else {
      // Position is saved - check for external forces and update position if needed
      
      // Detect external force (wind, push, etc.)
      bool externalForceDetected = detectExternalForce(state, setpoint, currentTime);
      
      // Continuously update saved position during stable flight (every UPDATE_INTERVAL_MS)
      // This allows the drone to adapt to slow drift while still correcting for sudden disturbances
      uint32_t timeSinceLastSave = currentTime - lastPositionSaveTime;
      bool shouldUpdatePosition = (timeSinceLastSave > M2T(POSITION_HOLD_UPDATE_INTERVAL_MS)) &&
                                  (timeSinceMovement > M2T(POSITION_HOLD_STILL_TIME_MS * 2)) &&
                                  !externalForceDetected;
      
      if (shouldUpdatePosition) {
        // Update saved position to current position (slow adaptation)
        // Only if no external force is detected and drone has been stable
        float posChange = sqrtf((state->position.x - lastPositionX)*(state->position.x - lastPositionX) +
                               (state->position.y - lastPositionY)*(state->position.y - lastPositionY) +
                               (state->position.z - lastPositionZ)*(state->position.z - lastPositionZ));
        
        // Only update if change is small (slow drift, not sudden push)
        // EKLE: Threshold artırıldığı için 0.5f faktörü yeterli (0.15m * 0.5 = 7.5cm)
        if (posChange < POSITION_HOLD_DRIFT_THRESHOLD * 0.5f) {
          savedPosition.x = state->position.x;
          savedPosition.y = state->position.y;
          savedPosition.z = state->position.z;
          lastSavedPosition = savedPosition;
          lastPositionSaveTime = currentTime;
          lastPositionX = state->position.x;
          lastPositionY = state->position.y;
          lastPositionZ = state->position.z;
          savePositionToNVS(&savedPosition);  // Update NVS
          DEBUG_PRINTD("Position hold: Position updated x=%.2f y=%.2f z=%.2f\n",
                       (double)savedPosition.x, (double)savedPosition.y, (double)savedPosition.z);
        }
      }
      
      // Hold position by setting absolute position setpoint
      // This will make the drone return to saved position if disturbed
      // Only if no movement command is being sent (already checked above)
      if (velMag < POSITION_HOLD_MOVEMENT_THRESHOLD) {
        setpoint->mode.x = modeAbs;
        setpoint->mode.y = modeAbs;
        setpoint->position.x = savedPosition.x;
        setpoint->position.y = savedPosition.y;
        // If external force detected, position controller will automatically correct
        // by returning to savedPosition (setpoint is already set above)
        // Keep Z in velocity mode (altitude hold via throttle)
        // Don't override velocity.z if it's set by altitude hold
      }
    }
  }
}
PARAM_GROUP_START(flightmode)
PARAM_ADD(PARAM_UINT8, althold, &altHoldMode)
PARAM_ADD(PARAM_UINT8, poshold, &posHoldMode)
PARAM_ADD(PARAM_UINT8, posSet, &posSetMode)
PARAM_ADD(PARAM_UINT8, yawMode, &yawMode)
PARAM_ADD(PARAM_UINT8, yawRst, &carefreeResetFront)
PARAM_ADD(PARAM_UINT8, stabModeRoll, &stabilizationModeRoll)
PARAM_ADD(PARAM_UINT8, stabModePitch, &stabilizationModePitch)
PARAM_ADD(PARAM_UINT8, stabModeYaw, &stabilizationModeYaw)
PARAM_GROUP_STOP(flightmode)