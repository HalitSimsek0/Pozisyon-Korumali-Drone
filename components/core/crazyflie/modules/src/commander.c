#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "commander.h"
#include "crtp_commander.h"
#include <math.h>
#include "cf_math.h"
#include "param.h"
#include "stm32_legacy.h"
#include "static_mem.h"
#include "esp_log.h"
static const char* TAG = "COMMANDER";
static bool isInit;
const static setpoint_t nullSetpoint;
static setpoint_t tempSetpoint;
static state_t lastState;
const static int priorityDisable = COMMANDER_PRIORITY_DISABLE;
static uint32_t lastUpdate;
static QueueHandle_t setpointQueue;
STATIC_MEM_QUEUE_ALLOC(setpointQueue, 1, sizeof(setpoint_t));
static QueueHandle_t priorityQueue;
STATIC_MEM_QUEUE_ALLOC(priorityQueue, 1, sizeof(int));
void commanderInit(void)
{
  setpointQueue = STATIC_MEM_QUEUE_CREATE(setpointQueue);
  ASSERT(setpointQueue);
  xQueueSend(setpointQueue, &nullSetpoint, 0);
  priorityQueue = STATIC_MEM_QUEUE_CREATE(priorityQueue);
  ASSERT(priorityQueue);
  xQueueSend(priorityQueue, &priorityDisable, 0);
  crtpCommanderInit();
  lastUpdate = xTaskGetTickCount();
  isInit = true;
}
void commanderSetSetpoint(setpoint_t *setpoint, int priority)
{
  int currentPriority;
  const BaseType_t peekResult = xQueuePeek(priorityQueue, &currentPriority, 0);
  ASSERT(peekResult == pdTRUE);
  if (priority >= currentPriority) {
    setpoint->timestamp = xTaskGetTickCount();
    xQueueOverwrite(setpointQueue, setpoint);
    xQueueOverwrite(priorityQueue, &priority);
  }
}
void commanderNotifySetpointsStop(int remainValidMillisecs)
{
  uint32_t currentTime = xTaskGetTickCount();
  int timeSetback = MIN(
    COMMANDER_WDT_TIMEOUT_SHUTDOWN - M2T(remainValidMillisecs),
    currentTime
  );
  xQueuePeek(setpointQueue, &tempSetpoint, 0);
  tempSetpoint.timestamp = currentTime - timeSetback;
  xQueueOverwrite(setpointQueue, &tempSetpoint);
}
void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state)
{
  xQueuePeek(setpointQueue, setpoint, 0);
  lastUpdate = setpoint->timestamp;
  uint32_t currentTime = xTaskGetTickCount();
  static bool failsafeActive = false;
  static uint32_t failsafeStartTime = 0;
  if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
    if (!failsafeActive) {
      failsafeActive = true;
      failsafeStartTime = currentTime;
      ESP_LOGW(TAG, "Failsafe activated - WiFi connection lost, starting soft landing");
    }
    uint32_t failsafeDuration = currentTime - failsafeStartTime;
    uint32_t softLandingDuration = M2T(3000);  // DÜZELTME L1: 3 saniye soft landing süresi
    const float LANDING_HEIGHT_THRESHOLD = 0.15f;  // DÜZELTME L1: 15cm yükseklikte dur
    const float SOFT_LANDING_VELOCITY = -0.3f;  // DÜZELTME L1: 0.3 m/s aşağı doğru hız
    
    // DÜZELTME L1: Soft landing mekanizması
    if (state->position.z > LANDING_HEIGHT_THRESHOLD) {
      // Yükseklik yeterli: Yumuşak iniş yap
      float landingProgress = (float)failsafeDuration / (float)softLandingDuration;
      if (landingProgress > 1.0f) landingProgress = 1.0f;
      
      // Z ekseni için velocity mode - yavaşça aşağı doğru
      setpoint->mode.z = modeVelocity;
      setpoint->velocity.z = SOFT_LANDING_VELOCITY * (1.0f - landingProgress * 0.5f);  // İniş ilerledikçe yavaşla
      
      // X, Y eksenleri disable (position hold kapatıldı)
      setpoint->mode.x = modeDisable;
      setpoint->mode.y = modeDisable;
      
      // Roll, Pitch, Yaw kontrolü - stabil tut
      setpoint->mode.roll = modeAbs;
      setpoint->mode.pitch = modeAbs;
      setpoint->mode.yaw = modeVelocity;
      setpoint->attitude.roll = 0;
      setpoint->attitude.pitch = 0;
      setpoint->attitudeRate.yaw = 0;
      
      // Thrust'ı azalt (iniş için)
      if (setpoint->thrust > 0) {
        float thrustFactor = 1.0f - (landingProgress * 0.7f);  // %70'e kadar azalt
        setpoint->thrust = (uint16_t)(setpoint->thrust * thrustFactor);
        if (setpoint->thrust < 1000) setpoint->thrust = 1000;  // Minimum thrust
      }
      
      static uint32_t landing_warn_count = 0;
      if (++landing_warn_count == 100) {
        ESP_LOGW(TAG, "Soft landing: z=%.2fm, velocity=%.2fm/s, progress=%.1f%%", 
                 (double)state->position.z, (double)setpoint->velocity.z, landingProgress * 100.0f);
        landing_warn_count = 0;
      }
    } else {
      // Yere yaklaştı: Motorları durdur
      ESP_LOGE(TAG, "Soft landing complete: z=%.2fm - Stopping motors", (double)state->position.z);
      extern void stabilizerSetEmergencyStop(void);
      stabilizerSetEmergencyStop(); 
      memcpy(setpoint, &nullSetpoint, sizeof(nullSetpoint));
    }
  } else if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_STABILIZE) {
    failsafeActive = false; 
    xQueueOverwrite(priorityQueue, &priorityDisable);
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.roll = modeAbs;
    setpoint->mode.pitch = modeAbs;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitude.roll = 0;
    setpoint->attitude.pitch = 0;
    setpoint->attitudeRate.yaw = 0;
  } else {
    failsafeActive = false;
  }
  if (crtpCommanderRpytGetPosHoldMode()) {
    crtpCommanderRpytUpdatePositionHold(setpoint, state, currentTime);
  }
  lastState = *state;
}
bool commanderTest(void)
{
  return isInit;
}
uint32_t commanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}
int commanderGetActivePriority(void)
{
  int priority;
  const BaseType_t peekResult = xQueuePeek(priorityQueue, &priority, 0);
  ASSERT(peekResult == pdTRUE);
  return priority;
}
PARAM_GROUP_START(commander)
PARAM_GROUP_STOP(commander)