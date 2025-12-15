#include <string.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "system.h"
#include "pm_esplane.h"
#include "adc_esp32.h"
#include "led.h"
#include "log.h"
#include "ledseq.h"
#include "commander.h"
// DÜZELTME D2: Buzzer kullanılmıyor - sound.h kaldırıldı
// Removed: #include "stm32_legacy.h" - STM32 legacy code not needed for ESP32
#define DEBUG_MODULE "PM"
#include "debug_cf.h"
#include "static_mem.h"
// Removed: PmSyslinkInfo struct - syslink not used, no external PM board
// Charging state detection not available without external PM board
static float     batteryVoltage;
static uint16_t  batteryVoltageMV;
static float     batteryVoltageMin = 6.0;
static float     batteryVoltageMax = 0.0;
static float     extBatteryVoltage;
static uint16_t  extBatteryVoltageMV;
static uint16_t extBatVoltDeckPin;
static bool      isExtBatVoltDeckPinSet = false;
static float     extBatVoltMultiplier;
static float     extBatteryCurrent;
static uint16_t extBatCurrDeckPin;
static bool      isExtBatCurrDeckPinSet = false;
static float     extBatCurrAmpPerVolt;
#ifdef PM_SYSTLINK_INLCUDE_TEMP
static float    temp;
#endif
static uint32_t batteryLowTimeStamp;
static uint32_t batteryCriticalLowTimeStamp;
static bool isInit;
static PMStates pmState;
// Removed: pmSyslinkInfo - syslink not used
static uint8_t batteryLevel;
static void pmSetBatteryVoltage(float voltage);
const static float bat671723HS25C[10] =
{
  3.00, 
  3.78, 
  3.83, 
  3.87, 
  3.89, 
  3.92, 
  3.96, 
  4.00, 
  4.04, 
  4.10  
};
STATIC_MEM_TASK_ALLOC(pmTask, PM_TASK_STACKSIZE);
void pmInit(void)
{
  if(isInit) {
    return;
  }
    pmEnableExtBatteryVoltMeasuring(CONFIG_ADC1_PIN, 2); 
    // Removed: pmSyslinkInfo initialization - syslink not used, no external PM board
    // Charging state detection not available (requires external PM board via syslink)
    pmSetBatteryVoltage(14.8f); // Default to 4S nominal voltage (14.8V)
    STATIC_MEM_TASK_CREATE(pmTask, pmTask, PM_TASK_NAME, NULL, PM_TASK_PRI);
    isInit = true;
}
bool pmTest(void)
{
  return isInit;
}
static void pmSetBatteryVoltage(float voltage)
{
  batteryVoltage = voltage;
  batteryVoltageMV = (uint16_t)(voltage * 1000);
  if (batteryVoltageMax < voltage)
  {
    batteryVoltageMax = voltage;
  }
  if (batteryVoltageMin > voltage)
  {
    batteryVoltageMin = voltage;
  }
}
static void pmSystemShutdown(void)
{
#ifdef ACTIVATE_AUTO_SHUTDOWN
#endif
}
static int32_t pmBatteryChargeFromVoltage(float voltage)
{
  int charge = 0;
  if (voltage < bat671723HS25C[0])
  {
    return 0;
  }
  if (voltage > bat671723HS25C[9])
  {
    return 9;
  }
  while (voltage >  bat671723HS25C[charge])
  {
    charge++;
  }
  return charge;
}
float pmGetBatteryVoltage(void)
{
  return batteryVoltage;
}
float pmGetBatteryVoltageMin(void)
{
  return batteryVoltageMin;
}
float pmGetBatteryVoltageMax(void)
{
  return batteryVoltageMax;
}
// Removed: pmSyslinkUpdate() - syslink not used, no external PM board
// Charging state is not detected (requires external PM board with syslink communication)
// Battery voltage is measured via ADC instead
void pmSetChargeState(PMChargeStates chgState)
{
}
PMStates pmUpdateState()
{
  PMStates state;
  // Removed: syslink-based charging detection - no external PM board available
  // isCharging and isPowerGood are always false without syslink
  // State machine simplified: only battery and lowPower states (no charging/charged detection)
  uint32_t batteryLowTime;
  batteryLowTime = xTaskGetTickCount() - batteryLowTimeStamp;
  
  // Without syslink, we can only detect low battery, not charging state
  // Check if battery is critically low
  if ((batteryLowTime > PM_BAT_LOW_TIMEOUT) && (pmGetBatteryVoltage() < PM_BAT_CRITICAL_LOW_VOLTAGE))
  {
    state = lowPower;
  }
  else
  {
    state = battery; // Normal battery state (charging/charged detection not available)
  }
  return state;
}
void pmEnableExtBatteryCurrMeasuring(uint8_t pin, float ampPerVolt)
{
  extBatCurrDeckPin = pin;
  isExtBatCurrDeckPinSet = true;
  extBatCurrAmpPerVolt = ampPerVolt;
}
float pmMeasureExtBatteryCurrent(void)
{
  float current;
  if (isExtBatCurrDeckPinSet)
  {
    current = analogReadVoltage(extBatCurrDeckPin) * extBatCurrAmpPerVolt;
  }
  else
  {
    current = 0.0;
  }
  return current;
}
void pmEnableExtBatteryVoltMeasuring(uint8_t pin, float multiplier)
{
  extBatVoltDeckPin = pin;
  isExtBatVoltDeckPinSet = true;
  extBatVoltMultiplier = multiplier;
}
float pmMeasureExtBatteryVoltage(void)
{
  float voltage;
  if (isExtBatVoltDeckPinSet)
  {
    voltage = analogReadVoltage(extBatVoltDeckPin) * extBatVoltMultiplier;
  }
  else
  {
    voltage = 0.0;
  }
  return voltage;
}
bool pmIsBatteryLow(void) {
  return (pmState == lowPower);
}
bool pmIsChargerConnected(void) {
  return (pmState == charging) || (pmState == charged);
}
bool pmIsCharging(void) {
  return (pmState == charging);
}
bool pmIsDischarging(void)
{
  PMStates pmState;
  pmState = pmUpdateState();
  return (pmState == lowPower) || (pmState == battery);
}
void pmTask(void *param)
{
  PMStates pmStateOld = battery;
  uint32_t tickCount = 0;
#ifdef configUSE_APPLICATION_TASK_TAG
	#if configUSE_APPLICATION_TASK_TAG == 1
    vTaskSetApplicationTaskTag(0, (void *)TASK_PM_ID_NBR);
    #endif
#endif
  tickCount = xTaskGetTickCount();
  batteryLowTimeStamp = tickCount;
  batteryCriticalLowTimeStamp = tickCount;
  pmSetChargeState(charge300mA);
  systemWaitStart();
  static uint32_t log_counter = 0;
  const uint32_t LOG_INTERVAL = 10; 
  while (1) {
  vTaskDelay(M2T(100));
  extBatteryVoltage = pmMeasureExtBatteryVoltage();
  extBatteryVoltageMV = (uint16_t)(extBatteryVoltage * 1000);
  extBatteryCurrent = pmMeasureExtBatteryCurrent();
  if (extBatteryVoltage <= 0.0f) {
    if (log_counter % LOG_INTERVAL == 0) {
      DEBUG_PRINTW("PM: Battery voltage measurement error (0.0V) - check ADC configuration, skipping update\n");
    }
    // Don't update batteryVoltage if ADC reading is invalid (0V)
  } else {
    // For 4S LiPo: convert pack voltage to per-cell voltage for charge calculation
    float cellVoltage = extBatteryVoltage / 4.0f;
    pmSetBatteryVoltage(extBatteryVoltage);
    batteryLevel = pmBatteryChargeFromVoltage(cellVoltage) * 10;
  }
  log_counter++;
  if (log_counter >= LOG_INTERVAL) {
    DEBUG_PRINTI("Battery: %.2fV (%.1f%%), min=%.2fV, max=%.2fV, current=%.2fA\n",
                 (double)extBatteryVoltage,
                 (double)batteryLevel,
                 (double)pmGetBatteryVoltageMin(),
                 (double)pmGetBatteryVoltageMax(),
                 (double)extBatteryCurrent);
    log_counter = 0;
  }
#ifdef DEBUG_EP2
  DEBUG_PRINTD("batteryLevel=%u extBatteryVoltageMV=%u \n", batteryLevel, extBatteryVoltageMV);
#endif
    tickCount = xTaskGetTickCount();
    if (pmGetBatteryVoltage() > PM_BAT_LOW_VOLTAGE)
    {
      batteryLowTimeStamp = tickCount;
    }
    if (pmGetBatteryVoltage() > PM_BAT_CRITICAL_LOW_VOLTAGE)
    {
      batteryCriticalLowTimeStamp = tickCount;
    }
        pmState = pmUpdateState();
    if (pmState != pmStateOld)
    {
      switch (pmState)
      {
        case charged:
          // DÜZELTME D2: Buzzer kullanılmıyor - soundSetEffect(SND_BAT_FULL) kaldırıldı
          systemSetCanFly(false);
          break;
        case charging:
          ledseqRunBlocking(&seq_charging);
          // DÜZELTME D2: Buzzer kullanılmıyor - soundSetEffect() kaldırıldı
          systemSetCanFly(false);
          break;
        case lowPower:
          ledseqRunBlocking(&seq_lowbat);
          // DÜZELTME D2: Buzzer kullanılmıyor - soundSetEffect(SND_BAT_LOW) kaldırıldı
          systemSetCanFly(false);  // FIXED: Low battery should prevent flight
          break;
        case battery:
          // Removed: SND_USB_DISC - USB power detection not available on ESP32 (STM32-specific)
          // No sound effect when switching to battery mode
          systemSetCanFly(true);
          break;
        default:
          systemSetCanFly(true);
          break;
      }
      pmStateOld = pmState;
    }
    switch (pmState)
    {
      case charged:
        break;
      case charging:
        {
          // For 4S LiPo: convert pack voltage to per-cell voltage for charge calculation
          float cellVoltage = pmGetBatteryVoltage() / 4.0f;
          float chargeLevel = pmBatteryChargeFromVoltage(cellVoltage) / 10.0f;
          ledseqSetChargeLevel(chargeLevel);
        }
        break;
      case lowPower:
        {
          uint32_t batteryCriticalLowTime;
          batteryCriticalLowTime = tickCount - batteryCriticalLowTimeStamp;
          if (batteryCriticalLowTime > PM_BAT_CRITICAL_LOW_TIMEOUT)
          {
            pmSystemShutdown();
          }
        }
        break;
      case battery:
        {
          if ((commanderGetInactivityTime() > PM_SYSTEM_SHUTDOWN_TIMEOUT))
          {
            pmSystemShutdown();
          }
        }
        break;
      default:
        break;
    }
  }
}
LOG_GROUP_START(pm)
LOG_ADD(LOG_FLOAT, vbat, &batteryVoltage)
LOG_ADD(LOG_UINT16, vbatMV, &batteryVoltageMV)
LOG_ADD(LOG_FLOAT, extVbat, &extBatteryVoltage)
LOG_ADD(LOG_UINT16, extVbatMV, &extBatteryVoltageMV)
LOG_ADD(LOG_FLOAT, extCurr, &extBatteryCurrent)
// Removed: chargeCurrent log - syslink not used, charge current not available
LOG_ADD(LOG_INT8, state, &pmState)
LOG_ADD(LOG_UINT8, batteryLevel, &batteryLevel)
#ifdef PM_SYSTLINK_INLCUDE_TEMP
LOG_ADD(LOG_FLOAT, temp, &temp)
#endif
LOG_GROUP_STOP(pm)
