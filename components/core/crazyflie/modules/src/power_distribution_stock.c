#include <string.h>
#include "power_distribution.h"
#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "platform.h"
#include "motors.h"
#include "pm_esplane.h"
#include "config.h"  // EKLE: QUAD_FORMATION_X define'ı için
#include "system.h"  // DÜZELTME I5: systemIsArmed() için
#include "FreeRTOS.h"  // DÜZELTME I5: TickCount için
#include "task.h"  // DÜZELTME I5: TickCount için
#define DEBUG_MODULE "PWR_DIST"
#include "debug_cf.h"
static bool motorSetEnable = false;
static uint32_t motorSetEnableTimestamp = 0;  // DÜZELTME I5: Timeout kontrolü için
#define MOTOR_SET_ENABLE_TIMEOUT_MS 5000  // DÜZELTME I5: 5 saniye timeout
static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;
static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;
#ifndef DEFAULT_IDLE_THRUST
#define DEFAULT_IDLE_THRUST 0
#endif
static uint32_t idleThrust = DEFAULT_IDLE_THRUST;
// 4S LiPo batarya voltaj seviyeleri:
// - Tam dolu: ~16.8V (4.2V x 4 hücre)
// - Nominal: ~14.8V (3.7V x 4 hücre)
// - %20-25 kalan: ~13.5V (3.375V x 4) - LOW seviyesi
// - %10 kalan: ~12.8V (3.2V x 4) - CRITICAL seviyesi (auto-landing tetiklenir)
#define BATTERY_LOW_VOLTAGE_4S    13.5f      // ~%20-25 kalan
#define BATTERY_CRITICAL_VOLTAGE_4S 12.8f    // ~%10 kalan - kritik seviye, acil iniş gerekli
#define BATTERY_THRUST_REDUCTION_LOW 0.7f    // %70 thrust (LOW seviyesinde)
#define BATTERY_THRUST_REDUCTION_CRITICAL 0.5f  // %50 thrust (CRITICAL seviyesinde, auto-landing öncesi)  
static bool batteryFailsafeEnabled = true;
static bool batteryAutoLandingActive = false;  // DÜZELTME L4: Auto-landing flag
static uint32_t batteryCriticalStartTime = 0;  // DÜZELTME L4: Critical voltaj başlangıç zamanı
#define BATTERY_AUTO_LANDING_DELAY_MS 2000  // DÜZELTME L4: 2 saniye sonra auto-landing başlasın  
void powerDistributionInit(void)
{
  motorsInit(platformConfigGetMotorMapping());
}
bool powerDistributionTest(void)
{
  bool pass = true;
  pass &= motorsTest();
  return pass;
}
#define limitThrust(VAL) limitUint16(VAL)
void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}
void powerDistribution(const control_t *control)
{
  // DÜZELTME I5: motorSetEnable güvenlik kontrolü
  // Armed durumunda veya timeout sonrası otomatik olarak false yap
  if (motorSetEnable) {
    uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool shouldDisable = false;
    
    // Armed durumunda motorSetEnable false olmalı (güvenlik)
    if (systemIsArmed()) {
      DEBUG_PRINTW("motorSetEnable disabled: system is armed (safety)\n");
      shouldDisable = true;
    }
    // Timeout kontrolü
    else if (motorSetEnableTimestamp > 0) {
      uint32_t elapsed = currentTime - motorSetEnableTimestamp;
      if (elapsed > MOTOR_SET_ENABLE_TIMEOUT_MS) {
        DEBUG_PRINTW("motorSetEnable disabled: timeout (%lu ms)\n", elapsed);
        shouldDisable = true;
      }
    } else {
      // İlk kez enable edildi, timestamp kaydet
      motorSetEnableTimestamp = currentTime;
    }
    
    if (shouldDisable) {
      motorSetEnable = false;
      motorSetEnableTimestamp = 0;
    }
  } else {
    // Disable edildiğinde timestamp'i sıfırla
    motorSetEnableTimestamp = 0;
  }
  
  float batteryVoltage = pmGetBatteryVoltage();
  float thrustMultiplier = 1.0f;
  
  // DÜZELTME L4: Battery failsafe auto-landing
  if (batteryFailsafeEnabled && batteryVoltage > 0.0f) {
    if (batteryVoltage < BATTERY_CRITICAL_VOLTAGE_4S) {
      uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
      
      // Critical voltaj başlangıç zamanını kaydet
      if (batteryCriticalStartTime == 0) {
        batteryCriticalStartTime = currentTime;
        DEBUG_PRINTW("Battery CRITICAL: %.2fV - Auto-landing will start in %d ms\n", 
                     (double)batteryVoltage, BATTERY_AUTO_LANDING_DELAY_MS);
      }
      
      // Auto-landing delay sonrası aktif et
      uint32_t elapsed = currentTime - batteryCriticalStartTime;
      if (elapsed >= BATTERY_AUTO_LANDING_DELAY_MS) {
        if (!batteryAutoLandingActive) {
          batteryAutoLandingActive = true;
          DEBUG_PRINTE("Battery CRITICAL: %.2fV - AUTO-LANDING ACTIVATED!\n", (double)batteryVoltage);
        }
        // Auto-landing aktif: Thrust'ı sıfıra yakın değere düşür (iniş için minimum thrust)
        thrustMultiplier = 0.1f;  // %10 thrust - sadece iniş için
        static uint32_t auto_land_warn_count = 0;
        if (++auto_land_warn_count == 50) {
          DEBUG_PRINTE("AUTO-LANDING: Thrust reduced to 10%% for emergency landing\n");
          auto_land_warn_count = 0;
        }
      } else {
        // Delay süresince hala %50 thrust
        thrustMultiplier = BATTERY_THRUST_REDUCTION_CRITICAL;
        static uint32_t critical_warn_count = 0;
        if (++critical_warn_count == 100) { 
          DEBUG_PRINTW("Battery CRITICAL: %.2fV - Thrust reduced to 50%%, auto-landing in %lu ms\n", 
                       (double)batteryVoltage, BATTERY_AUTO_LANDING_DELAY_MS - elapsed);
          critical_warn_count = 0;
        }
      }
    } else if (batteryVoltage < BATTERY_LOW_VOLTAGE_4S) {
      // Low voltaj: Auto-landing'i sıfırla (voltaj normale döndü)
      batteryCriticalStartTime = 0;
      batteryAutoLandingActive = false;
      thrustMultiplier = BATTERY_THRUST_REDUCTION_LOW;  
      static uint32_t low_warn_count = 0;
      if (++low_warn_count == 500) { 
        DEBUG_PRINTW("Battery LOW: %.2fV - Thrust reduced to 70%%\n", (double)batteryVoltage);
        low_warn_count = 0;
      }
    } else {
      // Normal voltaj: Auto-landing'i sıfırla
      batteryCriticalStartTime = 0;
      batteryAutoLandingActive = false;
    }
  } else {
    // Failsafe disabled veya voltaj okunamıyor: Auto-landing'i sıfırla
    batteryCriticalStartTime = 0;
    batteryAutoLandingActive = false;
  }
  float adjustedThrust = control->thrust * thrustMultiplier;
  #ifdef QUAD_FORMATION_X
    int16_t r = control->roll / 2.0f;
    int16_t p = control->pitch / 2.0f;
    motorPower.m1 = limitThrust(adjustedThrust - r + p + control->yaw);
    motorPower.m2 = limitThrust(adjustedThrust - r - p - control->yaw);
    motorPower.m3 =  limitThrust(adjustedThrust + r - p + control->yaw);
    motorPower.m4 =  limitThrust(adjustedThrust + r + p - control->yaw);
  #else 
    motorPower.m1 = limitThrust(adjustedThrust + control->pitch +
                               control->yaw);
    motorPower.m2 = limitThrust(adjustedThrust - control->roll -
                               control->yaw);
    motorPower.m3 =  limitThrust(adjustedThrust - control->pitch +
                               control->yaw);
    motorPower.m4 =  limitThrust(adjustedThrust + control->roll -
                               control->yaw);
  #endif
  if (motorSetEnable)
  {
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
  }
  else
  {
    if (motorPower.m1 < idleThrust) {
      motorPower.m1 = idleThrust;
    }
    if (motorPower.m2 < idleThrust) {
      motorPower.m2 = idleThrust;
    }
    if (motorPower.m3 < idleThrust) {
      motorPower.m3 = idleThrust;
    }
    if (motorPower.m4 < idleThrust) {
      motorPower.m4 = idleThrust;
    }
    motorsSetRatio(MOTOR_M1, motorPower.m1);
    motorsSetRatio(MOTOR_M2, motorPower.m2);
    motorsSetRatio(MOTOR_M3, motorPower.m3);
    motorsSetRatio(MOTOR_M4, motorPower.m4);
  }
}
PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(motorPowerSet)
PARAM_GROUP_START(powerDist)
PARAM_ADD(PARAM_UINT32, idleThrust, &idleThrust)
PARAM_ADD(PARAM_UINT8, batteryFailsafe, &batteryFailsafeEnabled)
PARAM_GROUP_STOP(powerDist)
LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT32, m1, &motorPower.m1)
LOG_ADD(LOG_UINT32, m2, &motorPower.m2)
LOG_ADD(LOG_UINT32, m3, &motorPower.m3)
LOG_ADD(LOG_UINT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)