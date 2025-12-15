#ifdef 0
#ifndef PM_H_
#define PM_H_
#include "adc.h"
// Removed: #include "syslink.h" - SYSLINK not used, WiFi communication used instead
#include "deck.h"
#ifndef CRITICAL_LOW_VOLTAGE
  #define PM_BAT_CRITICAL_LOW_VOLTAGE   3.0f
#else
  #define PM_BAT_CRITICAL_LOW_VOLTAGE   CRITICAL_LOW_VOLTAGE
#endif
#ifndef CRITICAL_LOW_TIMEOUT
  #define PM_BAT_CRITICAL_LOW_TIMEOUT   M2T(1000 * 5) 
#else
  #define PM_BAT_CRITICAL_LOW_TIMEOUT   CRITICAL_LOW_TIMEOUT
#endif
#ifndef LOW_VOLTAGE
  #define PM_BAT_LOW_VOLTAGE   3.2f
#else
  #define PM_BAT_LOW_VOLTAGE   LOW_VOLTAGE
#endif
#ifndef LOW_TIMEOUT
  #define PM_BAT_LOW_TIMEOUT   M2T(1000 * 5) 
#else
  #define PM_BAT_LOW_TIMEOUT   LOW_TIMEOUT
#endif
#ifndef SYSTEM_SHUTDOWN_TIMEOUT
  #define PM_SYSTEM_SHUTDOWN_TIMEOUT    M2T(1000 * 60 * 5) 
#else
  #define PM_SYSTEM_SHUTDOWN_TIMEOUT    M2T(1000 * 60 * SYSTEM_SHUTDOWN_TIMEOUT)
#endif
#define PM_BAT_DIVIDER                3.0f
#define PM_BAT_ADC_FOR_3_VOLT         (int32_t)(((3.0f / PM_BAT_DIVIDER) / 2.8f) * 4096)
#define PM_BAT_ADC_FOR_1p2_VOLT       (int32_t)(((1.2f / PM_BAT_DIVIDER) / 2.8f) * 4096)
#define PM_BAT_IIR_SHIFT     8
#define PM_BAT_WANTED_LPF_CUTOFF_HZ   1
#define PM_BAT_IIR_LPF_ATTENUATION (int)(ADC_SAMPLING_FREQ / (int)(2 * 3.1415f * PM_BAT_WANTED_LPF_CUTOFF_HZ))
#define PM_BAT_IIR_LPF_ATT_FACTOR  (int)((1<<PM_BAT_IIR_SHIFT) / PM_BAT_IIR_LPF_ATTENUATION)
typedef enum
{
  battery,
  charging,
  charged,
  lowPower,
  shutDown,
} PMStates;
typedef enum
{
  charge100mA,
  charge500mA,
  chargeMax,
} PMChargeStates;
// Removed: PMUSBPower enum - USB power detection not available on ESP32 (STM32-specific feature)
void pmInit(void);
bool pmTest(void);
void pmTask(void *param);
void pmSetChargeState(PMChargeStates chgState);
// Removed: pmSyslinkUpdate - SYSLINK not used, WiFi communication used instead
float pmGetBatteryVoltage(void);
float pmGetBatteryVoltageMin(void);
float pmGetBatteryVoltageMax(void);
void pmBatteryUpdate(AdcGroup* adcValues);
bool pmIsBatteryLow(void);
bool pmIsChargerConnected(void);
bool pmIsCharging(void);
bool pmIsDischarging(void);
void pmEnableExtBatteryVoltMeasuring(const deckPin_t pin, float multiplier);
float pmMeasureExtBatteryVoltage(void);
void pmEnableExtBatteryCurrMeasuring(const deckPin_t pin, float ampPerVolt);
float pmMeasureExtBatteryCurrent(void);
#endif 
#endif