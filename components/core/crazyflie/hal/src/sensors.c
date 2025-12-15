#define DEBUG_MODULE "SENSORS"
#include "sensors.h"
#include "platform.h"
#include "debug_cf.h"
#define xstr(s) str(s)
#define str(s) #s
// Removed: BMI088 sensor drivers - hardware not available, using MPU9250 instead
// Removed: Bosch sensor drivers - hardware not available, using MPU9250 instead
#ifdef SENSOR_INCLUDED_MPU9250_LPS25H
  #include "sensors_mpu9250_lps25h.h"
#endif
typedef struct {
  SensorImplementation_t implements;
  void (*init)(void);
  bool (*test)(void);
  bool (*areCalibrated)(void);
  bool (*manufacturingTest)(void);
  void (*acquire)(sensorData_t *sensors, const uint32_t tick);
  void (*waitDataReady)(void);
  bool (*readGyro)(Axis3f *gyro);
  bool (*readAcc)(Axis3f *acc);
  bool (*readMag)(Axis3f *mag);
  // Removed: bool (*readBaro)(baro_t *baro) - Barometer hardware not available
  void (*setAccMode)(accModes accMode);
  void (*dataAvailableCallback)(void);
} sensorsImplementation_t;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static void nullFunction(void) {}
#pragma GCC diagnostic pop
// Dynamic array - only contains actual implementations (prevents empty slot issues)
static const sensorsImplementation_t sensorImplementations[] = {
// Removed: BMI088 sensor implementations - hardware not available, using MPU9250 instead
#ifdef SENSOR_INCLUDED_MPU9250_LPS25H
  {
    .implements = SensorImplementation_mpu9250_lps25h,
    .init = sensorsMpu9250Lps25hInit,
    .test = sensorsMpu9250Lps25hTest,
    .areCalibrated = sensorsMpu9250Lps25hAreCalibrated,
    .manufacturingTest = sensorsMpu9250Lps25hManufacturingTest,
    .acquire = sensorsMpu9250Lps25hAcquire,
    .waitDataReady = sensorsMpu9250Lps25hWaitDataReady,
    .readGyro = sensorsMpu9250Lps25hReadGyro,
    .readAcc = sensorsMpu9250Lps25hReadAcc,
    .readMag = sensorsMpu9250Lps25hReadMag,
    // Removed: .readBaro - Barometer hardware not available
    .setAccMode = sensorsMpu9250Lps25hSetAccMode,
    .dataAvailableCallback = nullFunction,
  },
#endif
// Removed: Bosch sensor implementation - hardware not available, using MPU9250 instead
};
#define SENSOR_IMPLEMENTATIONS_COUNT (sizeof(sensorImplementations) / sizeof(sensorImplementations[0]))
static const sensorsImplementation_t* activeImplementation;
static bool isInit = false;
static const sensorsImplementation_t* findImplementation(SensorImplementation_t implementation);
void sensorsInit(void) {
  if (isInit) {
    return;
  }
#ifndef SENSORS_FORCE
  SensorImplementation_t sensorImplementation = platformConfigGetSensorImplementation();
#else
  SensorImplementation_t sensorImplementation = SENSORS_FORCE;
  DEBUG_PRINTD("Forcing sensors to " xstr(SENSORS_FORCE) "\n");
#endif
  activeImplementation = findImplementation(sensorImplementation);
  if (!activeImplementation || !activeImplementation->init) {
    DEBUG_PRINT("ERROR: Sensor implementation not found or invalid! (enum=%d)\n", (int)sensorImplementation);
    // ASSERT or return - for now we return to prevent crash
    return;
  }
  activeImplementation->init();
  isInit = true;
}
bool sensorsTest(void) {
  if (!activeImplementation || !activeImplementation->test) {
    return false;
  }
  return activeImplementation->test();
}
bool sensorsAreCalibrated(void) {
  if (!activeImplementation || !activeImplementation->areCalibrated) {
    return false;
  }
  return activeImplementation->areCalibrated();
}
bool sensorsManufacturingTest(void){
  // Fixed: Call the function pointer instead of returning the pointer itself
  if (!activeImplementation || !activeImplementation->manufacturingTest) {
    return false;
  }
  return activeImplementation->manufacturingTest();
}
void sensorsAcquire(sensorData_t *sensors, const uint32_t tick) {
  if (activeImplementation && activeImplementation->acquire) {
    activeImplementation->acquire(sensors, tick);
  }
}
void sensorsWaitDataReady(void) {
  if (activeImplementation && activeImplementation->waitDataReady) {
    activeImplementation->waitDataReady();
  }
}
bool sensorsReadGyro(Axis3f *gyro) {
  if (!activeImplementation || !activeImplementation->readGyro) {
    return false;
  }
  return activeImplementation->readGyro(gyro);
}
bool sensorsReadAcc(Axis3f *acc) {
  if (!activeImplementation || !activeImplementation->readAcc) {
    return false;
  }
  return activeImplementation->readAcc(acc);
}
bool sensorsReadMag(Axis3f *mag) {
  if (!activeImplementation || !activeImplementation->readMag) {
    return false;
  }
  return activeImplementation->readMag(mag);
}
// Removed: sensorsReadBaro - Barometer hardware not available
void sensorsSetAccMode(accModes accMode) {
  if (activeImplementation && activeImplementation->setAccMode) {
    activeImplementation->setAccMode(accMode);
  }
}
void __attribute__((used)) EXTI14_Callback(void) {
  // Note: MPU9250 driver uses EXTI13_Callback (STM32) or GPIO ISR (ESP32)
  // This callback is kept for compatibility but may not be used
  if (activeImplementation && activeImplementation->dataAvailableCallback) {
    activeImplementation->dataAvailableCallback();
  }
}
static const sensorsImplementation_t* findImplementation(SensorImplementation_t implementation) {
  const sensorsImplementation_t* result = NULL;
  // Use dynamic array size instead of SensorImplementation_COUNT to prevent empty slot issues
  for (size_t i = 0; i < SENSOR_IMPLEMENTATIONS_COUNT; i++) {
    if (sensorImplementations[i].implements == implementation) {
      result = &sensorImplementations[i];
      break;
    }
  }
  return result;
}