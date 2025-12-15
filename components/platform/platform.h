#ifndef PLATFORM_H_
#define PLATFORM_H_
#include <stdbool.h>
#include "motors.h"
#define PLATFORM_DEVICE_TYPE_STRING_MAX_LEN (32 + 1)
#define PLATFORM_DEVICE_TYPE_MAX_LEN (4 + 1)
#define SENSOR_INCLUDED_MPU9250_LPS25H
typedef enum {
// Removed: BMI088 sensor implementations - hardware not available, using MPU9250 instead
// Removed: Bosch sensor implementations - hardware not available, using MPU9250 instead
#ifdef SENSOR_INCLUDED_MPU9250_LPS25H
    SensorImplementation_mpu9250_lps25h,
#endif
    SensorImplementation_COUNT,
} SensorImplementation_t;
typedef struct {
    char deviceType[PLATFORM_DEVICE_TYPE_MAX_LEN];
    char deviceTypeName[20];
    SensorImplementation_t sensorImplementation;
    bool physicalLayoutAntennasAreClose;
    const MotorPerifDef **motorMap;
} platformConfig_t;
int platformInit(void);
void platformGetDeviceTypeString(char *deviceTypeString);
int platformParseDeviceTypeString(const char *deviceTypeString, char *deviceType);
int platformInitConfiguration(const platformConfig_t *configs, const int nrOfConfigs);
const platformConfig_t *platformGetListOfConfigurations(int *nrOfConfigs);
bool platformInitHardware();
// Removed: platformSetLowInterferenceRadioMode - Radio not used, WiFi communication used instead
const char *platformConfigGetPlatformName();
const char *platformConfigGetDeviceType();
const char *platformConfigGetDeviceTypeName();
SensorImplementation_t platformConfigGetSensorImplementation();
bool platformConfigPhysicalLayoutAntennasAreClose();
const MotorPerifDef **platformConfigGetMotorMapping();
#endif 