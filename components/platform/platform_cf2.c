#include <string.h>
#include "platform.h"
#include "motors.h"
#define DEBUG_MODULE "PLATFORM"
#include "debug_cf.h"
static platformConfig_t configs[] = {
    {
        .deviceType = "EP20",
        .deviceTypeName = "ESPlane 2.0 ",
        .sensorImplementation = SensorImplementation_mpu9250_lps25h,
        .physicalLayoutAntennasAreClose = false,
        .motorMap = motorMapDefaultBrushless,
    },
    {
        .deviceType = "ED12",
        .deviceTypeName = "ESP_Drone_v1_2",
        .sensorImplementation = SensorImplementation_mpu9250_lps25h,
        .physicalLayoutAntennasAreClose = false,
        .motorMap = motorMapDefaultBrushless,
    },
};
const platformConfig_t *platformGetListOfConfigurations(int *nrOfConfigs)
{
    *nrOfConfigs = sizeof(configs) / sizeof(platformConfig_t);
    return configs;
}
bool platformInitHardware()
{
    return true;
}
const char *platformConfigGetPlatformName()
{
    return "ED12";
}