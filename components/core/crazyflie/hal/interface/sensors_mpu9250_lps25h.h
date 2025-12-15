#ifndef __SENSORS_MPU9250_LPS25H_H__
#define __SENSORS_MPU9250_LPS25H_H__
#include "sensors.h"
void sensorsMpu9250Lps25hInit(void);
bool sensorsMpu9250Lps25hTest(void);
bool sensorsMpu9250Lps25hAreCalibrated(void);
bool sensorsMpu9250Lps25hManufacturingTest(void);
void sensorsMpu9250Lps25hAcquire(sensorData_t *sensors, const uint32_t tick);
void sensorsMpu9250Lps25hWaitDataReady(void);
bool sensorsMpu9250Lps25hReadGyro(Axis3f *gyro);
bool sensorsMpu9250Lps25hReadAcc(Axis3f *acc);
bool sensorsMpu9250Lps25hReadMag(Axis3f *mag);
// Removed: sensorsMpu9250Lps25hReadBaro - Barometer hardware not available
void sensorsMpu9250Lps25hSetAccMode(accModes accMode);
#endif 