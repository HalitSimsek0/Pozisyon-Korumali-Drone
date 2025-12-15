#ifndef __SENSORS_H__
#define __SENSORS_H__
#include "stabilizer_types.h"
typedef enum { ACC_MODE_PROPTEST, ACC_MODE_FLIGHT } accModes;
void sensorsInit(void);
bool sensorsTest(void);
bool sensorsAreCalibrated(void);
bool sensorsManufacturingTest(void);
void sensorsAcquire(sensorData_t *sensors, const uint32_t tick);
void sensorsWaitDataReady(void);
bool sensorsReadGyro(Axis3f *gyro);
bool sensorsReadAcc(Axis3f *acc);
bool sensorsReadMag(Axis3f *mag);
// Removed: sensorsReadBaro - Barometer hardware not available
void sensorsSetAccMode(accModes accMode);
#endif 