#ifndef IMU_H_
#define IMU_H_
#include <stdbool.h>
#include "filter.h"
#include "imu_types.h"
#define IMU_UPDATE_FREQ   500
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)
#define IMU_ACC_WANTED_LPF_CUTOFF_HZ  4
#define IMU_ACC_IIR_LPF_ATTENUATION (IMU_UPDATE_FREQ / (2 * 3.1415 * IMU_ACC_WANTED_LPF_CUTOFF_HZ))
#define IMU_ACC_IIR_LPF_ATT_FACTOR  (int)(((1<<IIR_SHIFT) / IMU_ACC_IIR_LPF_ATTENUATION) + 0.5)
void imu6Init(void);
bool imu6Test(void);
bool imu6ManufacturingTest(void);
void imu6Read(Axis3f* gyro, Axis3f* acc);
void imu9Read(Axis3f* gyroOut, Axis3f* accOut, Axis3f* magOut);
bool imu6IsCalibrated(void);
// Removed: imuHasBarometer - Barometer hardware not available
bool imuHasMangnetometer(void);
#endif 