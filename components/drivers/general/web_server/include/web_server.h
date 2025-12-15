#ifndef WEB_SERVER_H_
#define WEB_SERVER_H_
#include <stdbool.h>
#include <stdint.h>
bool webServerInit(void);
bool webServerTest(void);
void webServerLoadPidParamsFromNVS(void);
typedef enum {
    CALIB_NONE = 0,
    CALIB_GYRO = 1,
    CALIB_ACCEL = 2,
    CALIB_MAG = 3
} calibration_mode_t;
bool setCalibrationMode(calibration_mode_t mode);
calibration_mode_t getCalibrationMode(void);
#endif 