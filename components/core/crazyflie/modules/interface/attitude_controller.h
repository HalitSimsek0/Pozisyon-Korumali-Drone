#ifndef ATTITUDE_CONTROLLER_H_
#define ATTITUDE_CONTROLLER_H_
#include <stdbool.h>
#include "commander.h"
void attitudeControllerInit(const float updateDt);
bool attitudeControllerTest(void);
void attitudeControllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired);
void attitudeControllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired);
void attitudeControllerResetRollAttitudePID(void);
void attitudeControllerResetPitchAttitudePID(void);
void attitudeControllerResetAllPID(void);
void attitudeControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw);
#endif 