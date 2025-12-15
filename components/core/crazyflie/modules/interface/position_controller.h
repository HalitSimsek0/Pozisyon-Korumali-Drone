#ifndef POSITION_CONTROLLER_H_
#define POSITION_CONTROLLER_H_
#include "stabilizer_types.h"
void positionControllerInit();
void positionControllerResetAllPID();
void positionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state);
void velocityController(float* thrust, attitude_t *attitude, setpoint_t *setpoint,
                                                             const state_t *state);
#endif 