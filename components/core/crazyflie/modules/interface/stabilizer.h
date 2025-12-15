#ifndef STABILIZER_H_
#define STABILIZER_H_
#include <stdbool.h>
#include <stdint.h>
#include "estimator.h"
#define EMERGENCY_STOP_TIMEOUT_DISABLED (-1)
void stabilizerInit(StateEstimatorType estimator);
bool stabilizerTest(void);
void stabilizerSetEmergencyStop();
void stabilizerResetEmergencyStop();
void stabilizerSetEmergencyStopTimeout(int timeout);
bool stabilizerGetState(state_t *state);
#endif 