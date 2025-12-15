#ifndef __CONTROLLER_MELLINGER_H__
#define __CONTROLLER_MELLINGER_H__
#include "stabilizer_types.h"
void controllerMellingerInit(void);
bool controllerMellingerTest(void);
void controllerMellinger(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);
#endif 