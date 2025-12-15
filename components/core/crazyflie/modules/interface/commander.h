#ifndef COMMANDER_H_
#define COMMANDER_H_
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "stabilizer_types.h"
#define DEFAULT_YAW_MODE  XMODE
#define COMMANDER_WDT_TIMEOUT_STABILIZE  M2T(500)
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   M2T(750)  
#define COMMANDER_PRIORITY_DISABLE 0
#define COMMANDER_PRIORITY_CRTP    1
void commanderInit(void);
bool commanderTest(void);
uint32_t commanderGetInactivityTime(void);
void commanderSetSetpoint(setpoint_t *setpoint, int priority);
int commanderGetActivePriority(void);
void commanderNotifySetpointsStop(int remainValidMillisecs);
void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state);
#endif 