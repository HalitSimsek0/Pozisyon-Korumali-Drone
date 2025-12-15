#include <stddef.h>
#include <stdlib.h>
#include "stm32_legacy.h"
#include "trigger.h"
void triggerInit(trigger_t *trigger, triggerFunc_t func, float threshold, uint32_t triggerCount)
{
  assert_param(trigger != NULL);
  assert_param(func != triggerFuncNone);
  trigger->active = false;
  trigger->func = func;
  trigger->threshold = threshold;
  trigger->triggerCount = triggerCount;
  triggerReset(trigger);
}
void triggerRegisterHandler(trigger_t *trigger, triggerHandler_t handler, void *handlerArg)
{
  assert_param(trigger != NULL);
  assert_param(handler != NULL);
  trigger->handler = handler;
  trigger->handlerArg = handlerArg;
  trigger->handlerCalled = false;
}
void triggerDeInit(trigger_t *trigger)
{
  assert_param(trigger != NULL);
  triggerInit(trigger, triggerFuncNone, 0, 0);
  triggerRegisterHandler(trigger, NULL, NULL);
  triggerReset(trigger);
}
void triggerActivate(trigger_t *trigger, bool active)
{
  assert_param(trigger != NULL);
  triggerReset(trigger);
  trigger->active = active;
}
void triggerReset(trigger_t *trigger)
{
  assert_param(trigger != NULL);
  trigger->testCounter = 0;
  trigger->released = false;
  trigger->handlerCalled = false;
}
static void triggerIncTestCounter(trigger_t *trigger)
{
  assert_param(trigger != NULL);
   if(trigger->testCounter < trigger->triggerCount) {
     trigger->testCounter++;
   }
}
bool triggerTestValue(trigger_t *trigger, float testValue)
{
  assert_param(trigger != NULL);
  if(!trigger->active) {
    return false;
  }
  switch(trigger->func) {
    case triggerFuncIsLE: {
      if(testValue <= trigger->threshold) {
        triggerIncTestCounter(trigger);
        break;
      }
      else {
        triggerReset(trigger);
        return false;
      }
    }
    case triggerFuncIsGE: {
      if(testValue >= trigger->threshold) {
        triggerIncTestCounter(trigger);
        break;
      }
      else {
        triggerReset(trigger);
        return false;
      }
    }
    case triggerFuncNone: {
      break;
    }
  }
  trigger->released = (trigger->testCounter >= trigger->triggerCount);
  if(!trigger->released) {
    return false;
  }
  bool iReleased = trigger->released;
  if(trigger->released && (trigger->handler != NULL) && (!trigger->handlerCalled)) {
    trigger->handlerCalled = true;
    trigger->handler(trigger->handlerArg);
  }
  return iReleased;
}