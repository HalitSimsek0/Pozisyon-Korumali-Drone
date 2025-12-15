#ifndef __TRIGGER_H__
#define __TRIGGER_H__
#include <stdint.h>
#include <stdbool.h>
typedef enum {
  triggerFuncNone   = 0, 
  triggerFuncIsLE   = 1, 
  triggerFuncIsGE   = 2, 
} triggerFunc_t;
typedef void (*triggerHandler_t)(void *);
typedef struct {
    bool active;              
    triggerFunc_t func;       
    float threshold;          
    uint32_t triggerCount;    
    triggerHandler_t handler; 
    bool handlerCalled;       
    void *handlerArg;         
    uint32_t testCounter;     
    bool released;            
} trigger_t;
void triggerInit(trigger_t *trigger, triggerFunc_t func, float threshold, uint32_t triggerCount);
void triggerRegisterHandler(trigger_t *trigger, triggerHandler_t handler, void *handlerArg);
void triggerDeInit(trigger_t *trigger);
void triggerActivate(trigger_t *trigger, bool active);
void triggerReset(trigger_t *trigger);
bool triggerTestValue(trigger_t *trigger, float testValue);
#endif
