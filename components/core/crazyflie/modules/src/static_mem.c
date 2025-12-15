#include <FreeRTOS.h>
#include "static_mem.h"
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
  NO_DMA_CCM_SAFE_ZERO_INIT static StaticTask_t xIdleTaskTCB;
  NO_DMA_CCM_SAFE_ZERO_INIT static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
  NO_DMA_CCM_SAFE_ZERO_INIT static StaticTask_t xTimerTaskTCB;
  NO_DMA_CCM_SAFE_ZERO_INIT static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}