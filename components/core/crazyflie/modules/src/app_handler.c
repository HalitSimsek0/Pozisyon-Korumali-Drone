#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "static_mem.h"
#include "app.h"
#ifndef APP_STACKSIZE
#define APP_STACKSIZE 300
#endif
#ifndef APP_PRIORITY
#define APP_PRIORITY 0
#endif
static bool isInit = false;
STATIC_MEM_TASK_ALLOC(appTask, APP_STACKSIZE);
static void appTask(void *param);
void __attribute__((weak)) appInit()
{
  if (isInit) {
    return;
  }
  STATIC_MEM_TASK_CREATE(appTask, appTask, "app", NULL, APP_PRIORITY);
  isInit = true;
}
static void appTask(void *param)
{
  systemWaitStart();
  appMain();
  while(1) {
    vTaskDelay(portMAX_DELAY);
  }
}