#ifndef __QUEUE_MONITOR_H__
#define __QUEUE_MONITOR_H__
#include "FreeRTOS.h"
#ifdef DEBUG_QUEUE_MONITOR
  #include "queue.h"
  void queueMonitorInit();
  #define DEBUG_QUEUE_MONITOR_REGISTER(queue) qmRegisterQueue(queue, __FILE__, #queue)
  void qm_traceQUEUE_SEND(void* xQueue);
  void qm_traceQUEUE_SEND_FAILED(void* xQueue);
  void qmRegisterQueue(xQueueHandle* xQueue, char* fileName, char* queueName);
#else
  #define DEBUG_QUEUE_MONITOR_REGISTER(queue)
#endif 
#endif 