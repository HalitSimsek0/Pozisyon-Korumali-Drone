#define DEBUG_MODULE "QM"
#include "queuemonitor.h"
#ifdef DEBUG_QUEUE_MONITOR
#include <stdbool.h>
#include "timers.h"
#include "debug_cf.h"
#include "cfassert.h"
#define MAX_NR_OF_QUEUES 20
#define TIMER_PERIOD M2T(10000)
#define RESET_COUNTERS_AFTER_DISPLAY true
#define DISPLAY_ONLY_OVERFLOW_QUEUES true
typedef struct
{
  char* fileName;
  char* queueName;
  int sendCount;
  int maxWaiting;
  int fullCount;
} Data;
static Data data[MAX_NR_OF_QUEUES];
static xTimerHandle timer;
static StaticTimer_t timerBuffer;
static unsigned char nrOfQueues = 1; 
static bool initialized = false;
static void timerHandler(xTimerHandle timer);
static void debugPrint();
static bool filter(Data* queueData);
static void debugPrintQueue(Data* queueData);
static Data* getQueueData(xQueueHandle* xQueue);
static int getMaxWaiting(xQueueHandle* xQueue, int prevPeak);
static void resetCounters();
unsigned char ucQueueGetQueueNumber( xQueueHandle xQueue );
void queueMonitorInit() {
  ASSERT(!initialized);
  timer = xTimerCreateStatic( "queueMonitorTimer", TIMER_PERIOD,
    pdTRUE, NULL, timerHandler, &timerBuffer);
  xTimerStart(timer, 100);
  data[0].fileName = "Na";
  data[0].queueName = "Na";
  initialized = true;
}
void qm_traceQUEUE_SEND(void* xQueue) {
  if(initialized) {
    Data* queueData = getQueueData(xQueue);
    queueData->sendCount++;
    queueData->maxWaiting = getMaxWaiting(xQueue, queueData->maxWaiting);
  }
}
void qm_traceQUEUE_SEND_FAILED(void* xQueue) {
  if(initialized) {
    Data* queueData = getQueueData(xQueue);
    queueData->fullCount++;
  }
}
void qmRegisterQueue(xQueueHandle* xQueue, char* fileName, char* queueName) {
  ASSERT(initialized);
  ASSERT(nrOfQueues < MAX_NR_OF_QUEUES);
  Data* queueData = &data[nrOfQueues];
  queueData->fileName = fileName;
  queueData->queueName = queueName;
  vQueueSetQueueNumber(xQueue, nrOfQueues);
  nrOfQueues++;
}
static Data* getQueueData(xQueueHandle* xQueue) {
  unsigned char number = uxQueueGetQueueNumber(xQueue);
  ASSERT(number < MAX_NR_OF_QUEUES);
  return &data[number];
}
static int getMaxWaiting(xQueueHandle* xQueue, int prevPeak) {
  unsigned portBASE_TYPE waiting = uxQueueMessagesWaitingFromISR(xQueue) + 1;
  if (waiting > prevPeak) {
    return waiting;
  }
  return prevPeak;
}
static void debugPrint() {
  int i = 0;
  for (i = 0; i < nrOfQueues; i++) {
    Data* queueData = &data[i];
    if (filter(queueData)) {
      debugPrintQueue(queueData);
    }
  }
  if (RESET_COUNTERS_AFTER_DISPLAY) {
    resetCounters();
  }
}
static bool filter(Data* queueData) {
  bool doDisplay = false;
  if (DISPLAY_ONLY_OVERFLOW_QUEUES) {
    doDisplay = (queueData->fullCount != 0);
  } else {
    doDisplay = true;
  }
  return doDisplay;
}
static void debugPrintQueue(Data* queueData) {
  DEBUG_PRINT("%s:%s, sent: %i, peak: %i, full: %i\n",
    queueData->fileName, queueData->queueName, queueData->sendCount,
    queueData->maxWaiting, queueData->fullCount);
}
static void resetCounters() {
  int i = 0;
  for (i = 0; i < nrOfQueues; i++) {
    Data* queueData = &data[i];
    queueData->sendCount = 0;
    queueData->maxWaiting = 0;
    queueData->fullCount = 0;
  }
}
static void timerHandler(xTimerHandle timer) {
  debugPrint();
}
#endif 