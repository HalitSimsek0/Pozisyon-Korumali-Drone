#ifndef __WORKER_H
#define __WORKER_H
#include <stdbool.h>
void workerInit();
bool workerTest();
void workerLoop();
int workerSchedule(void (*function)(void*), void *arg);
#endif 