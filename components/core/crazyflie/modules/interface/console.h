#ifndef CONSOLE_H_
#define CONSOLE_H_
#include <stdbool.h>
#include "eprintf.h"
void consoleInit(void);
bool consoleTest(void);
int consolePutchar(int ch);
int consolePutcharFromISR(int ch);
int consolePuts(char *str);
void consoleFlush(void);
#define consolePrintf(FMT, ...) eprintf(consolePutchar, FMT, ## __VA_ARGS__)
#endif 