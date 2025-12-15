#ifndef _DEBUG_CF_H
#define _DEBUG_CF_H
#include "config.h"
#include "console.h"
#ifdef DEBUG_PRINT_ON_UART
  #include "uart1.h"
  #define uartPrintf uart1Printf
#endif
#ifdef DEBUG_PRINT_ON_SEGGER_RTT
  #include "SEGGER_RTT.h"
#endif
#ifndef DEBUG_MODULE
  #define DEBUG_MODULE "NULL"
  #define DEBUG_FMT(fmt) (DEBUG_MODULE ": " fmt)
#else
  #define DEBUG_FMT(fmt) (DEBUG_MODULE ": " fmt)
#endif
#ifndef DEBUG_FMT
#define DEBUG_FMT(fmt) fmt
#endif
void debugInit(void);
#include "esp_log.h"
#define DEBUG_PRINT_LOCAL(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,DEBUG_MODULE,fmt, ##__VA_ARGS__)
#define DEBUG_PRINT_REMOT(fmt, ...) consolePrintf(DEBUG_FMT(fmt), ##__VA_ARGS__)
#if defined(DEBUG_PRINT_ON_UART)
  #define DEBUG_PRINT(fmt, ...) uartPrintf(DEBUG_FMT(fmt), ##__VA_ARGS__)
  #define DEBUG_PRINT_OS(fmt, ...) uartPrintf(DEBUG_FMT(fmt), ##__VA_ARGS__)
#elif defined(DEBUG_PRINT_ON_SWO)
  #define DEBUG_PRINT(fmt, ...) eprintf(ITM_SendChar, fmt, ## __VA_ARGS__)
  #define DEBUG_PRINT_OS(fmt, ...) eprintf(ITM_SendChar, fmt, ## __VA_ARGS__)
#elif defined(DEBUG_PRINT_ON_SEGGER_RTT)
  #define DEBUG_PRINT(fmt, ...) SEGGER_RTT_printf(0, fmt, ## __VA_ARGS__)
  #define DEBUG_PRINT_OS(fmt, ...) SEGGER_RTT_printf(0, fmt, ## __VA_ARGS__)
#elif defined(DEBUG_PRINT_ON_CONSOLE)
  #include "esp_log.h"
  #define DEBUG_PRINT(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,DEBUG_MODULE,fmt, ##__VA_ARGS__) 
  #define DEBUG_PRINTE(fmt, ...) consolePrintf(DEBUG_FMT(fmt), ##__VA_ARGS__)
  #define DEBUG_PRINTW(fmt, ...) consolePrintf(DEBUG_FMT(fmt), ##__VA_ARGS__)
  #define DEBUG_PRINTI(fmt, ...) consolePrintf(DEBUG_FMT(fmt), ##__VA_ARGS__)
  #define DEBUG_PRINTD(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,DEBUG_MODULE,fmt, ##__VA_ARGS__) 
  #define DEBUG_PRINTV(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE,DEBUG_MODULE,fmt, ##__VA_ARGS__) 
  #define DEBUG_PRINT_OS(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,DEBUG_MODULE,fmt, ##__VA_ARGS__) 
#else
  #define DEBUG_PRINT(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,DEBUG_MODULE,fmt, ##__VA_ARGS__) 
  #define DEBUG_PRINTE(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR,DEBUG_MODULE,fmt, ##__VA_ARGS__)
  #define DEBUG_PRINTW(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN,DEBUG_MODULE,fmt, ##__VA_ARGS__)
  #define DEBUG_PRINTI(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,DEBUG_MODULE,fmt, ##__VA_ARGS__)
  #define DEBUG_PRINTD(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,DEBUG_MODULE,fmt, ##__VA_ARGS__)
  #define DEBUG_PRINTV(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE,DEBUG_MODULE,fmt, ##__VA_ARGS__)
  #define DEBUG_PRINT_OS(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,DEBUG_MODULE,fmt, ##__VA_ARGS__)
#endif
#ifndef PRINT_OS_DEBUG_INFO
  #undef DEBUG_PRINT_OS
  #define DEBUG_PRINT_OS(fmt, ...)
#endif
#ifdef TEST_PRINTS
  #define TEST_AND_PRINT(e, msgOK, msgFail)\
    if(e) { DEBUG_PRINT(msgOK); } else { DEBUG_PRINT(msgFail); }
  #define FAIL_PRINT(msg) DEBUG_PRINT(msg)
#else
  #define TEST_AND_PRINT(e, msgOK, msgFail)
  #define FAIL_PRINT(msg)
#endif
#endif