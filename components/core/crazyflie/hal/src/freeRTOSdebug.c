#include <stdint.h>
#include "freertos/FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#define DEBUG_MODULE "FreeRTOSConfig"
#include "debug_cf.h"
// FreeRTOS debug trace functions - currently unused but kept for compatibility
// UART_OUTPUT_TRACE_DATA is not enabled, so these are empty stubs
void debugSendTraceInfo(unsigned int taskNbr)
{
  // Empty stub - trace functionality not enabled
  (void)taskNbr; // Suppress unused parameter warning
}
void debugInitTrace(void)
{
  // Empty stub - trace functionality not enabled
}