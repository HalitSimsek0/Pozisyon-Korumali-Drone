#include "debug_cf.h"
void debugInit(void)
{
#ifdef DEBUG_PRINT_ON_SEGGER_RTT
  SEGGER_RTT_Init();
  SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);
#endif
}