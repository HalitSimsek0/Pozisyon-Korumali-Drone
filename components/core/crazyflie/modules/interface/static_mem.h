#pragma once
#include "cfassert.h"
#define CCM_NOT_SUPPORTED
#if defined(UNIT_TEST_MODE)
  #define NO_DMA_CCM_SAFE_ZERO_INIT
#elif defined(CCM_NOT_SUPPORTED)
  #define NO_DMA_CCM_SAFE_ZERO_INIT
#else
  #define NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((section(".ccmbss")))
#endif
#if defined(UNIT_TEST_MODE)
  #define FORCE_CCM_ZERO_INIT
#elif defined(CCM_NOT_SUPPORTED)
  #define FORCE_CCM_ZERO_INIT
#else
  #define FORCE_CCM_ZERO_INIT __attribute__((section(".ccmbss")))
#endif
#define STATIC_MEM_QUEUE_ALLOC(NAME, LENGTH, ITEM_SIZE)\
  static const int osSys_ ## NAME ## Length = (LENGTH); \
  static const int osSys_ ## NAME ## ItemSize = (ITEM_SIZE); \
  NO_DMA_CCM_SAFE_ZERO_INIT static uint8_t osSys_ ## NAME ## Storage[(LENGTH) * (ITEM_SIZE)]; \
  NO_DMA_CCM_SAFE_ZERO_INIT static StaticQueue_t osSys_ ## NAME ## Mgm;
#define STATIC_MEM_QUEUE_CREATE(NAME) xQueueCreateStatic(osSys_ ## NAME ## Length, osSys_ ## NAME ## ItemSize, osSys_ ## NAME ## Storage, &osSys_ ## NAME ## Mgm)
#define STATIC_MEM_TASK_ALLOC(NAME, STACK_DEPTH) \
  static const int osSys_ ## NAME ## StackDepth = (STACK_DEPTH); \
  static StackType_t osSys_ ## NAME ## StackBuffer[(STACK_DEPTH)]; \
  NO_DMA_CCM_SAFE_ZERO_INIT static StaticTask_t osSys_ ## NAME ## TaskBuffer;
#define STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(NAME, STACK_DEPTH) \
  static const int osSys_ ## NAME ## StackDepth = (STACK_DEPTH); \
  NO_DMA_CCM_SAFE_ZERO_INIT static StackType_t osSys_ ## NAME ## StackBuffer[(STACK_DEPTH)]; \
  NO_DMA_CCM_SAFE_ZERO_INIT static StaticTask_t osSys_ ## NAME ## TaskBuffer;
#define STATIC_MEM_TASK_CREATE(NAME, FUNCTION, TASK_NAME, PARAMETERS, PRIORITY) xTaskCreateStatic((FUNCTION), (TASK_NAME), osSys_ ## NAME ## StackDepth, (PARAMETERS), (PRIORITY), osSys_ ## NAME ## StackBuffer, &osSys_ ## NAME ## TaskBuffer)
