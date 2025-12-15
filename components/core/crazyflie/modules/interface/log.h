#ifndef __LOG_H__
#define __LOG_H__
#include <stdbool.h>
#include <stdint.h>
void logInit(void);
bool logTest(void);
typedef uint16_t logVarId_t;
logVarId_t logGetVarId(char* group, char* name);
#define LOG_VARID_IS_VALID(varId) (varId != 0xffffu)
int logGetType(logVarId_t varid);
void logGetGroupAndName(logVarId_t varid, char** group, char** name);
void* logGetAddress(logVarId_t varid);
uint8_t logVarSize(int type);
float logGetFloat(logVarId_t varid);
int logGetInt(logVarId_t varid);
unsigned int logGetUint(logVarId_t varid);
struct log_s {
  uint8_t type;
  char * name;
  void * address;
};
#define LOG_UINT8  1
#define LOG_UINT16 2
#define LOG_UINT32 3
#define LOG_INT8   4
#define LOG_INT16  5
#define LOG_INT32  6
#define LOG_FLOAT  7
#define LOG_FP16   8
typedef uint8_t (*logAcquireUInt8)(uint32_t timestamp, void* data);
typedef uint16_t (*logAcquireUInt16)(uint32_t timestamp, void* data);
typedef uint32_t (*logAcquireUInt32)(uint32_t timestamp, void* data);
typedef int8_t (*logAcquireInt8)(uint32_t timestamp, void* data);
typedef int16_t (*logAcquireInt16)(uint32_t timestamp, void* data);
typedef int32_t (*logAcquireInt32)(uint32_t timestamp, void* data);
typedef float (*logAcquireFloat)(uint32_t timestamp, void* data);
typedef struct {
  union {
    logAcquireUInt8 acquireUInt8;
    logAcquireUInt16 acquireUInt16;
    logAcquireUInt32 acquireUInt32;
    logAcquireInt8 acquireInt8;
    logAcquireInt16 acquireInt16;
    logAcquireInt32 acquireInt32;
    logAcquireFloat aquireFloat;
  };
  void* data;
} logByFunction_t;
#define LOG_GROUP 0x80
#define LOG_BY_FUNCTION 0x40
#define LOG_START 1
#define LOG_STOP  0
#ifndef UNIT_TEST_MODE
#define LOG_ADD(TYPE, NAME, ADDRESS) \
   { .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },
#define LOG_ADD_BY_FUNCTION(TYPE, NAME, ADDRESS) \
   { .type = TYPE | LOG_BY_FUNCTION, .name = #NAME, .address = (void*)(ADDRESS), },
#define LOG_ADD_GROUP(TYPE, NAME, ADDRESS) \
   { \
  .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },
#define LOG_GROUP_START(NAME)  \
  static const struct log_s __logs_##NAME[] __attribute__((section(".log." #NAME), used)) = { \
  LOG_ADD_GROUP(LOG_GROUP | LOG_START, NAME, 0x0)
#define LOG_GROUP_STOP(NAME) \
  LOG_ADD_GROUP(LOG_GROUP | LOG_STOP, stop_##NAME, 0x0) \
  };
#else 
#define LOG_ADD(TYPE, NAME, ADDRESS)
#define LOG_ADD_BY_FUNCTION(TYPE, NAME, ADDRESS)
#define LOG_ADD_GROUP(TYPE, NAME, ADDRESS)
#define LOG_GROUP_START(NAME)
#define LOG_GROUP_STOP(NAME)
#endif 
#endif 