#ifndef __PARAM_H__
#define __PARAM_H__
#include <stdbool.h>
#include <stdint.h>
void paramInit(void);
bool paramTest(void);
typedef struct paramVarId_s {
  uint16_t id;
  uint16_t ptr;
} __attribute__((packed)) paramVarId_t;
paramVarId_t paramGetVarId(char* group, char* name);
#define PARAM_VARID_IS_VALID(varId) (varId.id != 0xffffu)
int paramGetType(paramVarId_t varid);
void paramGetGroupAndName(paramVarId_t varid, char** group, char** name);
uint8_t paramVarSize(int type);
float paramGetFloat(paramVarId_t varid);
int paramGetInt(paramVarId_t varid);
unsigned int paramGetUint(paramVarId_t varid);
void paramSetInt(paramVarId_t varid, int valuei);
void paramSetFloat(paramVarId_t varid, float valuef);
struct param_s {
  uint8_t type;
  char * name;
  void * address;
};
#define PARAM_BYTES_MASK 0x03
#define PARAM_1BYTE  0x00
#define PARAM_2BYTES 0x01
#define PARAM_4BYTES 0x02
#define PARAM_8BYTES 0x03
#define PARAM_TYPE_INT   (0x00<<2)
#define PARAM_TYPE_FLOAT (0x01<<2)
#define PARAM_SIGNED (0x00<<3)
#define PARAM_UNSIGNED (0x01<<3)
#define PARAM_VARIABLE (0x00<<7)
#define PARAM_GROUP    (0x01<<7)
#define PARAM_RONLY (1<<6)
#define PARAM_START 1
#define PARAM_STOP  0
#define PARAM_SYNC 0x02
#define PARAM_UINT8 (PARAM_1BYTE | PARAM_TYPE_INT | PARAM_UNSIGNED)
#define PARAM_INT8  (PARAM_1BYTE | PARAM_TYPE_INT | PARAM_SIGNED)
#define PARAM_UINT16 (PARAM_2BYTES | PARAM_TYPE_INT | PARAM_UNSIGNED)
#define PARAM_INT16  (PARAM_2BYTES | PARAM_TYPE_INT | PARAM_SIGNED)
#define PARAM_UINT32 (PARAM_4BYTES | PARAM_TYPE_INT | PARAM_UNSIGNED)
#define PARAM_INT32  (PARAM_4BYTES | PARAM_TYPE_INT | PARAM_SIGNED)
#define PARAM_FLOAT (PARAM_4BYTES | PARAM_TYPE_FLOAT | PARAM_SIGNED)
#ifndef UNIT_TEST_MODE
#define PARAM_ADD(TYPE, NAME, ADDRESS) \
   { .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },
#define PARAM_ADD_GROUP(TYPE, NAME, ADDRESS) \
   { \
  .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },
#define PARAM_GROUP_START(NAME)  \
  static const struct param_s __params_##NAME[] __attribute__((section(".param." #NAME), used)) = { \
  PARAM_ADD_GROUP(PARAM_GROUP | PARAM_START, NAME, 0x0)
#define PARAM_GROUP_STOP(NAME) \
  PARAM_ADD_GROUP(PARAM_GROUP | PARAM_STOP, stop_##NAME, 0x0) \
  };
#else 
#define PARAM_ADD(TYPE, NAME, ADDRESS)
#define PARAM_ADD_GROUP(TYPE, NAME, ADDRESS)
#define PARAM_GROUP_START(NAME)
#define PARAM_GROUP_STOP(NAME)
#endif 
#endif 