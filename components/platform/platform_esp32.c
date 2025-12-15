#include <string.h>
#include "platform.h"
#define PLATFORM_INFO_OTP_NR_OF_BLOCKS 16
#define PLATFORM_INFO_OTP_BLOCK_LEN 32
#if PLATFORM_DEVICE_TYPE_STRING_MAX_LEN < (PLATFORM_INFO_OTP_BLOCK_LEN + 1)
#error
#endif
#define DEFAULT_PLATFORM_STRING "0;EP20"
void platformGetDeviceTypeString(char *deviceTypeString)
{
    char *block = 0;
    if (!block || ((unsigned char)block[0]) == 0xff) {
        block = DEFAULT_PLATFORM_STRING;
    }
    strncpy(deviceTypeString, block, PLATFORM_INFO_OTP_BLOCK_LEN);
    deviceTypeString[PLATFORM_INFO_OTP_BLOCK_LEN] = '\0';
}