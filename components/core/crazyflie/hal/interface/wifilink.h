#ifndef __WIFILINK_H__
#define __WIFILINK_H__
#include <stdbool.h>
#include "crtp.h"
// Removed: SYSLINK_MTU, SYSLINK_START_BYTE - SYSLINK not used, WiFi communication used instead
#define CRTP_START_BYTE  0xAA
void wifilinkInit();
bool wifilinkTest();
struct crtpLinkOperations *wifilinkGetLink();
#endif