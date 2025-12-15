#pragma once
#include <stddef.h>
#include <stdbool.h>
#include "crtp.h"
#define APPCHANNEL_WAIT_FOREVER (-1)
#define APPCHANNEL_MTU (31)
void appchannelSendPacket(void* data, size_t length);
size_t appchannelReceivePacket(void* buffer, size_t max_length, int timeout_ms);
bool appchannelHasOverflowOccured();
void appchannelInit();
void appchannelIncomingPacket(CRTPPacket *p);