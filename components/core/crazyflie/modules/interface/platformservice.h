#ifndef __PLATFORMSERVICE_H__
#define __PLATFORMSERVICE_H__
#include <stdbool.h>
#include "crtp.h"
void platformserviceInit(void);
bool platformserviceTest(void);
void platformserviceSendAppchannelPacket(CRTPPacket *p);
#endif 