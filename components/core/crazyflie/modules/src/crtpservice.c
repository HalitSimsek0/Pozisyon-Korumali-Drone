#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "crtp.h"
#include "crtpservice.h"
static bool isInit=false;
typedef enum {
  linkEcho   = 0x00,
  linkSource = 0x01,
  linkSink   = 0x02,
} LinkNbr;
void crtpserviceHandler(CRTPPacket *p);
void crtpserviceInit(void)
{
  if (isInit)
    return;
  crtpRegisterPortCB(CRTP_PORT_LINK, crtpserviceHandler);
  isInit = true;
}
bool crtpserviceTest(void)
{
  return isInit;
}
void crtpserviceHandler(CRTPPacket *p)
{
  switch (p->channel)
  {
    case linkEcho:
      crtpSendPacket(p);
      break;
    case linkSource:
      p->size = CRTP_MAX_DATA_SIZE;
      bzero(p->data, CRTP_MAX_DATA_SIZE);
      strcpy((char*)p->data, "Bitcraze Crazyflie");
      crtpSendPacket(p);
      break;
    case linkSink:
      break;
    default:
      break;
  }
}
