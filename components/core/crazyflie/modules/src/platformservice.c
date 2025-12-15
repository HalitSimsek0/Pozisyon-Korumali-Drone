#include <stdbool.h>
#include "FreeRTOS.h"
#include <stdint.h>
#include <string.h>
#include "config.h"
#include "crtp.h"
#include "platformservice.h"
#include "version.h"
#include "platform.h"
#include "app_channel.h"
static bool isInit=false;
typedef enum {
  platformCommand   = 0x00,
  versionCommand    = 0x01,
  appChannel        = 0x02,
} Channel;
typedef enum {
  setContinousWave   = 0x00,
} PlatformCommand;
typedef enum {
  getProtocolVersion = 0x00,
  getFirmwareVersion = 0x01,
  getDeviceTypeName  = 0x02,
} VersionCommand;
void platformserviceHandler(CRTPPacket *p);
static void platformCommandProcess(uint8_t command, uint8_t *data);
static void versionCommandProcess(CRTPPacket *p);
void platformserviceInit(void)
{
  if (isInit)
    return;
  appchannelInit();
  crtpRegisterPortCB(CRTP_PORT_PLATFORM, platformserviceHandler);
  isInit = true;
}
bool platformserviceTest(void)
{
  return isInit;
}
void platformserviceHandler(CRTPPacket *p)
{
  switch (p->channel)
  {
    case platformCommand:
      platformCommandProcess(p->data[0], &p->data[1]);
      crtpSendPacket(p);
      break;
    case versionCommand:
      versionCommandProcess(p);
      break;
    case appChannel:
      appchannelIncomingPacket(p);
      break;
    default:
      break;
  }
}
static void platformCommandProcess(uint8_t command, uint8_t *data)
{
  switch (command) {
    case setContinousWave:
      break;
    default:
      break;
  }
}
void platformserviceSendAppchannelPacket(CRTPPacket *p)
{
  p->port = CRTP_PORT_PLATFORM;
  p->channel = appChannel;
  crtpSendPacket(p);
}
static void versionCommandProcess(CRTPPacket *p)
{
  switch (p->data[0]) {
    case getProtocolVersion:
      *(int*)&p->data[1] = PROTOCOL_VERSION;
      p->size = 5;
      crtpSendPacket(p);
      break;
    case getFirmwareVersion:
      strncpy((char*)&p->data[1], V_STAG, CRTP_MAX_DATA_SIZE-1);
      p->size = (strlen(V_STAG)>CRTP_MAX_DATA_SIZE-1)?CRTP_MAX_DATA_SIZE:strlen(V_STAG)+1;
      crtpSendPacket(p);
      break;
    case getDeviceTypeName:
      {
      const char* name = platformConfigGetDeviceTypeName();
      strncpy((char*)&p->data[1], name, CRTP_MAX_DATA_SIZE-1);
      p->size = (strlen(name)>CRTP_MAX_DATA_SIZE-1)?CRTP_MAX_DATA_SIZE:strlen(name)+1;
      crtpSendPacket(p);
      }
      break;
    default:
      break;
  }
}