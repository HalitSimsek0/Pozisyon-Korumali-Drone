#include <stdbool.h>
#include <stddef.h>
#include "crtp_commander.h"
#include "cfassert.h"
#include "commander.h"
#include "crtp.h"
static bool isInit;
static void commanderCrtpCB(CRTPPacket* pk);
void crtpCommanderInit(void)
{
  if(isInit) {
    return;
  }
  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_SETPOINT, commanderCrtpCB);
  crtpRegisterPortCB(CRTP_PORT_SETPOINT_GENERIC, commanderCrtpCB);
  crtpCommanderRpytInit(); 
  isInit = true;
}
enum crtpSetpointGenericChannel {
  SET_SETPOINT_CHANNEL = 0,
  META_COMMAND_CHANNEL = 1,
};
enum metaCommand_e {
  metaNotifySetpointsStop = 0,
  nMetaCommands,
};
typedef void (*metaCommandDecoder_t)(const void *data, size_t datalen);
struct notifySetpointsStopPacket {
  uint32_t remainValidMillisecs;
} __attribute__((packed));
void notifySetpointsStopDecoder(const void *data, size_t datalen)
{
  ASSERT(datalen == sizeof(struct notifySetpointsStopPacket));
  const struct notifySetpointsStopPacket *values = data;
  commanderNotifySetpointsStop(values->remainValidMillisecs);
}
const static metaCommandDecoder_t metaCommandDecoders[] = {
  [metaNotifySetpointsStop] = notifySetpointsStopDecoder,
};
static void commanderCrtpCB(CRTPPacket* pk)
{
  static setpoint_t setpoint;
  if(pk->port == CRTP_PORT_SETPOINT && pk->channel == 0) {
    crtpCommanderRpytDecodeSetpoint(&setpoint, pk);
    commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
  } else if (pk->port == CRTP_PORT_SETPOINT_GENERIC) {
    switch (pk->channel) {
    case SET_SETPOINT_CHANNEL:
      crtpCommanderGenericDecodeSetpoint(&setpoint, pk);
      commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
      break;
    case META_COMMAND_CHANNEL: {
        uint8_t metaCmd = pk->data[0];
        if (metaCmd < nMetaCommands && (metaCommandDecoders[metaCmd] != NULL)) {
          metaCommandDecoders[metaCmd](pk->data + 1, pk->size - 1);
        }
      }
      break;
    default:
      break;
    }
  }
}