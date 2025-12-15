#include "app_channel.h"
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "crtp.h"
#include "platformservice.h"
#include "stm32_legacy.h"
static SemaphoreHandle_t sendMutex;
static xQueueHandle  rxQueue;
static bool overflow;
void appchannelSendPacket(void* data, size_t length)
{
  static CRTPPacket packet;
  xSemaphoreTake(sendMutex, portMAX_DELAY);
  packet.size = (length > APPCHANNEL_MTU)?APPCHANNEL_MTU:length;
  memcpy(packet.data, data, packet.size);
  platformserviceSendAppchannelPacket(&packet);
  xSemaphoreGive(sendMutex);
}
size_t appchannelReceivePacket(void* buffer, size_t max_length, int timeout_ms) {
  static CRTPPacket packet;
  int tickToWait = 0;
  if (timeout_ms < 0) {
    tickToWait = portMAX_DELAY;
  } else {
    tickToWait = M2T(timeout_ms);
  }
  int result = xQueueReceive(rxQueue, &packet, tickToWait);
  if (result == pdTRUE) {
    int lenghtToCopy = (max_length < packet.size)?max_length:packet.size;
    memcpy(buffer, packet.data, lenghtToCopy);
    return lenghtToCopy;
  } else {
    return 0;
  }
}
bool appchannelHasOverflowOccured()
{
  bool hasOverflowed = overflow;
  overflow = false;
  return hasOverflowed;
}
void appchannelInit()
{
  sendMutex = xSemaphoreCreateMutex();
  rxQueue = xQueueCreate(10, sizeof(CRTPPacket));
  overflow = false;
}
void appchannelIncomingPacket(CRTPPacket *p)
{
  int res = xQueueSend(rxQueue, p, 0);
  if (res != pdTRUE) {
    overflow = true;
  }
}