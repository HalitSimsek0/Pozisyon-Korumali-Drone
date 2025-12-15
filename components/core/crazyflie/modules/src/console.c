#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "crtp.h"
#include "stm32_legacy.h"
#ifndef SCB_ICSR_VECTACTIVE_Msk
#define SCB_ICSR_VECTACTIVE_Msk 0x1FFUL
#endif
static CRTPPacket messageToPrint;
static bool messageSendingIsPending = false;
static xSemaphoreHandle synch = NULL;
static const char bufferFullMsg[] = "<F>\n";
static bool isInit;
static void addBufferFullMarker();
static bool consoleSendMessage(void)
{
  if (crtpSendPacket(&messageToPrint) == pdTRUE)
  {
    messageToPrint.size = 0;
    messageSendingIsPending = false;
  }
  else
  {
    return false;
  }
  return true;
}
void consoleInit()
{
  if (isInit)
    return;
  messageToPrint.size = 0;
  messageToPrint.header = CRTP_HEADER(CRTP_PORT_CONSOLE, 0);
  vSemaphoreCreateBinary(synch);
  messageSendingIsPending = false;
  isInit = true;
}
bool consoleTest(void)
{
  return isInit;
}
int consolePutchar(int ch)
{
  bool isInInterrupt = false; 
  if (!isInit) {
    return 0;
  }
  if (isInInterrupt) {
    return consolePutcharFromISR(ch);
  }
  if (xSemaphoreTake(synch, portMAX_DELAY) == pdTRUE)
  {
    if (messageSendingIsPending) 
    {
      consoleSendMessage();
    }
    if (! messageSendingIsPending) 
    {
      if (messageToPrint.size < CRTP_MAX_DATA_SIZE)
      {
        messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
        messageToPrint.size++;
      }
      if (ch == '\n' || messageToPrint.size >= CRTP_MAX_DATA_SIZE)
      {
        if (crtpGetFreeTxQueuePackets() == 1)
        {
          addBufferFullMarker();
        }
        messageSendingIsPending = true;
        consoleSendMessage();
      }
    }
    xSemaphoreGive(synch);
  }
  return (unsigned char)ch;
}
int consolePutcharFromISR(int ch) {
  BaseType_t higherPriorityTaskWoken;
  if (xSemaphoreTakeFromISR(synch, &higherPriorityTaskWoken) == pdTRUE) {
    if (messageToPrint.size < CRTP_MAX_DATA_SIZE)
    {
      messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
      messageToPrint.size++;
    }
    xSemaphoreGiveFromISR(synch, &higherPriorityTaskWoken);
  }
  return ch;
}
int consolePuts(char *str)
{
  int ret = 0;
  while(*str)
    ret |= consolePutchar(*str++);
  return ret;
}
void consoleFlush(void)
{
  if (xSemaphoreTake(synch, portMAX_DELAY) == pdTRUE)
  {
    consoleSendMessage();
    xSemaphoreGive(synch);
  }
}
static int findMarkerStart()
{
  int start = messageToPrint.size;
  if (start > 0 && messageToPrint.data[start - 1] == '\n')
  {
    start -= 1;
  }
  return start;
}
static void addBufferFullMarker()
{
  int endMarker = findMarkerStart() + sizeof(bufferFullMsg);
  if (endMarker >= (CRTP_MAX_DATA_SIZE)) 
  {
    endMarker = CRTP_MAX_DATA_SIZE;
  }
  int startMarker = endMarker - sizeof(bufferFullMsg);
  memcpy(&messageToPrint.data[startMarker], bufferFullMsg, sizeof(bufferFullMsg));
  messageToPrint.size = startMarker + sizeof(bufferFullMsg);
}