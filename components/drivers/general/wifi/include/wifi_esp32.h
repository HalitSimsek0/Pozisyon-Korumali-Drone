#ifndef WIFI_ESP32_H_
#define WIFI_ESP32_H_
#include <stdbool.h>
#include <stdint.h>
#define WIFI_RX_TX_PACKET_SIZE   (32)
typedef struct
{
  uint8_t size;
  uint8_t data[WIFI_RX_TX_PACKET_SIZE];
} UDPPacket;
void wifiInit(void);
bool wifiTest(void);
bool wifiGetDataBlocking(UDPPacket *in);
bool wifiSendData(uint32_t size, uint8_t* data);
#endif