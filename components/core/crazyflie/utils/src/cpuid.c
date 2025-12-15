#include "cpuid.h"
#ifdef ESP_PLATFORM
#include "esp_system.h"
#include "esp_mac.h"
static uint8_t esp32_mac[6];
static bool mac_initialized = false;
#endif
unsigned char * cpuidGetId()
{
#ifdef ESP_PLATFORM
  if (!mac_initialized) {
    esp_read_mac(esp32_mac, ESP_MAC_WIFI_STA);
    mac_initialized = true;
  }
  return esp32_mac;
#else
  return (unsigned char *)(0x1FFFF7E8);
#endif
}
int cpuIdGetFlashSize()
{
#ifdef ESP_PLATFORM
  return 0;
#else
  return *((short*)(4*1024*1024));
#endif
}