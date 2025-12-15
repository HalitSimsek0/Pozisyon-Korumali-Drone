#include <stdbool.h>
#define DEBUG_MODULE "COMM"
#include "comm.h"
#include "config.h"
#include "crtp.h"
#include "console.h"
#include "crtpservice.h"
#include "param.h"
#include "debug_cf.h"
#include "log.h"
#include  "wifi_esp32.h"
#include "wifilink.h"
#include "platformservice.h"
#include "crtp_localization_service.h"
static bool isInit;
void commInit(void)
{
  if (isInit) {
    return;
  }
  logInit();
  paramInit();
  isInit = true;
}
bool commTest(void)
{
	bool pass = isInit;
	pass &= paramTest();
	DEBUG_PRINTI("paramTest = %d ", pass);
  return pass;
}