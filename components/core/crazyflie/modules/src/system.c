#include <stdbool.h>
#include <inttypes.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "version.h"
#include "config.h"
#include "param.h"
#include "log.h"
#include "ledseq.h"
#include "adc_esp32.h"
#include "pm_esplane.h"
#include "config.h"
#include "system.h"
#include "platform.h"
#include "configblock.h"
#include "pid_nvs.h"
#include "worker.h"
// freeRTOSdebug.h removed - functions not used anywhere
#include "wifi_esp32.h"
#include "comm.h"
#include "stabilizer.h"
#include "commander.h"
#include "crtp_commander.h"
#include "console.h"
#include "wifilink.h"
#include "web_server.h"
#include "mem.h"
#include "queuemonitor.h"
// DÜZELTME D2: Buzzer kullanılmıyor - sound.h kaldırıldı
#include "sysload.h"
#include "estimator_kalman.h"
#include "app.h"
#include "flowdeck_v1v2.h"
#include "zranger2.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "SYS"
#include "debug_cf.h"
#include "static_mem.h"
#include "cfassert.h"
// Güvenlik için başlangıçta disarmed (motorlar kapalı)
// Kullanıcı arayüzden "Başlat" butonuna basarak arm edecek
#ifndef START_DISARMED
#define ARM_INIT false  // Güvenlik: Başlangıçta disarmed
#else
#define ARM_INIT false
#endif
static bool selftestPassed;
static bool canFly;
static bool armed = ARM_INIT;
static bool forceArm;
static bool isInit;
STATIC_MEM_TASK_ALLOC(systemTask, SYSTEM_TASK_STACKSIZE);
xSemaphoreHandle canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;
static void systemTask(void *arg);
void systemLaunch(void)
{
  STATIC_MEM_TASK_CREATE(systemTask, systemTask, SYSTEM_TASK_NAME, NULL, SYSTEM_TASK_PRI);
}
void systemInit(void)
{
  if(isInit)
    return;
  DEBUG_PRINT_LOCAL("----------------------------\n");
  DEBUG_PRINT_LOCAL("%s is up and running!\n", platformConfigGetDeviceTypeName());
  canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
  xSemaphoreTake(canStartMutex, portMAX_DELAY);
  sysLoadInit();
  debugInit();
  configblockInit();
  pidNvsInit(); // Initialize PID NVS system
  workerInit();
  adcInit();
  ledseqInit();
  pmInit();
#ifdef APP_ENABLED
  appInit();
#endif
  isInit = true;
}
bool systemTest()
{
  bool pass=isInit;
  pass &= ledseqTest();
  pass &= pmTest();
  DEBUG_PRINTI("pmTest = %d", pass);
  pass &= workerTest();
  DEBUG_PRINTI("workerTest = %d", pass);
  return pass;
}
void systemTask(void *arg)
{
  bool pass = true;
  ledInit();
  ledSet(CHG_LED, 1);
  wifiInit();
  vTaskDelay(M2T(500));
  webServerInit();
#ifdef DEBUG_QUEUE_MONITOR
  queueMonitorInit();
#endif
  // DÜZELTME D5: UART init comment temizlendi - UART kullanılmıyor, comment kaldırıldı
  systemInit();
  commInit();
  commanderInit();
  StateEstimatorType estimator = kalmanEstimator;
  estimatorKalmanTaskInit();
  flowdeck2Init();  
  zRanger2Init();   
  stabilizerInit(estimator);
  DEBUG_PRINTI("Position hold mode: Continuous (manual control + auto-hold)");
  // DÜZELTME D2: Buzzer kullanılmıyor - soundInit() kaldırıldı
  memInit();
  // Removed: proximityInit() - proximity hardware not available
  pass &= wifiTest();
  pass &= webServerTest();
  DEBUG_PRINTI("webServerTest = %d ", pass);
  pass &= systemTest();
  DEBUG_PRINTI("systemTest = %d ", pass);
  pass &= configblockTest();
  DEBUG_PRINTI("configblockTest = %d ", pass);
  pass &= commTest();
  DEBUG_PRINTI("commTest = %d ", pass);
  pass &= commanderTest();
  DEBUG_PRINTI("commanderTest = %d ", pass);
  pass &= stabilizerTest();
  DEBUG_PRINTI("stabilizerTest = %d ", pass);
  pass &= estimatorKalmanTaskTest();
  DEBUG_PRINTI("estimatorKalmanTaskTest = %d ", pass);
  pass &= flowdeck2Test();
  DEBUG_PRINTI("flowdeck2Test = %d ", pass);
  pass &= zRanger2Test();
  DEBUG_PRINTI("zRanger2Test = %d ", pass);
  // DÜZELTME D2: Buzzer kullanılmıyor - soundTest() kaldırıldı
  pass &= memTest();
  DEBUG_PRINTI("memTest = %d ", pass);
  pass &= cfAssertNormalStartTest();
  if(pass)
  {
    selftestPassed = 1;
    systemStart();
    DEBUG_PRINTI("systemStart ! selftestPassed = %d", selftestPassed);
    // DÜZELTME D2: Buzzer kullanılmıyor - soundSetEffect(SND_STARTUP) kaldırıldı
    ledseqRun(&seq_alive);
    ledseqRun(&seq_testPassed);
  }
  else
  {
    selftestPassed = 0;
    if (systemTest())
    {
      while(1)
      {
        ledseqRun(&seq_testFailed);
        vTaskDelay(M2T(2000));
        if (selftestPassed)
        {
	        DEBUG_PRINT("Start forced.\n");
          systemStart();
          break;
        }
      }
    }
    else
    {
      ledInit();
      ledSet(SYS_LED, true);
    }
  }
  DEBUG_PRINT("Free heap: %"PRIu32" bytes\n", xPortGetFreeHeapSize());
  workerLoop();
  while(1)
    vTaskDelay(portMAX_DELAY);
}
void systemStart()
{
  xSemaphoreGive(canStartMutex);
#ifndef DEBUG_EP2
#endif
}
void systemWaitStart(void)
{
  while(!isInit)
    vTaskDelay(2);
  xSemaphoreTake(canStartMutex, portMAX_DELAY);
  xSemaphoreGive(canStartMutex);
}
void systemSetCanFly(bool val)
{
  canFly = val;
}
bool systemCanFly(void)
{
  return canFly;
}
static bool canArm(void)
{
  extern bool sensorsAreCalibrated(void);
  if (!sensorsAreCalibrated()) {
    DEBUG_PRINTW("Arming blocked: Sensors not calibrated\n");
    return false;
  }
  extern float pmGetBatteryVoltage(void);
  float batteryVoltage = pmGetBatteryVoltage();
  
  // EKLE: Batarya ölçümü geçersizse (0V veya negatif) uçuşa izin verme
  if (batteryVoltage <= 0.0f) {
    DEBUG_PRINTW("Arming blocked: Battery voltage measurement invalid (%.2fV) - check ADC connection\n", 
                 (double)batteryVoltage);
    return false;
  }
  
  #define MIN_ARMING_VOLTAGE_4S 12.8f  
  if (batteryVoltage < MIN_ARMING_VOLTAGE_4S) {
    DEBUG_PRINTW("Arming blocked: Battery voltage too low: %.2fV (min: %.2fV for 4S LiPo)\n", 
                 (double)batteryVoltage, (double)MIN_ARMING_VOLTAGE_4S);
    return false;
  }
  if (!selftestPassed) {
    DEBUG_PRINTW("Arming blocked: Self-test not passed (system may not be fully initialized)\n");
    DEBUG_PRINTW("Note: If system is running normally, this should not happen. Check system status.\n");
    return false;
  }
  return true;
}
void systemSetArmed(bool val)
{
  if (val && !forceArm) {
    if (!canArm()) {
      DEBUG_PRINTE("Arming denied due to safety checks\n");
      armed = false;
      return;
    }
    DEBUG_PRINTI("Arming allowed - all safety checks passed\n");
  }
  armed = val;
}
bool systemIsArmed()
{
  return armed || forceArm;
}
PARAM_GROUP_START(system)
PARAM_ADD(PARAM_INT8 | PARAM_RONLY, selftestPassed, &selftestPassed)
PARAM_ADD(PARAM_INT8, forceArm, &forceArm)
PARAM_GROUP_STOP(system)
LOG_GROUP_START(sys)
LOG_ADD(LOG_INT8, canfly, &canFly)
LOG_ADD(LOG_INT8, armed, &armed)
LOG_GROUP_STOP(sys)