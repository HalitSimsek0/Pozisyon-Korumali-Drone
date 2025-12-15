#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "i2cdev.h"
#include "zranger2.h"
#include "vl53l1x.h"
#include "cf_math.h"
#define DEBUG_MODULE "ZR2"
#include "debug_cf.h"
static const float expPointA = 2.5f;
static const float expStdA = 0.0025f; 
static const float expPointB = 4.0f;
static const float expStdB = 0.2f; 
static float expCoeff;
#define RANGE_OUTLIER_LIMIT 5000 
static int16_t range_last = 0;
static int16_t range_last_valid = 0;
static bool isInit;
static VL53L1_Dev_t dev;
static uint16_t zRanger2GetMeasurementAndRestart(VL53L1_Dev_t *dev)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    VL53L1_RangingMeasurementData_t rangingData;
    uint8_t dataReady = 0;
    uint16_t range = 0;
    const uint32_t TIMEOUT_MS = 100; 
    uint32_t timeout_start = xTaskGetTickCount();
    uint32_t timeout_ticks = pdMS_TO_TICKS(TIMEOUT_MS);
    while (dataReady == 0)
    {
        status = VL53L1_GetMeasurementDataReady(dev, &dataReady);
        if ((xTaskGetTickCount() - timeout_start) > timeout_ticks) {
            DEBUG_PRINTW("TOF data ready timeout\n");
            VL53L1_StopMeasurement(dev);
            VL53L1_StartMeasurement(dev);
            return range_last_valid; 
        }
        if (status != VL53L1_ERROR_NONE && status != VL53L1_ERROR_TIME_OUT) {
            DEBUG_PRINTW("TOF get data ready error: %d\n", status);
            VL53L1_StopMeasurement(dev);
            VL53L1_StartMeasurement(dev);
            return range_last_valid; 
        }
        vTaskDelay(M2T(1));
    }
    status = VL53L1_GetRangingMeasurementData(dev, &rangingData);
    if (status != VL53L1_ERROR_NONE) {
        DEBUG_PRINTW("TOF get ranging data error: %d\n", status);
        VL53L1_StopMeasurement(dev);
        VL53L1_StartMeasurement(dev);
        return range_last_valid; 
    }
    if (rangingData.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID ||
        rangingData.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED ||
        rangingData.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL ||
        rangingData.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE) {
        range = rangingData.RangeMilliMeter;
        if (range > 0 && range < RANGE_OUTLIER_LIMIT) {
            range_last_valid = range;
        }
    } else {
        DEBUG_PRINTW("TOF invalid range status: %d\n", rangingData.RangeStatus);
        range = range_last_valid; 
    }
    VL53L1_StopMeasurement(dev);
    status = VL53L1_StartMeasurement(dev);
    if (status != VL53L1_ERROR_NONE) {
        DEBUG_PRINTW("TOF start measurement error: %d\n", status);
    }
    return range;
}
void zRanger2Init(void)
{
  if (isInit)
    return;
  if (vl53l1xInit(&dev, I2C1_DEV))
  {
    DEBUG_PRINTI("Z-down sensor [OK]\n");
  }
  else
  {
    DEBUG_PRINTW("Z-down sensor [FAIL]\n");
    return;
  }
  xTaskCreate(zRanger2Task, ZRANGER2_TASK_NAME, ZRANGER2_TASK_STACKSIZE, NULL, ZRANGER2_TASK_PRI, NULL);
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);
  isInit = true;
}
bool zRanger2Test(void)
{
  if (!isInit)
    return false;
  return true;
}
void zRanger2Task(void* arg)
{
  TickType_t lastWakeTime;
  static uint32_t measurement_count = 0;
  const uint32_t LOG_INTERVAL = 40; 
  systemWaitStart();
  VL53L1_StopMeasurement(&dev);
  VL53L1_SetDistanceMode(&dev, VL53L1_DISTANCEMODE_MEDIUM);
  VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, 25000);
  VL53L1_StartMeasurement(&dev);
  lastWakeTime = xTaskGetTickCount();
  DEBUG_PRINTI("TOF sensor task started, measurement rate: 40Hz\n");
  while (1) {
    vTaskDelayUntil(&lastWakeTime, M2T(25));
    range_last = zRanger2GetMeasurementAndRestart(&dev);
    rangeSet(rangeDown, range_last / 1000.0f);
    measurement_count++;
    if (measurement_count % LOG_INTERVAL == 0) {
      float distance_m = range_last / 1000.0f;
      DEBUG_PRINTI("TOF: %.3f m (%.1f mm), valid: %d\n", 
                   distance_m, (float)range_last, 
                   (range_last > 0 && range_last < RANGE_OUTLIER_LIMIT) ? 1 : 0);
    }
    if (range_last > 0 && range_last < RANGE_OUTLIER_LIMIT) {
      float distance = (float)range_last * 0.001f; 
      float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
      rangeEnqueueDownRangeInEstimator(distance, stdDev, xTaskGetTickCount());
    }
  }
}
static uint8_t disable = 0;
#define PARAM_CORE (1<<5)
#define PARAM_PERSISTENT (1 << 8)
#define PARAM_ADD_CORE(TYPE, NAME, ADDRESS) \
  PARAM_ADD(TYPE | PARAM_CORE, NAME, ADDRESS)
PARAM_GROUP_START(deck)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger2, &isInit)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcACS37800, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcActiveMarker, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcAI, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcBigQuad, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcCPPM, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, cpxOverUART2, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlapperDeck, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcGTGPS, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLedRing, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLhTester, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLighthouse4, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLoadcell, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcDWM1000, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLoco, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcMultiranger, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcOA, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcServo, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcUSD, &disable)
PARAM_GROUP_STOP(deck)
static uint32_t effect = 0;
static uint32_t neffect = 0;
PARAM_GROUP_START(ring)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_PERSISTENT, effect, &effect)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect)
PARAM_GROUP_STOP(ring)
