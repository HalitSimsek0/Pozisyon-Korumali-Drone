#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pmw3901.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "sleepus.h"
#include "config.h"
#include "stabilizer_types.h"
#include "estimator.h"
#include "cf_math.h"
#define DEBUG_MODULE "FLOW"
#include "debug_cf.h"
#define AVERAGE_HISTORY_LENGTH 4
#define OULIER_LIMIT 100
#define LP_CONSTANT 0.8f
#if defined(USE_MA_SMOOTHING)
static struct {
    float32_t averageX[AVERAGE_HISTORY_LENGTH];
    float32_t averageY[AVERAGE_HISTORY_LENGTH];
    size_t ptr;
} pixelAverages;
#endif
float dpixelx_previous = 0;
float dpixely_previous = 0;
static uint8_t outlierCount = 0;
static float stdFlow = 2.0f;
static bool isInit1 = false;
static bool isInit2 = false;
motionBurst_t currentMotion;
static bool useFlowDisabled = false;
static bool useAdaptiveStd = true;
static float flowStdFixed = 2.0f;
#define NCS_PIN CONFIG_SPI_PIN_CS0
static void flowdeckTask(void *param)
{
    systemWaitStart();
    initUsecTimer();
    uint64_t lastTime  = usecTimestamp();
    static uint32_t measurement_count = 0;
    static uint32_t error_count = 0;
    const uint32_t LOG_INTERVAL = 100; 
    static motionBurst_t lastValidMotion = {0}; 
    while (1) {
#if CONFIG_FREERTOS_UNICORE
        vTaskDelay(10);
#else
        vTaskDelay(5);
#endif
        uint64_t spiStartTime = usecTimestamp();
        bool readSuccess = pmw3901ReadMotion(NCS_PIN, &currentMotion);
        uint64_t spiEndTime = usecTimestamp();
        uint32_t spiTransactionTime_us = (uint32_t)(spiEndTime - spiStartTime);
        measurement_count++;
        if (!readSuccess) {
            error_count++;
            if (lastValidMotion.motion == 0xB0) {
                currentMotion = lastValidMotion;
            } else {
                if (error_count % 50 == 0) { 
                    DEBUG_PRINTW("PMW3901: Read failed, skipping measurement (errors: %lu)\n", 
                                 (unsigned long)error_count);
                }
                continue;
            }
        } else {
            if (currentMotion.motion == 0xB0) {
                lastValidMotion = currentMotion;
            }
        }
        static uint32_t slowTransactionCount = 0;
        if (spiTransactionTime_us > 10000) { 
          slowTransactionCount++;
          if (slowTransactionCount % 100 == 0) { 
            DEBUG_PRINT("WARNING: SPI transaction slow: %lu us (count: %lu)\n", 
                        (unsigned long)spiTransactionTime_us, (unsigned long)slowTransactionCount);
          }
        }
        if (measurement_count % LOG_INTERVAL == 0) {
            DEBUG_PRINTI("PMW3901: dx=%d dy=%d squal=%d shutter=%d errors=%lu\n",
                         currentMotion.deltaX, currentMotion.deltaY,
                         currentMotion.squal, currentMotion.shutter,
                         (unsigned long)error_count);
        }
        int16_t accpx = -currentMotion.deltaY;
        int16_t accpy = -currentMotion.deltaX;
        if (abs(accpx) < OULIER_LIMIT && abs(accpy) < OULIER_LIMIT) {
        uint8_t squal = currentMotion.squal;
        uint16_t shutter = currentMotion.shutter;
        bool groundQualityGood = (squal >= 50) && (shutter < 2000);
        if (!groundQualityGood) {
            outlierCount++;
            continue; 
        }
        if (useAdaptiveStd)
        {
        float shutter_f = (float)shutter;
        stdFlow=0.0007984f *shutter_f + 0.4335f;
        if (stdFlow < 0.1f) stdFlow=0.1f;
        } else {
        stdFlow = flowStdFixed;
        }
            flowMeasurement_t flowData;
            flowData.stdDevX = stdFlow;    
            flowData.stdDevY = stdFlow;    
#if defined(USE_MA_SMOOTHING)
            pixelAverages.averageX[pixelAverages.ptr] = (float32_t)accpx;
            pixelAverages.averageY[pixelAverages.ptr] = (float32_t)accpy;
            float32_t meanX;
            float32_t meanY;
            xtensa_mean_f32(pixelAverages.averageX, AVERAGE_HISTORY_LENGTH, &meanX);
            xtensa_mean_f32(pixelAverages.averageY, AVERAGE_HISTORY_LENGTH, &meanY);
            pixelAverages.ptr = (pixelAverages.ptr + 1) % AVERAGE_HISTORY_LENGTH;
            flowData.dpixelx = (float)meanX;   
            flowData.dpixely = (float)meanY;   
#elif defined(USE_LP_FILTER)
            flowData.dpixelx = LP_CONSTANT * dpixelx_previous + (1.0f - LP_CONSTANT) * (float)accpx;
            flowData.dpixely = LP_CONSTANT * dpixely_previous + (1.0f - LP_CONSTANT) * (float)accpy;
            dpixelx_previous = flowData.dpixelx;
            dpixely_previous = flowData.dpixely;
#else
            flowData.dpixelx = (float)accpx;
            flowData.dpixely = (float)accpy;
#endif
            if (!useFlowDisabled && currentMotion.motion == 0xB0) {
                uint64_t currentTime = usecTimestamp();
                flowData.dt = (float)(currentTime - lastTime) / 1000000.0f;
                if (flowData.dt < 0.001f) {
                  flowData.dt = 0.001f; 
                } else if (flowData.dt > 0.05f) {
                  lastTime = currentTime;
                  continue;
                }
                lastTime = currentTime;
                estimatorEnqueueFlow(&flowData);
            }
        } else {
            outlierCount++;
        }
    }
}
void flowdeck2Init()
{
	if (isInit1 || isInit2) {
        return;
    }
    if (pmw3901Init(NCS_PIN)) {
        xTaskCreate(flowdeckTask, FLOW_TASK_NAME, FLOW_TASK_STACKSIZE, NULL, FLOW_TASK_PRI, NULL);
        isInit2 = true;
    }
}
bool flowdeck2Test()
{
    if (!isInit2) {
        DEBUG_PRINTD("Error while initializing the PMW3901 sensor\n");
    }
    return isInit2;
}
LOG_GROUP_START(motion)
LOG_ADD(LOG_UINT8, motion, &currentMotion.motion)
LOG_ADD(LOG_INT16, deltaX, &currentMotion.deltaX)
LOG_ADD(LOG_INT16, deltaY, &currentMotion.deltaY)
LOG_ADD(LOG_UINT16, shutter, &currentMotion.shutter)
LOG_ADD(LOG_UINT8, maxRaw, &currentMotion.maxRawData)
LOG_ADD(LOG_UINT8, minRaw, &currentMotion.minRawData)
LOG_ADD(LOG_UINT8, Rawsum, &currentMotion.rawDataSum)
LOG_ADD(LOG_UINT8, outlierCount, &outlierCount)
LOG_ADD(LOG_UINT8, squal, &currentMotion.squal)
LOG_ADD(LOG_FLOAT, std, &stdFlow)
LOG_GROUP_STOP(motion)
PARAM_GROUP_START(motion)
PARAM_ADD(PARAM_UINT8, disable, &useFlowDisabled)
PARAM_ADD(PARAM_UINT8, adaptive, &useAdaptiveStd)
PARAM_ADD(PARAM_FLOAT, flowStdFixed, &flowStdFixed)
PARAM_GROUP_STOP(motion)
PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcFlow, &isInit1)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcFlow2, &isInit2)
PARAM_GROUP_STOP(deck)