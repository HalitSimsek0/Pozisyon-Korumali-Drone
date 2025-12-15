#include <inttypes.h>
#include "kalman_core.h"
#include "estimator_kalman.h"
#include "kalman_supervisor.h"
#include "stm32_legacy.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "sensors.h"
#include "static_mem.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "physicalConstants.h"
#include "statsCnt.h"
#include "rateSupervisor.h"
#include "config.h"
#include "usec_time.h"
#include "cf_math.h"
#include "estimator.h"
#include <math.h>
#define DEBUG_MODULE "ESTKALMAN"
#include "debug_cf.h"
static xQueueHandle distDataQueue;
STATIC_MEM_QUEUE_ALLOC(distDataQueue, 10, sizeof(distanceMeasurement_t));
static inline bool stateEstimatorHasDistanceMeasurement(distanceMeasurement_t *dist) {
  return (pdTRUE == xQueueReceive(distDataQueue, dist, 0));
}
static xQueueHandle posDataQueue;
STATIC_MEM_QUEUE_ALLOC(posDataQueue, 10, sizeof(positionMeasurement_t));
static inline bool stateEstimatorHasPositionMeasurement(positionMeasurement_t *pos) {
  return (pdTRUE == xQueueReceive(posDataQueue, pos, 0));
}
static xQueueHandle poseDataQueue;
STATIC_MEM_QUEUE_ALLOC(poseDataQueue, 10, sizeof(poseMeasurement_t));
static inline bool stateEstimatorHasPoseMeasurement(poseMeasurement_t *pose) {
  return (pdTRUE == xQueueReceive(poseDataQueue, pose, 0));
}
// Removed: TDOA data queue - TDOA hardware (UWB/Loco deck) not available
static xQueueHandle flowDataQueue;
STATIC_MEM_QUEUE_ALLOC(flowDataQueue, 10, sizeof(flowMeasurement_t));
static inline bool stateEstimatorHasFlowPacket(flowMeasurement_t *flow) {
  return (pdTRUE == xQueueReceive(flowDataQueue, flow, 0));
}
static xQueueHandle tofDataQueue;
STATIC_MEM_QUEUE_ALLOC(tofDataQueue, 10, sizeof(tofMeasurement_t));
static inline bool stateEstimatorHasTOFPacket(tofMeasurement_t *tof) {
  return (pdTRUE == xQueueReceive(tofDataQueue, tof, 0));
}
static xQueueHandle heightDataQueue;
STATIC_MEM_QUEUE_ALLOC(heightDataQueue, 10, sizeof(heightMeasurement_t));
static inline bool stateEstimatorHasHeightPacket(heightMeasurement_t *height) {
  return (pdTRUE == xQueueReceive(heightDataQueue, height, 0));
}
static xQueueHandle yawErrorDataQueue;
STATIC_MEM_QUEUE_ALLOC(yawErrorDataQueue, 10, sizeof(yawErrorMeasurement_t));
static inline bool stateEstimatorHasYawErrorPacket(yawErrorMeasurement_t *error)
{
  return (pdTRUE == xQueueReceive(yawErrorDataQueue, error, 0));
}
static SemaphoreHandle_t runTaskSemaphore;
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;
#define CONTROL_TO_ACC (GRAVITY_MAGNITUDE*60.0f/(CF_MASS*1000.0f)/65536.0f)
#define PREDICT_RATE RATE_100_HZ 
// Removed: BARO_RATE - Barometer hardware not available
#define MAG_RATE RATE_10_HZ
#define RATE_10_HZ 10
#define IN_FLIGHT_THRUST_THRESHOLD (GRAVITY_MAGNITUDE*0.1f)
#define IN_FLIGHT_TIME_THRESHOLD (500)
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)
NO_DMA_CCM_SAFE_ZERO_INIT static kalmanCoreData_t coreData;
static bool isInit = false;
static Axis3f accAccumulator;
static float thrustAccumulator;
static Axis3f gyroAccumulator;
// Removed: baroAslAccumulator - Barometer hardware not available
#ifdef SENSORS_ENABLE_MAG_AK8963
static Axis3f magAccumulator;
static uint32_t magAccumulatorCount;
#endif
static uint32_t accAccumulatorCount;
static uint32_t thrustAccumulatorCount;
static uint32_t gyroAccumulatorCount;
// Removed: baroAccumulatorCount - Barometer hardware not available
static bool quadIsFlying = false;
static uint32_t lastFlightCmd;
static uint32_t takeoffTime;
static state_t taskEstimatorState; 
static Axis3f gyroSnapshot; 
static Axis3f accSnapshot; 
#define ONE_SECOND 1000
static STATS_CNT_RATE_DEFINE(updateCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(predictionCounter, ONE_SECOND);
// Removed: baroUpdateCounter - Barometer hardware not available
static STATS_CNT_RATE_DEFINE(finalizeCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(measurementAppendedCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(measurementNotAppendedCounter, ONE_SECOND);
static rateSupervisor_t rateSupervisorContext;
#define WARNING_HOLD_BACK_TIME M2T(2000)
static uint32_t warningBlockTime = 0;
// Removed: useBaroUpdate - Barometer hardware not available
static void kalmanTask(void* parameters);
static bool predictStateForward(uint32_t osTick, float dt);
#ifdef SENSORS_ENABLE_MAG_AK8963
static void processMagnetometerYawError(const Axis3f *mag, const kalmanCoreData_t *coreData, uint32_t tick);
#endif
static bool updateQueuedMeasurments(const Axis3f *gyro, const uint32_t tick);
STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(kalmanTask, KALMAN_TASK_STACKSIZE);
void estimatorKalmanTaskInit() {
  distDataQueue = STATIC_MEM_QUEUE_CREATE(distDataQueue);
  posDataQueue = STATIC_MEM_QUEUE_CREATE(posDataQueue);
  poseDataQueue = STATIC_MEM_QUEUE_CREATE(poseDataQueue);
  // Removed: tdoaDataQueue creation - TDOA hardware not available
  flowDataQueue = STATIC_MEM_QUEUE_CREATE(flowDataQueue);
  tofDataQueue = STATIC_MEM_QUEUE_CREATE(tofDataQueue);
  heightDataQueue = STATIC_MEM_QUEUE_CREATE(heightDataQueue);
  yawErrorDataQueue = STATIC_MEM_QUEUE_CREATE(yawErrorDataQueue);
  vSemaphoreCreateBinary(runTaskSemaphore);
  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);
  STATIC_MEM_TASK_CREATE(kalmanTask, kalmanTask, KALMAN_TASK_NAME, NULL, KALMAN_TASK_PRI);
  isInit = true;
}
bool estimatorKalmanTaskTest() {
  return isInit;
}
static void kalmanTask(void* parameters) {
  systemWaitStart();
  uint64_t lastPrediction_us = usecTimestamp();
  uint32_t nextPrediction = xTaskGetTickCount();
  uint64_t lastPNUpdate_us = usecTimestamp();
  // Removed: nextBaroUpdate - Barometer hardware not available
#ifdef SENSORS_ENABLE_MAG_AK8963
  uint32_t nextMagUpdate = xTaskGetTickCount();
#endif
  rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), M2T(1000), 99, 101, 1);
  while (true) {
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
    if (coreData.resetEstimation) {
      estimatorKalmanInit();
      paramSetInt(paramGetVarId("kalman", "resetEstimation"), 0);
    }
    bool doneUpdate = false;
    uint32_t osTick = xTaskGetTickCount(); 
  #ifdef KALMAN_DECOUPLE_XY
    kalmanCoreDecoupleXY(&coreData);
  #endif
    if (osTick >= nextPrediction) { 
      uint64_t currentTime_us = usecTimestamp();
      float dt = (float)(currentTime_us - lastPrediction_us) / 1000000.0f;
      if (dt < 0.001f) {
        dt = 0.001f; 
      } else if (dt > 0.02f) {
        dt = 0.02f; 
      }
      if (predictStateForward(osTick, dt)) {
        lastPrediction_us = currentTime_us;
        doneUpdate = true;
        STATS_CNT_RATE_EVENT(&predictionCounter);
      }
      nextPrediction = osTick + S2T(1.0f / PREDICT_RATE);
      if (!rateSupervisorValidate(&rateSupervisorContext, T2M(osTick))) {
        DEBUG_PRINT("WARNING: Kalman prediction rate low (%"PRIu32")\n", rateSupervisorLatestCount(&rateSupervisorContext));
      }
    }
    {
      uint64_t currentTime_us = usecTimestamp();
      float dt = (float)(currentTime_us - lastPNUpdate_us) / 1000000.0f;
      if (dt > 0.0f) {
        kalmanCoreAddProcessNoise(&coreData, dt);
        lastPNUpdate_us = currentTime_us;
      }
    }
    // Removed: barometer update logic - Barometer hardware not available
#ifdef SENSORS_ENABLE_MAG_AK8963
    if (osTick >= nextMagUpdate && magAccumulatorCount > 0) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      Axis3f magAverage;
      magAverage.x = magAccumulator.x / magAccumulatorCount;
      magAverage.y = magAccumulator.y / magAccumulatorCount;
      magAverage.z = magAccumulator.z / magAccumulatorCount;
      magAccumulator = (Axis3f){.axis={0}};
      magAccumulatorCount = 0;
      xSemaphoreGive(dataMutex);
      processMagnetometerYawError(&magAverage, &coreData, osTick);
      nextMagUpdate = osTick + S2T(1.0f / MAG_RATE);
      doneUpdate = true;
    }
#endif
    {
      Axis3f gyro;
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      memcpy(&gyro, &gyroSnapshot, sizeof(gyro));
      xSemaphoreGive(dataMutex);
      if(updateQueuedMeasurments(&gyro, osTick)) {
        doneUpdate = true;
      }
    }
    if (doneUpdate)
    {
      kalmanCoreFinalize(&coreData, osTick);
      STATS_CNT_RATE_EVENT(&finalizeCounter);
      if (! kalmanSupervisorIsStateWithinBounds(&coreData)) {
        coreData.resetEstimation = true;
        if (osTick > warningBlockTime) {
          warningBlockTime = osTick + WARNING_HOLD_BACK_TIME;
          DEBUG_PRINT("State out of bounds, resetting\n");
        }
      }
    }
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    kalmanCoreExternalizeState(&coreData, &taskEstimatorState, &accSnapshot, osTick);
    xSemaphoreGive(dataMutex);
    STATS_CNT_RATE_EVENT(&updateCounter);
  }
}
void estimatorKalman(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick)
{
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  #define MAX_ACCUMULATOR_COUNT 1000  
  if (sensorsReadAcc(&sensors->acc)) {
    if (accAccumulatorCount < MAX_ACCUMULATOR_COUNT) {
    accAccumulator.x += sensors->acc.x;
    accAccumulator.y += sensors->acc.y;
    accAccumulator.z += sensors->acc.z;
    accAccumulatorCount++;
    }
  }
  if (sensorsReadGyro(&sensors->gyro)) {
    if (gyroAccumulatorCount < MAX_ACCUMULATOR_COUNT) {
    gyroAccumulator.x += sensors->gyro.x;
    gyroAccumulator.y += sensors->gyro.y;
    gyroAccumulator.z += sensors->gyro.z;
    gyroAccumulatorCount++;
    }
  }
  if (thrustAccumulatorCount < MAX_ACCUMULATOR_COUNT) {
  thrustAccumulator += control->thrust;
  thrustAccumulatorCount++;
  }
#ifdef SENSORS_ENABLE_MAG_AK8963
  if (sensorsReadMag(&sensors->mag)) {
    if (magAccumulatorCount < MAX_ACCUMULATOR_COUNT) {
      magAccumulator.x += sensors->mag.x;
      magAccumulator.y += sensors->mag.y;
      magAccumulator.z += sensors->mag.z;
      magAccumulatorCount++;
    }
  }
#endif
  // Removed: barometer accumulator update - Barometer hardware not available
  memcpy(&gyroSnapshot, &sensors->gyro, sizeof(gyroSnapshot));
  memcpy(&accSnapshot, &sensors->acc, sizeof(accSnapshot));
  memcpy(state, &taskEstimatorState, sizeof(state_t));
  xSemaphoreGive(dataMutex);
  xSemaphoreGive(runTaskSemaphore);
}
static bool predictStateForward(uint32_t osTick, float dt) {
  if (gyroAccumulatorCount == 0
      || accAccumulatorCount == 0
      || thrustAccumulatorCount == 0)
  {
    return false;
  }
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  Axis3f gyroAverage;
  gyroAverage.x = gyroAccumulator.x * DEG_TO_RAD / gyroAccumulatorCount;
  gyroAverage.y = gyroAccumulator.y * DEG_TO_RAD / gyroAccumulatorCount;
  gyroAverage.z = gyroAccumulator.z * DEG_TO_RAD / gyroAccumulatorCount;
  Axis3f accAverage;
  accAverage.x = accAccumulator.x * GRAVITY_MAGNITUDE / accAccumulatorCount;
  accAverage.y = accAccumulator.y * GRAVITY_MAGNITUDE / accAccumulatorCount;
  accAverage.z = accAccumulator.z * GRAVITY_MAGNITUDE / accAccumulatorCount;
  float thrustAverage = thrustAccumulator * CONTROL_TO_ACC / thrustAccumulatorCount;
  accAccumulator = (Axis3f){.axis={0}};
  accAccumulatorCount = 0;
  gyroAccumulator = (Axis3f){.axis={0}};
  gyroAccumulatorCount = 0;
  thrustAccumulator = 0;
  thrustAccumulatorCount = 0;
  xSemaphoreGive(dataMutex);
  if (thrustAverage > IN_FLIGHT_THRUST_THRESHOLD) {
    lastFlightCmd = osTick;
    if (!quadIsFlying) {
      takeoffTime = lastFlightCmd;
    }
  }
  quadIsFlying = (osTick-lastFlightCmd) < IN_FLIGHT_TIME_THRESHOLD;
  kalmanCorePredict(&coreData, thrustAverage, &accAverage, &gyroAverage, dt, quadIsFlying);
  return true;
}
#ifdef SENSORS_ENABLE_MAG_AK8963
static void processMagnetometerYawError(const Axis3f *mag, const kalmanCoreData_t *coreData, uint32_t tick) {
  // Calculate yaw from magnetometer using rotation matrix
  // Rotate magnetometer vector from body frame to earth frame using R matrix
  float mx = mag->x;
  float my = mag->y;
  float mz = mag->z;
  
  // Use rotation matrix R to rotate magnetometer from body to earth frame
  // mag_earth = R * mag_body
  float magX_earth = coreData->R[0][0] * mx + coreData->R[0][1] * my + coreData->R[0][2] * mz;
  float magY_earth = coreData->R[1][0] * mx + coreData->R[1][1] * my + coreData->R[1][2] * mz;
  float magZ_earth = coreData->R[2][0] * mx + coreData->R[2][1] * my + coreData->R[2][2] * mz;
  
  // Calculate heading from magnetometer (atan2 of horizontal components)
  float magHeading = atan2f(magY_earth, magX_earth) * RAD_TO_DEG;
  
  // Calculate current yaw from quaternion
  float qw = coreData->q[0];
  float qx = coreData->q[1];
  float qy = coreData->q[2];
  float qz = coreData->q[3];
  float currentYaw = atan2f(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz) * RAD_TO_DEG;
  
  // Calculate yaw error (normalize to -180 to 180 degrees)
  float yawError = magHeading - currentYaw;
  if (yawError > 180.0f) yawError -= 360.0f;
  if (yawError < -180.0f) yawError += 360.0f;
  
  // Standard deviation for magnetometer yaw (higher uncertainty)
  float magYawStdDev = 5.0f; // degrees - magnetometer has higher uncertainty
  
  // Only use magnetometer if magnitude is reasonable (not too weak or too strong)
  float magMagnitude = sqrtf(magX_earth*magX_earth + magY_earth*magY_earth + magZ_earth*magZ_earth);
  if (magMagnitude > 0.1f && magMagnitude < 2.0f && fabsf(yawError) < 90.0f) {
    yawErrorMeasurement_t yawErrorMeas;
    yawErrorMeas.timestamp = tick;
    yawErrorMeas.yawError = yawError * DEG_TO_RAD; // Convert to radians
    yawErrorMeas.stdDev = magYawStdDev * DEG_TO_RAD;
    estimatorKalmanEnqueueYawError(&yawErrorMeas);
  }
}
#endif
static bool updateQueuedMeasurments(const Axis3f *gyro, const uint32_t tick) {
  bool doneUpdate = false;
  tofMeasurement_t tof;
  while (stateEstimatorHasTOFPacket(&tof))
  {
    kalmanCoreUpdateWithTof(&coreData, &tof);
    doneUpdate = true;
  }
  yawErrorMeasurement_t yawError;
  while (stateEstimatorHasYawErrorPacket(&yawError))
  {
    kalmanCoreUpdateWithYawError(&coreData, &yawError);
    doneUpdate = true;
  }
  heightMeasurement_t height;
  while (stateEstimatorHasHeightPacket(&height))
  {
    kalmanCoreUpdateWithAbsoluteHeight(&coreData, &height);
    doneUpdate = true;
  }
  distanceMeasurement_t dist;
  while (stateEstimatorHasDistanceMeasurement(&dist))
  {
    kalmanCoreUpdateWithDistance(&coreData, &dist);
    doneUpdate = true;
  }
  positionMeasurement_t pos;
  while (stateEstimatorHasPositionMeasurement(&pos))
  {
    kalmanCoreUpdateWithPosition(&coreData, &pos);
    doneUpdate = true;
  }
  poseMeasurement_t pose;
  while (stateEstimatorHasPoseMeasurement(&pose))
  {
    kalmanCoreUpdateWithPose(&coreData, &pose);
    doneUpdate = true;
  }
  // Removed: TDOA packet processing - TDOA hardware not available
  flowMeasurement_t flow;
  while (stateEstimatorHasFlowPacket(&flow))
  {
    kalmanCoreUpdateWithFlow(&coreData, &flow, gyro);
    doneUpdate = true;
  }
  return doneUpdate;
}
void estimatorKalmanInit(void) {
  xQueueReset(distDataQueue);
  xQueueReset(posDataQueue);
  xQueueReset(poseDataQueue);
  // Removed: tdoaDataQueue reset - TDOA hardware not available
  xQueueReset(flowDataQueue);
  xQueueReset(tofDataQueue);
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  accAccumulator = (Axis3f){.axis={0}};
  gyroAccumulator = (Axis3f){.axis={0}};
  thrustAccumulator = 0;
  // Removed: barometer accumulator reset - Barometer hardware not available
#ifdef SENSORS_ENABLE_MAG_AK8963
  magAccumulator = (Axis3f){.axis={0}};
  magAccumulatorCount = 0;
#endif
  accAccumulatorCount = 0;
  gyroAccumulatorCount = 0;
  thrustAccumulatorCount = 0;
  xSemaphoreGive(dataMutex);
  kalmanCoreInit(&coreData);
}
static bool appendMeasurement(xQueueHandle queue, void *measurement)
{
  portBASE_TYPE result;
  bool isInInterrupt = 0; 
  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueSendFromISR(queue, measurement, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE)
    {
      portYIELD();
    }
  } else {
    result = xQueueSend(queue, measurement, 0);
  }
  if (result == pdTRUE) {
    STATS_CNT_RATE_EVENT(&measurementAppendedCounter);
    return true;
  } else {
    STATS_CNT_RATE_EVENT(&measurementNotAppendedCounter);
    return true;
  }
}
// Removed: estimatorKalmanEnqueueTDOA() - TDOA hardware not available
bool estimatorKalmanEnqueuePosition(const positionMeasurement_t *pos)
{
  ASSERT(isInit);
  return appendMeasurement(posDataQueue, (void *)pos);
}
bool estimatorKalmanEnqueuePose(const poseMeasurement_t *pose)
{
  ASSERT(isInit);
  return appendMeasurement(poseDataQueue, (void *)pose);
}
bool estimatorKalmanEnqueueDistance(const distanceMeasurement_t *dist)
{
  ASSERT(isInit);
  return appendMeasurement(distDataQueue, (void *)dist);
}
bool estimatorKalmanEnqueueFlow(const flowMeasurement_t *flow)
{
  ASSERT(isInit);
  return appendMeasurement(flowDataQueue, (void *)flow);
}
bool estimatorKalmanEnqueueTOF(const tofMeasurement_t *tof)
{
  ASSERT(isInit);
  return appendMeasurement(tofDataQueue, (void *)tof);
}
bool estimatorKalmanEnqueueAbsoluteHeight(const heightMeasurement_t *height)
{
  ASSERT(isInit);
  return appendMeasurement(heightDataQueue, (void *)height);
}
bool estimatorKalmanEnqueueYawError(const yawErrorMeasurement_t* error)
{
  ASSERT(isInit);
  return appendMeasurement(yawErrorDataQueue, (void *)error);
}
bool estimatorKalmanTest(void)
{
  return isInit;
}
void estimatorKalmanGetEstimatedPos(point_t* pos) {
  pos->x = coreData.S[KC_STATE_X];
  pos->y = coreData.S[KC_STATE_Y];
  pos->z = coreData.S[KC_STATE_Z];
}
void estimatorKalmanGetEstimatedRot(float * rotationMatrix) {
  memcpy(rotationMatrix, coreData.R, 9*sizeof(float));
}
LOG_GROUP_START(kalman_states)
  LOG_ADD(LOG_FLOAT, ox, &coreData.S[KC_STATE_X])
  LOG_ADD(LOG_FLOAT, oy, &coreData.S[KC_STATE_Y])
  LOG_ADD(LOG_FLOAT, vx, &coreData.S[KC_STATE_PX])
  LOG_ADD(LOG_FLOAT, vy, &coreData.S[KC_STATE_PY])
LOG_GROUP_STOP(kalman_states)
LOG_GROUP_START(kalman)
  LOG_ADD(LOG_UINT8, inFlight, &quadIsFlying)
  LOG_ADD(LOG_FLOAT, stateX, &coreData.S[KC_STATE_X])
  LOG_ADD(LOG_FLOAT, stateY, &coreData.S[KC_STATE_Y])
  LOG_ADD(LOG_FLOAT, stateZ, &coreData.S[KC_STATE_Z])
  LOG_ADD(LOG_FLOAT, statePX, &coreData.S[KC_STATE_PX])
  LOG_ADD(LOG_FLOAT, statePY, &coreData.S[KC_STATE_PY])
  LOG_ADD(LOG_FLOAT, statePZ, &coreData.S[KC_STATE_PZ])
  LOG_ADD(LOG_FLOAT, stateD0, &coreData.S[KC_STATE_D0])
  LOG_ADD(LOG_FLOAT, stateD1, &coreData.S[KC_STATE_D1])
  LOG_ADD(LOG_FLOAT, stateD2, &coreData.S[KC_STATE_D2])
  LOG_ADD(LOG_FLOAT, varX, &coreData.P[KC_STATE_X][KC_STATE_X])
  LOG_ADD(LOG_FLOAT, varY, &coreData.P[KC_STATE_Y][KC_STATE_Y])
  LOG_ADD(LOG_FLOAT, varZ, &coreData.P[KC_STATE_Z][KC_STATE_Z])
  LOG_ADD(LOG_FLOAT, varPX, &coreData.P[KC_STATE_PX][KC_STATE_PX])
  LOG_ADD(LOG_FLOAT, varPY, &coreData.P[KC_STATE_PY][KC_STATE_PY])
  LOG_ADD(LOG_FLOAT, varPZ, &coreData.P[KC_STATE_PZ][KC_STATE_PZ])
  LOG_ADD(LOG_FLOAT, varD0, &coreData.P[KC_STATE_D0][KC_STATE_D0])
  LOG_ADD(LOG_FLOAT, varD1, &coreData.P[KC_STATE_D1][KC_STATE_D1])
  LOG_ADD(LOG_FLOAT, varD2, &coreData.P[KC_STATE_D2][KC_STATE_D2])
  LOG_ADD(LOG_FLOAT, q0, &coreData.q[0])
  LOG_ADD(LOG_FLOAT, q1, &coreData.q[1])
  LOG_ADD(LOG_FLOAT, q2, &coreData.q[2])
  LOG_ADD(LOG_FLOAT, q3, &coreData.q[3])
  STATS_CNT_RATE_LOG_ADD(rtUpdate, &updateCounter)
  STATS_CNT_RATE_LOG_ADD(rtPred, &predictionCounter)
  // Removed: barometer update counter log - Barometer hardware not available
  STATS_CNT_RATE_LOG_ADD(rtFinal, &finalizeCounter)
  STATS_CNT_RATE_LOG_ADD(rtApnd, &measurementAppendedCounter)
  STATS_CNT_RATE_LOG_ADD(rtRej, &measurementNotAppendedCounter)
LOG_GROUP_STOP(kalman)
PARAM_GROUP_START(kalman)
  PARAM_ADD(PARAM_UINT8, resetEstimation, &coreData.resetEstimation)
  PARAM_ADD(PARAM_UINT8, quadIsFlying, &quadIsFlying)
PARAM_GROUP_STOP(kalman)