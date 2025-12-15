#include "FreeRTOS.h"
#include "queue.h"
#include "stabilizer.h"
#include "estimator_complementary.h"
#include "sensfusion6.h"
#include "position_estimator.h"
#include "sensors.h"
#include "stabilizer_types.h"
#include "static_mem.h"
#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT 1.0/ATTITUDE_UPDATE_RATE
#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT 1.0/POS_UPDATE_RATE
static bool latestTofMeasurement(tofMeasurement_t* tofMeasurement);
#define TOF_QUEUE_LENGTH (1)
static xQueueHandle tofDataQueue;
STATIC_MEM_QUEUE_ALLOC(tofDataQueue, TOF_QUEUE_LENGTH, sizeof(tofMeasurement_t));
void estimatorComplementaryInit(void)
{
  tofDataQueue = STATIC_MEM_QUEUE_CREATE(tofDataQueue);
  sensfusion6Init();
}
bool estimatorComplementaryTest(void)
{
  bool pass = true;
  pass &= sensfusion6Test();
  return pass;
}
void estimatorComplementary(state_t *state, sensorData_t *sensorData, control_t *control, const uint32_t tick)
{
  sensorsAcquire(sensorData, tick); 
  if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) {
#ifdef SENSORS_ENABLE_MAG_AK8963
    // Use magnetometer if available for better yaw estimation
    if (sensorsReadMag(&sensorData->mag)) {
      sensfusion6UpdateQWithMag(sensorData->gyro.x, sensorData->gyro.y, sensorData->gyro.z,
                                sensorData->acc.x, sensorData->acc.y, sensorData->acc.z,
                                sensorData->mag.x, sensorData->mag.y, sensorData->mag.z,
                                ATTITUDE_UPDATE_DT);
    } else {
      sensfusion6UpdateQ(sensorData->gyro.x, sensorData->gyro.y, sensorData->gyro.z,
                         sensorData->acc.x, sensorData->acc.y, sensorData->acc.z,
                         ATTITUDE_UPDATE_DT);
    }
#else
    sensfusion6UpdateQ(sensorData->gyro.x, sensorData->gyro.y, sensorData->gyro.z,
                       sensorData->acc.x, sensorData->acc.y, sensorData->acc.z,
                       ATTITUDE_UPDATE_DT);
#endif
    sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);
    sensfusion6GetQuaternion(
      &state->attitudeQuaternion.x,
      &state->attitudeQuaternion.y,
      &state->attitudeQuaternion.z,
      &state->attitudeQuaternion.w);
    state->acc.z = sensfusion6GetAccZWithoutGravity(sensorData->acc.x,
                                                    sensorData->acc.y,
                                                    sensorData->acc.z);
    positionUpdateVelocity(state->acc.z, ATTITUDE_UPDATE_DT);
  }
  if (RATE_DO_EXECUTE(POS_UPDATE_RATE, tick)) {
    tofMeasurement_t tofMeasurement;
    latestTofMeasurement(&tofMeasurement);
    positionEstimate(state, sensorData, &tofMeasurement, POS_UPDATE_DT, tick);
  }
}
static bool latestTofMeasurement(tofMeasurement_t* tofMeasurement) {
  return xQueuePeek(tofDataQueue, tofMeasurement, 0) == pdTRUE;
}
static bool overwriteMeasurement(xQueueHandle queue, void *measurement)
{
  portBASE_TYPE result;
  bool isInInterrupt = false;
  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueOverwriteFromISR(queue, measurement, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE)
    {
      portYIELD();
    }
  } else {
    result = xQueueOverwrite(queue, measurement);
  }
  return (result==pdTRUE);
}
bool estimatorComplementaryEnqueueTOF(const tofMeasurement_t *tof)
{
  return overwriteMeasurement(tofDataQueue, (void *)tof);
}