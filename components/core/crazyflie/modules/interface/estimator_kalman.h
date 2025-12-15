#ifndef __ESTIMATOR_KALMAN_H__
#define __ESTIMATOR_KALMAN_H__
#include <stdint.h>
#include "stabilizer_types.h"
void estimatorKalmanInit(void);
bool estimatorKalmanTest(void);
void estimatorKalman(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);
void estimatorKalmanTaskInit();
bool estimatorKalmanTaskTest();
// Removed: estimatorKalmanEnqueueTDOA() - TDOA hardware not available
bool estimatorKalmanEnqueuePosition(const positionMeasurement_t *pos);
bool estimatorKalmanEnqueuePose(const poseMeasurement_t *pose);
bool estimatorKalmanEnqueueDistance(const distanceMeasurement_t *dist);
bool estimatorKalmanEnqueueTOF(const tofMeasurement_t *tof);
bool estimatorKalmanEnqueueAbsoluteHeight(const heightMeasurement_t *height);
bool estimatorKalmanEnqueueFlow(const flowMeasurement_t *flow);
bool estimatorKalmanEnqueueYawError(const yawErrorMeasurement_t* error);
void estimatorKalmanGetEstimatedPos(point_t* pos);
void estimatorKalmanGetEstimatedRot(float * rotationMatrix);
#endif 