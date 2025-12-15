#ifndef __ESTIMATOR_H__
#define __ESTIMATOR_H__
#include "stabilizer_types.h"
typedef enum {
  anyEstimator = 0,
  complementaryEstimator,
  kalmanEstimator,
  StateEstimatorTypeCount,
} StateEstimatorType;
bool registerRequiredEstimator(StateEstimatorType estimator);
StateEstimatorType deckGetRequiredEstimator();
void stateEstimatorInit(StateEstimatorType estimator);
bool stateEstimatorTest(void);
void stateEstimatorSwitchTo(StateEstimatorType estimator);
void stateEstimator(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);
StateEstimatorType getStateEstimator(void);
const char* stateEstimatorGetName();
// Removed: estimatorEnqueueTDOA() - TDOA hardware not available
bool estimatorEnqueuePosition(const positionMeasurement_t *pos);
bool estimatorEnqueuePose(const poseMeasurement_t *pose);
bool estimatorEnqueueDistance(const distanceMeasurement_t *dist);
bool estimatorEnqueueTOF(const tofMeasurement_t *tof);
bool estimatorEnqueueAbsoluteHeight(const heightMeasurement_t *height);
bool estimatorEnqueueFlow(const flowMeasurement_t *flow);
bool estimatorEnqueueYawError(const yawErrorMeasurement_t *error);
#endif 