#ifndef __KALMAN_CORE_H__
#define __KALMAN_CORE_H__
#include "cf_math.h"
#include "stabilizer_types.h"
typedef enum
{
  KC_STATE_X, KC_STATE_Y, KC_STATE_Z, KC_STATE_PX, KC_STATE_PY, KC_STATE_PZ, KC_STATE_D0, KC_STATE_D1, KC_STATE_D2, KC_STATE_DIM
} kalmanCoreStateIdx_t;
typedef struct {
  float S[KC_STATE_DIM];
  float q[4];
  float R[3][3];
  __attribute__((aligned(4))) float P[KC_STATE_DIM][KC_STATE_DIM];
  xtensa_matrix_instance_f32 Pm;
  bool resetEstimation;
  // Removed: baroReferenceHeight - Barometer hardware not available
} kalmanCoreData_t;
void kalmanCoreInit(kalmanCoreData_t* this);
// Removed: kalmanCoreUpdateWithBaro - Barometer hardware not available
void kalmanCoreUpdateWithAbsoluteHeight(kalmanCoreData_t* this, heightMeasurement_t* height);
void kalmanCoreUpdateWithPosition(kalmanCoreData_t* this, positionMeasurement_t *xyz);
void kalmanCoreUpdateWithPose(kalmanCoreData_t* this, poseMeasurement_t *pose);
void kalmanCoreUpdateWithDistance(kalmanCoreData_t* this, distanceMeasurement_t *d);
// Removed: kalmanCoreUpdateWithTDOA() - TDOA hardware not available
void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this, const flowMeasurement_t *flow, const Axis3f *gyro);
void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof);
void kalmanCoreUpdateWithYawError(kalmanCoreData_t *this, yawErrorMeasurement_t *error);
void kalmanCorePredict(kalmanCoreData_t* this, float thrust, Axis3f *acc, Axis3f *gyro, float dt, bool quadIsFlying);
void kalmanCoreAddProcessNoise(kalmanCoreData_t* this, float dt);
void kalmanCoreFinalize(kalmanCoreData_t* this, uint32_t tick);
void kalmanCoreExternalizeState(const kalmanCoreData_t* this, state_t *state, const Axis3f *acc, uint32_t tick);
void kalmanCoreDecoupleXY(kalmanCoreData_t* this);
#endif 