#include "kalman_core.h"
#include "cfassert.h"
#include "outlierFilter.h"
#include "physicalConstants.h"
#include "log.h"
#include "debug_cf.h"
#include "param.h"
#include "math3d.h"
#include "xtensa_math.h"
#include "static_mem.h"
#ifdef LPS_2D_POSITION_HEIGHT
#define ROLLPITCH_ZERO_REVERSION (0.0f)
#else
#define ROLLPITCH_ZERO_REVERSION (0.001f)
#endif
#ifdef DEBUG_STATE_CHECK
static void assertStateNotNaN(const kalmanCoreData_t* this) {
  if ((isnan(this->S[KC_STATE_X])) ||
      (isnan(this->S[KC_STATE_Y])) ||
      (isnan(this->S[KC_STATE_Z])) ||
      (isnan(this->S[KC_STATE_PX])) ||
      (isnan(this->S[KC_STATE_PY])) ||
      (isnan(this->S[KC_STATE_PZ])) ||
      (isnan(this->S[KC_STATE_D0])) ||
      (isnan(this->S[KC_STATE_D1])) ||
      (isnan(this->S[KC_STATE_D2])) ||
      (isnan(this->q[0])) ||
      (isnan(this->q[1])) ||
      (isnan(this->q[2])) ||
      (isnan(this->q[3])))
  {
    ASSERT(false);
  }
  for(int i=0; i<KC_STATE_DIM; i++) {
    for(int j=0; j<KC_STATE_DIM; j++)
    {
      if (isnan(this->P[i][j]))
      {
        ASSERT(false);
      }
    }
  }
}
#else
static void assertStateNotNaN(const kalmanCoreData_t* this)
{
  return;
}
#endif
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)
static const float stdDevInitialPosition_xy = 100;
static const float stdDevInitialPosition_z = 1;
static const float stdDevInitialVelocity = 0.01;
static const float stdDevInitialAttitude_rollpitch = 0.01;
static const float stdDevInitialAttitude_yaw = 0.01;
static float procNoiseAcc_xy = 0.5f;
static float procNoiseAcc_z = 1.0f;
static float procNoiseVel = 0;
static float procNoisePos = 0;
static float procNoiseAtt = 0;
// Removed: measNoiseBaro - Barometer hardware not available
static float measNoiseGyro_rollpitch = 0.1f; 
static float measNoiseGyro_yaw = 0.1f; 
static float initialX = 0.0;
static float initialY = 0.0;
static float initialZ = 0.0;
static float initialYaw = 0.0;
static float initialQuaternion[4] = {0.0, 0.0, 0.0, 0.0};
// Removed: tdoaCount - TDOA hardware not available
// Removed: sweepOutlierFilterState - Lighthouse hardware not available
void kalmanCoreInit(kalmanCoreData_t* this) {
  // Removed: tdoaCount initialization - TDOA hardware not available
  memset(this, 0, sizeof(kalmanCoreData_t));
  this->S[KC_STATE_X] = initialX;
  this->S[KC_STATE_Y] = initialY;
  this->S[KC_STATE_Z] = initialZ;
  initialQuaternion[0] = xtensa_cos_f32(initialYaw / 2);
  initialQuaternion[1] = 0.0;
  initialQuaternion[2] = 0.0;
  initialQuaternion[3] = xtensa_sin_f32(initialYaw / 2);
  for (int i = 0; i < 4; i++) { this->q[i] = initialQuaternion[i]; }
  for(int i=0; i<3; i++) { for(int j=0; j<3; j++) { this->R[i][j] = i==j ? 1 : 0; }}
  for (int i=0; i< KC_STATE_DIM; i++) {
    for (int j=0; j < KC_STATE_DIM; j++) {
      this->P[i][j] = 0; 
    }
  }
  this->P[KC_STATE_X][KC_STATE_X]  = powf(stdDevInitialPosition_xy, 2);
  this->P[KC_STATE_Y][KC_STATE_Y]  = powf(stdDevInitialPosition_xy, 2);
  this->P[KC_STATE_Z][KC_STATE_Z]  = powf(stdDevInitialPosition_z, 2);
  this->P[KC_STATE_PX][KC_STATE_PX] = powf(stdDevInitialVelocity, 2);
  this->P[KC_STATE_PY][KC_STATE_PY] = powf(stdDevInitialVelocity, 2);
  this->P[KC_STATE_PZ][KC_STATE_PZ] = powf(stdDevInitialVelocity, 2);
  this->P[KC_STATE_D0][KC_STATE_D0] = powf(stdDevInitialAttitude_rollpitch, 2);
  this->P[KC_STATE_D1][KC_STATE_D1] = powf(stdDevInitialAttitude_rollpitch, 2);
  this->P[KC_STATE_D2][KC_STATE_D2] = powf(stdDevInitialAttitude_yaw, 2);
  this->Pm.numRows = KC_STATE_DIM;
  this->Pm.numCols = KC_STATE_DIM;
  this->Pm.pData = (float*)this->P;
  // Removed: baroReferenceHeight - Barometer hardware not available
  // Removed: outlierFilterReset(&sweepOutlierFilterState, 0) - Lighthouse hardware not available
}
static void scalarUpdate(kalmanCoreData_t* this, xtensa_matrix_instance_f32 *Hm, float error, float stdMeasNoise)
{
  NO_DMA_CCM_SAFE_ZERO_INIT static float K[KC_STATE_DIM];
  static xtensa_matrix_instance_f32 Km = {KC_STATE_DIM, 1, (float *)K};
  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
  static xtensa_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};
  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
  static xtensa_matrix_instance_f32 tmpNN2m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};
  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN3d[KC_STATE_DIM * KC_STATE_DIM];
  static xtensa_matrix_instance_f32 tmpNN3m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN3d};
  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float HTd[KC_STATE_DIM * 1];
  static xtensa_matrix_instance_f32 HTm = {KC_STATE_DIM, 1, HTd};
  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float PHTd[KC_STATE_DIM * 1];
  static xtensa_matrix_instance_f32 PHTm = {KC_STATE_DIM, 1, PHTd};
  ASSERT(Hm->numRows == 1);
  ASSERT(Hm->numCols == KC_STATE_DIM);
  mat_trans(Hm, &HTm);
  mat_mult(&this->Pm, &HTm, &PHTm); 
  float R = stdMeasNoise*stdMeasNoise;
  float HPHR = R; 
  for (int i=0; i<KC_STATE_DIM; i++) { 
    HPHR += Hm->pData[i]*PHTd[i]; 
  }
  ASSERT(!isnan(HPHR));
  for (int i=0; i<KC_STATE_DIM; i++) {
    K[i] = PHTd[i]/HPHR; 
    this->S[i] = this->S[i] + K[i] * error; 
  }
  assertStateNotNaN(this);
  mat_mult(&Km, Hm, &tmpNN1m); 
  for (int i=0; i<KC_STATE_DIM; i++) { tmpNN1d[KC_STATE_DIM*i+i] -= 1; } 
  mat_trans(&tmpNN1m, &tmpNN2m); 
  mat_mult(&tmpNN1m, &this->Pm, &tmpNN3m); 
  mat_mult(&tmpNN3m, &tmpNN2m, &this->Pm); 
  assertStateNotNaN(this);
  for (int i=0; i<KC_STATE_DIM; i++) {
    for (int j=i; j<KC_STATE_DIM; j++) {
      float v = K[i] * R * K[j];
      float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i] + v; 
      if (isnan(p) || p > MAX_COVARIANCE) {
        this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
      } else {
        this->P[i][j] = this->P[j][i] = p;
      }
    }
  }
  assertStateNotNaN(this);
}
// Removed: kalmanCoreUpdateWithBaro - Barometer hardware not available
void kalmanCoreUpdateWithAbsoluteHeight(kalmanCoreData_t* this, heightMeasurement_t* height) {
  float h[KC_STATE_DIM] = {0};
  xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
  h[KC_STATE_Z] = 1;
  scalarUpdate(this, &H, height->height - this->S[KC_STATE_Z], height->stdDev);
}
void kalmanCoreUpdateWithPosition(kalmanCoreData_t* this, positionMeasurement_t *xyz)
{
  for (int i=0; i<3; i++) {
    float h[KC_STATE_DIM] = {0};
    xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
    h[KC_STATE_X+i] = 1;
    scalarUpdate(this, &H, xyz->pos[i] - this->S[KC_STATE_X+i], xyz->stdDev);
  }
}
void kalmanCoreUpdateWithPose(kalmanCoreData_t* this, poseMeasurement_t *pose)
{
  for (int i=0; i<3; i++) {
    float h[KC_STATE_DIM] = {0};
    xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
    h[KC_STATE_X+i] = 1;
    scalarUpdate(this, &H, pose->pos[i] - this->S[KC_STATE_X+i], pose->stdDevPos);
  }
  struct quat const q_ekf = mkquat(this->q[1], this->q[2], this->q[3], this->q[0]);
  struct quat const q_measured = mkquat(pose->quat.x, pose->quat.y, pose->quat.z, pose->quat.w);
  struct quat const q_residual = qqmul(qinv(q_ekf), q_measured);
  struct vec const err_quat = vscl(2.0f / q_residual.w, quatimagpart(q_residual));
  {
    float h[KC_STATE_DIM] = {0};
    xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
    h[KC_STATE_D0] = 1;
    scalarUpdate(this, &H, err_quat.x, pose->stdDevQuat);
    h[KC_STATE_D0] = 0;
    h[KC_STATE_D1] = 1;
    scalarUpdate(this, &H, err_quat.y, pose->stdDevQuat);
    h[KC_STATE_D1] = 0;
    h[KC_STATE_D2] = 1;
    scalarUpdate(this, &H, err_quat.z, pose->stdDevQuat);
  }
}
void kalmanCoreUpdateWithDistance(kalmanCoreData_t* this, distanceMeasurement_t *d)
{
  float h[KC_STATE_DIM] = {0};
  xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
  float dx = this->S[KC_STATE_X] - d->x;
  float dy = this->S[KC_STATE_Y] - d->y;
  float dz = this->S[KC_STATE_Z] - d->z;
  float measuredDistance = d->distance;
  float predictedDistance = xtensa_sqrt(powf(dx, 2) + powf(dy, 2) + powf(dz, 2));
  if (predictedDistance != 0.0f)
  {
    h[KC_STATE_X] = dx/predictedDistance;
    h[KC_STATE_Y] = dy/predictedDistance;
    h[KC_STATE_Z] = dz/predictedDistance;
  }
  else
  {
    h[KC_STATE_X] = 1.0f;
    h[KC_STATE_Y] = 0.0f;
    h[KC_STATE_Z] = 0.0f;
  }
  scalarUpdate(this, &H, measuredDistance-predictedDistance, d->stdDev);
}
// Removed: kalmanCoreUpdateWithTDOA() - TDOA hardware not available
static float predictedNX;
static float predictedNY;
static float measuredNX;
static float measuredNY;
void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this, const flowMeasurement_t *flow, const Axis3f *gyro)
{
  if (fabs(this->R[2][2]) <= 0.35f || this->R[2][2] <= 0.0f) {
    return; 
  }
  float Npix = 30.0;                      
  float thetapix = DEG_TO_RAD * 4.2f;
  if (thetapix < 0.001f) {
    thetapix = DEG_TO_RAD * 4.2f; 
  }
  float omegax_b = gyro->x * DEG_TO_RAD;
  float omegay_b = gyro->y * DEG_TO_RAD;
  float dx_g = this->R[0][0] * this->S[KC_STATE_PX] + this->R[0][1] * this->S[KC_STATE_PY] + this->R[0][2] * this->S[KC_STATE_PZ];
  float dy_g = this->R[1][0] * this->S[KC_STATE_PX] + this->R[1][1] * this->S[KC_STATE_PY] + this->R[1][2] * this->S[KC_STATE_PZ];
  float z_g = 0.0;
  const float FLOW_MIN_HEIGHT = 0.1f;
  const float FLOW_MAX_HEIGHT = 3.5f;
  if (this->S[KC_STATE_Z] < FLOW_MIN_HEIGHT) {
    z_g = FLOW_MIN_HEIGHT;
  } else if (this->S[KC_STATE_Z] > FLOW_MAX_HEIGHT) {
    return;
  } else {
    z_g = this->S[KC_STATE_Z];
  }
  if (z_g < FLOW_MIN_HEIGHT) {
    z_g = FLOW_MIN_HEIGHT;
  }
  float thetapix_inv = 1.0f / thetapix; 
  if (isnan(thetapix_inv) || isinf(thetapix_inv)) {
    return; 
  }
  float adaptiveStdDevX = flow->stdDevX;
  float adaptiveStdDevY = flow->stdDevY;
  if (z_g > 2.5f) {
    float heightFactor = 1.0f + ((z_g - 2.5f) / (FLOW_MAX_HEIGHT - 2.5f));
    adaptiveStdDevX *= heightFactor;
    adaptiveStdDevY *= heightFactor;
  }
  float omegaFactor = 1.25f;
  float hx[KC_STATE_DIM] = {0};
  xtensa_matrix_instance_f32 Hx = {1, KC_STATE_DIM, hx};
  predictedNX = (flow->dt * Npix * thetapix_inv) * ((dx_g * this->R[2][2] / z_g) - omegaFactor * omegay_b);
  measuredNX = flow->dpixelx;
  float flow_x_sensitivity = (Npix * flow->dt * thetapix_inv) * (this->R[2][2] / z_g);
  hx[KC_STATE_Z] = (Npix * flow->dt * thetapix_inv) * ((this->R[2][2] * dx_g) / (-z_g * z_g));
  hx[KC_STATE_PX] = flow_x_sensitivity * this->R[0][0];
  hx[KC_STATE_PY] = flow_x_sensitivity * this->R[0][1];
  hx[KC_STATE_PZ] = flow_x_sensitivity * this->R[0][2];
  scalarUpdate(this, &Hx, measuredNX-predictedNX, adaptiveStdDevX);
  float hy[KC_STATE_DIM] = {0};
  xtensa_matrix_instance_f32 Hy = {1, KC_STATE_DIM, hy};
  predictedNY = (flow->dt * Npix * thetapix_inv) * ((dy_g * this->R[2][2] / z_g) + omegaFactor * omegax_b);
  measuredNY = flow->dpixely;
  float flow_y_sensitivity = (Npix * flow->dt * thetapix_inv) * (this->R[2][2] / z_g);
  hy[KC_STATE_Z] = (Npix * flow->dt * thetapix_inv) * ((this->R[2][2] * dy_g) / (-z_g * z_g));
  hy[KC_STATE_PX] = flow_y_sensitivity * this->R[1][0];
  hy[KC_STATE_PY] = flow_y_sensitivity * this->R[1][1];
  hy[KC_STATE_PZ] = flow_y_sensitivity * this->R[1][2];
  scalarUpdate(this, &Hy, measuredNY-predictedNY, adaptiveStdDevY);
}
void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  float h[KC_STATE_DIM] = {0};
  xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
  if (fabs(this->R[2][2]) > 0.35f && this->R[2][2] > 0.0f) {
    float R22_inv = 1.0f / this->R[2][2];
    if (isnan(R22_inv) || isinf(R22_inv)) {
      return; 
    }
    float angle = fabsf(acosf(this->R[2][2])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }
    float predictedDistance = this->S[KC_STATE_Z] * R22_inv;
    float measuredDistance = tof->distance; 
    h[KC_STATE_Z] = R22_inv;
    scalarUpdate(this, &H, measuredDistance - predictedDistance, tof->stdDev);
  }
}
void kalmanCoreUpdateWithYawError(kalmanCoreData_t *this, yawErrorMeasurement_t *error)
{
    float h[KC_STATE_DIM] = {0};
    xtensa_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
    h[KC_STATE_D2] = 1;
    scalarUpdate(this, &H, this->S[KC_STATE_D2] - error->yawError, error->stdDev);
}
void kalmanCorePredict(kalmanCoreData_t* this, float cmdThrust, Axis3f *acc, Axis3f *gyro, float dt, bool quadIsFlying)
{
  NO_DMA_CCM_SAFE_ZERO_INIT static float A[KC_STATE_DIM][KC_STATE_DIM];
  static __attribute__((aligned(4))) xtensa_matrix_instance_f32 Am = { KC_STATE_DIM, KC_STATE_DIM, (float *)A}; 
  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
  static __attribute__((aligned(4))) xtensa_matrix_instance_f32 tmpNN1m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};
  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
  static __attribute__((aligned(4))) xtensa_matrix_instance_f32 tmpNN2m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};
  float dt2 = dt*dt;
  A[KC_STATE_X][KC_STATE_X] = 1;
  A[KC_STATE_Y][KC_STATE_Y] = 1;
  A[KC_STATE_Z][KC_STATE_Z] = 1;
  A[KC_STATE_PX][KC_STATE_PX] = 1;
  A[KC_STATE_PY][KC_STATE_PY] = 1;
  A[KC_STATE_PZ][KC_STATE_PZ] = 1;
  A[KC_STATE_D0][KC_STATE_D0] = 1;
  A[KC_STATE_D1][KC_STATE_D1] = 1;
  A[KC_STATE_D2][KC_STATE_D2] = 1;
  A[KC_STATE_X][KC_STATE_PX] = this->R[0][0]*dt;
  A[KC_STATE_Y][KC_STATE_PX] = this->R[1][0]*dt;
  A[KC_STATE_Z][KC_STATE_PX] = this->R[2][0]*dt;
  A[KC_STATE_X][KC_STATE_PY] = this->R[0][1]*dt;
  A[KC_STATE_Y][KC_STATE_PY] = this->R[1][1]*dt;
  A[KC_STATE_Z][KC_STATE_PY] = this->R[2][1]*dt;
  A[KC_STATE_X][KC_STATE_PZ] = this->R[0][2]*dt;
  A[KC_STATE_Y][KC_STATE_PZ] = this->R[1][2]*dt;
  A[KC_STATE_Z][KC_STATE_PZ] = this->R[2][2]*dt;
  A[KC_STATE_X][KC_STATE_D0] = (this->S[KC_STATE_PY]*this->R[0][2] - this->S[KC_STATE_PZ]*this->R[0][1])*dt;
  A[KC_STATE_Y][KC_STATE_D0] = (this->S[KC_STATE_PY]*this->R[1][2] - this->S[KC_STATE_PZ]*this->R[1][1])*dt;
  A[KC_STATE_Z][KC_STATE_D0] = (this->S[KC_STATE_PY]*this->R[2][2] - this->S[KC_STATE_PZ]*this->R[2][1])*dt;
  A[KC_STATE_X][KC_STATE_D1] = (- this->S[KC_STATE_PX]*this->R[0][2] + this->S[KC_STATE_PZ]*this->R[0][0])*dt;
  A[KC_STATE_Y][KC_STATE_D1] = (- this->S[KC_STATE_PX]*this->R[1][2] + this->S[KC_STATE_PZ]*this->R[1][0])*dt;
  A[KC_STATE_Z][KC_STATE_D1] = (- this->S[KC_STATE_PX]*this->R[2][2] + this->S[KC_STATE_PZ]*this->R[2][0])*dt;
  A[KC_STATE_X][KC_STATE_D2] = (this->S[KC_STATE_PX]*this->R[0][1] - this->S[KC_STATE_PY]*this->R[0][0])*dt;
  A[KC_STATE_Y][KC_STATE_D2] = (this->S[KC_STATE_PX]*this->R[1][1] - this->S[KC_STATE_PY]*this->R[1][0])*dt;
  A[KC_STATE_Z][KC_STATE_D2] = (this->S[KC_STATE_PX]*this->R[2][1] - this->S[KC_STATE_PY]*this->R[2][0])*dt;
  A[KC_STATE_PX][KC_STATE_PX] = 1; 
  A[KC_STATE_PY][KC_STATE_PX] =-gyro->z*dt;
  A[KC_STATE_PZ][KC_STATE_PX] = gyro->y*dt;
  A[KC_STATE_PX][KC_STATE_PY] = gyro->z*dt;
  A[KC_STATE_PY][KC_STATE_PY] = 1; 
  A[KC_STATE_PZ][KC_STATE_PY] =-gyro->x*dt;
  A[KC_STATE_PX][KC_STATE_PZ] =-gyro->y*dt;
  A[KC_STATE_PY][KC_STATE_PZ] = gyro->x*dt;
  A[KC_STATE_PZ][KC_STATE_PZ] = 1; 
  A[KC_STATE_PX][KC_STATE_D0] =  0;
  A[KC_STATE_PY][KC_STATE_D0] = -GRAVITY_MAGNITUDE*this->R[2][2]*dt;
  A[KC_STATE_PZ][KC_STATE_D0] =  GRAVITY_MAGNITUDE*this->R[2][1]*dt;
  A[KC_STATE_PX][KC_STATE_D1] =  GRAVITY_MAGNITUDE*this->R[2][2]*dt;
  A[KC_STATE_PY][KC_STATE_D1] =  0;
  A[KC_STATE_PZ][KC_STATE_D1] = -GRAVITY_MAGNITUDE*this->R[2][0]*dt;
  A[KC_STATE_PX][KC_STATE_D2] = -GRAVITY_MAGNITUDE*this->R[2][1]*dt;
  A[KC_STATE_PY][KC_STATE_D2] =  GRAVITY_MAGNITUDE*this->R[2][0]*dt;
  A[KC_STATE_PZ][KC_STATE_D2] =  0;
  float d0 = gyro->x*dt/2;
  float d1 = gyro->y*dt/2;
  float d2 = gyro->z*dt/2;
  A[KC_STATE_D0][KC_STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
  A[KC_STATE_D0][KC_STATE_D1] =  d2 + d0*d1/2;
  A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0*d2/2;
  A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0*d1/2;
  A[KC_STATE_D1][KC_STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
  A[KC_STATE_D1][KC_STATE_D2] =  d0 + d1*d2/2;
  A[KC_STATE_D2][KC_STATE_D0] =  d1 + d0*d2/2;
  A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1*d2/2;
  A[KC_STATE_D2][KC_STATE_D2] = 1 - d0*d0/2 - d1*d1/2;
  mat_mult(&Am, &this->Pm, &tmpNN1m); 
  mat_trans(&Am, &tmpNN2m); 
  mat_mult(&tmpNN1m, &tmpNN2m, &this->Pm); 
  float dx, dy, dz;
  float tmpSPX, tmpSPY, tmpSPZ;
  float zacc;
  if (quadIsFlying) 
  {
    zacc = acc->z;
    dx = this->S[KC_STATE_PX] * dt;
    dy = this->S[KC_STATE_PY] * dt;
    dz = this->S[KC_STATE_PZ] * dt + zacc * dt2 / 2.0f; 
    this->S[KC_STATE_X] += this->R[0][0] * dx + this->R[0][1] * dy + this->R[0][2] * dz;
    this->S[KC_STATE_Y] += this->R[1][0] * dx + this->R[1][1] * dy + this->R[1][2] * dz;
    this->S[KC_STATE_Z] += this->R[2][0] * dx + this->R[2][1] * dy + this->R[2][2] * dz - GRAVITY_MAGNITUDE * dt2 / 2.0f;
    tmpSPX = this->S[KC_STATE_PX];
    tmpSPY = this->S[KC_STATE_PY];
    tmpSPZ = this->S[KC_STATE_PZ];
    this->S[KC_STATE_PX] += dt * (gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * this->R[2][0]);
    this->S[KC_STATE_PY] += dt * (-gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_MAGNITUDE * this->R[2][1]);
    this->S[KC_STATE_PZ] += dt * (zacc + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_MAGNITUDE * this->R[2][2]);
  }
  else 
  {
    dx = this->S[KC_STATE_PX] * dt + acc->x * dt2 / 2.0f;
    dy = this->S[KC_STATE_PY] * dt + acc->y * dt2 / 2.0f;
    dz = this->S[KC_STATE_PZ] * dt + acc->z * dt2 / 2.0f; 
    this->S[KC_STATE_X] += this->R[0][0] * dx + this->R[0][1] * dy + this->R[0][2] * dz;
    this->S[KC_STATE_Y] += this->R[1][0] * dx + this->R[1][1] * dy + this->R[1][2] * dz;
    this->S[KC_STATE_Z] += this->R[2][0] * dx + this->R[2][1] * dy + this->R[2][2] * dz - GRAVITY_MAGNITUDE * dt2 / 2.0f;
    tmpSPX = this->S[KC_STATE_PX];
    tmpSPY = this->S[KC_STATE_PY];
    tmpSPZ = this->S[KC_STATE_PZ];
    this->S[KC_STATE_PX] += dt * (acc->x + gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * this->R[2][0]);
    this->S[KC_STATE_PY] += dt * (acc->y - gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_MAGNITUDE * this->R[2][1]);
    this->S[KC_STATE_PZ] += dt * (acc->z + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_MAGNITUDE * this->R[2][2]);
  }
  float dtwx = dt*gyro->x;
  float dtwy = dt*gyro->y;
  float dtwz = dt*gyro->z;
  float angle = xtensa_sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz);
  float ca = xtensa_cos_f32(angle/2.0f);
  float sa = xtensa_sin_f32(angle/2.0f);
  float dq[4] = {ca , sa*dtwx/angle , sa*dtwy/angle , sa*dtwz/angle};
  float tmpq0;
  float tmpq1;
  float tmpq2;
  float tmpq3;
  tmpq0 = dq[0]*this->q[0] - dq[1]*this->q[1] - dq[2]*this->q[2] - dq[3]*this->q[3];
  tmpq1 = dq[1]*this->q[0] + dq[0]*this->q[1] + dq[3]*this->q[2] - dq[2]*this->q[3];
  tmpq2 = dq[2]*this->q[0] - dq[3]*this->q[1] + dq[0]*this->q[2] + dq[1]*this->q[3];
  tmpq3 = dq[3]*this->q[0] + dq[2]*this->q[1] - dq[1]*this->q[2] + dq[0]*this->q[3];
  if (! quadIsFlying) {
    float keep = 1.0f - ROLLPITCH_ZERO_REVERSION;
    tmpq0 = keep * tmpq0 + ROLLPITCH_ZERO_REVERSION * initialQuaternion[0];
    tmpq1 = keep * tmpq1 + ROLLPITCH_ZERO_REVERSION * initialQuaternion[1];
    tmpq2 = keep * tmpq2 + ROLLPITCH_ZERO_REVERSION * initialQuaternion[2];
    tmpq3 = keep * tmpq3 + ROLLPITCH_ZERO_REVERSION * initialQuaternion[3];
  }
  float norm = xtensa_sqrt(tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3);
  this->q[0] = tmpq0/norm; this->q[1] = tmpq1/norm; this->q[2] = tmpq2/norm; this->q[3] = tmpq3/norm;
  assertStateNotNaN(this);
}
void kalmanCoreAddProcessNoise(kalmanCoreData_t* this, float dt)
{
  if (dt>0)
  {
    this->P[KC_STATE_X][KC_STATE_X] += powf(procNoiseAcc_xy*dt*dt + procNoiseVel*dt + procNoisePos, 2);  
    this->P[KC_STATE_Y][KC_STATE_Y] += powf(procNoiseAcc_xy*dt*dt + procNoiseVel*dt + procNoisePos, 2);  
    this->P[KC_STATE_Z][KC_STATE_Z] += powf(procNoiseAcc_z*dt*dt + procNoiseVel*dt + procNoisePos, 2);  
    this->P[KC_STATE_PX][KC_STATE_PX] += powf(procNoiseAcc_xy*dt + procNoiseVel, 2); 
    this->P[KC_STATE_PY][KC_STATE_PY] += powf(procNoiseAcc_xy*dt + procNoiseVel, 2); 
    this->P[KC_STATE_PZ][KC_STATE_PZ] += powf(procNoiseAcc_z*dt + procNoiseVel, 2); 
    this->P[KC_STATE_D0][KC_STATE_D0] += powf(measNoiseGyro_rollpitch * dt + procNoiseAtt, 2);
    this->P[KC_STATE_D1][KC_STATE_D1] += powf(measNoiseGyro_rollpitch * dt + procNoiseAtt, 2);
    this->P[KC_STATE_D2][KC_STATE_D2] += powf(measNoiseGyro_yaw * dt + procNoiseAtt, 2);
  }
  for (int i=0; i<KC_STATE_DIM; i++) {
    for (int j=i; j<KC_STATE_DIM; j++) {
      float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
      } else {
        this->P[i][j] = this->P[j][i] = p;
      }
    }
  }
  assertStateNotNaN(this);
}
void kalmanCoreFinalize(kalmanCoreData_t* this, uint32_t tick)
{
  NO_DMA_CCM_SAFE_ZERO_INIT static float A[KC_STATE_DIM][KC_STATE_DIM];
  static xtensa_matrix_instance_f32 Am = {KC_STATE_DIM, KC_STATE_DIM, (float *)A};
  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
  static xtensa_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};
  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
  static xtensa_matrix_instance_f32 tmpNN2m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};
  float v0 = this->S[KC_STATE_D0];
  float v1 = this->S[KC_STATE_D1];
  float v2 = this->S[KC_STATE_D2];
  if ((fabsf(v0) > 0.1e-3f || fabsf(v1) > 0.1e-3f || fabsf(v2) > 0.1e-3f) && (fabsf(v0) < 10 && fabsf(v1) < 10 && fabsf(v2) < 10))
  {
    float angle = xtensa_sqrt(v0*v0 + v1*v1 + v2*v2);
    float ca = xtensa_cos_f32(angle / 2.0f);
    float sa = xtensa_sin_f32(angle / 2.0f);
    float dq[4] = {ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle};
    float tmpq0 = dq[0] * this->q[0] - dq[1] * this->q[1] - dq[2] * this->q[2] - dq[3] * this->q[3];
    float tmpq1 = dq[1] * this->q[0] + dq[0] * this->q[1] + dq[3] * this->q[2] - dq[2] * this->q[3];
    float tmpq2 = dq[2] * this->q[0] - dq[3] * this->q[1] + dq[0] * this->q[2] + dq[1] * this->q[3];
    float tmpq3 = dq[3] * this->q[0] + dq[2] * this->q[1] - dq[1] * this->q[2] + dq[0] * this->q[3];
    float norm = xtensa_sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3);
    this->q[0] = tmpq0 / norm;
    this->q[1] = tmpq1 / norm;
    this->q[2] = tmpq2 / norm;
    this->q[3] = tmpq3 / norm;
    float d0 = v0/2; 
    float d1 = v1/2; 
    float d2 = v2/2;
    A[KC_STATE_X][KC_STATE_X] = 1;
    A[KC_STATE_Y][KC_STATE_Y] = 1;
    A[KC_STATE_Z][KC_STATE_Z] = 1;
    A[KC_STATE_PX][KC_STATE_PX] = 1;
    A[KC_STATE_PY][KC_STATE_PY] = 1;
    A[KC_STATE_PZ][KC_STATE_PZ] = 1;
    A[KC_STATE_D0][KC_STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
    A[KC_STATE_D0][KC_STATE_D1] =  d2 + d0*d1/2;
    A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0*d2/2;
    A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0*d1/2;
    A[KC_STATE_D1][KC_STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
    A[KC_STATE_D1][KC_STATE_D2] =  d0 + d1*d2/2;
    A[KC_STATE_D2][KC_STATE_D0] =  d1 + d0*d2/2;
    A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1*d2/2;
    A[KC_STATE_D2][KC_STATE_D2] = 1 - d0*d0/2 - d1*d1/2;
    mat_trans(&Am, &tmpNN1m); 
    mat_mult(&Am, &this->Pm, &tmpNN2m); 
    mat_mult(&tmpNN2m, &tmpNN1m, &this->Pm); 
  }
  this->R[0][0] = this->q[0] * this->q[0] + this->q[1] * this->q[1] - this->q[2] * this->q[2] - this->q[3] * this->q[3];
  this->R[0][1] = 2 * this->q[1] * this->q[2] - 2 * this->q[0] * this->q[3];
  this->R[0][2] = 2 * this->q[1] * this->q[3] + 2 * this->q[0] * this->q[2];
  this->R[1][0] = 2 * this->q[1] * this->q[2] + 2 * this->q[0] * this->q[3];
  this->R[1][1] = this->q[0] * this->q[0] - this->q[1] * this->q[1] + this->q[2] * this->q[2] - this->q[3] * this->q[3];
  this->R[1][2] = 2 * this->q[2] * this->q[3] - 2 * this->q[0] * this->q[1];
  this->R[2][0] = 2 * this->q[1] * this->q[3] - 2 * this->q[0] * this->q[2];
  this->R[2][1] = 2 * this->q[2] * this->q[3] + 2 * this->q[0] * this->q[1];
  this->R[2][2] = this->q[0] * this->q[0] - this->q[1] * this->q[1] - this->q[2] * this->q[2] + this->q[3] * this->q[3];
  this->S[KC_STATE_D0] = 0;
  this->S[KC_STATE_D1] = 0;
  this->S[KC_STATE_D2] = 0;
  for (int i=0; i<KC_STATE_DIM; i++) {
    for (int j=i; j<KC_STATE_DIM; j++) {
      float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
      } else {
        this->P[i][j] = this->P[j][i] = p;
      }
    }
  }
  assertStateNotNaN(this);
}
void kalmanCoreExternalizeState(const kalmanCoreData_t* this, state_t *state, const Axis3f *acc, uint32_t tick)
{
  state->position = (point_t){
      .timestamp = tick,
      .x = this->S[KC_STATE_X],
      .y = this->S[KC_STATE_Y],
      .z = this->S[KC_STATE_Z]
  };
  state->velocity = (velocity_t){
      .timestamp = tick,
      .x = this->R[0][0]*this->S[KC_STATE_PX] + this->R[0][1]*this->S[KC_STATE_PY] + this->R[0][2]*this->S[KC_STATE_PZ],
      .y = this->R[1][0]*this->S[KC_STATE_PX] + this->R[1][1]*this->S[KC_STATE_PY] + this->R[1][2]*this->S[KC_STATE_PZ],
      .z = this->R[2][0]*this->S[KC_STATE_PX] + this->R[2][1]*this->S[KC_STATE_PY] + this->R[2][2]*this->S[KC_STATE_PZ]
  };
  state->acc = (acc_t){
      .timestamp = tick,
      .x = this->R[0][0]*acc->x + this->R[0][1]*acc->y + this->R[0][2]*acc->z,
      .y = this->R[1][0]*acc->x + this->R[1][1]*acc->y + this->R[1][2]*acc->z,
      .z = this->R[2][0]*acc->x + this->R[2][1]*acc->y + this->R[2][2]*acc->z - 1
  };
  float yaw = atan2f(2*(this->q[1]*this->q[2]+this->q[0]*this->q[3]) , this->q[0]*this->q[0] + this->q[1]*this->q[1] - this->q[2]*this->q[2] - this->q[3]*this->q[3]);
  float pitch = asinf(-2*(this->q[1]*this->q[3] - this->q[0]*this->q[2]));
  float roll = atan2f(2*(this->q[2]*this->q[3]+this->q[0]*this->q[1]) , this->q[0]*this->q[0] - this->q[1]*this->q[1] - this->q[2]*this->q[2] + this->q[3]*this->q[3]);
  state->attitude = (attitude_t){
      .timestamp = tick,
      .roll = roll*RAD_TO_DEG,
      .pitch = -pitch*RAD_TO_DEG,
      .yaw = yaw*RAD_TO_DEG
  };
  state->attitudeQuaternion = (quaternion_t){
      .timestamp = tick,
      .w = this->q[0],
      .x = this->q[1],
      .y = this->q[2],
      .z = this->q[3]
  };
  assertStateNotNaN(this);
}
static void decoupleState(kalmanCoreData_t* this, kalmanCoreStateIdx_t state)
{
  for(int i=0; i<KC_STATE_DIM; i++) {
    this->P[state][i] = 0;
    this->P[i][state] = 0;
  }
  this->P[state][state] = MAX_COVARIANCE;
  this->S[state] = 0;
}
void kalmanCoreDecoupleXY(kalmanCoreData_t* this)
{
  decoupleState(this, KC_STATE_X);
  decoupleState(this, KC_STATE_PX);
  decoupleState(this, KC_STATE_Y);
  decoupleState(this, KC_STATE_PY);
}
LOG_GROUP_START(kalman_pred)
  LOG_ADD(LOG_FLOAT, predNX, &predictedNX)
  LOG_ADD(LOG_FLOAT, predNY, &predictedNY)
  LOG_ADD(LOG_FLOAT, measNX, &measuredNX)
  LOG_ADD(LOG_FLOAT, measNY, &measuredNY)
LOG_GROUP_STOP(kalman_pred)
LOG_GROUP_START(outlierf)
  // Removed: lhWin log - Lighthouse hardware not available
LOG_GROUP_STOP(outlierf)
PARAM_GROUP_START(kalman)
  PARAM_ADD(PARAM_FLOAT, pNAcc_xy, &procNoiseAcc_xy)
  PARAM_ADD(PARAM_FLOAT, pNAcc_z, &procNoiseAcc_z)
  PARAM_ADD(PARAM_FLOAT, pNVel, &procNoiseVel)
  PARAM_ADD(PARAM_FLOAT, pNPos, &procNoisePos)
  PARAM_ADD(PARAM_FLOAT, pNAtt, &procNoiseAtt)
  // Removed: mNBaro parameter - Barometer hardware not available
  PARAM_ADD(PARAM_FLOAT, mNGyro_rollpitch, &measNoiseGyro_rollpitch)
  PARAM_ADD(PARAM_FLOAT, mNGyro_yaw, &measNoiseGyro_yaw)
  PARAM_ADD(PARAM_FLOAT, initialX, &initialX)
  PARAM_ADD(PARAM_FLOAT, initialY, &initialY)
  PARAM_ADD(PARAM_FLOAT, initialZ, &initialZ)
  PARAM_ADD(PARAM_FLOAT, initialYaw, &initialYaw)
PARAM_GROUP_STOP(kalman)