#include <math.h>
#include "sensfusion6.h"
#include "log.h"
#include "param.h"
#include "physicalConstants.h"
#ifdef MADWICK_QUATERNION_IMU
  #define BETA_DEF     0.01f    
#else 
    #define TWO_KP_DEF  (2.0f * 0.4f) 
    #define TWO_KI_DEF  (2.0f * 0.001f) 
#endif
#ifdef MADWICK_QUATERNION_IMU
  float beta = BETA_DEF;     
#else 
  float twoKp = TWO_KP_DEF;    
  float twoKi = TWO_KI_DEF;    
  float integralFBx = 0.0f;
  float integralFBy = 0.0f;
  float integralFBz = 0.0f;  
#endif
float qw = 1.0f;
float qx = 0.0f;
float qy = 0.0f;
float qz = 0.0f;  
static float gravX, gravY, gravZ; 
static float baseZacc = 1.0;
static bool isInit;
static bool isCalibrated = false;
static void sensfusion6UpdateQImpl(float gx, float gy, float gz, float ax, float ay, float az, float dt);
static float sensfusion6GetAccZ(const float ax, const float ay, const float az);
static void estimatedGravityDirection(float* gx, float* gy, float* gz);
static float invSqrt(float x);
void sensfusion6Init()
{
  if(isInit)
    return;
  isInit = true;
}
bool sensfusion6Test(void)
{
  return isInit;
}
void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  sensfusion6UpdateQImpl(gx, gy, gz, ax, ay, az, dt);
  estimatedGravityDirection(&gravX, &gravY, &gravZ);
  if (!isCalibrated) {
    baseZacc = sensfusion6GetAccZ(ax, ay, az);
    isCalibrated = true;
  }
}
#ifdef SENSORS_ENABLE_MAG_AK8963
void sensfusion6UpdateQWithMag(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
  // First do standard 6DOF update
  sensfusion6UpdateQImpl(gx, gy, gz, ax, ay, az, dt);
  
  // Then apply magnetometer correction for yaw
  // Normalize magnetometer vector
  float magNorm = sqrtf(mx*mx + my*my + mz*mz);
  if (magNorm > 0.01f && magNorm < 2.0f) {
    mx /= magNorm;
    my /= magNorm;
    mz /= magNorm;
    
    // Rotate magnetometer to earth frame using current quaternion
    // mag_earth = q * (0, mx, my, mz) * q^-1
    float hx = 2.0f * (mx * (0.5f - qy*qy - qz*qz) + my * (qx*qy - qw*qz) + mz * (qx*qz + qw*qy));
    float hy = 2.0f * (mx * (qx*qy + qw*qz) + my * (0.5f - qx*qx - qz*qz) + mz * (qy*qz - qw*qx));
    // hz not used - only horizontal components needed for yaw calculation
    // float hz = 2.0f * (mx * (qx*qz - qw*qy) + my * (qy*qz + qw*qx) + mz * (0.5f - qx*qx - qy*qy));
    
    // Normalize horizontal component
    float bx = sqrtf(hx*hx + hy*hy);
    if (bx > 0.01f) {
      bx = 1.0f / bx;
      hx *= bx;
      hy *= bx;
      
      // Calculate reference direction (pointing north)
      float refX = 1.0f;
      float refY = 0.0f;
      
      // Calculate error between measured and reference
      float errorX = refX - hx;
      float errorY = refY - hy;
      
      // Apply correction to quaternion (small correction for yaw)
      float correctionGain = 0.01f; // Small gain to avoid instability
      qw += (-qx * errorX - qy * errorY) * correctionGain * dt;
      qx += (qw * errorX + qy * errorX - qz * errorY) * correctionGain * dt;
      qy += (qw * errorY - qx * errorY + qz * errorX) * correctionGain * dt;
      qz += (qx * errorX + qy * errorY) * correctionGain * dt;
      
      // Renormalize quaternion
      float qNorm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
      if (qNorm > 0.001f) {
        qNorm = 1.0f / qNorm;
        qw *= qNorm;
        qx *= qNorm;
        qy *= qNorm;
        qz *= qNorm;
      }
    }
  }
  
  estimatedGravityDirection(&gravX, &gravY, &gravZ);
  if (!isCalibrated) {
    baseZacc = sensfusion6GetAccZ(ax, ay, az);
    isCalibrated = true;
  }
}
#endif
#ifdef MADWICK_QUATERNION_IMU
static void sensfusion6UpdateQImpl(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2qw, _2qx, _2qy, _2qz, _4qw, _4qx, _4qy ,_8qx, _8qy, qwqw, qxqx, qyqy, qzqz;
  qDot1 = 0.5f * (-qx * gx - qy * gy - qz * gz);
  qDot2 = 0.5f * (qw * gx + qy * gz - qz * gy);
  qDot3 = 0.5f * (qw * gy - qx * gz + qz * gx);
  qDot4 = 0.5f * (qw * gz + qx * gy - qy * gx);
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    _2qw = 2.0f * qw;
    _2qx = 2.0f * qx;
    _2qy = 2.0f * qy;
    _2qz = 2.0f * qz;
    _4qw = 4.0f * qw;
    _4qx = 4.0f * qx;
    _4qy = 4.0f * qy;
    _8qx = 8.0f * qx;
    _8qy = 8.0f * qy;
    qwqw = qw * qw;
    qxqx = qx * qx;
    qyqy = qy * qy;
    qzqz = qz * qz;
    s0 = _4qw * qyqy + _2qy * ax + _4qw * qxqx - _2qx * ay;
    s1 = _4qx * qzqz - _2qz * ax + 4.0f * qwqw * qx - _2qw * ay - _4qx + _8qx * qxqx + _8qx * qyqy + _4qx * az;
    s2 = 4.0f * qwqw * qy + _2qw * ax + _4qy * qzqz - _2qz * ay - _4qy + _8qy * qxqx + _8qy * qyqy + _4qy * az;
    s3 = 4.0f * qxqx * qz - _2qx * ax + 4.0f * qyqy * qz - _2qy * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }
  qw += qDot1 * dt;
  qx += qDot2 * dt;
  qy += qDot3 * dt;
  qz += qDot4 * dt;
  recipNorm = invSqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  qw *= recipNorm;
  qx *= recipNorm;
  qy *= recipNorm;
  qz *= recipNorm;
}
#else 
static void sensfusion6UpdateQImpl(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  gx = gx * M_PI_F / 180;
  gy = gy * M_PI_F / 180;
  gz = gz * M_PI_F / 180;
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    halfvx = qx * qz - qw * qy;
    halfvy = qw * qx + qy * qz;
    halfvz = qw * qw - 0.5f + qz * qz;
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    if(twoKi > 0.0f)
    {
      integralFBx += twoKi * halfex * dt;  
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;  
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f; 
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  gx *= (0.5f * dt);   
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = qw;
  qb = qx;
  qc = qy;
  qw += (-qb * gx - qc * gy - qz * gz);
  qx += (qa * gx + qc * gz - qz * gy);
  qy += (qa * gy - qb * gz + qz * gx);
  qz += (qa * gz + qb * gy - qc * gx);
  recipNorm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  qw *= recipNorm;
  qx *= recipNorm;
  qy *= recipNorm;
  qz *= recipNorm;
}
#endif
void sensfusion6GetQuaternion(float* q_x, float* q_y, float* q_z, float* q_w)
{
  *q_x = qx;
  *q_y = qy;
  *q_z = qz;
  *q_w = qw;
}
void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw)
{
  float gx = gravX;
  float gy = gravY;
  float gz = gravZ;
  if (gx>1) gx=1;
  if (gx<-1) gx=-1;
  *yaw = atan2f(2*(qw*qz + qx*qy), qw*qw + qx*qx - qy*qy - qz*qz) * 180 / M_PI_F;
  *pitch = asinf(gx) * 180 / M_PI_F; 
  *roll = atan2f(gy, gz) * 180 / M_PI_F;
}
float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az)
{
  return sensfusion6GetAccZ(ax, ay, az) - baseZacc;
}
float sensfusion6GetInvThrustCompensationForTilt()
{
  return gravZ;
}
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}
static float sensfusion6GetAccZ(const float ax, const float ay, const float az)
{
  return (ax * gravX + ay * gravY + az * gravZ);
}
static void estimatedGravityDirection(float* gx, float* gy, float* gz)
{
  *gx = 2 * (qx * qz - qw * qy);
  *gy = 2 * (qw * qx + qy * qz);
  *gz = qw * qw - qx * qx - qy * qy + qz * qz;
}
LOG_GROUP_START(sensfusion6)
  LOG_ADD(LOG_FLOAT, qw, &qw)
  LOG_ADD(LOG_FLOAT, qx, &qx)
  LOG_ADD(LOG_FLOAT, qy, &qy)
  LOG_ADD(LOG_FLOAT, qz, &qz)
  LOG_ADD(LOG_FLOAT, gravityX, &gravX)
  LOG_ADD(LOG_FLOAT, gravityY, &gravY)
  LOG_ADD(LOG_FLOAT, gravityZ, &gravZ)
  LOG_ADD(LOG_FLOAT, accZbase, &baseZacc)
  LOG_ADD(LOG_UINT8, isInit, &isInit)
  LOG_ADD(LOG_UINT8, isCalibrated, &isCalibrated)
LOG_GROUP_STOP(sensfusion6)
PARAM_GROUP_START(sensfusion6)
#ifdef MADWICK_QUATERNION_IMU
PARAM_ADD(PARAM_FLOAT, beta, &beta)
#else 
PARAM_ADD(PARAM_FLOAT, kp, &twoKp)
PARAM_ADD(PARAM_FLOAT, ki, &twoKi)
#endif
PARAM_ADD(PARAM_FLOAT, baseZacc, &baseZacc)
PARAM_GROUP_STOP(sensfusion6)