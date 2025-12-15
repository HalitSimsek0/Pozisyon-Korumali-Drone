#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "crtp_commander.h"
#include "commander.h"
#include "param.h"
#include "crtp.h"
#include "num.h"
#include "quatcompress.h"
#include "FreeRTOS.h"
#include "cfassert.h"
typedef void (*packetDecoder_t)(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen);
enum packet_type {
  stopType          = 0,
  velocityWorldType = 1,
  zDistanceType     = 2,
  cppmEmuType       = 3,
  altHoldType       = 4,
  hoverType         = 5,
  fullStateType     = 6,
  positionType      = 7,
};
static void stopDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  return;
}
struct velocityPacket_s {
  float vx;        
  float vy;        
  float vz;        
  float yawrate;  
} __attribute__((packed));
static void velocityDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct velocityPacket_s *values = data;
  ASSERT(datalen == sizeof(struct velocityPacket_s));
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->mode.z = modeVelocity;
  setpoint->velocity.x = values->vx;
  setpoint->velocity.y = values->vy;
  setpoint->velocity.z = values->vz;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = -values->yawrate;
}
struct zDistancePacket_s {
  float roll;            
  float pitch;           
  float yawrate;         
  float zDistance;        
} __attribute__((packed));
static void zDistanceDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct zDistancePacket_s *values = data;
  ASSERT(datalen == sizeof(struct zDistancePacket_s));
  setpoint->mode.z = modeAbs;
  setpoint->position.z = values->zDistance;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = -values->yawrate;
  setpoint->mode.roll = modeAbs;
  setpoint->mode.pitch = modeAbs;
  setpoint->attitude.roll = values->roll;
  setpoint->attitude.pitch = values->pitch;
}
#define MAX_AUX_RC_CHANNELS 10
static float s_CppmEmuRollMaxRateDps = 720.0f; 
static float s_CppmEmuPitchMaxRateDps = 720.0f; 
static float s_CppmEmuRollMaxAngleDeg = 50.0f; 
static float s_CppmEmuPitchMaxAngleDeg = 50.0f; 
static float s_CppmEmuYawMaxRateDps = 400.0f; 
struct cppmEmuPacket_s {
  struct {
      uint8_t numAuxChannels : 4;   
      uint8_t reserved : 4;
  } hdr;
  uint16_t channelRoll;
  uint16_t channelPitch;
  uint16_t channelYaw;
  uint16_t channelThrust;
  uint16_t channelAux[MAX_AUX_RC_CHANNELS];
} __attribute__((packed));
static inline float getChannelUnitMultiplier(uint16_t channelValue, uint16_t channelMidpoint, uint16_t channelRange)
{
  return ((float)channelValue - (float)channelMidpoint) / (float)channelRange;
}
static void cppmEmuDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  bool isSelfLevelEnabled = true;
  ASSERT(datalen >= 9); 
  const struct cppmEmuPacket_s *values = data;
  ASSERT(datalen == 9 + (2*values->hdr.numAuxChannels)); 
  isSelfLevelEnabled = !(values->hdr.numAuxChannels >= 1 && values->channelAux[0] < 1500);
  setpoint->mode.x = modeDisable;
  setpoint->mode.y = modeDisable;
  setpoint->mode.z = modeDisable;
  setpoint->mode.yaw = modeVelocity;
  setpoint->mode.roll = isSelfLevelEnabled ? modeAbs : modeVelocity;
  setpoint->mode.pitch = isSelfLevelEnabled ? modeAbs : modeVelocity;
  if(isSelfLevelEnabled)
  {
    setpoint->attitude.roll = -1 * getChannelUnitMultiplier(values->channelRoll, 1500, 500) * s_CppmEmuRollMaxAngleDeg; 
    setpoint->attitude.pitch = -1 * getChannelUnitMultiplier(values->channelPitch, 1500, 500) * s_CppmEmuPitchMaxAngleDeg; 
  }
  else
  {
    setpoint->attitudeRate.roll = -1 * getChannelUnitMultiplier(values->channelRoll, 1500, 500) * s_CppmEmuRollMaxRateDps; 
    setpoint->attitudeRate.pitch = -1 * getChannelUnitMultiplier(values->channelPitch, 1500, 500) * s_CppmEmuPitchMaxRateDps; 
  }
  setpoint->attitudeRate.yaw = -1 * getChannelUnitMultiplier(values->channelYaw, 1500, 500) * s_CppmEmuYawMaxRateDps; 
  setpoint->thrust = getChannelUnitMultiplier(values->channelThrust, 1000, 1000) * (float)UINT16_MAX; 
  if(setpoint->thrust < 0)
  {
    setpoint->thrust = 0;
  }
}
struct altHoldPacket_s {
  float roll;            
  float pitch;           
  float yawrate;         
  float zVelocity;       
} __attribute__((packed));
static void altHoldDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct altHoldPacket_s *values = data;
  ASSERT(datalen == sizeof(struct altHoldPacket_s));
  setpoint->mode.z = modeVelocity;
  setpoint->velocity.z = values->zVelocity;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = -values->yawrate;
  setpoint->mode.roll = modeAbs;
  setpoint->mode.pitch = modeAbs;
  setpoint->attitude.roll = values->roll;
  setpoint->attitude.pitch = values->pitch;
}
struct hoverPacket_s {
  float vx;           
  float vy;           
  float yawrate;      
  float zDistance;    
} __attribute__((packed));
static void hoverDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct hoverPacket_s *values = data;
  ASSERT(datalen == sizeof(struct hoverPacket_s));
  setpoint->mode.z = modeAbs;
  setpoint->position.z = values->zDistance;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = -values->yawrate;
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = values->vx;
  setpoint->velocity.y = values->vy;
  setpoint->velocity_body = true;
}
struct fullStatePacket_s {
  int16_t x;         
  int16_t y;
  int16_t z;
  int16_t vx;        
  int16_t vy;
  int16_t vz;
  int16_t ax;        
  int16_t ay;
  int16_t az;
  int32_t quat;      
  int16_t rateRoll;  
  int16_t ratePitch; 
  int16_t rateYaw;   
} __attribute__((packed));
static void fullStateDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct fullStatePacket_s *values = data;
  ASSERT(datalen == sizeof(struct fullStatePacket_s));
  // X axis
  setpoint->mode.x = modeAbs;
  setpoint->position.x = values->x / 1000.0f;
  setpoint->velocity.x = values->vx / 1000.0f;
  setpoint->acceleration.x = values->ax / 1000.0f;
  // Y axis
  setpoint->mode.y = modeAbs;
  setpoint->position.y = values->y / 1000.0f;
  setpoint->velocity.y = values->vy / 1000.0f;
  setpoint->acceleration.y = values->ay / 1000.0f;
  // Z axis
  setpoint->mode.z = modeAbs;
  setpoint->position.z = values->z / 1000.0f;
  setpoint->velocity.z = values->vz / 1000.0f;
  setpoint->acceleration.z = values->az / 1000.0f;
  float const millirad2deg = 180.0f / ((float)M_PI * 1000.0f);
  setpoint->attitudeRate.roll = millirad2deg * values->rateRoll;
  setpoint->attitudeRate.pitch = millirad2deg * values->ratePitch;
  setpoint->attitudeRate.yaw = millirad2deg * values->rateYaw;
  quatdecompress(values->quat, (float *)&setpoint->attitudeQuaternion.q0);
  setpoint->mode.quat = modeAbs;
  setpoint->mode.roll = modeDisable;
  setpoint->mode.pitch = modeDisable;
  setpoint->mode.yaw = modeDisable;
}
 struct positionPacket_s {
   float x;     
   float y;
   float z;
   float yaw;   
 } __attribute__((packed));
static void positionDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct positionPacket_s *values = data;
  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;
  setpoint->position.x = values->x;
  setpoint->position.y = values->y;
  setpoint->position.z = values->z;
  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = values->yaw;
}
const static packetDecoder_t packetDecoders[] = {
  [stopType]          = stopDecoder,
  [velocityWorldType] = velocityDecoder,
  [zDistanceType]     = zDistanceDecoder,
  [cppmEmuType]       = cppmEmuDecoder,
  [altHoldType]       = altHoldDecoder,
  [hoverType]         = hoverDecoder,
  [fullStateType]     = fullStateDecoder,
  [positionType]      = positionDecoder,
};
void crtpCommanderGenericDecodeSetpoint(setpoint_t *setpoint, CRTPPacket *pk)
{
  static int nTypes = -1;
  ASSERT(pk->size > 0);
  if (nTypes<0) {
    nTypes = sizeof(packetDecoders)/sizeof(packetDecoders[0]);
  }
  uint8_t type = pk->data[0];
  memset(setpoint, 0, sizeof(setpoint_t));
  if (type<nTypes && (packetDecoders[type] != NULL)) {
    packetDecoders[type](setpoint, type, ((char*)pk->data)+1, pk->size-1);
  }
}
PARAM_GROUP_START(cmdrCPPM)
PARAM_ADD(PARAM_FLOAT, rateRoll, &s_CppmEmuRollMaxRateDps)
PARAM_ADD(PARAM_FLOAT, ratePitch, &s_CppmEmuPitchMaxRateDps)
PARAM_ADD(PARAM_FLOAT, rateYaw, &s_CppmEmuYawMaxRateDps)
PARAM_ADD(PARAM_FLOAT, angRoll, &s_CppmEmuRollMaxAngleDeg)
PARAM_ADD(PARAM_FLOAT, angPitch, &s_CppmEmuPitchMaxAngleDeg)
PARAM_GROUP_STOP(cmdrCPPM)