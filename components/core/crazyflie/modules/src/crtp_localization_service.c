#include <string.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "crtp.h"
#include "crtp_localization_service.h"
#include "log.h"
#include "param.h"
#include "stabilizer_types.h"
#include "stabilizer.h"
#include "configblock.h"
#include "estimator.h"
#include "quatcompress.h"
#include "num.h"
#define NBR_OF_RANGES_IN_PACKET   5
// Removed: Lighthouse-related defines - Lighthouse hardware not available
#define NBR_OF_
#define DEFAULT_EMERGENCY_STOP_TIMEOUT (1 * RATE_MAIN_LOOP)
typedef enum
{
  EXT_POSITION        = 0,
  GENERIC_TYPE        = 1,
  EXT_POSITION_PACKED = 2,
} locsrvChannels_t;
typedef struct
{
  uint8_t type;
  struct
  {
    uint8_t id;
    float range;
  } __attribute__((packed)) ranges[NBR_OF_RANGES_IN_PACKET];
} __attribute__((packed)) rangePacket;
// Removed: anglePacket struct - Lighthouse hardware not available
typedef struct {
  uint8_t id; 
  int16_t x; 
  int16_t y; 
  int16_t z; 
} __attribute__((packed)) extPositionPackedItem;
typedef struct {
  uint8_t id; 
  int16_t x; 
  int16_t y; 
  int16_t z; 
  uint32_t quat; 
} __attribute__((packed)) extPosePackedItem;
static positionMeasurement_t ext_pos;
static poseMeasurement_t ext_pose;
static CRTPPacket pkRange;
static uint8_t rangeIndex;
static bool enableRangeStreamFloat = false;
// Removed: Lighthouse angle stream support - Lighthouse hardware not available
static float extPosStdDev = 0.01;
static float extQuatStdDev = 4.5e-3;
static bool isInit = false;
static uint8_t my_id;
static uint16_t tickOfLastPacket; 
static void locSrvCrtpCB(CRTPPacket* pk);
static void extPositionHandler(CRTPPacket* pk);
static void genericLocHandle(CRTPPacket* pk);
static void extPositionPackedHandler(CRTPPacket* pk);
void locSrvInit()
{
  if (isInit) {
    return;
  }
  uint64_t address = configblockGetRadioAddress();
  my_id = address & 0xFF;
  crtpRegisterPortCB(CRTP_PORT_LOCALIZATION, locSrvCrtpCB);
  isInit = true;
}
static void locSrvCrtpCB(CRTPPacket* pk)
{
  switch (pk->channel)
  {
    case EXT_POSITION:
      extPositionHandler(pk);
      break;
    case GENERIC_TYPE:
      genericLocHandle(pk);
      break;
    case EXT_POSITION_PACKED:
      extPositionPackedHandler(pk);
      break;
    default:
      break;
  }
}
static void extPositionHandler(CRTPPacket* pk) {
  const struct CrtpExtPosition* data = (const struct CrtpExtPosition*)pk->data;
  ext_pos.x = data->x;
  ext_pos.y = data->y;
  ext_pos.z = data->z;
  ext_pos.stdDev = extPosStdDev;
  estimatorEnqueuePosition(&ext_pos);
  tickOfLastPacket = xTaskGetTickCount();
}
static void extPoseHandler(const CRTPPacket* pk) {
  const struct CrtpExtPose* data = (const struct CrtpExtPose*)&pk->data[1];
  ext_pose.x = data->x;
  ext_pose.y = data->y;
  ext_pose.z = data->z;
  ext_pose.quat.x = data->qx;
  ext_pose.quat.y = data->qy;
  ext_pose.quat.z = data->qz;
  ext_pose.quat.w = data->qw;
  ext_pose.stdDevPos = extPosStdDev;
  ext_pose.stdDevQuat = extQuatStdDev;
  estimatorEnqueuePose(&ext_pose);
  tickOfLastPacket = xTaskGetTickCount();
}
static void extPosePackedHandler(const CRTPPacket* pk) {
  uint8_t numItems = (pk->size - 1) / sizeof(extPosePackedItem);
  for (uint8_t i = 0; i < numItems; ++i) {
    const extPosePackedItem* item = (const extPosePackedItem*)&pk->data[1 + i * sizeof(extPosePackedItem)];
    if (item->id == my_id) {
      ext_pose.x = item->x / 1000.0f;
      ext_pose.y = item->y / 1000.0f;
      ext_pose.z = item->z / 1000.0f;
      quatdecompress(item->quat, (float *)&ext_pose.quat.q0);
      ext_pose.stdDevPos = extPosStdDev;
      ext_pose.stdDevQuat = extQuatStdDev;
      estimatorEnqueuePose(&ext_pose);
      tickOfLastPacket = xTaskGetTickCount();
    }
  }
}
static void lpsShortLppPacketHandler(CRTPPacket* pk) {
  if (pk->size >= 2) {
    bool success = lpsSendLppShort(pk->data[1], &pk->data[2], pk->size-2);
    pk->port = CRTP_PORT_LOCALIZATION;
    pk->channel = GENERIC_TYPE;
    pk->size = 3;
    pk->data[0] = LPS_SHORT_LPP_PACKET;
    pk->data[2] = success?1:0;
    crtpSendPacket(pk);
  }
}
// Removed: Lighthouse persist data handler - Lighthouse hardware not available
static void genericLocHandle(CRTPPacket* pk)
{
  const uint8_t type = pk->data[0];
  if (pk->size < 1) return;
  switch (type) {
    case LPS_SHORT_LPP_PACKET:
      lpsShortLppPacketHandler(pk);
      break;
    case EMERGENCY_STOP:
      stabilizerSetEmergencyStop();
      break;
    case EMERGENCY_STOP_WATCHDOG:
      stabilizerSetEmergencyStopTimeout(DEFAULT_EMERGENCY_STOP_TIMEOUT);
      break;
    case EXT_POSE:
      extPoseHandler(pk);
      break;
    case EXT_POSE_PACKED:
      extPosePackedHandler(pk);
      break;
    // Removed: LH_PERSIST_DATA case - Lighthouse hardware not available
    default:
      break;
  }
}
static void extPositionPackedHandler(CRTPPacket* pk)
{
  uint8_t numItems = pk->size / sizeof(extPositionPackedItem);
  for (uint8_t i = 0; i < numItems; ++i) {
    const extPositionPackedItem* item = (const extPositionPackedItem*)&pk->data[i * sizeof(extPositionPackedItem)];
    ext_pos.x = item->x / 1000.0f;
    ext_pos.y = item->y / 1000.0f;
    ext_pos.z = item->z / 1000.0f;
    ext_pos.stdDev = extPosStdDev;
    if (item->id == my_id) {
      estimatorEnqueuePosition(&ext_pos);
      tickOfLastPacket = xTaskGetTickCount();
    }
  }
}
void locSrvSendRangeFloat(uint8_t id, float range)
{
  rangePacket *rp = (rangePacket *)pkRange.data;
  ASSERT(rangeIndex <= NBR_OF_RANGES_IN_PACKET);
  if (enableRangeStreamFloat)
  {
    rp->ranges[rangeIndex].id = id;
    rp->ranges[rangeIndex].range = range;
    rangeIndex++;
    if (rangeIndex >= 5)
    {
      rp->type = RANGE_STREAM_FLOAT;
      pkRange.port = CRTP_PORT_LOCALIZATION;
      pkRange.channel = GENERIC_TYPE;
      pkRange.size = sizeof(rangePacket);
      crtpSendPacket(&pkRange);
      rangeIndex = 0;
    }
  }
}
// Removed: locSrvSendLighthouseAngle() - Lighthouse hardware not available
LOG_GROUP_START(ext_pos)
  LOG_ADD(LOG_FLOAT, X, &ext_pos.x)
  LOG_ADD(LOG_FLOAT, Y, &ext_pos.y)
  LOG_ADD(LOG_FLOAT, Z, &ext_pos.z)
LOG_GROUP_STOP(ext_pos)
LOG_GROUP_START(locSrvZ)
  LOG_ADD(LOG_UINT16, tick, &tickOfLastPacket)  
LOG_GROUP_STOP(locSrvZ)
PARAM_GROUP_START(locSrv)
  PARAM_ADD(PARAM_UINT8, enRangeStreamFP32, &enableRangeStreamFloat)
  // Removed: enLhAngleStream param - Lighthouse hardware not available
  PARAM_ADD(PARAM_FLOAT, extPosStdDev, &extPosStdDev)
  PARAM_ADD(PARAM_FLOAT, extQuatStdDev, &extQuatStdDev)
PARAM_GROUP_STOP(locSrv)