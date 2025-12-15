#ifndef _CRTP_LOCALIZATION_SERVICE_H_
#define _CRTP_LOCALIZATION_SERVICE_H_
#include "stabilizer_types.h"
struct CrtpExtPosition
{
  float x; 
  float y; 
  float z; 
} __attribute__((packed));
struct CrtpExtPose
{
  float x; 
  float y; 
  float z; 
  float qx;
  float qy;
  float qz;
  float qw;
} __attribute__((packed));
typedef enum
{
  RANGE_STREAM_FLOAT      = 0,
  RANGE_STREAM_FP16       = 1,
  LPS_SHORT_LPP_PACKET    = 2,
  EMERGENCY_STOP          = 3,
  EMERGENCY_STOP_WATCHDOG = 4,
  COMM_GNSS_NMEA           = 6,
  COMM_GNSS_PROPRIETARY    = 7,
  EXT_POSE                 = 8,
  EXT_POSE_PACKED          = 9,
  // Removed: LH_ANGLE_STREAM, LH_PERSIST_DATA - Lighthouse hardware not available
} locsrv_t;
void locSrvInit(void);
void locSrvSendRangeFloat(uint8_t id, float range);
#endif 