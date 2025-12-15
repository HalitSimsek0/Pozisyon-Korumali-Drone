#ifndef __STABILIZER_TYPES_H__
#define __STABILIZER_TYPES_H__
#include <stdint.h>
#include <stdbool.h>
#include "imu_types.h"
typedef struct attitude_s {
  uint32_t timestamp;  
  float roll;
  float pitch;
  float yaw;
} attitude_t;
#define vec3d_size 3
typedef float vec3d[vec3d_size];
typedef float mat3d[vec3d_size][vec3d_size];
struct vec3_s {
  uint32_t timestamp; 
  float x;
  float y;
  float z;
};
typedef struct vec3_s vector_t;
typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;
typedef struct quaternion_s {
  uint32_t timestamp;
  union {
    struct {
      float q0;
      float q1;
      float q2;
      float q3;
    };
    struct {
      float x;
      float y;
      float z;
      float w;
    };
  };
} quaternion_t;
// Removed: tdoaMeasurement_t typedef - TDOA hardware not available
// Removed: baro_t typedef - Barometer hardware not available
typedef struct positionMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  float stdDev;
} positionMeasurement_t;
typedef struct poseMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  quaternion_t quat;
  float stdDevPos;
  float stdDevQuat;
} poseMeasurement_t;
typedef struct distanceMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  float distance;
  float stdDev;
} distanceMeasurement_t;
typedef struct zDistance_s {
  uint32_t timestamp;
  float distance;           
} zDistance_t;
typedef struct sensorData_s {
  Axis3f acc;               
  Axis3f gyro;              
  Axis3f mag;               
  // Removed: baro_t baro - Barometer hardware not available
#ifdef LOG_SEC_IMU
  Axis3f accSec;            
  Axis3f gyroSec;           
#endif
  uint64_t interruptTimestamp;
} sensorData_t;
typedef struct state_s {
  attitude_t attitude;      
  quaternion_t attitudeQuaternion;
  point_t position;         
  velocity_t velocity;      
  acc_t acc;                
} state_t;
typedef struct control_s {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  float thrust;
} control_t;
typedef enum mode_e {
  modeDisable = 0,
  modeAbs,
  modeVelocity
} stab_mode_t;
typedef struct setpoint_s {
  uint32_t timestamp;
  attitude_t attitude;      
  attitude_t attitudeRate;  
  quaternion_t attitudeQuaternion;
  float thrust;
  point_t position;         
  velocity_t velocity;      
  acc_t acceleration;       
  bool velocity_body;       
  struct {
    stab_mode_t x;
    stab_mode_t y;
    stab_mode_t z;
    stab_mode_t roll;
    stab_mode_t pitch;
    stab_mode_t yaw;
    stab_mode_t quat;
  } mode;
} setpoint_t;
typedef struct estimate_s {
  uint32_t timestamp; 
  point_t position;
} estimate_t;
typedef struct setpointZ_s {
  float z;
  bool isUpdate; 
} setpointZ_t;
typedef struct flowMeasurement_s {
  uint32_t timestamp;
  union {
    struct {
      float dpixelx;  
      float dpixely;  
    };
    float dpixel[2];  
  };
  float stdDevX;      
  float stdDevY;      
  float dt;           
} flowMeasurement_t;
typedef struct tofMeasurement_s {
  uint32_t timestamp;
  float distance;
  float stdDev;
} tofMeasurement_t;
typedef struct heightMeasurement_s {
  uint32_t timestamp;
  float height;
  float stdDev;
} heightMeasurement_t;
typedef struct {
  uint32_t timestamp;
  float yawError;
  float stdDev;
} yawErrorMeasurement_t;
#define RATE_1000_HZ 1000
#define RATE_500_HZ 500
#define RATE_250_HZ 250
#define RATE_100_HZ 100
#define RATE_50_HZ 50
#define RATE_25_HZ 25
#define RATE_MAIN_LOOP RATE_1000_HZ
#define ATTITUDE_RATE RATE_1000_HZ  
#define POSITION_RATE RATE_100_HZ
#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (RATE_MAIN_LOOP / RATE_HZ)) == 0)
#endif