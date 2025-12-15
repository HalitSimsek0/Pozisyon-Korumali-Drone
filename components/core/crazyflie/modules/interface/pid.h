#ifndef PID_H_
#define PID_H_
#include <stdbool.h>
#include "filter.h"
#ifdef CONFIG_TARGET_ESP32_S2_DRONE_V1_2
  #define PID_ROLL_RATE_KP  85.0   
  #define PID_ROLL_RATE_KI  50.0   
  #define PID_ROLL_RATE_KD  23.0   
  #define PID_ROLL_RATE_INTEGRATION_LIMIT    25.0  
  #define PID_PITCH_RATE_KP  85.0  
  #define PID_PITCH_RATE_KI  50.0
  #define PID_PITCH_RATE_KD  23.0
  #define PID_PITCH_RATE_INTEGRATION_LIMIT   25.0
  #define PID_YAW_RATE_KP  85.0    
  #define PID_YAW_RATE_KI  45.0    
  #define PID_YAW_RATE_KD  0.0     
  #define PID_YAW_RATE_INTEGRATION_LIMIT     25.0  
  #define PID_ROLL_KP  4.5   
  #define PID_ROLL_KI  2.8   
  #define PID_ROLL_KD  0.0
  #define PID_ROLL_INTEGRATION_LIMIT    20.0
  #define PID_PITCH_KP  4.5  
  #define PID_PITCH_KI  2.8
  #define PID_PITCH_KD  0.0
  #define PID_PITCH_INTEGRATION_LIMIT   20.0
  #define PID_YAW_KP  6.0
  #define PID_YAW_KI  1.0
  #define PID_YAW_KD  0.35
  #define PID_YAW_INTEGRATION_LIMIT     360.0
  #define DEFAULT_PID_INTEGRATION_LIMIT 5000.0
  #define DEFAULT_PID_OUTPUT_LIMIT      0.0
#else
  #define PID_ROLL_RATE_KP  250.0
  #define PID_ROLL_RATE_KI  500.0
  #define PID_ROLL_RATE_KD  2.5
  #define PID_ROLL_RATE_INTEGRATION_LIMIT    33.3
  #define PID_PITCH_RATE_KP  250.0
  #define PID_PITCH_RATE_KI  500.0
  #define PID_PITCH_RATE_KD  2.5
  #define PID_PITCH_RATE_INTEGRATION_LIMIT   33.3
  #define PID_YAW_RATE_KP  120.0
  #define PID_YAW_RATE_KI  16.7
  #define PID_YAW_RATE_KD  0.0
  #define PID_YAW_RATE_INTEGRATION_LIMIT     166.7
  #define PID_ROLL_KP  5.9
  #define PID_ROLL_KI  2.9
  #define PID_ROLL_KD  0.0
  #define PID_ROLL_INTEGRATION_LIMIT    20.0
  #define PID_PITCH_KP  5.9
  #define PID_PITCH_KI  2.9
  #define PID_PITCH_KD  0.0
  #define PID_PITCH_INTEGRATION_LIMIT   20.0
  #define PID_YAW_KP  6.0
  #define PID_YAW_KI  1.0
  #define PID_YAW_KD  0.35
  #define PID_YAW_INTEGRATION_LIMIT     360.0
  #define DEFAULT_PID_INTEGRATION_LIMIT 5000.0
  #define DEFAULT_PID_OUTPUT_LIMIT      0.0
#endif 
typedef enum {
  ITERM_RELAX_OFF = 0,
  ITERM_RELAX_RP = 1,
  ITERM_RELAX_RPY = 2,
  ITERM_RELAX_RP_INC = 3,
  ITERM_RELAX_RPY_INC = 4
} ItermRelaxType;
typedef struct
{
  float desired;      
  float error;        
  float prevError;    
  float integ;        
  float deriv;        
  float kp;           
  float ki;           
  float kd;           
  float kf;           // Feedforward gain
  float outP;         
  float outI;         
  float outD;         
  float outF;         // Feedforward output
  float iLimit;       
  float outputLimit;  
  float dt;           
  float rate;         // 1/dt
  float pScale;       // P term scale
  float iScale;       // I term scale
  float dScale;       // D term scale
  float fScale;       // F term scale
  float itermRelax;   // I-term relax mode
  float itermRelaxFactor;
  float itermRelaxBase;
  float iTermError;
  float prevMeasurement;
  float prevSetpoint;
  bool outputSaturated;
  lpf2pData dFilter;  
  lpf2pData dFilter2;
  lpf2pData dtermNotchFilter;
  lpf2pData ptermFilter;
  lpf2pData ftermFilter;
  lpf2pData itermRelaxFilter;
  bool enableDFilter; 
  bool enablePFilter;
  bool enableFFilter;
} PidObject;
 void pidInit(PidObject* pid, const float desired, const float kp,
              const float ki, const float kd, const float dt,
              const float samplingRate, const float cutoffFreq,
              bool enableDFilter);
void pidSetIntegralLimit(PidObject* pid, const float limit);
void pidReset(PidObject* pid);
float pidUpdate(PidObject* pid, const float measured, const bool updateError);
void pidSetDesired(PidObject* pid, const float desired);
float pidGetDesired(PidObject* pid);
bool pidIsActive(PidObject* pid);
void pidSetError(PidObject* pid, const float error);
void pidSetKp(PidObject* pid, const float kp);
void pidSetKi(PidObject* pid, const float ki);
void pidSetKd(PidObject* pid, const float kd);
void pidSetKf(PidObject* pid, const float kf);
void pidSetDt(PidObject* pid, const float dt);
void pidInitFilters(PidObject* pid, float samplingRate, float dtermCutoff, float ptermCutoff, float ftermCutoff);
#endif 