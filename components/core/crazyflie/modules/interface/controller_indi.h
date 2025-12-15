#ifndef __CONTROLLER_INDI_H__
#define __CONTROLLER_INDI_H__
#include "stabilizer_types.h"
#include "filter.h"
#include "math3d.h"
#include "log.h"
#include "param.h"
#include "position_controller.h"
#include "attitude_controller.h"
#include "position_controller_indi.h"
#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)
#define STABILIZATION_INDI_FILT_CUTOFF 8.0f
#define STABILIZATION_INDI_FILT_CUTOFF_R STABILIZATION_INDI_FILT_CUTOFF
#define STABILIZATION_INDI_G1_P 0.0066146f
#define STABILIZATION_INDI_G1_Q 0.0052125f
#define STABILIZATION_INDI_G1_R -0.001497f
#define STABILIZATION_INDI_G2_R 0.000043475f
#define STABILIZATION_INDI_REF_ERR_P 24.0f
#define STABILIZATION_INDI_REF_ERR_Q 24.0f
#define STABILIZATION_INDI_REF_ERR_R 24.0f
#define STABILIZATION_INDI_REF_RATE_P 14.0f
#define STABILIZATION_INDI_REF_RATE_Q 14.0f
#define STABILIZATION_INDI_REF_RATE_R 14.0f
#define STABILIZATION_INDI_ACT_DYN_P 0.03149f
#define STABILIZATION_INDI_ACT_DYN_Q 0.03149f
#define STABILIZATION_INDI_ACT_DYN_R 0.03149f
struct FloatRates {
  float p; 
  float q; 
  float r; 
};
struct ReferenceSystem {
  float err_p;
  float err_q;
  float err_r;
  float rate_p;
  float rate_q;
  float rate_r;
};
struct IndiVariables {
  float thrust;
  struct FloatRates angular_accel_ref;
  struct FloatRates du;
  struct FloatRates u_in;
  struct FloatRates u_act_dyn;
  float rate_d[3];
  Butterworth2LowPass u[3];
  Butterworth2LowPass rate[3];
  struct FloatRates g1;
  float g2;
  struct ReferenceSystem reference_acceleration;
  struct FloatRates act_dyn;
  float filt_cutoff;
  float filt_cutoff_r;
};
void controllerINDIInit(void);
bool controllerINDITest(void);
void controllerINDI(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);
#endif 