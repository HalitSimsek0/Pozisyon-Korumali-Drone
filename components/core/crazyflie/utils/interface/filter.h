#ifndef FILTER_H_
#define FILTER_H_
#include <stdint.h>
#include "math.h"
#define IIR_SHIFT         8
int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt);
typedef struct {
  float a1;
  float a2;
  float b0;
  float b1;
  float b2;
  float delay_element_1;
  float delay_element_2;
} lpf2pData;
void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
float lpf2pApply(lpf2pData* lpfData, float sample);
float lpf2pReset(lpf2pData* lpfData, float sample);
struct SecondOrderLowPass {
  float a[2]; 
  float b[2]; 
  float i[2]; 
  float o[2]; 
};
static inline void init_second_order_low_pass(struct SecondOrderLowPass *filter, float tau, float Q, float sample_time,
    float value)
{
  float K = tanf(sample_time / (2.0f * tau));
  float poly = K * K + K / Q + 1.0f;
  filter->a[0] = 2.0f * (K * K - 1.0f) / poly;
  filter->a[1] = (K * K - K / Q + 1.0f) / poly;
  filter->b[0] = K * K / poly;
  filter->b[1] = 2.0f * filter->b[0];
  filter->i[0] = filter->i[1] = filter->o[0] = filter->o[1] = value;
}
static inline float update_second_order_low_pass(struct SecondOrderLowPass *filter, float value)
{
  float out = filter->b[0] * value
              + filter->b[1] * filter->i[0]
              + filter->b[0] * filter->i[1]
              - filter->a[0] * filter->o[0]
              - filter->a[1] * filter->o[1];
  filter->i[1] = filter->i[0];
  filter->i[0] = value;
  filter->o[1] = filter->o[0];
  filter->o[0] = out;
  return out;
}
static inline float get_second_order_low_pass(struct SecondOrderLowPass *filter)
{
  return filter->o[0];
}
struct SecondOrderLowPass_int {
  int32_t a[2]; 
  int32_t b[2]; 
  int32_t i[2]; 
  int32_t o[2]; 
  int32_t loop_gain; 
};
typedef struct SecondOrderLowPass Butterworth2LowPass;
static inline void init_butterworth_2_low_pass(Butterworth2LowPass *filter, float tau, float sample_time, float value)
{
  init_second_order_low_pass((struct SecondOrderLowPass *)filter, tau, 0.7071, sample_time, value);
}
static inline float update_butterworth_2_low_pass(Butterworth2LowPass *filter, float value)
{
  return update_second_order_low_pass((struct SecondOrderLowPass *)filter, value);
}
static inline float get_butterworth_2_low_pass(Butterworth2LowPass *filter)
{
  return filter->o[0];
}
#endif 