#include "xtensa_math.h"
void xtensa_biquad_cascade_stereo_df2T_init_f32(
  xtensa_biquad_cascade_stereo_df2T_instance_f32 * S,
  uint8_t numStages,
  float32_t * pCoeffs,
  float32_t * pState)
{
  S->numStages = numStages;
  S->pCoeffs = pCoeffs;
  memset(pState, 0, (4U * (uint32_t) numStages) * sizeof(float32_t));
  S->pState = pState;
}