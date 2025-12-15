#include "xtensa_math.h"
void xtensa_lms_norm_init_f32(
  xtensa_lms_norm_instance_f32 * S,
  uint16_t numTaps,
  float32_t * pCoeffs,
  float32_t * pState,
  float32_t mu,
  uint32_t blockSize)
{
  S->numTaps = numTaps;
  S->pCoeffs = pCoeffs;
  memset(pState, 0, (numTaps + (blockSize - 1U)) * sizeof(float32_t));
  S->pState = pState;
  S->mu = mu;
  S->energy = 0.0f;
  S->x0 = 0.0f;
}