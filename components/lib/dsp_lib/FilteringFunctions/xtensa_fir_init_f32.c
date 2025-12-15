#include "xtensa_math.h"
void xtensa_fir_init_f32(
  xtensa_fir_instance_f32 * S,
  uint16_t numTaps,
  float32_t * pCoeffs,
  float32_t * pState,
  uint32_t blockSize)
{
  S->numTaps = numTaps;
  S->pCoeffs = pCoeffs;
  memset(pState, 0, (numTaps + (blockSize - 1U)) * sizeof(float32_t));
  S->pState = pState;
}