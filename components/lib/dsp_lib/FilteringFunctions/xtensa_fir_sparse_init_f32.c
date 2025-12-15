#include "xtensa_math.h"
void xtensa_fir_sparse_init_f32(
  xtensa_fir_sparse_instance_f32 * S,
  uint16_t numTaps,
  float32_t * pCoeffs,
  float32_t * pState,
  int32_t * pTapDelay,
  uint16_t maxDelay,
  uint32_t blockSize)
{
  S->numTaps = numTaps;
  S->pCoeffs = pCoeffs;
  S->pTapDelay = pTapDelay;
  S->maxDelay = maxDelay;
  S->stateIndex = 0U;
  memset(pState, 0, (maxDelay + blockSize) * sizeof(float32_t));
  S->pState = pState;
}