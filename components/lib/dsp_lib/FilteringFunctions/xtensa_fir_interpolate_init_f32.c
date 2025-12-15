#include "xtensa_math.h"
xtensa_status xtensa_fir_interpolate_init_f32(
  xtensa_fir_interpolate_instance_f32 * S,
  uint8_t L,
  uint16_t numTaps,
  float32_t * pCoeffs,
  float32_t * pState,
  uint32_t blockSize)
{
  xtensa_status status;
  if ((numTaps % L) != 0U)
  {
    status = XTENSA_MATH_LENGTH_ERROR;
  }
  else
  {
    S->pCoeffs = pCoeffs;
    S->L = L;
    S->phaseLength = numTaps / L;
    memset(pState, 0,
           (blockSize +
            ((uint32_t) S->phaseLength - 1U)) * sizeof(float32_t));
    S->pState = pState;
    status = XTENSA_MATH_SUCCESS;
  }
  return (status);
}
