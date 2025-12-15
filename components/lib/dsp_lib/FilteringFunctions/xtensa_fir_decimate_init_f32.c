#include "xtensa_math.h"
xtensa_status xtensa_fir_decimate_init_f32(
  xtensa_fir_decimate_instance_f32 * S,
  uint16_t numTaps,
  uint8_t M,
  float32_t * pCoeffs,
  float32_t * pState,
  uint32_t blockSize)
{
  xtensa_status status;
  if ((blockSize % M) != 0U)
  {
    status = XTENSA_MATH_LENGTH_ERROR;
  }
  else
  {
    S->numTaps = numTaps;
    S->pCoeffs = pCoeffs;
    memset(pState, 0, (numTaps + (blockSize - 1U)) * sizeof(float32_t));
    S->pState = pState;
    S->M = M;
    status = XTENSA_MATH_SUCCESS;
  }
  return (status);
}