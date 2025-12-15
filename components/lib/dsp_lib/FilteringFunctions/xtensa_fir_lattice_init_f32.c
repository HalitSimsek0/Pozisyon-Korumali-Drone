#include "xtensa_math.h"
void xtensa_fir_lattice_init_f32(
  xtensa_fir_lattice_instance_f32 * S,
  uint16_t numStages,
  float32_t * pCoeffs,
  float32_t * pState)
{
  S->numStages = numStages;
  S->pCoeffs = pCoeffs;
  memset(pState, 0, (numStages) * sizeof(float32_t));
  S->pState = pState;
}