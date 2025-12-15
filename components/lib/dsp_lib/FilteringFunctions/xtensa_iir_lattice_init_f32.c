#include "xtensa_math.h"
void xtensa_iir_lattice_init_f32(
  xtensa_iir_lattice_instance_f32 * S,
  uint16_t numStages,
  float32_t * pkCoeffs,
  float32_t * pvCoeffs,
  float32_t * pState,
  uint32_t blockSize)
{
  S->numStages = numStages;
  S->pkCoeffs = pkCoeffs;
  S->pvCoeffs = pvCoeffs;
  memset(pState, 0, (numStages + blockSize) * sizeof(float32_t));
  S->pState = pState;
}