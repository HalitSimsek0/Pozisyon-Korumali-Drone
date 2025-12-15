#include "xtensa_math.h"
void xtensa_cmplx_mag_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t numSamples)
{
  float32_t realIn, imagIn;                      
  while (numSamples > 0U)
  {
    realIn = *pSrc++;
    imagIn = *pSrc++;
    xtensa_sqrt_f32((realIn * realIn) + (imagIn * imagIn), pDst++);
    numSamples--;
  }
}