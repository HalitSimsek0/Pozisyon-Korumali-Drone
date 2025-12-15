#include "xtensa_math.h"
void xtensa_cmplx_mag_squared_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t numSamples)
{
  float32_t real, imag;                          
  uint32_t blkCnt;                               
  blkCnt = numSamples;
  while (blkCnt > 0U)
  {
    real = *pSrc++;
    imag = *pSrc++;
    *pDst++ = (real * real) + (imag * imag);
    blkCnt--;
  }
}