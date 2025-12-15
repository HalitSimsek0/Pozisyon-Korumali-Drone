#include "xtensa_math.h"
void xtensa_cmplx_conj_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t numSamples)
{
  uint32_t blkCnt;                               
  blkCnt = numSamples;
  while (blkCnt > 0U)
  {
    *pDst++ = *pSrc++;
    *pDst++ = -*pSrc++;
    blkCnt--;
  }
}