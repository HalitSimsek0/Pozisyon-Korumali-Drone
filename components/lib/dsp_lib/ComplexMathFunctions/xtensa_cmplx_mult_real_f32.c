#include "xtensa_math.h"
void xtensa_cmplx_mult_real_f32(
  float32_t * pSrcCmplx,
  float32_t * pSrcReal,
  float32_t * pCmplxDst,
  uint32_t numSamples)
{
  float32_t in;                                  
  uint32_t blkCnt;                               
  blkCnt = numSamples;
  while (blkCnt > 0U)
  {
    in = *pSrcReal++;
    *pCmplxDst++ = (*pSrcCmplx++) * (in);
    *pCmplxDst++ = (*pSrcCmplx++) * (in);
    blkCnt--;
  }
}