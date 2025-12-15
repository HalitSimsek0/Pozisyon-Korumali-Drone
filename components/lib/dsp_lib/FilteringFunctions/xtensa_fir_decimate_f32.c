#include "xtensa_math.h"
void xtensa_fir_decimate_f32(
  const xtensa_fir_decimate_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize)
{
  float32_t *pState = S->pState;                 
  float32_t *pCoeffs = S->pCoeffs;               
  float32_t *pStateCurnt;                        
  float32_t *px, *pb;                            
  float32_t sum0;                                
  float32_t x0, c0;                              
  uint32_t numTaps = S->numTaps;                 
  uint32_t i, tapCnt, blkCnt, outBlockSize = blockSize / S->M;  
  pStateCurnt = S->pState + (numTaps - 1U);
  blkCnt = outBlockSize;
  while (blkCnt > 0U)
  {
    i = S->M;
    do
    {
      *pStateCurnt++ = *pSrc++;
    } while (--i);
    sum0 = 0.0f;
    px = pState;
    pb = pCoeffs;
    tapCnt = numTaps;
    while (tapCnt > 0U)
    {
      c0 = *pb++;
      x0 = *px++;
      sum0 += x0 * c0;
      tapCnt--;
    }
    pState = pState + S->M;
    *pDst++ = sum0;
    blkCnt--;
  }
  pStateCurnt = S->pState;
  i = (numTaps - 1U);
  while (i > 0U)
  {
    *pStateCurnt++ = *pState++;
    i--;
  }
}