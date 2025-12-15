#include "xtensa_math.h"
void xtensa_fir_interpolate_f32(
  const xtensa_fir_interpolate_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize)
{
  float32_t *pState = S->pState;                 
  float32_t *pCoeffs = S->pCoeffs;               
  float32_t *pStateCurnt;                        
  float32_t *ptr1, *ptr2;                        
  float32_t sum;                                 
  uint32_t i, blkCnt;                            
  uint16_t phaseLen = S->phaseLength, tapCnt;    
  pStateCurnt = S->pState + (phaseLen - 1U);
  blkCnt = blockSize;
  while (blkCnt > 0U)
  {
    *pStateCurnt++ = *pSrc++;
    i = S->L;
    while (i > 0U)
    {
      sum = 0.0f;
      ptr1 = pState;
      ptr2 = pCoeffs + (i - 1U);
      tapCnt = phaseLen;
      while (tapCnt > 0U)
      {
        sum += *ptr1++ * *ptr2;
        ptr2 += S->L;
        tapCnt--;
      }
      *pDst++ = sum;
      i--;
    }
    pState = pState + 1;
    blkCnt--;
  }
  pStateCurnt = S->pState;
  tapCnt = phaseLen - 1U;
  while (tapCnt > 0U)
  {
    *pStateCurnt++ = *pState++;
    tapCnt--;
  }
}