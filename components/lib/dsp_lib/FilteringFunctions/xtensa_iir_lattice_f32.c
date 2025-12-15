#include "xtensa_math.h"
void xtensa_iir_lattice_f32(
  const xtensa_iir_lattice_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize)
{
  float32_t fcurr, fnext = 0, gcurr, gnext;      
  float32_t acc;                                 
  uint32_t blkCnt, tapCnt;                       
  float32_t *px1, *px2, *pk, *pv;                
  uint32_t numStages = S->numStages;             
  float32_t *pState;                             
  float32_t *pStateCurnt;                        
  blkCnt = blockSize;
  pState = &S->pState[0];
  while (blkCnt > 0U)
  {
    fcurr = *pSrc++;
    px1 = pState;
    px2 = pState;
    acc = 0.0f;
    pv = &S->pvCoeffs[0];
    pk = &S->pkCoeffs[0];
    tapCnt = numStages;
    while (tapCnt > 0U)
    {
      gcurr = *px1++;
      fnext = fcurr - ((*pk) * gcurr);
      gnext = (fnext * (*pk++)) + gcurr;
      acc += (gnext * (*pv++));
      *px2++ = gnext;
      fcurr = fnext;
      tapCnt--;
    }
    acc += (fnext * (*pv));
    *px2++ = fnext;
    *pDst++ = acc;
    pState = pState + 1U;
    blkCnt--;
  }
  pStateCurnt = &S->pState[0];
  pState = &S->pState[blockSize];
  tapCnt = numStages;
  while (tapCnt > 0U)
  {
    *pStateCurnt++ = *pState++;
    tapCnt--;
  }
}