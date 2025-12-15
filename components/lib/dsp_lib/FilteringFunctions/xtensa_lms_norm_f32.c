#include "xtensa_math.h"
void xtensa_lms_norm_f32(
  xtensa_lms_norm_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pRef,
  float32_t * pOut,
  float32_t * pErr,
  uint32_t blockSize)
{
  float32_t *pState = S->pState;                 
  float32_t *pCoeffs = S->pCoeffs;               
  float32_t *pStateCurnt;                        
  float32_t *px, *pb;                            
  float32_t mu = S->mu;                          
  uint32_t numTaps = S->numTaps;                 
  uint32_t tapCnt, blkCnt;                       
  float32_t energy;                              
  float32_t sum, e, d;                           
  float32_t w, x0, in;                           
  e = 0.0f;
  d = 0.0f;
  w = 0.0f;
  energy = S->energy;
  x0 = S->x0;
  pStateCurnt = &(S->pState[(numTaps - 1U)]);
  blkCnt = blockSize;
  while (blkCnt > 0U)
  {
    *pStateCurnt++ = *pSrc;
    px = pState;
    pb = pCoeffs;
    in = *pSrc++;
    energy -= x0 * x0;
    energy += in * in;
    sum = 0.0f;
    tapCnt = numTaps;
    while (tapCnt > 0U)
    {
      sum += (*px++) * (*pb++);
      tapCnt--;
    }
    *pOut++ = sum;
    d = (float32_t) (*pRef++);
    e = d - sum;
    *pErr++ = e;
    w = (e * mu) / (energy + 0.000000119209289f);
    px = pState;
    pb = pCoeffs;
    tapCnt = numTaps;
    while (tapCnt > 0U)
    {
      *pb += w * (*px++);
      pb++;
      tapCnt--;
    }
    x0 = *pState;
    pState = pState + 1;
    blkCnt--;
  }
  S->energy = energy;
  S->x0 = x0;
  pStateCurnt = S->pState;
  tapCnt = (numTaps - 1U);
  while (tapCnt > 0U)
  {
    *pStateCurnt++ = *pState++;
    tapCnt--;
  }
}