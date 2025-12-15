#include "xtensa_math.h"
void xtensa_fir_f32(
const xtensa_fir_instance_f32 * S,
float32_t * pSrc,
float32_t * pDst,
uint32_t blockSize)
{
   float32_t *pState = S->pState;                 
   float32_t *pCoeffs = S->pCoeffs;               
   float32_t *pStateCurnt;                        
   float32_t *px, *pb;                            
   uint32_t numTaps = S->numTaps;                 
   uint32_t i, tapCnt, blkCnt;                    
   float32_t acc;
   pStateCurnt = &(S->pState[(numTaps - 1U)]);
   blkCnt = blockSize;
   while (blkCnt > 0U)
   {
      *pStateCurnt++ = *pSrc++;
      acc = 0.0f;
      px = pState;
      pb = pCoeffs;
      i = numTaps;
      do
      {
         acc += *px++ * *pb++;
         i--;
      } while (i > 0U);
      *pDst++ = acc;
      pState = pState + 1;
      blkCnt--;
   }
   pStateCurnt = S->pState;
   tapCnt = numTaps - 1U;
   while (tapCnt > 0U)
   {
      *pStateCurnt++ = *pState++;
      tapCnt--;
   }
}