#include "xtensa_math.h"
void xtensa_biquad_cascade_df2T_f32(
const xtensa_biquad_cascade_df2T_instance_f32 * S,
float32_t * pSrc,
float32_t * pDst,
uint32_t blockSize)
{
   float32_t *pIn = pSrc;                         
   float32_t *pOut = pDst;                        
   float32_t *pState = S->pState;                 
   float32_t *pCoeffs = S->pCoeffs;               
   float32_t acc1;                                
   float32_t b0, b1, b2, a1, a2;                  
   float32_t Xn1;                                 
   float32_t d1, d2;                              
   uint32_t sample, stage = S->numStages;         
   do
   {
      b0 = *pCoeffs++;
      b1 = *pCoeffs++;
      b2 = *pCoeffs++;
      a1 = *pCoeffs++;
      a2 = *pCoeffs++;
      d1 = pState[0];
      d2 = pState[1];
      sample = blockSize;
      while (sample > 0U)
      {
         Xn1 = *pIn++;
         acc1 = (b0 * Xn1) + d1;
         *pOut++ = acc1;
         d1 = ((b1 * Xn1) + (a1 * acc1)) + d2;
         d2 = (b2 * Xn1) + (a2 * acc1);
         sample--;
      }
      *pState++ = d1;
      *pState++ = d2;
      pIn = pDst;
      pOut = pDst;
      stage--;
   } while (stage > 0U);
}