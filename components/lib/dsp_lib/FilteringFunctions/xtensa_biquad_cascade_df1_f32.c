#include "xtensa_math.h"
void xtensa_biquad_cascade_df1_f32(
  const xtensa_biquad_casd_df1_inst_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize)
{
  float32_t *pIn = pSrc;                         
  float32_t *pOut = pDst;                        
  float32_t *pState = S->pState;                 
  float32_t *pCoeffs = S->pCoeffs;               
  float32_t acc;                                 
  float32_t b0, b1, b2, a1, a2;                  
  float32_t Xn1, Xn2, Yn1, Yn2;                  
  float32_t Xn;                                  
  uint32_t sample, stage = S->numStages;         
  do
  {
    b0 = *pCoeffs++;
    b1 = *pCoeffs++;
    b2 = *pCoeffs++;
    a1 = *pCoeffs++;
    a2 = *pCoeffs++;
    Xn1 = pState[0];
    Xn2 = pState[1];
    Yn1 = pState[2];
    Yn2 = pState[3];
    sample = blockSize;
    while (sample > 0U)
    {
      Xn = *pIn++;
      acc = (b0 * Xn) + (b1 * Xn1) + (b2 * Xn2) + (a1 * Yn1) + (a2 * Yn2);
      *pOut++ = acc;
      Xn2 = Xn1;
      Xn1 = Xn;
      Yn2 = Yn1;
      Yn1 = acc;
      sample--;
    }
    *pState++ = Xn1;
    *pState++ = Xn2;
    *pState++ = Yn1;
    *pState++ = Yn2;
    pIn = pDst;
    pOut = pDst;
    stage--;
  } while (stage > 0U);
}