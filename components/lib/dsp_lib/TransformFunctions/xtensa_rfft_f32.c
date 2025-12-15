#include "xtensa_math.h"
extern void xtensa_radix4_butterfly_f32(
    float32_t * pSrc,
    uint16_t fftLen,
    float32_t * pCoef,
    uint16_t twidCoefModifier);
extern void xtensa_radix4_butterfly_inverse_f32(
    float32_t * pSrc,
    uint16_t fftLen,
    float32_t * pCoef,
    uint16_t twidCoefModifier,
    float32_t onebyfftLen);
extern void xtensa_bitreversal_f32(
    float32_t * pSrc,
    uint16_t fftSize,
    uint16_t bitRevFactor,
    uint16_t * pBitRevTab);
void xtensa_split_rfft_f32(
  float32_t * pSrc,
  uint32_t fftLen,
  float32_t * pATable,
  float32_t * pBTable,
  float32_t * pDst,
  uint32_t modifier);
void xtensa_split_rifft_f32(
  float32_t * pSrc,
  uint32_t fftLen,
  float32_t * pATable,
  float32_t * pBTable,
  float32_t * pDst,
  uint32_t modifier);
void xtensa_rfft_f32(
  const xtensa_rfft_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst)
{
  const xtensa_cfft_radix4_instance_f32 *S_CFFT = S->pCfft;
  if (S->ifftFlagR == 1U)
  {
    xtensa_split_rifft_f32(pSrc, S->fftLenBy2, S->pTwiddleAReal,
                        S->pTwiddleBReal, pDst, S->twidCoefRModifier);
    xtensa_radix4_butterfly_inverse_f32(pDst, S_CFFT->fftLen,
                                     S_CFFT->pTwiddle,
                                     S_CFFT->twidCoefModifier,
                                     S_CFFT->onebyfftLen);
    if (S->bitReverseFlagR == 1U)
    {
      xtensa_bitreversal_f32(pDst, S_CFFT->fftLen,
                          S_CFFT->bitRevFactor, S_CFFT->pBitRevTable);
    }
  }
  else
  {
    xtensa_radix4_butterfly_f32(pSrc, S_CFFT->fftLen,
                             S_CFFT->pTwiddle, S_CFFT->twidCoefModifier);
    if (S->bitReverseFlagR == 1U)
    {
      xtensa_bitreversal_f32(pSrc, S_CFFT->fftLen,
                          S_CFFT->bitRevFactor, S_CFFT->pBitRevTable);
    }
    xtensa_split_rfft_f32(pSrc, S->fftLenBy2, S->pTwiddleAReal,
                       S->pTwiddleBReal, pDst, S->twidCoefRModifier);
  }
}
void xtensa_split_rfft_f32(
  float32_t * pSrc,
  uint32_t fftLen,
  float32_t * pATable,
  float32_t * pBTable,
  float32_t * pDst,
  uint32_t modifier)
{
  uint32_t i;                                    
  float32_t outR, outI;                          
  float32_t *pCoefA, *pCoefB;                    
  float32_t CoefA1, CoefA2, CoefB1;              
  float32_t *pDst1 = &pDst[2], *pDst2 = &pDst[(4U * fftLen) - 1U];      
  float32_t *pSrc1 = &pSrc[2], *pSrc2 = &pSrc[(2U * fftLen) - 1U];      
  pCoefA = &pATable[modifier * 2U];
  pCoefB = &pBTable[modifier * 2U];
  i = fftLen - 1U;
  while (i > 0U)
  {
    CoefA1 = *pCoefA++;
    CoefA2 = *pCoefA;
    outR = *pSrc1 * CoefA1;
    outI = *pSrc1++ * CoefA2;
    outR -= (*pSrc1 + *pSrc2) * CoefA2;
    outI += *pSrc1++ * CoefA1;
    CoefB1 = *pCoefB;
    outI -= *pSrc2-- * CoefB1;
    outI -= *pSrc2 * CoefA2;
    outR += *pSrc2-- * CoefB1;
    *pDst1++ = outR;
    *pDst1++ = outI;
    *pDst2-- = -outI;
    *pDst2-- = outR;
    pCoefB = pCoefB + (modifier * 2U);
    pCoefA = pCoefA + ((modifier * 2U) - 1U);
    i--;
  }
  pDst[2U * fftLen] = pSrc[0] - pSrc[1];
  pDst[(2U * fftLen) + 1U] = 0.0f;
  pDst[0] = pSrc[0] + pSrc[1];
  pDst[1] = 0.0f;
}
void xtensa_split_rifft_f32(
  float32_t * pSrc,
  uint32_t fftLen,
  float32_t * pATable,
  float32_t * pBTable,
  float32_t * pDst,
  uint32_t modifier)
{
  float32_t outR, outI;                          
  float32_t *pCoefA, *pCoefB;                    
  float32_t CoefA1, CoefA2, CoefB1;              
  float32_t *pSrc1 = &pSrc[0], *pSrc2 = &pSrc[(2U * fftLen) + 1U];
  pCoefA = &pATable[0];
  pCoefB = &pBTable[0];
  while (fftLen > 0U)
  {
    CoefA1 = *pCoefA++;
    CoefA2 = *pCoefA;
    outR = *pSrc1 * CoefA1;
    outI = -(*pSrc1++) * CoefA2;
    outR += (*pSrc1 + *pSrc2) * CoefA2;
    outI += (*pSrc1++) * CoefA1;
    CoefB1 = *pCoefB;
    outI -= *pSrc2-- * CoefB1;
    outR += *pSrc2 * CoefB1;
    outI += *pSrc2-- * CoefA2;
    *pDst++ = outR;
    *pDst++ = outI;
    pCoefB = pCoefB + (modifier * 2U);
    pCoefA = pCoefA + ((modifier * 2U) - 1U);
    fftLen--;
  }
}