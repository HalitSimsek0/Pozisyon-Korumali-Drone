#include "xtensa_math.h"
void xtensa_radix2_butterfly_f32(
  float32_t * pSrc,
  uint32_t fftLen,
  float32_t * pCoef,
  uint16_t twidCoefModifier);
void xtensa_radix2_butterfly_inverse_f32(
  float32_t * pSrc,
  uint32_t fftLen,
  float32_t * pCoef,
  uint16_t twidCoefModifier,
  float32_t onebyfftLen);
extern void xtensa_bitreversal_f32(
    float32_t * pSrc,
    uint16_t fftSize,
    uint16_t bitRevFactor,
    uint16_t * pBitRevTab);
void xtensa_cfft_radix2_f32(
const xtensa_cfft_radix2_instance_f32 * S,
float32_t * pSrc)
{
   if (S->ifftFlag == 1U)
   {
      xtensa_radix2_butterfly_inverse_f32(pSrc, S->fftLen, S->pTwiddle,
      S->twidCoefModifier, S->onebyfftLen);
   }
   else
   {
      xtensa_radix2_butterfly_f32(pSrc, S->fftLen, S->pTwiddle,
      S->twidCoefModifier);
   }
   if (S->bitReverseFlag == 1U)
   {
      xtensa_bitreversal_f32(pSrc, S->fftLen, S->bitRevFactor, S->pBitRevTable);
   }
}
void xtensa_radix2_butterfly_f32(
float32_t * pSrc,
uint32_t fftLen,
float32_t * pCoef,
uint16_t twidCoefModifier)
{
   uint32_t i, j, k, l;
   uint32_t n1, n2, ia;
   float32_t xt, yt, cosVal, sinVal;
   float32_t p0, p1, p2, p3;
   float32_t a0, a1;
   n2 = fftLen;
   for (k = fftLen; k > 1; k = k >> 1)
   {
      n1 = n2;
      n2 = n2 >> 1;
      ia = 0;
      j = 0;
      do
      {
         cosVal = pCoef[ia * 2];
         sinVal = pCoef[(ia * 2) + 1];
         ia += twidCoefModifier;
         i = j;
         do
         {
            l = i + n2;
            a0 = pSrc[2 * i] + pSrc[2 * l];
            xt = pSrc[2 * i] - pSrc[2 * l];
            yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
            a1 = pSrc[2 * l + 1] + pSrc[2 * i + 1];
            p0 = xt * cosVal;
            p1 = yt * sinVal;
            p2 = yt * cosVal;
            p3 = xt * sinVal;
            pSrc[2 * i] = a0;
            pSrc[2 * i + 1] = a1;
            pSrc[2 * l]     = p0 + p1;
            pSrc[2 * l + 1] = p2 - p3;
            i += n1;
         } while (i < fftLen);
         j++;
      } while (j < n2);
      twidCoefModifier <<= 1U;
   }
}
void xtensa_radix2_butterfly_inverse_f32(
float32_t * pSrc,
uint32_t fftLen,
float32_t * pCoef,
uint16_t twidCoefModifier,
float32_t onebyfftLen)
{
   uint32_t i, j, k, l;
   uint32_t n1, n2, ia;
   float32_t xt, yt, cosVal, sinVal;
   float32_t p0, p1, p2, p3;
   float32_t a0, a1;
   n2 = fftLen;
   for (k = fftLen; k > 2; k = k >> 1)
   {
      n1 = n2;
      n2 = n2 >> 1;
      ia = 0;
      j = 0;
      do
      {
         cosVal = pCoef[ia * 2];
         sinVal = pCoef[(ia * 2) + 1];
         ia = ia + twidCoefModifier;
         i = j;
         do
         {
            l = i + n2;
            a0 = pSrc[2 * i] + pSrc[2 * l];
            xt = pSrc[2 * i] - pSrc[2 * l];
            yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
            a1 = pSrc[2 * l + 1] + pSrc[2 * i + 1];
            p0 = xt * cosVal;
            p1 = yt * sinVal;
            p2 = yt * cosVal;
            p3 = xt * sinVal;
            pSrc[2 * i] = a0;
            pSrc[2 * i + 1] = a1;
            pSrc[2 * l]     = p0 - p1;
            pSrc[2 * l + 1] = p2 + p3;
            i += n1;
         } while ( i < fftLen );                    
         j++;
      } while ( j < n2 );                      
      twidCoefModifier = twidCoefModifier << 1U;
   }                             
   n1 = n2;
   n2 = n2 >> 1;
   for (i = 0; i < fftLen; i += n1)
   {
      l = i + n2;
      a0 = pSrc[2 * i] + pSrc[2 * l];
      xt = pSrc[2 * i] - pSrc[2 * l];
      a1 = pSrc[2 * l + 1] + pSrc[2 * i + 1];
      yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
      p0 = a0 * onebyfftLen;
      p2 = xt * onebyfftLen;
      p1 = a1 * onebyfftLen;
      p3 = yt * onebyfftLen;
      pSrc[2 * i] = p0;
      pSrc[2U * l] = p2;
      pSrc[2 * i + 1] = p1;
      pSrc[2U * l + 1U] = p3;
   }                             
}
