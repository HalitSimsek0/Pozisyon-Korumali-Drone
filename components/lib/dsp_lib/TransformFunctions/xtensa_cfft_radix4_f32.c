#include "xtensa_math.h"
extern void xtensa_bitreversal_f32(
float32_t * pSrc,
uint16_t fftSize,
uint16_t bitRevFactor,
uint16_t * pBitRevTab);
void xtensa_radix4_butterfly_f32(
float32_t * pSrc,
uint16_t fftLen,
float32_t * pCoef,
uint16_t twidCoefModifier);
void xtensa_radix4_butterfly_inverse_f32(
float32_t * pSrc,
uint16_t fftLen,
float32_t * pCoef,
uint16_t twidCoefModifier,
float32_t onebyfftLen);
void xtensa_cfft_radix4_f32(
  const xtensa_cfft_radix4_instance_f32 * S,
  float32_t * pSrc)
{
   if (S->ifftFlag == 1U)
   {
      xtensa_radix4_butterfly_inverse_f32(pSrc, S->fftLen, S->pTwiddle, S->twidCoefModifier, S->onebyfftLen);
   }
   else
   {
      xtensa_radix4_butterfly_f32(pSrc, S->fftLen, S->pTwiddle, S->twidCoefModifier);
   }
   if (S->bitReverseFlag == 1U)
   {
      xtensa_bitreversal_f32(pSrc, S->fftLen, S->bitRevFactor, S->pBitRevTable);
   }
}
void xtensa_radix4_butterfly_f32(
float32_t * pSrc,
uint16_t fftLen,
float32_t * pCoef,
uint16_t twidCoefModifier)
{
   float32_t co1, co2, co3, si1, si2, si3;
   uint32_t ia1, ia2, ia3;
   uint32_t i0, i1, i2, i3;
   uint32_t n1, n2, j, k;
   float32_t t1, t2, r1, r2, s1, s2;
   n2 = fftLen;
   n1 = n2;
   for (k = fftLen; k > 1U; k >>= 2U)
   {
      n1 = n2;
      n2 >>= 2U;
      ia1 = 0U;
      j = 0;
      do
      {
         ia2 = ia1 + ia1;
         ia3 = ia2 + ia1;
         co1 = pCoef[ia1 * 2U];
         si1 = pCoef[(ia1 * 2U) + 1U];
         co2 = pCoef[ia2 * 2U];
         si2 = pCoef[(ia2 * 2U) + 1U];
         co3 = pCoef[ia3 * 2U];
         si3 = pCoef[(ia3 * 2U) + 1U];
         ia1 = ia1 + twidCoefModifier;
         i0 = j;
         do
         {
            i1 = i0 + n2;
            i2 = i1 + n2;
            i3 = i2 + n2;
            r1 = pSrc[(2U * i0)] + pSrc[(2U * i2)];
            r2 = pSrc[(2U * i0)] - pSrc[(2U * i2)];
            s1 = pSrc[(2U * i0) + 1U] + pSrc[(2U * i2) + 1U];
            s2 = pSrc[(2U * i0) + 1U] - pSrc[(2U * i2) + 1U];
            t1 = pSrc[2U * i1] + pSrc[2U * i3];
            pSrc[2U * i0] = r1 + t1;
            r1 = r1 - t1;
            t2 = pSrc[(2U * i1) + 1U] + pSrc[(2U * i3) + 1U];
            pSrc[(2U * i0) + 1U] = s1 + t2;
            s1 = s1 - t2;
            t1 = pSrc[(2U * i1) + 1U] - pSrc[(2U * i3) + 1U];
            t2 = pSrc[2U * i1] - pSrc[2U * i3];
            pSrc[2U * i1] = (r1 * co2) + (s1 * si2);
            pSrc[(2U * i1) + 1U] = (s1 * co2) - (r1 * si2);
            r1 = r2 + t1;
            r2 = r2 - t1;
            s1 = s2 - t2;
            s2 = s2 + t2;
            pSrc[2U * i2] = (r1 * co1) + (s1 * si1);
            pSrc[(2U * i2) + 1U] = (s1 * co1) - (r1 * si1);
            pSrc[2U * i3] = (r2 * co3) + (s2 * si3);
            pSrc[(2U * i3) + 1U] = (s2 * co3) - (r2 * si3);
            i0 += n1;
         } while ( i0 < fftLen);
         j++;
      } while (j <= (n2 - 1U));
      twidCoefModifier <<= 2U;
   }
}
void xtensa_radix4_butterfly_inverse_f32(
float32_t * pSrc,
uint16_t fftLen,
float32_t * pCoef,
uint16_t twidCoefModifier,
float32_t onebyfftLen)
{
   float32_t co1, co2, co3, si1, si2, si3;
   uint32_t ia1, ia2, ia3;
   uint32_t i0, i1, i2, i3;
   uint32_t n1, n2, j, k;
   float32_t t1, t2, r1, r2, s1, s2;
   n2 = fftLen;
   n1 = n2;
   for (k = fftLen; k > 4U; k >>= 2U)
   {
      n1 = n2;
      n2 >>= 2U;
      ia1 = 0U;
      j = 0;
      do
      {
         ia2 = ia1 + ia1;
         ia3 = ia2 + ia1;
         co1 = pCoef[ia1 * 2U];
         si1 = pCoef[(ia1 * 2U) + 1U];
         co2 = pCoef[ia2 * 2U];
         si2 = pCoef[(ia2 * 2U) + 1U];
         co3 = pCoef[ia3 * 2U];
         si3 = pCoef[(ia3 * 2U) + 1U];
         ia1 = ia1 + twidCoefModifier;
         i0 = j;
         do
         {
            i1 = i0 + n2;
            i2 = i1 + n2;
            i3 = i2 + n2;
            r1 = pSrc[(2U * i0)] + pSrc[(2U * i2)];
            r2 = pSrc[(2U * i0)] - pSrc[(2U * i2)];
            s1 = pSrc[(2U * i0) + 1U] + pSrc[(2U * i2) + 1U];
            s2 = pSrc[(2U * i0) + 1U] - pSrc[(2U * i2) + 1U];
            t1 = pSrc[2U * i1] + pSrc[2U * i3];
            pSrc[2U * i0] = r1 + t1;
            r1 = r1 - t1;
            t2 = pSrc[(2U * i1) + 1U] + pSrc[(2U * i3) + 1U];
            pSrc[(2U * i0) + 1U] = s1 + t2;
            s1 = s1 - t2;
            t1 = pSrc[(2U * i1) + 1U] - pSrc[(2U * i3) + 1U];
            t2 = pSrc[2U * i1] - pSrc[2U * i3];
            pSrc[2U * i1] = (r1 * co2) - (s1 * si2);
            pSrc[(2U * i1) + 1U] = (s1 * co2) + (r1 * si2);
            r1 = r2 - t1;
            r2 = r2 + t1;
            s1 = s2 + t2;
            s2 = s2 - t2;
            pSrc[2U * i2] = (r1 * co1) - (s1 * si1);
            pSrc[(2U * i2) + 1U] = (s1 * co1) + (r1 * si1);
            pSrc[2U * i3] = (r2 * co3) - (s2 * si3);
            pSrc[(2U * i3) + 1U] = (s2 * co3) + (r2 * si3);
            i0 += n1;
         } while ( i0 < fftLen);
         j++;
      } while (j <= (n2 - 1U));
      twidCoefModifier <<= 2U;
   }
   n1 = n2;
   n2 >>= 2U;
   for (i0 = 0U; i0 <= (fftLen - n1); i0 += n1)
   {
      i1 = i0 + n2;
      i2 = i1 + n2;
      i3 = i2 + n2;
      r1 = pSrc[2U * i0] + pSrc[2U * i2];
      r2 = pSrc[2U * i0] - pSrc[2U * i2];
      s1 = pSrc[(2U * i0) + 1U] + pSrc[(2U * i2) + 1U];
      s2 = pSrc[(2U * i0) + 1U] - pSrc[(2U * i2) + 1U];
      t1 = pSrc[2U * i1] + pSrc[2U * i3];
      pSrc[2U * i0] = (r1 + t1) * onebyfftLen;
      r1 = r1 - t1;
      t2 = pSrc[(2U * i1) + 1U] + pSrc[(2U * i3) + 1U];
      pSrc[(2U * i0) + 1U] = (s1 + t2) * onebyfftLen;
      s1 = s1 - t2;
      t1 = pSrc[(2U * i1) + 1U] - pSrc[(2U * i3) + 1U];
      t2 = pSrc[2U * i1] - pSrc[2U * i3];
      pSrc[2U * i1] = r1 * onebyfftLen;
      pSrc[(2U * i1) + 1U] = s1 * onebyfftLen;
      r1 = r2 - t1;
      r2 = r2 + t1;
      s1 = s2 + t2;
      s2 = s2 - t2;
      pSrc[2U * i2] = r1 * onebyfftLen;
      pSrc[(2U * i2) + 1U] = s1 * onebyfftLen;
      pSrc[2U * i3] = r2 * onebyfftLen;
      pSrc[(2U * i3) + 1U] = s2 * onebyfftLen;
   }
}