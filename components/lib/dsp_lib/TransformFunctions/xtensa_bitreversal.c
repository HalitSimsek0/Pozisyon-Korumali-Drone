#include "xtensa_math.h"
#include "xtensa_common_tables.h"
void xtensa_bitreversal_f32(
float32_t * pSrc,
uint16_t fftSize,
uint16_t bitRevFactor,
uint16_t * pBitRevTab)
{
   uint16_t fftLenBy2, fftLenBy2p1;
   uint16_t i, j;
   float32_t in;
   j = 0U;
   fftLenBy2 = fftSize >> 1U;
   fftLenBy2p1 = (fftSize >> 1U) + 1U;
   for (i = 0U; i <= (fftLenBy2 - 2U); i += 2U)
   {
      if (i < j)
      {
         in = pSrc[2U * i];
         pSrc[2U * i] = pSrc[2U * j];
         pSrc[2U * j] = in;
         in = pSrc[(2U * i) + 1U];
         pSrc[(2U * i) + 1U] = pSrc[(2U * j) + 1U];
         pSrc[(2U * j) + 1U] = in;
         in = pSrc[2U * (i + fftLenBy2p1)];
         pSrc[2U * (i + fftLenBy2p1)] = pSrc[2U * (j + fftLenBy2p1)];
         pSrc[2U * (j + fftLenBy2p1)] = in;
         in = pSrc[(2U * (i + fftLenBy2p1)) + 1U];
         pSrc[(2U * (i + fftLenBy2p1)) + 1U] =
         pSrc[(2U * (j + fftLenBy2p1)) + 1U];
         pSrc[(2U * (j + fftLenBy2p1)) + 1U] = in;
      }
      in = pSrc[2U * (i + 1U)];
      pSrc[2U * (i + 1U)] = pSrc[2U * (j + fftLenBy2)];
      pSrc[2U * (j + fftLenBy2)] = in;
      in = pSrc[(2U * (i + 1U)) + 1U];
      pSrc[(2U * (i + 1U)) + 1U] = pSrc[(2U * (j + fftLenBy2)) + 1U];
      pSrc[(2U * (j + fftLenBy2)) + 1U] = in;
      j = *pBitRevTab;
      pBitRevTab += bitRevFactor;
   }
}