#include "xtensa_math.h"
void xtensa_conv_f32(
  float32_t * pSrcA,
  uint32_t srcALen,
  float32_t * pSrcB,
  uint32_t srcBLen,
  float32_t * pDst)
{
  float32_t *pIn1 = pSrcA;                       
  float32_t *pIn2 = pSrcB;                       
  float32_t sum;                                 
  uint32_t i, j;                                 
  for (i = 0U; i < ((srcALen + srcBLen) - 1U); i++)
  {
    sum = 0.0f;
    for (j = 0U; j <= i; j++)
    {
      if ((((i - j) < srcBLen) && (j < srcALen)))
      {
        sum += pIn1[j] * pIn2[i - j];
      }
    }
    pDst[i] = sum;
  }
}