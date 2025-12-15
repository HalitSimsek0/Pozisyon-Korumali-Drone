#include "xtensa_math.h"
void xtensa_correlate_f32(
  float32_t * pSrcA,
  uint32_t srcALen,
  float32_t * pSrcB,
  uint32_t srcBLen,
  float32_t * pDst)
{
  float32_t *pIn1 = pSrcA;                       
  float32_t *pIn2 = pSrcB + (srcBLen - 1U);      
  float32_t sum;                                 
  uint32_t i = 0U, j;                            
  uint32_t inv = 0U;                             
  uint32_t tot = 0U;                             
  tot = ((srcALen + srcBLen) - 2U);
  if (srcALen > srcBLen)
  {
    j = srcALen - srcBLen;
    pDst += j;
  }
  else if (srcALen < srcBLen)
  {
    pIn1 = pSrcB;
    pIn2 = pSrcA + (srcALen - 1U);
    pDst = pDst + tot;
    j = srcALen;
    srcALen = srcBLen;
    srcBLen = j;
    inv = 1;
  }
  for (i = 0U; i <= tot; i++)
  {
    sum = 0.0f;
    for (j = 0U; j <= i; j++)
    {
      if ((((i - j) < srcBLen) && (j < srcALen)))
      {
        sum += pIn1[j] * pIn2[-((int32_t) i - j)];
      }
    }
    if (inv == 1)
      *pDst-- = sum;
    else
      *pDst++ = sum;
  }
}