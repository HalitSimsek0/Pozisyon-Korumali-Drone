#include "xtensa_math.h"
xtensa_status xtensa_conv_partial_f32(
  float32_t * pSrcA,
  uint32_t srcALen,
  float32_t * pSrcB,
  uint32_t srcBLen,
  float32_t * pDst,
  uint32_t firstIndex,
  uint32_t numPoints)
{
  float32_t *pIn1 = pSrcA;                       
  float32_t *pIn2 = pSrcB;                       
  float32_t sum;                                 
  uint32_t i, j;                                 
  xtensa_status status;                             
  if ((firstIndex + numPoints) > ((srcALen + (srcBLen - 1U))))
  {
    status = XTENSA_MATH_ARGUMENT_ERROR;
  }
  else
  {
    for (i = firstIndex; i <= (firstIndex + numPoints - 1); i++)
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
    status = XTENSA_MATH_SUCCESS;
  }
  return (status);
}