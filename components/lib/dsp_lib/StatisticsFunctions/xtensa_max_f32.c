#include "xtensa_math.h"
void xtensa_max_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult,
  uint32_t * pIndex)
{
  float32_t maxVal1, out;                        
  uint32_t blkCnt, outIndex;                     
  outIndex = 0U;
  out = *pSrc++;
  blkCnt = (blockSize - 1U);
  while (blkCnt > 0U)
  {
    maxVal1 = *pSrc++;
    if (out < maxVal1)
    {
      out = maxVal1;
      outIndex = blockSize - blkCnt;
    }
    blkCnt--;
  }
  *pResult = out;
  *pIndex = outIndex;
}