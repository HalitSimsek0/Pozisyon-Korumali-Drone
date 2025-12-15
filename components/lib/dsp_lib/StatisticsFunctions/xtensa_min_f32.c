#include "xtensa_math.h"
void xtensa_min_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult,
  uint32_t * pIndex)
{
  float32_t minVal1, out;                        
  uint32_t blkCnt, outIndex;                     
  outIndex = 0U;
  out = *pSrc++;
  blkCnt = (blockSize - 1U);
  while (blkCnt > 0U)
  {
    minVal1 = *pSrc++;
    if (out > minVal1)
    {
      out = minVal1;
      outIndex = blockSize - blkCnt;
    }
    blkCnt--;
  }
  *pResult = out;
  *pIndex = outIndex;
}