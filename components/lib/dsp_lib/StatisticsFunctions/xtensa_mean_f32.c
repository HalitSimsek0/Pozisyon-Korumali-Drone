#include "xtensa_math.h"
void xtensa_mean_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult)
{
  float32_t sum = 0.0f;                          
  uint32_t blkCnt;                               
  blkCnt = blockSize;
  while (blkCnt > 0U)
  {
    sum += *pSrc++;
    blkCnt--;
  }
  *pResult = sum / (float32_t) blockSize;
}