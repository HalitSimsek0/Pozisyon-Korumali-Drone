#include "xtensa_math.h"
void xtensa_scale_f32(
  float32_t * pSrc,
  float32_t scale,
  float32_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               
  blkCnt = blockSize;
  while (blkCnt > 0U)
  {
    *pDst++ = (*pSrc++) * scale;
    blkCnt--;
  }
}