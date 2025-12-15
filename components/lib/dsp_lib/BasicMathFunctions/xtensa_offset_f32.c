#include "xtensa_math.h"
void xtensa_offset_f32(
  float32_t * pSrc,
  float32_t offset,
  float32_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               
  blkCnt = blockSize;
  while (blkCnt > 0U)
  {
    *pDst++ = (*pSrc++) + offset;
    blkCnt--;
  }
}