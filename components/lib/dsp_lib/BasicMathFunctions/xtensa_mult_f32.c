#include "xtensa_math.h"
void xtensa_mult_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  float32_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               
  blkCnt = blockSize;
  while (blkCnt > 0U)
  {
    *pDst++ = (*pSrcA++) * (*pSrcB++);
    blkCnt--;
  }
}