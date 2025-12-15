#include "xtensa_math.h"
#include <math.h>
void xtensa_abs_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               
  blkCnt = blockSize;
  while (blkCnt > 0U)
  {
    *pDst++ = fabsf(*pSrc++);
    blkCnt--;
  }
}