#include "xtensa_math.h"
void xtensa_dot_prod_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  uint32_t blockSize,
  float32_t * result)
{
  float32_t sum = 0.0f;                          
  uint32_t blkCnt;                               
  blkCnt = blockSize;
  while (blkCnt > 0U)
  {
    sum += (*pSrcA++) * (*pSrcB++);
    blkCnt--;
  }
  *result = sum;
}
