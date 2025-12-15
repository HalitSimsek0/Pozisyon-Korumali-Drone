#include "xtensa_math.h"
void xtensa_rms_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult)
{
  float32_t sum = 0.0f;                          
  float32_t in;                                  
  uint32_t blkCnt;                               
  blkCnt = blockSize;
  while (blkCnt > 0U)
  {
    in = *pSrc++;
    sum += in * in;
    blkCnt--;
  }
  xtensa_sqrt_f32(sum / (float32_t) blockSize, pResult);
}