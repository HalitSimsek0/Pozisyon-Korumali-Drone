#include "xtensa_math.h"
void xtensa_power_f32(
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
  *pResult = sum;
}