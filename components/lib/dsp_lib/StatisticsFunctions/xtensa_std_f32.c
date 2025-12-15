#include "xtensa_math.h"
void xtensa_std_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult)
{
  float32_t sum = 0.0f;                          
  float32_t sumOfSquares = 0.0f;                 
  float32_t in;                                  
  uint32_t blkCnt;                               
  float32_t squareOfSum;                         
  float32_t var;                                 
  if (blockSize == 1U)
  {
    *pResult = 0;
    return;
  }
  blkCnt = blockSize;
  while (blkCnt > 0U)
  {
    in = *pSrc++;
    sumOfSquares += in * in;
    sum += in;
    blkCnt--;
  }
  squareOfSum = ((sum * sum) / (float32_t) blockSize);
  var = ((sumOfSquares - squareOfSum) / (float32_t) (blockSize - 1.0f));
  xtensa_sqrt_f32(var, pResult);
}