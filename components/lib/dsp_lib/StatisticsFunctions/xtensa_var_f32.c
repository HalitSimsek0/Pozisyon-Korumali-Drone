#include "xtensa_math.h"
void xtensa_var_f32(
                 float32_t * pSrc,
                 uint32_t blockSize,
                 float32_t * pResult)
{
    float32_t fMean, fValue;
    uint32_t blkCnt;            
    float32_t * pInput = pSrc;
    float32_t sum = 0.0f;
    float32_t fSum = 0.0f;
    #if defined(ARM_MATH_DSP)
    float32_t in1, in2, in3, in4;
    #endif
    if (blockSize <= 1U)
    {
        *pResult = 0;
        return;
    }
        blkCnt = blockSize;
    while (blkCnt > 0U)
    {
        sum += *pInput++;
        blkCnt--;
    }
    fMean = sum / (float32_t) blockSize;
    pInput = pSrc;
    blkCnt = blockSize;
    while (blkCnt > 0U)
    {
        fValue = *pInput++ - fMean;
        fSum += fValue * fValue;
        blkCnt--;
    }
    *pResult = fSum / (float32_t)(blockSize - 1.0f);
}