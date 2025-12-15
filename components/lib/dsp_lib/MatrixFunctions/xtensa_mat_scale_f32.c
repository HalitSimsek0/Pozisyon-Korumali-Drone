#include "xtensa_math.h"
xtensa_status xtensa_mat_scale_f32(
  const xtensa_matrix_instance_f32 * pSrc,
  float32_t scale,
  xtensa_matrix_instance_f32 * pDst)
{
  float32_t *pIn = pSrc->pData;                  
  float32_t *pOut = pDst->pData;                 
  uint32_t numSamples;                           
  uint32_t blkCnt;                               
  xtensa_status status;                             
#ifdef XTENSA_MATH_MATRIX_CHECK
  if ((pSrc->numRows != pDst->numRows) || (pSrc->numCols != pDst->numCols))
  {
    status = XTENSA_MATH_SIZE_MISMATCH;
  }
  else
#endif 
  {
    numSamples = (uint32_t) pSrc->numRows * pSrc->numCols;
    blkCnt = numSamples;
    while (blkCnt > 0U)
    {
      *pOut++ = (*pIn++) * scale;
      blkCnt--;
    }
    status = XTENSA_MATH_SUCCESS;
  }
  return (status);
}