#include "xtensa_math.h"
xtensa_status xtensa_mat_add_f32(
  const xtensa_matrix_instance_f32 * pSrcA,
  const xtensa_matrix_instance_f32 * pSrcB,
  xtensa_matrix_instance_f32 * pDst)
{
  float32_t *pIn1 = pSrcA->pData;                
  float32_t *pIn2 = pSrcB->pData;                
  float32_t *pOut = pDst->pData;                 
  uint32_t numSamples;                           
  uint32_t blkCnt;                               
  xtensa_status status;                             
#ifdef XTENSA_MATH_MATRIX_CHECK
  if ((pSrcA->numRows != pSrcB->numRows) ||
     (pSrcA->numCols != pSrcB->numCols) ||
     (pSrcA->numRows != pDst->numRows) || (pSrcA->numCols != pDst->numCols))
  {
    status = XTENSA_MATH_SIZE_MISMATCH;
  }
  else
#endif
  {
    numSamples = (uint32_t) pSrcA->numRows * pSrcA->numCols;
    blkCnt = numSamples;
    while (blkCnt > 0U)
    {
      *pOut++ = (*pIn1++) + (*pIn2++);
      blkCnt--;
    }
    status = XTENSA_MATH_SUCCESS;
  }
  return (status);
}