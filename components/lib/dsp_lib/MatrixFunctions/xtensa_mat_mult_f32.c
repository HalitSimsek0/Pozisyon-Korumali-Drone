#include "xtensa_math.h"
xtensa_status xtensa_mat_mult_f32(
  const xtensa_matrix_instance_f32 * pSrcA,
  const xtensa_matrix_instance_f32 * pSrcB,
  xtensa_matrix_instance_f32 * pDst)
{
  float32_t *pIn1 = pSrcA->pData;                
  float32_t *pIn2 = pSrcB->pData;                
  float32_t *pInA = pSrcA->pData;                
  float32_t *pOut = pDst->pData;                 
  float32_t *px;                                 
  float32_t sum;                                 
  uint16_t numRowsA = pSrcA->numRows;            
  uint16_t numColsB = pSrcB->numCols;            
  uint16_t numColsA = pSrcA->numCols;            
  float32_t *pInB = pSrcB->pData;                
  uint16_t col, i = 0U, row = numRowsA, colCnt;  
  xtensa_status status;                             
#ifdef XTENSA_MATH_MATRIX_CHECK
  if ((pSrcA->numCols != pSrcB->numRows) ||
     (pSrcA->numRows != pDst->numRows) || (pSrcB->numCols != pDst->numCols))
  {
    status = XTENSA_MATH_SIZE_MISMATCH;
  }
  else
#endif 
  {
    do
    {
      px = pOut + i;
      col = numColsB;
      pIn2 = pSrcB->pData;
      do
      {
        sum = 0.0f;
        pIn1 = pInA;
        colCnt = numColsA;
        while (colCnt > 0U)
        {
          sum += *pIn1++ * (*pIn2);
          pIn2 += numColsB;
          colCnt--;
        }
        *px++ = sum;
        col--;
        pIn2 = pInB + (numColsB - col);
      } while (col > 0U);
      i = i + numColsB;
      pInA = pInA + numColsA;
      row--;
    } while (row > 0U);
    status = XTENSA_MATH_SUCCESS;
  }
  return (status);
}