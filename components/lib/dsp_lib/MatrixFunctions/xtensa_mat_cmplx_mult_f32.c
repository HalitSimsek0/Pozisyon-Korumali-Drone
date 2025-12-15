#include "xtensa_math.h"
xtensa_status xtensa_mat_cmplx_mult_f32(
  const xtensa_matrix_instance_f32 * pSrcA,
  const xtensa_matrix_instance_f32 * pSrcB,
  xtensa_matrix_instance_f32 * pDst)
{
  float32_t *pIn1 = pSrcA->pData;                
  float32_t *pIn2 = pSrcB->pData;                
  float32_t *pInA = pSrcA->pData;                
  float32_t *pOut = pDst->pData;                 
  float32_t *px;                                 
  uint16_t numRowsA = pSrcA->numRows;            
  uint16_t numColsB = pSrcB->numCols;            
  uint16_t numColsA = pSrcA->numCols;            
  float32_t sumReal1, sumImag1;                  
  float32_t a0, b0, c0, d0;
  float32_t a1, b1, c1, d1;
  float32_t sumReal2, sumImag2;                  
  uint16_t col, i = 0U, j, row = numRowsA, colCnt;      
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
      px = pOut + 2 * i;
      col = numColsB;
      pIn2 = pSrcB->pData;
      j = 0U;
      do
      {
        sumReal1 = 0.0f;
        sumImag1 = 0.0f;
        sumReal2 = 0.0f;
        sumImag2 = 0.0f;
        pIn1 = pInA;
        colCnt = numColsA >> 2;
        while (colCnt > 0U)
        {
          a0 = *pIn1;
          c0 = *pIn2;
          b0 = *(pIn1 + 1U);
          d0 = *(pIn2 + 1U);
          sumReal1 += a0 * c0;
          sumImag1 += b0 * c0;
          pIn1 += 2U;
          pIn2 += 2 * numColsB;
          sumReal2 -= b0 * d0;
          sumImag2 += a0 * d0;
          a1 = *pIn1;
          c1 = *pIn2;
          b1 = *(pIn1 + 1U);
          d1 = *(pIn2 + 1U);
          sumReal1 += a1 * c1;
          sumImag1 += b1 * c1;
          pIn1 += 2U;
          pIn2 += 2 * numColsB;
          sumReal2 -= b1 * d1;
          sumImag2 += a1 * d1;
          a0 = *pIn1;
          c0 = *pIn2;
          b0 = *(pIn1 + 1U);
          d0 = *(pIn2 + 1U);
          sumReal1 += a0 * c0;
          sumImag1 += b0 * c0;
          pIn1 += 2U;
          pIn2 += 2 * numColsB;
          sumReal2 -= b0 * d0;
          sumImag2 += a0 * d0;
          a1 = *pIn1;
          c1 = *pIn2;
          b1 = *(pIn1 + 1U);
          d1 = *(pIn2 + 1U);
          sumReal1 += a1 * c1;
          sumImag1 += b1 * c1;
          pIn1 += 2U;
          pIn2 += 2 * numColsB;
          sumReal2 -= b1 * d1;
          sumImag2 += a1 * d1;
          colCnt--;
        }
        colCnt = numColsA % 0x4U;
        while (colCnt > 0U)
        {
          a1 = *pIn1;
          c1 = *pIn2;
          b1 = *(pIn1 + 1U);
          d1 = *(pIn2 + 1U);
          sumReal1 += a1 * c1;
          sumImag1 += b1 * c1;
          pIn1 += 2U;
          pIn2 += 2 * numColsB;
          sumReal2 -= b1 * d1;
          sumImag2 += a1 * d1;
          colCnt--;
        }
        sumReal1 += sumReal2;
        sumImag1 += sumImag2;
        *px++ = sumReal1;
        *px++ = sumImag1;
        j++;
        pIn2 = pSrcB->pData + 2U * j;
        col--;
      } while (col > 0U);
      i = i + numColsB;
      pInA = pInA + 2 * numColsA;
      row--;
    } while (row > 0U);
    status = XTENSA_MATH_SUCCESS;
  }
  return (status);
}