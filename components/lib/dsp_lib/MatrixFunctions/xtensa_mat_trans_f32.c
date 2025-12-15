#include "xtensa_math.h"
xtensa_status xtensa_mat_trans_f32(
  const xtensa_matrix_instance_f32 * pSrc,
  xtensa_matrix_instance_f32 * pDst)
{
  float32_t *pIn = pSrc->pData;                  
  float32_t *pOut = pDst->pData;                 
  float32_t *px;                                 
  uint16_t nRows = pSrc->numRows;                
  uint16_t nColumns = pSrc->numCols;             
  uint16_t col, i = 0U, row = nRows;             
  xtensa_status status;                             
#ifdef XTENSA_MATH_MATRIX_CHECK
  if ((pSrc->numRows != pDst->numCols) || (pSrc->numCols != pDst->numRows))
  {
    status = XTENSA_MATH_SIZE_MISMATCH;
  }
  else
#endif 
  {
    do
    {
      px = pOut + i;
      col = nColumns;
      while (col > 0U)
      {
        *px = *pIn++;
        px += nRows;
        col--;
      }
      i++;
      row--;
    } while (row > 0U);          
    status = XTENSA_MATH_SUCCESS;
  }
  return (status);
}