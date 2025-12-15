#include "xtensa_math.h"
xtensa_status xtensa_mat_inverse_f32(
  const xtensa_matrix_instance_f32 * pSrc,
  xtensa_matrix_instance_f32 * pDst)
{
  float32_t *pIn = pSrc->pData;                  
  float32_t *pOut = pDst->pData;                 
  float32_t *pInT1, *pInT2;                      
  float32_t *pOutT1, *pOutT2;                    
  float32_t *pPivotRowIn, *pPRT_in, *pPivotRowDst, *pPRT_pDst;  
  uint32_t numRows = pSrc->numRows;              
  uint32_t numCols = pSrc->numCols;              
  float32_t Xchg, in = 0.0f;                     
  uint32_t i, rowCnt, flag = 0U, j, loopCnt, k, l;      
  xtensa_status status;                             
#ifdef XTENSA_MATH_MATRIX_CHECK
  if ((pSrc->numRows != pSrc->numCols) || (pDst->numRows != pDst->numCols)
     || (pSrc->numRows != pDst->numRows))
  {
    status = XTENSA_MATH_SIZE_MISMATCH;
  }
  else
#endif 
  {
    pOutT1 = pOut;
    rowCnt = numRows;
    while (rowCnt > 0U)
    {
      j = numRows - rowCnt;
      while (j > 0U)
      {
        *pOutT1++ = 0.0f;
        j--;
      }
      *pOutT1++ = 1.0f;
      j = rowCnt - 1U;
      while (j > 0U)
      {
        *pOutT1++ = 0.0f;
        j--;
      }
      rowCnt--;
    }
    loopCnt = numCols;
    l = 0U;
    while (loopCnt > 0U)
    {
      pInT1 = pIn + (l * numCols);
      pOutT1 = pOut + (l * numCols);
      in = *pInT1;
      k = 1U;
      if (*pInT1 == 0.0f)
      {
        for (i = (l + 1U); i < numRows; i++)
        {
          pInT2 = pInT1 + (numCols * l);
          pOutT2 = pOutT1 + (numCols * k);
          if (*pInT2 != 0.0f)
          {
            for (j = 0U; j < (numCols - l); j++)
            {
              Xchg = *pInT2;
              *pInT2++ = *pInT1;
              *pInT1++ = Xchg;
            }
            for (j = 0U; j < numCols; j++)
            {
              Xchg = *pOutT2;
              *pOutT2++ = *pOutT1;
              *pOutT1++ = Xchg;
            }
            flag = 1U;
            break;
          }
          k++;
        }
      }
      if ((flag != 1U) && (in == 0.0f))
      {
        return XTENSA_MATH_SINGULAR;
      }
      pPivotRowIn = pIn + (l * numCols);
      pPivotRowDst = pOut + (l * numCols);
      pInT1 = pPivotRowIn;
      pOutT1 = pPivotRowDst;
      in = *(pIn + (l * numCols));
      for (j = 0U; j < (numCols - l); j++)
      {
        *pInT1 = *pInT1 / in;
        pInT1++;
      }
      for (j = 0U; j < numCols; j++)
      {
        *pOutT1 = *pOutT1 / in;
        pOutT1++;
      }
      pInT1 = pIn;
      pOutT1 = pOut;
      for (i = 0U; i < numRows; i++)
      {
        if (i == l)
        {
          pInT1 += numCols - l;
          pOutT1 += numCols;
        }
        else
        {
          in = *pInT1;
          pPRT_in = pPivotRowIn;
          pPRT_pDst = pPivotRowDst;
          for (j = 0U; j < (numCols - l); j++)
          {
            *pInT1 = *pInT1 - (in * *pPRT_in++);
            pInT1++;
          }
          for (j = 0U; j < numCols; j++)
          {
            *pOutT1 = *pOutT1 - (in * *pPRT_pDst++);
            pOutT1++;
          }
        }
        pInT1 = pInT1 + l;
      }
      pIn++;
      loopCnt--;
      l++;
    }
    status = XTENSA_MATH_SUCCESS;
    if ((flag != 1U) && (in == 0.0f))
    {
      pIn = pSrc->pData;
      for (i = 0; i < numRows * numCols; i++)
      {
        if (pIn[i] != 0.0f)
            break;
      }
      if (i == numRows * numCols)
        status = XTENSA_MATH_SINGULAR;
    }
  }
  return (status);
}