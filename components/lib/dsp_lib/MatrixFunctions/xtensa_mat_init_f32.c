#include "xtensa_math.h"
void xtensa_mat_init_f32(
  xtensa_matrix_instance_f32 * S,
  uint16_t nRows,
  uint16_t nColumns,
  float32_t * pData)
{
  S->numRows = nRows;
  S->numCols = nColumns;
  S->pData = pData;
}