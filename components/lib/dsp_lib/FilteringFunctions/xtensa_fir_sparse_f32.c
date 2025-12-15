#include "xtensa_math.h"
void xtensa_fir_sparse_f32(
  xtensa_fir_sparse_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  float32_t * pScratchIn,
  uint32_t blockSize)
{
  float32_t *pState = S->pState;                 
  float32_t *pCoeffs = S->pCoeffs;               
  float32_t *px;                                 
  float32_t *py = pState;                        
  float32_t *pb = pScratchIn;                    
  float32_t *pOut;                               
  int32_t *pTapDelay = S->pTapDelay;             
  uint32_t delaySize = S->maxDelay + blockSize;  
  uint16_t numTaps = S->numTaps;                 
  int32_t readIndex;                             
  uint32_t tapCnt, blkCnt;                       
  float32_t coeff = *pCoeffs++;                  
  xtensa_circularWrite_f32((int32_t *) py, delaySize, &S->stateIndex, 1,
                        (int32_t *) pSrc, 1, blockSize);
  readIndex = ((int32_t) S->stateIndex - (int32_t) blockSize) - *pTapDelay++;
  if (readIndex < 0)
  {
    readIndex += (int32_t) delaySize;
  }
  py = pState;
  xtensa_circularRead_f32((int32_t *) py, delaySize, &readIndex, 1,
                       (int32_t *) pb, (int32_t *) pb, blockSize, 1,
                       blockSize);
  px = pb;
  pOut = pDst;
  blkCnt = blockSize;
  while (blkCnt > 0U)
  {
    *pOut++ = *px++ * coeff;
    blkCnt--;
  }
  coeff = *pCoeffs++;
  readIndex = ((int32_t) S->stateIndex - (int32_t) blockSize) - *pTapDelay++;
  if (readIndex < 0)
  {
    readIndex += (int32_t) delaySize;
  }
  tapCnt = (uint32_t) numTaps - 2U;
  while (tapCnt > 0U)
  {
    py = pState;
    xtensa_circularRead_f32((int32_t *) py, delaySize, &readIndex, 1,
                         (int32_t *) pb, (int32_t *) pb, blockSize, 1,
                         blockSize);
    px = pb;
    pOut = pDst;
    blkCnt = blockSize;
    while (blkCnt > 0U)
    {
      *pOut++ += *px++ * coeff;
      blkCnt--;
    }
    coeff = *pCoeffs++;
    readIndex =
      ((int32_t) S->stateIndex - (int32_t) blockSize) - *pTapDelay++;
    if (readIndex < 0)
    {
      readIndex += (int32_t) delaySize;
    }
    tapCnt--;
  }
	py = pState;
	xtensa_circularRead_f32((int32_t *) py, delaySize, &readIndex, 1,
											 (int32_t *) pb, (int32_t *) pb, blockSize, 1,
											 blockSize);
	px = pb;
	pOut = pDst;
	blkCnt = blockSize;
	while (blkCnt > 0U)
	{
		*pOut++ += *px++ * coeff;
		blkCnt--;
	}
}