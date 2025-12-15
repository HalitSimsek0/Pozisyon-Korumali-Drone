#include "xtensa_math.h"
void xtensa_dct4_f32(
  const xtensa_dct4_instance_f32 * S,
  float32_t * pState,
  float32_t * pInlineBuffer)
{
  uint32_t i;                                    
  float32_t *weights = S->pTwiddle;              
  float32_t *cosFact = S->pCosFactor;            
  float32_t *pS1, *pS2, *pbuff;                  
  float32_t in;                                  
  xtensa_scale_f32(pInlineBuffer, 2.0f, pInlineBuffer, S->N);
  xtensa_mult_f32(pInlineBuffer, cosFact, pInlineBuffer, S->N);
  pS1 = pState;
  pS2 = pState + (S->N - 1U);
  pbuff = pInlineBuffer;
  i = (uint32_t) S->Nby2;
  do
  {
    *pS1++ = *pbuff++;
    *pS2-- = *pbuff++;
    i--;
  } while (i > 0U);
  pbuff = pInlineBuffer;
  pS1 = pState;
  i = (uint32_t) S->N;
  do
  {
    *pbuff++ = *pS1++;
    i--;
  } while (i > 0U);
  xtensa_rfft_f32(S->pRfft, pInlineBuffer, pState);
  xtensa_cmplx_mult_cmplx_f32(pState, weights, pState, S->N);
  pbuff = pInlineBuffer;
  pS1 = pState;
  in = *pS1++ * (float32_t) 0.5;
  *pbuff++ = in;
  pS1++;
  i = ((uint32_t) S->N - 1U);
  do
  {
    in = *pS1++ - in;
    *pbuff++ = in;
    pS1++;
    i--;
  } while (i > 0U);
  i = (uint32_t) S->N;
  pbuff = pInlineBuffer;
  do
  {
    in = *pbuff;
    *pbuff++ = in * S->normalize;
    i--;
  } while (i > 0U);
}