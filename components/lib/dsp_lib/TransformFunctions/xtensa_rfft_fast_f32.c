#include "xtensa_math.h"
void stage_rfft_f32(
  xtensa_rfft_fast_instance_f32 * S,
  float32_t * p, float32_t * pOut)
{
   uint32_t  k;								   
   float32_t twR, twI;						   
   float32_t * pCoeff = S->pTwiddleRFFT;  
   float32_t *pA = p;						   
   float32_t *pB = p;						   
   float32_t xAR, xAI, xBR, xBI;				
   float32_t t1a, t1b;				         
   float32_t p0, p1, p2, p3;				   
   k = (S->Sint).fftLen - 1;
   xBR = pB[0];
   xBI = pB[1];
   xAR = pA[0];
   xAI = pA[1];
   twR = *pCoeff++ ;
   twI = *pCoeff++ ;
   t1a = xBR + xAR  ;
   t1b = xBI + xAI  ;
   *pOut++ = 0.5f * ( t1a + t1b );
   *pOut++ = 0.5f * ( t1a - t1b );
   pB  = p + 2*k;
   pA += 2;
   do
   {
      xBI = pB[1];
      xBR = pB[0];
      xAR = pA[0];
      xAI = pA[1];
      twR = *pCoeff++;
      twI = *pCoeff++;
      t1a = xBR - xAR ;
      t1b = xBI + xAI ;
      p0 = twR * t1a;
      p1 = twI * t1a;
      p2 = twR * t1b;
      p3 = twI * t1b;
      *pOut++ = 0.5f * (xAR + xBR + p0 + p3 ); 
      *pOut++ = 0.5f * (xAI - xBI + p1 - p2 ); 
      pA += 2;
      pB -= 2;
      k--;
   } while (k > 0U);
}
void merge_rfft_f32(
xtensa_rfft_fast_instance_f32 * S,
float32_t * p, float32_t * pOut)
{
   uint32_t  k;								
   float32_t twR, twI;						
   float32_t *pCoeff = S->pTwiddleRFFT;		
   float32_t *pA = p;						
   float32_t *pB = p;						
   float32_t xAR, xAI, xBR, xBI;			
   float32_t t1a, t1b, r, s, t, u;			
   k = (S->Sint).fftLen - 1;
   xAR = pA[0];
   xAI = pA[1];
   pCoeff += 2 ;
   *pOut++ = 0.5f * ( xAR + xAI );
   *pOut++ = 0.5f * ( xAR - xAI );
   pB  =  p + 2*k ;
   pA +=  2	   ;
   while (k > 0U)
   {
      xBI =   pB[1]    ;
      xBR =   pB[0]    ;
      xAR =  pA[0];
      xAI =  pA[1];
      twR = *pCoeff++;
      twI = *pCoeff++;
      t1a = xAR - xBR ;
      t1b = xAI + xBI ;
      r = twR * t1a;
      s = twI * t1b;
      t = twI * t1a;
      u = twR * t1b;
      *pOut++ = 0.5f * (xAR + xBR - r - s ); 
      *pOut++ = 0.5f * (xAI - xBI + t - u ); 
      pA += 2;
      pB -= 2;
      k--;
   }
}
void xtensa_rfft_fast_f32(
xtensa_rfft_fast_instance_f32 * S,
float32_t * p, float32_t * pOut,
uint8_t ifftFlag)
{
   xtensa_cfft_instance_f32 * Sint = &(S->Sint);
   Sint->fftLen = S->fftLenRFFT / 2;
   if (ifftFlag)
   {
      merge_rfft_f32(S, p, pOut);
      xtensa_cfft_f32( Sint, pOut, ifftFlag, 1);
   }
   else
   {
      xtensa_cfft_f32( Sint, p, ifftFlag, 1);
      stage_rfft_f32(S, p, pOut);
   }
}