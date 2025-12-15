#include "xtensa_math.h"
#include "xtensa_common_tables.h"
void xtensa_sin_cos_f32(
                      float32_t theta,
                      float32_t * pSinVal,
                      float32_t * pCosVal)
{
    float32_t fract, in;                             
    uint16_t indexS, indexC;                         
    float32_t f1, f2, d1, d2;                        
    float32_t findex, Dn, Df, temp;
    in = theta * 0.00277777777778f;
    if (in < 0.0f)
    {
        in = -in;
    }
    in = in - (int32_t)in;
    findex = (float32_t) FAST_MATH_TABLE_SIZE * in;
    indexS = ((uint16_t)findex) & 0x1ff;
    indexC = (indexS + (FAST_MATH_TABLE_SIZE / 4)) & 0x1ff;
    fract = findex - (float32_t) indexS;
    f1 = sinTable_f32[indexC+0];
    f2 = sinTable_f32[indexC+1];
    d1 = -sinTable_f32[indexS+0];
    d2 = -sinTable_f32[indexS+1];
    temp = (1.0f - fract) * f1 + fract * f2;
    Dn = 0.0122718463030f; 
    Df = f2 - f1;          
    temp = Dn *(d1 + d2) - 2 * Df;
    temp = fract * temp + (3 * Df - (d2 + 2 * d1) * Dn);
    temp = fract * temp + d1 * Dn;
    *pCosVal = fract * temp + f1;
    f1 = sinTable_f32[indexS+0];
    f2 = sinTable_f32[indexS+1];
    d1 = sinTable_f32[indexC+0];
    d2 = sinTable_f32[indexC+1];
    temp = (1.0f - fract) * f1 + fract * f2;
    Df = f2 - f1; 
    temp = Dn*(d1 + d2) - 2*Df;
    temp = fract*temp + (3*Df - (d2 + 2*d1)*Dn);
    temp = fract*temp + d1*Dn;
    *pSinVal = fract*temp + f1;
    if (theta < 0.0f)
    {
        *pSinVal = -*pSinVal;
    }
}