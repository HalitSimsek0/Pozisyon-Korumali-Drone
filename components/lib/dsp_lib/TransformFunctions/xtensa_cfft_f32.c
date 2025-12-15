#include "xtensa_math.h"
#include "xtensa_common_tables.h"
extern void xtensa_radix8_butterfly_f32(
    float32_t * pSrc,
    uint16_t fftLen,
    const float32_t * pCoef,
    uint16_t twidCoefModifier);
extern void xtensa_bitreversal_32(
    uint32_t * pSrc,
    const uint16_t bitRevLen,
    const uint16_t * pBitRevTable);
void xtensa_cfft_radix8by2_f32( xtensa_cfft_instance_f32 * S, float32_t * p1)
{
    uint32_t    L  = S->fftLen;
    float32_t * pCol1, * pCol2, * pMid1, * pMid2;
    float32_t * p2 = p1 + L;
    const float32_t * tw = (float32_t *) S->pTwiddle;
    float32_t t1[4], t2[4], t3[4], t4[4], twR, twI;
    float32_t m0, m1, m2, m3;
    uint32_t l;
    pCol1 = p1;
    pCol2 = p2;
    L >>= 1;
    pMid1 = p1 + L;
    pMid2 = p2 + L;
    for ( l = L >> 2; l > 0; l-- )
    {
        t1[0] = p1[0];
        t1[1] = p1[1];
        t1[2] = p1[2];
        t1[3] = p1[3];
        t2[0] = p2[0];
        t2[1] = p2[1];
        t2[2] = p2[2];
        t2[3] = p2[3];
        t3[0] = pMid1[0];
        t3[1] = pMid1[1];
        t3[2] = pMid1[2];
        t3[3] = pMid1[3];
        t4[0] = pMid2[0];
        t4[1] = pMid2[1];
        t4[2] = pMid2[2];
        t4[3] = pMid2[3];
        *p1++ = t1[0] + t2[0];
        *p1++ = t1[1] + t2[1];
        *p1++ = t1[2] + t2[2];
        *p1++ = t1[3] + t2[3];    
        t2[0] = t1[0] - t2[0];
        t2[1] = t1[1] - t2[1];
        t2[2] = t1[2] - t2[2];
        t2[3] = t1[3] - t2[3];    
        *pMid1++ = t3[0] + t4[0];
        *pMid1++ = t3[1] + t4[1];
        *pMid1++ = t3[2] + t4[2];
        *pMid1++ = t3[3] + t4[3]; 
        t4[0] = t4[0] - t3[0];
        t4[1] = t4[1] - t3[1];
        t4[2] = t4[2] - t3[2];
        t4[3] = t4[3] - t3[3];    
        twR = *tw++;
        twI = *tw++;
        m0 = t2[0] * twR;
        m1 = t2[1] * twI;
        m2 = t2[1] * twR;
        m3 = t2[0] * twI;
        *p2++ = m0 + m1;
        *p2++ = m2 - m3;
        m0 = t4[0] * twI;
        m1 = t4[1] * twR;
        m2 = t4[1] * twI;
        m3 = t4[0] * twR;
        *pMid2++ = m0 - m1;
        *pMid2++ = m2 + m3;
        twR = *tw++;
        twI = *tw++;
        m0 = t2[2] * twR;
        m1 = t2[3] * twI;
        m2 = t2[3] * twR;
        m3 = t2[2] * twI;
        *p2++ = m0 + m1;
        *p2++ = m2 - m3;
        m0 = t4[2] * twI;
        m1 = t4[3] * twR;
        m2 = t4[3] * twI;
        m3 = t4[2] * twR;
        *pMid2++ = m0 - m1;
        *pMid2++ = m2 + m3;
    }
    xtensa_radix8_butterfly_f32( pCol1, L, (float32_t *) S->pTwiddle, 2U);
    xtensa_radix8_butterfly_f32( pCol2, L, (float32_t *) S->pTwiddle, 2U);
}
void xtensa_cfft_radix8by4_f32( xtensa_cfft_instance_f32 * S, float32_t * p1)
{
    uint32_t    L  = S->fftLen >> 1;
    float32_t * pCol1, *pCol2, *pCol3, *pCol4, *pEnd1, *pEnd2, *pEnd3, *pEnd4;
    const float32_t *tw2, *tw3, *tw4;
    float32_t * p2 = p1 + L;
    float32_t * p3 = p2 + L;
    float32_t * p4 = p3 + L;
    float32_t t2[4], t3[4], t4[4], twR, twI;
    float32_t p1ap3_0, p1sp3_0, p1ap3_1, p1sp3_1;
    float32_t m0, m1, m2, m3;
    uint32_t l, twMod2, twMod3, twMod4;
    pCol1 = p1;         
    pCol2 = p2;
    pCol3 = p3;
    pCol4 = p4;
    pEnd1 = p2 - 1;     
    pEnd2 = p3 - 1;
    pEnd3 = p4 - 1;
    pEnd4 = pEnd3 + L;
    tw2 = tw3 = tw4 = (float32_t *) S->pTwiddle;
    L >>= 1;
    twMod2 = 2;
    twMod3 = 4;
    twMod4 = 6;
    p1ap3_0 = p1[0] + p3[0];
    p1sp3_0 = p1[0] - p3[0];
    p1ap3_1 = p1[1] + p3[1];
    p1sp3_1 = p1[1] - p3[1];
    t2[0] = p1sp3_0 + p2[1] - p4[1];
    t2[1] = p1sp3_1 - p2[0] + p4[0];
    t3[0] = p1ap3_0 - p2[0] - p4[0];
    t3[1] = p1ap3_1 - p2[1] - p4[1];
    t4[0] = p1sp3_0 - p2[1] + p4[1];
    t4[1] = p1sp3_1 + p2[0] - p4[0];
    *p1++ = p1ap3_0 + p2[0] + p4[0];
    *p1++ = p1ap3_1 + p2[1] + p4[1];
    *p2++ = t2[0];
    *p2++ = t2[1];
    *p3++ = t3[0];
    *p3++ = t3[1];
    *p4++ = t4[0];
    *p4++ = t4[1];
    tw2 += twMod2;
    tw3 += twMod3;
    tw4 += twMod4;
    for (l = (L - 2) >> 1; l > 0; l-- )
    {
        p1ap3_0 = p1[0] + p3[0];
        p1sp3_0 = p1[0] - p3[0];
        p1ap3_1 = p1[1] + p3[1];
        p1sp3_1 = p1[1] - p3[1];
        t2[0] = p1sp3_0 + p2[1] - p4[1];
        t2[1] = p1sp3_1 - p2[0] + p4[0];
        t3[0] = p1ap3_0 - p2[0] - p4[0];
        t3[1] = p1ap3_1 - p2[1] - p4[1];
        t4[0] = p1sp3_0 - p2[1] + p4[1];
        t4[1] = p1sp3_1 + p2[0] - p4[0];
        *p1++ = p1ap3_0 + p2[0] + p4[0];
        *p1++ = p1ap3_1 + p2[1] + p4[1];
        p1ap3_1 = pEnd1[-1] + pEnd3[-1];
        p1sp3_1 = pEnd1[-1] - pEnd3[-1];
        p1ap3_0 = pEnd1[0] + pEnd3[0];
        p1sp3_0 = pEnd1[0] - pEnd3[0];
        t2[2] = pEnd2[0]  - pEnd4[0] + p1sp3_1;
        t2[3] = pEnd1[0] - pEnd3[0] - pEnd2[-1] + pEnd4[-1];
        t3[2] = p1ap3_1 - pEnd2[-1] - pEnd4[-1];
        t3[3] = p1ap3_0 - pEnd2[0]  - pEnd4[0];
        t4[2] = pEnd2[0]  - pEnd4[0]  - p1sp3_1;
        t4[3] = pEnd4[-1] - pEnd2[-1] - p1sp3_0;
        *pEnd1-- = p1ap3_0 + pEnd2[0] + pEnd4[0];
        *pEnd1-- = p1ap3_1 + pEnd2[-1] + pEnd4[-1];
        twR = *tw2++;
        twI = *tw2++;
        m0 = t2[0] * twR;
        m1 = t2[1] * twI;
        m2 = t2[1] * twR;
        m3 = t2[0] * twI;
        *p2++ = m0 + m1;
        *p2++ = m2 - m3;
        m0 = t2[3] * twI;
        m1 = t2[2] * twR;
        m2 = t2[2] * twI;
        m3 = t2[3] * twR;
        *pEnd2-- = m0 - m1;
        *pEnd2-- = m2 + m3;
        twR = tw3[0];
        twI = tw3[1];
        tw3 += twMod3;
        m0 = t3[0] * twR;
        m1 = t3[1] * twI;
        m2 = t3[1] * twR;
        m3 = t3[0] * twI;
        *p3++ = m0 + m1;
        *p3++ = m2 - m3;
        m0 = -t3[3] * twR;
        m1 = t3[2] * twI;
        m2 = t3[2] * twR;
        m3 = t3[3] * twI;
        *pEnd3-- = m0 - m1;
        *pEnd3-- = m3 - m2;
        twR = tw4[0];
        twI = tw4[1];
        tw4 += twMod4;
        m0 = t4[0] * twR;
        m1 = t4[1] * twI;
        m2 = t4[1] * twR;
        m3 = t4[0] * twI;
        *p4++ = m0 + m1;
        *p4++ = m2 - m3;
        m0 = t4[3] * twI;
        m1 = t4[2] * twR;
        m2 = t4[2] * twI;
        m3 = t4[3] * twR;
        *pEnd4-- = m0 - m1;
        *pEnd4-- = m2 + m3;
    }
    p1ap3_0 = p1[0] + p3[0];
    p1sp3_0 = p1[0] - p3[0];
    p1ap3_1 = p1[1] + p3[1];
    p1sp3_1 = p1[1] - p3[1];
    t2[0] = p1sp3_0 + p2[1] - p4[1];
    t2[1] = p1sp3_1 - p2[0] + p4[0];
    t3[0] = p1ap3_0 - p2[0] - p4[0];
    t3[1] = p1ap3_1 - p2[1] - p4[1];
    t4[0] = p1sp3_0 - p2[1] + p4[1];
    t4[1] = p1sp3_1 + p2[0] - p4[0];
    *p1++ = p1ap3_0 + p2[0] + p4[0];
    *p1++ = p1ap3_1 + p2[1] + p4[1];
    twR = tw2[0];
    twI = tw2[1];
    m0 = t2[0] * twR;
    m1 = t2[1] * twI;
    m2 = t2[1] * twR;
    m3 = t2[0] * twI;
    *p2++ = m0 + m1;
    *p2++ = m2 - m3;
    twR = tw3[0];
    twI = tw3[1];
    m0 = t3[0] * twR;
    m1 = t3[1] * twI;
    m2 = t3[1] * twR;
    m3 = t3[0] * twI;
    *p3++ = m0 + m1;
    *p3++ = m2 - m3;
    twR = tw4[0];
    twI = tw4[1];
    m0 = t4[0] * twR;
    m1 = t4[1] * twI;
    m2 = t4[1] * twR;
    m3 = t4[0] * twI;
    *p4++ = m0 + m1;
    *p4++ = m2 - m3;
    xtensa_radix8_butterfly_f32( pCol1, L, (float32_t *) S->pTwiddle, 4U);
    xtensa_radix8_butterfly_f32( pCol2, L, (float32_t *) S->pTwiddle, 4U);
    xtensa_radix8_butterfly_f32( pCol3, L, (float32_t *) S->pTwiddle, 4U);
    xtensa_radix8_butterfly_f32( pCol4, L, (float32_t *) S->pTwiddle, 4U);
}
void xtensa_cfft_f32(
    const xtensa_cfft_instance_f32 * S,
    float32_t * p1,
    uint8_t ifftFlag,
    uint8_t bitReverseFlag)
{
    uint32_t  L = S->fftLen, l;
    float32_t invL, * pSrc;
    if (ifftFlag == 1U)
    {
        pSrc = p1 + 1;
        for(l=0; l<L; l++)
        {
            *pSrc = -*pSrc;
            pSrc += 2;
        }
    }
    switch (L)
    {
    case 16:
    case 128:
    case 1024:
        xtensa_cfft_radix8by2_f32  ( (xtensa_cfft_instance_f32 *) S, p1);
        break;
    case 32:
    case 256:
    case 2048:
        xtensa_cfft_radix8by4_f32  ( (xtensa_cfft_instance_f32 *) S, p1);
        break;
    case 64:
    case 512:
    case 4096:
        xtensa_radix8_butterfly_f32( p1, L, (float32_t *) S->pTwiddle, 1);
        break;
    }
    if ( bitReverseFlag )
        xtensa_bitreversal_32((uint32_t*)p1,S->bitRevLength,S->pBitRevTable);
    if (ifftFlag == 1U)
    {
        invL = 1.0f/(float32_t)L;
        pSrc = p1;
        for(l=0; l<L; l++)
        {
            *pSrc++ *=   invL ;
            *pSrc  = -(*pSrc) * invL;
            pSrc++;
        }
    }
}
