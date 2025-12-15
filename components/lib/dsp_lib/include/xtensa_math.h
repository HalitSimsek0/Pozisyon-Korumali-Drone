#ifndef _XTENSA_MATH_H
#define _XTENSA_MATH_H
#if   defined ( __CC_XTENSA )
#elif defined ( __XTENSACC_VERSION ) && ( __XTENSACC_VERSION >= 6010050 )
#elif defined ( __GNUC__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#elif defined ( __ICCXTENSA__ )
#elif defined ( __TI_XTENSA__ )
#elif defined ( __CSMC__ )
#elif defined ( __TASKING__ )
#else
  #error Unknown compiler
#endif
#include "string.h"
#include "math.h"
#include "stdint.h"
#ifdef   __cplusplus
extern "C"
{
#endif
#define INDEX_MASK         0x0000003F
#ifndef PI
  #define PI               3.14159265358979f
#endif
#define FAST_MATH_TABLE_SIZE  512
  typedef enum
  {
    XTENSA_MATH_SUCCESS = 0,                
    XTENSA_MATH_ARGUMENT_ERROR = -1,        
    XTENSA_MATH_LENGTH_ERROR = -2,          
    XTENSA_MATH_SIZE_MISMATCH = -3,         
    XTENSA_MATH_NANINF = -4,                
    XTENSA_MATH_SINGULAR = -5,              
    XTENSA_MATH_TEST_FAILURE = -6           
  } xtensa_status;
  typedef float float32_t;
  typedef struct
  {
    uint16_t numTaps;     
    float32_t *pState;    
    float32_t *pCoeffs;   
  } xtensa_fir_instance_f32;
  void xtensa_fir_f32(
  const xtensa_fir_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);
  void xtensa_fir_init_f32(
  xtensa_fir_instance_f32 * S,
  uint16_t numTaps,
  float32_t * pCoeffs,
  float32_t * pState,
  uint32_t blockSize);
  typedef struct
  {
    uint32_t numStages;      
    float32_t *pState;       
    float32_t *pCoeffs;      
  } xtensa_biquad_casd_df1_inst_f32;
  void xtensa_biquad_cascade_df1_f32(
  const xtensa_biquad_casd_df1_inst_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);
  void xtensa_biquad_cascade_df1_init_f32(
  xtensa_biquad_casd_df1_inst_f32 * S,
  uint8_t numStages,
  float32_t * pCoeffs,
  float32_t * pState);
  typedef struct
  {
    uint16_t numRows;     
    uint16_t numCols;     
    float32_t *pData;     
  } xtensa_matrix_instance_f32;
  xtensa_status xtensa_mat_add_f32(
  const xtensa_matrix_instance_f32 * pSrcA,
  const xtensa_matrix_instance_f32 * pSrcB,
  xtensa_matrix_instance_f32 * pDst);
  xtensa_status xtensa_mat_cmplx_mult_f32(
  const xtensa_matrix_instance_f32 * pSrcA,
  const xtensa_matrix_instance_f32 * pSrcB,
  xtensa_matrix_instance_f32 * pDst);
  xtensa_status xtensa_mat_trans_f32(
  const xtensa_matrix_instance_f32 * pSrc,
  xtensa_matrix_instance_f32 * pDst);
  xtensa_status xtensa_mat_mult_f32(
  const xtensa_matrix_instance_f32 * pSrcA,
  const xtensa_matrix_instance_f32 * pSrcB,
  xtensa_matrix_instance_f32 * pDst);
  xtensa_status xtensa_mat_sub_f32(
  const xtensa_matrix_instance_f32 * pSrcA,
  const xtensa_matrix_instance_f32 * pSrcB,
  xtensa_matrix_instance_f32 * pDst);
  xtensa_status xtensa_mat_scale_f32(
  const xtensa_matrix_instance_f32 * pSrc,
  float32_t scale,
  xtensa_matrix_instance_f32 * pDst);
  void xtensa_mat_init_f32(
  xtensa_matrix_instance_f32 * S,
  uint16_t nRows,
  uint16_t nColumns,
  float32_t * pData);
  typedef struct
  {
    float32_t A0;          
    float32_t A1;          
    float32_t A2;          
    float32_t state[3];    
    float32_t Kp;          
    float32_t Ki;          
    float32_t Kd;          
  } xtensa_pid_instance_f32;
  void xtensa_pid_init_f32(
  xtensa_pid_instance_f32 * S,
  int32_t resetStateFlag);
  void xtensa_pid_reset_f32(
  xtensa_pid_instance_f32 * S);
  typedef struct
  {
    uint32_t nValues;           
    float32_t x1;               
    float32_t xSpacing;         
    float32_t *pYData;          
  } xtensa_linear_interp_instance_f32;
  typedef struct
  {
    uint16_t numRows;   
    uint16_t numCols;   
    float32_t *pData;   
  } xtensa_bilinear_interp_instance_f32;
  void xtensa_mult_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  float32_t * pDst,
  uint32_t blockSize);
  typedef struct
  {
    uint16_t fftLen;                   
    uint8_t ifftFlag;                  
    uint8_t bitReverseFlag;            
    float32_t *pTwiddle;               
    uint16_t *pBitRevTable;            
    uint16_t twidCoefModifier;         
    uint16_t bitRevFactor;             
    float32_t onebyfftLen;             
  } xtensa_cfft_radix2_instance_f32;
  xtensa_status xtensa_cfft_radix2_init_f32(
  xtensa_cfft_radix2_instance_f32 * S,
  uint16_t fftLen,
  uint8_t ifftFlag,
  uint8_t bitReverseFlag);
  void xtensa_cfft_radix2_f32(
  const xtensa_cfft_radix2_instance_f32 * S,
  float32_t * pSrc);
  typedef struct
  {
    uint16_t fftLen;                   
    uint8_t ifftFlag;                  
    uint8_t bitReverseFlag;            
    float32_t *pTwiddle;               
    uint16_t *pBitRevTable;            
    uint16_t twidCoefModifier;         
    uint16_t bitRevFactor;             
    float32_t onebyfftLen;             
  } xtensa_cfft_radix4_instance_f32;
  xtensa_status xtensa_cfft_radix4_init_f32(
  xtensa_cfft_radix4_instance_f32 * S,
  uint16_t fftLen,
  uint8_t ifftFlag,
  uint8_t bitReverseFlag);
  void xtensa_cfft_radix4_f32(
  const xtensa_cfft_radix4_instance_f32 * S,
  float32_t * pSrc);
  typedef struct
  {
    uint16_t fftLen;                   
    const float32_t *pTwiddle;         
    const uint16_t *pBitRevTable;      
    uint16_t bitRevLength;             
  } xtensa_cfft_instance_f32;
  void xtensa_cfft_f32(
  const xtensa_cfft_instance_f32 * S,
  float32_t * p1,
  uint8_t ifftFlag,
  uint8_t bitReverseFlag);
  typedef struct
  {
    uint32_t fftLenReal;                        
    uint16_t fftLenBy2;                         
    uint8_t ifftFlagR;                          
    uint8_t bitReverseFlagR;                    
    uint32_t twidCoefRModifier;                     
    float32_t *pTwiddleAReal;                   
    float32_t *pTwiddleBReal;                   
    xtensa_cfft_radix4_instance_f32 *pCfft;        
  } xtensa_rfft_instance_f32;
  xtensa_status xtensa_rfft_init_f32(
  xtensa_rfft_instance_f32 * S,
  xtensa_cfft_radix4_instance_f32 * S_CFFT,
  uint32_t fftLenReal,
  uint32_t ifftFlagR,
  uint32_t bitReverseFlag);
  void xtensa_rfft_f32(
  const xtensa_rfft_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst);
typedef struct
  {
    xtensa_cfft_instance_f32 Sint;      
    uint16_t fftLenRFFT;             
    float32_t * pTwiddleRFFT;        
  } xtensa_rfft_fast_instance_f32 ;
xtensa_status xtensa_rfft_fast_init_f32 (
   xtensa_rfft_fast_instance_f32 * S,
   uint16_t fftLen);
void xtensa_rfft_fast_f32(
  xtensa_rfft_fast_instance_f32 * S,
  float32_t * p, float32_t * pOut,
  uint8_t ifftFlag);
  typedef struct
  {
    uint16_t N;                          
    uint16_t Nby2;                       
    float32_t normalize;                 
    float32_t *pTwiddle;                 
    float32_t *pCosFactor;               
    xtensa_rfft_instance_f32 *pRfft;        
    xtensa_cfft_radix4_instance_f32 *pCfft; 
  } xtensa_dct4_instance_f32;
  xtensa_status xtensa_dct4_init_f32(
  xtensa_dct4_instance_f32 * S,
  xtensa_rfft_instance_f32 * S_RFFT,
  xtensa_cfft_radix4_instance_f32 * S_CFFT,
  uint16_t N,
  uint16_t Nby2,
  float32_t normalize);
  void xtensa_dct4_f32(
  const xtensa_dct4_instance_f32 * S,
  float32_t * pState,
  float32_t * pInlineBuffer);
  void xtensa_add_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  float32_t * pDst,
  uint32_t blockSize);
  void xtensa_sub_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  float32_t * pDst,
  uint32_t blockSize);
  void xtensa_scale_f32(
  float32_t * pSrc,
  float32_t scale,
  float32_t * pDst,
  uint32_t blockSize);
  void xtensa_abs_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);
  void xtensa_dot_prod_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  uint32_t blockSize,
  float32_t * result);
  void xtensa_offset_f32(
  float32_t * pSrc,
  float32_t offset,
  float32_t * pDst,
  uint32_t blockSize);
  void xtensa_negate_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);
  void xtensa_copy_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);
  void xtensa_fill_f32(
  float32_t value,
  float32_t * pDst,
  uint32_t blockSize);
  void xtensa_conv_f32(
  float32_t * pSrcA,
  uint32_t srcALen,
  float32_t * pSrcB,
  uint32_t srcBLen,
  float32_t * pDst);
  xtensa_status xtensa_conv_partial_f32(
  float32_t * pSrcA,
  uint32_t srcALen,
  float32_t * pSrcB,
  uint32_t srcBLen,
  float32_t * pDst,
  uint32_t firstIndex,
  uint32_t numPoints);
  typedef struct
  {
    uint8_t M;                  
    uint16_t numTaps;           
    float32_t *pCoeffs;         
    float32_t *pState;          
  } xtensa_fir_decimate_instance_f32;
  void xtensa_fir_decimate_f32(
  const xtensa_fir_decimate_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);
  xtensa_status xtensa_fir_decimate_init_f32(
  xtensa_fir_decimate_instance_f32 * S,
  uint16_t numTaps,
  uint8_t M,
  float32_t * pCoeffs,
  float32_t * pState,
  uint32_t blockSize);
  typedef struct
  {
    uint8_t L;                     
    uint16_t phaseLength;          
    float32_t *pCoeffs;            
    float32_t *pState;             
  } xtensa_fir_interpolate_instance_f32;
  void xtensa_fir_interpolate_f32(
  const xtensa_fir_interpolate_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);
  xtensa_status xtensa_fir_interpolate_init_f32(
  xtensa_fir_interpolate_instance_f32 * S,
  uint8_t L,
  uint16_t numTaps,
  float32_t * pCoeffs,
  float32_t * pState,
  uint32_t blockSize);
  typedef struct
  {
    uint8_t numStages;         
    float32_t *pState;         
    float32_t *pCoeffs;        
  } xtensa_biquad_cascade_df2T_instance_f32;
  typedef struct
  {
    uint8_t numStages;         
    float32_t *pState;         
    float32_t *pCoeffs;        
  } xtensa_biquad_cascade_stereo_df2T_instance_f32;
  void xtensa_biquad_cascade_df2T_f32(
  const xtensa_biquad_cascade_df2T_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);
  void xtensa_biquad_cascade_stereo_df2T_f32(
  const xtensa_biquad_cascade_stereo_df2T_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);
  void xtensa_biquad_cascade_df2T_init_f32(
  xtensa_biquad_cascade_df2T_instance_f32 * S,
  uint8_t numStages,
  float32_t * pCoeffs,
  float32_t * pState);
  void xtensa_biquad_cascade_stereo_df2T_init_f32(
  xtensa_biquad_cascade_stereo_df2T_instance_f32 * S,
  uint8_t numStages,
  float32_t * pCoeffs,
  float32_t * pState);
  typedef struct
  {
    uint16_t numStages;                  
    float32_t *pState;                   
    float32_t *pCoeffs;                  
  } xtensa_fir_lattice_instance_f32;
  void xtensa_fir_lattice_init_f32(
  xtensa_fir_lattice_instance_f32 * S,
  uint16_t numStages,
  float32_t * pCoeffs,
  float32_t * pState);
  void xtensa_fir_lattice_f32(
  const xtensa_fir_lattice_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);
  typedef struct
  {
    uint16_t numStages;                  
    float32_t *pState;                   
    float32_t *pkCoeffs;                 
    float32_t *pvCoeffs;                 
  } xtensa_iir_lattice_instance_f32;
  void xtensa_iir_lattice_f32(
  const xtensa_iir_lattice_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);
  void xtensa_iir_lattice_init_f32(
  xtensa_iir_lattice_instance_f32 * S,
  uint16_t numStages,
  float32_t * pkCoeffs,
  float32_t * pvCoeffs,
  float32_t * pState,
  uint32_t blockSize);
  typedef struct
  {
    uint16_t numTaps;    
    float32_t *pState;   
    float32_t *pCoeffs;  
    float32_t mu;        
  } xtensa_lms_instance_f32;
  void xtensa_lms_f32(
  const xtensa_lms_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pRef,
  float32_t * pOut,
  float32_t * pErr,
  uint32_t blockSize);
  void xtensa_lms_init_f32(
  xtensa_lms_instance_f32 * S,
  uint16_t numTaps,
  float32_t * pCoeffs,
  float32_t * pState,
  float32_t mu,
  uint32_t blockSize);
  typedef struct
  {
    uint16_t numTaps;     
    float32_t *pState;    
    float32_t *pCoeffs;   
    float32_t mu;         
    float32_t energy;     
    float32_t x0;         
  } xtensa_lms_norm_instance_f32;
  void xtensa_lms_norm_f32(
  xtensa_lms_norm_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pRef,
  float32_t * pOut,
  float32_t * pErr,
  uint32_t blockSize);
  void xtensa_lms_norm_init_f32(
  xtensa_lms_norm_instance_f32 * S,
  uint16_t numTaps,
  float32_t * pCoeffs,
  float32_t * pState,
  float32_t mu,
  uint32_t blockSize);
  void xtensa_correlate_f32(
  float32_t * pSrcA,
  uint32_t srcALen,
  float32_t * pSrcB,
  uint32_t srcBLen,
  float32_t * pDst);
  typedef struct
  {
    uint16_t numTaps;             
    uint16_t stateIndex;          
    float32_t *pState;            
    float32_t *pCoeffs;           
    uint16_t maxDelay;            
    int32_t *pTapDelay;           
  } xtensa_fir_sparse_instance_f32;
  void xtensa_fir_sparse_f32(
  xtensa_fir_sparse_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  float32_t * pScratchIn,
  uint32_t blockSize);
  void xtensa_fir_sparse_init_f32(
  xtensa_fir_sparse_instance_f32 * S,
  uint16_t numTaps,
  float32_t * pCoeffs,
  float32_t * pState,
  int32_t * pTapDelay,
  uint16_t maxDelay,
  uint32_t blockSize);
  void xtensa_sin_cos_f32(
  float32_t theta,
  float32_t * pSinVal,
  float32_t * pCosVal);
  void xtensa_cmplx_conj_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t numSamples);
  void xtensa_cmplx_mag_squared_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t numSamples);
   static inline float32_t xtensa_pid_f32(
  xtensa_pid_instance_f32 * S,
  float32_t in)
  {
    float32_t out;
    out = (S->A0 * in) +
      (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);
    S->state[1] = S->state[0];
    S->state[0] = in;
    S->state[2] = out;
    return (out);
  }
  xtensa_status xtensa_mat_inverse_f32(
  const xtensa_matrix_instance_f32 * src,
  xtensa_matrix_instance_f32 * dst);
   static inline void xtensa_clarke_f32(
  float32_t Ia,
  float32_t Ib,
  float32_t * pIalpha,
  float32_t * pIbeta)
  {
    *pIalpha = Ia;
    *pIbeta = ((float32_t) 0.57735026919 * Ia + (float32_t) 1.15470053838 * Ib);
  }
   static inline void xtensa_inv_clarke_f32(
  float32_t Ialpha,
  float32_t Ibeta,
  float32_t * pIa,
  float32_t * pIb)
  {
    *pIa = Ialpha;
    *pIb = -0.5f * Ialpha + 0.8660254039f * Ibeta;
  }
   static inline void xtensa_park_f32(
  float32_t Ialpha,
  float32_t Ibeta,
  float32_t * pId,
  float32_t * pIq,
  float32_t sinVal,
  float32_t cosVal)
  {
    *pId = Ialpha * cosVal + Ibeta * sinVal;
    *pIq = -Ialpha * sinVal + Ibeta * cosVal;
  }
   static inline void xtensa_inv_park_f32(
  float32_t Id,
  float32_t Iq,
  float32_t * pIalpha,
  float32_t * pIbeta,
  float32_t sinVal,
  float32_t cosVal)
  {
    *pIalpha = Id * cosVal - Iq * sinVal;
    *pIbeta = Id * sinVal + Iq * cosVal;
  }
   static inline float32_t xtensa_linear_interp_f32(
  xtensa_linear_interp_instance_f32 * S,
  float32_t x)
  {
    float32_t y;
    float32_t x0, x1;                            
    float32_t y0, y1;                            
    float32_t xSpacing = S->xSpacing;            
    int32_t i;                                   
    float32_t *pYData = S->pYData;               
    i = (int32_t) ((x - S->x1) / xSpacing);
    if (i < 0)
    {
      y = pYData[0];
    }
    else if ((uint32_t)i >= S->nValues)
    {
      y = pYData[S->nValues - 1];
    }
    else
    {
      x0 = S->x1 +  i      * xSpacing;
      x1 = S->x1 + (i + 1) * xSpacing;
      y0 = pYData[i];
      y1 = pYData[i + 1];
      y = y0 + (x - x0) * ((y1 - y0) / (x1 - x0));
    }
    return (y);
  }
  float32_t xtensa_sin_f32(
  float32_t x);
  float32_t xtensa_cos_f32(
  float32_t x);
inline float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
   static inline xtensa_status xtensa_sqrt_f32(
  float32_t in,
  float32_t * pOut)
  {
    if (in >= 0.0f)
    {
#if   (__FPU_USED == 1) && defined ( __CC_XTENSA   )
      *pOut = __sqrtf(in);
#elif (__FPU_USED == 1) && (defined(__XTENSACC_VERSION) && (__XTENSACC_VERSION >= 6010050))
      *pOut = __builtin_sqrtf(in);
#elif (__FPU_USED == 1) && defined(__GNUC__)
      *pOut = __builtin_sqrtf(in);
#elif (__FPU_USED == 1) && defined ( __ICCXTENSA__ ) && (__VER__ >= 6040000)
      __ASM("VSQRT.F32 %0,%1" : "=t"(*pOut) : "t"(in));
#else
      *pOut = sqrtf(in);
#endif
      return (XTENSA_MATH_SUCCESS);
    }
    else
    {
      *pOut = 0.0f;
      return (XTENSA_MATH_ARGUMENT_ERROR);
    }
  }
   static inline void xtensa_circularWrite_f32(
  int32_t * circBuffer,
  int32_t L,
  uint16_t * writeOffset,
  int32_t bufferInc,
  const int32_t * src,
  int32_t srcInc,
  uint32_t blockSize)
  {
    uint32_t i = 0U;
    int32_t wOffset;
    wOffset = *writeOffset;
    i = blockSize;
    while (i > 0U)
    {
      circBuffer[wOffset] = *src;
      src += srcInc;
      wOffset += bufferInc;
      if (wOffset >= L)
        wOffset -= L;
      i--;
    }
    *writeOffset = (uint16_t)wOffset;
  }
   static inline void xtensa_circularRead_f32(
  int32_t * circBuffer,
  int32_t L,
  int32_t * readOffset,
  int32_t bufferInc,
  int32_t * dst,
  int32_t * dst_base,
  int32_t dst_length,
  int32_t dstInc,
  uint32_t blockSize)
  {
    uint32_t i = 0U;
    int32_t rOffset, dst_end;
    rOffset = *readOffset;
    dst_end = (int32_t) (dst_base + dst_length);
    i = blockSize;
    while (i > 0U)
    {
      *dst = circBuffer[rOffset];
      dst += dstInc;
      if (dst == (int32_t *) dst_end)
      {
        dst = dst_base;
      }
      rOffset += bufferInc;
      if (rOffset >= L)
      {
        rOffset -= L;
      }
      i--;
    }
    *readOffset = rOffset;
  }
  void xtensa_power_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult);
  void xtensa_mean_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult);
  void xtensa_var_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult);
  void xtensa_rms_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult);
  void xtensa_std_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult);
  void xtensa_cmplx_mag_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t numSamples);
  void xtensa_cmplx_dot_prod_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  uint32_t numSamples,
  float32_t * realResult,
  float32_t * imagResult);
  void xtensa_cmplx_mult_real_f32(
  float32_t * pSrcCmplx,
  float32_t * pSrcReal,
  float32_t * pCmplxDst,
  uint32_t numSamples);
  void xtensa_min_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult,
  uint32_t * pIndex);
  void xtensa_max_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult,
  uint32_t * pIndex);
  void xtensa_cmplx_mult_cmplx_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  float32_t * pDst,
  uint32_t numSamples);
   static inline float32_t xtensa_bilinear_interp_f32(
  const xtensa_bilinear_interp_instance_f32 * S,
  float32_t X,
  float32_t Y)
  {
    float32_t out;
    float32_t f00, f01, f10, f11;
    float32_t *pData = S->pData;
    int32_t xIndex, yIndex, index;
    float32_t xdiff, ydiff;
    float32_t b1, b2, b3, b4;
    xIndex = (int32_t) X;
    yIndex = (int32_t) Y;
    if (xIndex < 0 || xIndex > (S->numRows - 1) || yIndex < 0 || yIndex > (S->numCols - 1))
    {
      return (0);
    }
    index = (xIndex - 1) + (yIndex - 1) * S->numCols;
    f00 = pData[index];
    f01 = pData[index + 1];
    index = (xIndex - 1) + (yIndex) * S->numCols;
    f10 = pData[index];
    f11 = pData[index + 1];
    b1 = f00;
    b2 = f01 - f00;
    b3 = f10 - f00;
    b4 = f00 - f01 - f10 + f11;
    xdiff = X - xIndex;
    ydiff = Y - yIndex;
    out = b1 + b2 * xdiff + b3 * ydiff + b4 * xdiff * ydiff;
    return (out);
  }
#if   defined ( __CC_XTENSA )
  #if defined( XTENSA_MATH_CM4 ) || defined( XTENSA_MATH_CM7)
    #define LOW_OPTIMIZATION_ENTER \
       _Pragma ("push")         \
       _Pragma ("O1")
  #else
    #define LOW_OPTIMIZATION_ENTER
  #endif
  #if defined ( XTENSA_MATH_CM4 ) || defined ( XTENSA_MATH_CM7 )
    #define LOW_OPTIMIZATION_EXIT \
       _Pragma ("pop")
  #else
    #define LOW_OPTIMIZATION_EXIT
  #endif
  #define IAR_ONLY_LOW_OPTIMIZATION_ENTER
  #define IAR_ONLY_LOW_OPTIMIZATION_EXIT
#elif defined (__XTENSACC_VERSION ) && ( __XTENSACC_VERSION >= 6010050 )
  #define LOW_OPTIMIZATION_ENTER
  #define LOW_OPTIMIZATION_EXIT
  #define IAR_ONLY_LOW_OPTIMIZATION_ENTER
  #define IAR_ONLY_LOW_OPTIMIZATION_EXIT
#elif defined ( __GNUC__ )
  #define LOW_OPTIMIZATION_ENTER \
       __attribute__(( optimize("-O1") ))
  #define LOW_OPTIMIZATION_EXIT
  #define IAR_ONLY_LOW_OPTIMIZATION_ENTER
  #define IAR_ONLY_LOW_OPTIMIZATION_EXIT
#elif defined ( __ICCXTENSA__ )
  #if defined ( XTENSA_MATH_CM4 ) || defined ( XTENSA_MATH_CM7 )
    #define LOW_OPTIMIZATION_ENTER \
       _Pragma ("optimize=low")
  #else
    #define LOW_OPTIMIZATION_ENTER
  #endif
  #define LOW_OPTIMIZATION_EXIT
  #if defined ( XTENSA_MATH_CM4 ) || defined ( XTENSA_MATH_CM7 )
    #define IAR_ONLY_LOW_OPTIMIZATION_ENTER \
       _Pragma ("optimize=low")
  #else
    #define IAR_ONLY_LOW_OPTIMIZATION_ENTER
  #endif
  #define IAR_ONLY_LOW_OPTIMIZATION_EXIT
#elif defined ( __TI_XTENSA__ )
  #define LOW_OPTIMIZATION_ENTER
  #define LOW_OPTIMIZATION_EXIT
  #define IAR_ONLY_LOW_OPTIMIZATION_ENTER
  #define IAR_ONLY_LOW_OPTIMIZATION_EXIT
#elif defined ( __CSMC__ )
  #define LOW_OPTIMIZATION_ENTER
  #define LOW_OPTIMIZATION_EXIT
  #define IAR_ONLY_LOW_OPTIMIZATION_ENTER
  #define IAR_ONLY_LOW_OPTIMIZATION_EXIT
#elif defined ( __TASKING__ )
  #define LOW_OPTIMIZATION_ENTER
  #define LOW_OPTIMIZATION_EXIT
  #define IAR_ONLY_LOW_OPTIMIZATION_ENTER
  #define IAR_ONLY_LOW_OPTIMIZATION_EXIT
#endif
#ifdef   __cplusplus
}
#endif
#if   defined ( __CC_XTENSA )
#elif defined ( __XTENSACC_VERSION ) && ( __XTENSACC_VERSION >= 6010050 )
#elif defined ( __GNUC__ )
#pragma GCC diagnostic pop
#elif defined ( __ICCXTENSA__ )
#elif defined ( __TI_XTENSA__ )
#elif defined ( __CSMC__ )
#elif defined ( __TASKING__ )
#else
  #error Unknown compiler
#endif
#endif 