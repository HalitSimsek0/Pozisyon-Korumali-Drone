#include "xtensa_math.h"
#include "xtensa_common_tables.h"
float32_t xtensa_cos_f32(
  float32_t x)
{
  float32_t cosVal, fract, in;                   
  uint16_t index;                                
  float32_t a, b;                                
  int32_t n;
  float32_t findex;
  in = x * 0.159154943092f + 0.25f;
  n = (int32_t) in;
  if (in < 0.0f)
  {
    n--;
  }
  in = in - (float32_t) n;
  findex = (float32_t) FAST_MATH_TABLE_SIZE * in;
  index = ((uint16_t)findex) & 0x1ff;
  fract = findex - (float32_t) index;
  a = sinTable_f32[index];
  b = sinTable_f32[index+1];
  cosVal = (1.0f-fract)*a + fract*b;
  return (cosVal);
}