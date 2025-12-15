#include "xtensa_math.h"
#include "xtensa_common_tables.h"
xtensa_status xtensa_cfft_radix4_init_f32(
  xtensa_cfft_radix4_instance_f32 * S,
  uint16_t fftLen,
  uint8_t ifftFlag,
  uint8_t bitReverseFlag)
{
  xtensa_status status = XTENSA_MATH_SUCCESS;
  S->fftLen = fftLen;
  S->pTwiddle = (float32_t *) twiddleCoef;
  S->ifftFlag = ifftFlag;
  S->bitReverseFlag = bitReverseFlag;
  switch (S->fftLen)
  {
  case 4096U:
    S->twidCoefModifier = 1U;
    S->bitRevFactor = 1U;
    S->pBitRevTable = (uint16_t *) xtensaBitRevTable;
    S->onebyfftLen = 0.000244140625;
    break;
  case 1024U:
    S->twidCoefModifier = 4U;
    S->bitRevFactor = 4U;
    S->pBitRevTable = (uint16_t *) & xtensaBitRevTable[3];
    S->onebyfftLen = 0.0009765625f;
    break;
  case 256U:
    S->twidCoefModifier = 16U;
    S->bitRevFactor = 16U;
    S->pBitRevTable = (uint16_t *) & xtensaBitRevTable[15];
    S->onebyfftLen = 0.00390625f;
    break;
  case 64U:
    S->twidCoefModifier = 64U;
    S->bitRevFactor = 64U;
    S->pBitRevTable = (uint16_t *) & xtensaBitRevTable[63];
    S->onebyfftLen = 0.015625f;
    break;
  case 16U:
    S->twidCoefModifier = 256U;
    S->bitRevFactor = 256U;
    S->pBitRevTable = (uint16_t *) & xtensaBitRevTable[255];
    S->onebyfftLen = 0.0625f;
    break;
  default:
    status = XTENSA_MATH_ARGUMENT_ERROR;
    break;
  }
  return (status);
}