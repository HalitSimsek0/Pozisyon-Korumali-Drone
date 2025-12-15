#include "xtensa_math.h"
#include "xtensa_common_tables.h"
xtensa_status xtensa_rfft_fast_init_f32(
  xtensa_rfft_fast_instance_f32 * S,
  uint16_t fftLen)
{
  xtensa_cfft_instance_f32 * Sint;
  xtensa_status status = XTENSA_MATH_SUCCESS;
  Sint = &(S->Sint);
  Sint->fftLen = fftLen/2;
  S->fftLenRFFT = fftLen;
  switch (Sint->fftLen)
  {
  case 2048U:
    Sint->bitRevLength = XTENSABITREVINDEXTABLE_2048_TABLE_LENGTH;
    Sint->pBitRevTable = (uint16_t *)xtensaBitRevIndexTable2048;
		Sint->pTwiddle     = (float32_t *) twiddleCoef_2048;
		S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_4096;
    break;
  case 1024U:
    Sint->bitRevLength = XTENSABITREVINDEXTABLE_1024_TABLE_LENGTH;
    Sint->pBitRevTable = (uint16_t *)xtensaBitRevIndexTable1024;
		Sint->pTwiddle     = (float32_t *) twiddleCoef_1024;
		S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_2048;
    break;
  case 512U:
    Sint->bitRevLength = XTENSABITREVINDEXTABLE_512_TABLE_LENGTH;
    Sint->pBitRevTable = (uint16_t *)xtensaBitRevIndexTable512;
		Sint->pTwiddle     = (float32_t *) twiddleCoef_512;
		S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_1024;
    break;
  case 256U:
    Sint->bitRevLength = XTENSABITREVINDEXTABLE_256_TABLE_LENGTH;
    Sint->pBitRevTable = (uint16_t *)xtensaBitRevIndexTable256;
		Sint->pTwiddle     = (float32_t *) twiddleCoef_256;
		S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_512;
    break;
  case 128U:
    Sint->bitRevLength = XTENSABITREVINDEXTABLE_128_TABLE_LENGTH;
    Sint->pBitRevTable = (uint16_t *)xtensaBitRevIndexTable128;
		Sint->pTwiddle     = (float32_t *) twiddleCoef_128;
		S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_256;
    break;
  case 64U:
    Sint->bitRevLength = XTENSABITREVINDEXTABLE_64_TABLE_LENGTH;
    Sint->pBitRevTable = (uint16_t *)xtensaBitRevIndexTable64;
		Sint->pTwiddle     = (float32_t *) twiddleCoef_64;
		S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_128;
    break;
  case 32U:
    Sint->bitRevLength = XTENSABITREVINDEXTABLE_32_TABLE_LENGTH;
    Sint->pBitRevTable = (uint16_t *)xtensaBitRevIndexTable32;
		Sint->pTwiddle     = (float32_t *) twiddleCoef_32;
		S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_64;
    break;
  case 16U:
    Sint->bitRevLength = XTENSABITREVINDEXTABLE_16_TABLE_LENGTH;
    Sint->pBitRevTable = (uint16_t *)xtensaBitRevIndexTable16;
		Sint->pTwiddle     = (float32_t *) twiddleCoef_16;
		S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_32;
    break;
  default:
    status = XTENSA_MATH_ARGUMENT_ERROR;
    break;
  }
  return (status);
}