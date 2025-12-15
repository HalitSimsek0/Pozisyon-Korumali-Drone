#include "crc.h"
#define WIDTH    (8 * sizeof(crc))
#define TOPBIT   (1 << (WIDTH - 1))
#if (REFLECT_DATA == TRUE)
#undef  REFLECT_DATA
#define REFLECT_DATA(X)			((unsigned char) reflect((X), 8))
#else
#undef  REFLECT_DATA
#define REFLECT_DATA(X)			(X)
#endif
#if (REFLECT_REMAINDER == TRUE)
#undef  REFLECT_REMAINDER
#define REFLECT_REMAINDER(X)	((crc) reflect((X), WIDTH))
#else
#undef  REFLECT_REMAINDER
#define REFLECT_REMAINDER(X)	(X)
#endif
static unsigned long reflect(unsigned long data, unsigned char nBits)
{
  unsigned long reflection = 0x00000000;
  unsigned char bit;
  for (bit = 0; bit < nBits; ++bit)
  {
    if (data & 0x01)
    {
      reflection |= (1 << ((nBits - 1) - bit));
    }
    data = (data >> 1);
  }
  return (reflection);
} 
crc crcSlow(void * datas, int nBytes)
{
  crc remainder = INITIAL_REMAINDER;
  unsigned char * message = datas;
  int byte;
  unsigned char bit;
  for (byte = 0; byte < nBytes; ++byte)
  {
    remainder ^= (REFLECT_DATA(message[byte]) << (WIDTH - 8));
    for (bit = 8; bit > 0; --bit)
    {
      if (remainder & TOPBIT)
      {
        remainder = (remainder << 1) ^ POLYNOMIAL;
      }
      else
      {
        remainder = (remainder << 1);
      }
    }
  }
  return (REFLECT_REMAINDER(remainder) ^ FINAL_XOR_VALUE);
} 
#ifdef CRC_FAST
static crc crcTable[256];
void crcInit(void)
{
  crc remainder;
  int dividend;
  unsigned char bit;
  for (dividend = 0; dividend < 256; ++dividend)
  {
    remainder = dividend << (WIDTH - 8);
    for (bit = 8; bit > 0; --bit)
    {
      if (remainder & TOPBIT)
      {
        remainder = (remainder << 1) ^ POLYNOMIAL;
      }
      else
      {
        remainder = (remainder << 1);
      }
    }
    crcTable[dividend] = remainder;
  }
} 
crc crcFast(void * datas, int nBytes)
{
  crc remainder = INITIAL_REMAINDER;
  unsigned char * message = datas;
  unsigned char data;
  int byte;
  for (byte = 0; byte < nBytes; ++byte)
  {
    data = REFLECT_DATA(message[byte]) ^ (remainder >> (WIDTH - 8));
    remainder = crcTable[data] ^ (remainder << 8);
  }
  return (REFLECT_REMAINDER(remainder) ^ FINAL_XOR_VALUE);
} 
#endif