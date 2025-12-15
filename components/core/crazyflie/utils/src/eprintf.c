#include "eprintf.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
static const char digit[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
                             'A', 'B', 'C', 'D', 'E', 'F'};
static int getIntLen (long int value)
{
  unsigned long int x = (value < 0) ? (unsigned long int)(-value) : (unsigned long int)value;
  int l = 1;
  while(x > 9)
  {
    l++;
    x /= 10;
  }
  return l;
}
int power(int a, int b)
{
  int x = 1;
  for (int i = 0; i < b; i++)
  {
    x *= a;
  }
  return x;
}
static int itoa10Unsigned(putc_t putcf, unsigned long long int num)
{
  int len = 0;
  if (num == 0)
  {
    putcf('0');
    return 1;
  }
  unsigned long long int i = 1;
  while ((num / i) > 9)
  {
    i *= 10L;
  }
  do
  {
    putcf(digit[(num / i) % 10L]);
    len++;
  }
  while (i /= 10L);
  return len;
}
static int itoa10(putc_t putcf, long long int num, int precision)
{
  int len = 0;
  if (num == 0)
  {
    putcf('0');
    return 1;
  }
  long long unsigned int n = num;
  if (num < 0)
  {
    n = -num;
    putcf('-');
    len++;
  }
  int numLenght = getIntLen(num);
  if (numLenght < precision)
  {
    int fillWithZero = precision - numLenght;
    while (fillWithZero > 0)
    {
      putcf('0');
      len++;
      fillWithZero--;
    }
  }
  return itoa10Unsigned(putcf, n) + len;
}
static int itoa16(putc_t putcf, uint64_t num, int width, char padChar)
{
  int len = 0;
  bool foundFirst = false;
  for (int i = 15; i >= 0; i--)
  {
    int shift = i * 4;
    uint64_t mask = (uint64_t)0x0F << shift;
    uint64_t val = (num & mask) >> shift;
    if (val > 0)
    {
      foundFirst = true;
    }
    if (foundFirst || i < width)
    {
      if (foundFirst)
      {
        putcf(digit[val]);
      }
      else
      {
        putcf(padChar);
      }
      len++;
    }
  }
  return len;
}
static int handleLongLong(putc_t putcf, char** fmt, unsigned long long int val, int width, char padChar)
{
  int len = 0;
  switch(*((*fmt)++))
  {
    case 'i':
    case 'd':
      len = itoa10(putcf, (long long int)val, 0);
      break;
    case 'u':
      len = itoa10Unsigned(putcf, val);
      break;
    case 'x':
    case 'X':
      len = itoa16(putcf, val, width, padChar);
      break;
    default:
      break;
  }
  return len;
}
static int handleLong(putc_t putcf, char** fmt, unsigned long int val, int width, char padChar)
{
  int len = 0;
  switch(*((*fmt)++))
  {
    case 'i':
    case 'd':
      len = itoa10(putcf, (long int)val, 0);
      break;
    case 'u':
      len = itoa10Unsigned(putcf, val);
      break;
    case 'x':
    case 'X':
      len = itoa16(putcf, val, width, padChar);
      break;
    default:
      break;
  }
  return len;
}
int evprintf(putc_t putcf, char * fmt, va_list ap)
{
  int len=0;
  double num;
  char* str;
  int precision;
  int width;
  char padChar;
  while (*fmt)
  {
    if (*fmt == '%')
    {
      precision = 6;
      padChar = ' ';
      width = 0;
      fmt++;
      while ('0' == *fmt)
      {
        padChar = '0';
        fmt++;
      }
			while(isdigit((unsigned)*fmt))
			{
				width *= 10;
				width += *fmt - '0';
				fmt++;
			}
      while (*fmt != '\0' && !isalpha((unsigned) *fmt))
      {
        if (*fmt == '.')
        {
          fmt++;
          if (isdigit((unsigned)*fmt))
          {
            precision = *fmt - '0';
            fmt++;
          }
        }
        else
        {
          fmt++;
        }
      }
      switch (*fmt++)
      {
        case 'i':
        case 'd':
          len += itoa10(putcf, va_arg(ap, int), 0);
          break;
        case 'u':
          len += itoa10Unsigned(putcf, va_arg(ap, unsigned int));
          break;
        case 'x':
        case 'X':
          len += itoa16(putcf, va_arg(ap, unsigned int), width, padChar);
          break;
        case 'l':
          if (*fmt == 'l') {
            fmt++;
            len += handleLongLong(putcf, &fmt, va_arg(ap, unsigned long long int), width, padChar);
          } else {
            len += handleLong(putcf, &fmt, va_arg(ap, unsigned long int), width, padChar);
          }
          break;
        case 'f':
          num = va_arg(ap, double);
          if(num<0)
          {
            putcf('-');
            num = -num;
            len++;
          }
          len += itoa10(putcf, (int)num, 0);
          putcf('.'); len++;
          len += itoa10(putcf, (num - (int)num) * power(10,precision), precision);
          break;
        case 's':
          str = va_arg(ap, char* );
          while(*str)
          {
            putcf(*str++);
            len++;
          }
          break;
        case 'c':
          putcf((char)va_arg(ap, int));
          len++;
          break;
        default:
          break;
      }
    }
    else
    {
      putcf(*fmt++);
      len++;
    }
  }
  return len;
}
int eprintf(putc_t putcf, char * fmt, ...)
{
  va_list ap;
  int len;
  va_start(ap, fmt);
  len = evprintf(putcf, fmt, ap);
  va_end(ap);
  return len;
}