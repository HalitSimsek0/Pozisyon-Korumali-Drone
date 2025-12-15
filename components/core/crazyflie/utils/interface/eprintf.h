#include <stdarg.h>
#ifndef	__EPRINTF_H__
#define __EPRINTF_H__
typedef int (*putc_t)(int c);
int eprintf(putc_t putcf, char * fmt, ...) 
    __attribute__ (( format(printf, 2, 3) ));
int evprintf(putc_t putcf, char * fmt, va_list ap);
#endif 