#include "console.h"
#ifndef __CFASSERT_H__
#define __CFASSERT_H__
#define ASSERT(e)  if (e) ; \
        else assertFail( #e, __FILE__, __LINE__ )
#ifdef DEBUG_EP2
#define IF_DEBUG_ASSERT(e)  if (e) ; \
        else assertFail( #e, __FILE__, __LINE__ )
#else
#define IF_DEBUG_ASSERT(e)
#endif
#define ASSERT_FAILED() assertFail( "", __FILE__, __LINE__ )
#define ASSERT_DMA_SAFE(PTR)
void assertFail(char *exp, char *file, int line);
void printAssertSnapshotData();
void storeAssertFileData(const char *file, int line);
void storeAssertHardfaultData(
    unsigned int r0,
    unsigned int r1,
    unsigned int r2,
    unsigned int r3,
    unsigned int r12,
    unsigned int lr,
    unsigned int pc,
    unsigned int psr);
void storeAssertTextData(const char *text);
bool cfAssertNormalStartTest(void);
#endif 