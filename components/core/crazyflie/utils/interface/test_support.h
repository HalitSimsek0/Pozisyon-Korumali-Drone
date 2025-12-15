#pragma once
#if UNIT_TEST_MODE
  #define TESTABLE_STATIC
#else
  #define TESTABLE_STATIC static
#endif