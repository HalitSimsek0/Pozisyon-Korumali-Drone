#ifndef __LEDSEQ_H__
#define __LEDSEQ_H__
#include <stdint.h>
#include <stdbool.h>
#include "led.h"
#define LEDSEQ_CHARGE_CYCLE_TIME_500MA  1000
#define LEDSEQ_CHARGE_CYCLE_TIME_MAX    500
#define LEDSEQ_WAITMS(X) (X)
#define LEDSEQ_STOP      -1
#define LEDSEQ_LOOP      -2
typedef struct {
  bool value;
  int action;
} ledseqStep_t;
typedef struct ledseqContext_s {
  ledseqStep_t* const sequence;
  struct ledseqContext_s* nextContext;
  int state;
  const led_t led;
} ledseqContext_t;
void ledseqInit(void);
bool ledseqTest(void);
void ledseqEnable(bool enable);
void ledseqRegisterSequence(ledseqContext_t* context);
bool ledseqRun(ledseqContext_t* context);
void ledseqRunBlocking(ledseqContext_t* context);
bool ledseqStop(ledseqContext_t* context);
void ledseqStopBlocking(ledseqContext_t* context);
void ledseqSetChargeLevel(const float chargeLevel);
extern ledseqContext_t seq_calibrated;
extern ledseqContext_t seq_alive;
extern ledseqContext_t seq_lowbat;
extern ledseqContext_t seq_linkUp;
extern ledseqContext_t seq_linkDown;
extern ledseqContext_t seq_charged;
extern ledseqContext_t seq_charging;
extern ledseqContext_t seq_testPassed;
extern ledseqContext_t seq_testFailed;
#endif
