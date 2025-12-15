#ifndef __DECK_DIGITAL_H__
#define __DECK_DIGITAL_H__
#include <stdint.h>
#define LOW 0x0
#define HIGH 0x1
#define INPUT           0x0
#define OUTPUT          0x1
#define INPUT_PULLUP    0x2
#define INPUT_PULLDOWN  0x3
void pinMode(uint32_t pin, uint32_t mode);
void digitalWrite(uint32_t pin, uint32_t val);
int digitalRead(uint32_t pin);
#endif