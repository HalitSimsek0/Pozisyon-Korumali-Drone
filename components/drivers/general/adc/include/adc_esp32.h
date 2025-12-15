#ifndef ADC_H_
#define ADC_H_
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "config.h"
typedef struct __attribute__((packed)){
    uint16_t vref;
    uint16_t val;
}AdcPair;
typedef struct __attribute__((packed)){
    AdcPair vbat;
}AdcGroup;
typedef struct {
    uint16_t vbat;
    uint16_t vbatVref;
} AdcDeciGroup;
void adcInit(void);
bool adcTest(void);
float adcConvertToVoltageFloat(uint16_t v, uint16_t vref);
void adcDmaStart(void);
void adcDmaStop(void);
void adcInterruptHandler(void);
void adcTask(void *param);
float analogReadVoltage(uint32_t pin);  
#endif 
