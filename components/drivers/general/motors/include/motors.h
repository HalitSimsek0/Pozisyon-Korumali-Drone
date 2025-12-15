#ifndef __MOTORS_H__
#define __MOTORS_H__
#include <stdint.h>
#include <stdbool.h>
#include "driver/ledc.h"
#include "config.h"
#include "stm32_legacy.h"
#define MOTORS_PWM_BITS           LEDC_TIMER_8_BIT
#define MOTORS_PWM_PERIOD         ((1<<MOTORS_PWM_BITS) - 1)
// DÜZELTME D2: MOTORS_TIM_BEEP_CLK_FREQ kaldırıldı - buzzer/motor beep kullanılmıyor
#define NBR_OF_MOTORS 4
#define MOTOR_M1  0
#define MOTOR_M2  1
#define MOTOR_M3  2
#define MOTOR_M4  3
#ifdef CONFIG_TARGET_ESPLANE_V2_S2
    #define MOTOR3_GPIO  CONFIG_MOTOR01_PIN         
    #define MOTOR4_GPIO  CONFIG_MOTOR02_PIN        
    #define MOTOR1_GPIO  CONFIG_MOTOR03_PIN        
    #define MOTOR2_GPIO  CONFIG_MOTOR04_PIN        
#else
    #define MOTOR1_GPIO  CONFIG_MOTOR01_PIN         
    #define MOTOR2_GPIO  CONFIG_MOTOR02_PIN        
    #define MOTOR3_GPIO  CONFIG_MOTOR03_PIN        
    #define MOTOR4_GPIO  CONFIG_MOTOR04_PIN        
#endif
#define MOT_PWM_CH1  4      
#define MOT_PWM_CH2  5      
#define MOT_PWM_CH3  6      
#define MOT_PWM_CH4  7      
#define MOTORS_TEST_RATIO         (uint16_t)(0.2*(1<<16))
#define MOTORS_TEST_ON_TIME_MS    50
#define MOTORS_TEST_DELAY_TIME_MS 150
#define C4    262
#define DES4  277
#define D4    294
#define ES4   311
#define E4    330
#define F4    349
#define GES4  370
#define G4    392
#define AS4   415
#define A4    440
#define B4    466
#define H4    493
#define C5    523
#define DES5  554
#define D5    587
#define ES5   622
#define E5    659
#define F5    698
#define GES5  740
#define G5    783
#define AS5   830
#define A5    880
#define B5    932
#define H5    987
#define C6    1046
#define DES6  1108
#define D6    1174
#define ES6   1244
#define E6    1318
#define F6    1396
#define GES6  1479
#define G6    1567
#define AS6   1661
#define A6    1760
#define B6    1864
#define H6    1975
#define C7    2093
#define DES7  2217
#define D7    2349
#define ES7   2489
#define E7    2637
#define F7    2793
#define GES7  2959
#define G7    3135
#define AS7   3322
#define A7    3520
#define H7    3729
#define B7    3951
#define EIGHTS 125
#define QUAD 250
#define HALF 500
#define FULL 1000
#define STOP 0
typedef struct {
} MotorPerifDef;
extern const MotorPerifDef *motorMapDefaultBrushless[NBR_OF_MOTORS]; 
// DÜZELTME D2: testsound kaldırıldı - buzzer kullanılmıyor
bool pwm_timmer_init();
void motorsInit(const MotorPerifDef **motorMapSelect);
void motorsDeInit(const MotorPerifDef **motorMapSelect);
bool motorsTest(void);
void motorsSetRatio(uint32_t id, uint16_t ratio);
int motorsGetRatio(uint32_t id);
void motorsSetPulseWidth(uint32_t id, float pulseWidth_us);
void motorsCalibrateESC(void);
void motorsTestTask(void *params);
// DÜZELTME D2: motorsBeep kaldırıldı - buzzer kullanılmıyor
#endif 