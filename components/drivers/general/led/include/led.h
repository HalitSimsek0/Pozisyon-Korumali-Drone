#ifndef __LED_H__
#define __LED_H__
#include <stdbool.h>
#define LED_POL_POS 0
#define LED_POL_NEG 1
#define LED_GPIO_BLUE  CONFIG_LED_PIN_BLUE
#define LED_POL_BLUE   LED_POL_POS
#define LED_GPIO_GREEN CONFIG_LED_PIN_GREEN  
#define LED_POL_GREEN  LED_POL_POS
#define LED_GPIO_RED   CONFIG_LED_PIN_RED
#define LED_POL_RED    LED_POL_POS
#define LINK_LED         LED_GREEN
#define CHG_LED          LED_RED
#define LOWBAT_LED       LED_RED
#define LINK_DOWN_LED    LED_BLUE
#define SYS_LED          LED_BLUE
#define ERR_LED1         LED_RED
#define ERR_LED2         LED_RED
#define LED_NUM 3
typedef enum {LED_BLUE = 0, LED_RED, LED_GREEN} led_t;
void ledInit();
bool ledTest();
void ledClearAll(void);
void ledSetAll(void);
void ledSet(led_t led, bool value);
void ledTask(void *param);
#define ledSetRed(VALUE) ledSet(LED_RED, VALUE)
#define ledSetGreen(VALUE) ledSet(LED_GREEN, VALUE)
#endif