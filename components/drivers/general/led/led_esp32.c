#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "led.h"
#include "stm32_legacy.h"
static unsigned int led_pin[] = {
    [LED_BLUE] = LED_GPIO_BLUE,
    [LED_RED]   = LED_GPIO_RED,
    [LED_GREEN] = LED_GPIO_GREEN,
};
static int led_polarity[] = {
    [LED_BLUE] = LED_POL_BLUE,
    [LED_RED]   = LED_POL_RED,
    [LED_GREEN] = LED_POL_GREEN,
};
static bool isInit = false;
void ledInit()
{
    int i;
    if (isInit) {
        return;
    }
    for (i = 0; i < LED_NUM; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << led_pin[i]),
            .pull_down_en = 0,
            .pull_up_en = 0,
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&io_conf);
        ledSet(i, 0);
    }
    isInit = true;
}
bool ledTest(void)
{
    ledSet(LED_GREEN, 1);
    ledSet(LED_RED, 0);
    vTaskDelay(M2T(250));
    ledSet(LED_GREEN, 0);
    ledSet(LED_RED, 1);
    vTaskDelay(M2T(250));
    ledClearAll();
    ledSet(LED_BLUE, 1);
    return isInit;
}
void ledClearAll(void)
{
    int i;
    for (i = 0; i < LED_NUM; i++) {
        ledSet(i, 0);
    }
}
void ledSetAll(void)
{
    int i;
    for (i = 0; i < LED_NUM; i++) {
        ledSet(i, 1);
    }
}
void ledSet(led_t led, bool value)
{
    if (led > LED_NUM || led == LED_NUM) {
        return;
    }
    if (led_polarity[led] == LED_POL_NEG) {
        value = !value;
    }
    if (value) {
        gpio_set_level(led_pin[led], 1);
    } else {
        gpio_set_level(led_pin[led], 0);
    }
}