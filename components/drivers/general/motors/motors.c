#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stm32_legacy.h"
#include "motors.h"
#include "pm_esplane.h"
#include "log.h"
#include "../../core/crazyflie/modules/interface/system.h"
#define DEBUG_MODULE "MOTORS"
#include "debug_cf.h"
static uint16_t motorsConvBitsTo16(uint16_t bits);
static uint16_t motorsConv16ToBits(uint16_t bits);
uint32_t motor_ratios[] = {0, 0, 0, 0};
// DÜZELTME D2: Buzzer/motor beep kullanılmıyor - motorsBeep, motorsPlayTone, motorsPlayMelody kaldırıldı
const MotorPerifDef **motorMap; 
const uint32_t MOTORS[] = {MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4};
// DÜZELTME D2: testsound kaldırıldı - buzzer kullanılmıyor
static bool isInit = false;
static bool isTimerInit = false;
ledc_channel_config_t motors_channel[NBR_OF_MOTORS] = {
    {
        .channel = MOT_PWM_CH1,
        .duty = 0,
        .gpio_num = MOTOR1_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH2,
        .duty = 0,
        .gpio_num = MOTOR2_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH3,
        .duty = 0,
        .gpio_num = MOTOR3_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH4,
        .duty = 0,
        .gpio_num = MOTOR4_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
};
static uint16_t motorsConvBitsTo16(uint16_t bits)
{
    return ((bits) << (16 - MOTORS_PWM_BITS));
}
static uint16_t motorsConv16ToBits(uint16_t bits)
{
    return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}
bool pwm_timmer_init()
{
    if (isTimerInit) {
        return TRUE;
    }
    uint32_t pwm_freq = 50;
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = MOTORS_PWM_BITS, 
        .freq_hz = pwm_freq,                
        .speed_mode = LEDC_LOW_SPEED_MODE, 
        .timer_num = LEDC_TIMER_0,			
    };
    if (ledc_timer_config(&ledc_timer) == ESP_OK) {
        isTimerInit = TRUE;
        return TRUE;
    }
    return FALSE;
}
void motorsInit(const MotorPerifDef **motorMapSelect)
{
    int i;
    if (isInit) {
        return;
    }
    motorMap = motorMapSelect;
    if (pwm_timmer_init() != TRUE) {
        DEBUG_PRINTE("Motor PWM timer init failed\n");
        return;
    }
    // EKLE: LEDC channel config hata kontrolü - Başarısız olursa init başarısız olur
    for (i = 0; i < NBR_OF_MOTORS; i++) {
        esp_err_t err = ledc_channel_config(&motors_channel[i]);
        if (err != ESP_OK) {
            DEBUG_PRINTE("Motor %d channel config failed: %s\n", i, esp_err_to_name(err));
            isInit = false;
            return;
        }
    }
    isInit = true;
    DEBUG_PRINTI("Motors initialized successfully\n");
}
void motorsDeInit(const MotorPerifDef **motorMapSelect)
{
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        ledc_stop(motors_channel[i].speed_mode, motors_channel[i].channel, 0);
    }
}
bool motorsTest(void)
{
    if (!isInit) {
        return false;
    }
    
    // DÜZELTME I4: Gerçek PWM çıkışı testi - PWM sinyali gönderip okunabilirliğini kontrol et
    // Timer frekans kontrolü
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        uint32_t freq = ledc_get_freq(motors_channel[i].speed_mode, motors_channel[i].timer_sel);
        if (freq == 0) {
            DEBUG_PRINTE("Motor %d PWM timer not configured (freq=0)\n", i);
            return false;
        }
    }
    
    // Gerçek PWM çıkışı testi: Her motor için test PWM değeri gönder ve oku
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        // Test PWM değeri (orta seviye - %7.5 duty cycle, yaklaşık 1500µs)
        uint16_t testRatio = (uint16_t)(0.075f * UINT16_MAX);
        uint32_t testDutyBits = motorsConv16ToBits(testRatio);
        
        // PWM sinyali gönder
        ledc_set_duty(motors_channel[i].speed_mode, motors_channel[i].channel, testDutyBits);
        ledc_update_duty(motors_channel[i].speed_mode, motors_channel[i].channel);
        
        // Kısa bir gecikme (PWM sinyalinin güncellenmesi için)
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // PWM değerini oku ve kontrol et
        uint32_t readDutyBits = ledc_get_duty(motors_channel[i].speed_mode, motors_channel[i].channel);
        uint16_t readRatio = motorsConvBitsTo16((uint16_t)readDutyBits);
        
        // Okunan değer gönderilen değere yakın olmalı (tolerans: %1)
        uint16_t tolerance = (uint16_t)(0.01f * UINT16_MAX);
        if (readRatio < (testRatio - tolerance) || readRatio > (testRatio + tolerance)) {
            DEBUG_PRINTE("Motor %d PWM output test FAILED: sent=%d, read=%d\n", i, testRatio, readRatio);
            // Test sonrası motoru durdur
            ledc_set_duty(motors_channel[i].speed_mode, motors_channel[i].channel, 0);
            ledc_update_duty(motors_channel[i].speed_mode, motors_channel[i].channel);
            return false;
        }
        
        // Test sonrası motoru durdur
        ledc_set_duty(motors_channel[i].speed_mode, motors_channel[i].channel, 0);
        ledc_update_duty(motors_channel[i].speed_mode, motors_channel[i].channel);
    }
    
    DEBUG_PRINTI("Motor PWM output test PASSED - all motors responding correctly\n");
    return true;
}
void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
    if (isInit) {
        uint16_t ratio;
        ASSERT(id < NBR_OF_MOTORS);
        float pulseWidth_us;
        if (!systemIsArmed()) {
            pulseWidth_us = 1000.0f;
        } else {
            pulseWidth_us = 1100.0f + ((float)ithrust / 65536.0f) * 900.0f; 
        }
        float dutyPercent = (pulseWidth_us / 20000.0f) * 100.0f; 
        dutyPercent = (dutyPercent < 5.0f) ? 5.0f : (dutyPercent > 10.0f) ? 10.0f : dutyPercent;
        ratio = (uint16_t)((dutyPercent / 100.0f) * UINT16_MAX);
        ledc_set_duty(motors_channel[id].speed_mode, motors_channel[id].channel, (uint32_t)motorsConv16ToBits(ratio));
        ledc_update_duty(motors_channel[id].speed_mode, motors_channel[id].channel);
        motor_ratios[id] = ratio;
#ifdef DEBUG_EP2
        DEBUG_PRINT_LOCAL("motors ID = %d ,ithrust_10bit = %d", id, (uint32_t)motorsConv16ToBits(ratio));
#endif
    }
}
void motorsSetPulseWidth(uint32_t id, float pulseWidth_us)
{
    if (isInit) {
        uint16_t ratio;
        ASSERT(id < NBR_OF_MOTORS);
        float dutyPercent = (pulseWidth_us / 20000.0f) * 100.0f;
        dutyPercent = (dutyPercent < 5.0f) ? 5.0f : (dutyPercent > 10.0f) ? 10.0f : dutyPercent;
        ratio = (uint16_t)((dutyPercent / 100.0f) * UINT16_MAX);
        ledc_set_duty(motors_channel[id].speed_mode, motors_channel[id].channel, (uint32_t)motorsConv16ToBits(ratio));
        ledc_update_duty(motors_channel[id].speed_mode, motors_channel[id].channel);
        motor_ratios[id] = ratio;
    }
}
void motorsCalibrateESC(void)
{
    if (!isInit) {
        return;
    }
    DEBUG_PRINTI("ESC Calibration: Starting...\n");
    DEBUG_PRINTI("WARNING: Remove propellers before calibration!\n");
    
    // Step 1: Set all motors to maximum PWM (2000us) for ESC calibration
    DEBUG_PRINTI("ESC Calibration: Setting all motors to MAX (2000us) - 5 seconds\n");
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        motorsSetPulseWidth(MOTORS[i], 2000.0f);
    }
    vTaskDelay(M2T(5000)); // Wait 5 seconds
    
    // Step 2: Set all motors to minimum PWM (1000us) to complete calibration
    DEBUG_PRINTI("ESC Calibration: Setting all motors to MIN (1000us) - 3 seconds\n");
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        motorsSetPulseWidth(MOTORS[i], 1000.0f);
    }
    vTaskDelay(M2T(3000)); // Wait 3 seconds
    
    // Step 3: Stop all motors
    DEBUG_PRINTI("ESC Calibration: Stopping all motors\n");
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        motorsSetPulseWidth(MOTORS[i], 1000.0f);
    }
    
    DEBUG_PRINTI("ESC Calibration: Complete!\n");
}
int motorsGetRatio(uint32_t id)
{
    int ratio;
    ASSERT(id < NBR_OF_MOTORS);
    ratio = motorsConvBitsTo16((uint16_t)ledc_get_duty(motors_channel[id].speed_mode, motors_channel[id].channel));
    return ratio;
}
// DÜZELTME D2: Buzzer/motor beep fonksiyonları kaldırıldı - buzzer kullanılmıyor
// Kaldırılan fonksiyonlar: motorsBeep, motorsPlayTone, motorsPlayMelody
LOG_GROUP_START(pwm)
LOG_ADD(LOG_UINT32, m1_pwm, &motor_ratios[0])
LOG_ADD(LOG_UINT32, m2_pwm, &motor_ratios[1])
LOG_ADD(LOG_UINT32, m3_pwm, &motor_ratios[2])
LOG_ADD(LOG_UINT32, m4_pwm, &motor_ratios[3])
LOG_GROUP_STOP(pwm)