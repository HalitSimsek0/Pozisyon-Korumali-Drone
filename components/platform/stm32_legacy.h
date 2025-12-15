#ifndef ESP32_BRIDGE_H
#define ESP32_BRIDGE_H
// NOT: Bu dosya ismi "stm32_legacy.h" olsa da, içeriği artık sadece FreeRTOS utility makroları
// ve STM32'den ESP32'ye port sırasında gerekli olan temel tipler için kullanılıyor.
// STM32'ye özgü GPIO register yapıları (GPIO_TypeDef gibi) ESP32'de kullanılmıyor
// ve kaldırılmıştır. ESP32'de GPIO işlemleri driver/gpio.h kullanılarak yapılıyor.
#include "esp_err.h"
#include "cfassert.h"
#ifndef __cplusplus
#endif
typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))
typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;
// KALDIRILDI: GPIO_TypeDef - ESP32'de kullanılmıyor, hiçbir yerde referans edilmiyor
// ESP32'de GPIO işlemleri driver/gpio.h API'si kullanılarak yapılıyor
#define TASK_LED_ID_NBR         1
#define TASK_RADIO_ID_NBR       2
#define TASK_STABILIZER_ID_NBR  3
#define TASK_ADC_ID_NBR         4
#define TASK_PM_ID_NBR          5
// Removed: TASK_PROXIMITY_ID_NBR - proximity hardware not available
typedef enum {
    Bit_RESET = 0,
    Bit_SET
} BitAction;
#define __IO
#ifndef FALSE
# define FALSE 0
#endif
#ifndef TRUE
# define TRUE 1
#endif
#define pdFALSE			( ( BaseType_t ) 0 )
#define pdTRUE			( ( BaseType_t ) 1 )
#define pdPASS			( pdTRUE )
#define pdFAIL			( pdFALSE )
#define errQUEUE_EMPTY	( ( BaseType_t ) 0 )
#define errQUEUE_FULL	( ( BaseType_t ) 0 )
// EKLE: STM32'den ESP32'ye port edilirken kullanılan legacy makrolar
// Bu makrolar FreeRTOS tick dönüşümleri için kullanılıyor
// ESP32'de de çalışıyor ama isimlendirme STM32'ye özgü
// 
// NOT: Bu makrolar geriye dönük uyumluluk için korunuyor.
// Yeni kod için ESP-IDF'nin pdMS_TO_TICKS() ve pdTICKS_TO_MS() kullanılabilir.

// Millisecond to Tick conversion
#define M2T(X) ((unsigned int)(X)/ portTICK_PERIOD_MS)  // Millisecond to FreeRTOS Tick

// Frequency to Tick conversion (for period calculation)
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))  // Frequency (Hz) to FreeRTOS Tick

// Tick to Millisecond conversion
#define T2M(X) ((unsigned int)(X)* portTICK_PERIOD_MS)  // FreeRTOS Tick to Millisecond

// Second to Tick conversion
#define S2T(X) ((portTickType)((X) * configTICK_RATE_HZ))  // Second to FreeRTOS Tick

// Tick to Second conversion
#define T2S(X) ((X) / (float)configTICK_RATE_HZ)  // FreeRTOS Tick to Second

// EKLE: Alternatif isimlendirme (daha açıklayıcı) - geriye dönük uyumluluk için
// Yeni kod için bu isimlendirmeler kullanılabilir:
#define MS_TO_TICKS(X) M2T(X)  // Alias for M2T - Millisecond to FreeRTOS Tick
#define FREQ_TO_TICKS(X) F2T(X)  // Alias for F2T - Frequency (Hz) to FreeRTOS Tick
#define TICKS_TO_MS(X) T2M(X)  // Alias for T2M - FreeRTOS Tick to Millisecond
#define SEC_TO_TICKS(X) S2T(X)  // Alias for S2T - Second to FreeRTOS Tick
#define TICKS_TO_SEC(X) T2S(X)  // Alias for T2S - FreeRTOS Tick to Second
#define assert_param(e)  if (e) ; \
    else assertFail( #e, __FILE__, __LINE__ )
uint64_t usecTimestamp(void);
#endif