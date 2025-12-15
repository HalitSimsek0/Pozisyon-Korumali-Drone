#ifndef CONFIG_H_
#define CONFIG_H_
#include "usec_time.h"
#include "sdkconfig.h"
#define PROTOCOL_VERSION 4
#define QUAD_FORMATION_X
#ifdef CONFIG_TARGET_ESPLANE_V2_S2
#ifndef CONFIG_IDF_TARGET_ESP32S2
#error "ESPLANE_V2 hardware with ESP32S2 onboard"
#endif
#elif defined(CONFIG_TARGET_ESPLANE_V1)
#ifndef CONFIG_IDF_TARGET_ESP32
#error "ESPLANE_V1 hardware with ESP32 onboard"
#endif
#elif defined(CONFIG_TARGET_ESP32_S2_DRONE_V1_2)
#ifdef CONFIG_IDF_TARGET_ESP32
#error "ESP32_S2_DRONE_V1_2 hardware with ESP32S2/S3 onboard"
#endif
#endif
#define SYSTEM_TASK_PRI         1
#define PM_TASK_PRI             1
#define LEDSEQCMD_TASK_PRI      1
#define UDP_TX_TASK_PRI         3
#define CRTP_TX_TASK_PRI        2
#define UDP_RX_TASK_PRI         3
#define EXTRX_TASK_PRI          2
#define UART2_TASK_PRI          2
// Removed: SYSLINK_TASK_PRI - SYSLINK not used, WiFi communication used instead
// Removed: USBLINK_TASK_PRI - USB link not used on ESP32 WROOM-32U (no native USB OTG)
#define WIFILINK_TASK_PRI       3
#define CRTP_RX_TASK_PRI        2
#define CMD_HIGH_LEVEL_TASK_PRI 3
#define INFO_TASK_PRI           2
#define LOG_TASK_PRI            2
#define MEM_TASK_PRI            2
#define PARAM_TASK_PRI          2
// Removed: PROXIMITY_TASK_PRI - proximity hardware not available
#define FLOW_TASK_PRI           5
#define ZRANGER2_TASK_PRI       5
#define ZRANGER_TASK_PRI        5
#define SENSORS_TASK_PRI        6
#define STABILIZER_TASK_PRI     7
#define KALMAN_TASK_PRI         4
#if CONFIG_FREERTOS_UNICORE
  #undef KALMAN_TASK_PRI
  #define KALMAN_TASK_PRI         1
#endif
#define CMD_HIGH_LEVEL_TASK_NAME "CMDHL"
#define CRTP_RX_TASK_NAME       "CRTP-RX"
#define CRTP_TX_TASK_NAME       "CRTP-TX"
#define EXTRX_TASK_NAME         "EXTRX"
#define FLOW_TASK_NAME          "FLOW"
#define KALMAN_TASK_NAME        "KALMAN"
#define LEDSEQCMD_TASK_NAME     "LEDSEQCMD"
#define LOG_TASK_NAME           "LOG"
#define MEM_TASK_NAME           "MEM"
#define PARAM_TASK_NAME         "PARAM"
#define PM_TASK_NAME            "PWRMGNT"
// Removed: PROXIMITY_TASK_NAME - proximity hardware not available
#define SENSORS_TASK_NAME       "SENSORS"
#define STABILIZER_TASK_NAME    "STABILIZER"
// Removed: SYSLINK_TASK_NAME - SYSLINK not used, WiFi communication used instead
#define SYSTEM_TASK_NAME        "SYSTEM"
#define UART2_TASK_NAME         "UART2"
#define UDP_RX_TASK_NAME        "UDP_RX"
#define UDP_TX_TASK_NAME        "UDP_TX"
// Removed: USBLINK_TASK_NAME - USB link not used on ESP32 WROOM-32U (no native USB OTG)
#define WIFILINK_TASK_NAME      "WIFILINK"
#define ZRANGER2_TASK_NAME      "ZRANGER2"
#define ZRANGER_TASK_NAME       "ZRANGER"
#define configBASE_STACK_SIZE CONFIG_BASE_STACK_SIZE
#define CMD_HIGH_LEVEL_TASK_STACKSIZE (2 * configBASE_STACK_SIZE)
#define CRTP_RX_TASK_STACKSIZE        (3 * configBASE_STACK_SIZE)
#define CRTP_TX_TASK_STACKSIZE        (3 * configBASE_STACK_SIZE)
#define EXTRX_TASK_STACKSIZE          (1 * configBASE_STACK_SIZE)
#define FLOW_TASK_STACKSIZE           (3 * configBASE_STACK_SIZE)
#define KALMAN_TASK_STACKSIZE         (3 * configBASE_STACK_SIZE)
#define LEDSEQCMD_TASK_STACKSIZE      (2 * configBASE_STACK_SIZE)
#define LOG_TASK_STACKSIZE            (3 * configBASE_STACK_SIZE)
#define MEM_TASK_STACKSIZE            (2 * configBASE_STACK_SIZE)
#define PARAM_TASK_STACKSIZE          (2 * configBASE_STACK_SIZE)
#define PM_TASK_STACKSIZE             (4 * configBASE_STACK_SIZE)
#define SENSORS_TASK_STACKSIZE        (5 * configBASE_STACK_SIZE)
#define STABILIZER_TASK_STACKSIZE     (5 * configBASE_STACK_SIZE)
// Removed: SYSLINK_TASK_STACKSIZE - SYSLINK not used, WiFi communication used instead
#define SYSTEM_TASK_STACKSIZE         (6 * configBASE_STACK_SIZE)
#define UART2_TASK_STACKSIZE          (1 * configBASE_STACK_SIZE)
#define UDP_RX_TASK_STACKSIZE         (4 * configBASE_STACK_SIZE)
#define UDP_TX_TASK_STACKSIZE         (4 * configBASE_STACK_SIZE)
// Removed: USBLINK_TASK_STACKSIZE - USB link not used on ESP32 WROOM-32U (no native USB OTG)
#define WIFILINK_TASK_STACKSIZE       (4 * configBASE_STACK_SIZE)
#define ZRANGER2_TASK_STACKSIZE       (4 * configBASE_STACK_SIZE)
#define ZRANGER_TASK_STACKSIZE        (2 * configBASE_STACK_SIZE)
// Removed: RADIO_* definitions - Radio/Crazyradio not used, WiFi communication used instead
#define PROPELLER_BALANCE_TEST_THRESHOLD  2.5f
#if defined(UART_OUTPUT_TRACE_DATA) && defined(ADC_OUTPUT_RAW_DATA)
#  error "Can't define UART_OUTPUT_TRACE_DATA and ADC_OUTPUT_RAW_DATA at the same time"
#endif
#if defined(UART_OUTPUT_TRACE_DATA) || defined(ADC_OUTPUT_RAW_DATA) || defined(IMU_OUTPUT_RAW_DATA_ON_UART)
#define UART_OUTPUT_RAW_DATA_ONLY
#endif
#if defined(UART_OUTPUT_TRACE_DATA) && defined(T_LAUNCH_ACC)
#  error "UART_OUTPUT_TRACE_DATA and T_LAUNCH_ACC doesn't work at the same time yet due to dma sharing..."
#endif
#endif 