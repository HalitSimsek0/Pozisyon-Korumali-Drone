#pragma once
#include "esp_now.h"
#include "esp_event.h"
#include "espnow.h"
#ifdef __cplusplus
extern "C" {
#endif 
#define ESP_EVENT_ESPNOW_CTRL_BIND      (ESP_EVENT_ESPNOW_CTRL_BASE + 0)
#define ESP_EVENT_ESPNOW_CTRL_UNBIND    (ESP_EVENT_ESPNOW_CTRL_BASE + 1)
typedef enum {
    ESPNOW_ATTRIBUTE_BASE           = 0x0000,
    ESPNOW_ATTRIBUTE_POWER          = 0x0001,
    ESPNOW_ATTRIBUTE_POWER_ADD      = 0x0002,
    ESPNOW_ATTRIBUTE_ATTRIBUTE      = 0x0003,
    ESPNOW_ATTRIBUTE_LIGHT_BASE     = 0x0100,
    ESPNOW_ATTRIBUTE_BRIGHTNESS     = 0x0101,
    ESPNOW_ATTRIBUTE_BRIGHTNESS_ADD = 0x0102,
    ESPNOW_ATTRIBUTE_HUE            = 0x0103,
    ESPNOW_ATTRIBUTE_HUE_ADD        = 0x0104,
    ESPNOW_ATTRIBUTE_SATURATION     = 0x0105,
    ESPNOW_ATTRIBUTE_SATURATION_ADD = 0x0106,
    ESPNOW_ATTRIBUTE_WARM           = 0x0107,
    ESPNOW_ATTRIBUTE_WARM_ADD       = 0x0108,
    ESPNOW_ATTRIBUTE_COLD           = 0x0109,
    ESPNOW_ATTRIBUTE_COLD_ADD       = 0x010a,
    ESPNOW_ATTRIBUTE_RED            = 0x010b,
    ESPNOW_ATTRIBUTE_RED_ADD        = 0x010c,
    ESPNOW_ATTRIBUTE_GREEN          = 0x010d,
    ESPNOW_ATTRIBUTE_GREEN_ADD      = 0x010e,
    ESPNOW_ATTRIBUTE_BLUE           = 0x010f,
    ESPNOW_ATTRIBUTE_BLUE_ADD       = 0x0110,
    ESPNOW_ATTRIBUTE_MODE           = 0x0111,
    ESPNOW_ATTRIBUTE_MODE_ADD       = 0x0112,
    ESPNOW_ATTRIBUTE_BUTTON_BASE    = 0x0200,
    ESPNOW_ATTRIBUTE_KEY_1          = 0x0201,
    ESPNOW_ATTRIBUTE_KEY_2          = 0x0202,
    ESPNOW_ATTRIBUTE_KEY_3          = 0x0203,
    ESPNOW_ATTRIBUTE_KEY_4          = 0x0204,
    ESPNOW_ATTRIBUTE_KEY_5          = 0x0205,
    ESPNOW_ATTRIBUTE_KEY_6          = 0x0206,
    ESPNOW_ATTRIBUTE_KEY_7          = 0x0207,
    ESPNOW_ATTRIBUTE_KEY_8          = 0x0208,
    ESPNOW_ATTRIBUTE_KEY_9          = 0x0209,
    ESPNOW_ATTRIBUTE_KEY_10         = 0x0210,
} espnow_attribute_t;
typedef struct {
    uint8_t mac[6];                         
    espnow_attribute_t initiator_attribute; 
} espnow_ctrl_bind_info_t;
typedef struct {
#ifdef CONFIG_ESPNOW_CONTROL_AUTO_CHANNEL_SENDING
    espnow_frame_head_t frame_head;
#endif
    espnow_attribute_t initiator_attribute;         
    espnow_attribute_t responder_attribute;         
    union {
        bool responder_value_b;   
        struct {
            int responder_value_i;
            int status_value_i;
            int left_x_value_f;
            int left_y_value_f;
            int right_x_value_f;
            int right_y_value_f;
            int channel_one_value_i;
            int channel_two_value_i;
        };
        struct {
            uint32_t responder_value_s_flag : 24; 
            uint8_t responder_value_s_size;       
        };
    };
    char responder_value_s[0];   
} espnow_ctrl_data_t;
typedef bool (* espnow_ctrl_bind_cb_t)(espnow_attribute_t initiator_attribute, uint8_t mac[6], int8_t rssi);
typedef void (* espnow_ctrl_data_cb_t)(espnow_attribute_t initiator_attribute,
                                       espnow_attribute_t responder_attribute,
                                       uint32_t responder_value,
                                       int status_value_i,
                                       int lx_value,
                                       int ly_value,
                                       int rx_value,
                                       int ry_value,
                                       int channel_one_value,
                                       int channel_two_value);
typedef void (* espnow_ctrl_data_raw_cb_t)(espnow_addr_t src_addr, espnow_ctrl_data_t *data, wifi_pkt_rx_ctrl_t *rx_ctrl);
esp_err_t espnow_ctrl_initiator_bind(espnow_attribute_t initiator_attribute, bool enable);
esp_err_t espnow_ctrl_initiator_send(espnow_attribute_t initiator_attribute, espnow_attribute_t responder_attribute, uint32_t responder_value, int status,
                                    int x_value, int y_value, int rx_value, int ry_value, int channel_one_value, int channel_two_value);
esp_err_t espnow_ctrl_responder_bind(uint32_t wait_ms, int8_t rssi, espnow_ctrl_bind_cb_t cb);
esp_err_t espnow_ctrl_responder_data(espnow_ctrl_data_cb_t cb);
esp_err_t espnow_ctrl_responder_get_bindlist(espnow_ctrl_bind_info_t *list, size_t *size);
esp_err_t espnow_ctrl_responder_set_bindlist(const espnow_ctrl_bind_info_t *info);
esp_err_t espnow_ctrl_responder_remove_bindlist(const espnow_ctrl_bind_info_t *info);
esp_err_t espnow_ctrl_send(const espnow_addr_t dest_addr, const espnow_ctrl_data_t *data, const espnow_frame_head_t *frame_head, TickType_t wait_ticks);
esp_err_t espnow_ctrl_recv(espnow_ctrl_data_raw_cb_t cb);
#ifdef __cplusplus
}
#endif 