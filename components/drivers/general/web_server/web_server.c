#include "web_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "param.h"
#include "config.h"
#include "stm32_legacy.h"
#include "commander.h"
#include "stabilizer.h"
#include "stabilizer_types.h"
// Removed: #include "crtp_commander_high_level.h" - high level commander removed
#include "crtp_commander.h"
#include "sensors.h"
#include "motors.h"
#include "pm_esplane.h"
#include "system.h"
#include "pid_nvs.h"  // DÜZELTME K4: pid_nvs.c API'sini kullanmak için
#include "pid.h"      // DÜZELTME K4: PidObject struct'ı için
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define DEBUG_MODULE "WEB_SERVER"
#include "debug_cf.h"
static void webFailsafeTask(void *arg);
static int read_request_body(httpd_req_t *req, char *buf, size_t buf_size);
static bool extract_json_float(const char* json, const char* key, float* value);
static httpd_handle_t server = NULL;
static bool isInit = false;
static volatile int64_t lastControlUs = 0;
// Failsafe timeout: Joystick gönderimi genellikle 20-50Hz (20-50ms aralık)
// Yavaş network veya düşük frekanslı gönderim için 1.5 saniye timeout kullanıyoruz
// Bu, 20Hz gönderimde 30 heartbeat, 10Hz gönderimde 15 heartbeat'e karşılık gelir
#define WEB_CONTROL_TIMEOUT_US (1500000)  // 1.5 saniye - Yavaş network ve düşük frekanslı gönderim için güvenli
static void get_pid_param(const char* group, const char* name, float* value) {
    paramVarId_t id = paramGetVarId((char*)group, (char*)name);
    if (PARAM_VARID_IS_VALID(id)) {
        *value = paramGetFloat(id);
    }
}
static void __attribute__((unused)) set_pid_param(const char* group, const char* name, float value) {
    paramVarId_t id = paramGetVarId((char*)group, (char*)name);
    if (PARAM_VARID_IS_VALID(id)) {
        paramSetFloat(id, value);
    }
}
// DÜZELTME K4: pid_nvs.c API'sini kullanmak için - web server kendi NVS implementasyonunu kaldırdı
// set_pid_param_with_nvs() - PID parametresini anında güncelle (param sistemi üzerinden)
static void set_pid_param_with_nvs(const char* group, const char* name, float value) {
    paramVarId_t id = paramGetVarId((char*)group, (char*)name);
    if (PARAM_VARID_IS_VALID(id)) {
        paramSetFloat(id, value);  // Anında etkili - PID parametresi runtime'da değişir
    }
}
// DÜZELTME K4: pid_nvs.c API'sini kullanarak PID parametrelerini kaydet
// param sisteminden pid_config_t struct'ını doldur, integration limitleri PID objelerinden oku
static void save_all_pid_params_to_nvs(void) {
    // External PID objects from attitude_pid_controller.c
    extern PidObject pidRollRate;
    extern PidObject pidPitchRate;
    extern PidObject pidYawRate;
    extern PidObject pidRoll;
    extern PidObject pidPitch;
    extern PidObject pidYaw;
    
    pid_config_t pidConfig;
    
    // Param sisteminden PID katsayılarını oku
    get_pid_param("pid_attitude", "roll_kp", &pidConfig.roll_kp);
    get_pid_param("pid_attitude", "roll_ki", &pidConfig.roll_ki);
    get_pid_param("pid_attitude", "roll_kd", &pidConfig.roll_kd);
    get_pid_param("pid_attitude", "roll_kf", &pidConfig.roll_kf);
    get_pid_param("pid_attitude", "pitch_kp", &pidConfig.pitch_kp);
    get_pid_param("pid_attitude", "pitch_ki", &pidConfig.pitch_ki);
    get_pid_param("pid_attitude", "pitch_kd", &pidConfig.pitch_kd);
    get_pid_param("pid_attitude", "pitch_kf", &pidConfig.pitch_kf);
    get_pid_param("pid_attitude", "yaw_kp", &pidConfig.yaw_kp);
    get_pid_param("pid_attitude", "yaw_ki", &pidConfig.yaw_ki);
    get_pid_param("pid_attitude", "yaw_kd", &pidConfig.yaw_kd);
    get_pid_param("pid_attitude", "yaw_kf", &pidConfig.yaw_kf);
    
    get_pid_param("pid_rate", "roll_kp", &pidConfig.roll_rate_kp);
    get_pid_param("pid_rate", "roll_ki", &pidConfig.roll_rate_ki);
    get_pid_param("pid_rate", "roll_kd", &pidConfig.roll_rate_kd);
    get_pid_param("pid_rate", "pitch_kp", &pidConfig.pitch_rate_kp);
    get_pid_param("pid_rate", "pitch_ki", &pidConfig.pitch_rate_ki);
    get_pid_param("pid_rate", "pitch_kd", &pidConfig.pitch_rate_kd);
    get_pid_param("pid_rate", "yaw_kp", &pidConfig.yaw_rate_kp);
    get_pid_param("pid_rate", "yaw_ki", &pidConfig.yaw_rate_ki);
    get_pid_param("pid_rate", "yaw_kd", &pidConfig.yaw_rate_kd);
    
    // Integration limitleri PID objelerinden direkt oku (param sisteminde yok)
    pidConfig.roll_iLimit = pidRoll.iLimit;
    pidConfig.pitch_iLimit = pidPitch.iLimit;
    pidConfig.yaw_iLimit = pidYaw.iLimit;
    pidConfig.roll_rate_iLimit = pidRollRate.iLimit;
    pidConfig.pitch_rate_iLimit = pidPitchRate.iLimit;
    pidConfig.yaw_rate_iLimit = pidYawRate.iLimit;
    
    // pid_nvs.c API'sini kullanarak kaydet
    esp_err_t err = pidNvsSave(&pidConfig);
    if (err == ESP_OK) {
        // PID objelerine de uygula (anında etkili olması için)
        pidNvsApply(&pidConfig);
        DEBUG_PRINTD("PID parameters saved to NVS using pid_nvs.c API");
    } else {
        DEBUG_PRINTE("Failed to save PID parameters to NVS: %s", esp_err_to_name(err));
    }
}
// DÜZELTME K4: webServerLoadPidParamsFromNVS() kaldırıldı
// PID parametreleri zaten boot'ta attitude_pid_controller.c içinde pidNvsLoad() ile yükleniyor
static esp_err_t get_pid_params_handler(httpd_req_t *req) {
    float roll_kp = 0.0f, roll_ki = 0.0f, roll_kd = 0.0f;
    float pitch_kp = 0.0f, pitch_ki = 0.0f, pitch_kd = 0.0f;
    float yaw_kp = 0.0f, yaw_ki = 0.0f, yaw_kd = 0.0f;
    float roll_rate_kp = 0.0f, roll_rate_ki = 0.0f, roll_rate_kd = 0.0f;
    float pitch_rate_kp = 0.0f, pitch_rate_ki = 0.0f, pitch_rate_kd = 0.0f;
    float yaw_rate_kp = 0.0f, yaw_rate_ki = 0.0f, yaw_rate_kd = 0.0f;
    get_pid_param("pid_attitude", "roll_kp", &roll_kp);
    get_pid_param("pid_attitude", "roll_ki", &roll_ki);
    get_pid_param("pid_attitude", "roll_kd", &roll_kd);
    get_pid_param("pid_attitude", "pitch_kp", &pitch_kp);
    get_pid_param("pid_attitude", "pitch_ki", &pitch_ki);
    get_pid_param("pid_attitude", "pitch_kd", &pitch_kd);
    get_pid_param("pid_attitude", "yaw_kp", &yaw_kp);
    get_pid_param("pid_attitude", "yaw_ki", &yaw_ki);
    get_pid_param("pid_attitude", "yaw_kd", &yaw_kd);
    get_pid_param("pid_rate", "roll_kp", &roll_rate_kp);
    get_pid_param("pid_rate", "roll_ki", &roll_rate_ki);
    get_pid_param("pid_rate", "roll_kd", &roll_rate_kd);
    get_pid_param("pid_rate", "pitch_kp", &pitch_rate_kp);
    get_pid_param("pid_rate", "pitch_ki", &pitch_rate_ki);
    get_pid_param("pid_rate", "pitch_kd", &pitch_rate_kd);
    get_pid_param("pid_rate", "yaw_kp", &yaw_rate_kp);
    get_pid_param("pid_rate", "yaw_ki", &yaw_rate_ki);
    get_pid_param("pid_rate", "yaw_kd", &yaw_rate_kd);
    char resp[1024];
    snprintf(resp, sizeof(resp),
        "{\"success\":true,"
        "\"attitude\":{"
        "\"roll\":{\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f},"
        "\"pitch\":{\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f},"
        "\"yaw\":{\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f}},"
        "\"rate\":{"
        "\"roll\":{\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f},"
        "\"pitch\":{\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f},"
        "\"yaw\":{\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f}}}",
        roll_kp, roll_ki, roll_kd,
        pitch_kp, pitch_ki, pitch_kd,
        yaw_kp, yaw_ki, yaw_kd,
        roll_rate_kp, roll_rate_ki, roll_rate_kd,
        pitch_rate_kp, pitch_rate_ki, pitch_rate_kd,
        yaw_rate_kp, yaw_rate_ki, yaw_rate_kd
    );
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
static esp_err_t set_pid_params_handler(httpd_req_t *req) {
    char content[2048];
    int ret = read_request_body(req, content, sizeof(content));
    if (ret < 0) {
        if (req->content_len > sizeof(content) - 1) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Payload too large");
        } else {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
        }
        return ESP_FAIL;
    }
    if (ret == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty request body");
        return ESP_FAIL;
    }
    int updated = 0;
    float value;
    char* attitude_pos = strstr(content, "\"attitude\"");
    if (attitude_pos) {
        char* roll_pos = strstr(attitude_pos, "\"roll\"");
        if (roll_pos) {
            if (extract_json_float(roll_pos, "kp", &value)) {
                set_pid_param_with_nvs("pid_attitude", "roll_kp", value);
                updated++;
            }
            if (extract_json_float(roll_pos, "ki", &value)) {
                set_pid_param_with_nvs("pid_attitude", "roll_ki", value);
                updated++;
            }
            if (extract_json_float(roll_pos, "kd", &value)) {
                set_pid_param_with_nvs("pid_attitude", "roll_kd", value);
                updated++;
            }
        }
        char* pitch_pos = strstr(attitude_pos, "\"pitch\"");
        if (pitch_pos) {
            if (extract_json_float(pitch_pos, "kp", &value)) {
                set_pid_param_with_nvs("pid_attitude", "pitch_kp", value);
                updated++;
            }
            if (extract_json_float(pitch_pos, "ki", &value)) {
                set_pid_param_with_nvs("pid_attitude", "pitch_ki", value);
                updated++;
            }
            if (extract_json_float(pitch_pos, "kd", &value)) {
                set_pid_param_with_nvs("pid_attitude", "pitch_kd", value);
                updated++;
            }
        }
        char* yaw_pos = strstr(attitude_pos, "\"yaw\"");
        if (yaw_pos) {
            if (extract_json_float(yaw_pos, "kp", &value)) {
                set_pid_param_with_nvs("pid_attitude", "yaw_kp", value);
                updated++;
            }
            if (extract_json_float(yaw_pos, "ki", &value)) {
                set_pid_param_with_nvs("pid_attitude", "yaw_ki", value);
                updated++;
            }
            if (extract_json_float(yaw_pos, "kd", &value)) {
                set_pid_param_with_nvs("pid_attitude", "yaw_kd", value);
                updated++;
            }
        }
    }
    char* rate_pos = strstr(content, "\"rate\"");
    if (rate_pos) {
        char* roll_rate_pos = strstr(rate_pos, "\"roll\"");
        if (roll_rate_pos) {
            if (extract_json_float(roll_rate_pos, "kp", &value)) {
                set_pid_param_with_nvs("pid_rate", "roll_kp", value);
                updated++;
            }
            if (extract_json_float(roll_rate_pos, "ki", &value)) {
                set_pid_param_with_nvs("pid_rate", "roll_ki", value);
                updated++;
            }
            if (extract_json_float(roll_rate_pos, "kd", &value)) {
                set_pid_param_with_nvs("pid_rate", "roll_kd", value);
                updated++;
            }
        }
        char* pitch_rate_pos = strstr(rate_pos, "\"pitch\"");
        if (pitch_rate_pos) {
            if (extract_json_float(pitch_rate_pos, "kp", &value)) {
                set_pid_param_with_nvs("pid_rate", "pitch_kp", value);
                updated++;
            }
            if (extract_json_float(pitch_rate_pos, "ki", &value)) {
                set_pid_param_with_nvs("pid_rate", "pitch_ki", value);
                updated++;
            }
            if (extract_json_float(pitch_rate_pos, "kd", &value)) {
                set_pid_param_with_nvs("pid_rate", "pitch_kd", value);
                updated++;
            }
        }
        char* yaw_rate_pos = strstr(rate_pos, "\"yaw\"");
        if (yaw_rate_pos) {
            if (extract_json_float(yaw_rate_pos, "kp", &value)) {
                set_pid_param_with_nvs("pid_rate", "yaw_kp", value);
                updated++;
            }
            if (extract_json_float(yaw_rate_pos, "ki", &value)) {
                set_pid_param_with_nvs("pid_rate", "yaw_ki", value);
                updated++;
            }
            if (extract_json_float(yaw_rate_pos, "kd", &value)) {
                set_pid_param_with_nvs("pid_rate", "yaw_kd", value);
                updated++;
            }
        }
    }
    // PID parametreleri anında etkili (set_pid_param_with_nvs() içinde paramSetFloat() çağrılıyor)
    // NVS kaydetme işlemi handler sonunda yapılır - parametreler zaten runtime'da aktif olduğu için reboot gerekmez
    if (updated > 0) {
        save_all_pid_params_to_nvs();
        DEBUG_PRINTI("PID parameters updated: %d params changed, saved to NVS\n", updated);
    }
    char resp[128];
    snprintf(resp, sizeof(resp), "{\"success\":true,\"updated\":%d}", updated);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
static esp_err_t get_status_handler(httpd_req_t *req) {
    char resp[256];
    snprintf(resp, sizeof(resp),
        "{\"success\":true,\"status\":\"ok\",\"firmware\":\"ESP-Drone\",\"sensors\":{"
        "\"imu\":\"mpu9250\",\"flow\":\"pmw3901\",\"tof\":\"vl53l1x\"}}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
static int read_request_body(httpd_req_t *req, char *buf, size_t buf_size) {
    int total_len = req->content_len;
    if (total_len < 0 || total_len >= (int)buf_size) {
        return -1;
    }
    if (total_len == 0) {
        buf[0] = '\0';
        return 0;
    }
    int cur_len = 0;
    int received = 0;
    int retry_count = 0;
    const int max_retries = 10;
    while (cur_len < total_len && retry_count < max_retries) {
        received = httpd_req_recv(req, buf + cur_len, total_len - cur_len);
        if (received < 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                retry_count++;
                continue;
            }
            return -1;
        } else if (received == 0) {
            break;
        }
        cur_len += received;
        retry_count = 0;
    }
    if (cur_len < total_len) {
        return -1;
    }
    buf[cur_len] = '\0';
    return cur_len;
}
// Joystick filtering and deadband
#define JOYSTICK_DEADBAND 2.0f  // Deadband threshold (2% of full range)
#define JOYSTICK_LPF_ALPHA 0.3f  // Low-pass filter coefficient (0.0-1.0, lower = smoother)
static float lastRoll = 0.0f;
static float lastPitch = 0.0f;
static float lastYaw = 0.0f;
static float lastThrottle = 0.0f;

static float applyDeadband(float value, float deadband) {
    if (fabsf(value) < deadband) {
        return 0.0f;
    }
    // Scale to compensate for deadband
    if (value > 0) {
        return (value - deadband) / (100.0f - deadband) * 100.0f;
    } else {
        return (value + deadband) / (100.0f - deadband) * 100.0f;
    }
}

static float applyLowPassFilter(float newValue, float oldValue, float alpha) {
    return alpha * newValue + (1.0f - alpha) * oldValue;
}

static esp_err_t control_handler(httpd_req_t *req) {
    char content[256];
    int ret = read_request_body(req, content, sizeof(content));
    if (ret < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
        return ESP_FAIL;
    }
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f, throttle_f = 0.0f;
    int throttle = 0;
    if (ret > 0) {
        extract_json_float(content, "roll", &roll);
        extract_json_float(content, "pitch", &pitch);
        extract_json_float(content, "yaw", &yaw);
        extract_json_float(content, "throttle", &throttle_f);
        
        // Clamp values to valid range
        if (roll > 100.0f) roll = 100.0f;
        if (roll < -100.0f) roll = -100.0f;
        if (pitch > 100.0f) pitch = 100.0f;
        if (pitch < -100.0f) pitch = -100.0f;
        if (yaw > 100.0f) yaw = 100.0f;
        if (yaw < -100.0f) yaw = -100.0f;
        
        // Apply deadband and low-pass filter
        roll = applyDeadband(roll, JOYSTICK_DEADBAND);
        pitch = applyDeadband(pitch, JOYSTICK_DEADBAND);
        yaw = applyDeadband(yaw, JOYSTICK_DEADBAND);
        
        roll = applyLowPassFilter(roll, lastRoll, JOYSTICK_LPF_ALPHA);
        pitch = applyLowPassFilter(pitch, lastPitch, JOYSTICK_LPF_ALPHA);
        yaw = applyLowPassFilter(yaw, lastYaw, JOYSTICK_LPF_ALPHA);
        
        lastRoll = roll;
        lastPitch = pitch;
        lastYaw = yaw;
        
        // Throttle doesn't need deadband, but apply low-pass filter
        throttle_f = applyLowPassFilter(throttle_f, lastThrottle, JOYSTICK_LPF_ALPHA);
        lastThrottle = throttle_f;
        
        throttle = (int)roundf(throttle_f);
        if (throttle > 100) throttle = 100;
        if (throttle < 0) throttle = 0;
    }
    setpoint_t setpoint = {0};
    extern bool crtpCommanderRpytGetPosHoldMode(void);
    bool posHoldActive = crtpCommanderRpytGetPosHoldMode();
    if (posHoldActive) {
        setpoint.mode.x = modeVelocity;        
        setpoint.mode.y = modeVelocity;        
        setpoint.mode.z = modeVelocity;        
        setpoint.mode.roll = modeDisable;      
        setpoint.mode.pitch = modeDisable;     
        setpoint.mode.yaw = modeVelocity;      
        setpoint.velocity_body = true;         
        
        // EKLE: Joystick bırakıldığında (deadband içindeyse) velocity setpoint'lerini sıfırla
        // Bu, position hold modunda joystick bırakıldığında drone'un sabit kalmasını sağlar
        if (fabsf(roll) < JOYSTICK_DEADBAND && fabsf(pitch) < JOYSTICK_DEADBAND) {
            // Joystick bırakıldı - velocity'leri sıfırla (position hold aktif)
            setpoint.velocity.x = 0.0f;
            setpoint.velocity.y = 0.0f;
        } else {
            // Joystick aktif - velocity setpoint'lerini ayarla
            setpoint.velocity.x = pitch * 0.5f / 100.0f;  
            setpoint.velocity.y = -roll * 0.5f / 100.0f;   
        }
        
        float t = ((float)throttle - 50.0f) / 50.0f;
        if (t > 1.0f) t = 1.0f;
        if (t < -1.0f) t = -1.0f;
        setpoint.velocity.z = t * 0.3f;  
        setpoint.attitudeRate.yaw = -yaw * 200.0f / 100.0f;  
        setpoint.thrust = 0;
    } else {
        setpoint.mode.x = modeDisable;
        setpoint.mode.y = modeDisable;
        setpoint.mode.z = modeDisable;
        setpoint.mode.roll = modeAbs;
        setpoint.mode.pitch = modeAbs;
        setpoint.mode.yaw = modeVelocity;
        setpoint.attitude.roll = roll * 30.0f / 100.0f;  
        setpoint.attitude.pitch = pitch * 30.0f / 100.0f;
        setpoint.attitudeRate.yaw = -yaw * 200.0f / 100.0f;  
        if (throttle > 0) {
            setpoint.thrust = (uint16_t)((throttle / 100.0f) * 60000.0f);
            if (setpoint.thrust < 1000) setpoint.thrust = 1000;
            if (setpoint.thrust > 60000) setpoint.thrust = 60000;
        } else {
            setpoint.thrust = 0;
        }
    }
    commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
    // EKLE: Heartbeat mekanizması - Her joystick komutunda timestamp güncelle
    // Bu, joystick aktifken failsafe'in tetiklenmesini önler
    lastControlUs = esp_timer_get_time();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true}");
    return ESP_OK;
}
// Removed: takeoff_handler and land_handler - high level commander removed
// Takeoff and landing are now controlled via joystick throttle
static esp_err_t takeoff_handler(httpd_req_t *req) {
    // High level commander removed - use joystick throttle for takeoff/landing
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":false,\"error\":\"High level commander not available. Use joystick throttle for takeoff/landing.\"}");
    return ESP_OK;
}
static esp_err_t land_handler(httpd_req_t *req) {
    // High level commander removed - use joystick throttle for takeoff/landing
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":false,\"error\":\"High level commander not available. Use joystick throttle for takeoff/landing.\"}");
    return ESP_OK;
}
static esp_err_t emergency_handler(httpd_req_t *req) {
    stabilizerSetEmergencyStop();
    // Removed: crtpCommanderHighLevelStop() - high level commander removed
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true,\"message\":\"Emergency stop activated\"}");
    return ESP_OK;
}
static esp_err_t arm_handler(httpd_req_t *req) {
    systemSetArmed(true);
    bool armed = systemIsArmed();
    char resp[128];
    if (armed) {
        snprintf(resp, sizeof(resp), "{\"success\":true,\"armed\":true,\"message\":\"Drone armed - motors ready\"}");
    } else {
        snprintf(resp, sizeof(resp), "{\"success\":false,\"armed\":false,\"message\":\"Arming failed - safety checks not passed\"}");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
static esp_err_t disarm_handler(httpd_req_t *req) {
    systemSetArmed(false);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true,\"armed\":false,\"message\":\"Drone disarmed - motors disabled\"}");
    return ESP_OK;
}
static esp_err_t get_pid_get_handler(httpd_req_t *req) {
    return get_pid_params_handler(req);
}
static bool extract_json_float(const char* json, const char* key, float* value) {
    if (!json || !key || !value) {
        return false;
    }
    char search_key[64];
    snprintf(search_key, sizeof(search_key), "\"%s\"", key);
    char* key_pos = strstr(json, search_key);
    if (key_pos) {
        char* colon = strchr(key_pos, ':');
        if (colon) {
            char* end_ptr = NULL;
            float parsed = strtof(colon + 1, &end_ptr);
            if (end_ptr != colon + 1 && (end_ptr == NULL || *end_ptr == ',' || *end_ptr == '}' || *end_ptr == ' ' || *end_ptr == '\n' || *end_ptr == '\r')) {
                *value = parsed;
                return true;
            }
        }
    }
    return false;
}
static esp_err_t set_pid_set_handler(httpd_req_t *req) {
    return set_pid_params_handler(req);
}
static esp_err_t reset_pid_handler(httpd_req_t *req) {
    set_pid_param_with_nvs("pid_attitude", "roll_kp", 5.9f);
    set_pid_param_with_nvs("pid_attitude", "roll_ki", 2.9f);
    set_pid_param_with_nvs("pid_attitude", "roll_kd", 0.0f);
    set_pid_param_with_nvs("pid_attitude", "pitch_kp", 5.9f);
    set_pid_param_with_nvs("pid_attitude", "pitch_ki", 2.9f);
    set_pid_param_with_nvs("pid_attitude", "pitch_kd", 0.0f);
    set_pid_param_with_nvs("pid_attitude", "yaw_kp", 6.0f);
    set_pid_param_with_nvs("pid_attitude", "yaw_ki", 1.0f);
    set_pid_param_with_nvs("pid_attitude", "yaw_kd", 0.35f);
    set_pid_param_with_nvs("pid_rate", "roll_kp", 250.0f);
    set_pid_param_with_nvs("pid_rate", "roll_ki", 500.0f);
    set_pid_param_with_nvs("pid_rate", "roll_kd", 2.5f);
    set_pid_param_with_nvs("pid_rate", "pitch_kp", 250.0f);
    set_pid_param_with_nvs("pid_rate", "pitch_ki", 500.0f);
    set_pid_param_with_nvs("pid_rate", "pitch_kd", 2.5f);
    set_pid_param_with_nvs("pid_rate", "yaw_kp", 120.0f);
    set_pid_param_with_nvs("pid_rate", "yaw_ki", 16.7f);
    set_pid_param_with_nvs("pid_rate", "yaw_kd", 0.0f);
    save_all_pid_params_to_nvs();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true,\"message\":\"PID parameters reset to defaults and saved to flash\"}");
    return ESP_OK;
}
static esp_err_t calibration_status_handler(httpd_req_t *req) {
    bool calibrated = sensorsAreCalibrated();
    char resp[256];
    if (calibrated) {
        snprintf(resp, sizeof(resp), "{\"success\":true,\"calibrated\":true,\"message\":\"Sensors are calibrated\"}");
    } else {
        snprintf(resp, sizeof(resp), "{\"success\":true,\"calibrated\":false,\"message\":\"Sensors are not yet calibrated\"}");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
// Task function for ESC calibration
static void escCalibrationTask(void *arg) {
    extern void motorsCalibrateESC(void);
    motorsCalibrateESC();
    vTaskDelete(NULL);
}

static esp_err_t esc_calibration_handler(httpd_req_t *req) {
    // ESC calibration must be done when disarmed for safety
    if (systemIsArmed()) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"success\":false,\"message\":\"ESC calibration requires drone to be disarmed. Please disarm first.\"}");
        return ESP_OK;
    }
    
    // Start ESC calibration in a separate task to avoid blocking
    xTaskCreate(escCalibrationTask, "esc_calib", 4096, NULL, 5, NULL);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true,\"message\":\"ESC calibration started. This will take ~8 seconds. Remove propellers before calibration!\"}");
    return ESP_OK;
}
static esp_err_t telemetry_handler(httpd_req_t *req) {
    state_t state;
    bool stateValid = stabilizerGetState(&state);
    if (!stateValid) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"success\":false,\"error\":\"Stabilizer not initialized\"}");
        return ESP_OK;
    }
    float batteryVoltage = pmGetBatteryVoltage();
    int m1_ratio = motorsGetRatio(MOTOR_M1);
    int m2_ratio = motorsGetRatio(MOTOR_M2);
    int m3_ratio = motorsGetRatio(MOTOR_M3);
    int m4_ratio = motorsGetRatio(MOTOR_M4);
    float m1 = (m1_ratio >= 0) ? ((float)m1_ratio / 65536.0f) : 0.0f;
    float m2 = (m2_ratio >= 0) ? ((float)m2_ratio / 65536.0f) : 0.0f;
    float m3 = (m3_ratio >= 0) ? ((float)m3_ratio / 65536.0f) : 0.0f;
    float m4 = (m4_ratio >= 0) ? ((float)m4_ratio / 65536.0f) : 0.0f;
    bool armed = systemIsArmed();
    extern bool crtpCommanderRpytGetPosHoldMode(void);
    bool posHoldActive = crtpCommanderRpytGetPosHoldMode();
    bool calibrated = sensorsAreCalibrated();
    char resp[1024];
    snprintf(resp, sizeof(resp),
        "{\"success\":true,"
        "\"batt\":%.2f,"
        "\"armed\":%s,"
        "\"posHold\":%s,"
        "\"calibrated\":%s,"
        "\"attitude\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},"
        "\"position\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
        "\"velocity\":{\"vx\":%.3f,\"vy\":%.3f,\"vz\":%.3f},"
        "\"motors\":{\"m1\":%.3f,\"m2\":%.3f,\"m3\":%.3f,\"m4\":%.3f}}",
        batteryVoltage,
        armed ? "true" : "false",
        posHoldActive ? "true" : "false",
        calibrated ? "true" : "false",
        state.attitude.roll,
        state.attitude.pitch,
        state.attitude.yaw,
        state.position.x,
        state.position.y,
        state.position.z,
        state.velocity.x,
        state.velocity.y,
        state.velocity.z,
        m1, m2, m3, m4
    );
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
static const httpd_uri_t get_pid_params = {
    .uri       = "/api/pid",
    .method    = HTTP_GET,
    .handler   = get_pid_params_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t set_pid_params = {
    .uri       = "/api/pid",
    .method    = HTTP_POST,
    .handler   = set_pid_params_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t get_status = {
    .uri       = "/api/status",
    .method    = HTTP_GET,
    .handler   = get_status_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t control = {
    .uri       = "/api/control",
    .method    = HTTP_POST,
    .handler   = control_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t takeoff = {
    .uri       = "/api/takeoff",
    .method    = HTTP_POST,
    .handler   = takeoff_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t land = {
    .uri       = "/api/land",
    .method    = HTTP_POST,
    .handler   = land_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t emergency = {
    .uri       = "/api/emergency",
    .method    = HTTP_POST,
    .handler   = emergency_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t arm = {
    .uri       = "/api/arm",
    .method    = HTTP_POST,
    .handler   = arm_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t disarm = {
    .uri       = "/api/disarm",
    .method    = HTTP_POST,
    .handler   = disarm_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t get_pid_get = {
    .uri       = "/api/pid/get",
    .method    = HTTP_GET,
    .handler   = get_pid_get_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t set_pid_set = {
    .uri       = "/api/pid/set",
    .method    = HTTP_POST,
    .handler   = set_pid_set_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t reset_pid = {
    .uri       = "/api/pid/reset",
    .method    = HTTP_POST,
    .handler   = reset_pid_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t calibration_status = {
    .uri       = "/api/calibration/status",
    .method    = HTTP_GET,
    .handler   = calibration_status_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t esc_calibration = {
    .uri       = "/api/calibration/esc",
    .method    = HTTP_POST,
    .handler   = esc_calibration_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t telemetry = {
    .uri       = "/api/telemetry",
    .method    = HTTP_GET,
    .handler   = telemetry_handler,
    .user_ctx  = NULL
};
static esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err) {
    if (strncmp(req->uri, "/api/", 5) == 0) {
        httpd_resp_set_status(req, "404 Not Found");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"success\":false,\"error\":\"not_found\"}");
        return ESP_OK;
    }
    if (strcmp(req->uri, "/favicon.ico") == 0 ||
        strstr(req->uri, "apple-touch-icon") != NULL ||
        strstr(req->uri, "manifest") != NULL) {
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, "Redirecting...", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
// EKLE: Captive portal handler - Mobil cihazlarda otomatik captive portal algılaması için
// Farklı cihazlar farklı URI'ler kullanır (Android: /generate_204, iOS: /hotspot-detect.html, vb.)
static esp_err_t captive_handler(httpd_req_t *req) {
    // 302 redirect ile root sayfaya yönlendir
    // Bu, mobil cihazlarda captive portal algılamasını tetikler
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}
static const httpd_uri_t captive1 = {
    .uri       = "/generate_204",
    .method    = HTTP_GET,
    .handler   = captive_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t captive2 = {
    .uri       = "/gen_204",
    .method    = HTTP_GET,
    .handler   = captive_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t captive3 = {
    .uri       = "/hotspot-detect.html",
    .method    = HTTP_GET,
    .handler   = captive_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t captive4 = {
    .uri       = "/connecttest.txt",
    .method    = HTTP_GET,
    .handler   = captive_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t captive5 = {
    .uri       = "/ncsi.txt",
    .method    = HTTP_GET,
    .handler   = captive_handler,
    .user_ctx  = NULL
};
static esp_err_t root_handler(httpd_req_t *req) {
    extern const char index_html_start[] asm("_binary_esp_drone_control_html_start");
    extern const char index_html_end[] asm("_binary_esp_drone_control_html_end");
    const size_t index_html_size = (index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Expires", "0");
    httpd_resp_send(req, index_html_start, index_html_size);
    return ESP_OK;
}
static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_handler,
    .user_ctx  = NULL
};
bool webServerInit(void) {
    if (isInit) {
        return true;
    }
    // DÜZELTME K4: webServerLoadPidParamsFromNVS() kaldırıldı
    // PID parametreleri zaten boot'ta attitude_pid_controller.c içinde pidNvsLoad() ile yükleniyor
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 20;
    config.task_priority = 5;
    config.stack_size = 12288;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 5;
    config.max_open_sockets = 8; 
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &get_pid_params);
        httpd_register_uri_handler(server, &set_pid_params);
        httpd_register_uri_handler(server, &get_status);
        httpd_register_uri_handler(server, &control);
        httpd_register_uri_handler(server, &takeoff);
        httpd_register_uri_handler(server, &land);
        httpd_register_uri_handler(server, &emergency);
        httpd_register_uri_handler(server, &arm);
        httpd_register_uri_handler(server, &disarm);
        httpd_register_uri_handler(server, &get_pid_get);
        httpd_register_uri_handler(server, &set_pid_set);
        httpd_register_uri_handler(server, &reset_pid);
        httpd_register_uri_handler(server, &calibration_status);
        httpd_register_uri_handler(server, &esc_calibration);
        httpd_register_uri_handler(server, &telemetry);
        httpd_register_uri_handler(server, &captive1);
        httpd_register_uri_handler(server, &captive2);
        httpd_register_uri_handler(server, &captive3);
        httpd_register_uri_handler(server, &captive4);
        httpd_register_uri_handler(server, &captive5);
        httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);
        xTaskCreate(webFailsafeTask, "web_failsafe", 2048, NULL, 5, NULL);
        isInit = true;
        DEBUG_PRINTI("Web server started on port %d", config.server_port);
        return true;
    }
    DEBUG_PRINTE("Failed to start web server");
    return false;
}
static void webFailsafeTask(void *arg) {
    static bool failsafeActive = false;
    static int64_t failsafeStartTime = 0;
    static uint16_t failsafeThrust = 0;
    
    while (1) {
        int64_t now = esp_timer_get_time();
        int64_t lastControl = lastControlUs;
        
        if (lastControl != 0 && (now - lastControl) > WEB_CONTROL_TIMEOUT_US) {
            if (!failsafeActive) {
                // Failsafe tetiklendi - başlat
                failsafeActive = true;
                failsafeStartTime = now;
                
                // Position hold modunu kapat (failsafe'de position hold aktif kalmamalı)
                extern void setCommandermode(FlightMode mode);
                setCommandermode(STABILIZE_MODE);  // Position hold'u kapat, stabilize mode'a geç
                
                // Failsafe log - Debug için
                DEBUG_PRINTW("FAILSAFE: Web control timeout! Last control: %lld us ago (%.2f s)\n", 
                             now - lastControl, (double)(now - lastControl) / 1000000.0);
                DEBUG_PRINTW("FAILSAFE: Initiating soft landing...\n");
                
                // Başlangıç thrust değeri (güvenli minimum - idle thrust seviyesi)
                failsafeThrust = 1000;  // Minimum safe thrust (idle seviyesi)
            }
            
            // Soft landing: Gradual thrust reduction
            int64_t failsafeDuration = now - failsafeStartTime;
            if (failsafeDuration < 2000000) {  // İlk 2 saniye: gradual reduction
                // 2 saniyede 1000'den 0'a düşür
                float reduction = (float)failsafeDuration / 2000000.0f;  // 0.0 - 1.0
                failsafeThrust = (uint16_t)(1000.0f * (1.0f - reduction));
            } else {
                // 2 saniye sonra: thrust = 0
                failsafeThrust = 0;
            }
            
            // Stabilize mode'da level attitude ile soft landing
            setpoint_t sp = {0};
            sp.mode.x = modeDisable;
            sp.mode.y = modeDisable;
            sp.mode.z = modeDisable;
            sp.mode.roll = modeAbs;      // Level attitude
            sp.mode.pitch = modeAbs;     // Level attitude
            sp.mode.yaw = modeDisable;
            sp.attitude.roll = 0.0f;     // Level
            sp.attitude.pitch = 0.0f;    // Level
            sp.thrust = failsafeThrust;
            commanderSetSetpoint(&sp, COMMANDER_PRIORITY_CRTP);
        } else {
            // Control geri geldi - failsafe'i sıfırla
            if (failsafeActive) {
                failsafeActive = false;
                failsafeStartTime = 0;
                failsafeThrust = 0;
                DEBUG_PRINTI("FAILSAFE: Control restored, failsafe cleared\n");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
bool webServerTest(void) {
    return isInit;
}