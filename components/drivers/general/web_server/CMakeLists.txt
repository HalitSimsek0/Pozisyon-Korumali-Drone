idf_component_register(SRCS "web_server.c"
                      INCLUDE_DIRS "." "include"
                      EMBED_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../../../esp_drone_control.html"
                      REQUIRES esp_http_server esp_wifi nvs_flash json fatfs crazyflie)




