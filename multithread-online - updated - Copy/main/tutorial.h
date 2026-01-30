#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"

// Initialize Wi-Fi
esp_err_t tutorial_init(void);

// Connect to Wi-Fi using SSID and password
esp_err_t tutorial_connect(char* wifi_ssid, char* wifi_password);

// Disconnect from Wi-Fi
esp_err_t tutorial_disconnect(void);

// Deinitialize Wi-Fi resources
esp_err_t tutorial_deinit(void);
