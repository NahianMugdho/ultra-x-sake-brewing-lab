#pragma once

#include "esp_err.h"
#include "esp_wifi.h"

// Wi-Fi Manager Configuration
#define WIFI_MANAGER_AP_SSID "ESP32-Setup"
#define WIFI_MANAGER_AP_PASSWORD ""  // খালি = Open network
#define WIFI_MANAGER_AP_CHANNEL 1
#define WIFI_MANAGER_AP_MAX_CONN 4

// Storage keys for NVS
#define WIFI_NAMESPACE "wifi_config"
#define WIFI_SSID_KEY "ssid"
#define WIFI_PASS_KEY "password"
#define WIFI_CONFIGURED_KEY "configured"

// Timeouts
#define WIFI_MANAGER_PORTAL_TIMEOUT_MS 300000  // 5 মিনিট
#define WIFI_CONNECT_TIMEOUT_MS 30000          // 30 সেকেন্ড

/**
 * @brief Wi-Fi Manager ইনিশিয়ালাইজ করুন
 * 
 * এটি NVS চেক করবে stored credentials আছে কিনা।
 * যদি না থাকে, AP mode এ যাবে এবং captive portal চালু করবে।
 * 
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_init(void);

/**
 * @brief Wi-Fi এ কানেক্ট করার চেষ্টা করুন stored credentials দিয়ে
 * 
 * @return ESP_OK if connected, ESP_FAIL otherwise
 */
esp_err_t wifi_manager_connect(void);

/**
 * @brief Captive Portal চালু করুন AP mode এ
 * 
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_start_portal(void);

/**
 * @brief Captive Portal বন্ধ করুন
 * 
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_stop_portal(void);

/**
 * @brief নতুন Wi-Fi credentials সেভ করুন
 * 
 * @param ssid Wi-Fi SSID
 * @param password Wi-Fi Password
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_save_credentials(const char *ssid, const char *password);

/**
 * @brief Saved credentials মুছে ফেলুন (factory reset)
 * 
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_clear_credentials(void);

/**
 * @brief Check করুন credentials saved আছে কিনা
 * 
 * @return true if configured, false otherwise
 */
bool wifi_manager_is_configured(void);