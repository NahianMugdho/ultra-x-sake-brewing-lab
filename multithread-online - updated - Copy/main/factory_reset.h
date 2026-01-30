#pragma once

#include "esp_err.h"

// Factory Reset Button Configuration
#define FACTORY_RESET_GPIO 0  // GPIO0 (BOOT button on most ESP32 boards)
#define FACTORY_RESET_HOLD_TIME_MS 5000  // 5 সেকেন্ড হোল্ড করতে হবে

/**
 * @brief Factory reset button ইনিশিয়ালাইজ করুন
 * 
 * GPIO0 (BOOT button) 5 সেকেন্ড ধরে রাখলে Wi-Fi credentials রিসেট হবে
 * 
 * @return ESP_OK on success
 */
esp_err_t factory_reset_init(void);