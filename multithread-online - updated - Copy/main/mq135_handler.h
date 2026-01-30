#pragma once

#include "esp_err.h"

/**
 * @brief Initialize MQ135 sensor
 * @return ESP_OK on success
 */
esp_err_t mq135_init(void);

/**
 * @brief Read alcohol value from MQ135
 * @param alcohol Pointer to store alcohol concentration (ppm)
 * @return ESP_OK on success
 */
esp_err_t mq135_read_alcohol(float *alcohol);