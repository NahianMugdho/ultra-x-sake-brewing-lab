#pragma once

#include "esp_err.h"

/**
 * @brief Initialize DS18B20 sensor
 * @return ESP_OK on success
 */
esp_err_t ds18b20_init(void);

/**
 * @brief Read temperature from DS18B20
 * @param temperature Pointer to store temperature value
 * @return ESP_OK on success
 */
esp_err_t ds18b20_read(float *temperature);