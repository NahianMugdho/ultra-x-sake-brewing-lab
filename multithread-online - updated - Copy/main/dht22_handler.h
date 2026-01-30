#pragma once

#include "esp_err.h"

typedef struct {
    float temperature;
    float humidity;
} dht22_data_t;

/**
 * @brief Initialize DHT22 sensor
 * @return ESP_OK on success
 */
esp_err_t dht22_init(void);

/**
 * @brief Read temperature and humidity from DHT22
 * @param data Pointer to store sensor data
 * @return ESP_OK on success
 */
esp_err_t dht22_read(dht22_data_t *data);