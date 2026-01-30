#pragma once

#include "esp_err.h"

/**
 * @brief Initialize ultrasonic sensors
 * @return ESP_OK on success
 */
esp_err_t ultrasonic_init(void);

/**
 * @brief Read distance from Sonar1
 * @param distance Pointer to store distance in cm
 * @return ESP_OK on success
 */
esp_err_t ultrasonic_read_sonar1(float *distance);

/**
 * @brief Read distance from Sonar2
 * @param distance Pointer to store distance in cm
 * @return ESP_OK on success
 */
esp_err_t ultrasonic_read_sonar2(float *distance);