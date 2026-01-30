#pragma once

#include "esp_err.h"
#include <stdbool.h>  // ← ADD THIS
/**
 * @brief Initialize motor controllers (L298N)
 * @return ESP_OK on success
 */
esp_err_t motor_controller_init(void);

/**
 * @brief Set water pump state
 * @param enable true to turn on, false to turn off
 * @return ESP_OK on success
 */
esp_err_t motor_set_pump(bool enable);

/**
 * @brief Set fan motor speed (0-255)
 * @param speed Speed value (0 = off, 255 = max speed)
 * @return ESP_OK on success
 */
esp_err_t motor_set_fan_speed(uint8_t speed);