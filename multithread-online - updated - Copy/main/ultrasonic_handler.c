#include "ultrasonic_handler.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"  // For esp_rom_delay_us

static const char *TAG = "ULTRASONIC";

#define TIMEOUT_US 30000 // 30ms timeout

static float read_distance(gpio_num_t trig_pin, gpio_num_t echo_pin) {
    // Send trigger pulse
    gpio_set_level(trig_pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(trig_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trig_pin, 0);
    
    // Wait for echo start
    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 0) {
        if ((esp_timer_get_time() - start_time) > TIMEOUT_US) {
            return -1;
        }
    }
    
    // Measure pulse width
    int64_t pulse_start = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 1) {
        if ((esp_timer_get_time() - pulse_start) > TIMEOUT_US) {
            return -1;
        }
    }
    int64_t pulse_end = esp_timer_get_time();
    
    // Calculate distance (speed of sound = 343 m/s)
    int64_t pulse_duration = pulse_end - pulse_start;
    float distance = (pulse_duration * 0.0343) / 2.0;
    
    return distance;
}

esp_err_t ultrasonic_init(void) {
    // Configure trigger pins
    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << SONAR1_TRIG_PIN) | (1ULL << SONAR2_TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&trig_conf);
    
    // Configure echo pins
    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << SONAR1_ECHO_PIN) | (1ULL << SONAR2_ECHO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&echo_conf);
    
    gpio_set_level(SONAR1_TRIG_PIN, 0);
    gpio_set_level(SONAR2_TRIG_PIN, 0);
    
    ESP_LOGI(TAG, "Ultrasonic sensors initialized");
    return ESP_OK;
}

esp_err_t ultrasonic_read_sonar1(float *distance) {
    *distance = read_distance(SONAR1_TRIG_PIN, SONAR1_ECHO_PIN);
    return (*distance >= 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t ultrasonic_read_sonar2(float *distance) {
    *distance = read_distance(SONAR2_TRIG_PIN, SONAR2_ECHO_PIN);
    return (*distance >= 0) ? ESP_OK : ESP_FAIL;
}