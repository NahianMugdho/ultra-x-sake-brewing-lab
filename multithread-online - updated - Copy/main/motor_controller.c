#include "motor_controller.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "MOTOR_CTRL";

#define PUMP_PWM_CHANNEL LEDC_CHANNEL_0
#define MOTOR_PWM_CHANNEL LEDC_CHANNEL_1
#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_FREQUENCY 5000

esp_err_t motor_controller_init(void) {
    // Configure GPIO pins for direction control
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PUMP_IN3_PIN) | (1ULL << PUMP_IN4_PIN) |
                       (1ULL << MOTOR_IN1_PIN) | (1ULL << MOTOR_IN2_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Initialize all to LOW
    gpio_set_level(PUMP_IN3_PIN, 0);
    gpio_set_level(PUMP_IN4_PIN, 0);
    gpio_set_level(MOTOR_IN1_PIN, 0);
    gpio_set_level(MOTOR_IN2_PIN, 0);
    
    // Configure PWM timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);
    
    // Configure PWM for pump enable
    ledc_channel_config_t pump_pwm = {
        .gpio_num = PUMP_ENABLE_PIN,
        .speed_mode = PWM_MODE,
        .channel = PUMP_PWM_CHANNEL,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&pump_pwm);
    
    // Configure PWM for motor enable
    ledc_channel_config_t motor_pwm = {
        .gpio_num = MOTOR_ENABLE_PIN,
        .speed_mode = PWM_MODE,
        .channel = MOTOR_PWM_CHANNEL,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&motor_pwm);
    
    ESP_LOGI(TAG, "Motor controller initialized");
    return ESP_OK;
}

esp_err_t motor_set_pump(bool enable) {
    if (enable) {
        gpio_set_level(PUMP_IN3_PIN, 1);
        gpio_set_level(PUMP_IN4_PIN, 0);
        ledc_set_duty(PWM_MODE, PUMP_PWM_CHANNEL, 255);
        ledc_update_duty(PWM_MODE, PUMP_PWM_CHANNEL);
        ESP_LOGI(TAG, "💧 Water pump ON");
    } else {
        gpio_set_level(PUMP_IN3_PIN, 0);
        gpio_set_level(PUMP_IN4_PIN, 0);
        ledc_set_duty(PWM_MODE, PUMP_PWM_CHANNEL, 0);
        ledc_update_duty(PWM_MODE, PUMP_PWM_CHANNEL);
        ESP_LOGI(TAG, "💧 Water pump OFF");
    }
    return ESP_OK;
}

esp_err_t motor_set_fan_speed(uint8_t speed) {
    if (speed > 0) {
        gpio_set_level(MOTOR_IN1_PIN, 1);
        gpio_set_level(MOTOR_IN2_PIN, 0);
        ledc_set_duty(PWM_MODE, MOTOR_PWM_CHANNEL, speed);
        ledc_update_duty(PWM_MODE, MOTOR_PWM_CHANNEL);
        ESP_LOGI(TAG, "🌀 Fan motor speed: %d", speed);
    } else {
        gpio_set_level(MOTOR_IN1_PIN, 0);
        gpio_set_level(MOTOR_IN2_PIN, 0);
        ledc_set_duty(PWM_MODE, MOTOR_PWM_CHANNEL, 0);
        ledc_update_duty(PWM_MODE, MOTOR_PWM_CHANNEL);
        ESP_LOGI(TAG, "🌀 Fan motor OFF");
    }
    return ESP_OK;
}