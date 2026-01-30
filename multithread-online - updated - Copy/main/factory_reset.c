#include "factory_reset.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "wifi_manager.h"

static const char *TAG = "FACTORY_RESET";

// Factory reset task
static void factory_reset_task(void *param) {
    uint32_t press_time = 0;
    bool button_pressed = false;
    
    for (;;) {
        if (gpio_get_level(FACTORY_RESET_GPIO) == 0) {  // বাটন প্রেস করা
            if (!button_pressed) {
                button_pressed = true;
                press_time = 0;
                ESP_LOGI(TAG, "Button pressed, hold for 5 seconds to reset...");
            }
            
            press_time += 100;
            
            if (press_time >= FACTORY_RESET_HOLD_TIME_MS) {
                ESP_LOGW(TAG, "🔄 Factory Reset Triggered!");
                ESP_LOGW(TAG, "Clearing Wi-Fi credentials...");
                
                wifi_manager_clear_credentials();
                
                vTaskDelay(pdMS_TO_TICKS(1000));
                ESP_LOGI(TAG, "Restarting...");
                esp_restart();
            }
        } else {
            if (button_pressed && press_time < FACTORY_RESET_HOLD_TIME_MS) {
                ESP_LOGI(TAG, "Button released (held for %lu ms)", press_time);
            }
            button_pressed = false;
            press_time = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Initialize factory reset button
esp_err_t factory_reset_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FACTORY_RESET_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    gpio_config(&io_conf);
    
    xTaskCreate(factory_reset_task, "FactoryResetTask", 2048, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Factory reset button initialized on GPIO%d", FACTORY_RESET_GPIO);
    return ESP_OK;
}