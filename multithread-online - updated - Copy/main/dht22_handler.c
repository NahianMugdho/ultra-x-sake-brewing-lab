#include "dht22_handler.h"
#include "config.h"
#include "dht.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DHT22";

esp_err_t dht22_init(void) {
    ESP_LOGI(TAG, "DHT22 initialized on GPIO%d", DHT_PIN);
    return ESP_OK;
}

esp_err_t dht22_read(dht22_data_t *data) {
    // DHT_TYPE_AM2301 is DHT22
    // Reading requires about 250ms, so we handle this in the task
    esp_err_t res = dht_read_float_data(DHT_TYPE_AM2301, 
                                        DHT_PIN, 
                                        &data->humidity, 
                                        &data->temperature);
    
    if (res != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read DHT22: %s", esp_err_to_name(res));
        return res;
    }
    
    // Validate readings
    if (data->humidity < 0 || data->humidity > 100 || 
        data->temperature < -40 || data->temperature > 80) {
        ESP_LOGW(TAG, "Invalid DHT22 readings");
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    return ESP_OK;
}