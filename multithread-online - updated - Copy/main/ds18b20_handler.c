#include "ds18b20_handler.h"
#include "config.h"
#include "ds18x20.h"
#include "esp_log.h"

static const char *TAG = "DS18B20";
static ds18x20_addr_t ds18b20_addr;
static bool sensor_found = false;

esp_err_t ds18b20_init(void) {
    // Search for DS18B20 devices on the OneWire bus
    size_t sensor_count = 0;
    ds18x20_addr_t addrs[1]; // We expect only 1 sensor
    
    esp_err_t res = ds18x20_scan_devices(DS18B20_PIN, addrs, 1, &sensor_count);
    
    if (res != ESP_OK || sensor_count == 0) {
        ESP_LOGE(TAG, "No DS18B20 found on GPIO%d", DS18B20_PIN);
        return ESP_FAIL;
    }
    
    ds18b20_addr = addrs[0];
    sensor_found = true;
    
    ESP_LOGI(TAG, "DS18B20 found on GPIO%d (addr: %08X%08X)", 
             DS18B20_PIN, 
             (uint32_t)(ds18b20_addr >> 32), 
             (uint32_t)ds18b20_addr);
    
    return ESP_OK;
}

esp_err_t ds18b20_read(float *temperature) {
    if (!sensor_found) {
        ESP_LOGE(TAG, "DS18B20 not initialized");
        return ESP_FAIL;
    }
    
    // Measure and read temperature (takes ~750ms for 12-bit resolution)
    esp_err_t res = ds18x20_measure_and_read(DS18B20_PIN, 
                                             ds18b20_addr, 
                                             temperature);
    
    if (res != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read DS18B20: %s", esp_err_to_name(res));
        return res;
    }
    
    // Validate temperature reading (-55°C to +125°C is DS18B20 range)
    if (*temperature < -55.0 || *temperature > 125.0) {
        ESP_LOGW(TAG, "Invalid DS18B20 reading: %.2f°C", *temperature);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    return ESP_OK;
}