
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "config.h"
#include "tutorial.h"
#include "mqtt_client_config.h"  // For mqtt_start and mqtt_publish
#include "driver/ledc.h"
#include "esp_system.h"
#include "driver/adc.h"  // For adc1_get_raw
#include "led_config.h"  // ✅ led_init() এখানে থেকে আসবে

#define TAG "APP_MAIN"

// Queue and tasks
static QueueHandle_t sensorQueue;
static TaskHandle_t TaskSensorHandle;
static TaskHandle_t TaskMQTTHandle;

typedef struct {
    int rawValue;
    float voltage;
} SensorData;

// --- Watchdog Task ---
static void watchdog_task(void *param) {
    TickType_t lastSensorTick = xTaskGetTickCount();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        if (xTaskGetTickCount() - lastSensorTick > pdMS_TO_TICKS(30000)) {
            ESP_LOGW(TAG, "Watchdog: No sensor update, restarting...");
            esp_restart();
        }
    }
}

// --- Sensor Task ---
static void sensor_task(void *param) {
    for (;;) {
        int val = adc1_get_raw(ADC1_CHANNEL_5);  // GPIO33
        float voltage = val * (3.3f / 4095.0f);

        SensorData data = {val, voltage};
        if (xQueueSend(sensorQueue, &data, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ESP_LOGI(TAG, "[Sensor] Sent: raw=%d, voltage=%.2f", val, voltage);
        } else {
            ESP_LOGW(TAG, "[Sensor] Queue full, skipping");
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

// --- MQTT Task ---
static void mqtt_task(void *param) {
    char msg1[16], msg2[16];
    for (;;) {
        SensorData data;
        if (xQueueReceive(sensorQueue, &data, pdMS_TO_TICKS(500))) {
            snprintf(msg1, sizeof(msg1), "%d", data.rawValue);
            snprintf(msg2, sizeof(msg2), "%.2f", data.voltage);

            mqtt_publish(MQTT_TOPIC_PUB1, msg1);
            mqtt_publish(MQTT_TOPIC_PUB2, msg2);

            ESP_LOGI(TAG, "[MQTT] Published: %s, %s", msg1, msg2);
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Wi-Fi...");
    ESP_ERROR_CHECK(tutorial_init());
    ESP_ERROR_CHECK(tutorial_connect(WIFI_SSID, WIFI_PASSWORD));
    ESP_LOGI(TAG, "Wi-Fi Connected ✅");

    ESP_ERROR_CHECK(mqtt_start());
    ESP_LOGI(TAG, "MQTT Client Started ✅");

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);

    led_init();   // ✅ led_config.c থেকে কল হচ্ছে

    // Queue
    sensorQueue = xQueueCreate(5, sizeof(SensorData));
    if (sensorQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue!");
        return;
    }

    // Tasks
    xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, NULL, 2, &TaskSensorHandle, 1);
    xTaskCreatePinnedToCore(mqtt_task, "MQTTTask", 4096, NULL, 3, &TaskMQTTHandle, 0);
    xTaskCreatePinnedToCore(watchdog_task, "WatchdogTask", 2048, NULL, 1, NULL, 1);
}
