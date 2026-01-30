// // #include <stdio.h>
// // #include "freertos/FreeRTOS.h"
// // #include "freertos/task.h"
// // #include "freertos/queue.h"
// // #include "esp_log.h"
// // #include "mqtt_client_config.h"
// // #include "driver/ledc.h"
// // #include "esp_system.h"1
// // #include "driver/adc.h"
// // #include "led_config.h"
// // #include "wifi_manager.h"  // ✅ নতুন Wi-Fi Manager
// // #include "factory_reset.h"  // ✅ Factory Reset বাটন

// // #define TAG "APP_MAIN"

// // // Queue and tasks
// // static QueueHandle_t sensorQueue;
// // static TaskHandle_t TaskSensorHandle;
// // static TaskHandle_t TaskMQTTHandle;

// // typedef struct {
// //     int rawValue;
// //     float voltage;
// // } SensorData;

// // // --- Watchdog Task ---
// // static void watchdog_task(void *param) {
// //     TickType_t lastSensorTick = xTaskGetTickCount();

// //     for (;;) {
// //         vTaskDelay(pdMS_TO_TICKS(10000));
// //         if (xTaskGetTickCount() - lastSensorTick > pdMS_TO_TICKS(30000)) {
// //             ESP_LOGW(TAG, "Watchdog: No sensor update, restarting...");
// //             esp_restart();
// //         }
// //     }
// // }

// // // --- Sensor Task ---
// // static void sensor_task(void *param) {
// //     for (;;) {
// //         int val = adc1_get_raw(ADC1_CHANNEL_5);  // GPIO33
// //         float voltage = val * (3.3f / 4095.0f);

// //         SensorData data = {val, voltage};
// //         if (xQueueSend(sensorQueue, &data, pdMS_TO_TICKS(1000)) == pdTRUE) {
// //             ESP_LOGI(TAG, "[Sensor] Sent: raw=%d, voltage=%.2f", val, voltage);
// //         } else {
// //             ESP_LOGW(TAG, "[Sensor] Queue full, skipping");
// //         }
// //         vTaskDelay(pdMS_TO_TICKS(3000));
// //     }
// // }

// // // --- MQTT Task ---
// // static void mqtt_task(void *param) {
// //     char msg1[16], msg2[16];
// //     for (;;) {
// //         SensorData data;
// //         if (xQueueReceive(sensorQueue, &data, pdMS_TO_TICKS(500))) {
// //             snprintf(msg1, sizeof(msg1), "%d", data.rawValue);
// //             snprintf(msg2, sizeof(msg2), "%.2f", data.voltage);

// //             mqtt_publish(MQTT_TOPIC_PUB1, msg1);
// //             mqtt_publish(MQTT_TOPIC_PUB2, msg2);

// //             ESP_LOGI(TAG, "[MQTT] Published: %s, %s", msg1, msg2);
// //         }
// //     }
// // }

// // void app_main(void) {
// //     ESP_LOGI(TAG, "🚀 ESP32 Starting with Wi-Fi Manager...");
    
// //     // ============ Factory Reset Button Setup ============
// //     factory_reset_init();  // ✅ BOOT বাটন দিয়ে reset
    
// //     // ============ Wi-Fi Manager Setup ============
// //     ESP_ERROR_CHECK(wifi_manager_init());
    
// //     if (wifi_manager_is_configured()) {
// //         ESP_LOGI(TAG, "✅ Wi-Fi credentials found, connecting...");
        
// //         if (wifi_manager_connect() == ESP_OK) {
// //             ESP_LOGI(TAG, "✅ Connected to Wi-Fi");
// //         } else {
// //             ESP_LOGW(TAG, "⚠️ Failed to connect, starting portal...");
// //             wifi_manager_start_portal();
// //             ESP_LOGI(TAG, "📱 Connect to 'ESP32-Setup' and go to 192.168.4.1");
// //             // Portal mode এ থাকবে, main tasks start হবে না
// //             return;
// //         }
// //     } else {
// //         ESP_LOGI(TAG, "⚙️ No credentials, starting configuration portal...");
// //         wifi_manager_start_portal();
// //         ESP_LOGI(TAG, "📱 Connect to 'ESP32-Setup' and go to 192.168.4.1");
// //         return;
// //     }
    
// //     // ============ MQTT Setup ============
// //     ESP_ERROR_CHECK(mqtt_start());
// //     ESP_LOGI(TAG, "✅ MQTT Client Started");

// //     // ============ ADC Setup ============
// //     adc1_config_width(ADC_WIDTH_BIT_12);
// //     adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);

// //     // ============ LED Setup ============
// //     led_init();

// //     // ============ Queue Creation ============
// //     sensorQueue = xQueueCreate(5, sizeof(SensorData));
// //     if (sensorQueue == NULL) {
// //         ESP_LOGE(TAG, "❌ Failed to create queue!");
// //         return;
// //     }

// //     // ============ Task Creation ============
// //     xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, NULL, 2, &TaskSensorHandle, 1);
// //     xTaskCreatePinnedToCore(mqtt_task, "MQTTTask", 4096, NULL, 3, &TaskMQTTHandle, 0);
// //     xTaskCreatePinnedToCore(watchdog_task, "WatchdogTask", 2048, NULL, 1, NULL, 1);
    
// //     ESP_LOGI(TAG, "✅ All tasks started successfully!");
// // }

// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "esp_log.h"
// #include "mqtt_client_config.h"
// #include "driver/ledc.h"
// #include "esp_system.h"
// #include "wifi_manager.h"
// #include "factory_reset.h"
// #include "dht22_handler.h"
// #include "ds18b20_handler.h"
// #include "mq135_handler.h"
// #include "ultrasonic_handler.h"
// #include "motor_controller.h"
// #include "config.h"
// #define TAG "APP_MAIN"

// // Queue handles
// static QueueHandle_t dht22Queue;
// static QueueHandle_t ds18b20Queue;
// static QueueHandle_t mq135Queue;
// static QueueHandle_t sonar1Queue;
// static QueueHandle_t sonar2Queue;

// // Task handles
// static TaskHandle_t TaskDHT22Handle;
// static TaskHandle_t TaskDS18B20Handle;
// static TaskHandle_t TaskMQ135Handle;
// static TaskHandle_t TaskSonar1Handle;
// static TaskHandle_t TaskSonar2Handle;
// static TaskHandle_t TaskControlHandle;
// static TaskHandle_t TaskMQTTHandle;

// // Data structures
// typedef dht22_data_t DHT22Data;

// typedef struct {
//     float temperature;
// } DS18B20Data;

// typedef struct {
//     float alcohol;
// } MQ135Data;

// typedef struct {
//     float distance;
// } SonarData;

// // Global motor speed control
// static uint8_t target_fan_speed = 0;

// // ============ DHT22 Task ============
// static void dht22_task(void *param) {
//     DHT22Data data;
    
//     for (;;) {
//         if (dht22_read(&data) == ESP_OK) {
//             ESP_LOGI(TAG, "🌡️ DHT22 - Temp: %.1f°C, Humidity: %.1f%%", 
//                      data.temperature, data.humidity);
            
//             if (xQueueSend(dht22Queue, &data, 0) != pdTRUE) {
//                 ESP_LOGW(TAG, "DHT22 queue full");
//             }
//         } else {
//             ESP_LOGW(TAG, "Failed to read DHT22");
//         }
        
//         vTaskDelay(pdMS_TO_TICKS(3000)); // Read every 3 seconds
//     }
// }

// // ============ DS18B20 Task ============
// static void ds18b20_task(void *param) {
//     DS18B20Data data;
    
//     for (;;) {
//         if (ds18b20_read(&data.temperature) == ESP_OK) {
//             ESP_LOGI(TAG, "🔥 DS18B20 - Temp: %.2f°C", data.temperature);
            
//             if (xQueueSend(ds18b20Queue, &data, 0) != pdTRUE) {
//                 ESP_LOGW(TAG, "DS18B20 queue full");
//             }
//         } else {
//             ESP_LOGW(TAG, "Failed to read DS18B20");
//         }
        
//         vTaskDelay(pdMS_TO_TICKS(2000)); // Read every 2 seconds
//     }
// }

// // ============ MQ135 Task ============
// static void mq135_task(void *param) {
//     MQ135Data data;
    
//     for (;;) {
//         if (mq135_read_alcohol(&data.alcohol) == ESP_OK) {
//             ESP_LOGI(TAG, "🍷 MQ135 - Alcohol: %.2f ppm", data.alcohol);
            
//             if (xQueueSend(mq135Queue, &data, 0) != pdTRUE) {
//                 ESP_LOGW(TAG, "MQ135 queue full");
//             }
//         } else {
//             ESP_LOGW(TAG, "Failed to read MQ135");
//         }
        
//         vTaskDelay(pdMS_TO_TICKS(2000)); // Read every 2 seconds
//     }
// }

// // ============ Sonar1 Task ============
// static void sonar1_task(void *param) {
//     SonarData data;
    
//     for (;;) {
//         if (ultrasonic_read_sonar1(&data.distance) == ESP_OK) {
//             ESP_LOGI(TAG, "📏 Sonar1 - Distance: %.2f cm", data.distance);
            
//             if (xQueueSend(sonar1Queue, &data, 0) != pdTRUE) {
//                 ESP_LOGW(TAG, "Sonar1 queue full");
//             }
//         } else {
//             ESP_LOGW(TAG, "Failed to read Sonar1");
//         }
        
//         vTaskDelay(pdMS_TO_TICKS(500)); // Read every 500ms
//     }
// }

// // ============ Sonar2 Task ============
// static void sonar2_task(void *param) {
//     SonarData data;
    
//     for (;;) {
//         if (ultrasonic_read_sonar2(&data.distance) == ESP_OK) {
//             ESP_LOGI(TAG, "📏 Sonar2 - Distance: %.2f cm", data.distance);
            
//             if (xQueueSend(sonar2Queue, &data, 0) != pdTRUE) {
//                 ESP_LOGW(TAG, "Sonar2 queue full");
//             }
//         } else {
//             ESP_LOGW(TAG, "Failed to read Sonar2");
//         }
        
//         vTaskDelay(pdMS_TO_TICKS(500)); // Read every 500ms
//     }
// }

// // ============ Control Task (Motor Logic) ============
// static void control_task(void *param) {
//     DS18B20Data ds_data;
//     SonarData sonar_data;
//     bool pump_state = false;
    
//     for (;;) {
//         // Check DS18B20 temperature for fan control
//         if (xQueuePeek(ds18b20Queue, &ds_data, 0) == pdTRUE) {
//             if (ds_data.temperature > DS18B20_TEMP_THRESHOLD) {
//                 if (target_fan_speed == 0) {
//                     target_fan_speed = 128; // Default medium speed
//                 }
//                 motor_set_fan_speed(target_fan_speed);
//                 ESP_LOGI(TAG, "🌡️ Temp > 40°C, Fan ON at speed %d", target_fan_speed);
//             } else {
//                 motor_set_fan_speed(0);
//             }
//         }
        
//         // Check Sonar1 for water pump control
//         if (xQueuePeek(sonar1Queue, &sonar_data, 0) == pdTRUE) {
//             if (sonar_data.distance < SONAR1_DISTANCE_THRESHOLD && !pump_state) {
//                 motor_set_pump(true);
//                 pump_state = true;
//                 ESP_LOGI(TAG, "💧 Distance < 5cm, Pump ON");
//             } else if (sonar_data.distance >= SONAR1_DISTANCE_THRESHOLD && pump_state) {
//                 motor_set_pump(false);
//                 pump_state = false;
//                 ESP_LOGI(TAG, "💧 Distance >= 5cm, Pump OFF");
//             }
//         }
        
//         vTaskDelay(pdMS_TO_TICKS(200)); // Control loop at 200ms
//     }
// }

// // ============ MQTT Task ============
// static void mqtt_task(void *param) {
//     char msg[32];
//     DHT22Data dht_data;
//     DS18B20Data ds_data;
//     MQ135Data mq_data;
//     SonarData sonar2_data;
//      SonarData sonar1_data;  // ← Changed from sonar2_data
//     for (;;) {
//         // Publish DHT22 data
//         if (xQueuePeek(dht22Queue, &dht_data, 0) == pdTRUE) {
//             snprintf(msg, sizeof(msg), "%.1f", dht_data.temperature);
//             mqtt_publish(MQTT_TOPIC_PUB1, msg); // "ESP" topic
            
//             snprintf(msg, sizeof(msg), "%.1f", dht_data.humidity);
//             mqtt_publish(MQTT_TOPIC_PUB2, msg); // "ESP2" topic
            
//             ESP_LOGI(TAG, "📤 MQTT: DHT22 - Temp=%.1f°C, Humidity=%.1f%%", 
//                      dht_data.temperature, dht_data.humidity);
//         }
        
//         // Publish DS18B20 data to ESP3 topic
//         if (xQueuePeek(ds18b20Queue, &ds_data, 0) == pdTRUE) {
//             snprintf(msg, sizeof(msg), "%.2f", ds_data.temperature);
//             mqtt_publish(MQTT_TOPIC_PUB3, msg); // "ESP3" topic
//             ESP_LOGI(TAG, "📤 MQTT: DS18B20 - Temp=%.2f°C", ds_data.temperature);
//         }
        
//         // Publish MQ135 alcohol data to CO2 topic
//         if (xQueueReceive(mq135Queue, &mq_data, 0) == pdTRUE) {
//             snprintf(msg, sizeof(msg), "%.2f", mq_data.alcohol);
//             mqtt_publish("CO2", msg);
//             ESP_LOGI(TAG, "📤 MQTT: MQ135 - Alcohol=%.2f ppm", mq_data.alcohol);
//         }
        
//          // Publish Sonar1 data to "level" topic ← NEW
//         if (xQueuePeek(sonar1Queue, &sonar1_data, 0) == pdTRUE) {
//             snprintf(msg, sizeof(msg), "%.2f", sonar1_data.distance);
//             mqtt_publish("level", msg);
//             ESP_LOGI(TAG, "📤 MQTT: Sonar1 - Distance=%.2f cm (level)", sonar1_data.distance);
//         }
//         // Publish Sonar2 data to sugar topic
//         if (xQueueReceive(sonar2Queue, &sonar2_data, 0) == pdTRUE) {
//             snprintf(msg, sizeof(msg), "%.2f", sonar2_data.distance);
//             mqtt_publish("sugar", msg);
//             ESP_LOGI(TAG, "📤 MQTT: Sonar2 - Distance=%.2f cm", sonar2_data.distance);
//         }
        
//         vTaskDelay(pdMS_TO_TICKS(5000)); // Publish every 5 seconds
//     }
// }

// // ============ Watchdog Task ============
// static void watchdog_task(void *param) {
//     TickType_t lastCheck = xTaskGetTickCount();
    
//     for (;;) {
//         vTaskDelay(pdMS_TO_TICKS(30000)); // Check every 30 seconds
        
//         if (xTaskGetTickCount() - lastCheck > pdMS_TO_TICKS(120000)) {
//             ESP_LOGW(TAG, "⚠️ Watchdog timeout, restarting...");
//             esp_restart();
//         }
//         lastCheck = xTaskGetTickCount();
//     }
// }

// // ============ Main ============
// void app_main(void) {
//     ESP_LOGI(TAG, "🚀 ESP32 Multi-Sensor System Starting...");
    
//     // Factory Reset
//     factory_reset_init();
    
//     // Wi-Fi Manager
//     ESP_ERROR_CHECK(wifi_manager_init());
    
//     if (wifi_manager_is_configured()) {
//         ESP_LOGI(TAG, "✅ Wi-Fi credentials found");
//         if (wifi_manager_connect() == ESP_OK) {
//             ESP_LOGI(TAG, "✅ Connected to Wi-Fi");
//         } else {
//             ESP_LOGW(TAG, "⚠️ Connection failed, starting portal...");
//             wifi_manager_start_portal();
//             ESP_LOGI(TAG, "📱 Connect to 'ESP32-Setup' at 192.168.4.1");
//             return;
//         }
//     } else {
//         ESP_LOGI(TAG, "⚙️ No credentials, starting portal...");
//         wifi_manager_start_portal();
//         ESP_LOGI(TAG, "📱 Connect to 'ESP32-Setup' at 192.168.4.1");
//         return;
//     }
    
//     // MQTT
//     ESP_ERROR_CHECK(mqtt_start());
//     ESP_LOGI(TAG, "✅ MQTT Started");
    
//     // Initialize all hardware
//     ESP_ERROR_CHECK(dht22_init());
//     ESP_ERROR_CHECK(ds18b20_init());
//     ESP_ERROR_CHECK(mq135_init());
//     ESP_ERROR_CHECK(ultrasonic_init());
//     ESP_ERROR_CHECK(motor_controller_init());
    
//     // Create queues
//     dht22Queue = xQueueCreate(2, sizeof(DHT22Data));
//     ds18b20Queue = xQueueCreate(2, sizeof(DS18B20Data));
//     mq135Queue = xQueueCreate(5, sizeof(MQ135Data));
//     sonar1Queue = xQueueCreate(5, sizeof(SonarData));
//     sonar2Queue = xQueueCreate(5, sizeof(SonarData));
    
//     // Create tasks
//     xTaskCreatePinnedToCore(dht22_task, "DHT22Task", 4096, NULL, 2, &TaskDHT22Handle, 1);
//     xTaskCreatePinnedToCore(ds18b20_task, "DS18B20Task", 4096, NULL, 2, &TaskDS18B20Handle, 1);
//     xTaskCreatePinnedToCore(mq135_task, "MQ135Task", 4096, NULL, 2, &TaskMQ135Handle, 0);
//     xTaskCreatePinnedToCore(sonar1_task, "Sonar1Task", 3072, NULL, 2, &TaskSonar1Handle, 0);
//     xTaskCreatePinnedToCore(sonar2_task, "Sonar2Task", 3072, NULL, 2, &TaskSonar2Handle, 1);
//     xTaskCreatePinnedToCore(control_task, "ControlTask", 4096, NULL, 3, &TaskControlHandle, 0);
//     xTaskCreatePinnedToCore(mqtt_task, "MQTTTask", 4096, NULL, 3, &TaskMQTTHandle, 0);
//     xTaskCreatePinnedToCore(watchdog_task, "WatchdogTask", 2048, NULL, 1, NULL, 1);
    
//     ESP_LOGI(TAG, "✅ All tasks started successfully!");
// }

// // MQTT callback to control fan speed via "text" topic
// void mqtt_set_fan_speed_from_text(uint8_t speed) {
//     target_fan_speed = speed;
//     ESP_LOGI(TAG, "🎛️ Fan speed updated from MQTT: %d", speed);
// }

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "mqtt_client_config.h"
#include "driver/ledc.h"
#include "esp_system.h"
#include "wifi_manager.h"
#include "factory_reset.h"
#include "dht22_handler.h"
#include "ds18b20_handler.h"
#include "mq135_handler.h"
#include "ultrasonic_handler.h"
#include "motor_controller.h"
#include "config.h"

#define TAG "APP_MAIN"

// Queue handles (size = 1, will use xQueueOverwrite)
static QueueHandle_t dht22Queue;
static QueueHandle_t ds18b20Queue;
static QueueHandle_t mq135Queue;
static QueueHandle_t sonar1Queue;
static QueueHandle_t sonar2Queue;

// Task handles
static TaskHandle_t TaskDHT22Handle;
static TaskHandle_t TaskDS18B20Handle;
static TaskHandle_t TaskMQ135Handle;
static TaskHandle_t TaskSonar1Handle;
static TaskHandle_t TaskSonar2Handle;
static TaskHandle_t TaskControlHandle;
static TaskHandle_t TaskMQTTHandle;

// Data structures
typedef dht22_data_t DHT22Data;
typedef struct { float temperature; } DS18B20Data;
typedef struct { float alcohol; } MQ135Data;
typedef struct { float distance; } SonarData;

// Global motor speed control
static uint8_t target_fan_speed = 0;

// ============ DHT22 Task ============
static void dht22_task(void *param) {
    DHT22Data data;
    int consecutive_errors = 0;
    
    for (;;) {
        esp_err_t result = dht22_read(&data);
        
        if (result == ESP_OK) {
            consecutive_errors = 0;
            ESP_LOGI(TAG, "🌡️ DHT22 - Temp: %.1f°C, Humidity: %.1f%%", 
                     data.temperature, data.humidity);
            xQueueOverwrite(dht22Queue, &data);
        } else {
            consecutive_errors++;
            if (consecutive_errors % 5 == 1) { // Log every 5th error
                ESP_LOGW(TAG, "Failed to read DHT22 (error count: %d)", consecutive_errors);
            }
            if (consecutive_errors > 20) {
                ESP_LOGE(TAG, "DHT22 failed 20+ times, restarting...");
                esp_restart();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

// ============ DS18B20 Task ============
static void ds18b20_task(void *param) {
    DS18B20Data data;
    
    for (;;) {
        if (ds18b20_read(&data.temperature) == ESP_OK) {
            ESP_LOGI(TAG, "🔥 DS18B20 - Temp: %.2f°C", data.temperature);
            xQueueOverwrite(ds18b20Queue, &data);
        } else {
            ESP_LOGW(TAG, "Failed to read DS18B20");
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ============ MQ135 Task ============
static void mq135_task(void *param) {
    MQ135Data data;
    
    for (;;) {
        if (mq135_read_alcohol(&data.alcohol) == ESP_OK) {
            ESP_LOGI(TAG, "🍷 MQ135 - Alcohol: %.2f ppm", data.alcohol);
            xQueueOverwrite(mq135Queue, &data);
        } else {
            ESP_LOGW(TAG, "Failed to read MQ135");
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ============ Sonar1 Task ============
static void sonar1_task(void *param) {
    SonarData data;
    
    for (;;) {
        if (ultrasonic_read_sonar1(&data.distance) == ESP_OK) {
            ESP_LOGI(TAG, "📏 Sonar1 - Distance: %.2f cm", data.distance);
            xQueueOverwrite(sonar1Queue, &data);
        } else {
            ESP_LOGW(TAG, "Failed to read Sonar1");
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ============ Sonar2 Task ============
static void sonar2_task(void *param) {
    SonarData data;
    
    for (;;) {
        if (ultrasonic_read_sonar2(&data.distance) == ESP_OK) {
            ESP_LOGI(TAG, "📏 Sonar2 - Distance: %.2f cm", data.distance);
            xQueueOverwrite(sonar2Queue, &data);
        } else {
            ESP_LOGW(TAG, "Failed to read Sonar2");
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ============ Control Task (Motor Logic) ============
static void control_task(void *param) {
    DS18B20Data ds_data;
    SonarData sonar_data;
    bool pump_state = false;
    bool fan_state = false;
    uint8_t current_fan_speed = 0;
    
    for (;;) {
        // Check DS18B20 temperature for fan control
        if (xQueuePeek(ds18b20Queue, &ds_data, 0) == pdTRUE) {
            if (ds_data.temperature > DS18B20_TEMP_THRESHOLD) {
                if (target_fan_speed == 0) {
                    target_fan_speed = 128;
                }
                
                if (!fan_state || current_fan_speed != target_fan_speed) {
                    motor_set_fan_speed(target_fan_speed);
                    fan_state = true;
                    current_fan_speed = target_fan_speed;
                    ESP_LOGI(TAG, "🌡️ Temp > %.1f°C, Fan ON at speed %d", 
                             DS18B20_TEMP_THRESHOLD, target_fan_speed);
                }
            } else {
                if (fan_state) {
                    motor_set_fan_speed(0);
                    fan_state = false;
                    current_fan_speed = 0;
                    ESP_LOGI(TAG, "🌡️ Temp < %.1f°C, Fan OFF", DS18B20_TEMP_THRESHOLD);
                }
            }
        }
        
        // Check Sonar1 for water pump control
        if (xQueuePeek(sonar1Queue, &sonar_data, 0) == pdTRUE) {
            if (sonar_data.distance < SONAR1_DISTANCE_THRESHOLD && !pump_state) {
                motor_set_pump(true);
                pump_state = true;
                ESP_LOGI(TAG, "💧 Distance < %.1fcm, Pump ON", SONAR1_DISTANCE_THRESHOLD);
            } else if (sonar_data.distance >= SONAR1_DISTANCE_THRESHOLD && pump_state) {
                motor_set_pump(false);
                pump_state = false;
                ESP_LOGI(TAG, "💧 Distance >= %.1fcm, Pump OFF", SONAR1_DISTANCE_THRESHOLD);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ============ MQTT Task ============
static void mqtt_task(void *param) {
    char msg[32];
    DHT22Data dht_data;
    DS18B20Data ds_data;
    MQ135Data mq_data;
    SonarData sonar2_data;
    SonarData sonar1_data;
    
    for (;;) {
        if (xQueuePeek(dht22Queue, &dht_data, 0) == pdTRUE) {
            snprintf(msg, sizeof(msg), "%.1f", dht_data.temperature);
            mqtt_publish(MQTT_TOPIC_PUB1, msg);
            
            snprintf(msg, sizeof(msg), "%.1f", dht_data.humidity);
            mqtt_publish(MQTT_TOPIC_PUB2, msg);
            
            ESP_LOGI(TAG, "📤 MQTT: DHT22 - Temp=%.1f°C, Humidity=%.1f%%", 
                     dht_data.temperature, dht_data.humidity);
        }
        
        if (xQueuePeek(ds18b20Queue, &ds_data, 0) == pdTRUE) {
            snprintf(msg, sizeof(msg), "%.2f", ds_data.temperature);
            mqtt_publish("ds18", msg);
            ESP_LOGI(TAG, "📤 MQTT: DS18B20 - Temp=%.2f°C", ds_data.temperature);
        }
        
        if (xQueuePeek(mq135Queue, &mq_data, 0) == pdTRUE) {
            snprintf(msg, sizeof(msg), "%.2f", mq_data.alcohol);
            mqtt_publish("CO2", msg);
            ESP_LOGI(TAG, "📤 MQTT: MQ135 - Alcohol=%.2f ppm", mq_data.alcohol);
        }
        
        if (xQueuePeek(sonar1Queue, &sonar1_data, 0) == pdTRUE) {
            snprintf(msg, sizeof(msg), "%.2f", sonar1_data.distance);
            mqtt_publish("level", msg);
            ESP_LOGI(TAG, "📤 MQTT: Sonar1 - Distance=%.2f cm (level)", sonar1_data.distance);
        }
        
        if (xQueuePeek(sonar2Queue, &sonar2_data, 0) == pdTRUE) {
            snprintf(msg, sizeof(msg), "%.2f", sonar2_data.distance);
            mqtt_publish("sugar", msg);
            ESP_LOGI(TAG, "📤 MQTT: Sonar2 - Distance=%.2f cm", sonar2_data.distance);
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ============ Watchdog Task ============
static void watchdog_task(void *param) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        
        // Check if critical tasks are still running
        if (eTaskGetState(TaskDHT22Handle) == eDeleted ||
            eTaskGetState(TaskMQTTHandle) == eDeleted) {
            ESP_LOGW(TAG, "⚠️ Critical task died, restarting...");
            esp_restart();
        }
    }
}

// ============ Main ============
void app_main(void) {
    ESP_LOGI(TAG, "🚀 ESP32 Multi-Sensor System Starting...");
    
    factory_reset_init();
    ESP_ERROR_CHECK(wifi_manager_init());
    
    if (wifi_manager_is_configured()) {
        ESP_LOGI(TAG, "✅ Wi-Fi credentials found");
        if (wifi_manager_connect() == ESP_OK) {
            ESP_LOGI(TAG, "✅ Connected to Wi-Fi");
        } else {
            ESP_LOGW(TAG, "⚠️ Connection failed, starting portal...");
            wifi_manager_start_portal();
            ESP_LOGI(TAG, "📱 Connect to 'ESP32-Setup' at 192.168.4.1");
            return;
        }
    } else {
        ESP_LOGI(TAG, "⚙️ No credentials, starting portal...");
        wifi_manager_start_portal();
        ESP_LOGI(TAG, "📱 Connect to 'ESP32-Setup' at 192.168.4.1");
        return;
    }
    
    ESP_ERROR_CHECK(mqtt_start());
    ESP_LOGI(TAG, "✅ MQTT Started");
    
    // Initialize hardware
    ESP_ERROR_CHECK(dht22_init());
    ESP_ERROR_CHECK(ds18b20_init());
    ESP_ERROR_CHECK(mq135_init());
    ESP_ERROR_CHECK(ultrasonic_init());
    ESP_ERROR_CHECK(motor_controller_init());
    
    // Create queues with size 1 (latest value only)
    dht22Queue = xQueueCreate(1, sizeof(DHT22Data));
    ds18b20Queue = xQueueCreate(1, sizeof(DS18B20Data));
    mq135Queue = xQueueCreate(1, sizeof(MQ135Data));
    sonar1Queue = xQueueCreate(1, sizeof(SonarData));
    sonar2Queue = xQueueCreate(1, sizeof(SonarData));
    
    // Create tasks
    xTaskCreatePinnedToCore(dht22_task, "DHT22Task", 4096, NULL, 2, &TaskDHT22Handle, 1);
    xTaskCreatePinnedToCore(ds18b20_task, "DS18B20Task", 4096, NULL, 2, &TaskDS18B20Handle, 1);
    xTaskCreatePinnedToCore(mq135_task, "MQ135Task", 4096, NULL, 2, &TaskMQ135Handle, 0);
    xTaskCreatePinnedToCore(sonar1_task, "Sonar1Task", 3072, NULL, 2, &TaskSonar1Handle, 0);
    xTaskCreatePinnedToCore(sonar2_task, "Sonar2Task", 3072, NULL, 2, &TaskSonar2Handle, 1);
    xTaskCreatePinnedToCore(control_task, "ControlTask", 4096, NULL, 3, &TaskControlHandle, 0);
    xTaskCreatePinnedToCore(mqtt_task, "MQTTTask", 4096, NULL, 3, &TaskMQTTHandle, 0);
    xTaskCreatePinnedToCore(watchdog_task, "WatchdogTask", 2048, NULL, 1, NULL, 1);
    
    ESP_LOGI(TAG, "✅ All tasks started successfully!");
}

// MQTT callback to control fan speed
void mqtt_set_fan_speed_from_text(uint8_t speed) {
    target_fan_speed = speed;
    ESP_LOGI(TAG, "🎛️ Fan speed updated from MQTT: %d", speed);
}