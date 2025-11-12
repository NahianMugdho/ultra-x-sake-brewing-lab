#include "mqtt_client_config.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "led_config.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "ESP_MQTT5";
static esp_mqtt_client_handle_t client;

// Utility: map function
static int map_value(int x, int in_min, int in_max, int out_min, int out_max) {
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ================== Publisher Task ==================
void publisher_task(void *pvParameters) {
    char msg[16];
    while (1) {
        snprintf(msg, sizeof(msg), "%d", 50);   // Fixed value = 50
        mqtt_publish(MQTT_TOPIC_PUB3, msg);     // Publish to ESP3
        ESP_LOGI(TAG, "📤 Published to %s: %s", MQTT_TOPIC_PUB3, msg);

        vTaskDelay(10000 / portTICK_PERIOD_MS); // Delay 10s
    }
}

// MQTT Event Handler
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
    client = event->client;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "✅ Connected to TLS broker");
            esp_mqtt_client_subscribe(client, MQTT_TOPIC_SUB, 1);
            esp_mqtt_client_subscribe(client, "CO2", 1);
            esp_mqtt_client_subscribe(client, "bowl", 1);
            esp_mqtt_client_subscribe(client, "sonar", 1);
            esp_mqtt_client_subscribe(client, "sugar", 1);
            ESP_LOGI(TAG, "📡 Subscribed to CO2, bowl, sonar, sugar topics");

            // Start publisher task once connected
            xTaskCreate(publisher_task, "publisher_task", 4096, NULL, 5, NULL);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "⚠️ Disconnected from broker");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "📌 Subscribed to topic, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_DATA: {
    char topic[64];
    char data[64];

    snprintf(topic, event->topic_len + 1, "%.*s", event->topic_len, event->topic);
    snprintf(data, event->data_len + 1, "%.*s", event->data_len, event->data);

    int value = atoi(data);
    ESP_LOGI(TAG, "📥 Data: %s -> %d", topic, value);

    // Existing behavior: LED control for your old topic
    if (strcmp(topic, MQTT_TOPIC_SUB) == 0) {
        int duty = map_value(value, 25, 40, 0, 255);
        ESP_LOGI(TAG, "💡 Setting LED duty=%d from value=%d", duty, value);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }

    // === New logic for CO2 / bowl / sonar / sugar ===
    else if (strcmp(topic, "CO2") == 0) {
        if (value > 30) mqtt_publish("CO2T", "AF");
        else mqtt_publish("CO2T", "CF");
    }
    else if (strcmp(topic, "bowl") == 0) {
        if (value > 30) mqtt_publish("bowlT", "FO");
        else mqtt_publish("bowlT", "FS");
    }
    else if (strcmp(topic, "sonar") == 0) {
        if (value > 30) mqtt_publish("sonarT", "PO");
        else mqtt_publish("sonarT", "PS");
    }
    else if (strcmp(topic, "sugar") == 0) {
        if (value < 30) mqtt_publish("sugarT", "FFC");
        else mqtt_publish("sugarT", "FFO");
    }
    break;
}


        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "❌ MQTT Event Error");
            break;

        default:
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
    mqtt_event_handler_cb(event_data);
}

// Start MQTT client
esp_err_t mqtt_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = MQTT_BROKER_URI,
            .verification.certificate = mqtt_ca_cert_pem,
        },
        .credentials = {
            .username = MQTT_BROKER_USER,
            .authentication.password = MQTT_BROKER_PASS,
            .client_id = MQTT_CLIENT_ID,
        },
        .session = {
            .keepalive = MQTT_KEEPALIVE,
            .disable_clean_session = false,
            .last_will = {
                .topic = MQTT_LWT_TOPIC,
                .msg = MQTT_LWT_MESSAGE,
                .qos = MQTT_LWT_QOS,
                .retain = MQTT_LWT_RETAIN,
            },
        },
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    return esp_mqtt_client_start(client);
}

// Function to publish message
void mqtt_publish(const char *topic, const char *msg) {
    if (client) {
        esp_mqtt_client_publish(client, topic, msg, 0, 1, 0);
    }
}
