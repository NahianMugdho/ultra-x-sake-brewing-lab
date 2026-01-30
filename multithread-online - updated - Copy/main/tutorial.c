#include "tutorial.h"
#include <string.h>
#include "freertos/event_groups.h"

#define TAG "tutorial"
#define WIFI_AUTHMODE WIFI_AUTH_WPA2_PSK
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const int WIFI_RETRY_ATTEMPT = 3;
static int wifi_retry_count = 0;

static esp_netif_t *tutorial_netif = NULL;
static esp_event_handler_instance_t ip_event_handler;
static esp_event_handler_instance_t wifi_event_handler;
static EventGroupHandle_t s_wifi_event_group = NULL;

// IP Event Callback
static void ip_event_cb(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event_ip = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event_ip->ip_info.ip));
        wifi_retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Wi-Fi Event Callback
static void wifi_event_cb(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            if (wifi_retry_count < WIFI_RETRY_ATTEMPT) {
                esp_wifi_connect();
                wifi_retry_count++;
                ESP_LOGI(TAG, "Retrying connection to Wi-Fi...");
            } else {
                ESP_LOGE(TAG, "Failed to connect to Wi-Fi after %d attempts", WIFI_RETRY_ATTEMPT);
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
            break;
        default:
            break;
    }
}

// Initialize Wi-Fi system
esp_err_t tutorial_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_wifi_set_default_wifi_sta_handlers());

    tutorial_netif = esp_netif_create_default_wifi_sta();
    if (!tutorial_netif) return ESP_FAIL;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_cb,
                                                        NULL,
                                                        &wifi_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &ip_event_cb,
                                                        NULL,
                                                        &ip_event_handler));
    return ESP_OK;
}

// Connect to Wi-Fi
esp_err_t tutorial_connect(char* wifi_ssid, char* wifi_password)
{
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTHMODE,
        },
    };

    strncpy((char*)wifi_config.sta.ssid, wifi_ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, wifi_password, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) return ESP_OK;
    else return ESP_FAIL;
}

// Disconnect Wi-Fi
esp_err_t tutorial_disconnect(void)
{
    return esp_wifi_disconnect();
}

// Deinitialize Wi-Fi
esp_err_t tutorial_deinit(void)
{
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(tutorial_netif));
    esp_netif_destroy(tutorial_netif);
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, ESP_EVENT_ANY_ID, ip_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler));
    if (s_wifi_event_group) vEventGroupDelete(s_wifi_event_group);
    return ESP_OK;
}
