#include "wifi_manager.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_http_server.h"
#include <string.h>

static const char *TAG = "WIFI_MANAGER";

static EventGroupHandle_t wifi_event_group;
static esp_netif_t *ap_netif = NULL;
static esp_netif_t *sta_netif = NULL;
static httpd_handle_t server = NULL;
static int retry_count = 0;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define MAX_RETRY 8

// HTML Page for Captive Portal
static const char *html_page = 
"<!DOCTYPE html><html><head>"
"<meta name='viewport' content='width=device-width, initial-scale=1'>"
"<meta charset='UTF-8'>"
"<title>ESP32 Wi-Fi Setup</title>"
"<style>"
"body{font-family:Arial;text-align:center;background:#f0f0f0;padding:20px}"
".container{max-width:400px;margin:auto;background:white;padding:30px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}"
"h1{color:#333;margin-bottom:10px}"
".subtitle{color:#666;margin-bottom:30px}"
"input,button{width:100%;padding:12px;margin:10px 0;border:1px solid #ddd;border-radius:5px;box-sizing:border-box;font-size:16px}"
"button{background:#4CAF50;color:white;border:none;cursor:pointer;font-weight:bold}"
"button:hover{background:#45a049}"
".info{color:#666;font-size:14px;margin-top:20px}"
"</style></head><body>"
"<div class='container'>"
"<h1>🌐 ESP32 Setup</h1>"
"<p class='subtitle'>Wi-Fi কনফিগার করুন</p>"
"<form action='/save' method='POST'>"
"<input type='text' name='ssid' placeholder='Wi-Fi নাম (SSID)' required>"
"<input type='password' name='password' placeholder='Password' required>"
"<button type='submit'>সংরক্ষণ করুন</button>"
"</form>"
"<div class='info'>ডিভাইস সংরক্ষণের পরে রিস্টার্ট হবে</div>"
"</div></body></html>";

static const char *success_page = 
"<!DOCTYPE html><html><head>"
"<meta charset='UTF-8'>"
"<title>Success</title>"
"<style>body{font-family:Arial;text-align:center;padding:50px}"
"h1{color:#4CAF50}</style></head><body>"
"<h1>✅ সফল!</h1>"
"<p>Wi-Fi কনফিগারেশন সংরক্ষিত হয়েছে।</p>"
"<p>ডিভাইস রিস্টার্ট হচ্ছে...</p></body></html>";

// Wi-Fi Event Handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_count < MAX_RETRY) {
            esp_wifi_connect();
            retry_count++;
            ESP_LOGI(TAG, "Retry connecting... (%d/%d)", retry_count, MAX_RETRY);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Failed to connect to Wi-Fi");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// HTTP Handler: Root (/)
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}

// HTTP Handler: Save credentials
static esp_err_t save_handler(httpd_req_t *req)
{
    char content[200];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    content[ret] = '\0';
    
    char ssid[33] = {0};
    char password[65] = {0};
    
    // Parse form data (simple parsing)
    char *ssid_ptr = strstr(content, "ssid=");
    char *pass_ptr = strstr(content, "password=");
    
    if (ssid_ptr && pass_ptr) {
        ssid_ptr += 5;  // Skip "ssid="
        char *ssid_end = strchr(ssid_ptr, '&');
        if (ssid_end) {
            int ssid_len = ssid_end - ssid_ptr;
            if (ssid_len > 32) ssid_len = 32;
            strncpy(ssid, ssid_ptr, ssid_len);
        }
        
        pass_ptr += 9;  // Skip "password="
        char *pass_end = strchr(pass_ptr, '&');
        int pass_len = pass_end ? (pass_end - pass_ptr) : strlen(pass_ptr);
        if (pass_len > 64) pass_len = 64;
        strncpy(password, pass_ptr, pass_len);
        
        // URL decode (replace + with space, etc.)
        for (int i = 0; ssid[i]; i++) {
            if (ssid[i] == '+') ssid[i] = ' ';
        }
        for (int i = 0; password[i]; i++) {
            if (password[i] == '+') password[i] = ' ';
        }
        
        ESP_LOGI(TAG, "Received SSID: %s", ssid);
        
        // Save credentials
        wifi_manager_save_credentials(ssid, password);
        
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, success_page, strlen(success_page));
        
        // Restart after 3 seconds
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    } else {
        httpd_resp_send_500(req);
    }
    
    return ESP_OK;
}

// Start HTTP Server
static esp_err_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler,
        };
        httpd_register_uri_handler(server, &root);
        
        httpd_uri_t save = {
            .uri = "/save",
            .method = HTTP_POST,
            .handler = save_handler,
        };
        httpd_register_uri_handler(server, &save);
        
        ESP_LOGI(TAG, "Web server started");
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to start web server");
    return ESP_FAIL;
}

// Initialize Wi-Fi Manager
esp_err_t wifi_manager_init(void)
{
    wifi_event_group = xEventGroupCreate();
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    return ESP_OK;
}

// Check if configured
bool wifi_manager_is_configured(void)
{
    nvs_handle_t handle;
    uint8_t configured = 0;
    
    if (nvs_open(WIFI_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        nvs_get_u8(handle, WIFI_CONFIGURED_KEY, &configured);
        nvs_close(handle);
    }
    
    return configured == 1;
}

// Save credentials to NVS
esp_err_t wifi_manager_save_credentials(const char *ssid, const char *password)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    
    nvs_set_str(handle, WIFI_SSID_KEY, ssid);
    nvs_set_str(handle, WIFI_PASS_KEY, password);
    nvs_set_u8(handle, WIFI_CONFIGURED_KEY, 1);
    nvs_commit(handle);
    nvs_close(handle);
    
    ESP_LOGI(TAG, "Credentials saved");
    return ESP_OK;
}

// Clear credentials
esp_err_t wifi_manager_clear_credentials(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    
    nvs_erase_all(handle);
    nvs_commit(handle);
    nvs_close(handle);
    
    ESP_LOGI(TAG, "Credentials cleared");
    return ESP_OK;
}

// Connect to Wi-Fi
esp_err_t wifi_manager_connect(void)
{
    char ssid[33] = {0};
    char password[65] = {0};
    size_t len;
    
    nvs_handle_t handle;
    if (nvs_open(WIFI_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return ESP_FAIL;
    }
    
    len = sizeof(ssid);
    nvs_get_str(handle, WIFI_SSID_KEY, ssid, &len);
    len = sizeof(password);
    nvs_get_str(handle, WIFI_PASS_KEY, password, &len);
    nvs_close(handle);
    
    if (strlen(ssid) == 0) {
        ESP_LOGW(TAG, "No saved credentials");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Connecting to: %s", ssid);
    
    sta_netif = esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    
    wifi_config_t wifi_config = {0};
    strcpy((char *)wifi_config.sta.ssid, ssid);
    strcpy((char *)wifi_config.sta.password, password);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to Wi-Fi");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to connect");
        return ESP_FAIL;
    }
}

// Start Captive Portal
esp_err_t wifi_manager_start_portal(void)
{
    ESP_LOGI(TAG, "Starting AP Mode...");
    
    ap_netif = esp_netif_create_default_wifi_ap();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_MANAGER_AP_SSID,
            .ssid_len = strlen(WIFI_MANAGER_AP_SSID),
            .channel = WIFI_MANAGER_AP_CHANNEL,
            .password = WIFI_MANAGER_AP_PASSWORD,
            .max_connection = WIFI_MANAGER_AP_MAX_CONN,
            .authmode = WIFI_AUTH_OPEN,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "AP Started: %s", WIFI_MANAGER_AP_SSID);
    
    start_webserver();
    
    return ESP_OK;
}

// Stop Portal
esp_err_t wifi_manager_stop_portal(void)
{
    if (server) {
        httpd_stop(server);
        server = NULL;
    }
    
    esp_wifi_stop();
    esp_wifi_deinit();
    
    return ESP_OK;
}