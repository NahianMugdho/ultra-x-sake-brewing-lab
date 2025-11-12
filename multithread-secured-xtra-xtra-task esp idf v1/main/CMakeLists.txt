idf_component_register(SRCS "app_main.c" "mqtt_client_config.c" "tutorial.c" "led_config.c"
                       PRIV_REQUIRES spi_flash nvs_flash
                       INCLUDE_DIRS ""
                       REQUIRES driver esp_wifi esp_netif freertos lwip esp_event log mqtt
                       )

