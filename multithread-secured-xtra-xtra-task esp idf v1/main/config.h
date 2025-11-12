#pragma once

#include "esp_err.h"
#include "mqtt_client.h"

// Wi-Fi Credentials (replace with your actual values)
#define WIFI_SSID "Ultra-X BD"
#define WIFI_PASSWORD "Ky260=VmW.m!RC,uN&O{"

// ===================== SENSOR CONFIG ===================
#define SENSOR_PIN           33
#define INPUT_MIN            25
#define INPUT_MAX            40

// ===================== LEDC =============================
#define LEDC_PIN             2
#define LEDC_RESOLUTION      8
#define LEDC_FREQ            5000