// #pragma once

// #include "esp_err.h"
// #include "mqtt_client.h"

// // Wi-Fi Credentials (replace with your actual values)
// // #define WIFI_SSID "Ultra-X BD"
// // #define WIFI_PASSWORD "Ky260=VmW.m!RC,uN&O{"

// // ===================== SENSOR CONFIG ===================
// #define SENSOR_PIN           33
// #define INPUT_MIN            25
// #define INPUT_MAX            40

// // ===================== LEDC =============================
// #define LEDC_PIN             2
// #define LEDC_RESOLUTION      8
// #define LEDC_FREQ            5000

#pragma once
#include "esp_err.h"
#include "mqtt_client.h"

// ===================== SENSOR PINS ===================
#define DHT_PIN 5
#define MQ135_PIN 33
#define DS18B20_PIN 4

// ===================== MOTOR PINS ====================
// Water Pump (L298N Side 1)
#define PUMP_ENABLE_PIN 13
#define PUMP_IN3_PIN 27
#define PUMP_IN4_PIN 32

// Fan Motor (L298N Side 2)
#define MOTOR_ENABLE_PIN 14
#define MOTOR_IN1_PIN 12
#define MOTOR_IN2_PIN 26

// ===================== SONAR PINS ====================
#define SONAR1_TRIG_PIN 15
#define SONAR1_ECHO_PIN 16

#define SONAR2_TRIG_PIN 17
#define SONAR2_ECHO_PIN 18

// ===================== LED PIN =======================
#define LEDC_PIN 2
#define LEDC_RESOLUTION 8
#define LEDC_FREQ 5000

// ===================== THRESHOLDS ====================
#define DS18B20_TEMP_THRESHOLD 40.0f
#define SONAR1_DISTANCE_THRESHOLD 5.0f