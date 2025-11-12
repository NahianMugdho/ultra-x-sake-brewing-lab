#pragma once

#include "esp_err.h"
#include "mqtt_client.h"

// ===================== BROKER CONFIG ======================
#define MQTT_BROKER_URI      "mqtts://192.168.88.221:8883"   // ✅ TLS (mqtts://)
#define MQTT_BROKER_USER     "admin"
#define MQTT_BROKER_PASS     "StrongPassword123"
#define MQTT_TOPIC_SUB       "text"
#define MQTT_TOPIC_PUB1      "ESP"
#define MQTT_TOPIC_PUB2      "ESP2"
#define MQTT_TOPIC_PUB3      "ESP3"
#define MQTT_CLIENT_ID       "ESP32_Client"
#define MQTT_KEEPALIVE       60  // seconds

// ===================== LWT CONFIG ========================
#define MQTT_LWT_TOPIC       "esp/test/lwt"
#define MQTT_LWT_MESSAGE     "ESP32 Disconnected Unexpectedly"
#define MQTT_LWT_QOS         1
#define MQTT_LWT_RETAIN      0

// ===================== CERTIFICATE =======================
// Embed broker/server CA certificate here (PEM format)
static const char mqtt_ca_cert_pem[] = 
"-----BEGIN CERTIFICATE-----\n"
"MIIDlzCCAn+gAwIBAgIUYXFm5E0lKt2DLHlYel4aBm5J1IUwDQYJKoZIhvcNAQEL\n"
"BQAwZjELMAkGA1UEBhMCQkQxDjAMBgNVBAgMBURoYWthMQ4wDAYDVQQHDAVEaGFr\n"
"YTEQMA4GA1UECgwHVWx0cmEtWDEMMAoGA1UECwwDSW9UMRcwFQYDVQQDDA4xOTIu\n"
"MTY4Ljg4LjIyMTAeFw0yNTA5MTIwMzE3NDhaFw0yNjA5MTIwMzE3NDhaMGYxCzAJ\n"
"BgNVBAYTAkJEMQ4wDAYDVQQIDAVEaGFrYTEOMAwGA1UEBwwFRGhha2ExEDAOBgNV\n"
"BAoMB1VsdHJhLVgxDDAKBgNVBAsMA0lvVDEXMBUGA1UEAwwOMTkyLjE2OC44OC4y\n"
"MjEwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDgBW91N8tV3sz3rX2K\n"
"MeysPQeOvYO4zuejuwin+TC3emR0cU4DauZcPKfycKQf8d2zozmdn/UlMVRlNIx8\n"
"/yEuPiwVMxBuzacQfZu5XjTt2lqHY643jgCHH//BTbMr9AI5/8PhBPr7QW31YErf\n"
"z3eaYBPI5+4/XvtC1BHP/SlhtW/yebtTz7cD+Z27iCmhD5285G8EyIIOT66F0Pld\n"
"6hp0dsEvO3CCXeJIsgZrBhCrDEbjx6r+j8+3uo0/bzMZslmFoTh2BdLXawaZccur\n"
"+w8lLo/Oat37CBqGH/FL1AwwevNwypic/QoF2Nu/3Re5EzmpC4O3wEOBH7W218X6\n"
"6nQfAgMBAAGjPTA7MBoGA1UdEQQTMBGHBMCoWN2CCWxvY2FsaG9zdDAdBgNVHQ4E\n"
"FgQUiDReSeQdzQf9A8E9zWc1Xzw+JbcwDQYJKoZIhvcNAQELBQADggEBAEhE9gfK\n"
"E90jv9fNNqqruuRlhF7LQf2KNNRTUFP9mVhzjZC2Zi13LUFxzaRA2WW7p6DW4boP\n"
"WRsfdNRURQlBdWAzJKhvbXuKqSynOmYnLglbx8TI7d8Z844CeQ1M2WUeuMmZGj7x\n"
"2E6wZJMgZJ5OKS96eyhwyftwMwsF9BvwPyOjj5K0Nd2+Jjvg1UpxBrRL919arXSv\n"
"jQPBqm5prEQNMcl36Ox5Xz2MVIfeP1iDVpBwo1+UY13cPrAhXydHx7LjCA0sk38t\n"
"TdrDVmz7P/7YpYgveODctySHC84F1j30IwXkxGT+37T0z2x70aW9SlSsVQsq6kF7\n"
"LkmSwhUjaTo4MHs=\n"
"-----END CERTIFICATE-----\n";

// ===================== FUNCTION DECLARATIONS =============
esp_err_t mqtt_start(void);
void mqtt_publish(const char *topic, const char *msg);
