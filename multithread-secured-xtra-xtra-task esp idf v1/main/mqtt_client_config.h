
#pragma once

#include "esp_err.h"
#include "mqtt_client.h"

// ===================== BROKER CONFIG ======================
#define MQTT_BROKER_URI      "mqtts://192.168.88.60:8883"   // ✅ TLS with IP (no hostname)
#define MQTT_BROKER_USER     "admin"
#define MQTT_BROKER_PASS     "StrongPassword123"
#define MQTT_TOPIC_SUB       "text"
#define MQTT_TOPIC_PUB1      "ESP"
#define MQTT_TOPIC_PUB2      "ESP2"
#define MQTT_TOPIC_PUB3      "ESP3"
#define MQTT_CLIENT_ID       "ESP32_Client"
#define MQTT_KEEPALIVE       60  // seconds


#define MQTT_TOPIC_CO2       "CO2"
#define MQTT_TOPIC_BOWL      "bowl"
#define MQTT_TOPIC_SONAR     "sonar"
#define MQTT_TOPIC_SUGAR     "sugar"

#define MQTT_TOPIC_CO2T      "CO2T"
#define MQTT_TOPIC_BOWLT     "bowlT"
#define MQTT_TOPIC_SONART    "sonarT"
#define MQTT_TOPIC_SUGART    "sugarT"

// ===================== LWT CONFIG ========================
#define MQTT_LWT_TOPIC       "esp/test/lwt"
#define MQTT_LWT_MESSAGE     "ESP32 Disconnected Unexpectedly"
#define MQTT_LWT_QOS         1
#define MQTT_LWT_RETAIN      0

// ===================== CERTIFICATE =======================
// Embed broker/server CA certificate here (PEM format) - REPLACE WITH YOUR NEW CERT FROM Step 2
// Embed broker/server CA certificate here (PEM format)
static const char mqtt_ca_cert_pem[] = 
"-----BEGIN CERTIFICATE-----\n"
"MIIDlTCCAn2gAwIBAgIUIluSc2wCk3YhGMzBk9w5X9CeKLAwDQYJKoZIhvcNAQEL\n"
"BQAwZTELMAkGA1UEBhMCQkQxDjAMBgNVBAgMBURoYWthMQ4wDAYDVQQHDAVEaGFr\n"
"YTEQMA4GA1UECgwHVWx0cmEtWDEMMAoGA1UECwwDSW9UMRYwFAYDVQQDDA0xOTIu\n"
"MTY4Ljg4LjYwMB4XDTI1MTAwOTA0MzM0M1oXDTI2MTAwOTA0MzM0M1owZTELMAkG\n"
"A1UEBhMCQkQxDjAMBgNVBAgMBURoYWthMQ4wDAYDVQQHDAVEaGFrYTEQMA4GA1UE\n"
"CgwHVWx0cmEtWDEMMAoGA1UECwwDSW9UMRYwFAYDVQQDDA0xOTIuMTY4Ljg4LjYw\n"
"MIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAugydhbbW+YBCkKAk+1Ul\n"
"XXAN16MhCFc+KZPHe2Kbx1hsDju1z4hy0D00DO1ZtJJgSvbZFZqFjJrNpjZh8fBV\n"
"a4EJRT1XH1RJ51HVxXLseuOxWLCZoGMHm+lzjIsnrsW+CDTz2QdPVRBuXOvkERuF\n"
"h1bjDGk7Kg4MZoaRCmwc07s03g4+6lziI/bjaH6tQzKsfR7YbLoDNhpGUc58CyJO\n"
"v/TO2Yn2z3oYUMOJvAKcyhrMKqcppnb48n+8yjdps9ja63E6h2lECeJD2i2aGPHu\n"
"IQ7eYhef0i70NZbtRiz89EutRMKyDRYqCDM3O8HTjDBZXfFQlwX0qpoGpM3FAt8q\n"
"lQIDAQABoz0wOzAaBgNVHREEEzARhwTAqFg8gglsb2NhbGhvc3QwHQYDVR0OBBYE\n"
"FE6p7o9b9TMRPmKheR5+2VUUu8tOMA0GCSqGSIb3DQEBCwUAA4IBAQAByRa+Ry9N\n"
"v7biClTm8YkCxNJOk+qaLSHVV409v/F1ST/gl3KDStPq38GjGUH1HNpzzmmTKFUe\n"
"o4g7ugKC5VfSmTqc/iOb+Sq9A+2M8KJjjhjUYu3X3tbcE6BF0CwuM6FmOd3ghrLz\n"
"oTXl2awobVKW1VXhkHYcsS8kNOtfVbLCLKAjvkDOA8+/qdq/31jP8UqocHKR7C2q\n"
"uDhWkWr+eqReKgzec2XvW2qEHoRz4HY049DSvirUMxKduAVbRUtfPNawDNCKuMrI\n"
"8kwWXM6tS7y5iDuFTHJvMyrVGvOiC5LaVxqy1R7YrfULNEmahX/2gHY4IQlBD1/r\n"
"zVjRpENi+bd9\n"
"-----END CERTIFICATE-----\n";

// ===================== FUNCTION DECLARATIONS =============
esp_err_t mqtt_start(void);
void mqtt_publish(const char *topic, const char *msg);