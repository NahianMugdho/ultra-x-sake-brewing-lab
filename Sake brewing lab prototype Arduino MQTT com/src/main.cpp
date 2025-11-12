#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"
#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ------------------- WiFi -------------------
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""
#define WIFI_CHANNEL 6

// ------------------- Debug / LED -------------------
#define DEBUG_LED 2  // Built-in LED (ESP32 GPIO2)

// ------------------- Pins -------------------
#define ONE_WIRE_BUS 4
#define DHT_PIN 5
#define DHT_TYPE DHT22
#define SERVO_PIN 13
#define STEP_PIN 12
#define DIR_PIN 14
#define TRIG1 15
#define ECHO1 16
#define TRIG2 17
#define ECHO2 18
#define GAS_PIN 33
#define GAS_THRESHOLD 2000

// ------------------- Globals -------------------
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds(&oneWire);
DHT dht(DHT_PIN, DHT_TYPE);
Servo servo;
WebServer server(80);

// FreeRTOS mutex for protecting shared state
SemaphoreHandle_t stateMutex;

// Shared sensor / state variables (use volatile where appropriate)
volatile float dsTemp = 0.0;
volatile float dhtTemp = 0.0;
volatile float dhtHum = 0.0;
volatile float brix = 0.0;
volatile int distance1 = 0;
volatile int distance2 = 0;
volatile int gasValue = 0;
volatile double ppm = 0;
volatile bool servoActive = false;
volatile bool stepperActive = false;
volatile int stepSpeed = 30; // 0..100

String co2 = "COFF";
String sug = "FON";
String res = "Farmentation OFF";

const float RL_VALUE = 5.0;    // load resistor value in kΩ
const float RO = 10.0;         // fixed Ro in kΩ (simulation)
const int ADC_MAX = 4095;      // ADC resolution max (ESP32 = 4095).

// calibration
float slope = 3.8333;  // old slope (you can re-calibrate)
float intercept = -9.5; // old intercept

// sensor timing
#define READ_INTERVAL 1000  // ms
#define CONTROL_INTERVAL 20 // ms

// curves: { log10(ppm_point), log10(Rs/Ro_at_point), slope }
float AlcoholCurve[3] = {2.3, 0.28, -0.47};

// ---------- helpers ----------
float MQResistanceCalculation(int raw_adc) {
  if (raw_adc <= 0) return -1.0; // invalid
  if (raw_adc >= ADC_MAX) return 1e9;
  return ( RL_VALUE * ( (float)(ADC_MAX - raw_adc) / (float)raw_adc ) );
}

float MQRead(int mq_pin) {
  float rs = 0.0;
  const int samples = 3;        // fewer samples to reduce blocking
  for (int i = 0; i < samples; ++i) {
    int raw = analogRead(mq_pin);
    float r = MQResistanceCalculation(raw);
    if (r < 0) return -1.0;
    rs += r;
    // short yield so other tasks remain responsive
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return rs / (float)samples;
}

bool isFinitePositive(float v) {
  return isfinite(v) && (v > 0.0);
}

float MQGetPercentageFloat(float rs_ro_ratio, float *pcurve) {
  if (!isFinitePositive(rs_ro_ratio)) return NAN;
  float log_ratio = log10(rs_ro_ratio);
  float x = ( (log_ratio - pcurve[1]) / pcurve[2] ) + pcurve[0];
  return pow(10.0, x);
}

// Function to measure distance from sonar (in cm)
float getDistance() {
  digitalWrite(TRIG2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG2, LOW);

  long duration = pulseIn(ECHO2, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 999.0; // no echo
  float distance = duration * 0.034 / 2.0;
  return distance;
}

// Convert distance (cm) to °Brix
float distanceToBrix(float distance) {
  float b = slope * distance + intercept;
  if (b < 0) b = 0;
  return b;
}

// ------------------- Web Dashboard (same HTML but with full absolute fetch you had) -------------------
void handleRoot() {
  String html = R"rawliteral(
  <!DOCTYPE html><html>
  <head>
    <title>Fermentation Dashboard</title>
    <meta name='viewport' content='width=device-width, initial-scale=1'>
    <style>
      body { font-family: Arial; background:#f2f2f2; text-align:center; padding:10px;}
      .card { background:white; border-radius:12px; padding:20px; margin:10px auto; width:320px;
              box-shadow:0 2px 6px rgba(0,0,0,0.2);}
      .on {color:green; font-weight:bold;}
      .off {color:red; font-weight:bold;}
      input[type=range] { width: 90%; }
    </style>
  </head>
  <body>
    <h1>Fermentation Monitoring</h1>
    <div class='card'>
      <h3>Sensor Readings</h3>
      DS18B20: <span id='ds'>0</span>°C<br>
      DHT22 Temp: <span id='dt'>0</span>°C<br>
      Humidity: <span id='dh'>0</span>%<br>
      Sonar1: <span id='s1'>0</span> cm<br>
      Sonar2: <span id='s2'>0</span> °brix<br>
      MQ: <span id='mq'>0</span><br>
      Fermentation: <span id='sug'> </span><br>
      CO2: <span id='co2'> </span><br>
      Result: <span id='res'> </span><br>
    </div>

    <div class='card'>
      <h3>Actuators</h3>
      Pump: <span id='pump'>OFF</span><br>
      Fan (Stepper): <span id='fan'>OFF</span><br><br>

      <h4>Stepper Speed Control</h4>
      <input type="range" id="spd" min="0" max="100" value="30" oninput="setSpeed(this.value)">
      <div>Speed: <span id="spdVal">30</span></div>
    </div>

    <script>
      async function setSpeed(v){
        document.getElementById("spdVal").innerText = v;
        try {
          await fetch(`http://localhost:8180/control?target=fan&state=on&speed=${v}`);
        } catch(e) {
          console.log("Error setting speed", e);
        }
      }

      async function updateData(){
        try {
          const controller = new AbortController();
          const timeoutId = setTimeout(() => controller.abort(), 900);
          let res = await fetch('http://localhost:8180/sensor', { signal: controller.signal });
          clearTimeout(timeoutId);
          let d = await res.json();
          document.getElementById('ds').innerText = d.ds18b20;
          document.getElementById('dt').innerText = d.dht_temp;
          document.getElementById('dh').innerText = d.dht_hum;
          document.getElementById('s1').innerText = d.sonar1;
          document.getElementById('s2').innerText = d.sonar2;
          document.getElementById('mq').innerText = d.mq;
          document.getElementById('pump').innerText = d.pump ? "ON" : "OFF";
          document.getElementById('fan').innerText = d.fan ? "ON" : "OFF";
          document.getElementById('sug').innerText = d.sug;
          document.getElementById('co2').innerText = d.co2;
          document.getElementById('res').innerText = d.res;
          document.getElementById("spdVal").innerText = d.speed;
          document.getElementById("spd").value = d.speed;
        } catch(e) {
          console.log("updateData error", e);
        }
      }

      setInterval(updateData, 2000);
      window.onload = updateData;
    </script>
  </body>
  </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

// ------------------- JSON API -------------------
void handleSensor() {
  // gather snapshot under mutex
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    String json = "{";
    json += "\"ds18b20\":" + String((double)dsTemp, 2) + ",";
    json += "\"dht_temp\":" + String((double)dhtTemp, 2) + ",";
    json += "\"dht_hum\":" + String((double)dhtHum, 2) + ",";
    json += "\"sonar1\":" + String((int)distance1) + ",";
    json += "\"sonar2\":" + String((double)brix) + ",";
    json += "\"mq\":" + String((double)ppm) + ",";
    json += "\"pump\":" + String(servoActive ? 1 : 0) + ",";
    json += "\"fan\":" + String(stepperActive ? 1 : 0) + ",";
    json += "\"speed\":" + String(stepSpeed);
    json += ",\"sug\":\"" + sug + "\"";
    json += ",\"co2\":\"" + co2 + "\"";
    json += ",\"res\":\"" + res + "\"";
    json += "}";
    xSemaphoreGive(stateMutex);
    server.send(200, "application/json", json);
  } else {
    server.send(503, "text/plain", "busy");
  }
}

// ------------------- Control API -------------------
void handleControl() {
  if (!server.hasArg("target") || !server.hasArg("state")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }

  String target = server.arg("target");
  String state = server.arg("state");

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (target == "fan") {
      if (server.hasArg("speed")) {
        int sp = server.arg("speed").toInt();
        if (sp < 0) sp = 0;
        if (sp > 100) sp = 100;
        stepSpeed = sp;
        Serial.print("[HTTP] Speed set to: ");
        Serial.println(stepSpeed);
      }
      stepperActive = (state == "on");
      server.send(200, "text/plain", "OK");
      xSemaphoreGive(stateMutex);
      return;
    } else if (target == "pump") {
      servoActive = (state == "on");
      servo.write(servoActive ? 180 : 90);
      server.send(200, "text/plain", "OK");
      xSemaphoreGive(stateMutex);
      return;
    } else {
      xSemaphoreGive(stateMutex);
      server.send(404, "text/plain", "Unknown target");
      return;
    }
  } else {
    server.send(503, "text/plain", "busy");
  }
}

// ------------------- Tasks -------------------

// sensorTask: reads sensors every READ_INTERVAL ms
void sensorTask(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    // read DS18B20
    ds.requestTemperatures();
    float ds_t = ds.getTempCByIndex(0);
    if (ds_t == -127.0) ds_t = 0;

    // DHT
    float dht_t = dht.readTemperature();
    float dht_h = dht.readHumidity();
    if (isnan(dht_t)) dht_t = 0;
    if (isnan(dht_h)) dht_h = 0;

    // Sonar1
    digitalWrite(TRIG1, LOW); delayMicroseconds(2);
    digitalWrite(TRIG1, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG1, LOW);
    long duration1 = pulseIn(ECHO1, HIGH, 30000);
    int dist1 = (duration1 == 0) ? 999 : (int)(duration1 / 58.2);

    // Sonar2 -> distanceToBrix
    float distance = getDistance();
    float b = distanceToBrix(distance);

    // MQ sensor
    float rs = MQRead(GAS_PIN);
    double p = NAN;
    if (isFinitePositive(rs) && RO > 0.0) {
      float ratio = rs / RO;
      p = MQGetPercentageFloat(ratio, AlcoholCurve);
    }

    // compute flags & result string
    String local_co2 = (isnan(p) ? "COFF" : (p < 1000 ? "COFF" : "CON"));
    String local_sug = (b < 10 ? "FEND" : "FON");
    String local_res;
    if (local_co2 == "CON" && local_sug == "FEND") local_res = "Fermentation Complete";
    else if (local_co2 == "COFF" && local_sug == "FEND") local_res = "Farmentation off, Check CO2, Sugar level ok";
    else if (local_co2 == "CON" && local_sug == "FON") local_res = "Farmentation Ongoing, High Sugar";
    else local_res = "Farmentation OFF";

    // store snapshot under mutex
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      dsTemp = ds_t;
      dhtTemp = dht_t;
      dhtHum = dht_h;
      distance1 = dist1;
      brix = b;
      gasValue = (int)rs;
      ppm = isnan(p) ? NAN : p;
      co2 = local_co2;
      sug = local_sug;
      res = local_res;
      xSemaphoreGive(stateMutex);
    }

    // breathing time between reads
    vTaskDelay(pdMS_TO_TICKS(READ_INTERVAL));
  }
}

// controlTask: runs actuator logic (non-blocking)
void controlTask(void *pvParameters) {
  (void) pvParameters;
  unsigned long lastStepTimeUs = micros();
  for (;;) {
    // sample snapshot quick
    float local_dsTemp;
    int local_distance1;
    int local_stepSpeed;
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      local_dsTemp = dsTemp;
      local_distance1 = distance1;
      local_stepSpeed = stepSpeed;
      xSemaphoreGive(stateMutex);
    } else {
      local_dsTemp = 0;
      local_distance1 = 999;
      local_stepSpeed = 30;
    }

    // stepperActive logic
    bool local_stepperActive = local_dsTemp > 19;

    // compute pulse interval from stepSpeed
    unsigned long intervalUs = map(local_stepSpeed, 0, 100, 2000, 200);

    unsigned long nowUs = micros();
    if (local_stepperActive && (nowUs - lastStepTimeUs >= intervalUs)) {
      lastStepTimeUs = nowUs;
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN, LOW);
      // toggle LED for visible heartbeat
      
    }
    // LED heartbeat independent of step pulse
static unsigned long lastLedToggle = 0;
unsigned long ledIntervalMs = map(local_stepSpeed, 0, 100, 1000, 100);
if (millis() - lastLedToggle >= ledIntervalMs) {
    lastLedToggle = millis();
    digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED));
}

    // servo control based on distance1
    static int pos = 90;
    static int dir = 1;
    if (local_distance1 < 20) {
      servoActive = true;
      pos += dir;
      if (pos >= 180) dir = -1;
      if (pos <= 0) dir = 1;
      servo.write(pos);
    } else {
      servoActive = false;
      servo.write(90);
    }

    // commit stepperActive & servoActive under mutex
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      stepperActive = local_stepperActive;
      xSemaphoreGive(stateMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(CONTROL_INTERVAL));
  }
}

// webTask: keep server responsive (call handleClient frequently)
void webTask(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    server.handleClient();
    // tiny delay to yield CPU to other tasks
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ------------------- Setup & Loop -------------------
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Booting...");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);
  Serial.print("Connecting");
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
    if (millis() - startAttempt > 10000) break;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi not connected (continuing in AP-less mode)");
  }

  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);

  ds.begin();
  dht.begin();
  servo.attach(SERVO_PIN);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  pinMode(GAS_PIN, INPUT);

  digitalWrite(DIR_PIN, HIGH);

  // create mutex
  stateMutex = xSemaphoreCreateMutex();
  if (stateMutex == NULL) {
    Serial.println("Failed to create mutex!");
    while (1) { delay(1000); }
  }

  server.on("/", handleRoot);
  server.on("/sensor", handleSensor);
  server.on("/control", handleControl);
  server.begin();

  Serial.println("HTTP server started");

  // create tasks
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 2, NULL, 1); // core 1
  xTaskCreatePinnedToCore(controlTask, "ControlTask", 3072, NULL, 2, NULL, 1); // core 1
  xTaskCreatePinnedToCore(webTask, "WebTask", 4096, NULL, 3, NULL, 0); // core 0
}

void loop() {
  // empty — work is handled in tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}
