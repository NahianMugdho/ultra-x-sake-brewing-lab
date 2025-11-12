
# import paho.mqtt.client as mqtt
# import threading
# import requests
# import time
# import datetime

# # ------------------- Configuration -------------------
# BROKER = "localhost"
# PORT = 1883
# USERNAME = "admin"
# PASSWORD = "StrongPassword123"

# WOKWI_HOST = "http://localhost:8180"
# SENSOR_URL = f"{WOKWI_HOST}/sensor"
# CONTROL_URL = f"{WOKWI_HOST}/control"

# # Topics for each sensor/actuator
# TOPICS_SEND = {
#     "ESP_S_1": "dht_temp",
#     "ESP_S_2": "dht_hum",
#     "ESP_S_3": "ds18b20",
#     "ESP_S_4": "sonar1",
#     "ESP_S_5": "sonar2",
#     "ESP_S_6": "mq"
# }

# TOPIC_RECEIVE = "ESP_S_7"  # fan speed control

# # ------------------- MQTT Callbacks -------------------
# def on_connect(client, userdata, flags, rc, properties=None):
#     if rc == 0:
#         print("[MQTT] ✅ Connected to broker")
#         client.subscribe(TOPIC_RECEIVE)  # subscribe to fan speed topic
#     else:
#         print(f"[MQTT] ❌ Connection failed ({rc})")

# def on_message(client, userdata, msg):
#     print(f"[MQTT] Received {msg.topic}: {msg.payload.decode()}")
#     try:
#         if msg.topic == TOPIC_RECEIVE:
#             speed = int(msg.payload.decode())
#             if 25 <= speed <= 40:
#                 # Send fan speed to ESP via HTTP
#                 requests.get(f"{CONTROL_URL}?target=fan&state=on&speed={speed}")
#                 print(f"[Wokwi] Fan speed set to {speed}")
#             else:
#                 print("[Warning] Fan speed out of range (25-40)")
#     except Exception as e:
#         print("[Error] Failed to send fan speed:", e)

# # ------------------- MQTT Setup -------------------
# client = mqtt.Client(client_id="py_bridge_sensors", clean_session=True)
# client.username_pw_set(USERNAME, PASSWORD)
# client.on_connect = on_connect
# client.on_message = on_message

# client.connect(BROKER, PORT, keepalive=60)
# client.loop_start()

# # ------------------- Sensor Loop -------------------
# def sensor_loop():
#     while True:
#         try:
#             res = requests.get(SENSOR_URL)
#             if res.status_code == 200:
#                 data = res.json()
#                 timestamp = datetime.datetime.now().isoformat()

#                 # Publish each sensor individually
#                 for topic, key in TOPICS_SEND.items():
#                     if key in data:
#                         client.publish(topic, str(data[key]))
#                         print(f"[MQTT] Published {topic}: {data[key]} | {timestamp}")

#             else:
#                 print(f"[HTTP] ❌ {res.status_code}")
#         except Exception as e:
#             print("[HTTP] Error:", e)

#         time.sleep(2)  # match sensor read interval

# # ------------------- Main -------------------
# try:
#     sensor_loop()
# except KeyboardInterrupt:
#     print("\nStopping...")
# finally:
#     client.loop_stop()
#     client.disconnect()
import paho.mqtt.client as mqtt
import requests
import time
import datetime

# ------------------- Configuration -------------------
BROKER = "localhost"
PORT = 1883
USERNAME = "admin"
PASSWORD = "StrongPassword123"

WOKWI_HOST = "http://localhost:8180"  # ESP WebServer
SENSOR_URL = f"{WOKWI_HOST}/sensor"
CONTROL_URL = f"{WOKWI_HOST}/control"

TOPICS_SEND = {
    "ESP_S_1": "dht_temp",
    "ESP_S_2": "dht_hum",
    "ESP_S_3": "ds18b20",
    "ESP_S_4": "sonar1",
    "ESP_S_5": "sonar2",
    "ESP_S_6": "mq",
    "ESP_S_8":"co2",
     "ESP_S_9":"sug",
     "ESP_S_10":"res",
}

TOPIC_RECEIVE = "ESP_S_7"  # fan speed control

# ------------------- MQTT Callbacks -------------------
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("[MQTT] ✅ Connected to broker")
        client.subscribe(TOPIC_RECEIVE)
    else:
        print(f"[MQTT] ❌ Connection failed ({rc})")

def on_message(client, userdata, msg):
    print(f"[MQTT] Received {msg.topic}: {msg.payload.decode()}")
    try:
        if msg.topic == TOPIC_RECEIVE:
            speed = int(msg.payload.decode())
            if 0 <= speed <= 100:  # Stepper speed range (0-100)
                # Send speed to ESP32 via HTTP
                url = f"{CONTROL_URL}?target=fan&state=on&speed={speed}"
                requests.get(url, timeout=1)
                print(f"[Wokwi] Stepper speed set to {speed}")
            else:
                print("[Warning] Speed out of range (0-100)")
    except Exception as e:
        print("[Error] Failed to send speed:", e)

# ------------------- MQTT Setup -------------------
client = mqtt.Client(client_id="py_bridge_sensors", clean_session=True)
client.username_pw_set(USERNAME, PASSWORD)
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT, keepalive=60)
client.loop_start()

# ------------------- Sensor Loop -------------------
def sensor_loop():
    while True:
        try:
            res = requests.get(SENSOR_URL, timeout=1)
            if res.status_code == 200:
                data = res.json()
                timestamp = datetime.datetime.now().isoformat()

                # Publish each sensor individually
                for topic, key in TOPICS_SEND.items():
                    if key in data:
                        client.publish(topic, str(data[key]))
                        print(f"[MQTT] Published {topic}: {data[key]} | {timestamp}")
            else:
                print(f"[HTTP] ❌ Status {res.status_code}")
        except Exception as e:
            print("[HTTP] Error:", e)

        time.sleep(2)

# ------------------- Main -------------------
try:
    sensor_loop()
except KeyboardInterrupt:
    print("\nStopping...")
finally:
    client.loop_stop()
    client.disconnect()
