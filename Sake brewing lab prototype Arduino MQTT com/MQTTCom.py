import paho.mqtt.client as mqtt
import datetime
import threading
import ssl
import time

# ============================================================
# MQTT BROKER CONFIGURATION
# ============================================================
BROKER = "localhost"             # or your PC IP if using remotely
PORT = 8883                      # TLS port
USERNAME = "admin"
PASSWORD = "StrongPassword123"
CA_CERT = "broker.crt"           # Certificate Authority / self-signed cert

# ============================================================
# TOPIC STRUCTURE
# ============================================================
TOPICS = {
    "DHT_TEMP": "ESP_S_1",       # DHT22 Temperature (send)
    "DHT_HUM": "ESP_S_2",        # DHT22 Humidity (send)
    "DS18_TEMP": "ESP_S_3",      # DS18B20 Temperature (send)
    "SONAR1": "ESP_S_4",         # Ultrasonic 1 (send)
    "SONAR2": "ESP_S_5",         # Ultrasonic 2 (send)
    "MQ": "ESP_S_6",             # Gas Sensor (send)
    "FAN": "ESP_S_7",
    "DATA1": "ESP_S_8" ,
    "DATA2": "ESP_S_9",
    "Result":"ESP_S_10",
    "pic": "lab/detections" # Fan speed control (receive)
}

# ============================================================
# MQTT CALLBACKS
# ============================================================
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[MQTT] ✅ Connected to broker (TLS secured)")
        # Subscribe to all ESP sensor data + Fan control
        for key, topic in TOPICS.items():
            if key != "FAN":  # subscribe only to sensor data topics
                client.subscribe(topic, qos=1)
                print(f"[MQTT] Subscribed to {topic}")
        print(f"[MQTT] Ready to publish fan control via {TOPICS['FAN']}")
    else:
        print(f"[MQTT] ❌ Failed to connect (rc={rc})")


def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    timestamp = datetime.datetime.now().strftime("%H:%M:%S")
    print(f"[MQTT] RECV | Topic={msg.topic} | Payload={payload} | Time={timestamp}")


# ============================================================
# MQTT CLIENT SETUP
# ============================================================
client = mqtt.Client(client_id="fermentation_monitor_sub", clean_session=True)
client.username_pw_set(USERNAME, PASSWORD)

# Enable TLS Encryption
client.tls_set(
    ca_certs=CA_CERT,
    certfile=None,
    keyfile=None,
    tls_version=ssl.PROTOCOL_TLSv1_2
)
client.tls_insecure_set(True)  # Allow self-signed certificates

client.on_connect = on_connect
client.on_message = on_message

# Connect to broker
client.connect(BROKER, PORT, keepalive=60)
client.loop_start()


# ============================================================
# PUBLISHER THREAD — FAN CONTROL
# ============================================================
def fan_control_publisher():
    """
    Allows you to manually send fan speed or control commands
    Example commands:
      0   → Stop fan
      50  → Medium speed
      100 → Full speed
    """
    try:
        while True:
            speed = input("[Fan Control] Enter fan speed (0-100 or 'off'): ").strip()
            if speed == "":
                continue
            client.publish(TOPICS["FAN"], payload=speed, qos=1)
            print(f"[Fan Control] Sent to {TOPICS['FAN']}: {speed}")
    except KeyboardInterrupt:
        print("\n[Fan Control] Exiting...")


# ============================================================
# MAIN LOOP
# ============================================================
publisher_thread = threading.Thread(target=fan_control_publisher, daemon=True)
publisher_thread.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n[Main] Shutting down...")
finally:
    client.loop_stop()
    client.disconnect()
