
import time
import sys
import ssl
import random
import paho.mqtt.client as mqtt

BROKER = "localhost"       # অথবা broker IP/Domain
PORT = 8883                # TLS enabled port
ESP_TOPIC = "ESPX3"
STATUS_TOPIC = "ESPY3"
VOLT_TOPIC = "voltX3"
Thres_Topic = "thresX3"

client_id = "py_publisher_003"

# Must match broker .env
USERNAME = "admin"
PASSWORD = "StrongPassword123"

# Broker Certificate
CA_CERT = "broker.crt"     # তোমার broker এর CA cert

# ---------- MQTT Callbacks ----------
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[publisher] Connected to broker ✅ (TLS)")
        client.subscribe(STATUS_TOPIC, qos=1)
        print(f"[publisher] Subscribed to {STATUS_TOPIC}")
    else:
        print(f"[publisher] Failed to connect, rc={rc}")

def on_message(client, userdata, msg):
    print(f"[publisher] RECV {msg.topic} | payload={msg.payload.decode()}")

# ---------- MQTT Client Setup ----------
client = mqtt.Client(client_id=client_id, clean_session=True)
client.username_pw_set(USERNAME, PASSWORD)

# TLS config
client.tls_set(
    ca_certs=CA_CERT,
    certfile=None,
    keyfile=None,
    tls_version=ssl.PROTOCOL_TLSv1_2
)
client.tls_insecure_set(True)  # self-signed হলে allow করবে

client.on_connect = on_connect
client.on_message = on_message

# Connect to broker
client.connect(BROKER, PORT, keepalive=60)
client.loop_start()

# ---------- fixed channel ----------
last_fixed_time = 0  # track last publish time for FIXED_TOPIC


# ---------- Publisher Loop ----------
try:
    while True:
        # 3110 - 3170 range
        esp_value = random.randint(3110, 3170)

        # 2.5 - 3.3 range (2 decimal precision)
        volt_value = round(random.uniform(2.5, 3.3), 2)
        thres_value = 50
        # publish to ESP_TOPIC
        client.publish(ESP_TOPIC, esp_value, qos=1)
        print(f"[publisher] Published {esp_value} to {ESP_TOPIC}")

        # publish to VOLT_TOPIC
        client.publish(VOLT_TOPIC, volt_value, qos=1)
        print(f"[publisher] Published {volt_value} to {VOLT_TOPIC}")
        # check if 30 sec passed for FIXED_TOPIC
        current_time = time.time()
        if current_time - last_fixed_time >= 30:
            client.publish(Thres_Topic, 50, qos=1)
            print(f"[publisher] Published 50 to {Thres_Topic}")
            last_fixed_time = current_time

        time.sleep(3)

except KeyboardInterrupt:
    print("\n[publisher] stopping...")
finally:
    client.loop_stop()
    client.disconnect()
    sys.exit(0)

