import time
import sys
import ssl
import random
import paho.mqtt.client as mqtt

BROKER = "localhost"       # অথবা broker IP/Domain
PORT = 8883                # TLS enabled port
ESP_TOPIC = "bowl"


client_id = "bowl_publisher"

# Must match broker .env
USERNAME = "admin"
PASSWORD = "StrongPassword123"

# Broker Certificate
CA_CERT = "broker.crt"     # তোমার broker এর CA cert

# ---------- MQTT Callbacks ----------
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[publisher] Connected to broker ✅ (TLS)")

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

# ---------- Publisher Loop ----------
try:
    start_time = time.time()
    while True:
        # Determine elapsed time in 20-second cycle
        elapsed = (time.time() - start_time) % 20

        if elapsed < 10:
            # First 10 sec: 40-50
            esp_value = random.randint(40, 50)
            volt_value = round(random.uniform(40, 50), 2)
        else:
            # Next 10 sec: 20-30
            esp_value = random.randint(20, 30)
            volt_value = round(random.uniform(20, 30), 2)

        thres_value = 50  # can still use as fixed if needed

        # Publish to ESP_TOPIC
        client.publish(ESP_TOPIC, esp_value, qos=1)
        print(f"[publisher] Published {esp_value} to {ESP_TOPIC}")





        time.sleep(3)

except KeyboardInterrupt:
    print("\n[publisher] stopping...")
finally:
    client.loop_stop()
    client.disconnect()
    sys.exit(0)
