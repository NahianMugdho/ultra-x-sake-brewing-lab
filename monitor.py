import paho.mqtt.client as mqtt
import datetime
import threading
import ssl

BROKER = "localhost"       # বা তোমার PC-এর IP
PORT = 8883                # Secure MQTT TLS port
ESP_TOPIC = "ESP2"
ESP2_TOPIC = "test/topic"
TEXT_TOPIC = "text"
ESP3_TOPIC = "ESP3"
JM_TOPIC = "jmeter"
USERNAME = "admin"         # .env এ যেটা সেট করেছো
PASSWORD = "StrongPassword123"
SEN1_TOPIC = "CO2T"
SEN2_TOPIC = "sugarT"
SEN3_TOPIC = "sonarT"
SEN4_TOPIC = "CO2T"
Simulink_Topic = "bowlT"
CA_CERT = "broker.crt"     # ব্রোকারের certificate (CA বা self-signed cert)

# ---------- MQTT Callbacks ----------
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[subscriber] Connected to broker ✅ (TLS)")
        client.subscribe(ESP_TOPIC, qos=1)
        client.subscribe(ESP2_TOPIC, qos=1)
        client.subscribe(JM_TOPIC, qos=1)
        client.subscribe(ESP3_TOPIC, qos=1)
        client.subscribe(TEXT_TOPIC, qos=1)
        client.subscribe(SEN1_TOPIC, qos=1)
        client.subscribe(SEN2_TOPIC, qos=1)
        client.subscribe(SEN3_TOPIC, qos=1)
        client.subscribe(SEN4_TOPIC, qos=1)
        client.subscribe(Simulink_Topic, qos=1)
    else:
        print(f"[subscriber] Failed to connect, rc={rc}")

def on_message(client, userdata, msg):
    print(f"[subscriber] RECV {msg.topic} | payload={msg.payload.decode()} | time={datetime.datetime.now()}")

# ---------- MQTT Client Setup ----------
client = mqtt.Client(client_id="py_subscriber_001", clean_session=True)
client.username_pw_set(USERNAME, PASSWORD)

# Enable TLS
client.tls_set(
    ca_certs=CA_CERT,
    certfile=None,
    keyfile=None,
    tls_version=ssl.PROTOCOL_TLSv1_2
)
client.tls_insecure_set(True)  # self-signed হলে allow করবে

client.on_connect = on_connect
client.on_message = on_message

# Connect to TLS broker
client.connect(BROKER, PORT, keepalive=60)
client.loop_start()  # starts network loop in background

# ---------- Publisher Function ----------
def publish_loop():
    try:
        while True:
            message = input("[publisher] Enter message to send: ")
            if message.strip() == "":
                continue
            client.publish(TEXT_TOPIC, payload=message, qos=1)
            print(f"[publisher] Sent: {message}")
    except KeyboardInterrupt:
        print("\n[publisher] stopping...")

# Run publisher in separate thread so subscriber keeps working
publisher_thread = threading.Thread(target=publish_loop, daemon=True)
publisher_thread.start()

# Keep main thread alive to continue receiving messages
try:
    while True:
        import time
        time.sleep(1)
except KeyboardInterrupt:
    print("\n[main] stopping...")
finally:
    client.loop_stop()
    client.disconnect()
