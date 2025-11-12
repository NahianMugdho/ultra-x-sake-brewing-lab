import cv2
import numpy as np
from ultralytics import YOLO
import csv
import os
import time
from datetime import datetime
import paho.mqtt.client as mqtt

# ============================================================
# CONFIGURATION
# ============================================================
IMAGE_DIR = r"D:\mugdho\ultra-X sake brewing lab\v3Test\client-SIde\client\images"
CSV_FILE = "detection_log.csv"
CONF_THRESHOLD = 0.2
CHECK_INTERVAL = 5  # seconds

# MQTT BROKER CONFIGURATION
BROKER = "localhost"
PORT = 8883
USERNAME = "admin"
PASSWORD = "StrongPassword123"
CA_CERT = "broker.crt"
TOPIC = "lab/detections"

# ============================================================
# YOLO + MQTT INITIALIZATION
# ============================================================
model = YOLO("yolov8n.pt")

client = mqtt.Client()
client.username_pw_set(USERNAME, PASSWORD)
client.tls_set(ca_certs=CA_CERT)
client.connect(BROKER, PORT)
client.loop_start()

# Prepare CSV if missing
if not os.path.exists(CSV_FILE):
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Image", "Class", "Confidence"])


def process_image(image_path):
    """Run YOLO detection on a single image and send results."""
    try:
        image = cv2.imread(image_path)
        if image is None:
            return
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = model(image_rgb)[0]

        boxes = results.boxes
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        detections = []
        for box in boxes:
            conf = float(box.conf[0])
            if conf >= CONF_THRESHOLD:
                class_id = int(box.cls[0])
                class_name = results.names[class_id]
                detections.append([timestamp, os.path.basename(image_path), class_name, round(conf, 3)])

                # Publish to MQTT broker
                message = f"{class_name} detected in lab at {timestamp}"
                client.publish(TOPIC, message)
                print(f"📡 Sent MQTT: {message}")

        # Log to CSV
        if detections:
            with open(CSV_FILE, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerows(detections)
            print(f"✅ Logged {len(detections)} detections from {os.path.basename(image_path)}")

    except Exception as e:
        print(f"❌ Error processing {image_path}: {e}")


def auto_check_folder():
    """Continuously monitor folder and process only new images."""
    # 👉 স্টার্টের সময় যত ফাইল আছে, সব স্কিপ করবে
    processed = set(os.listdir(IMAGE_DIR))
    print(f"🚀 Ignoring {len(processed)} existing files. Watching for new ones...")

    while True:
        all_images = [f for f in os.listdir(IMAGE_DIR) if f.lower().endswith(".jpg")]
        new_images = [f for f in all_images if f not in processed]

        for img in new_images:
            img_path = os.path.join(IMAGE_DIR, img)
            process_image(img_path)
            processed.add(img)

        time.sleep(CHECK_INTERVAL)


if __name__ == "__main__":
    print("🔍 Auto detection + MQTT started... watching 'images/' folder.")
    auto_check_folder()
