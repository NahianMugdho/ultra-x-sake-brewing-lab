

import React, { useEffect, useState } from "react";
import { io } from "socket.io-client";

const SOCKET_SERVER_URL = "http://192.168.88.60:4000";

const EmailRoom = () => {
  const [socket, setSocket] = useState(null);
  const [mqttData, setMqttData] = useState({});
  const [alertMessage, setAlertMessage] = useState(""); // message to show when email is sent

  useEffect(() => {
    const sock = io(SOCKET_SERVER_URL);
    setSocket(sock);

    sock.on("mqtt_message", ({ topic, payload, timestamp }) => {
      setMqttData((prev) => ({
        ...prev,
        [topic]: { payload, timestamp },
      }));

      // Trigger email if topic = 'ESP3' and payload = '50'
      if (topic === "ESP3" && payload === "50") {
        fetch("http://192.168.88.60:5000/api/email/send-email", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            to: "mugdho.ux@gmail.com",
            subject: "Alert: ESP3 value is 50",
            text: `The MQTT topic ESP3 has reached ${payload}.`,
          }),
        })
          .then((res) => res.json())
          .then((data) => {
            console.log(data);
            setAlertMessage(`Alert! Email sent because ESP3 value is ${payload}`);
          })
          .catch((err) => {
            console.error(err);
            setAlertMessage("Failed to send email.");
          });
      }
    });

    return () => sock.disconnect();
  }, []);

  return (
    <div className="p-4">
      <h2 className="text-2xl font-bold mb-4">MQTT Email Alert</h2>

      {alertMessage && (
        <div className="bg-yellow-100 text-yellow-800 p-3 mb-4 rounded">
          {alertMessage}
        </div>
      )}

      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        {Object.entries(mqttData).map(([topic, { payload, timestamp }]) => (
          <div key={topic} className="border rounded p-4 shadow bg-white">
            <h3 className="font-semibold text-lg mb-2">Topic: {topic}</h3>
            <p>
              <span className="font-medium">Payload:</span> {payload}
            </p>
            <p>
              <span className="font-medium">Last Updated:</span>{" "}
              {new Date(timestamp).toLocaleString()}
            </p>
          </div>
        ))}
      </div>
    </div>
  );
};

export default EmailRoom;
