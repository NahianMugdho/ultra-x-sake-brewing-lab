import React, { useEffect, useState } from "react";
import { io } from "socket.io-client";

const SOCKET_SERVER_URL = "http://192.168.88.60:4000"; // backend ব্রিজ যে পোর্টে চলছে

export default function Dashboard() {
  const [data, setData] = useState([]);

  useEffect(() => {
    // Socket.IO connect
    const socket = io(SOCKET_SERVER_URL);

    // প্রথমে API থেকে ডাটা ফেচ (যদি route থাকে)
    const fetchData = async () => {
      try {
        const res = await fetch(`${SOCKET_SERVER_URL}/temperature/latest`);
        if (res.ok) {
          const json = await res.json();
          setData(Array.isArray(json) ? json : [json]);
        }
      } catch (err) {
        console.error("Fetch error:", err);
      }
    };

    fetchData();

    // নতুন ডাটা এলে সব রাখবে
    socket.on("mqtt_message", (msg) => {
      setData((prev) => [...prev, msg]); // পুরনো + নতুন
    });

    return () => {
      socket.disconnect();
    };
  }, []);

  return (
    <div className="flex flex-col items-center">
      <h2>MQTT Data (Realtime)</h2>
      <ul>
        {data.map((item, i) => (
          <li key={i}>
            {JSON.stringify(item)}
          </li>
        ))}
      </ul>
    </div>
  );
}
