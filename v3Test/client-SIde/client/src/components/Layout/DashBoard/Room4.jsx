
import  { useEffect, useState } from "react";
import { io } from "socket.io-client";

const SOCKET_SERVER_URL = "http://192.168.88.60:4000";
const Room = () => {
 const [socket, setSocket] = useState(null);
  const [mqttData, setMqttData] = useState({});
     const [text, setText] = useState("");
  
useEffect(() => {
  const sock = io(SOCKET_SERVER_URL);
  setSocket(sock);

 // Subscribe only ESPX & ESPY data
    sock.on("mqtt_message", ({ topic, payload, timestamp }) => {
      if (topic === "ESPX" || topic === "voltX") {
        setMqttData((prev) => ({
          ...prev,
          [topic]: { payload, timestamp },
        }));
      }
    });

    return () => sock.disconnect();
  }, []);
   const handleSend = () => {
    if (socket && text.trim() !== "") {
      socket.emit("publish_mqtt", { topic: "text", message: text });
      setText("");
    }
  };

    return (
        <div>
            

    <div className="p-4">
       <h2 className="text-2xl font-bold mb-4">Realtime MQTT Dashboard</h2>

       {Object.keys(mqttData).length === 0 && <p>No data yet...</p>}

      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        {Object.entries(mqttData).map(([topic, { payload, timestamp }]) => (
          <div
            key={topic}
            className="border rounded p-4 shadow bg-white"
          >
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

<div>
      <input 
        type="text" 
        value={text} 
        onChange={(e) => setText(e.target.value)} 
        placeholder="Send MQTT message" 
      />
      <button onClick={handleSend}>Send</button>
    </div>






        </div>
    );
};

export default Room;