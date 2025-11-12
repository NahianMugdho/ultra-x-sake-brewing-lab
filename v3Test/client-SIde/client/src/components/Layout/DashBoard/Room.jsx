
// import  { useEffect, useState } from "react";
// import { io } from "socket.io-client";

// const SOCKET_SERVER_URL = "http://192.168.88.60:4000";
// const Room = () => {
//  const [socket, setSocket] = useState(null);
//   const [mqttData, setMqttData] = useState({});
//      const [text, setText] = useState("");
  
// useEffect(() => {
//   const sock = io(SOCKET_SERVER_URL);
//   setSocket(sock);

//   sock.on("mqtt_message", ({ topic, payload, timestamp }) => {
//     setMqttData((prev) => ({
//       ...prev,
//       [topic]: { payload, timestamp },
//     }));
//   });

//   return () => sock.disconnect();
// }, []);
//    const handleSend = () => {
//     if (socket && text.trim() !== "") {
//       socket.emit("publish_mqtt", { topic: "text", message: text });
//       setText("");
//     }
//   };

//     return (
//         <div>
            

//     <div className="p-4">
//        <h2 className="text-2xl font-bold mb-4">Realtime MQTT Dashboard</h2>

//        {Object.keys(mqttData).length === 0 && <p>No data yet...</p>}

//       <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
//         {Object.entries(mqttData).map(([topic, { payload, timestamp }]) => (
//           <div
//             key={topic}
//             className="border rounded p-4 shadow bg-white"
//           >
//             <h3 className="font-semibold text-lg mb-2">Topic: {topic}</h3>
//             <p>
//               <span className="font-medium">Payload:</span> {payload}
//             </p>
//             <p>
//               <span className="font-medium">Last Updated:</span>{" "}
//               {new Date(timestamp).toLocaleString()}
//             </p>
//           </div>
//         ))}
//       </div>
//     </div>

// <div>
//       <input 
//         type="text" 
//         value={text} 
//         onChange={(e) => setText(e.target.value)} 
//         placeholder="Send MQTT message" 
//       />
//       <button onClick={handleSend}>Send</button>
//     </div>






//         </div>
//     );
// };

// export default Room;
import { useEffect, useState, useContext } from "react";
import { io } from "socket.io-client";
import AuthContext from "../../Auth/AuthContext";
import EmailRoom from "./EmailRoom";

const SOCKET_SERVER_URL = "http://192.168.88.60:4000";

const Room = () => {
  const { user } = useContext(AuthContext); // Logged-in user
  const [socket, setSocket] = useState(null);
  const [mqttData, setMqttData] = useState({});
  const [text, setText] = useState("");
  const [csvData, setCsvData] = useState([]); // CSV তে জমা

  useEffect(() => {
    const sock = io(SOCKET_SERVER_URL);
    setSocket(sock);

    sock.on("mqtt_message", ({ topic, payload, timestamp }) => {
      setMqttData((prev) => ({
        ...prev,
        [topic]: { payload, timestamp },
      }));

      // CSV data update
      setCsvData((prev) => [
        ...prev,
        {
          email: user?.email || "guest",
          topic,
          payload,
          timestamp,
        },
      ]);
    });

    return () => sock.disconnect();
  }, [user]);

  const handleSend = () => {
    if (socket && text.trim() !== "") {
      socket.emit("publish_mqtt", { topic: "text", message: text });
      setText("");
    }
  };

  // CSV ডাউনলোড হ্যান্ডলার
  const downloadCSV = () => {
    if (!csvData.length) return;

    const headers = ["email", "topic", "payload", "timestamp"];
    const rows = csvData.map((row) =>
      headers.map((field) => JSON.stringify(row[field] || "")).join(",")
    );

    const csvContent = [headers.join(","), ...rows].join("\r\n");
    const blob = new Blob([csvContent], { type: "text/csv;charset=utf-8;" });
    const url = URL.createObjectURL(blob);

    const link = document.createElement("a");
    link.href = url;
    link.setAttribute("download", `mqtt_data_${Date.now()}.csv`);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
  };

  return (
    <div className="p-4">
      <h2 className="text-2xl font-bold mb-4">Realtime MQTT Dashboard</h2>

      {Object.keys(mqttData).length === 0 && <p>No data yet...</p>}

      <div className="grid grid-cols-1 md:grid-cols-2 gap-4 mb-4">
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

      {/* Send message */}
      <div className="flex mb-4">
        <input
          type="text"
          value={text}
          onChange={(e) => setText(e.target.value)}
          placeholder="Send MQTT message"
          className="border p-2 rounded mr-2 flex-1"
        />
        <button
          onClick={handleSend}
          className="bg-blue-600 text-white px-4 py-2 rounded hover:bg-blue-700"
        >
          Send
        </button>
      </div>

      {/* Download CSV */}
      <button
        onClick={downloadCSV}
        className="bg-green-600 text-white px-4 py-2 rounded hover:bg-green-700"
      >
        Download CSV
      </button>

      <div>

{/* <EmailRoom></EmailRoom> */}

      </div>
    </div>
  );
};

export default Room;
