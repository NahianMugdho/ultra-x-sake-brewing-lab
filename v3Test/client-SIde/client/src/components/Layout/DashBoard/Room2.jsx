
import { useEffect, useState, useRef } from "react";
import { io } from "socket.io-client";
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, PieChart, Pie, Cell } from "recharts";

const SOCKET_SERVER_URL = "http://192.168.88.60:4000";

const COLORS = ["#8884d8", "#82ca9d", "#ffc658", "#ff8042", "#8dd1e1", "#d0ed57", "#a4de6c"];

const Room2 = () => {
  const [socket, setSocket] = useState(null);
  const [mqttData, setMqttData] = useState({});
  const [value, setValue] = useState(25);
  const [isDragging, setIsDragging] = useState(false);
  const [startAngle, setStartAngle] = useState(0);
  const [currentRotation, setCurrentRotation] = useState(0);
  const containerRef = useRef(null);

  // Connect Socket.IO
  useEffect(() => {
    const sock = io(SOCKET_SERVER_URL);
    setSocket(sock);

    sock.on("mqtt_message", ({ topic, payload, timestamp }) => {
      setMqttData(prev => ({
        ...prev,
        [topic]: { payload: Number(payload), timestamp: new Date(timestamp) }
      }));
    });

    return () => sock.disconnect();
  }, []);

  // Send MQTT message when value changes
  useEffect(() => {
    if (socket) {
      socket.emit("publish_mqtt", { topic: "text", message: value.toString() });
    }
  }, [value, socket]);

  const getCenter = () => {
    if (containerRef.current) {
      const rect = containerRef.current.getBoundingClientRect();
      return {
        x: rect.left + rect.width / 2,
        y: rect.top + rect.height / 2,
      };
    }
    return { x: 0, y: 0 };
  };

  const getAngle = (e) => {
    const center = getCenter();
    return Math.atan2(e.clientY - center.y, e.clientX - center.x) * (180 / Math.PI);
  };

  const handleMouseDown = (e) => {
    setIsDragging(true);
    setStartAngle(getAngle(e) - currentRotation);
  };

  useEffect(() => {
    const handleMouseMove = (e) => {
      if (isDragging) {
        const newAngle = getAngle(e) - startAngle;
        setCurrentRotation(newAngle);
        let normalized = ((newAngle % 360) + 360) % 360;
        const fraction = normalized / 360;
        const newValue = Math.round(25 + fraction * 15);
        setValue(newValue);
      }
    };

    const handleMouseUp = () => {
      setIsDragging(false);
    };

    if (isDragging) {
      document.addEventListener("mousemove", handleMouseMove);
      document.addEventListener("mouseup", handleMouseUp);
    }

    return () => {
      document.removeEventListener("mousemove", handleMouseMove);
      document.removeEventListener("mouseup", handleMouseUp);
    };
  }, [isDragging, startAngle]);

  // Prepare data for charts
  const lineData = Object.entries(mqttData).map(([topic, { payload, timestamp }]) => ({
    name: topic,
    value: payload,
  }));

  const pieData = Object.entries(mqttData).map(([topic, { payload }]) => ({
    name: topic,
    value: payload,
  }));

  return (
    <div className="p-6 bg-gray-100 min-h-screen">
      <h1 className="text-3xl font-bold mb-6">Realtime MQTT Dashboard</h1>

      {/* Cards */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mb-6">
        {Object.entries(mqttData).map(([topic, { payload }], index) => (
          <div key={topic} className="bg-white rounded shadow p-4 flex flex-col items-center">
            <h2 className="font-semibold text-lg mb-2">{topic}</h2>
            <p className="text-2xl font-bold">{payload}</p>
          </div>
        ))}
      </div>

      {/* Charts */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mb-6">
        <div className="bg-white p-4 rounded shadow">
          <h2 className="font-semibold mb-2">Line Chart (Topics)</h2>
          <LineChart width={400} height={250} data={lineData}>
            <CartesianGrid strokeDasharray="3 3" />
            <XAxis dataKey="name" />
            <YAxis />
            <Tooltip />
            <Legend />
            <Line type="monotone" dataKey="value" stroke="#8884d8" activeDot={{ r: 8 }} />
          </LineChart>
        </div>

        <div className="bg-white p-4 rounded shadow flex flex-col items-center">
          <h2 className="font-semibold mb-2">Pie Chart (Topics)</h2>
          <PieChart width={400} height={250}>
            <Pie
              data={pieData}
              dataKey="value"
              nameKey="name"
              cx="50%"
              cy="50%"
              outerRadius={80}
              fill="#8884d8"
              label
            >
              {pieData.map((entry, index) => (
                <Cell key={`cell-${index}`} fill={COLORS[index % COLORS.length]} />
              ))}
            </Pie>
          </PieChart>
        </div>
      </div>

      {/* Publish MQTT */}
      <div className="bg-white p-4 rounded shadow flex items-center gap-2">
        <div
          ref={containerRef}
          onMouseDown={handleMouseDown}
          className="flex items-center justify-center cursor-grab"
          style={{
            position: "relative",
            width: "100px",
            height: "100px",
            borderRadius: "50%",
            background: "lightblue",
            userSelect: "none",
          }}
          title="Drag circularly to adjust value between 25 and 40"
        >
          <div
            style={{
              position: "absolute",
              width: "2px",
              height: "40px",
              background: "red",
              top: "5px",
              left: "49px",
              transformOrigin: "bottom",
              transform: `rotate(${currentRotation}deg)`,
            }}
          />
          <div
            style={{
              position: "absolute",
              top: "50%",
              left: "50%",
              transform: "translate(-50%, -50%)",
            }}
          >
            {value}
          </div>
        </div>
      </div>
    </div>
  );
};

export default Room2;