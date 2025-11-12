import { io } from "socket.io-client";

// Change the URL to your backend server
const socket = io("http://192.168.88.60:4000", {
  transports: ["websocket"], // ensures WebSocket connection
});

export default socket;
