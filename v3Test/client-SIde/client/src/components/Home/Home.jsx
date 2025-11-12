import React, { useEffect, useRef, useState } from 'react';
import Room from '../Layout/DashBoard/Room';
import Room2 from '../Layout/DashBoard/Room2';
import Room3 from '../Layout/DashBoard/Room3';

const Home = () => {
  const [autoSave, setAutoSave] = useState(false);
  const intervalRef = useRef(null);

  // This function calls your backend (Flask or Node.js)
  // const captureImageToServer = async () => {
  //   try {
  //     const response = await fetch("http://localhost:5000/capture");
  //     if (!response.ok) throw new Error("Failed to capture image");
  //     console.log("📸 Image captured and saved on backend");
  //   } catch (error) {
  //     console.error("Error:", error);
  //   }
  // };
  
const captureImageToServer = async () => {
  try {
    await fetch("http://localhost:5000/capture");
    console.log("Image saved to MySQL!");
  } catch (err) {
    console.error("Error saving image:", err);
  }
};

  // Automatically capture every few seconds when autoSave = true
  useEffect(() => {
    if (autoSave) {
      intervalRef.current = setInterval(() => {
        captureImageToServer();
      }, 5000); // every 5 seconds
    } else if (intervalRef.current) {
      clearInterval(intervalRef.current);
    }
    return () => clearInterval(intervalRef.current);
  }, [autoSave]);

  return (
    <div>
      <h1>Home</h1>

      <div>
        <h2>Live Camera Feed</h2>
        <iframe
          src="http://192.168.88.20:8080/jsfs.html"
          width="800"
          height="600"
          title="IP Camera"
          style={{ border: "none" }}
        />

        <div style={{ marginTop: '10px' }}>
          <button
            onClick={() => captureImageToServer()}
            style={{
              marginRight: '10px',
              padding: '8px 16px',
              borderRadius: '8px',
              backgroundColor: '#007bff',
              color: 'white',
              border: 'none',
              cursor: 'pointer',
            }}
          >
            Capture Now
          </button>

          <button
            onClick={() => setAutoSave(!autoSave)}
            style={{
              padding: '8px 16px',
              borderRadius: '8px',
              backgroundColor: autoSave ? '#dc3545' : '#28a745',
              color: 'white',
              border: 'none',
              cursor: 'pointer',
            }}
          >
            {autoSave ? 'Stop Auto Save' : 'Start Auto Save'}
          </button>
        </div>
      </div>

      <div><Room /></div>
      <div><Room2 /></div>
      <div><Room3 /></div>
    </div>
  );
};

export default Home;
