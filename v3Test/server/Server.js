// const express = require("express");
// const cors = require("cors");
// const dotenv = require("dotenv");
// const authRoutes = require("./routes/authRoutes");
// const emailRoutes = require("./routes/emailRoutes");


// dotenv.config();

// const app = express();

// app.use(cors());
// app.use(express.json());
// app.use("/api/auth", authRoutes);
// app.use("/api/email", emailRoutes);
// const PORT = process.env.PORT || 5000;
// app.listen(PORT,"0.0.0.0", () => console.log(`Server running on port ${PORT}`));
// server.js (or your main Express file)


const express = require("express");
const cors = require("cors");
const dotenv = require("dotenv");
const mysql = require("mysql2/promise");

dotenv.config();

const app = express();
app.use(cors());
app.use(express.json());

// MySQL connection pool
const db = mysql.createPool({
  host: process.env.DB_HOST || "localhost",
  user: process.env.DB_USER || "root",
  password: process.env.DB_PASS || "",
  database: process.env.DB_NAME || "iot_users",
});

// Function to capture image and save to MySQL
const captureAndSave = async () => {
  try {
    const cameraUrl = process.env.CAMERA_URL;
    const response = await fetch(cameraUrl);
    const buffer = await response.arrayBuffer();
    const filename = `snapshot_${Date.now()}.jpg`;

    await db.execute(
      "INSERT INTO snapshots (filename, image) VALUES (?, ?)",
      [filename, Buffer.from(buffer)]
    );

    console.log(`📸 Saved ${filename} from ${cameraUrl} to MySQL`);
  } catch (err) {
    console.error("❌ Error saving image:", err);
  }
};
// Manual capture route
app.get("/capture", async (req, res) => {
  try {
    await captureAndSave();
    res.json({ success: true });
  } catch (err) {
    res.status(500).json({ success: false, error: err.message });
  }
});

// Start auto-capture interval
const interval = parseInt(process.env.CAPTURE_INTERVAL || 5000, 10);
setInterval(captureAndSave, interval);


// Your existing routes
app.use("/api/auth", require("./routes/authRoutes"));
app.use("/api/email", require("./routes/emailRoutes"));

const PORT = process.env.PORT || 5000;
app.listen(PORT, "0.0.0.0", () =>
  console.log(`Server running on port ${PORT}`)
);
