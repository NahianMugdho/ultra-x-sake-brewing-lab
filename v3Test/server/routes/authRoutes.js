const express = require("express");
const router = express.Router();
const User = require("../models/userModel"); // 👈 এটা মিসিং ছিল
const { register, login, getMe } = require("../controllers/authController");
const auth = require("../middlewares/authMiddleware"); // ✅ এটা যোগ করো
router.post("/register", register);
router.post("/login", login);
router.get("/me", auth(), getMe); // 👈 নতুন route


// 🔹 সব ইউজার লিস্ট (শুধু এডমিনের জন্য)
router.get("/users", auth("admin"), (req, res) => {
  const sql = `SELECT id, fullName, email FROM users`;
  require("../config/db").query(sql, (err, results) => {
    if (err) return res.status(500).json(err);
    res.json(results);
  });
});





// ✅ এডমিন ইউজারকে রুম অ্যাসাইন করবে
router.post("/assign-room", auth("admin"), (req, res) => {
  const { userId, roomName } = req.body;
  if (!userId || !roomName) {
    return res.status(400).json({ message: "UserId and RoomName required" });
  }

  User.assignRoom(userId, roomName, (err, result) => {
    if (err) return res.status(500).json(err);
    res.json({ message: "Room assigned successfully" });
  });
});

// Admin removes room assignment
router.delete("/remove-room", auth("admin"), (req, res) => {
  const { userId, roomName } = req.body;
  if (!userId || !roomName) {
    return res.status(400).json({ message: "UserId and RoomName required" });
  }

  User.deleteRoomAssignment(userId, roomName, (err, result) => {
    if (err) return res.status(500).json(err);
    if (result.affectedRows === 0) {
      return res.status(404).json({ message: "Room assignment not found" });
    }
    res.json({ message: "Room assignment removed successfully" });
  });
});
// Get assigned rooms for a user (admin only)
router.get("/users/:userId/rooms", auth("admin"), (req, res) => {
  const { userId } = req.params;
  if (!userId) return res.status(400).json({ message: "UserId required" });

  User.getUserRooms(userId, (err, results) => {
    if (err) return res.status(500).json(err);
    const rooms = results.map(r => r.room_name);
    res.json({ rooms });
  });
});


module.exports = router;
