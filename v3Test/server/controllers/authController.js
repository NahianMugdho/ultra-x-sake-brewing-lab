const bcrypt = require("bcryptjs");
const jwt = require("jsonwebtoken");
const User = require("../models/userModel");
const dotenv = require("dotenv");
dotenv.config();

// ======================= REGISTER =======================
const register = async (req, res) => {
  const { fullName, email, password, role, photoURL } = req.body;

  if (!fullName || !email || !password) {
    return res.status(400).json({ message: "All fields are required" });
  }

  User.findByEmail(email, async (err, results) => {
    if (err) return res.status(500).json(err);
    if (results.length > 0) {
      return res.status(400).json({ message: "Email already exists" });
    }

    const hashedPassword = await bcrypt.hash(password, 10);
    const userData = { fullName, email, password: hashedPassword, role, photoURL };

    User.create(userData, (err, result) => {
      if (err) return res.status(500).json(err);
      res
        .status(201)
        .json({ message: "User registered successfully", userId: result.insertId });
    });
  });
};

// ======================= LOGIN =======================
const login = (req, res) => {
  const { email, password } = req.body;

  User.findByEmail(email, async (err, results) => {
    if (err) return res.status(500).json(err);
    if (results.length === 0) {
      return res.status(404).json({ message: "User not found" });
    }

    const user = results[0];
    const isMatch = await bcrypt.compare(password, user.password);
    if (!isMatch) {
      return res.status(400).json({ message: "Invalid password" });
    }

    // রুমগুলো বের করা
    User.getUserRooms(user.id, (err, roomResults) => {
      if (err) return res.status(500).json(err);

      const userRooms = roomResults.map(r => r.room_name);

      const token = jwt.sign(
        { id: user.id, role: user.role },
        process.env.JWT_SECRET,
        { expiresIn: "1d" }
      );

      res.json({
        token,
        user: {
          id: user.id,
          fullName: user.fullName,
          email: user.email,
          role: user.role,
          photoURL: user.photoURL,
          rooms: userRooms
        }
      });
    });
  });
};

// ======================= GET CURRENT USER =======================
const getMe = (req, res) => {
  const userId = req.user.id;

  User.findById(userId, (err, results) => {
    if (err) return res.status(500).json(err);
    if (results.length === 0) {
      return res.status(404).json({ message: "User not found" });
    }

    const user = results[0];
    User.getUserRooms(userId, (err, roomResults) => {
      if (err) return res.status(500).json(err);

      const userRooms = roomResults.map(r => r.room_name);

      res.json({
        id: user.id,
        fullName: user.fullName,
        email: user.email,
        role: user.role,
        photoURL: user.photoURL,
        rooms: userRooms
      });
    });
  });
};

module.exports = {
  register,
  login,
  getMe
};
