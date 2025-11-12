const db = require("../config/db");

const User = {
  create: (userData, callback) => {
    const { fullName, email, password, role, photoURL } = userData;
    const sql = `INSERT INTO users (fullName, email, password, role, photoURL) VALUES (?, ?, ?, ?, ?)`;
    db.query(sql, [fullName, email, password, role, photoURL], callback);
  },

  findByEmail: (email, callback) => {
    const sql = `SELECT * FROM users WHERE email = ?`;
    db.query(sql, [email], callback);
  },

  findById: (id, callback) => {
    const sql = `SELECT * FROM users WHERE id = ?`;
    db.query(sql, [id], callback);
  },

   getUserRooms: (id, callback) => {
    const sql = `SELECT room_name FROM user_rooms WHERE user_id = ?`;
    db.query(sql, [id], callback);
  },

  assignRoom: (userId, roomName, callback) => {
    const sql = `INSERT INTO user_rooms (user_id, room_name) VALUES (?, ?)`;
    db.query(sql, [userId, roomName], callback);
  },
   deleteRoomAssignment: (userId, roomName, callback) => {
    const sql = `DELETE FROM user_rooms WHERE user_id = ? AND room_name = ?`;
    db.query(sql, [userId, roomName], callback);
  }
};

module.exports = User;
