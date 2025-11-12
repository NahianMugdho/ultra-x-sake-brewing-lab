// import React, { useEffect, useState, useContext } from "react";
// import AuthContext from "../../Auth/AuthContext";


// const Admin = () => {
//   const { user } = useContext(AuthContext); // লগইন করা ইউজার (admin হতে হবে)
//   const [users, setUsers] = useState([]);
//   const [selectedUser, setSelectedUser] = useState("");
//   const [selectedRoom, setSelectedRoom] = useState("");

//   // সব ইউজার লোড করা
//   useEffect(() => {
//     fetch("http://192.168.88.60:5000/api/auth/users", {
//       headers: {
//         Authorization: `Bearer ${localStorage.getItem("token")}`,
//       },
//     })
//       .then((res) => res.json())
//       .then((data) => setUsers(data))
//       .catch((err) => console.error(err));
//   }, []);

//   // রুম অ্যাসাইন করা
//   const handleAssign = () => {
//     if (!selectedUser || !selectedRoom) {
//       alert("Please select user and room!");
//       return;
//     }

//     fetch("http://192.168.88.60:5000/api/auth/assign-room", {
//       method: "POST",
//       headers: {
//         "Content-Type": "application/json",
//         Authorization: `Bearer ${localStorage.getItem("token")}`,
//       },
//       body: JSON.stringify({ userId: selectedUser, roomName: selectedRoom }),
//     })
//       .then((res) => res.json())
//       .then((data) => {
//         alert(data.message);
//         setSelectedUser("");
//         setSelectedRoom("");
//       })
//       .catch((err) => console.error(err));
//   };

//   // ফ্রন্টএন্ড UI
//   return (
//     <div className="p-6 max-w-lg mx-auto bg-white shadow-md rounded">
//       <h2 className="text-xl font-bold mb-4">Admin Panel - Assign Rooms</h2>

//       {/* ইউজার সিলেক্ট */}
//       <label className="block mb-2">Select User</label>
//       <select
//         value={selectedUser}
//         onChange={(e) => setSelectedUser(e.target.value)}
//         className="w-full p-2 border rounded mb-4"
//       >
//         <option value="">-- Select User --</option>
//         {users.map((u) => (
//           <option key={u.id} value={u.id}>
//             {u.fullName} ({u.email})
//           </option>
//         ))}
//       </select>

//       {/* রুম সিলেক্ট */}
//       <label className="block mb-2">Select Room</label>
//       <select
//         value={selectedRoom}
//         onChange={(e) => setSelectedRoom(e.target.value)}
//         className="w-full p-2 border rounded mb-4"
//       >
//         <option value="">-- Select Room --</option>
//         <option value="room">Room</option>
//         <option value="room2">Room2</option>
//         <option value="room3">Room3</option>
//         <option value="room4">Room4</option>
//       </select>

//       <button
//         onClick={handleAssign}
//         className="bg-blue-600 text-white px-4 py-2 rounded hover:bg-blue-700"
//       >
//         Assign Room
//       </button>
//     </div>
//   );
// };

// export default Admin;
import React, { useEffect, useState, useContext } from "react";
import AuthContext from "../../Auth/AuthContext";

const Admin = () => {
  const { user } = useContext(AuthContext); // Logged-in user (must be admin)
  const [users, setUsers] = useState([]);
  const [selectedUser, setSelectedUser] = useState("");
  const [selectedRoom, setSelectedRoom] = useState("");
  const [assignedRooms, setAssignedRooms] = useState([]); // To store rooms assigned to the selected user

  // Load all users
  useEffect(() => {
    fetch("http://192.168.88.60:5000/api/auth/users", {
      headers: {
        Authorization: `Bearer ${localStorage.getItem("token")}`,
      },
    })
      .then((res) => res.json())
      .then((data) => setUsers(data))
      .catch((err) => console.error("Error loading users:", err));
  }, []);

  // Load assigned rooms for the selected user
  useEffect(() => {
    if (selectedUser) {
      fetch(`http://192.168.88.60:5000/api/auth/users/${selectedUser}/rooms`, {
        headers: {
          Authorization: `Bearer ${localStorage.getItem("token")}`,
        },
      })
        .then((res) => {
          if (!res.ok) {
            throw new Error(`HTTP error! status: ${res.status}`);
          }
          return res.json();
        })
        .then((data) => setAssignedRooms(data.rooms || []))
        .catch((err) => console.error("Error loading assigned rooms:", err));
    } else {
      setAssignedRooms([]);
    }
  }, [selectedUser]);

  // Assign room
  const handleAssign = () => {
    if (!selectedUser || !selectedRoom) {
      alert("Please select user and room!");
      return;
    }

    fetch("http://192.168.88.60:5000/api/auth/assign-room", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        Authorization: `Bearer ${localStorage.getItem("token")}`,
      },
      body: JSON.stringify({ userId: selectedUser, roomName: selectedRoom }),
    })
      .then((res) => {
        if (!res.ok) {
          throw new Error(`HTTP error! status: ${res.status}`);
        }
        return res.json();
      })
      .then((data) => {
        alert(data.message);
        setSelectedRoom("");
        // Refresh assigned rooms
        fetch(`http://192.168.88.60:5000/api/auth/users/${selectedUser}/rooms`, {
          headers: {
            Authorization: `Bearer ${localStorage.getItem("token")}`,
          },
        })
          .then((res) => res.json())
          .then((data) => setAssignedRooms(data.rooms || []))
          .catch((err) => console.error("Error refreshing rooms:", err));
      })
      .catch((err) => console.error("Error assigning room:", err));
  };

  // Delete room assignment
  const handleDelete = (roomName) => {
    if (!selectedUser || !roomName) {
      alert("Please select a user and room to delete!");
      return;
    }

    fetch("http://192.168.88.60:5000/api/auth/remove-room", {
      method: "DELETE",
      headers: {
        "Content-Type": "application/json",
        Authorization: `Bearer ${localStorage.getItem("token")}`,
      },
      body: JSON.stringify({ userId: selectedUser, roomName }),
    })
      .then((res) => {
        if (!res.ok) {
          throw new Error(`HTTP error! status: ${res.status}`);
        }
        return res.json();
      })
      .then((data) => {
        alert(data.message);
        // Refresh assigned rooms
        fetch(`http://192.168.88.60:5000/api/auth/users/${selectedUser}/rooms`, {
          headers: {
            Authorization: `Bearer ${localStorage.getItem("token")}`,
          },
        })
          .then((res) => res.json())
          .then((data) => setAssignedRooms(data.rooms || []))
          .catch((err) => console.error("Error refreshing rooms:", err));
      })
      .catch((err) => console.error("Error deleting room:", err));
  };

  // Frontend UI
  return (
    <div className="p-6 max-w-lg mx-auto bg-white shadow-md rounded">
      <h2 className="text-xl font-bold mb-4">Admin Panel - Assign Rooms</h2>

      {/* Select User */}
      <label className="block mb-2">Select User</label>
      <select
        value={selectedUser}
        onChange={(e) => setSelectedUser(e.target.value)}
        className="w-full p-2 border rounded mb-4"
      >
        <option value="">-- Select User --</option>
        {users.map((u) => (
          <option key={u.id} value={u.id}>
            {u.fullName} ({u.email})
          </option>
        ))}
      </select>

      {/* Select Room to Assign */}
      <label className="block mb-2">Select Room to Assign</label>
      <select
        value={selectedRoom}
        onChange={(e) => setSelectedRoom(e.target.value)}
        className="w-full p-2 border rounded mb-4"
      >
        <option value="">-- Select Room --</option>
        <option value="room">Room</option>
        <option value="room2">Room2</option>
        <option value="room3">Room3</option>
        <option value="room4">Room4</option>
      </select>

      <button
        onClick={handleAssign}
        className="bg-blue-600 text-white px-4 py-2 rounded hover:bg-blue-700 mb-4"
      >
        Assign Room
      </button>

      {/* Assigned Rooms */}
      <label className="block mb-2">Assigned Rooms</label>
      <ul className="mb-4">
        {assignedRooms.length > 0 ? (
          assignedRooms.map((room) => (
            <li key={room} className="flex justify-between items-center mb-2">
              <span>{room}</span>
              <button
                onClick={() => handleDelete(room)}
                className="bg-red-600 text-white px-2 py-1 rounded hover:bg-red-700"
              >
                Delete
              </button>
            </li>
          ))
        ) : (
          <li>No rooms assigned to this user.</li>
        )}
      </ul>
    </div>
  );
};

export default Admin;