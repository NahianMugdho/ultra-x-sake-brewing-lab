
//.........................................................................................

// import React, { useContext } from "react";
// import { Link, useNavigate } from "react-router-dom";
// import AuthContext from "../Auth/AuthContext";

// const Navbar = () => {
//   const { user, logOut } = useContext(AuthContext);
//   const navigate = useNavigate();

//   const handleLogOut = () => {
//     logOut()
//       .then(() => {
//         console.log("Sign-out successful");
//         navigate("/");
//       })
//       .catch((error) => console.error("Sign-out error:", error));
//   };

//   return (
//     <div className="mb-4 text-center flex justify-between items-center px-4">
//       <div className="flex items-center">
//         <Link to="/" className="mx-2">Home</Link>

//         {!user && (
//           <>
//             <Link to="/login" className="mx-2">Login</Link>
//             <Link to="/registration" className="mx-2">Registration</Link>
//           </>
//         )}

//        {user?.role === "user" && (
//   <>
//     {user.rooms?.includes("room3") && (
//       <Link to="/room3" className="mx-2">Room3</Link>
//     )}
//     {user.rooms?.includes("room4") && (
//       <Link to="/room4" className="mx-2">Room4</Link>
//     )}
//     <button onClick={handleLogOut} className="mx-2 btn btn-ghost">Logout</button>
//   </>
// )}

// {user?.role === "admin" && (
//   <>
//     <Link to="/dashboard" className="mx-2">Dashboard</Link>
//     <Link to="/admin" className="mx-2">Admin Dashboard</Link>
//     {user.rooms?.includes("room") && <Link to="/room" className="mx-2">Room</Link>}
//     {user.rooms?.includes("room2") && <Link to="/room2" className="mx-2">Room2</Link>}
//     {user.rooms?.includes("room3") && <Link to="/room3" className="mx-2">Room3</Link>}
//     <button onClick={handleLogOut} className="mx-2 btn btn-ghost">Logout</button>
//   </>
// )}
//       </div>

//       {/* Right side - User avatar */}
//       {user && (
//         <div className="flex items-center">
//           {user.photoURL && (
//             <img
//               src={user.photoURL}
//               alt="User Avatar"
//               className="w-10 h-10 rounded-full border-2 border-gray-300 mr-2"
//             />
//           )}
//           <span className="font-medium">{user.fullName}</span>
//         </div>
//       )}
//     </div>
//   );
// };

// export default Navbar;




//.........................................................................................

import React, { useContext } from "react";
import { Link, useNavigate } from "react-router-dom";
import AuthContext from "../Auth/AuthContext";

const Navbar = () => {
  const { user, logOut } = useContext(AuthContext);
  const navigate = useNavigate();

  const handleLogOut = () => {
    logOut()
      .then(() => {
        navigate("/");
      })
      .catch((error) => console.error("Sign-out error:", error));
  };

  // Helper: generate room links dynamically
  const renderRoomLinks = () => {
    if (!user?.rooms) return null;

    return user.rooms.map((room) => {
      switch (room) {
        case "room":
          return <Link key={room} to="/room" className="mx-2">Room</Link>;
        case "room2":
          return <Link key={room} to="/room2" className="mx-2">Room2</Link>;
        case "room3":
          return <Link key={room} to="/room3" className="mx-2">Room3</Link>;
        case "room4":
          return <Link key={room} to="/room4" className="mx-2">Room4</Link>;
        default:
          return null;
      }
    });
  };

  return (
    <div className="mb-4 text-center flex justify-between items-center px-4">
      <div className="flex items-center">
        <Link to="/" className="mx-2">Home</Link>

        {/* Guest */}
        {!user && (
          <>
            <Link to="/login" className="mx-2">Login</Link>
            <Link to="/registration" className="mx-2">Registration</Link>
          </>
        )}

        {/* User */}
        {user?.role === "user" && (
          <>
            {renderRoomLinks()}
            <button onClick={handleLogOut} className="mx-2 btn btn-ghost">Logout</button>
          </>
        )}

        {/* Admin */}
        {user?.role === "admin" && (
          <>
            <Link to="/dashboard" className="mx-2">Dashboard</Link>
            <Link to="/admin" className="mx-2">Admin Dashboard</Link>
            {renderRoomLinks()}
            <button onClick={handleLogOut} className="mx-2 btn btn-ghost">Logout</button>
          </>
        )}
      </div>

      {/* Right side - User avatar */}
      {user && (
        <div className="flex items-center">
          {user.photoURL && (
            <img
              src={user.photoURL}
              alt="User Avatar"
              className="w-10 h-10 rounded-full border-2 border-gray-300 mr-2"
            />
          )}
          <span className="font-medium">{user.fullName}</span>
        </div>
      )}
    </div>
  );
};

export default Navbar;
