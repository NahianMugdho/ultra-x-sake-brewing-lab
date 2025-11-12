import React, { useEffect, useState } from "react";
import AuthContext from "./AuthContext";

const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [token, setToken] = useState(localStorage.getItem("token") || null);
  const [loading, setLoading] = useState(true);

  const logIn = async (jwtToken) => {
    localStorage.setItem("token", jwtToken);
    setToken(jwtToken);

    try {
      const res = await fetch("http://192.168.88.60:5000/api/auth/me", {
        headers: { Authorization: `Bearer ${jwtToken}` },
      });

      if (!res.ok) {
        throw new Error("Failed to fetch user data");
      }

      const data = await res.json();
      setUser(data);
    } catch (err) {
      console.error("Login error:", err);
      setUser(null);
    } finally {
      setLoading(false);
    }
  };

  const logOut = () => {
    localStorage.removeItem("token");
    setUser(null);
    setToken(null);
    return Promise.resolve();
  };

  // 🔑 এখানে token change হলে useEffect আবার run হবে
  useEffect(() => {
    if (token) {
      logIn(token);
    } else {
      setLoading(false);
    }
  }, [token]);

  const authInfo = { user, token, loading, logIn, logOut };

  return (
    <AuthContext.Provider value={authInfo}>
      {loading ? <div style={{ textAlign: "center", marginTop: "50px" }}>Loading...</div> : children}
    </AuthContext.Provider>
  );
};

export default AuthProvider;
