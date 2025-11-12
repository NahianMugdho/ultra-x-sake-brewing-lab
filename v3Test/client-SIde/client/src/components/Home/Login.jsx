import React, { useContext, useState } from "react";
import AuthContext from "../Auth/AuthContext";
import { useLocation, useNavigate, Link } from "react-router-dom";
import { AiOutlineEye, AiOutlineEyeInvisible } from "react-icons/ai";

const Login = () => {
  const { logIn } = useContext(AuthContext); // logIn will store token & user
  const location = useLocation();
  const navigate = useNavigate();
  const from = location.state?.from || "/";

  const [showPassword, setShowPassword] = useState(false);
  const togglePassword = () => setShowPassword((prev) => !prev);


const handleLogIn = async (e) => {
  e.preventDefault();
  const email = e.target.email.value;
  const password = e.target.password.value;

  try {
    const res = await fetch("http://192.168.88.60:5000/api/auth/login", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ email, password }),
    });

    const data = await res.json();

    if (!res.ok) throw new Error(data.message || "Login failed");

    await logIn(data.token);

    // Redirect based on role
    if (data.role === "admin") navigate("/dashboard");
    else navigate("/room3");
  } catch (error) {
    alert(error.message);
  }
};

  return (
    <div className="min-h-screen bg-base-200 flex items-center justify-center px-4 py-10">
      <div className="w-full max-w-4xl grid grid-cols-1 lg:grid-cols-2 gap-10 bg-base-100 rounded-2xl shadow-md p-8">
        {/* Left Section */}
        <div className="flex flex-col justify-center items-center text-center space-y-4">
          <h1 className="text-4xl font-bold text-primary">Welcome Back!</h1>
          <p className="text-base-content">Please log in to continue</p>
          <img
            src="https://i.ibb.co.com/dgY6rcV/computer.png"
            alt="Login Illustration"
            className="max-w-sm w-full"
          />
        </div>

        {/* Right Section */}
        <div className="flex items-center">
          <form className="w-full space-y-4" onSubmit={handleLogIn}>
            <h2 className="text-2xl font-semibold text-center mb-4 text-base-content">
              Login to your account
            </h2>

            <div>
              <label className="label">
                <span className="label-text">Email</span>
              </label>
              <input
                type="email"
                name="email"
                placeholder="Email"
                className="input input-bordered w-full"
                required
              />
            </div>

            <div>
              <label className="label">
                <span className="label-text">Password</span>
              </label>
              <div className="relative">
                <input
                  type={showPassword ? "text" : "password"}
                  name="password"
                  placeholder="Password"
                  className="input input-bordered w-full pr-10"
                  required
                />
                <div
                  className="absolute right-3 top-1/2 -translate-y-1/2 text-xl cursor-pointer text-base-content"
                  onClick={togglePassword}
                >
                  {showPassword ? <AiOutlineEyeInvisible /> : <AiOutlineEye />}
                </div>
              </div>
              <label className="label">
                <Link to="/password-reset" className="link link-hover">
                  Forget Password?
                </Link>
              </label>
            </div>

            <div className="flex justify-center">
              <button className="btn btn-primary w-full max-w-xs mt-4">
                Login
              </button>
            </div>

            <p className="text-xl font-semibold text-center">
              Don't have an account?{" "}
              <Link to={`/registration`}>
                <span className="text-blue-300">Register</span>
              </Link>
            </p>
          </form>
        </div>
      </div>
    </div>
  );
};

export default Login;
