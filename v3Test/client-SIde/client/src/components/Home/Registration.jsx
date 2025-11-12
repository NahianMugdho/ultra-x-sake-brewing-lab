import React, { useState } from "react";
import { Link, useNavigate } from "react-router-dom";
import { AiOutlineEye, AiOutlineEyeInvisible } from "react-icons/ai";

const Registration = () => {
  const [showPassword, setShowPassword] = useState(false);
  const [role, setRole] = useState("user"); // default role
  const navigate = useNavigate();

  const togglePassword = () => setShowPassword((prev) => !prev);

  const handleRegister = async (e) => {
    e.preventDefault();
    const formData = e.target;

    const firstName = formData.firstName.value;
    const lastName = formData.lastName.value;
    const fullName = `${firstName} ${lastName}`;
    const email = formData.email.value;
    const password = formData.password.value;
    const imageFile = formData.image.files[0];

    if (password.length < 6) {
      alert("Password must be at least 6 characters");
      return;
    }

    try {
      // 1️⃣ Upload image to imgbb
      const imageFormData = new FormData();
      imageFormData.append("image", imageFile);
      const imgbbApiKey = import.meta.env.VITE_Image_hosting_key;
      const imgbbUploadURL = `https://api.imgbb.com/1/upload?key=${imgbbApiKey}`;

      const imgRes = await fetch(imgbbUploadURL, {
        method: "POST",
        body: imageFormData,
      });
      const imgData = await imgRes.json();

      if (imgData.success) {
        const photoURL = imgData.data.url;

        // 2️⃣ Send data to backend
        const res = await fetch("http://192.168.88.60:5000/api/auth/register", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            fullName,
            email,
            password,
            role,      // send selected role
            photoURL,
          }),
        });

        const data = await res.json();

        if (res.ok) {
          alert("Registration successful! 🎉");
          formData.reset();
          navigate("/login"); // redirect to login after registration
        } else {
          alert(data.message || "Registration failed");
        }
      }
    } catch (error) {
      console.error("Error:", error.message);
      alert(error.message);
    }
  };

  return (
    <div className="min-h-screen bg-base-200 flex items-center justify-center px-4 py-10">
      <div className="w-full max-w-lg bg-base-100 rounded-2xl shadow-md p-8 space-y-5 text-base-content">
        <h2 className="text-3xl font-bold text-center text-primary mb-6">
          Create Your Account
        </h2>

        <form onSubmit={handleRegister} className="space-y-4">
          {/* Name fields */}
          <div className="grid grid-cols-2 gap-4">
            <input
              type="text"
              name="firstName"
              placeholder="First Name"
              className="input input-bordered w-full"
              required
            />
            <input
              type="text"
              name="lastName"
              placeholder="Last Name"
              className="input input-bordered w-full"
              required
            />
          </div>

          {/* Email */}
          <input
            type="email"
            name="email"
            placeholder="Email"
            className="input input-bordered w-full"
            required
          />

          {/* Password */}
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

          {/* Role Selection */}
          <select
            className="select select-bordered w-full"
            value={role}
            onChange={(e) => setRole(e.target.value)}
          >
            <option value="user">User</option>
            <option value="admin">Admin</option>
          </select>

          {/* Profile Photo */}
          <input
            type="file"
            name="image"
            accept="image/*"
            className="file-input file-input-bordered w-full"
            required
          />

          <button className="btn btn-primary w-full mt-4">Register</button>
          <p className="text-xl font-semibold text-center">
            Already have an account?{" "}
            <Link to={`/login`}>
              <span className="text-blue-300">Login</span>
            </Link>
          </p>
        </form>
      </div>
    </div>
  );
};

export default Registration;
