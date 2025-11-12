import React, { useContext } from 'react';
import { Navigate, useLocation } from 'react-router-dom';
import AuthContext from '../Auth/AuthContext';


const PrivateRoute = ({ children }) => {
  const { user, loading } = useContext(AuthContext);
  const location = useLocation();

  // যখন Firebase এখনও ইউজার স্টেট ধরে নাই
  if (loading) {
    return <div className="text-center mt-10">Loading...</div>; // অথবা Spinner
  }

  // ইউজার লগইন করা থাকলে
  if (user && user.email) {
    return children;
  }

  // ইউজার না থাকলে লগইন পেজে রিডাইরেক্ট
  return <Navigate to="/login" state={{ from: location }} replace />;
};

export default PrivateRoute;
