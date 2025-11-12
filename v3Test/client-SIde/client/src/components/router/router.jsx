import React from 'react';
import { createBrowserRouter,RouterProvider } from 'react-router-dom';
import Layout from '../Layout/Layout';
import Home from '../Home/Home';
import Dashboard from '../Layout/DashBoard/Dashboard';
import PrivateRoute from './PrivateRoute';
import Login from '../Home/Login';
import Registration from '../Home/Registration';
import Room from '../Layout/DashBoard/Room';
import Room2 from '../Layout/DashBoard/Room2';
import Room3 from '../Layout/DashBoard/Room3';
import Room4 from '../Layout/DashBoard/Room4';
import Admin from '../Layout/DashBoard/Admin';

const router = createBrowserRouter([
    {
        path: '/',  
        element: <Layout></Layout>,
        errorElement: <h1>Page Not Found</h1>,
        children: [
            {
                path: '/',
                element: <Home></Home>,

            } ,
             {
    path: "dashboard",
    element:
(<PrivateRoute> <Dashboard></Dashboard></PrivateRoute>)


     ,
  },

    {
    path: "room",
    element:

<Room></Room>

   ,
  },

    {
    path: "room2",
    element:

<Room2></Room2>

   ,
  },

    {
    path: "room3",
    element:

<Room3></Room3>

   ,
  },


   {
    path: "room4",
    element:

<Room4></Room4>

   ,
  },








         {
    path: "login",
    element:

<Login></Login>

   ,
  },

      {
    path: "registration",
    element:

<Registration></Registration>

   ,
  },
    {
    path: "admin",
    element:

<Admin></Admin>

   ,
  },


    
        ],


    },
 
 
]);

export default router
