// Import the functions you need from the SDKs you need
import { initializeApp } from "firebase/app";
import { getAuth } from "firebase/auth";
// TODO: Add SDKs for Firebase products that you want to use
// https://firebase.google.com/docs/web/setup#available-libraries

// Your web app's Firebase configuration
const firebaseConfig = {
  apiKey: "AIzaSyDHjnyzd1DuQ6JF21Ilv6fmELW-R3EH_cQ",
  authDomain: "iot-temp-b95e4.firebaseapp.com",
  projectId: "iot-temp-b95e4",
  storageBucket: "iot-temp-b95e4.firebasestorage.app",
  messagingSenderId: "478466774895",
  appId: "1:478466774895:web:2d77b472fba6f5c08dba9a"
};

// Initialize Firebase
// Initialize Firebase
const app = initializeApp(firebaseConfig);
const auth = getAuth(app)

export default auth;