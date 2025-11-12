// server/controllers/emailController.js
const nodemailer = require("nodemailer");
const dotenv = require("dotenv");
dotenv.config();

const sendEmail = async (req, res) => {
  const { to, subject, text } = req.body;

  try {
    // Create transporter (example using Gmail SMTP)
    const transporter = nodemailer.createTransport({
      service: "gmail",
      auth: {
        user: process.env.EMAIL_USER, // your email
        pass: process.env.EMAIL_PASS, // app password
      },
    });

    const info = await transporter.sendMail({
      from: `"MQTT Alert" <${process.env.EMAIL_USER}>`,
      to,
      subject,
      text,
    });

    res.json({ message: "Email sent successfully", info });
  } catch (err) {
    console.error(err);
    res.status(500).json({ message: "Failed to send email", error: err });
  }
};

module.exports = { sendEmail };
