const nodemailer = require("nodemailer");

(async () => {
  try {
    // Create a transporter (using Ethereal for testing)
    const transporter = nodemailer.createTransport({
      host: "smtp.ethereal.email",
      port: 587,
      secure: false, // false for 587
      auth: {
         user: 'samson.wisoky@ethereal.email',
        pass: 'bK2u6UBKrNqnnyW9ad'
      },
    });

    // Send a test email
    const info = await transporter.sendMail({
      from: '"MQTT Alert" <samson.wisoky@ethereal.email>',
      to: "",   // replace with the email you want to test
      subject: "Test Email ✔",
      text: "This is a test email from Nodemailer",
      html: "<b>This is a test email from Nodemailer</b>",
    });

    console.log("Email sent successfully!");
    console.log("Preview URL: " + nodemailer.getTestMessageUrl(info));
  } catch (err) {
    console.error("Error sending email:", err);
  }
})();
