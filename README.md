# StudyBot 🤖

StudyBot is a small self-balancing robot that hangs out on your desk while you study. It can recognize your face, say hi, listen to your questions through a mic, and answer them using OpenAI’s API. It rolls around, looks cute, and sometimes tries to be smart.

---

## What it can do

- Keeps its balance using a gyroscope, PID, and Kalman filter
- Uses a camera to recognize your face and greet you
- Listens through a microphone and replies via speaker
- Talks to OpenAI to answer your questions
- Avoids obstacles with an ultrasonic sensor
- Follows lines (if you want it to)
- Displays a little face or text on a small screen
- Can be controlled via Bluetooth

---

## What’s inside

| Part                    | Model         |
|-------------------------|---------------|
| Microcontroller         | ESP32         |
| Gyroscope               | MPU6050       |
| Ultrasonic Sensor       | HC-SR04       |
| Camera                  | ESP32-CAM     |
| Screen                  | ST7789 (240x240) |
| Motors + Wheels + Encoder | JBG37-520 (65mm, 170 RPM) |
| Microphone              | INMP441       |
| Bluetooth               | JDY-31        |
| Line Sensor             | TCRT5000      |
| Other stuff             | Buttons, breadboard, cables |

---

## Getting started

1. Upload the code to the ESP32.
2. Set up your OpenAI API key in the script.
3. Load some known faces for the camera.
4. Power it on and let it roll.

That’s pretty much it. Tweak things as needed.

---

Made for fun and learning.
