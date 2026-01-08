# 🛵 Activa Smart Keyless System

Turn your standard scooter into a smart vehicle with **Keyless Entry**, **Push-to-Start**, and a **Premium Dashboard**.

<img src="https://github.com/user-attachments/assets/1f6a882b-34bf-4881-af67-a8cbd3fd7b33" hight="80" >


This project uses an **Arduino Nano** and **HC-05 Bluetooth Module** controlled by a custom Android App (built with MIT App Inventor) featuring a high-end HTML/CSS interface.

## 🌟 Key Features

### 📱 Premium Dashboard App
* **Dark Mode UI:** Glassmorphism design with a "Deep Blue/Black" gradient.
* **"Ready To Race" Boot Sequence:** Simulates a bike ECU boot-up with a flag animation and system check.
* **Auto-Connect & Heartbeat:** The app automatically connects when near the bike and syncs state (Red/Green indicators) every 2 seconds.
* **Visual Feedback:** "Sonar" pulsing animation for the Locator and "Lock Slam" animation for security mode.

### 🧠 Smart Logic (Arduino Nano)
* **Latched Relay Control:** Handles Ignition and Starter relays efficiently.
* **Anti-Hijack Mode:** Automatically cuts the engine if the phone disconnects for more than 2 minutes while running.
* **Deep Sleep Mode:** Saves battery by turning off Bluetooth if left disconnected for 10 minutes.
* **Secret Tap Code:** A physical backup button allows you to start the bike with a secret rhythm (e.g., Tap-Tap-Hold) if you forget your phone.

## 🛠️ Hardware Required
* Arduino Nano
* HC-05 Bluetooth Module
* 2-Channel 5V Relay Module (Active Low)
* 12V to 5V Buck Converter
* Physical Push Button (for backup code)
* Piezo Buzzer (for "Find My Bike" feature)

## Wiring 
<img width="1208" height="910" alt="image" src="https://github.com/user-attachments/assets/b60c6840-b8d0-4471-bfdf-6747c21edce9" />


<img width="797" height="580" alt="image" src="https://github.com/user-attachments/assets/93600ab5-fe1e-4ecc-9f2d-60650625e6cc" />

## 📂 File Structure
* `/Arduino`: Contains the `Activa_Smart_System.ino` code (Bug-fixed version with `BT.setTimeout`).
* `/App`: Contains the MIT App Inventor project (`.aia`) and installable (`.apk`).
* `/Assets`: Contains the `index.html` dashboard and images (`activa.png`).

## 🚀 How to Use
1.  **Upload** the Arduino code to your Nano.
2.  **Install** the APK on your Android phone.
3.  **Pair** your phone with the HC-05 module (Password usually `1234` or `0000`).
4.  **Open the App**: Watch the "Ready to Race" animation and tap "Start Engine".

---
*Created by [Your Name]*
