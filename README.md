# 🚗 Vehicle Alert System

An IoT-based **Vehicle Alert System** built on **Raspberry Pi 4** to improve driver and passenger safety.
The system integrates sensors, machine learning, and computer vision to provide **real-time alerts** and prevent accidents.

---

## ✨ Features

1. 🚨 **Accident Detection & SMS Alert**

   * Detects sudden impact or collision using an **accelerometer**.
   * Sends **emergency SMS** with GPS location via GSM module to predefined contacts.

2. 🛑 **Traffic Sign Recognition (Driver Assist)**

   * Uses a **Machine Learning model (CNN trained on GTSRB dataset)** to detect traffic signs.
   * Assists drivers by alerting them to **speed limits, stop signs, and other signals**.

3. 🌫️ **Fog/Obstacle Detection**

   * Detects obstacles ahead in **low-visibility conditions** (fog, dark, or poor weather) using an **ultrasonic sensor**.
   * Provides instant **voice/audio alerts** to the driver.

4. 😴 **Drowsiness Detection**

   * Monitors driver’s eyes using **Pi Camera + OpenCV**.
   * Triggers an **alarm/voice alert** if the driver shows signs of fatigue or sleepiness.

---

## 🛠️ System Architecture

| Component                             | Function                                               |
| ------------------------------------- | ------------------------------------------------------ |
| **Raspberry Pi 4**                    | Central controller for data processing and integration |
| **Pi Camera**                         | Captures driver’s face for drowsiness detection        |
| **Accelerometer (ADXL335 + ADS1115)** | Detects vehicle collisions                             |
| **GPS6MV2 Module**                    | Captures vehicle location for SMS alerts               |
| **SIM900A GSM Module**                | Sends emergency SMS notifications                      |
| **Ultrasonic Sensor**                 | Detects nearby obstacles                               |
| **Speaker**                           | Provides real-time voice alerts                        |
| **Software**                          | Python, OpenCV, TensorFlow, PyTorch, pyttsx3           |

---

## ⚙️ Installation & Setup

1. **Clone the repository**

   ```bash
   git clone https://github.com/KevalSonariya-dev/vehicle-alert-system-.git
   cd vehicle-alert-system-
   ```
2. **Install dependencies**

   ```bash
   pip install -r requirements.txt
   ```
3. **Connect hardware components** (accelerometer, GPS, GSM, camera, ultrasonic sensor).
4. **Run the system**

   ```bash
   python vehicle_alert_system.py
   ```

---

## 🚦 How It Works

* Continuously monitors **driver behavior** and **road conditions**.
* Alerts the driver via **voice/audio notifications**.
* In case of accidents, **sends an emergency SMS with GPS location**.

---

## 🧰 Technologies Used

* **Hardware:** Raspberry Pi 4, Pi Camera, ADXL335 Accelerometer, ADS1115 ADC, GPS6MV2, SIM900A GSM, Ultrasonic Sensor
* **Software:** Python, OpenCV, TensorFlow, PyTorch, pyttsx3
* **Concepts:** IoT, Machine Learning, Computer Vision, Real-Time Monitoring

---

## 🚀 Future Enhancements

* Cloud integration for live monitoring 📡
* Dashboard for visualizing trip and safety data 📊
* Multi-language voice assistance 🌍
* Advanced AI models for more accurate detection 🤖

---

## 👤 Author

**Keval Sonariya**
🔗 [GitHub Profile](https://github.com/kevalsonariya)

---

