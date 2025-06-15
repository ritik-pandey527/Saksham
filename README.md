# Saksham 🛠️

> Empowering individuals with assistive technology — a collection of innovative projects including ESP‑powered wheelchair control, eye‑controlled interfaces, and fall‑detection systems.

---

## 📖 Description

Saksham is a collection of innovative embedded system modules designed to assist individuals with physical disabilities. It features:
- Smart ESP32-powered Wheelchair control
- Eye-tracking interface for hands-free operation
- AI-based fall detection wearable

These modules aim to reduce caregiver dependency, improve mobility, and ensure safety.

---

## 🧱 Project Structure

📁 ESP_Wheelchair/   → Wheelchair control using ESP and motor driver  
📁 Eye_Control/      → Gaze-tracking and control interface using camera  
📁 Fall_Device/      → Fall detection module using IMU + ESP32  
📄 analysis.ipynb    → Sensor data analysis, classification & visualization

---

## 🚀 Installation

1. Clone the repository:
   git clone https://github.com/ritik-pandey527/Saksham.git
   cd Saksham

2. For Python-based modules (Eye_Control & Fall_Device):
   - Create and activate a virtual environment:
     python3 -m venv venv
     source venv/bin/activate
   - Install required packages:
     pip install -r requirements.txt

3. For ESP32 modules:
   - Use Arduino IDE or PlatformIO
   - Open `.ino` files in ESP_Wheelchair or Fall_Device
   - Upload to the ESP32 board

---

## 📦 Usage

📌 ESP_Wheelchair:
   - Control wheelchair wirelessly using a joystick
   - Connect motors and driver to ESP32

📌 Eye_Control:
   - Connect USB webcam
   - Run the gaze tracker Python script
   - Map gaze points to actions

📌 Fall_Device:
   - Wear the device with ESP + MPU6050
   - Detects sudden falls and triggers alerts

📌 analysis.ipynb:
   - Load sensor data
   - Visualize graphs and test classification

---

## 🧰 Technologies Used

- ESP32 & Arduino IDE
- Python 3
- OpenCV & TensorFlow Lite
- Jupyter Notebook
- MPU6050 (Accelerometer + Gyro)
- Motor Driver (L298N)

---

## 💡 Features

✅ Wireless motorized wheelchair  
✅ Gaze-based interaction system  
✅ AI-powered fall detection  
✅ Real-time alerts and analysis  
✅ Affordable and scalable design 

---

## 🛠 Hardware Requirements

| Module          | Components                                 |
| --------------- | ------------------------------------------ |
| ESP\_Wheelchair | ESP32, Joystick, L298N, DC motors, battery |
| Eye\_Control    | USB camera, PC (Python environment)        |
| Fall\_Device    | ESP32, MPU6050, buzzer                     |

---

## 📊 Data & Analysis

Open the analysis.ipynb file to:
- Analyze IMU sensor data
- Simulate fall detection
- Visualize event classification

---

## 🤝 Contributing

Contributions are welcome!

1. Fork the repository  
2. Create your feature branch  
3. Commit your changes  
4. Push to the branch  
5. Open a Pull Request

---

## 👤 Author

Ritik Pandey  
Electronics Engineer | Embedded & AI Developer  
GitHub: @ritik-pandey527  
LinkedIn: https://linkedin.com/in/ritik33  ](https://www.linkedin.com/in/ritik-pandey-2546b81bb/

---

## ⚖️ License

This project is licensed under the MIT License – see the LICENSE file for details.
