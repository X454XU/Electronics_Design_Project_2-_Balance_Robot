# BalanceRobot Team Bonus

## Overview
This repository contains the code and documentation for the Balance Robot project, a two-wheeled self-balancing robot with autonomous navigation capabilities. Our project integrates various subsystems including balance control, movement control, image processing, and user interface to achieve a stable and autonomous robotic system.

## Subsystems

### 1. Balance Control
- **Inner Loop: Tilt Control**
  - Implements PID control for managing the robot's tilt using an MPU6050 sensor.
- **Outer Loop: Speed Control**
  - Manages the robot's speed using feedback from the stepper motors.

### 2. Movement Control
- **Linear and Rotational Movement**
  - Functions to move the robot forward, backward, and to turn left or right.
- **PID Controllers**
  - Utilized for precise control of speed and direction.

### 3. Image Processing
- **Object Detection**
  - Uses a Raspberry Pi and camera module for real-time image processing to detect and follow specified objects.
- **OpenCV**
  - Employed for converting images to HSV format, creating masks, and identifying objects.

### 4. Communication
- **WiFi Connection**
  - Establishes communication between the ESP32 microcontroller and a server for remote control and monitoring.
- **Server and Client Integration**
  - Flask-based server to handle video streaming and control commands; client to send video and execute received commands.

### 5. User Interface
- **Web-based UI**
  - Provides manual control and live monitoring of the robot, displaying key information such as battery life and PID values.

### 6. Power Management
- **Voltage Monitoring**
  - Ensures stable power supply and monitors battery levels to prevent power-related issues.

## Getting Started
### Prerequisites
- ESP32 Development Board
- Raspberry Pi with Camera Module
- Stepper Motors
- MPU6050 Sensor
- WiFi Network

### Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/Arc-Cloud/BalanceRobot.git
   cd BalanceRobot
   ```
2. Set up the firmware environment:
   ```bash
   cd firmware
   platformio run
   ```
   Or use the automated extension in VS code
   
4. Install dependencies for the UI:
   ```bash
   cd UI
   npm install
   ```

### Usage
1. Flash the firmware to the ESP32.
2. Run the server for image processing and control:
   ```bash
   cd Sensing/Object\ Detection
   python main.py
   ```
3. Launch the UI to start controlling the robot:
   ```bash
   cd UI
   npm start
   ```

## Contributors
- Maximilian Adam
- Xiaoyang Xu
- Fedor Dakhnenko
- Yueming Wang
- Maxim Kelly
- Yingkai Xu

## Acknowledgments
Special thanks to Dr. Edward Stott for guidance and support throughout the project.

## Repository Structure
```
BalanceRobot/
├── .idea/
│   ├── inspectionProfiles/
│   │   └── profiles_settings.xml
│   ├── .gitignore
│   ├── BalanceRobot.iml
│   ├── misc.xml
│   ├── modules.xml
│   └── vcs.xml
├── .vscode/
│   ├── c_cpp_properties.json
│   ├── launch.json
│   └── settings.json
├── firmware/
│   ├── .pio/
│   ├── .venv/
│   ├── .vscode/
│   ├── include/
│   │   ├── mpu6050.h
│   │   ├── PIDController.h
│   │   └── README
│   ├── lib/
│   │   └── README
│   ├── src/
│   │   ├── main.cpp
│   │   ├── mpu6050.cpp
│   │   ├── output.csv
│   │   ├── PIDController.cpp
│   │   ├── plot.py
│   │   └── step.h
│   ├── test/
│   │   ├── esp32_mpu6050_test.ino
│   │   ├── mpu6050.h
│   │   ├── output.csv
│   │   ├── platformio.ini
│   │   ├── plot.py
│   │   ├── speedTiltPlot.py
│   │   └── tuning.py
│   ├── .DS_Store
│   ├── .gitignore
│   ├── Measuring Speed.txt
│   ├── platformio.ini
│   └── Tuning Method.txt
├── Sensing/
│   ├── camera/
│   │   └── cameraCalibration.py 
│   │   └── objectDetection.py
│   ├── Object Detection/
│   │   ├── yolov5/
│   │   ├── .gitignore
│   │   ├── findCam.py
│   │   ├── main.py
│   │   └── testCam.py
│   └── Power Sensing/
│       ├── data_processing.py
│       └── voltage_to_charge.json
├── UI/
│   ├── assets/
│   ├── css/
│   │   └── style.css
│   ├── js/
│   │   ├── camera.js
│   │   ├── main.js
│   │   ├── map.js
│   │   ├── speed.js
│   │   ├── switch.js
│   │   └── wasd.js
│   ├── Power UI/
│   │   ├── templates/
│   │   │   └── display.html
│   │   └── server_script.py
│   ├── .DS_Store
│   └── index.html
├── .DS_Store
├── .gitignore
└── README.md
```
