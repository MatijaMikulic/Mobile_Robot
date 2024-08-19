
# ESP32 Mobile Robot with IMU Integration

This project demonstrates the implementation of an ESP32-based mobile robot, equipped with multiple sensors including MPU6050 and MPU9250 IMUs (Inertial Measurement Units). The robot can be controlled via Bluetooth, and it includes features such as straight-line control, rotation control, and distance measurement.

## Table of Contents
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation and Setup](#installation-and-setup)
- [Project Structure](#project-structure)
- [How It Works](#how-it-works)
- [Usage](#usage)
- [Acknowledgements](#acknowledgements)

## Features
- **MPU6050 and MPU9250 IMU Sensors**: Used for measuring orientation, angular velocity, and acceleration.
- **Bluetooth Control**: Allows the robot to be controlled remotely via Bluetooth using predefined commands.
- **PID Control for Straight-Line Movement**: The robot uses a PID controller to ensure it moves in a straight line.
- **Encoder-based Distance Measurement**: The robot can measure the distance traveled using encoder feedback.
- **Complementary Filter**: Combines gyroscope, accelerometer, magnetometer data to provide accurate orientation.

## Hardware Requirements
- ESP32 NodeMCU-32S Board
- MPU6050 and MPU9250 IMU sensors
- L298N Motor Driver
- DC Motors with Encoders
- IR Sensors (for encoders)
- Power Supply
- Bluetooth-enabled device (for control)

## Software Requirements
- [PlatformIO](https://platformio.org/) with support for ESP32
- Arduino framework
- Serial monitor for debugging (115200 baud rate)
- Libraries:
  - Wire
  - I2Cdev
  - MPU6050 by electroniccats
  - Madgwick filter
  - Bolder Flight Systems MPU9250

## Installation and Setup
1. **Clone the Repository**:
    ```bash
    git clone https://github.com/yourusername/esp32-mobile-robot.git
    cd esp32-mobile-robot
    ```

2. **Install PlatformIO**:
   Ensure you have PlatformIO installed on your system. If not, follow the installation guide [here](https://platformio.org/install).

3. **Open Project in PlatformIO**:
   Open the project in your preferred IDE that supports PlatformIO (e.g., VS Code).

4. **Install Dependencies**:
   PlatformIO will automatically handle the installation of required libraries listed in `platformio.ini`.

5. **Upload Code to ESP32**:
   Connect your ESP32 board to your computer and upload the code using PlatformIO's upload command.

## Project Structure
```
esp32-mobile-robot/
│
├── src/
│   ├── main.cpp                     # Main source file
│   ├── CommandQueue.h               # Command queue for handling multiple commands
│   └── ...                          # Additional source files
│
├── lib/
│   ├── IMU/
│   │   ├── MPU6050Imp/
│   │   │   └── MPU6050Imp.h         # MPU6050 Implementation
│   │   ├── MPU9250Imp/
│   │   │   └── MPU9250Imp.h         # MPU9250 Implementation
│   │   └── IMU.h                    # IMU Interface
│   └── ...                          # Additional libraries
│
├── platformio.ini                   # PlatformIO configuration file
└── README.md                        # This README file
```

## How It Works
- **Initialization**: The IMU sensors are initialized, and Bluetooth is set up for remote control.
- **Control Flow**: The robot processes commands received via Bluetooth and executes them (e.g., move forward, turn left).
- **Motion Control**: The robot uses feedback from IMU sensors and encoders to adjust motor speeds and maintain a straight path or rotate accurately.
- **Complementary Filter**: The yaw angle is calculated using a complementary filter, combining data from the gyroscope, accelerometer and magnetometer.

## Usage
1. **Pair the ESP32 with a Bluetooth-enabled device**.
2. **Send Commands**:
   - `&w;`: Move forward
   - `&s;`: Move backward
   - `&a;`: Turn left
   - `&d;`: Turn right
   - `&x;`: Stop the robot
   -  send auto commands


## Acknowledgements
- This project makes use of open-source libraries and tools. Special thanks to the developers of the MPU6050, MPU9250, and other libraries used.
