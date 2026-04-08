# Deep Learning-based Situation-Aware Platooning Assist System

## Introduction
This project implements a Deep Learning-based Situation-Aware Platooning Assist System using Raspberry Pi and the Adeept PiCar-B. The system enables multiple autonomous vehicles to form a platoon, maintaining safe distances and adapting to various traffic scenarios such as obstacle avoidance and leader changes using YOLO-based object detection and lane tracking.

## Key Features
- **Lane Tracking:** Real-time lane detection and following using OpenCV and specialized image processing.
- **Platooning:** Cooperative adaptive cruise control to maintain safe distances between vehicles in a fleet.
- **Situation Awareness:** Object detection for identifying obstacles and other vehicles using YOLOv5 and YOLOv8 models.
- **Dynamic Mode Switching:** Supports multiple operational modes including Normal, Platoon, Overtake, and Leader Change.
- **Remote Control & Monitoring:** Flask-based web interface for real-time video streaming, sensor data monitoring, and remote car control.

## Hardware Configuration
- **Controller:** Raspberry Pi (Raspberry Pi 4B or 5 recommended)
- **Chassis:** Adeept PiCar-B (Mars Rover Smart Car Kit)
- **Sensors:**
  - Raspberry Pi Camera Module
  - Ultrasonic Sensor (HC-SR04) for obstacle detection
  - Line Tracking Module for lane following
- **Actuators:**
  - PCA9685 PWM Driver for motor and servo control
  - DC Motors for driving
  - MG90S Servos for steering and camera pan/tilt control

## Repository Structure
- **src/Platooning_System/**: Main source code including the core logic, image processor, and motor control modules.
- **docs/**: Project presentation materials, reports, and related research papers.
- **media/**: Folder for project demonstration videos (Note: No MP4 files were found in the source directory).
- **libs/**: Essential hardware driver libraries and setup scripts for the PiCar-B.

## Getting Started

### Library Installation
Before running the code, ensure the following dependencies are installed:
1. Install Python packages:
   ```bash
   pip3 install adafruit-circuitpython-pca9685 adafruit-circuitpython-motor adafruit-circuitpython-servokit flask opencv-python numpy torch torchvision
   ```
2. Configure the hardware driver:
   ```bash
   cd libs/adeept_picar-b2
   sudo python3 setup.py
   ```

### Code Execution
1. Navigate to the source directory:
   ```bash
   cd src/Platooning_System
   ```
2. Execute the main program:
   ```bash
   python3 Platooning_System.py
   ```
   *(Note: The main entry point `Platooning_System.py` was originally `main_lane_follow.py`.)*
