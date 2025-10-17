Snake Robot: Design, Power Architecture, and Simulation

A modular, biologically-inspired snake robot platform featuring 3D-printed segments, ESP32-based control, and ROS2/Gazebo simulation for serpentine locomotion research.

Table of Contents
- Overview
- Features
- System Architecture
- Hardware Components
- Installation
- Usage
- Simulation
- Hardware Setup
- Control Algorithm
- Results
- Future Work
- Contributors
- References
- License

Overview

This project presents a low-cost, modular snake robot capable of executing smooth serpentine locomotion through coordinated servo control. The system integrates:

- 8 articulated 3D-printed segments driven by SG90 micro servos
- ESP32 microcontroller for embedded control
- ROS2 Humble + Gazebo simulation framework
- Phase-shifted sinusoidal control for biomimetic gait generation
- Optimized power architecture (2S Li-Po + Buck converter)

Key Achievements:
- Stable serpentine motion with synchronized 33-joint actuation
- High-fidelity ROS2/Gazebo simulation matching hardware behavior
- Modular design enabling easy assembly and expansion
- Complete simulation-to-hardware validation pipeline

Features

Mechanical Design:
- Modular Architecture: 10 segments (8 body + head + tail)
- 3D-Printed PLA: Lightweight (520g total), easy to fabricate
- ±90° Joint Range: Sufficient for serpentine undulation
- Integrated Cable Management: Internal routing channels

Electronics:
- ESP32 Microcontroller: Arduino-compatible, 8× PWM outputs
- Power System: 7.4V 2S Li-Po + 5V/6A Buck converter
- Stable Voltage Regulation: Eliminates brownouts and jitter
- Decoupling: 470µF electrolytic + 0.1µF ceramic capacitors

Control & Simulation:
- Traveling Wave Gait: Phase-shifted sine wave control θᵢ(t) = A·sin(ωt + φᵢ)
- ROS2 Integration: ros2_control, joint_trajectory_controller
- Gazebo Physics: Realistic dynamics and terrain interaction
- Teleop Control: Keyboard-driven forward/backward/stop commands

System Architecture

ROS2 Humble connects to Gazebo Classic for simulation-to-hardware pipeline. ESP32 Microcontroller implements sinusoidal control algorithm θᵢ(t) = A·sin(ωt + φᵢ) with PWM refresh at 6ms intervals, driving 8× SG90 Micro Servos with 5V regulated power.

Hardware Components

Component | Specification | Quantity
SG90 Servo | 4.8-6.0V, ±90°, 0.75A stall | 8
ESP32 | Arduino Uno-compatible | 1
Li-Po Battery | 2S (7.4V), 1000-2000 mAh | 1
Buck Converter | 5V @ 6A continuous output | 1
3D-Printed Segments | PLA, 20% infill | 10
M2 Screws | Servo mounting | ~24
Capacitors | 470µF + 0.1µF (filtering) | 2

Total Weight: ~520g
Estimated Cost: ₹2,000-₹2,500 (~$25-$30 USD)

Installation

Prerequisites:
Ubuntu 22.04 with ROS2 Humble installed

sudo apt update
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control

Clone Repository:
mkdir -p ~/snake_ws/src
cd ~/snake_ws/src
git clone https://github.com/yourusername/snake_robot.git
cd ~/snake_ws

Build Package:
colcon build --packages-select snake_robot --symlink-install
source install/setup.bash

Usage

Simulation Launch (3 Terminals Required)

Terminal 1: Gazebo + Controllers
cd ~/snake_ws
source install/setup.bash
ros2 launch snake_robot spawn_snake_gazebo.launch.py

Wait for controllers to show active status.

Terminal 2: Snake Controller
source ~/snake_ws/install/setup.bash
ros2 run snake_robot snake_robot_node

Terminal 3: Teleop Control
source ~/snake_ws/install/setup.bash
ros2 run snake_robot snake_teleop --ros-args -p command_topic:=snake_command

Keyboard Controls:
w / ↑ : Forward serpentine motion
s / ↓ : Backward motion
Space : Stop
q : Quit teleop

Simulation

The simulation uses URDF/Xacro for robot description (33 revolute joints), ros2_control with joint_trajectory_controller, and Gazebo Classic for physics (friction, gravity, collisions).

Tuning Gait Parameters:
Edit locomotion.launch.py with parameters for num_joints: 33, amplitude: 0.25 (Joint angle swing degrees), frequency: 0.3 (Wave speed Hz), phase_offset: 0.5 (Phase shift between joints), spatial_frequency: 0.8 (Waves along body)

Verify Controllers:
ros2 control list_controllers
ros2 action info /joint_trajectory_controller/follow_joint_trajectory

Hardware Setup

Power Configuration:
Connect 2S Li-Po (7.4V) → Buck Converter (5V 6A) → Servo Rail + ESP32 → 470µF + 0.1µF caps

ESP32 Wiring:
GPIO 2-9: Servo PWM signals (8 servos)
5V / GND: Buck converter output
SDA/SCL: MPU6050 IMU (optional)

Upload Control Code:
cd esp32_code/
Open snake_robot_control.ino in Arduino IDE
Select Board: ESP32 Dev Module
Upload to ESP32

Control Algorithm

The serpentine gait is generated using a traveling wave equation:
θᵢ(t) = A · sin(ωt + φᵢ)

Where:
A = Amplitude (max joint angle)
ω = Angular frequency (wave speed)
φᵢ = Phase offset for joint i

ESP32 Code Snippet:
include Servo.h
Servo servos[8];
float amplitude = 30, frequency = 2.0, phase[8];

void setup() {
  for (int i = 0; i < 8; i++) {
    servos[i].attach(i + 2);
    phase[i] = i * 0.5;
  }
}

void loop() {
  for (int i = 0; i < 8; i++) {
    int angle = 90 + amplitude * sin(frequency * millis() / 1000.0 + phase[i]);
    servos[i].write(angle);
  }
  delay(6);
}

Results

Simulation vs Hardware:
Parameter | Simulation | Hardware | Match
Amplitude | 25-35° | 25-30° | High
Frequency | 1-3 Hz | 1-2.5 Hz | Good
Phase Offset | 30-45° | 30-40° | High
Motion Smoothness | Excellent | Minor jitter at >2.5Hz | Good

Performance:
- Stable voltage regulation (no brownouts)
- Synchronized joint motion across 8 segments
- Continuous serpentine gait for 30+ minutes
- Torque limited at higher payloads (SG90 constraint)

Future Work

1. Sensor Integration: Add IMU (MPU6050) for closed-loop stability control
2. Terrain Adaptation: Implement adaptive gait using reinforcement learning
3. Hardware Upgrades: Replace SG90 with MG90S metal-gear servos for higher torque
4. Multi-Gait Switching: Add sidewinding, concertina, and rolling locomotion modes
5. Autonomous Navigation: Integrate LiDAR/camera for obstacle avoidance
6. ROS2 Iron/Jazzy Migration: Upgrade to latest ROS2 distributions

Contributors

CH.SC.U4AIE23054 – Subhash R
CH.SC.U4AIE23057 – Renuka V J
CH.SC.U4AIE23059 – Vaishnavi S
CH.SC.U4AIE23060 – Veena Sundari A

Faculty Advisor: Dr. Dev Kunwar Singh Chauhan
Course: 22AIE442: Robot Operating System & Robot Simulation
Institution: Chitkara University

References

1. Bartnik, N. (2016). 3D Printed Snake Robot with SG90 Servos
2. Adafruit Industries (2016). DIY Snake Robot Tutorials
3. MENG411 Group 3 (2021). Multi-Servo Snake Robot Torque Analysis
4. ROS2 Documentation: https://docs.ros.org/en/humble/
5. Gazebo Classic: http://gazebosim.org/
6. ESP32 Arduino Core: https://github.com/espressif/arduino-esp32

License

MIT License

Copyright (c) 2025 Snake Robot Team

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Acknowledgments

We would like to thank Dr. Dev Kunwar Singh Chauhan for guidance throughout this project, Chitkara University for providing resources and facilities, and the open-source robotics community for ROS2 and Gazebo tools.

Contact

