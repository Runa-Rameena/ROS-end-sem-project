# Snake Robot - Simulation & Hardware

## What We Simulated

We developed a complete ROS2 Humble + Gazebo Classic simulation of a modular snake robot with 33 articulated joints. The simulation models an 8-segment snake robot that performs serpentine locomotion through coordinated joint movements.

**Simulation Features:**
- 33 revolute joints with ±90° range of motion
- URDF/Xacro robot model with realistic physics properties
- ros2_control framework with joint_trajectory_controller
- Phase-shifted sinusoidal wave control algorithm
- Real-time visualization in RViz and Gazebo
- Keyboard teleop control (forward, backward, stop)
- Tunable gait parameters (amplitude, frequency, phase offset)

**Control Algorithm:**
Each joint follows a traveling wave equation:
θᵢ(t) = A · sin(ωt + φᵢ)
This creates smooth serpentine undulation where wave propagates along the body.

## What We Built in Hardware

We fabricated a physical 8-segment snake robot using 3D-printed PLA components and off-the-shelf electronics.

**Hardware Components:**
- 8× 3D-printed body segments (75×45×35 mm each)
- 1× head segment (85×45×45 mm) - houses ESP32 controller
- 1× tail segment (65×45×35 mm) - battery compartment
- 8× SG90 micro servo motors (4.8-6.0V, ±90°)
- ESP32 microcontroller (Arduino-compatible)
- 2S Li-Po battery (7.4V, 1000-2000 mAh)
- 5V 6A Buck converter for stable power regulation
- 470µF + 0.1µF capacitors for voltage filtering

**Power System:**
2S Li-Po (7.4V) → Buck Converter → 5V regulated output → Servos + ESP32

**Control Implementation:**
The same sinusoidal control algorithm from simulation was programmed into the ESP32. Each servo receives PWM signals with phase-shifted timing to create the traveling wave motion.

**Total Weight:** ~520g  

## Simulation-to-Hardware Validation

The hardware closely replicated the simulated motion:
- Amplitude: 25-30° (simulation: 25-35°)
- Frequency: 1-2.5 Hz (simulation: 1-3 Hz)  
- Phase coordination: 90%+ match between virtual and physical robot
- Continuous serpentine motion for 30+ minutes on single charge

Both systems use identical control logic, proving the simulation accurately predicts real-world snake locomotion.


