# FRA502 Robot Hand Control – Vision-Based Teleoperation (ROS2 + micro-ROS)

This project presents a vision-based teleoperation system for a 5-finger robotic hand. The system uses camera-based gesture recognition combined with ROS2 and micro-ROS to drive five servo motors in real time. The user simply moves their real hand, and the robotic hand follows by detecting finger poses through Mediapipe and sending the results to an embedded MCU.


## System Overview
The system steps is 
→ Camera  
→ OpenCV  
→ Mediapipe  
→ vision_node.py (ROS2)  
→ /finger_states  
→ micro-ROS Agent  
→ STM32 firmware  
→ PWM Servo Control  

The complete system consists of 2 major node:

### 1. Vision_node(PC)
- Captures video from a webcam
- Uses Mediapipe Hands to detect 21 hand landmarks
- Evaluates each finger state: open (1) or closed (0)
- Publishes /finger_states to microros_hand_node with provide [T, I, M, R, P]

```
/finger_states   (std_msgs/Int32MultiArray)
```

### 2. microros_hand_node(STM32)
- STM32G474RE run micro-ROS Client
- Receives finger states from the agent
- Generates PWM signals
- Controls all 5 servos in real time

## Mechanical Structure

- Custom 3D-printed robotic hand 
- Pin-joint knuckles
- Tendon-driven flexion and Rubber for retracting
- Five servos for direct actuation of each finger


## Electrical System

### 1.Hardware
- STM32G474RE Nucleo-64 board 
- 5 servo motors (MG996R)
- 5V 5A power supply
- USB-UART connection for micro-ROS agent

### PWM Output Pins  
- TIM2 and TIM3
- Frequency: 50 Hz
- Pulse range: 500–2500 µs (0–180°)


## 11. How to Run

for ROS2 Humble you can follow this guidline https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

---
sudo apt update && sudo apt upgrade -y
---


Install Required Tools

---
sudo apt install python3-colcon-common-extensions -y
sudo apt install ros-dev-tools -y
pip3 install opencv-python mediapipe numpy
sudo apt install ros-humble-micro-ros-agent -y
---







