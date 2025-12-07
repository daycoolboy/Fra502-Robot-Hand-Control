# FRA502 Robot Hand Control – Vision-Based Teleoperation (ROS2 + micro-ROS)

This project presents a vision-based teleoperation system for a 5-finger robotic hand. The system uses camera-based gesture recognition combined with ROS2 and micro-ROS(LPUART1) to drive five servo motors in real time. The user simply moves their real hand, and the robotic hand follows by detecting finger poses through Mediapipe and sending the results to an embedded MCU.

The system supports data logging and dual-graph plotting

## System Overview
-PC side (ROS2 – Python)
-MediaPipe landmark detection
-Finger angle computation
-Smoothing + limit_step
-Publishes /finger_states
-Records angle data for plotting (Desired + Feedback)
-STM32 side (micro-ROS)
-Subscribes to /finger_states
-Converts angle → PWM (TIM2, TIM3)
-Controls 5 MG996R servos
-Publishes /servo_feedback back to PC


![Demo Image](https://drive.google.com/uc?export=view&id=1DAyUcjQwDTDwypAqe26dqotBo5nLHeH7)


The system consists of 2 major node:

### 1. Vision_node(PC-Ros2)
- Captures video from a webcam
- Uses Mediapipe Hands to detect 21 hand landmarks
- Computes **servo angles** for all 5 fingers  
  - Thumb: uses **x-axis distance** between landmarks 2 and 4 + normalization  
  - Other fingers: use **joint angle at PIP** and map to servo 0–180°  
- Smoothing and limiting:
  - Moving average with deque
  - limit_step() to avoid sudden big jumps
- Publishes topic:
  /finger_states   (std_msgs/Int32MultiArray)
  data = [thumb, index, middle, ring, pinky]
  
- Subscribes topic:
 /servo_feedback  (std_msgs/Int32MultiArray)
data = [thumb, index, middle, ring, pinky]  


### 2. microros_hand_node(STM32-Micro-Ros)
- Runs micro-ROS Client on STM32G474RE
- Subscribes to /finger_states
- For each finger:
 - Reads angle value from the array
 - Calls angle_to_pulse() to map 0–180° → 500–2500 µs
 - Updates TIM2/TIM3 PWM compare register
- Publishes feedback:
  - /servo_feedback  (std_msgs/Int32MultiArray)
## Rqt graph

![Demo Image](https://drive.google.com/uc?export=view&id=18ngL3L3Ifkf344Cjnxc-mJuXMpv1j-bx)



## Mechanical Structure

- Custom 3D-printed robotic hand 
- Pin-joint knuckles
- Tendon-driven flexion and Rubber for retracting
- Five servos for direct actuation of each finger

![Demo Image](https://drive.google.com/uc?export=view&id=16WGPum65DUiu9hxenZ87CaHENc8OIv-R)


## Electrical System

### 1.Hardware
- STM32G474RE Nucleo-64 board 
- 5 servo motors (MG996R)
- 5V 5A power supply
- USB-UART connection for micro-ROS agent

### PWM Output Pins  
- TIM2 and TIM3
- Frequency: 50 Hz (standard)
- Pulse range: 500–2500 µs (0–180°)

### Data Logging & Graph Comparison
The system records two datasets automatically

- Desired angles (from vision_node)
  - After MediaPipe detection, angle calculation, smoothing, and limit_step
  - Represent the intended servo command from vision
- Servo feedback angles (from STM32)
  - Read from /servo_feedback topic
  - Represent the actual angles sent to PWM
  - 
In Python, we use a helper class AngleRecorder (in plotter.py) that saves for each frame

![Demo Image](https://drive.google.com/uc?export=view&id=1d-3eLU3lDRC0PXjo-6ajEEltJfwlolX6)

The top graph is the angle the Vision system “commands” to rotate, while the bottom graph is the actual angle measured by the STM32. You can see that the graph shape of each finger is almost identical when increasing the angle, holding the angle, and releasing it. The STM32 graph is just a little slower and the end angle is a little lower, which is a result of the response of the servo and the actual mechanism, showing that the STM32 system can follow the angle command from the Vision well.

## 11. How to Run

### Requirements

- Ubuntu 22.04 + ROS 2 Humble
- Python 3.10+
- Webcam (tested with Logitech C270)
- STM32G474RE Nucleo-64 board
- 5 × MG996R servo motors
- 5V 5A power supply



ROS2 installation instructions are available from the official documentation

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

open terminal1 and 

```
sudo apt update && sudo apt upgrade -y
```


Install Required Tools

```
sudo apt install python3-colcon-common-extensions -y
sudo apt install ros-dev-tools -y
pip3 install opencv-python mediapipe numpy
sudo apt install ros-humble-micro-ros-agent -y
```

This repository contains two main branches, separating the PC-side ROS2 code(main) and the STM32 firmware(STM)

### PC(ROS2)
find your free space and clone my project

```
git clone https://github.com/<your-repo>/hand_controller.git
```

go into workspace and build

```
colcon build
source install/setup.bash
```

Run the vision node

```
ros2 run hand_controller vision_node
```

If everything have no problem, you will see webcam output and if you show the right hand to cam the finger state should pub arrays like [180,180,180,180,180] up to your hand condition i forgot to mention that this project can only be run on the right hand with Hand positioning likes img.

![Demo Image](https://drive.google.com/uc?export=view&id=1-4t7ahAamZF1v8QD26irzphGtASMLLc7)




### STM32 Side (micro-ROS)
Required Applications STM32CubeIDE whitch you can download in this site https://www.st.com/en/development-tools/stm32cubeide.html

Clone the Firmware Branch and dont for got to build

open ternimal 2 

find your free space clone and Extract the files properly.

```
git clone -b stm https://github.com/<your-github>/Fra502-Robot-Hand-Control.git
```


Build the Firmware in STM32CubeIDE by open Projects from File System and search for ioc.

Right-click and Build Project or Press the gear ioc

after that run the code to your stm

If build is successful, you will see

```
Build Finished. 0 errors.
```

continue in terminal2 

Fix Docker permission if required

```
sudo chmod 666 /var/run/docker.sock
```

entry you password

Then start the agent

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

now everything is ready to test 

Terminal 1 vision_node

Terminal 2 micro-ROS agent

Move your right hand robotic hand moves in real time

For the system test, I have made a simple video for you guy to watch.
 
[![🎥 Demo Video (YouTube)](https://img.youtube.com/vi/XuOGI15YNFc/0.jpg)](https://youtu.be/XuOGI15YNFc?si=NSAwgebZN-E9HBus)

