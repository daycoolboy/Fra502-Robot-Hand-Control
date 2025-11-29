# FRA502 Robot Hand Control – Vision-Based Teleoperation (ROS2 + micro-ROS)

This project presents a vision-based teleoperation system for a 5-finger robotic hand. The system uses camera-based gesture recognition combined with ROS2 and micro-ROS(LPUART1) to drive five servo motors in real time. The user simply moves their real hand, and the robotic hand follows by detecting finger poses through Mediapipe and sending the results to an embedded MCU.


## System Overview
System workflow
→ Camera  
→ OpenCV  
→ Mediapipe  
→ vision_node.py (ROS2)  
→ /finger_states  
→ micro-ROS Agent  
→ STM32 firmware  
→ PWM Servo Control  

![Demo Image](https://drive.google.com/uc?export=view&id=1DAyUcjQwDTDwypAqe26dqotBo5nLHeH7)


The system consists of 2 major node:

### 1. Vision_node(PC-Ros2)
- Captures video from a webcam
- Uses Mediapipe Hands to detect 21 hand landmarks
- Evaluates each finger state: open (1) or closed (0)
- Publishes /finger_states to microros_hand_node with provide [T, I, M, R, P]

```
/finger_states   (std_msgs/Int32MultiArray)
```

### 2. microros_hand_node(STM32-Micro-Ros)
- Runs micro-ROS Client on STM32G474RE
- Subscribes to /finger_states
- Converts finger states → servo angles
- Generates PWM TIM2/TIM3
- Controls all 5 servos motors in real time
- 
## Rqt graph

![Demo Image](https://drive.google.com/uc?export=view&id=1BTa7gDMX81j447Gf6ygunLe-nqOfkbLI)


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
- Frequency: 50 Hz
- Pulse range: 500–2500 µs (0–180°)


## 11. How to Run

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

If everything have no problem, you will see webcam output and if you show the right hand to cam the finger state should pub arrays like [1,0,1,0,1] up to your hand condition i forgot to mention that this project can only be run on the right hand.

![Demo Image](https://drive.google.com/uc?export=view&id=1y2nNWWZv8Sxz5cm6wnFGONGF9EI6lhcL)

### STM32 Side (micro-ROS)
Required Applications STM32CubeIDE whitch you can download in this site https://www.st.com/en/development-tools/stm32cubeide.html

Clone the Firmware Branch

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
 
[![Robot Hand Demo](https://img.youtube.com/vi/V4dZimPWutI/0.jpg)](https://www.youtube.com/watch?v=V4dZimPWutI)
