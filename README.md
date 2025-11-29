# FRA502 Robot Hand Control – Vision-Based Teleoperation (ROS2 + micro-ROS)

โปรเจกต์นี้เป็นระบบควบคุมมือกล 5 นิ้วโดยใช้การประมวลผลภาพจากกล้อง (Vision) ร่วมกับ ROS2 และ micro-ROS เพื่อควบคุมเซอร์โวแต่ละนิ้วแบบเรียลไทม์ ผู้ใช้เพียงขยับมือจริง ระบบจะวิเคราะห์ท่าทางนิ้วด้วย Mediapipe และส่งผลไปสั่ง Servo ให้เคลื่อนตามมือมนุษย์

------------------------------------------------------------

## 1. System Overview

ระบบประกอบด้วย 3 ส่วนหลัก

### 1) Vision Processing (PC)
- อ่านภาพจากกล้อง
- ใช้ Mediapipe ตรวจ landmark 21 จุดของมือ
- คำนวณสถานะนิ้วว่า งอ(0) หรือ เหยียด(1)

### 2) ROS2 Node (PC)
ส่งค่าผ่าน Topic:

```
/finger_states   (std_msgs/Int32MultiArray)
```

### 3) Embedded micro-ROS (STM32)
- STM32G474RE ทำงานเป็น micro-ROS Client
- รับค่าจาก agent แล้วควบคุม PWM
- ขยับ Servo 5 ตัวแบบเรียลไทม์

------------------------------------------------------------

## 2. Mechanical Structure

- พิมพ์มือ 3D พร้อมข้อพับแบบ pin joint  
- ใช้เชือกดึงเพื่องอนิ้ว  
- ใช้ยาง/สปริงคืนรูปเมื่อต้องการเหยียด  
- Servo 5 ตัวควบคุมนิ้วแต่ละนิ้วโดยตรง  

------------------------------------------------------------

## 3. Electrical System

### Hardware
- STM32G474RE Nucleo-64  
- Servo motor 5 ตัว  
- Power supply 5V 5A  
- USB–UART สำหรับ micro-ROS agent  

### PWM Output Pins  
- ใช้ TIM2 + TIM3  
- ความถี่ 50Hz  
- Pulse 500–2500 µs (0–180°)

------------------------------------------------------------

## 4. Software Architecture

ลำดับการประมวลผลระบบ:

Camera  
→ OpenCV  
→ Mediapipe  
→ vision_node.py (ROS2)  
→ /finger_states  
→ micro-ROS Agent  
→ STM32 firmware  
→ PWM Servo Control  

------------------------------------------------------------

## 5. Finger State Evaluation

นิ้วแต่ละนิ้วถูกคำนวณจาก landmark เช่น  
- นิ้วชี้/กลาง/นาง/ก้อย — เทียบ tip.y < pip.y  
- นิ้วโป้ง — เทียบ tip.x กับ ip.x  

ผลลัพธ์ส่งเป็น array:

```
[T, I, M, R, P]
เช่น [1,0,1,0,1]
```

------------------------------------------------------------

## 6. Firmware Logic (STM32)

ฟังก์ชันหลัก:
- micro-ROS executor
- subscribe `/finger_states`
- แปลง 0 → 0° และ 1 → 180°
- PWM control servo 5 channels
- pulse คงที่: 500–2500 µs

------------------------------------------------------------

## 7. Testing Summary

- ทำงานแบบเรียลไทม์ FPS สูง  
- response time ต่ำกว่า 50 ms  
- Servo 5 ตัวขยับพร้อมกันได้  
- agent ไม่หลุดการเชื่อมต่อ  

------------------------------------------------------------

## 8. Known Issues & Solutions

| ปัญหา | สาเหตุ | วิธีแก้ |
|-------|--------|---------|
| Servo สั่น | กระแสไม่พอ | ใช้ไฟ 5V 5A |
| micro-ROS agent ไม่เชื่อมต่อ | USB permission | sudo chmod 666 |
| นิ้วฝืด | เชือกตึงผิด | ปรับ routing |

------------------------------------------------------------

## 9. Work Timeline

- สัปดาห์ 1–3: ออกแบบมือ + พิมพ์ 3D  
- สัปดาห์ 2–4: ออกแบบวงจร + ต่ออุปกรณ์  
- สัปดาห์ 3–5: พัฒนา vision node + STM32 firmware  
- สัปดาห์ 5–6: ทดสอบระบบจริง  
- สัปดาห์ 7: จัดทำรายงานและสรุปผล  

------------------------------------------------------------

## 10. Project Structure

```
hand_ws/
 ├── src/
 │   └── hand_controller/
 │       ├── vision_node.py
 │       └── ...
 ├── firmware/
 │   └── nucleo_uros/
 │       ├── Core/
 │       ├── Drivers/
 │       └── micro_ros_stm32cubemx_utils/
 ├── README.md
 └── ...
```

------------------------------------------------------------

## 11. How to Run

### 1) Start micro-ROS Agent
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

### 2) Run vision node
```
ros2 run hand_controller vision_node
```

### 3) ตรวจสอบค่า
```
ros2 topic echo /finger_states
```

------------------------------------------------------------

## 12. Credits

Project for FRA502 – Robot Control System  
Developed by Daycoolboy  
KMUTT – FIBO Robotics  
```
