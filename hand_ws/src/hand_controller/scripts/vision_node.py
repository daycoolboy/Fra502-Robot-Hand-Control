#!/bin/python3

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from collections import deque


from plotter import AngleRecorder
rec = AngleRecorder()

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils



def find_camera():
    for i in range(5):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            continue

        ok, frame = cap.read()
        if ok:
            print(f"✅ Camera found at index {i}")
            return cap

        cap.release()

    print("❌ No camera found — skip vision, no plot, no crash.")
    return None



def angle(a, b, c):
    ba = np.array([a.x - b.x, a.y - b.y, a.z - b.z])
    bc = np.array([c.x - b.x, c.y - b.y, c.z - b.z])

    cosang = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    cosang = np.clip(cosang, -1.0, 1.0)
    return np.degrees(np.arccos(cosang))


def angle_to_servo(ang):
    ang = max(60, min(ang, 180))
    x = (ang - 60) / (180 - 60)
    return int(np.sin(x * np.pi / 2) * 180)


def stable(buff):
    return int(sum(buff) / len(buff))


def limit_step(prev, new, step=10):
    if new > prev + step:
        return prev + step
    if new < prev - step:
        return prev - step
    return new




class Vision(Node):
    def __init__(self):
        super().__init__('vision_node')

        # publisher
        self.pub = self.create_publisher(Int32MultiArray, 'finger_states', 10)

        # subscribe feedback from MCU
        self.create_subscription(
            Int32MultiArray,
            "servo_feedback",
            self.feedback_callback,
            10
        )

        self.prev_thumb = 0

    def publish_fingers(self, arr):
        msg = Int32MultiArray()
        msg.data = arr
        self.pub.publish(msg)

    def feedback_callback(self, msg):
        # servo feedback จาก MCU
        rec.add_feedback(list(msg.data))



def main():
    rclpy.init()
    node = Vision()

    # กล้อง auto-detect
    cap = find_camera()
    if cap is None:
        rclpy.shutdown()
        return

    # smoothing buffer
    buff = [
        deque(maxlen=10),
        deque(maxlen=4),
        deque(maxlen=4),
        deque(maxlen=4),
        deque(maxlen=4),
    ]

    with mp_hands.Hands(
        max_num_hands=1,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.6
    ) as hands:

        while cap.isOpened():

            rclpy.spin_once(node, timeout_sec=0)

            ok, frame = cap.read()
            if not ok:
                print("⚠ Frame read failed")
                break

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = hands.process(rgb)

            if result.multi_hand_landmarks:
                hand = result.multi_hand_landmarks[0]

                dx = hand.landmark[4].x - hand.landmark[2].x
                dx_min = -0.09
                dx_max = 0.07

                norm = np.interp(dx, [dx_min, dx_max], [1.0, 0.0])
                norm = np.clip(norm, 0.0, 1.0)

                raw_thumb = int(norm * 180)
                thumb_servo = limit_step(node.prev_thumb, raw_thumb, step=10)
                node.prev_thumb = thumb_servo

                idx_ang = angle(hand.landmark[5], hand.landmark[6], hand.landmark[8])
                mid_ang = angle(hand.landmark[9], hand.landmark[10], hand.landmark[12])
                ring_ang = angle(hand.landmark[13], hand.landmark[14], hand.landmark[16])
                pin_ang = angle(hand.landmark[17], hand.landmark[18], hand.landmark[20])

                index_servo = angle_to_servo(idx_ang)
                middle_servo = angle_to_servo(mid_ang)
                ring_servo = angle_to_servo(ring_ang)
                pinky_servo = angle_to_servo(pin_ang)


                angles = [
                    thumb_servo,
                    index_servo,
                    middle_servo,
                    ring_servo,
                    pinky_servo,
                ]

                for k in range(5):
                    buff[k].append(angles[k])

                final_angles = [stable(buff[k]) for k in range(5)]

                print("Angles:", final_angles)


                rec.add(final_angles)

                node.publish_fingers(final_angles)

                mp_drawing.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

            cv2.imshow("Hand Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


    cap.release()
    cv2.destroyAllWindows()

    rec.plot()         
    rclpy.shutdown()


if __name__ == "__main__":
    main()
