#!/usr/bin/env python3
import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from collections import deque

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils


# ===============================
#  Finger Logic
# ===============================

def finger_is_closed(hand_landmarks, idx_tip, idx_pip):
    tip = hand_landmarks.landmark[idx_tip].y
    pip = hand_landmarks.landmark[idx_pip].y
    return tip > pip    


def thumb_is_closed(hand_landmarks):
    tip = hand_landmarks.landmark[4].x
    ip  = hand_landmarks.landmark[3].x
    return tip < ip     


# ===============================
# Debounce / Stabilizer
# ===============================

def stable(buff):
    """Return majority value (0/1) of the buffer"""
    return 1 if sum(buff) > len(buff) / 2 else 0


# ===============================
# ROS2 Node
# ===============================

class FingerArrayPublisher(Node):
    def __init__(self):
        super().__init__('finger_array_pub')
        self.pub = self.create_publisher(Int32MultiArray, 'finger_states', 10)

    def publish_fingers(self, arr):
        msg = Int32MultiArray()
        msg.data = arr
        self.pub.publish(msg)


# ===============================
# Main
# ===============================

def main():
    rclpy.init()
    node = FingerArrayPublisher()

    cap = cv2.VideoCapture(0)

    # Create 3-frame buffer for each finger
    buff_t = deque(maxlen=3)
    buff_i = deque(maxlen=3)
    buff_m = deque(maxlen=3)
    buff_r = deque(maxlen=3)
    buff_p = deque(maxlen=3)

    with mp_hands.Hands(
        max_num_hands=1,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.6
    ) as hands:

        while cap.isOpened():
            rclpy.spin_once(node, timeout_sec=0)

            ok, frame = cap.read()
            if not ok:
                continue

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = hands.process(rgb)

            if result.multi_hand_landmarks:
                hand = result.multi_hand_landmarks[0]

                # Detect each finger
                thumb_closed  = not thumb_is_closed(hand)
                index_closed  = finger_is_closed(hand, 8, 6)
                middle_closed = finger_is_closed(hand, 12, 10)
                ring_closed   = finger_is_closed(hand, 16, 14)
                pinky_closed  = finger_is_closed(hand, 20, 18)

                # Convert to 0/1 (1=open, 0=closed)
                thumb  = int(not thumb_closed)
                index  = int(not index_closed)
                middle = int(not middle_closed)
                ring   = int(not ring_closed)
                pinky  = int(not pinky_closed)

                # Add into buffer
                buff_t.append(thumb)
                buff_i.append(index)
                buff_m.append(middle)
                buff_r.append(ring)
                buff_p.append(pinky)

                # Use stable output (majority vote)
                thumb_s  = stable(buff_t)
                index_s  = stable(buff_i)
                middle_s = stable(buff_m)
                ring_s   = stable(buff_r)
                pinky_s  = stable(buff_p)

                stable_arr = [thumb_s, index_s, middle_s, ring_s, pinky_s]

                print("State: ", stable_arr)

                # ส่งเป็น array ไป micro-ROS
                node.publish_fingers(stable_arr)

                # Draw skeleton
                mp_drawing.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

            cv2.imshow("Hand Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
