#! /usr/bin/env python3

import cv2 as cv
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class HandTeleopNode(Node):
    def __init__(self):
        super().__init__('hand_teleop_node')
        self.publisher = self.create_publisher(Twist, '/a200_0000/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.update)  # Llamada periódica cada 0.1s
        self.vid = cv.VideoCapture(0)
        self.mp_hands = mp.solutions.hands.Hands(model_complexity=0,
                                                 max_num_hands=1,
                                                 min_detection_confidence=0.5,
                                                 min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

    def get_hand_move(self, hand_landmarks):
        """ Detecta si la mano está abierta (gesto 'forward'). """
        landmarks = hand_landmarks.landmark
        
        # Condición para mano abierta (todos los dedos extendidos)
        if all(landmarks[i].y > landmarks[i + 3].y for i in range(5, 20, 4)):
            return "forward"
        elif landmarks[5].y > landmarks[9].y and landmarks[9].y > landmarks[13].y and landmarks[13].y > landmarks[17].y:
            return "turn left"
        elif landmarks[5].y < landmarks[9].y and landmarks[9].y < landmarks[13].y and landmarks[13].y < landmarks[17].y:
            return "turn right"
        elif landmarks[13].y < landmarks[16].y and landmarks[17].y < landmarks[20].y and landmarks[5].y > landmarks[8].y and landmarks[9].y > landmarks[12].y:
            return "home position"
        else:
            return "unknown"

    def update(self):
        """ Captura la imagen, detecta el gesto y publica la velocidad. """
        ret, frame = self.vid.read()
        if not ret or frame is None:
            return

        frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.mp_hands.process(frame)
        frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

        black_frame = np.zeros_like(frame)
        hand_move = "no hand detected"

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(black_frame, 
                                               hand_landmarks, 
                                               mp.solutions.hands.HAND_CONNECTIONS,
                                               self.mp_drawing_styles.get_default_hand_landmarks_style(),
                                               self.mp_drawing_styles.get_default_hand_connections_style())

                hand_move = self.get_hand_move(hand_landmarks)

        black_frame = cv.flip(black_frame, 1)
        cv.putText(black_frame, "Hand Move: " + hand_move, (50, 50), 
                   cv.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 2, cv.LINE_AA)
        cv.imshow('Hand Teleop', black_frame)

        # Publicar velocidad si la mano está abierta
        twist = Twist()
        if hand_move == "forward":
            twist.linear.x = 0.1  # Velocidad hacia adelante
        elif hand_move == "turn left":
            twist.angular.z = 0.1
        elif hand_move == "turn right":
            twist.angular.z = -0.1
        else:
            twist.linear.x = 0.0  # Detenerse

        self.publisher.publish(twist)

        if cv.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            self.vid.release()
            cv.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = HandTeleopNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
