import cv2
import mediapipe as mp
import numpy as np
from collections import deque

import rospy
from svan_simple_control.msg import SvanCommand

rospy.init_node("hand_control")
trotCMD = SvanCommand()
trotCMD.command_type = SvanCommand.COMMAND_OPERATION_MODE
trotCMD.operation_mode = SvanCommand.MODE_TROT

stopCMD = SvanCommand()
stopCMD.command_type = SvanCommand.COMMAND_MOVEMENT
stopCMD.vel_x = 0.0
stopCMD.vel_y = 0.0


publisher = rospy.Publisher("/svan/simple_control", SvanCommand, queue_size=2)
publisher.publish(trotCMD)
print("Publisher initialized")

current_command = None

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)


def count_fingers_up(hand_landmarks):
    finger_tips = [8, 12, 16, 20]
    finger_pips = [6, 10, 14, 18]
    count = 0

    for tip, pip in zip(finger_tips, finger_pips):
        if hand_landmarks[tip].y < hand_landmarks[pip].y:
            count += 1

    if hand_landmarks[4].x > hand_landmarks[3].x:
        count += 1

    return count


def recognize_command(hand_landmarks):
    if not hand_landmarks:
        return "NO HAND DETECTED"

    fingers_up = count_fingers_up(hand_landmarks)

    if (
        fingers_up == 2
        and hand_landmarks[8].y < hand_landmarks[5].y
        and hand_landmarks[12].y < hand_landmarks[9].y
    ):
        return "MOVE"

    elif fingers_up == 0:
        return "STOP"

    return "NO COMMAND"


class CommandErrorCorrection:
    def __init__(self, buffer_size=10, threshold=0.6):
        self.buffer = deque(maxlen=buffer_size)
        self.threshold = threshold
        self.current_command = "NO COMMAND"
        self.stability_counter = 0
        self.min_stable_frames = 3

    def update(self, new_command):
        if new_command == "NO HAND DETECTED":
            self.print_decision_process("NO HAND DETECTED", {}, None, None)
            return "NO HAND DETECTED"

        self.buffer.append(new_command)

        command_counts = {}
        for cmd in self.buffer:
            if cmd in command_counts:
                command_counts[cmd] += 1
            else:
                command_counts[cmd] = 1

        most_common_command = max(command_counts.items(), key=lambda x: x[1])
        most_common = most_common_command[0]
        count = most_common_command[1]

        ratio = count / len(self.buffer) if len(self.buffer) > 0 else 0
        meets_threshold = ratio >= self.threshold

        previous_command = self.current_command

        if meets_threshold:
            if most_common == self.current_command:
                self.stability_counter += 1
            else:
                self.stability_counter = 1

            if self.stability_counter >= self.min_stable_frames:
                self.current_command = most_common

        self.print_decision_process(new_command, command_counts, most_common, ratio)

        return self.current_command

    def print_decision_process(self, new_command, command_counts, most_common, ratio):
        print("\n" + "=" * 50)
        print(f"NEW INPUT: {new_command}")
        print(f"BUFFER CONTENTS: {list(self.buffer)}")

        if new_command != "NO HAND DETECTED":
            print("\nCOMMAND COUNTS IN BUFFER:")
            for cmd, count in command_counts.items():
                print(
                    f"  {cmd}: {count}/{len(self.buffer)} "
                    + f"({count / len(self.buffer) * 100:.1f}%)"
                )

            if most_common:
                print(
                    f"\nMOST COMMON: {most_common} " + f"({ratio * 100:.1f}% of buffer)"
                )
                print(f"THRESHOLD: {self.threshold * 100:.1f}%")
                print(f"MEETS THRESHOLD: {ratio >= self.threshold}")
                print(
                    f"STABILITY COUNTER: {self.stability_counter}/{self.min_stable_frames}"
                )

        print(f"\nPREVIOUS COMMAND: {self.current_command}")
        print(f"CURRENT COMMAND: {self.current_command}")
        print("=" * 50)


error_correction = CommandErrorCorrection(buffer_size=10, threshold=0.6)

with mp_hands.Hands(
    min_detection_confidence=0.7, min_tracking_confidence=0.7, max_num_hands=1
) as hands:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb_frame)

        raw_command = "NO HAND DETECTED"

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                raw_command = recognize_command(hand_landmarks.landmark)

        corrected_command = error_correction.update(raw_command)

        cv2.putText(
            frame,
            f"Raw: {raw_command}",
            (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

        text_size = cv2.getTextSize(
            f"Corrected: {corrected_command}", cv2.FONT_HERSHEY_SIMPLEX, 1, 2
        )[0]
        cv2.rectangle(
            frame, (50, 80), (50 + text_size[0], 100 + text_size[1]), (0, 0, 0), -1
        )
        cv2.putText(
            frame,
            f"Corrected: {corrected_command}",
            (50, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )

        instructions = {
            "MOVE": "Point with index and middle fingers",
            "STOP": "Show closed fist",
            "NO COMMAND": "Waiting for command gesture...",
            "NO HAND DETECTED": "Place hand in view",
        }
        instruction = instructions.get(corrected_command, "")
        cv2.putText(
            frame,
            instruction,
            (50, frame.shape[0] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        if current_command != corrected_command and corrected_command in [
            "STOP",
            "MOVE",
        ]:
            if current_command == "STOP":
                publisher.publish(stopCMD)
            elif current_command == "MOVE":
                cmd = SvanCommand()
                cmd.command_type = SvanCommand.COMMAND_MOVEMENT
                cmd.vel_x = 0.0
                cmd.vel_y = 0.5
                publisher.publish(cmd)
            current_command = corrected_command

        cv2.imshow("Simplified Robot Commands", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cap.release()
cv2.destroyAllWindows()
