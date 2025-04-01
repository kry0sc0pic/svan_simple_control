import cv2
import mediapipe as mp
import numpy as np
from collections import deque

import rospy
from svan_simple_control_msgs.msg import SvanCommand

rospy.init_node('hand_control')
trotCMD = SvanCommand()
trotCMD.command_type = SvanCommand.COMMAND_OPERATION_MODE
trotCMD.operation_mode = SvanCommand.MODE_TROT

stopCMD = SvanCommand()
stopCMD.command_type = SvanCommand.COMMAND_MOVEMENT
stopCMD.direction = SvanCommand.DIRECTION_NONE


publisher = rospy.Publisher('/svan/simple_control', SvanCommand, queue_size=2)
publisher.publish(trotCMD)
print("Publisher initialized")

current_command = None

# Initialize MediaPipe Hands module
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

# Open webcam
cap = cv2.VideoCapture(0)



def count_fingers_up(hand_landmarks):
    """Count number of fingers that are up"""
    finger_tips = [8, 12, 16, 20]  # Index, Middle, Ring, Pinky tips
    finger_pips = [6, 10, 14, 18]  # Index, Middle, Ring, Pinky PIPs
    count = 0
    
    # Check each finger
    for tip, pip in zip(finger_tips, finger_pips):
        if hand_landmarks[tip].y < hand_landmarks[pip].y:
            count += 1
            
    # Special check for thumb
    if hand_landmarks[4].x > hand_landmarks[3].x:
        count += 1
        
    return count

def recognize_command(hand_landmarks):
    """Recognize only STOP and MOVE commands"""
    if not hand_landmarks:
        return "NO HAND DETECTED"
    
    fingers_up = count_fingers_up(hand_landmarks)
    
    # MOVE: Two fingers up (index and middle), palm forward
    if fingers_up == 2 and hand_landmarks[8].y < hand_landmarks[5].y and hand_landmarks[12].y < hand_landmarks[9].y:
        return "MOVE"
    
    # STOP: Closed fist, palm forward
    elif fingers_up == 0:
        return "STOP"
    
    return "NO COMMAND"

# Error correction class
class CommandErrorCorrection:
    def __init__(self, buffer_size=10, threshold=0.6):
        """
        Initialize error correction system
        
        Args:
            buffer_size: Number of recent commands to store
            threshold: Proportion of commands needed to confirm a command
        """
        self.buffer = deque(maxlen=buffer_size)
        self.threshold = threshold
        self.current_command = "NO COMMAND"
        self.stability_counter = 0
        self.min_stable_frames = 3  # Minimum number of stable frames before changing command
    
    def update(self, new_command):
        """
        Update with a new command detection and return the corrected command
        """
        # Skip NO HAND DETECTED
        if new_command == "NO HAND DETECTED":
            self.print_decision_process("NO HAND DETECTED", {}, None, None)
            return "NO HAND DETECTED"
            
        # Add the new command to our buffer
        self.buffer.append(new_command)
        
        # Count occurrences of each command in the buffer
        command_counts = {}
        for cmd in self.buffer:
            if cmd in command_counts:
                command_counts[cmd] += 1
            else:
                command_counts[cmd] = 1
        
        # Find the most common command
        most_common_command = max(command_counts.items(), key=lambda x: x[1])
        most_common = most_common_command[0]
        count = most_common_command[1]
        
        # Check if it meets our threshold
        ratio = count / len(self.buffer) if len(self.buffer) > 0 else 0
        meets_threshold = ratio >= self.threshold
        
        previous_command = self.current_command
        
        if meets_threshold:
            # Check if this is the same as our current command
            if most_common == self.current_command:
                self.stability_counter += 1
            else:
                # Reset stability counter for new potential command
                self.stability_counter = 1
                
            # Only change command if it's been stable for multiple frames
            if self.stability_counter >= self.min_stable_frames:
                self.current_command = most_common
        
        # Print decision process to terminal
        self.print_decision_process(new_command, command_counts, most_common, ratio)
        
        return self.current_command
    
    def print_decision_process(self, new_command, command_counts, most_common, ratio):
        """Print the decision-making process to the terminal"""
        print("\n" + "="*50)
        print(f"NEW INPUT: {new_command}")
        print(f"BUFFER CONTENTS: {list(self.buffer)}")
        
        if new_command != "NO HAND DETECTED":
            print("\nCOMMAND COUNTS IN BUFFER:")
            for cmd, count in command_counts.items():
                print(f"  {cmd}: {count}/{len(self.buffer)} " + 
                      f"({count/len(self.buffer)*100:.1f}%)")
            
            if most_common:
                print(f"\nMOST COMMON: {most_common} " + 
                      f"({ratio*100:.1f}% of buffer)")
                print(f"THRESHOLD: {self.threshold*100:.1f}%")
                print(f"MEETS THRESHOLD: {ratio >= self.threshold}")
                print(f"STABILITY COUNTER: {self.stability_counter}/{self.min_stable_frames}")
        
        print(f"\nPREVIOUS COMMAND: {self.current_command}")
        print(f"CURRENT COMMAND: {self.current_command}")
        print("="*50)

# Initialize error correction
error_correction = CommandErrorCorrection(buffer_size=10, threshold=0.6)

# Start capturing video
with mp_hands.Hands(
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7,
    max_num_hands=1
) as hands:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Convert frame to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb_frame)

        # Default raw command when no hand is detected
        raw_command = "NO HAND DETECTED"
        
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                # Draw hand landmarks
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Recognize raw command
                raw_command = recognize_command(hand_landmarks.landmark)
        
        # Apply error correction
        corrected_command = error_correction.update(raw_command)

        # Display both raw and corrected commands
        cv2.putText(frame, f"Raw: {raw_command}", (50, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Make the corrected command more prominent
        text_size = cv2.getTextSize(f"Corrected: {corrected_command}", cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
        cv2.rectangle(frame, (50, 80), (50 + text_size[0], 100 + text_size[1]), (0, 0, 0), -1)
        cv2.putText(frame, f"Corrected: {corrected_command}", (50, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Display instructions for current command
        instructions = {
            "MOVE": "Point with index and middle fingers",
            "STOP": "Show closed fist",
            "NO COMMAND": "Waiting for command gesture...",
            "NO HAND DETECTED": "Place hand in view"
        }
        instruction = instructions.get(corrected_command, "")
        cv2.putText(frame, instruction, (50, frame.shape[0] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        if current_command != corrected_command and corrected_command in ["STOP", "MOVE"]:
            if current_command == "STOP":
                publisher.publish(stopCMD)
            elif current_command == "MOVE":
                cmd = SvanCommand()
                cmd.command_type = SvanCommand.COMMAND_MOVEMENT
                cmd.direction = SvanCommand.DIRECTION_FORWARD
                publisher.publish(cmd)
            current_command = corrected_command

        # Show the output window
        cv2.imshow("Simplified Robot Commands", frame)

        # Quit when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release camera resources
cap.release()
cv2.destroyAllWindows()
