
import cv2
import mediapipe as mp
import numpy as np
from collections import deque
import requests

curr_state = 0
hand_position = "STRAIGHT"
SEND_COMMANDS = True  # Flag to enable/disable sending commands to endpoints, default is False

# Initialize MediaPipe Hands module
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

SIM = False
if SIM:
    SVAN_BASE = 'http://localhost:8888'
else:
    SVAN_BASE = 'http://10.42.4.9:8888'

# Open webcam
cap = cv2.VideoCapture(0)

def count_fingers_up(hand_landmarks):
    """Count number of fingers that are up"""
    finger_tips = [8, 12, 16, 20]  # Index, Middle, Ring, Pinky tips
    finger_pips = [6, 10, 14, 18]  # Index, Middle, Ring, Pinky PIPs
    count = 0
    
    for tip, pip in zip(finger_tips, finger_pips):
        if hand_landmarks[tip].y < hand_landmarks[pip].y:
            count += 1
            
    if hand_landmarks[4].x > hand_landmarks[3].x:
        count += 1
        
    return count

def calculate_angle(wrist, index_finger, middle_finger):
    """Calculate the angle between wrist and the average of two fingers (index and middle)"""
    # Get vectors from wrist to both fingers
    vector_index = np.array([index_finger.x - wrist.x, index_finger.y - wrist.y])
    vector_middle = np.array([middle_finger.x - wrist.x, middle_finger.y - wrist.y])
    
    # Average vector between the index and middle fingers
    avg_vector = (vector_index + vector_middle) / 2.0
    
    # Horizontal vector along the wrist (x-axis)
    horizontal_vector = np.array([1.0, 0.0])  # Horizontal line along x-axis
    
    # Dot product and magnitudes
    dot_product = np.dot(avg_vector, horizontal_vector)
    magnitude_avg_vector = np.linalg.norm(avg_vector)
    magnitude_horizontal = np.linalg.norm(horizontal_vector)

    # Calculate the angle (in degrees)
    cos_theta = dot_product / (magnitude_avg_vector * magnitude_horizontal)
    angle = np.arccos(np.clip(cos_theta, -1.0, 1.0))  # Ensure the value is between -1 and 1
    return np.degrees(angle)

def recognize_command(hand_landmarks):
    """Recognize only STOP and MOVE commands"""
    if not hand_landmarks:
        return "NO HAND DETECTED"
    
    fingers_up = count_fingers_up(hand_landmarks)
    
    if fingers_up == 2 and hand_landmarks[8].y < hand_landmarks[5].y and hand_landmarks[12].y < hand_landmarks[9].y:
        return "MOVE"
    
    elif fingers_up == 0:
        return "STOP"
    
    return "NO COMMAND"

# Error correction class
class CommandErrorCorrection:
    def __init__(self, buffer_size=10, threshold=0.6):
        self.buffer = deque(maxlen=buffer_size)
        self.threshold = threshold
        self.current_command = "NO COMMAND"
        self.stability_counter = 0
        self.min_stable_frames = 2  # Minimum number of stable frames before changing command
    
    def update(self, new_command):
        if new_command == "NO HAND DETECTED":
            return "NO HAND DETECTED"
            
        self.buffer.append(new_command)
        
        command_counts = {}
        for cmd in self.buffer:
            command_counts[cmd] = command_counts.get(cmd, 0) + 1
        
        most_common, count = max(command_counts.items(), key=lambda x: x[1])
        ratio = count / len(self.buffer)
        meets_threshold = ratio >= self.threshold

        if meets_threshold:
            if most_common == self.current_command:
                self.stability_counter += 1
            else:
                if self.stability_counter < self.min_stable_frames:
                    self.stability_counter += 1  # Gradual change instead of hard reset

            if self.stability_counter >= self.min_stable_frames:
                self.current_command = most_common

        return self.current_command

# Initialize error correction
error_correction = CommandErrorCorrection(buffer_size=10, threshold=0.6)

# Define angle thresholds (in degrees)
LEFT_THRESHOLD = 75.0    # Angle less than this means "left"
RIGHT_THRESHOLD = 105.0   # Angle greater than this means "right"

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

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb_frame)

        raw_command = "NO HAND DETECTED"
        hand_angle = 0.0  # Default angle is 0.0
          # Default position is "straight"
        
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                raw_command = recognize_command(hand_landmarks.landmark)
                
                # If MOVE command, calculate the angle using both the index and middle fingers
                if raw_command == "MOVE":
                    wrist = hand_landmarks.landmark[0]
                    index_finger = hand_landmarks.landmark[8]
                    middle_finger = hand_landmarks.landmark[12]
                    hand_angle = calculate_angle(wrist, index_finger, middle_finger)
                    
                    # Classify the hand position (left, straight, right) based on the angle
                    if hand_angle < LEFT_THRESHOLD:
                        if hand_position != 'LEFT':
                            requests.post(SVAN_BASE+"/yaw",json={
                                'yaw': 0
                            })
                        hand_position = "LEFT"
                    elif hand_angle > RIGHT_THRESHOLD:
                        if hand_position != 'RIGHT':
                            requests.post(SVAN_BASE+"/yaw",json={
                                    'yaw': 1
                                })
                        hand_position = "RIGHT"
                    else:
                        if hand_position != 'STRAIGHT':
                            requests.post(SVAN_BASE+"/yaw",json={
                                    'yaw': 2
                                })
                        hand_position = "STRAIGHT"
                        
                    
                    # Draw the horizontal line at the wrist
                    wrist_pos = (int(wrist.x * frame.shape[1]), int(wrist.y * frame.shape[0]))
                    cv2.line(frame, (0, wrist_pos[1]), (frame.shape[1], wrist_pos[1]), (255, 0, 0), 2)
                    
                    # Draw lines to index and middle fingers
                    index_pos = (int(index_finger.x * frame.shape[1]), int(index_finger.y * frame.shape[0]))
                    middle_pos = (int(middle_finger.x * frame.shape[1]), int(middle_finger.y * frame.shape[0]))
                    cv2.line(frame, wrist_pos, index_pos, (0, 255, 0), 2)
                    cv2.line(frame, wrist_pos, middle_pos, (0, 255, 0), 2)

        corrected_command = error_correction.update(raw_command)
        
        # Flip the frame horizontally
        frame = cv2.flip(frame, 1)
        # Display angle and hand position if MOVE command is recognized
        if corrected_command == 'MOVE':
            cv2.putText(frame, f"Angle: {hand_angle:.2f} degrees", (50, 150), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            cv2.putText(frame, f"Position: {hand_position}", (50, 180), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

            if SEND_COMMANDS and curr_state == 0:
                # Use the steering angle to control the robot movement
                requests.post(SVAN_BASE+'/movement',json={
                    'vel_x': 0.0,  # Adjust speed based on angle
                    'vel_y': 0.2  # Maintain forward motion
                })
                curr_state = 1
            
        elif corrected_command == 'STOP':
            if SEND_COMMANDS and curr_state == 1:
                requests.post(SVAN_BASE+"/mode",json={
                    'operation_mode': 1
                })
                curr_state = 0

        else:
            if corrected_command in ['NO COMMAND','NO HAND DETECTED'] and curr_state == 1:
                if SEND_COMMANDS:
                    requests.post(SVAN_BASE+"/mode",json={
                        'operation_mode': 1
                    })
                curr_state = 0
                
        cv2.putText(frame, f"Raw: {raw_command}", (50, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        text_size = cv2.getTextSize(f"Corrected: {corrected_command}", cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
        cv2.rectangle(frame, (50, 80), (50 + text_size[0], 100 + text_size[1]), (0, 0, 0), -1)
        cv2.putText(frame, f"Corrected: {corrected_command}", (50, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        instructions = {
            "MOVE": "Point with index and middle fingers",
            "STOP": "Show closed fist",
            "NO COMMAND": "Waiting for command gesture...",
            "NO HAND DETECTED": "Place hand in view"
        }
        instruction = instructions.get(corrected_command, "")
        cv2.putText(frame, instruction, (50, frame.shape[0] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


        cv2.imshow("Simplified Robot Commands", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()

