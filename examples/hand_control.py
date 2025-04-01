import cv2
import mediapipe as mp
import numpy as np
import time
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

def get_palm_orientation(hand_landmarks):
    """Get if palm is facing up or down"""
    wrist = hand_landmarks[0]
    middle_mcp = hand_landmarks[9]
    
    # Simple check if palm is facing up or down
    return middle_mcp.y < wrist.y

def recognize_command(hand_landmarks):
    """Recognize the 5 basic commands"""
    if not hand_landmarks:
        return "NO HAND DETECTED"
    
    fingers_up = count_fingers_up(hand_landmarks)
    palm_up = get_palm_orientation(hand_landmarks)
    
    # STAND UP: Palm up, all fingers up
    if palm_up and fingers_up == 5:
        publisher.publish(stopCMD)
        cmd = SvanCommand()
        cmd.command_type = SvanCommand.COMMAND_HEIGHT
        cmd.height = SvanCommand.HEIGHT_UP
        publisher.publish(cmd)
        return "STAND UP"
    
    # MOVE: Two fingers up (index and middle), palm forward
    elif fingers_up == 2 and hand_landmarks[8].y < hand_landmarks[5].y and hand_landmarks[12].y < hand_landmarks[9].y:
        publisher.publish(trotCMD)
        cmd = SvanCommand()
        cmd.command_type = SvanCommand.COMMAND_MOVEMENT
        cmd.direction = SvanCommand.DIRECTION_FORWARD
        cmd.velocity = 0.3
        publisher.publish(cmd)
        return "MOVE"
    
    # STOP: Closed fist, palm forward
    elif fingers_up == 0:
        publisher.publish(stopCMD)
        return "STOP"
    
    # SEARCH: Three fingers up (index, middle, ring)
    elif fingers_up == 3:
        publisher.publish(stopCMD)
        cmd1 = SvanCommand()
        cmd1.command_type = SvanCommand.COMMAND_MOVEMENT
        cmd1.direction = SvanCommand.DIRECTION_FORWARD
        cmd1.velocity = 0.001
        cmd2 = SvanCommand()
        cmd2.command_type = SvanCommand.COMMAND_YAW
        cmd2.direction = SvanCommand.YAW_LEFT
        cmd2.velocity = 0.5
        publisher.publish(cmd1)
        publisher.publish(cmd2)
        return "SEARCH"
    
    # DOWN: Palm down, all fingers up
    elif not palm_up and fingers_up == 5:
        publisher.publish(stopCMD)

        cmd = SvanCommand()
        cmd.command_type = SvanCommand.COMMAND_HEIGHT
        cmd.height = SvanCommand.HEIGHT_DOWN
        publisher.publish(cmd)
        return "DOWN"
    
    return "NO COMMAND"

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

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                # Draw hand landmarks
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Recognize command
                command = recognize_command(hand_landmarks.landmark)

                # Display the command name with background
                text_size = cv2.getTextSize(command, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
                cv2.rectangle(frame, (50, 30), (50 + text_size[0], 50 + text_size[1]), (0, 0, 0), -1)
                cv2.putText(frame, command, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # Display instructions for current command
                instructions = {
                    "STAND UP": "Show palm up, all fingers extended",
                    "MOVE": "Point with index and middle fingers",
                    "STOP": "Show closed fist",
                    "SEARCH": "Show three fingers (index, middle, ring)",
                    "DOWN": "Show palm down, all fingers extended",
                    "NO COMMAND": "Waiting for command gesture...",
                    "NO HAND DETECTED": "Place hand in view"
                }

                
                instruction = instructions.get(command, "")
                cv2.putText(frame, instruction, (50, frame.shape[0] - 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Show the output window
        cv2.imshow("Robot Dog Commands", frame)

        # Quit when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    

# Release camera resources
cap.release()
cv2.destroyAllWindows()
