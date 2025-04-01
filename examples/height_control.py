import cv2
import mediapipe as mp
import rospy
from svan_simple_control_msgs.msg import SvanCommand
import time

# Initialize ROS node
rospy.init_node('height_control')

# Create a publisher to send commands
cmd = SvanCommand()
cmd.command_type = SvanCommand.COMMAND_OPERATION_MODE
cmd.operation_mode = SvanCommand.MODE_TROT
publisher = rospy.Publisher('/svan/simple_control', SvanCommand, queue_size=1)
publisher.publish(cmd)
current = 'sit'
current = 'stand'
# Initialize MediaPipe Hand Tracking
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

# Open webcam
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to RGB
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Extract key landmarks
            wrist = hand_landmarks.landmark[0]  # Wrist
            index_finger_tip = hand_landmarks.landmark[8]  # Index finger tip

            # Determine the hand height relative to the wrist
            if index_finger_tip.y < wrist.y - 0.1 and current == 'sit':  
                # cmd_pub.publish("stand")
                # key_data.data[8] = 1
                cmd = SvanCommand()
                cmd.command_type = SvanCommand.COMMAND_HEIGHT
                cmd.height = SvanCommand.HEIGHT_UP
                publisher.publish(cmd)
                print("Command: Stand")
                current = 'stand'
            elif index_finger_tip.y > wrist.y + 0.1 and current == 'stand':
                cmd = SvanCommand()
                cmd.command_type = SvanCommand.COMMAND_HEIGHT
                cmd.height = SvanCommand.HEIGHT_DOWN
                publisher.publish(cmd)
                print("Command: Sit")
                current = 'sit'
    # Show the camera feed
    cv2.imshow("Gesture Control", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
