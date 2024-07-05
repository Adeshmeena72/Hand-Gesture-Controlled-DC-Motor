import cv2
import mediapipe as mp
import serial
import time

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

# Set up the serial connection (adjust the COM port as needed)
ser = serial.Serial('COM8', 9600)
time.sleep(2)  # Wait for the connection to establish

# Start capturing video input from the camera
cap = cv2.VideoCapture(0)

# Previous hand positions for gesture detection
prev_x, prev_y = 0, 0

# Function to calculate the area of the hand
def calculate_hand_area(hand_landmarks, image_width, image_height):
    # Get the bounding box coordinates
    bounding_box = mp_hands.HandLandmark
    x_max = max([hand_landmarks.landmark[point].x for point in bounding_box])
    x_min = min([hand_landmarks.landmark[point].x for point in bounding_box])
    y_max = max([hand_landmarks.landmark[point].y for point in bounding_box])
    y_min = min([hand_landmarks.landmark[point].y for point in bounding_box])
    
    # Calculate the area
    area = (x_max - x_min) * (y_max - y_min) * image_width * image_height
    return area

# Function to determine if the hand is a child's hand based on area
def is_child_hand(area):
    # Define the threshold area below which the hand is considered a child's hand
    child_hand_area_threshold = 100000  # This is an example value and needs to be adjusted based on testing
    return area < child_hand_area_threshold

while cap.isOpened():
    success, image = cap.read()
    if not success:
        print("Ignoring empty camera frame.")
        continue


    # Flip the image horizontally for a later selfie-view display
    image = cv2.flip(image, 1)
    
    # Convert the BGR image to RGB
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # Process the image and find hands
    results = hands.process(image)
    image_width, image_height = image.shape[1], image.shape[0]

    # Draw the hand annotations on the image
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
            current_x, current_y = wrist.x, wrist.y
            index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
            
            # Calculate the hand area
            hand_area = calculate_hand_area(hand_landmarks, image_width, image_height)
            
            # Check if it's a child's hand
            if is_child_hand(hand_area):
                ser.write(b'child\n')  # Send 'child' command to Arduino
                continue  # Skip the rest of the loop
            
            # Gesture recognition logic
            if prev_x is not None and prev_y is not None:
                # Moving hand upward
                if index_tip.y < prev_y - 0.05:
                    ser.write(b'up\n')
                # Moving hand downward
                elif index_tip.y > prev_y + 0.05:
                    ser.write(b'down\n')
                # Moving hand to the right
                elif index_tip.x > prev_x + 0.05:
                    ser.write(b'right\n')
                # Moving hand to the left
                elif index_tip.x < prev_x - 0.05:
                    ser.write(b'left\n')

            # Update previous position
            prev_x, prev_y = index_tip.x, index_tip.y
            
            # Check for a closed fist gesture to stop the motor
            if all([hand_landmarks.landmark[finger_tip].y > wrist.y for finger_tip in [0, 0, 0, 0, 0]]):
                ser.write(b'stop\n')# Send 'stop' command to Arduino

    # Show the image
    cv2.imshow('Hand Gesture Recognition', image)
    if cv2.waitKey(5) & 0xFF == 27:
        break

# Clean up
hands.close()
cap.release()
cv2.destroyAllWindows()
ser.close()