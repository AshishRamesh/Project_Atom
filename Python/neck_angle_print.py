import cv2
import mediapipe as mp
import numpy as np
import requests

ESP8266_IP = '192.168.87.128'  # Adjust to match your ESP8266's IP address
ESP8266_PORT = '8080'

# Initialize Mediapipe pose model
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(static_image_mode=False, model_complexity=1, smooth_landmarks=True, 
                    min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Start capturing video
cap = cv2.VideoCapture(0)

# Stabilization parameters
stabilized_angle = 90
smoothing_factor = 0.9  # Adjust to control stability; closer to 1 = more stability
movement_threshold = 2  # Degrees of change to consider as significant movement

# Variable to hold the previous stabilized angle
previous_angle = stabilized_angle

# Function to calculate the neck angle
def calculate_neck_angle(left_shoulder, right_shoulder, neck):
    mid_shoulder_x = (left_shoulder[0] + right_shoulder[0]) / 2
    neck_offset = neck[0] - mid_shoulder_x
    neck_angle = 90 + (90 * neck_offset) / abs(right_shoulder[0] - left_shoulder[0])
    neck_angle = np.clip(neck_angle, 0, 180)
    return neck_angle

# Main video capture loop
while cap.isOpened():
    success, frame = cap.read()
    if not success:
        print("Ignoring empty frame.")
        continue

    frame = cv2.flip(frame, 1)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(frame_rgb)
    
    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark
        left_shoulder = (landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].x * frame.shape[1], 
                         landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y * frame.shape[0])
        right_shoulder = (landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].x * frame.shape[1], 
                          landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y * frame.shape[0])
        neck = (landmarks[mp_pose.PoseLandmark.NOSE].x * frame.shape[1], 
                landmarks[mp_pose.PoseLandmark.NOSE].y * frame.shape[0])

        # Calculate the raw neck angle and apply smoothing
        raw_neck_angle = calculate_neck_angle(left_shoulder, right_shoulder, neck)
        angle_difference = abs(raw_neck_angle - stabilized_angle)

        # Apply smoothing if there's significant movement
        if angle_difference > movement_threshold:
            stabilized_angle = (smoothing_factor * stabilized_angle) + ((1 - smoothing_factor) * raw_neck_angle)

        # Convert stabilized angle to integer for sending
        stabilized_angle_int = int(round(stabilized_angle))

        # Check if the stabilized angle is different from the previous angle before sending
        if stabilized_angle_int != previous_angle:
            # Send the stabilized angle to the ESP8266
            try:
                response = requests.get(f'http://{ESP8266_IP}:{ESP8266_PORT}/update?angle={stabilized_angle_int}')
                print(f"Sent Angle: {stabilized_angle_int}, Response: {response.text}")
                previous_angle = stabilized_angle_int  # Update previous angle
            except requests.exceptions.RequestException as e:
                print("Failed to send data to ESP8266:", e)

        # Display the calculated neck angle on the video feed
        cv2.putText(frame, f"Neck Angle: {stabilized_angle:.2f} degrees", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Display the video feed with the neck angle overlay
    cv2.imshow("Robot Neck Angle", frame)
    
    # Press 'Esc' to exit the loop
    if cv2.waitKey(5) & 0xFF == 27:
        break

# Release the capture and destroy all windows
cap.release()
cv2.destroyAllWindows()
