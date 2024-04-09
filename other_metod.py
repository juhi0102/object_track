import cv2
import numpy as np
from Pluto import pluto
import time

# Import drone control functions from PlutoMultiwii module
from PlutoMultiwii import *

drone = pluto()


# Define color ranges for red, green, and blue
color_ranges = {
    'red': ([0, 100, 100], [10, 255, 255]),
    'green': ([40, 100, 100], [80, 255, 255]),
    'blue': ([100, 100, 100], [140, 255, 255])
}

# Function to detect color
def detect_color(frame, color):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define lower and upper bounds for the specified color
    lower_bound, upper_bound = color_ranges[color]
    lower_bound = np.array(lower_bound)
    upper_bound = np.array(upper_bound)

    # Threshold the HSV image to get only specified color
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame, frame, mask=mask)
    
    return res

# Open default camera
cap = cv2.VideoCapture(1)

# Flag to track if a command has been executed
command_executed = False

while True:
    # Read input from the Xbox controller
    

    # Read a frame from the camera
    ret, frame = cap.read()

    # Detect colors
    red_detection = detect_color(frame, 'red')
    green_detection = detect_color(frame, 'green')
    blue_detection = detect_color(frame, 'blue')

    # Display the resulting frames
    cv2.imshow('Red Detection', red_detection)
    cv2.imshow('Green Detection', green_detection)
    cv2.imshow('Blue Detection', blue_detection)

    # Drone control based on color detection
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    elif np.any(red_detection) and not command_executed:
        # Takeoff if red color is detected
        drone.take_off()
        command_executed = True
    elif np.any(blue_detection) and not command_executed:
        # Disarm if blue color is detected
        drone.disarm()
        command_executed = True
    elif np.any(green_detection) and not command_executed:
        # Land if green color is detected
        drone.land()
        command_executed = True

    # Reset command flag after 2 seconds to allow for new command
    if command_executed:
        time.sleep(2)
        command_executed = False

# Release the capture
cap.release()
cv2.destroyAllWindows()
