import cv2
import pyautogui
import numpy as np
import threading
from Joystick_controls import XboxController
from Pluto import pluto

class DroneController:
    def __init__(self):
        # Initialize Xbox controller and Pluto drone objects
        self.joy = XboxController()
        self.drone = pluto()

    def mapping(self, x, inMin, inMax, outMin, outMax):
        x = (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin
        if x < outMin:
            return int(outMin)
        elif x > outMax:
            return int(outMax)
        else:
            return int(x)

    def control_loop(self):
        while True:
            [x, y, a, b, A, B, X, Y, rb, lb, rt, lt, ld, rd, ud, dd] = self.joy.read()

            self.drone.rcThrottle = self.mapping(y, 1, -1, 1000, 2000)
            self.drone.rcYaw = self.mapping(x, -1, 1, 1000, 2000)
            self.drone.rcPitch = self.mapping(b, 1, -1, 1000, 2000)
            self.drone.rcRoll = self.mapping(a, -1, 1, 1000, 2000)

            if A:
                self.drone.arm()
                print("Arming")
            elif B:
                self.drone.disarm()
                print("Disarming")
            elif Y:
                self.drone.take_off()
                print("Takeoff")
            elif X:
                self.drone.land()
                print("Landing")

# Create an instance of the DroneController class
drone_controller = DroneController()

# Start the joystick control loop in a separate thread
joystick_thread = threading.Thread(target=drone_controller.control_loop)
joystick_thread.daemon = True
joystick_thread.start()

# Global variables to store cursor position
x_pos, y_pos = 0, 0

def mouse_event(event, x, y, flags, param):
    global x_pos, y_pos
    if event == cv2.EVENT_MOUSEMOVE:
        x_pos, y_pos = x, y
        # Map cursor position to normalized range [-1, 1]
        normalized_x = mapping(x_pos, 0, 800, -1, 1)

        # Additional movement based on cursor position
        if x_pos < 100:  # Move left
            roll_value = mapping(normalized_x, -1, 0, -1, 0)
            print(f"Moving Left: {roll_value}")
        elif x_pos > 500:  # Move right
            roll_value = mapping(normalized_x, 0, 1, 0, 1)
            print(f"Moving Right: {roll_value}")
        else:  # Center position
            roll_value = 0

        # Map the normalized roll value to the range [1000, 2000]
        rcRoll = mapping(roll_value, -1, 1, 1000, 2000)

        # Set the rcRoll value
        drone_controller.drone.rcRoll = int(rcRoll)

# Map a value from one range to another
def mapping(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min



# Initialize webcam
cap = cv2.VideoCapture(0)

# Set mouse event callback
cv2.namedWindow('Webcam with Quadrants')
cv2.setMouseCallback('Webcam with Quadrants', mouse_event)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Get frame dimensions
    height, width, _ = frame.shape

    # Calculate quadrant dimensions
    quadrant_height = height // 3
    quadrant_width = width // 3

    # Draw vertical lines to split into 3 columns
    cv2.line(frame, (quadrant_width, 0), (quadrant_width, height), (255, 0, 0), 1)
    cv2.line(frame, (2 * quadrant_width, 0), (2 * quadrant_width, height), (255, 0, 0), 1)

    # Draw horizontal lines to split into 3 rows
    cv2.line(frame, (0, quadrant_height), (width, quadrant_height), (255, 0, 0), 1)
    cv2.line(frame, (0, 2 * quadrant_height), (width, 2 * quadrant_height), (255, 0, 0), 1)

    # Display cursor position
    cv2.putText(frame, f'X: {x_pos}, Y: {y_pos}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # Display the frame
    cv2.imshow('Webcam with Quadrants', frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release webcam and close all windows
cap.release()
cv2.destroyAllWindows()
