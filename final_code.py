import threading
import cv2
import numpy as np
from Pluto import pluto
import time

from Joystick_control import XboxController
from PlutoMultiwii import *

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

# Function to detect color
def detect_color(frame, color):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_bound, upper_bound = color_ranges[color]
    lower_bound = np.array(lower_bound)
    upper_bound = np.array(upper_bound)
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    return res

# Open default camera
cap = cv2.VideoCapture(0)

# Define color ranges
color_ranges = {
    'red': ([0, 100, 100], [10, 255, 255]),
    'green': ([40, 100, 100], [80, 255, 255]),
    'blue': ([100, 100, 100], [140, 255, 255])
}

while True:
    ret, frame = cap.read()

    red_detection = detect_color(frame, 'red')
    green_detection = detect_color(frame, 'green')
    blue_detection = detect_color(frame, 'blue')

    cv2.imshow('Red Detection', red_detection)
    cv2.imshow('Green Detection', green_detection)
    cv2.imshow('Blue Detection', blue_detection)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    elif np.any(red_detection):
        drone_controller.drone.take_off()
    elif np.any(blue_detection):
        drone_controller.drone.disarm()
    elif np.any(green_detection):
        drone_controller.drone.arm()

# Release the capture
cap.release()
cv2.destroyAllWindows()
