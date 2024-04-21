import cv2
import numpy as np
from pynput import keyboard
from Pluto import pluto
import threading

class DroneController:
    def __init__(self):
        # Initialize Pluto drone object
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
        def on_press(key):
            try:
                if key.char == 'a':
                    self.drone.rcYaw = 1000
                elif key.char == 'd':
                    self.drone.rcYaw = 2000
                elif key.char == 'w':
                    self.drone.rcPitch = 1000
                elif key.char == 's':
                    self.drone.rcPitch = 2000
                elif key.char == 'q':
                    self.drone.rcRoll = 1000
                elif key.char == 'e':
                    self.drone.rcRoll = 2000
                elif key.char == 't':
                    self.drone.take_off()
                    print("Takeoff")
                elif key.char == 'l':
                    self.drone.land()
                    print("Landing")
                elif key.char == 'x':
                    self.drone.disarm()
                elif key.char == 'z':
                    self.drone.decrease_height()
                elif key.char == 'c':
                    self.drone.increase_height()    
            except AttributeError:
                pass

        def on_release(key):
            try:
                if key.char == 'a' or key.char == 'd':
                    self.drone.rcYaw = 1500
                elif key.char == 'w' or key.char == 's':
                    self.drone.rcPitch = 1500
                elif key.char == 'q' or key.char == 'e':
                    self.drone.rcRoll = 1500
            except AttributeError:
                pass

        # Collect events until released
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

# Create an instance of the DroneController class
drone_controller = DroneController()

# Start the control loop in a separate thread
control_thread = threading.Thread(target=drone_controller.control_loop)
control_thread.daemon = True
control_thread.start()


# Global variables to store cursor position
x_pos, y_pos = 0, 0

# Mouse event callback function
def mouse_event(event, x, y, flags, param):
    global x_pos, y_pos
    if event == cv2.EVENT_MOUSEMOVE:
        x_pos, y_pos = x, y

        # Additional movement based on cursor position
        if x_pos < 200:  # Move left
            drone_controller.drone.rcRoll = mapping(x_pos, 0, 200, 1400, 1500)
            print("Moving left:", drone_controller.drone.rcRoll)
        elif x_pos > 500:  # Move right
            drone_controller.drone.rcRoll = mapping(x_pos, 450, 600, 1500, 1600)
            print("Moving right:", drone_controller.drone.rcRoll)
        else:  # Center position
            drone_controller.drone.rcRoll = 1500
            print("Center position")


def mapping(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


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
