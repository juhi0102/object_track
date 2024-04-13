
import threading
import time
import cv2
import numpy as np
from Pluto import pluto  # Assuming Pluto class is defined in Pluto.py file
from Joystick_controls import XboxController
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

# Initialize camera parameters
width = 640  # Width of the image
height = 480  # Height of the image
deadZone = 100
me = pluto()
startCounter =0
# Initialize Pluto commands
me.rcPitch = 0
me.rcRoll = 0
me.rcThrottle = 0
me.rcYaw = 0

# Initialize camera
frameWidth = width
frameHeight = height
cap = cv2.VideoCapture(0)
cap.set(3, frameWidth)
cap.set(4, frameHeight)
cap.set(10, 200)

# Initialize global variables for object tracking
global imgContour
global dir
global start_time
start_time = None

# Function to handle empty trackbar callback
def empty(a):
    pass

# Create HSV trackbars for object detection
cv2.namedWindow("HSV")
cv2.resizeWindow("HSV", 640, 240)
cv2.createTrackbar("HUE Min", "HSV", 20, 179, empty)
cv2.createTrackbar("HUE Max", "HSV", 40, 179, empty)
cv2.createTrackbar("SAT Min", "HSV", 148, 255, empty)
cv2.createTrackbar("SAT Max", "HSV", 255, 255, empty)
cv2.createTrackbar("VALUE Min", "HSV", 89, 255, empty)
cv2.createTrackbar("VALUE Max", "HSV", 255, 255, empty)

# Create trackbars for Canny edge detection parameters
cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters", 640, 240)
cv2.createTrackbar("Threshold1", "Parameters", 166, 255, empty)
cv2.createTrackbar("Threshold2", "Parameters", 171, 255, empty)
cv2.createTrackbar("Area", "Parameters", 1750, 30000, empty)

# Function to stack images horizontally
def stackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2:
                    imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank] * rows
        hor_con = [imageBlank] * rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2:
                imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver

# Function to get contours from the image
def getContours(img, imgContour):
    global dir, start_time
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        areaMin = cv2.getTrackbarPos("Area", "Parameters")
        if area > areaMin:
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 7)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            x, y, w, h = cv2.boundingRect(approx)
            cx = int(x + (w / 2))  # CENTER X OF THE OBJECT
            cy = int(y + (h / 2))  # CENTER X OF THE OBJECT

            if cx < int(frameWidth / 2) - deadZone:
                cv2.putText(imgContour, " GO LEFT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                cv2.rectangle(imgContour, (0, int(frameHeight / 2 - deadZone)), (int(frameWidth / 2) - deadZone, int(frameHeight / 2) + deadZone), (0, 0, 255), cv2.FILLED)
                dir = 1
                start_time = time.time()
            elif cx > int(frameWidth / 2) + deadZone:
                cv2.putText(imgContour, " GO RIGHT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                cv2.rectangle(imgContour, (int(frameWidth / 2 + deadZone), int(frameHeight / 2 - deadZone)), (frameWidth, int(frameHeight / 2) + deadZone), (0, 0, 255), cv2.FILLED)
                dir = 2
                start_time = time.time()
            elif cy < int(frameHeight / 2) - deadZone:
                cv2.putText(imgContour, " GO UP ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                cv2.rectangle(imgContour, (int(frameWidth / 2 - deadZone), 0), (int(frameWidth / 2 + deadZone), int(frameHeight    / 2) - deadZone), (0, 0, 255), cv2.FILLED)
                dir = 3
                start_time = time.time()
            elif cy > int(frameHeight / 2) + deadZone:
                cv2.putText(imgContour, " GO DOWN ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                cv2.rectangle(imgContour, (int(frameWidth / 2 - deadZone), int(frameHeight / 2) + deadZone), (int(frameWidth / 2 + deadZone), frameHeight), (0, 0, 255), cv2.FILLED)
                dir = 4
                start_time = time.time()
            else:
                dir = 0

            cv2.line(imgContour, (int(frameWidth / 2), int(frameHeight / 2)), (cx, cy), (0, 0, 255), 3)
            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 5)
            cv2.putText(imgContour, "Points: " + str(len(approx)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7, (0, 255, 0), 2)
            cv2.putText(imgContour, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(imgContour, " " + str(int(x)) + " " + str(int(y)), (x - 20, y - 45), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
        else:
            dir = 0

# Function to display directional indicators on the image
def display(img):
    cv2.line(img, (int(frameWidth / 2) - deadZone, 0), (int(frameWidth / 2) - deadZone, frameHeight), (255, 255, 0), 3)
    cv2.line(img, (int(frameWidth / 2) + deadZone, 0), (int(frameWidth / 2) + deadZone, frameHeight), (255, 255, 0), 3)
    cv2.circle(img, (int(frameWidth / 2), int(frameHeight / 2)), 5, (0, 0, 255), 5)
    cv2.line(img, (0, int(frameHeight / 2) - deadZone), (frameWidth, int(frameHeight / 2) - deadZone), (255, 255, 0), 3)
    cv2.line(img, (0, int(frameHeight / 2) + deadZone), (frameWidth, int(frameHeight / 2) + deadZone), (255, 255, 0), 3)



while True:
    # Get the image from the camera
    ret, img = cap.read()
    imgContour = img.copy()
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Get HSV trackbar positions
    h_min = cv2.getTrackbarPos("HUE Min", "HSV")
    h_max = cv2.getTrackbarPos("HUE Max", "HSV")
    s_min = cv2.getTrackbarPos("SAT Min", "HSV")
    s_max = cv2.getTrackbarPos("SAT Max", "HSV")
    v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
    v_max = cv2.getTrackbarPos("VALUE Max", "HSV")

    # Create lower and upper HSV boundaries
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(imgHsv, lower, upper)
    result = cv2.bitwise_and(img, img, mask=mask)
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    # Apply Gaussian blur and Canny edge detection
    imgBlur = cv2.GaussianBlur(result, (7, 7), 1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
    threshold1 = cv2.getTrackbarPos("Threshold1", "Parameters")
    threshold2 = cv2.getTrackbarPos("Threshold2", "Parameters")
    imgCanny = cv2.Canny(imgGray, threshold1, threshold2)
    kernel = np.ones((5, 5))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
    getContours(imgDil, imgContour)
    display(imgContour)

    if startCounter == 0:
       me.take_off()
       startCounter = 1

   
# Calculate elapsed time since the drone started moving in a direction
    elapsed_time = time.time() - start_time if start_time is not None else 0.0

    # Flight control based on object tracking, considering time limit
    if dir == 1 and elapsed_time < 0.01:
        me.left()  # Adjust roll to make the drone move left
    elif dir == 2 and elapsed_time < 0.01:
        me.right()  # Adjust roll to make the drone move right
    elif dir == 3 and elapsed_time < 0.01:
        me.forward()  # Adjust pitch to make the drone move forward
    elif dir == 4 and elapsed_time < 0.01:
        me.land()  # Adjust throttle to make the drone move down
    else:
        # Stop movement if no direction is specified or time limit exceeded
        me.reset()

    # Stack images for display
    stack = stackImages(0.6, ([img, result], [imgDil, imgContour]))
    cv2.imshow('Horizontal Stacking', stack)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Land the drone before closing the program
me.land()

# Release the capture and close all windows
cap.release()
cv2.destroyAllWindows()
