import pygame
import threading
import time
from Pluto import pluto
import cv2
import numpy as np

# Function to map input range to output range
def mapping(x, inMin, inMax, outMin, outMax): 
    x = (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin
    if x < outMin:
        return int(outMin)
    elif x > outMax:
        return int(outMax)
    else:
        return int(x)

class XboxController(object):
    MAX_TRIG_VAL = 255
    MAX_JOY_VAL = 32768

    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(self):
        pygame.event.pump()
        self.LeftJoystickX = self.joystick.get_axis(0)
        self.LeftJoystickY = -self.joystick.get_axis(1)
        self.RightJoystickX = self.joystick.get_axis(3)
        self.RightJoystickY = -self.joystick.get_axis(4)
        self.LeftTrigger = max(0, self.joystick.get_axis(2))
        self.RightTrigger = max(0, self.joystick.get_axis(5))
        self.A = self.joystick.get_button(0)
        self.B = self.joystick.get_button(1)
        self.X = self.joystick.get_button(2)
        self.Y = self.joystick.get_button(3)
        self.LeftBumper = self.joystick.get_button(4)
        self.RightBumper = self.joystick.get_button(5)
        self.Back = self.joystick.get_button(6)
        self.Start = self.joystick.get_button(7)
        self.LeftThumb = self.joystick.get_button(8)
        self.RightThumb = self.joystick.get_button(9)
        self.LeftDPad = self.joystick.get_button(10)
        self.RightDPad = self.joystick.get_button(11)
        self.UpDPad = self.joystick.get_button(12)
        self.DownDPad = self.joystick.get_button(13)
        return [self.LeftJoystickX, self.LeftJoystickY, self.RightJoystickX, self.RightJoystickY,
                self.A, self.B, self.X, self.Y, self.RightBumper, self.LeftBumper]

    def _monitor_controller(self):
        while True:
            self.read()
            time.sleep(0.01)

def joystick_control():
    joy = XboxController()
    while True:
        # Read input from the Xbox controller
        [x, y, a, b, A, B, X, Y, rb, lb] = joy.read()

        # Map controller input to drone controls
        me.rcThrottle = mapping(y, 1, -1, 1000, 2000)
        me.rcYaw = mapping(x, -1, 1, 1000, 2000)
        me.rcPitch = mapping(b, 1, -1, 1000, 2000)
        me.rcRoll = mapping(a, -1, 1, 1000, 2000)

        # Check button states for drone actions
        print(me.rcRoll)
        if A:
            me.arm()  # Arm the drone
            print("arming", A)
        elif B:
            me.disarm()  # Disarm the drone
            print("disarming", B)
        elif Y:
            me.take_off()  # Take off the drone
            print("taken off", X)
        elif X:
            me.land()  # Land the drone
            me.disarm()
            print("landing", Y)

        time.sleep(0.01)

######################################################################
width = 640  # WIDTH OF THE IMAGE
height = 480  # HEIGHT OF THE IMAGE
deadZone = 100
######################################################################

me = pluto()

me.rcPitch = 0
me.rcRoll = 0
me.rcThrottle = 0
me.rcYaw = 0

frameWidth = width
frameHeight = height
cap = cv2.VideoCapture(0)
cap.set(3, frameWidth)
cap.set(4, frameHeight)
cap.set(10, 200)

global imgContour
global dir;

def empty(a):
    pass

cv2.namedWindow("HSV")
cv2.resizeWindow("HSV", 640, 240)
cv2.createTrackbar("HUE Min", "HSV", 20, 179, empty)
cv2.createTrackbar("HUE Max", "HSV", 40, 179, empty)
cv2.createTrackbar("SAT Min", "HSV", 148, 255, empty)
cv2.createTrackbar("SAT Max", "HSV", 255, 255, empty)
cv2.createTrackbar("VALUE Min", "HSV", 89, 255, empty)
cv2.createTrackbar("VALUE Max", "HSV", 255, 255, empty)

cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters", 640, 240)

cv2.createTrackbar("Threshold1", "Parameters", 166, 255, empty)
cv2.createTrackbar("Threshold2", "Parameters", 171, 255, empty)
cv2.createTrackbar("Area", "Parameters", 1750, 30000, empty)


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
                    imgArray[x][y]= cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
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


def getContours(img, imgContour):
    global dir
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
            elif cx > int(frameWidth / 2) + deadZone:
                cv2.putText(imgContour, " GO RIGHT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                cv2.rectangle(imgContour, (int(frameWidth / 2 + deadZone), int(frameHeight / 2 - deadZone)), (frameWidth, int(frameHeight / 2) + deadZone), (0, 0, 255), cv2.FILLED)
                dir = 2
            elif cy < int(frameHeight / 2) - deadZone:
                cv2.putText(imgContour, " GO UP ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                cv2.rectangle(imgContour, (int(frameWidth / 2 - deadZone), 0), (int(frameWidth / 2 + deadZone), int(frameHeight / 2) - deadZone), (0, 0, 255), cv2.FILLED)
                dir = 3
            elif cy > int(frameHeight / 2) + deadZone:
                cv2.putText(imgContour, " GO DOWN ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                cv2.rectangle(imgContour, (int(frameWidth / 2 - deadZone), int(frameHeight / 2) + deadZone), (int(frameWidth / 2 + deadZone), frameHeight), (0, 0, 255), cv2.FILLED)
                dir = 4
            else:
                dir = 0

            cv2.line(imgContour, (int(frameWidth / 2), int(frameHeight / 2)), (cx, cy), (0, 0, 255), 3)
            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 5)
            cv2.putText(imgContour, "Points: " + str(len(approx)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7, (0, 255, 0), 2)
            cv2.putText(imgContour, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(imgContour, " " + str(int(x)) + " " + str(int(y)), (x - 20, y - 45), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
        else:
            dir = 0

def display(img):
    cv2.line(img, (int(frameWidth / 2) - deadZone, 0), (int(frameWidth / 2) - deadZone, frameHeight), (255, 255, 0), 3)
    cv2.line(img, (int(frameWidth / 2) + deadZone, 0), (int(frameWidth / 2) + deadZone, frameHeight), (255, 255, 0), 3)
    cv2.circle(img, (int(frameWidth / 2), int(frameHeight / 2)), 5, (0, 0, 255), 5)
    cv2.line(img, (0, int(frameHeight / 2) - deadZone), (frameWidth, int(frameHeight / 2) - deadZone), (255, 255, 0), 3)
    cv2.line(img, (0, int(frameHeight / 2) + deadZone), (frameWidth, int(frameHeight / 2) + deadZone), (255, 255, 0), 3)

# Start joystick control thread
joystick_thread = threading.Thread(target=joystick_control)
joystick_thread.daemon = True
joystick_thread.start()

while True:
    # GET THE IMAGE FROM TELLO
    ret, img = cap.read()
    imgContour = img.copy()
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("HUE Min", "HSV")
    h_max = cv2.getTrackbarPos("HUE Max", "HSV")
    s_min = cv2.getTrackbarPos("SAT Min", "HSV")
    s_max = cv2.getTrackbarPos("SAT Max", "HSV")
    v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
    v_max = cv2.getTrackbarPos("VALUE Max", "HSV")

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(imgHsv, lower, upper)
    result = cv2.bitwise_and(img, img, mask=mask)
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    imgBlur = cv2.GaussianBlur(result, (7, 7), 1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
    threshold1 = cv2.getTrackbarPos("Threshold1", "Parameters")
    threshold2 = cv2.getTrackbarPos("Threshold2", "Parameters")
    imgCanny = cv2.Canny(imgGray, threshold1, threshold2)
    kernel = np.ones((5, 5))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
    getContours(imgDil, imgContour)
    display(imgContour)

    ################# FLIGHT
    # if startCounter == 0:
    #     me.take_off()   
    #     startCounter = 1

    if dir == 1:
        me.left = -60
    elif dir == 2:
        me.right = 60
    elif dir == 3:
        me.forward = 60
    elif dir == 4:
        me.land()
    else:
        me.rcRoll = 0
        me.rcPitch = 0
        me.rcThrottle = 0
        me.rcYaw = 0

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

