import cv2
import numpy as np
from Pluto import pluto
me = pluto()
def detect_color(image, color):
    # Convert image to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Define lower and upper bounds for the color to detect
    if color == 'red':
        lower_bound = np.array([0, 100, 100])
        upper_bound = np.array([10, 255, 255])
        lower_bound2 = np.array([160, 100, 100])
        upper_bound2 = np.array([179, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_bound, upper_bound)
        mask2 = cv2.inRange(hsv_image, lower_bound2, upper_bound2)
        mask = mask1 + mask2
    elif color == 'blue':
        lower_bound = np.array([100, 100, 100])
        upper_bound = np.array([140, 255, 255])
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    elif color == 'green':
        lower_bound = np.array([40, 100, 100])
        upper_bound = np.array([80, 255, 255])
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    else:
        print("Invalid color")
        return None
    
    # Apply the mask to the original image
    result = cv2.bitwise_and(image, image, mask=mask)
    
    return result

# Open the camera
cap = cv2.VideoCapture(1)

armed = False

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    # Detect and highlight red color
    red_detection = detect_color(frame, 'red')
    cv2.imshow('Red Detection', red_detection)

    # Detect and highlight blue color
    blue_detection = detect_color(frame, 'blue')
    cv2.imshow('Blue Detection', blue_detection)

    # Detect and highlight green color
    green_detection = detect_color(frame, 'green')
    cv2.imshow('Green Detection', green_detection)
    
    # Arming, disarming, and taking off logic
    if cv2.countNonZero(cv2.cvtColor(red_detection, cv2.COLOR_BGR2GRAY)) > 1000:
        me.take_off()
        print("Takeoff")
    elif cv2.countNonZero(cv2.cvtColor(blue_detection, cv2.COLOR_BGR2GRAY)) > 1000:
        me.disarm()
        print("Disarm")
        armed = False
    elif cv2.countNonZero(cv2.cvtColor(green_detection, cv2.COLOR_BGR2GRAY)) > 1000:
        me.arm()
        print("Arm")
        armed = True
    
    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()
