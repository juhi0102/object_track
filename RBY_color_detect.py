import cv2
import numpy as np

frameWidth = 640
frameHeight = 480
cap = cv2.VideoCapture(0)
cap.set(3, frameWidth)
cap.set(4, frameHeight)

deadZone = 100
global imgContour

def empty(a):
    pass

cv2.namedWindow("HSV")
cv2.resizeWindow("HSV", 640, 240)
cv2.createTrackbar("HUE Min", "HSV", 19, 179, empty)
cv2.createTrackbar("HUE Max", "HSV", 35, 179, empty)
cv2.createTrackbar("SAT Min", "HSV", 107, 255, empty)
cv2.createTrackbar("SAT Max", "HSV", 255, 255, empty)
cv2.createTrackbar("VALUE Min", "HSV", 89, 255, empty)
cv2.createTrackbar("VALUE Max", "HSV", 255, 255, empty)

# Trackbars for blue color
cv2.createTrackbar("Blue HUE Min", "HSV", 90, 179, empty)
cv2.createTrackbar("Blue HUE Max", "HSV", 120, 179, empty)
cv2.createTrackbar("Blue SAT Min", "HSV", 100, 255, empty)
cv2.createTrackbar("Blue SAT Max", "HSV", 255, 255, empty)
cv2.createTrackbar("Blue VALUE Min", "HSV", 100, 255, empty)
cv2.createTrackbar("Blue VALUE Max", "HSV", 255, 255, empty)

# Trackbars for red color
cv2.createTrackbar("Red HUE Min", "HSV", 0, 179, empty)
cv2.createTrackbar("Red HUE Max", "HSV", 10, 179, empty)
cv2.createTrackbar("Red SAT Min", "HSV", 150, 255, empty)
cv2.createTrackbar("Red SAT Max", "HSV", 255, 255, empty)
cv2.createTrackbar("Red VALUE Min", "HSV", 50, 255, empty)
cv2.createTrackbar("Red VALUE Max", "HSV", 255, 255, empty)

# Trackbars for yellow color
cv2.createTrackbar("Yellow HUE Min", "HSV", 20, 179, empty)
cv2.createTrackbar("Yellow HUE Max", "HSV", 30, 179, empty)
cv2.createTrackbar("Yellow SAT Min", "HSV", 100, 255, empty)
cv2.createTrackbar("Yellow SAT Max", "HSV", 255, 255, empty)
cv2.createTrackbar("Yellow VALUE Min", "HSV", 100, 255, empty)
cv2.createTrackbar("Yellow VALUE Max", "HSV", 255, 255, empty)

cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters", 640, 240)
cv2.createTrackbar("Threshold1", "Parameters", 166, 255, empty)
cv2.createTrackbar("Threshold2", "Parameters", 171, 255, empty)
cv2.createTrackbar("Area", "Parameters", 3750, 30000, empty)

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
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]),
                                                 None, scale, scale)
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
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale,
                                         scale)
            if len(imgArray[x].shape) == 2:
                imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver

def getContours(img, imgContour):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        areaMin = cv2.getTrackbarPos("Area", "Parameters")
        if area > areaMin:
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 7)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            print(len(approx))
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 5)
            cv2.putText(imgContour, "Points: " + str(len(approx)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7,
                        (0, 255, 0), 2)
            cv2.putText(imgContour, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                        (0, 255, 0), 2)
            cv2.putText(imgContour, " " + str(int(x)) + " " + str(int(y)), (x - 20, y - 45),
                        cv2.FONT_HERSHEY_COMPLEX, 0.7,
                        (0, 255, 0), 2)
            cx = int(x + (w / 2))
            cy = int(y + (h / 2))
            if (cx < int(frameWidth / 2) - deadZone):
                cv2.putText(imgContour, " GO LEFT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                cv2.rectangle(imgContour, (0, int(frameHeight / 2 - deadZone)),
                              (int(frameWidth / 2) - deadZone, int(frameHeight / 2) + deadZone), (0, 0, 255),
                              cv2.FILLED)
            elif (cx > int(frameWidth / 2) + deadZone):
                cv2.putText(imgContour, " GO RIGHT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                cv2.rectangle(imgContour, (int(frameWidth / 2 + deadZone), int(frameHeight / 2 - deadZone)),
                              (frameWidth, int(frameHeight / 2) + deadZone), (0, 0, 255), cv2.FILLED)
            elif (cy < int(frameHeight / 2) - deadZone):
                cv2.putText(imgContour, " GO UP ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                cv2.rectangle(imgContour, (int(frameWidth / 2 - deadZone), 0),
                              (int(frameWidth / 2 + deadZone), int(frameHeight / 2) - deadZone), (0, 0, 255),
                              cv2.FILLED)
            elif (cy > int(frameHeight / 2) + deadZone):
                cv2.putText(imgContour, " GO DOWN ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                cv2.rectangle(imgContour, (int(frameWidth / 2 - deadZone), int(frameHeight / 2) + deadZone),
                              (int(frameWidth / 2 + deadZone), frameHeight), (0, 0, 255), cv2.FILLED)
            cv2.line(imgContour, (int(frameWidth / 2), int(frameHeight / 2)), (cx, cy),
                     (0, 0, 255), 3)

def display(img):
    cv2.line(img, (int(frameWidth / 2) - deadZone, 0), (int(frameWidth / 2) - deadZone, frameHeight), (255, 255, 0),
             3)
    cv2.line(img, (int(frameWidth / 2) + deadZone, 0), (int(frameWidth / 2) + deadZone, frameHeight), (255, 255, 0),
             3)
    cv2.circle(img, (int(frameWidth / 2), int(frameHeight / 2)), 5, (0, 0, 255), 5)
    cv2.line(img, (0, int(frameHeight / 2) - deadZone), (frameWidth, int(frameHeight / 2) - deadZone),
             (255, 255, 0), 3)
    cv2.line(img, (0, int(frameHeight / 2) + deadZone), (frameWidth, int(frameHeight / 2) + deadZone),
             (255, 255, 0), 3)

while True:

    _, img = cap.read()
    imgContour = img.copy()
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Get trackbar positions for blue color
    blue_h_min = cv2.getTrackbarPos("Blue HUE Min", "HSV")
    blue_h_max = cv2.getTrackbarPos("Blue HUE Max", "HSV")
    blue_s_min = cv2.getTrackbarPos("Blue SAT Min", "HSV")
    blue_s_max = cv2.getTrackbarPos("Blue SAT Max", "HSV")
    blue_v_min = cv2.getTrackbarPos("Blue VALUE Min", "HSV")
    blue_v_max = cv2.getTrackbarPos("Blue VALUE Max", "HSV")

    # Get trackbar positions for red color
    red_h_min = cv2.getTrackbarPos("Red HUE Min", "HSV")
    red_h_max = cv2.getTrackbarPos("Red HUE Max", "HSV")
    red_s_min = cv2.getTrackbarPos("Red SAT Min", "HSV")
    red_s_max = cv2.getTrackbarPos("Red SAT Max", "HSV")
    red_v_min = cv2.getTrackbarPos("Red VALUE Min", "HSV")
    red_v_max = cv2.getTrackbarPos("Red VALUE Max", "HSV")

    # Get trackbar positions for yellow color
    yellow_h_min = cv2.getTrackbarPos("Yellow HUE Min", "HSV")
    yellow_h_max = cv2.getTrackbarPos("Yellow HUE Max", "HSV")
    yellow_s_min = cv2.getTrackbarPos("Yellow SAT Min", "HSV")
    yellow_s_max = cv2.getTrackbarPos("Yellow SAT Max", "HSV")
    yellow_v_min = cv2.getTrackbarPos("Yellow VALUE Min", "HSV")
    yellow_v_max = cv2.getTrackbarPos("Yellow VALUE Max", "HSV")

    # Define HSV ranges for each color
    # Blue color range
    lower_blue = np.array([blue_h_min, blue_s_min, blue_v_min])
    upper_blue = np.array([blue_h_max, blue_s_max, blue_v_max])

    # Red color range
    lower_red = np.array([red_h_min, red_s_min, red_v_min])
    upper_red = np.array([red_h_max, red_s_max, red_v_max])

    # Yellow color range
    lower_yellow = np.array([yellow_h_min, yellow_s_min, yellow_v_min])
    upper_yellow = np.array([yellow_h_max, yellow_s_max, yellow_v_max])

    # Create masks for each color
    mask_blue = cv2.inRange(imgHsv, lower_blue, upper_blue)
    mask_red = cv2.inRange(imgHsv, lower_red, upper_red)
    mask_yellow = cv2.inRange(imgHsv, lower_yellow, upper_yellow)

    # Apply masks to the original image
    result_blue = cv2.bitwise_and(img, img, mask=mask_blue)
    result_red = cv2.bitwise_and(img, img, mask=mask_red)
    result_yellow = cv2.bitwise_and(img, img, mask=mask_yellow)

    # Combine results into one image
    result_combined = cv2.add(result_blue, result_red)
    result_combined = cv2.add(result_combined, result_yellow)

    # Display contours
    imgBlur = cv2.GaussianBlur(result_combined, (7, 7), 1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
    threshold1 = cv2.getTrackbarPos("Threshold1", "Parameters")
    threshold2 = cv2.getTrackbarPos("Threshold2", "Parameters")
    imgCanny = cv2.Canny(imgGray, threshold1, threshold2)
    kernel = np.ones((5, 5))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
    getContours(imgDil, imgContour)
    display(imgContour)

    # Stack images for display
    stack = stackImages(0.7, ([img, result_combined], [imgDil, imgContour]))

    # Show the stacked images
    cv2.imshow('Horizontal Stacking', stack)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
