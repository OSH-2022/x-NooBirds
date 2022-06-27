import numpy as np
import cv2 as cv

cap = cv.VideoCapture(0)
cap.set(3, 800)
cap.set(4, 600)

while(True):
    _, frame = cap.read()

    gs_frame = cv.GaussianBlur(frame, (5, 5), 0)
    hsv = cv.cvtColor(gs_frame, cv.COLOR_BGR2HSV)
    erode_hsv = cv.erode(hsv, None, iterations=2)

    # define range of blue color in HSV
    lower_red = np.array([-10, 40, 40])
    upper_red = np.array([10, 255, 255])

    mask = cv.inRange(erode_hsv, lower_red, upper_red)
    cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
    cnts.sort(key=cv.contourArea, reverse=True)
    cnts = cnts[:4]
    for cnt in cnts:
        rect = cv.minAreaRect(cnt)
        box = cv.boxPoints(rect)
        cv.drawContours(frame, [np.int0(box)], -1, (0, 255, 255), 1)

    cv.imshow('frame', frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
