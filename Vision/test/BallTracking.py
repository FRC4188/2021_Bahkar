import cv2
import numpy as np

cap = cv2.VideoCapture(0)

def process(frame):
    blur = cv2.blur(frame,(50,50))
    
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    lower = np.array([20,100, 100])
    upper = np.array([40,255,255])

    mask = cv2.inRange(hsv, lower, upper)

    colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    contours, hierarchy = cv2.findContours(mask, 1, 2)

    areas = []
    
    for i in range(0, len(contours)):
        cnt = contours[i]
        areas.append(cv2.contourArea(cnt))

    if(len(areas) > 0):
        maxArea  = np.argmax(areas)
        M = cv2.moments(contours[maxArea])
        try:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        except:
            cx = 0
            cy = 0 
        hasTarget = True
    else:
        maxArea = -1
        cx = 0
        cy = 0
        hasTarget = False
    
    cv2.drawContours(frame, contours, maxArea, (255, 0, 0), 5)

    product = [frame, (cx, cy), hasTarget]
    
    return frame

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    product = process(frame)

    # Display the resulting frame
    cv2.imshow('frame', product)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

