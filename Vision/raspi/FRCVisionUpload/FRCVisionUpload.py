import cv2
import numpy as np
from networktables import NetworkTables
from cscore import CameraServer

def process(frame):
    blur = cv2.blur(frame,(20,20))
    
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
    
    return product

NetworkTables.initialize(server='10.41.88.2')

table = NetworkTables.getTable("BallTracker")

cs = CameraServer.getInstance()

camera = cs.startAutomaticCapture()
camera.setResolution(320, 240)

cvSink = cs.getVideo()

outputStream = cs.putVideo("IntakeCam", 320,240)

img = np.zeros(shape=(240, 320, 3), dtype=np.unit8)

while(True):
    time, img = cvSink.grabFrame(img)

    if time == 0:
        outputStream.notifyError(cvSink.getError())
        continue

    product = process(img)

    table.putNumber("cX", product[1][0])
    table.putNumber("cY", product[1][1])
    table.putBoolean("hasTarget", product[2])

    outputStream.putFrame(product[0])
