from picamera2 import Picamera2, Preview
import time
import cv2 as cv
import numpy as np
def rec():
    cap = cv.VideoCapture('/dev/video0',cv.CAP_V4L)
    ret,src=cap.read()
    time.sleep(0.5)
    gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    rows = gray.shape[0]
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 20,
                               param1=180                                                                                                                                                                                                               , param2=30,
                               minRadius=10, maxRadius=50)
    
    return len(circles) if circles else None
    #if circles is not None:
            
            #circles = np.uint16(np.around(circles))
            #print(circles)
            #for i in circles[0, :]:
            #    center = (i[0], i[1])
                # circle center
            #    cv.circle(src, center, 1, (0, 100, 100), 3)
                # circle outline
            #    radius = i[2]
            #    cv.circle(src, center, radius, (255, 0, 255), 3)

        # cv.imshow("detected circles", src)
        #cv.imwrite("circles.jpg",src)
