import sys
import cv2 as cv
import numpy as np
from picamera2 import Picamera2, Preview
import time

def main():
    
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration()
    picam2.configure(camera_config)
    picam2.start()

    src = picam2.capture_array()
    
    gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
     
    gray = cv.medianBlur(gray, 7)
    # cv.imshow("gray image", gray)

    rows = gray.shape[0]
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 20,
                               param1=100, param2=30,
                               minRadius=1, maxRadius=200)
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv.circle(src, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv.circle(src, center, radius, (255, 0, 255), 3)

    
    cv.imshow("detected circles", src)
    cv.waitKey(0)
    
    return 0 
if __name__ == "__main__":
    main()