from picamera2 import Picamera2, Preview
import time
import cv2 as cv
import numpy as np
#picam2 = Picamera2()
#camera_config = picam2.create_preview_configuration()
#picam2.configure(camera_config)
#picam2.start_preview(Preview.QTGL)
#picam2.start()

cap = cv.VideoCapture('/dev/video0',cv.CAP_V4L)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH,2560)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1440)


while True:
        ret,src=cap.read()
        time.sleep(0.5)
        
        gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
         
        #gray = cv.GaussianBlur(gray, (5,5),0)
        #cv.imshow("gray image", gray)

        rows = gray.shape[0]
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 20,
                                   param1=180, param2=20,
                                   minRadius=5, maxRadius=50)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            print(circles.shape[1])
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv.circle(src, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = i[2]
                cv.circle(src, center, radius, (255, 0, 255), 3)

        # cv.imshow("detected circles", src)
        cv.imwrite("circles.jpg",src)