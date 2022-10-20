from picamera2 import Picamera2, Preview
import time
import cv2
#picam2 = Picamera2()
#camera_config = picam2.create_preview_configuration()
#picam2.configure(camera_config)
#picam2.start_preview(Preview.QTGL)
#picam2.start()

cap = cv2.VideoCapture('/dev/video0',cv2.CAP_V4L)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH,2560)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1440)


while True:
        ret,src=cap.read()
        time.sleep(0.5)
        src = picam2.capture_array()
        
        gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
         
        #gray = cv.GaussianBlur(gray, (5,5),0)
        #cv.imshow("gray image", gray)

        rows = gray.shape[0]
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 20,
                                   param1=180                                                                                                                                                                                                               , param2=30,
                                   minRadius=10, maxRadius=30)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            print(circles)
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv.circle(src, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = i[2]
                cv.circle(src, center, radius, (255, 0, 255), 3)

        # cv.imshow("detected circles", src)
        cv2.imwrite("circles.jpg",src)