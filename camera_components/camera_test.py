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
    ret,frame=cap.read()
    time.sleep(0.5)
    print(frame.shape)
    #picam2.capture_file("test-{}.jpg".format(i))
    cv2.imwrite("test.jpg",frame)
