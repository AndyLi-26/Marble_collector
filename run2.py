from utils.MarbalRunnerST import MarbalRunnerST
from utils.const import CONST
import numpy as np
import time,sys,datetime
from picamera2 import Picamera2, Preview
import time
import cv2 as cv
import numpy as np
def rec(src):
    time.sleep(0.5)
    gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    rows = gray.shape[0]
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 20,param1=180,param2=30,minRadius=10, maxRadius=50)
    return len(circles) if circles else None

if __name__=="__main__":
    t0=datetime.datetime.now()
    robot = MarbalRunnerST([10,10,CONST.pi/2],[0,0])
    cap = cv.VideoCapture('/dev/video0',cv.CAP_V4L)
    while 1:
        #time.sleep(0.2)
        #robot.testSensor()
        
        new_oir=robot.pickWP()
        flag=robot.rotate(new_oir)
        for _ in range(5):
            ret,src = cap.read()
            if ret and rec(src)>3:
                break
            new_oir=robot.pickWP()
            flag=robot.rotate(new_oir)
        robot.moveUntilHit()
        
