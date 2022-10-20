from utils.MarbalRunnerST import MarbalRunnerST
from utils.const import CONST
import numpy as np
import time,sys,datetime
if __name__=="__main__":
    t0=datetime.datetime.now()
    robot = MarbalRunnerST([10,10,CONST.pi/2],[0,0])
    #robot.rotate(3*CONST.pi/4)
    #while (datetime.datetime.now()-t0).total_seconds()<=60:
    while 1:
        #time.sleep(0.2)
        #robot.testSensor()
        
        flag=-1
        while flag==-1:
            new_oir=robot.pickWP()
            flag=robot.rotate(new_oir)
            print(flag)
        robot.moveUntilHit()
        
