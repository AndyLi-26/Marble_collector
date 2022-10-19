from utils.MarbalRunnerST import MarbalRunnerST
from utils.const import CONST
import numpy as np
import time,sys,datetime
if __name__=="__main__":
    robot = MarbalRunnerST([10,10,CONST.pi/2],[0,0])
    #new_oir=robot.pickWP()
    #flag=robot.rotate(new_oir)
    while flag==-1:
        new_oir=robot.pickWP()
        flag=robot.rotate(new_oir)
    
    robot.moveUntilHit()
