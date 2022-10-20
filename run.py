from utils.MarbalRunner import MarbalRunner
import time
import numpy as np
if __name__=="__main__":
    init_pos=[30.0,20.0,np.pi/2]
    init_w=[0.0,0.0]
    robot=MarbalRunner(init_pos,init_w)
    def prelim(robot):
        robot.rotate(np.pi/4)
        robot.moveTo((90,80)) 
        robot.rotate(-1*np.pi/2)
        #robot.set_perendicular to  the wall
        #robot.moveTo((90,80,-np.pi/2))
        #robot._step2()
    robot.run(prelim,(robot,))
