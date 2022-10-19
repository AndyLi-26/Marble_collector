from utils.MarbalRunnerST import MarbalRunnerST
import time
import numpy as np
if __name__=="__main__":
    init_pos=[30.0,20.0,np.pi/2]
    init_w=[0.0,0.0]
    robot=MarbalRunnerST(init_pos,init_w)
    def prelim(robot):
        # robot.calibrate(np.pi/2)
        # time.sleep(3)
        # print(*robot.pos)
        # robot.moveTo((robot.pos[0],robot.pos[1]+20)) 

        # robot.moveTo((30,40)) 
        # time.sleep(3)
        # robot.moveTo((30,80)) 
        # robot.calibrate(-np.pi/2)
        # robot.calibrate(-np.pi/2)
        # Goto waypoint one(RED)
        robot.rotate(np.pi/4)
        print("########################")
        time.sleep(2)
        robot.moveTo((80,75)) 
        time.sleep(3)

        # Goto waypoint two(Yellow)
        robot.rotate(-np.pi/2)
        time.sleep(2)
        robot.calibrate(-np.pi/2)
        time.sleep(2)
        print("########################")
        robot.moveTo((robot.pos[0],30))    
        time.sleep(3)

        robot.rotate(3.14159)
        time.sleep(2)
        robot.calibrate(np.pi)
        time.sleep(3)
        print("########################")
        robot.moveTo((30,robot.pos[1]))    
        time.sleep(3)

        robot.rotate(np.pi/2)
        time.sleep(2)
        
        robot.calibrate(np.pi/2)
        time.sleep(3)
        print("########################")
        robot.moveTo((robot.pos[0],80))    
        time.sleep(3)

    robot.run(prelim,(robot,))
