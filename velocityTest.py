from utils.MarbalRunnerST import MarbalRunnerST
from utils.const import CONST
import numpy as np
import time
import datetime
if __name__=="__main__":
    init_pos=[30.0,20.0,np.pi/2]
    robot = MarbalRunnerST(init_pos,[0,0])
    t=13
    initialT = datetime.datetime.now()
    robot._prevT = datetime.datetime.now()
    while 1:
        robot._updateLoc()
        v_desired = 0
        w_desired = -0.5 #desired w to face to the target
        #print("desired val",v_desired,w_desired)
        duty_cycle_l, duty_cycle_r = robot.robot_controller.drive(v_desired,w_desired,robot.w[0],robot.w[1]) #contorl
        #print(duty_cycle_l)
        robot._pwmL.value = abs(duty_cycle_l)
        robot._pwmR.value = abs(duty_cycle_r)
        robot._setDirL(duty_cycle_l>0)
        robot._setDirR(duty_cycle_r>0)
        time.sleep(2*CONST.dt)
        if (datetime.datetime.now()-initialT).total_seconds()>t:
            break
        print(*robot.pos)

    robot._pwmL.value = 0
    robot._pwmR.value = 0

        