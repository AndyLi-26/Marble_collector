from utils.MarbalRunner import MarbalRunner
import time
import numpy as np
<<<<<<< HEAD
from utils.const import CONST
from utils.robo_ctrl import RobotController
from utils.util_func import *
from utils.cam_func import main

pwmL = gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=100)
pwmR = gpiozero.PWMOutputDevice(pin=10,active_high=True,initial_value=0,frequency=100)

dirL1 = gpiozero.OutputDevice(pin=5);_dirL2 = gpiozero.OutputDevice(pin=6)
dirR1 = gpiozero.OutputDevice(pin=25);_dirR2 = gpiozero.OutputDevice(pin=9)
 
sensor_back = gpiozero.DistanceSensor(echo=19,trigger=20, max_distance=1.7)
sensor_front = gpiozero.DistanceSensor(echo=11,trigger=16, max_distance=1.7)
sensor_right = gpiozero.DistanceSensor(echo=0,trigger=8, max_distance=1.7)
sensor_left = gpiozero.DistanceSensor(echo=26,trigger=1, max_distance=1.7)

pos=mp.Array("f",[30,20,np.pi]) #hardcode the initial position here
w=mp.Array("f",[0,0])

def main2(pos,w):
    while 1:
        time.sleep(1)
        print("pos: ",*pos,"w: ",*w)

p_loc=mp.Process(target=updateLoc,args=(pos,w))
p_main=mp.Process(target=main2,args=(pos,w))
p_cam=mp.Process(target=main)
p_loc.start()
p_main.start()
p_cam.start()
p_main.join()
p_loc.terminate()
=======
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
>>>>>>> b4553fc29f0297e322de36c57ea643e196428dd6
