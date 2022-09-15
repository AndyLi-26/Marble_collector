import gpiozero,time
import multiprocessing as mp
import numpy as np
from utils.const import CONST
from utils.robo_ctrl import RobotController
from utils.util_func import *

pwmL = gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=100)
pwmR = gpiozero.PWMOutputDevice(pin=10,active_high=True,initial_value=0,frequency=100)

dirL1 = gpiozero.OutputDevice(pin=5);self._dirL2 = gpiozero.OutputDevice(pin=6)
dirR1 = gpiozero.OutputDevice(pin=25);self._dirR2 = gpiozero.OutputDevice(pin=9)
 
sensor_back = gpiozero.DistanceSensor(echo=19,trigger=20, max_distance=1.7)
sensor_front = gpiozero.DistanceSensor(echo=11,trigger=16, max_distance=1.7)
sensor_right = gpiozero.DistanceSensor(echo=0,trigger=8, max_distance=1.7)
sensor_left = gpiozero.DistanceSensor(echo=26,trigger=1, max_distance=1.7)

pos=mp.Array("f",[30,20]) #hardcode the initial position here
w=Array("f",[0,0])

def main(pos,w):
    print(pos,w)

p_loc=Process(target=updateLoc,args=(pos,w))
p_main=Process(target=main,args=(pos,w))
p_loc.start()
p_main.start()
p_main.join()
p_loc.terminate()
