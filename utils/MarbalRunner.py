import gpiozero,time
from multiprocessing import Value, Process, Array
import numpy as np
from CONST import CONST
from robo_ctrl import RobotController

class MarbalRunner:
    '''note: distance in this class will be in mm,
             time in this class will be in s
       this module requires at least 2 cores to run efficiently
       '''
    def __init__(self,pos,w):
        #set up the motor pins
        self._pwmL = gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=100)
        self._pwmR = gpiozero.PWMOutputDevice(pin=10,active_high=True,initial_value=0,frequency=100)

        self._dirL1 = gpiozero.OutputDevice(pin=5);dirL2 = gpiozero.OutputDevice(pin=6)
        self._dirR1 = gpiozero.OutputDevice(pin=25);dirR2 = gpiozero.OutputDevice(pin=9)
        
        self._encoderL = gpiozero.RotaryEncoder(a=3, b=4,max_steps=100000)
        self._encoderR = gpiozero.RotaryEncoder(a=17, b=27,max_steps=100000)

        #set up the sensor pins    
        self._sensor_back = gpiozero.DistanceSensor(echo=19,trigger=20, max_distance=1.7)
        self._sensor_front = gpiozero.DistanceSensor(echo=11,trigger=16, max_distance=1.7)
        self._sensor_right = gpiozero.DistanceSensor(echo=0,trigger=8, max_distance=1.7)
        self._sensor_left = gpiozero.DistanceSensor(echo=26,trigger=1, max_distance=1.7)
        
        #set up initial vals
        self.pos=Array("f",pos[:]) #(x,y,orientation) in (mm,mm,true bearing degress)
        self.w=Array("f",w[:]) #[wl,wr]
        self.robot_controller=RobotController()
        
    def _updateLoc(self,w,pos):
        while 1:
            encoderL.steps = 0
            encoderR.steps = 0
            time.sleep(CONST.dt)
            readL=encoderL.steps
            readR=encoderR.steps
            wl=readL/CONST.dt
            wr=readR/CONST.dt
            v = (wl*CONST.r + wr*CONST.r)/2.0
            w = (wl*CONST.r - wr*CONST.r)/CONST.wheel_sep
            pos[0] = pos[0] + CONST.dt*v*np.cos(pos[2])
            pos[1] = pos[1] + CONST.dt*v*np.sin(pos[2])
            pos[2] = pos[2] + CONST.dt*w #check the unit here, this is from michael's code, we need it to be true bearing
            
    def moveTo(self,pos):
        e_dis=999
        e_ori=360
        while e_dis>CONST.arena_dim*CONST.tol or e_ori>CONST.tol*360:
            v_desired=1 ## currently pass full power until at goal
            w_desired= #desired w to face to the target
            duty_cycle_l, duty_cycle_r = self.robot_controller.drive(v_desired,w_desired,self.w[0],self.w[1]) #contorl
            ##################
            #check obstical
            ##################
            
            #drive
            self._pwmL=abs(duty_cycle_l)
            self._pwmR = abs(duty_cycle_r)
            self._setDirL(duty_cycle_l>0)
            self._setDirR(duty_cycle_r>0)
            time.sleep(2*CONST.dt) #ensure at least one loc update between 2 iteration
            
            #check error (dis to target)
            e_dis=sqrt((self.pos[0]-pos[0])**2+(self.pos[1]-pos[1])**2)
            if len(pos)<=3:
                e_ori=0
            else:
                e_ori=abs(self.pos[2]-pos[2])
                
            CONT.time_out-=2*CONST.dt
            if CONST.time_out<=0:
                break
            
    def _setDirL(self,dir:bool):
        assert type(dir)== bool
        self.dirL2.value=dir
        self.dirL1 = not self.dirL2
    
    def _setDirR(self,dir:int):
        assert type(dir)== bool
        self.dirR2.value=dir
        self.dirR1 = not self.dirR2
        
    def run(self,f,args):
        #set up process
        p_loc=multiprocessing.Process(target=self._updateLoc,args=(self.w,self.pos))
        p_main=multiprocessing.Process(target=f,args=(*args))
        p_loc.start()
        p_main.start()
        
        p_main.join()
        p_loc.terminate() 

        #turn off the driver
        pwmL.value =0 
        pwmL.off()
        pwmR.value =0 
        pwmR.off()
    



