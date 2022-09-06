import gpiozero,time
from multiprocessing import Value, Process, Array
import numpy as np
from utils.const import CONST
from utils.robo_ctrl import RobotController

class MarbalRunner:
    '''note: distance in this class will be in mm,
             time in this class will be in s
       this module requires at least 2 cores to run efficiently
       '''
    def __init__(self,pos,w):
        #set up the motor pins
        self._pwmL = gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=100)
        self._pwmR = gpiozero.PWMOutputDevice(pin=10,active_high=True,initial_value=0,frequency=100)

        self._dirL1 = gpiozero.OutputDevice(pin=5);self._dirL2 = gpiozero.OutputDevice(pin=6)
        self._dirR1 = gpiozero.OutputDevice(pin=25);self._dirR2 = gpiozero.OutputDevice(pin=9)
        
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
            self._encoderL.steps = 0
            self._encoderR.steps = 0
            time.sleep(CONST.dt)
            readL=self._encoderL.steps
            readR=self._encoderR.steps
            wl=readL/CONST.dt
            wr=readR/CONST.dt
            w[0]=wl;w[1]=wr
            v = (wl*CONST.wheel_r + wr*CONST.wheel_r)/2.0
            _w = (wl*CONST.wheel_r - wr*CONST.wheel_r)/CONST.wheel_sep
            pos[0] = pos[0] + CONST.dt*v*np.cos(pos[2])
            pos[1] = pos[1] + CONST.dt*v*np.sin(pos[2])
            pos[2] = pos[2] + CONST.dt*_w #check the unit here, this is from michael's code, we need it to be true bearing
            
    def moveTo(self,pos):
        e_dis=999
        e_ori=2*np.pi

        print("target:",pos)

        while e_dis>CONST.arena_dim[0]*CONST.tol or e_ori>CONST.tol*2*np.pi:
            v_desired = 1 ## currently pass full power until at goal
            w_desired = CONST.Kw*self._computeHeading(pos) #desired w to face to the target
            duty_cycle_l, duty_cycle_r = self.robot_controller.drive(v_desired,w_desired,self.w[0],self.w[1]) #contorl
            print("dutyL",duty_cycle_l,"dutyR",duty_cycle_r)

            self._pwmL.value = abs(duty_cycle_l)
            self._pwmR.value = abs(duty_cycle_r)
            self._setDirL(duty_cycle_l>0)
            self._setDirR(duty_cycle_r>0)
            
            print("pwmL",self._pwmL.value,"pwmR",self._pwmR.value)
            print("dirL: (",self._dirL1.value,self._dirL2.value,") dirR: (",self._dirR1.value,self._dirR2.value,")")
            time.sleep(2*CONST.dt) #ensure at least one loc update between 2 iteration
            
            print("(",self.pos[0],self.pos[1],self.pos[2],")")
                
            #check error (dis to target)
            e_dis = self._computeDistance(pos)
            if len(pos)<=3:
                e_ori=0
            else:
                e_ori=abs(self.pos[2]-pos[2])
                
            CONST.time_out-=2*CONST.dt
            if CONST.time_out<=0:
                break
    
    def _step2(self):
        target=(90,20)
        while e_dis>CONST.arena_dim[0]*CONST.tol or e_ori>CONST.tol*2*np.pi:
            v_desired = 1 ## currently pass full power until at goal
            w_desired = CONST.Kw*self._computeHeading(pos) #desired w to face to the target
            duty_cycle_l, duty_cycle_r = self.robot_controller.drive(v_desired,w_desired,self.w[0],self.w[1]) #contorl
            
            #drive
            self._pwmL.value = abs(duty_cycle_l)
            self._pwmR.value = abs(duty_cycle_r)
            self._setDirL(duty_cycle_l>0)
            self._setDirR(duty_cycle_r>0)
            time.sleep(2*CONST.dt) #ensure at least one loc update between 2 iteration
            
            if self._sensor_right.distance>50:
                target=(90,max(self.pos[1]-20,20))
            
            e_dis = self._computeDistance(pos)
            if len(pos)<=3:
                e_ori=0
            else:
                e_ori=abs(self.pos[2]-pos[2])
                
            CONT.time_out-=2*CONST.dt
            if CONST.time_out<=0:
                break
        self.moveTo((self.pos[0],self.pos[1],np.pi))
        self.moveTo((30,self.pos[1],np.pi))
        self.moveTo((30,self.pos[1],np.pi/2))
        self.moveTo((30,60))


    def _computeHeading(self,pos):
        x_diff = pos[0]-self.pos[0]
        y_diff = pos[1]-self.pos[1]

        theta = np.arctan2(x_diff,y_diff)-self.pos[2]
        return (theta + np.pi) % (2 * np.pi) -np.pi #make the angle between -pi and pi / -180 and 180

    def _computeDistance(self,pos):
        return np.sqrt((self.pos[0]-pos[0])**2+(self.pos[1]-pos[1])**2)
            
    def _setDirL(self,d:bool):
        assert type(d)== bool
        self._dirL2.value = int(d)
        self._dirL1.value = not self._dirL2.value
    
    def _setDirR(self,d:int):
        assert type(d)== bool
        self._dirR2.value = int(d)
        self._dirR1.value = not self._dirR2.value
        
    def run(self,f,args):
        #set up process
        p_loc=Process(target=self._updateLoc,args=(self.w,self.pos))
        p_main=Process(target=f,args=args)
        p_loc.start()
        p_main.start()
        
        p_main.join()
        p_loc.terminate() 

        #turn off the driver
        self._pwmL.value =0 
        self._pwmL.off()
        self._pwmR.value =0 
        self._pwmR.off()
    




