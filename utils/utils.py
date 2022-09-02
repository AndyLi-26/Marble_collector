import gpiozero
import time
from multiprocessing import Value, Process, Array
import numpy as np
class CONST:
    self.wheel_r=30
    self.wheel_sep=220
    self.car_dim=(260,300) #width,length
    self.arena_dim=(120,120)
    self.sensor_offset=(-20,0) #left,right sensor is 20mm inwards than the boundry of the car
    self.shaft_pulse_per_rotation=900
    self.dt=0.1
    self.tol=0.1

class RobotController:
    ''' modified based on @Michael's 
    '''
    def __init__(self,Kp=0.1,Ki=0.01):
        self.Kp = Kp
        self.Ki = Ki
        self.e_sum_l = 0
        self.e_sum_r = 0
        
    def p_control(self,w_desired,w_measured,e_sum):
        duty_cycle = min(max(-1,self.Kp*(w_desired-w_measured) + self.Ki*e_sum),1)
        e_sum = e_sum + (w_desired-w_measured)
        return duty_cycle, e_sum
        
    def drive(self,v_desired,w_desired,wl,wr):
        wl_desired = (v_desired + self.wheel_sep*w_desired/2)/self.wheel_r
        wr_desired = (v_desired - self.wheel_sep*w_desired/2)/self.wheel_r
        duty_cycle_l,self.e_sum_l = self.p_control(wl_desired,wl,self.e_sum_l)
        duty_cycle_r,self.e_sum_r = self.p_control(wr_desired,wr,self.e_sum_r)
        return duty_cycle_l, duty_cycle_r
        
class MarbalRunner:
    '''note: distance in this class will be in mm,
             time in this class will be in s
       this module requires at least 2 cores to run efficiently
       '''
    def __init__(self):
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
        self.pos=Array("f",[0.0,0.0,0.0]) #(x,y,orientation) in (mm,mm,true bearing degress)
        self.w=Array("f",[0.0,0.0]) #[wl,wr]
        self.robot_controller=RobotController()
        
    def updateLoc(self):
        while 1:
            encoderL.steps = 0
            encoderR.steps = 0
            time.sleep(CONST.dt)
            readL=encoderL.steps
            readR=encoderR.steps
            self.w[0]=readL/CONST.dt
            self.w[1]=readR/CONST.dt
            v = (self.w[0]*CONST.r + self.w[1]*CONST.r)/2.0
            w = (self.w[0]*CONST.r - self.w[0]*CONST.r)/self.l
            self.pos[0] = self.pos[0] + CONST.dt*v*np.cos(self.pos[2])
            self.pos[1] = self.pos[1] + CONST.dt*v*np.sin(self.pos[2])
            self.pos[2] = self.pos[2] + CONST.dt*w
            
    def moveTo(self,pos):
        while e>CONST.arena_dim*CONST.tol:
            self.robot_controller
            
    def rotate(self,)
            
            
    def run(self):
        pwmL.value =0 
        pwmL.off()
        pwmR.value =0 
        pwmR.off()
    
#ledpin = 12

dirL1.value=0;dirL2.value=1
dirR1.value=0;dirR2.value=1
for _ in range(1):
    for i in range(10):
        pwmL.value = 1
        pwmR.value = 1
        encoderL.steps = 0
        encoderR.steps = 0
        time.sleep(1)
        print("encoderL:",encoderL.steps)
        print("encoderR:",encoderR.steps)
        print('Duty cycleL:',pwmL.value,'Duty cycleR:',pwmR.value)
    dirL1.value = not dirL1.value
    dirL2.value = not dirL2.value
    dirR1.value = not dirR1.value
    dirR2.value = not dirR2.value



