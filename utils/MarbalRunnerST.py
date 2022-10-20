import gpiozero,time
import numpy as np
from utils.const import CONST
from utils.robo_ctrl import RobotController
import datetime
from random import randint
import math
class MarbalRunnerST:
    '''note: distance in this class will be in cm,
             time in this class will be in s
       this module requires at least 2 cores to run efficiently
       '''
    def __init__(self,pos,w):
        #set up the motor pins
        self._pwmL = gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=100)
        self._pwmR = gpiozero.PWMOutputDevice(pin=13,active_high=True,initial_value=0,frequency=100)

        self._dirL1 = gpiozero.OutputDevice(pin=24);self._dirL2 = gpiozero.OutputDevice(pin=23)
        self._dirR1 = gpiozero.OutputDevice(pin=5);self._dirR2 = gpiozero.OutputDevice(pin=6)

        self._encoderL = gpiozero.RotaryEncoder(a=20, b=16,max_steps=100000)
        self._encoderR = gpiozero.RotaryEncoder(a=19, b=26,max_steps=100000)
        #set up the sensor pins    
        self._sensorFront_Left = gpiozero.DistanceSensor(echo=0,trigger=10, max_distance=2,queue_len=15)
        self._sensorRight = gpiozero.DistanceSensor(echo=1,trigger=25, max_distance=2,queue_len=15)
        self._sensorLeft = gpiozero.DistanceSensor(echo=11,trigger=9, max_distance=2,queue_len=15)
        self._sensorFront_Right = gpiozero.DistanceSensor(echo=7,trigger=18, max_distance=2,queue_len=15)
        
        #set up initial vals
        self.pos=pos[:] #(x,y,orientation) in (mm,mm,true bearing degress)
        self.w=w[:] #[wl,wr]
        self.robot_controller=RobotController()
        self._prevT = datetime.datetime.now()


    def _updateLoc(self):
        readL=self._encoderL.steps
        readR=self._encoderR.steps
        timeElapsed = (datetime.datetime.now() - self._prevT).total_seconds()
        wl=((readL/CONST.shaft_pulse_per_rotation_l)/timeElapsed)*2*np.pi
        wr=((readR/CONST.shaft_pulse_per_rotation_r)/timeElapsed)*2*np.pi
        print('r: ',wl,',',wr)
        self.w[0]=wl
        self.w[1]=wr
        v = (wr*CONST.wheel_r + wl*CONST.wheel_r)/2.0
        _w = (wl*CONST.wheel_r - wr*CONST.wheel_r)/CONST.wheel_sep
        print("_w",_w)
        self.pos[0] = self.pos[0] + timeElapsed*v*np.cos(self.pos[2])
        self.pos[1] = self.pos[1] + timeElapsed*v*np.sin(self.pos[2])
        self.pos[2] = self.pos[2] + timeElapsed*_w #check the unit here, this is from michael's code, we need it to be true bearing
        
        self.pos[2] = (self.pos[2] + np.pi) % (2 * np.pi) -np.pi 
        self._prevT = datetime.datetime.now()
        self._encoderL.steps = 0
        self._encoderR.steps = 0

    def rotate(self,ori):
        e=100
        self._prevT = datetime.datetime.now()
        d=ori-self.pos[2]
        if d>CONST.pi:
            self._setDirL(bool(1))
            self._setDirR(bool(0))
        else:
            self._setDirL(bool(0))
            self._setDirR(bool(1))
        #self._pwmL.value = 0.5#abs(duty_cycle_l)
        #self._pwmR.value = 0.5#abs(duty_cycle_r)
        #print("initial ori:",self.pos[2])
        while e>0.05*2*np.pi:
            threshold=10
            #distance_fl = (self._sensorFront_Left.distance*100)<threshold
            #distance_fr = (self._sensorFront_Right.distance*100)<threshold
            #distance_l =  (self._sensorLeft.distance*100)<threshold
            #distance_r =  (self._sensorRight.distance*100)<threshold
            #if (distance_r  or distance_l or distance_fl or distance_fr):
            #    print("stoped")
            #    if distance_fl: print("fl:", round(self._sensorFront_Left.distance*100,2))
            #    if distance_fr: print("fr:", round(self._sensorFront_Right.distance*100,2))
            #    if distance_r: print("r:", round(self._sensorRight.distance*100,2))
            #    if distance_l: print("l:", round(self._sensorLeft.distance*100,2))
            #    self._pwmL.value=0.0
            #    self._pwmR.value=0.0
            #    return -1
            time.sleep(CONST.dt)
            self._updateLoc()
            diff = ori-self.pos[2]
            diff = math.fmod(diff,2*np.pi)
            if(diff<-np.pi):
                diff = diff+2*np.pi
            elif (diff>np.pi):
                diff = diff-2*np.pi
            v_desired = 0.1
            w_desired = min(max(-1,CONST.Kw*((diff))),1) #desired w to face to the target
            #print(w_desired)
            print(*self.pos)
            duty_cycle_l, duty_cycle_r = self.robot_controller.drive(v_desired,w_desired,self.w[0],self.w[1]) #contorl
            self._pwmL.value = abs(duty_cycle_l)
            self._pwmR.value = abs(duty_cycle_r)

            self._setDirL(duty_cycle_l>0)
            self._setDirR(duty_cycle_r>0)

            #print("duty cycle: ",duty_cycle_l,duty_cycle_r)
            e=math.fmod(abs(ori-self.pos[2]),np.pi) #MAY CAUSE BUG
            CONST.time_out-=CONST.dt
            #print("dutycycle:\n",self._pwmL.value,self._pwmR.value)
            print("e:    ",e)
        #self._pwmL.value=0.0
        #self._pwmR.value=0.0
        self._pwmL.off()
        self._pwmR.off()
        time.sleep(CONST.dt)
        self._updateLoc()

    def pickWP(self):
        dis=0
        if dis<3600:
            newx=randint(0,120)
            newy=randint(0,120)
            dis=(self.pos[0]-newx)**2+(self.pos[1]-newy)**2
        print(newx,newy)
        return self._computeAbsHeading([newx,newy])
    
    def testSensor(self):
        fl=self._sensorFront_Left.distance*100
        fr=self._sensorFront_Right.distance*100
        l=self._sensorLeft.distance*100
        r=self._sensorRight.distance*100
        print("fl:",fl,"fr:",fr,"l:",l,"r:",r)

    def moveUntilHit(self):
        distance_fl = (self._sensorFront_Left.distance*100)<20
        distance_fr = (self._sensorFront_Right.distance*100)<20
        distance_l =  (self._sensorLeft.distance*100)<25
        distance_r =  (self._sensorRight.distance*100)<25
        if (distance_r  or distance_l or distance_fl or distance_fr):
            print("STOP")
            if distance_fl: print("fl:", round(self._sensorFront_Left.distance*100,2))
            if distance_fr: print("fr:", round(self._sensorFront_Right.distance*100,2))
            if distance_r: print("r:", round(self._sensorRight.distance*100,2))
            if distance_l: print("l:", round(self._sensorLeft.distance*100,2))
            self._pwmL.value=0.0
            self._pwmR.value=0.0
            #self._pwmL.off()
            #self._pwmR.off()
            return
        e_dis=999
        e_ori=2*np.pi
        v_desired=5
        w_desired=0
        self._pwmL.value = 0.5#abs(duty_cycle_l)
        self._pwmR.value = 0.48#abs(duty_cycle_r)
        self._setDirL(bool(1))
        self._setDirR(bool(1))  
        while 1:
            #duty_cycle_l, duty_cycle_r = self.robot_controller.drive(v_desired,w_desired,self.w[0],self.w[1]) #contorl
            time.sleep(CONST.dt)
            self._updateLoc()
            fl=self._sensorFront_Left.distance*100
            fr=self._sensorFront_Right.distance*100
            l=self._sensorLeft.distance*100
            r=self._sensorRight.distance*100
            print("fl:",fl,"fr:",fr,"l:",l,"r:",r)
            print(*self.pos)
            distance_fl = (self._sensorFront_Left.distance*100)<20
            distance_fr = (self._sensorFront_Right.distance*100)<20
            distance_l =  (self._sensorLeft.distance*100)<10
            distance_r =  (self._sensorRight.distance*100)<10
            if (distance_r  or distance_l or distance_fl or distance_fr):
                print("STOP")
                if distance_fl: print("fl:", round(self._sensorFront_Left.distance*100,2))
                if distance_fr: print("fr:", round(self._sensorFront_Right.distance*100,2))
                if distance_r: print("r:", round(self._sensorRight.distance*100,2))
                if distance_l: print("l:", round(self._sensorLeft.distance*100,2))
                self._pwmL.value=0.0
                self._pwmR.value=0.0
                #self._pwmL.off()
                #self._pwmR.off()
                return


    def _computeHeading(self,pos):
        x_diff = pos[0]-self.pos[0]
        y_diff = pos[1]-self.pos[1]
        theta = np.arctan2(y_diff,x_diff)-self.pos[2]
        temp=(theta + np.pi) % (2 * np.pi) -np.pi #make the angle between -pi and pi / -180 and 180
        return temp.item()

    def _computeAbsHeading(self,pos):
        x_diff = pos[0]-self.pos[0]
        y_diff = pos[1]-self.pos[1]
        theta = np.arctan2(y_diff,x_diff)
        temp=(theta + np.pi) % (2 * np.pi) -np.pi #make the angle between -pi and pi / -180 and 180
        return temp.item()

    def _setDirL(self,d:bool):
        assert type(d)== bool, type(d)
        self._dirL2.value = int(d)
        self._dirL1.value = not self._dirL2.value
    
    def _setDirR(self,d:int):
        assert type(d)== bool, type(d)
        self._dirR2.value = int(d)
        self._dirR1.value = not self._dirR2.value

