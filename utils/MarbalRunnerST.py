import gpiozero,time
from multiprocessing import Value, Process, Array
import numpy as np
from utils.const import CONST
from utils.robo_ctrl import RobotController
import datetime
from random import randint

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
        self._pwmR = gpiozero.PWMOutputDevice(pin=10,active_high=True,initial_value=0,frequency=100)

        self._dirL1 = gpiozero.OutputDevice(pin=5);self._dirL2 = gpiozero.OutputDevice(pin=6)
        self._dirR1 = gpiozero.OutputDevice(pin=25);self._dirR2 = gpiozero.OutputDevice(pin=9)
        
        self._encoderL = gpiozero.RotaryEncoder(a=3, b=4,max_steps=100000)
        self._encoderR = gpiozero.RotaryEncoder(a=17, b=27,max_steps=100000)

        #set up the sensor pins    
        # self._sensor_back = gpiozero.DistanceSensor(echo=19,trigger=20, max_distance=1.7)
        # self._sensor_front = gpiozero.DistanceSensor(echo=11,trigger=16, max_distance=1.7)
        # self._sensor_right = gpiozero.DistanceSensor(echo=0,trigger=8, max_distance=1.7)
        # self._sensor_left = gpiozero.DistanceSensor(echo=26,trigger=1, max_distance=1.7)
        self._sensorFront_Right = gpiozero.DistanceSensor(echo=0,trigger=10, max_distance=2,queue_len=15)
        self._sensorLeft = gpiozero.DistanceSensor(echo=1,trigger=25, max_distance=2,queue_len=15)
        self._sensorRight = gpiozero.DistanceSensor(echo=11,trigger=9, max_distance=2,queue_len=15)
        self._sensorFront_Left = gpiozero.DistanceSensor(echo=7,trigger=18, max_distance=2,queue_len=15)
        self.sensorFront_Right = gpiozero.DistanceSensor(echo=19,trigger=16, max_distance=2,queue_len=15)
        self.sensorFront_Left = gpiozero.DistanceSensor(echo=26,trigger=1, max_distance=2,queue_len=15)
        self.sensorLeft = gpiozero.DistanceSensor(echo=11,trigger=20, max_distance=2,queue_len=15)
        
        #set up initial vals
        self.pos=Array("f",pos[:]) #(x,y,orientation) in (mm,mm,true bearing degress)
        self.w=Array("f",w[:]) #[wl,wr]
        self.robot_controller=RobotController()
        self._prevT = datetime.datetime.now()
        self._prevT = 0
        #print("id in init",id(self._pwmL))


    def _updateLoc(self):
        #while 1:
        #print(id(self.pos))
        readL=self._encoderL.steps
        readR=self._encoderR.steps
        timeElapsed = (datetime.datetime.now() - self._prevT).total_seconds()
        #print("encoder reading", readL,readR)
        wl=((readL/CONST.shaft_pulse_per_rotation_l)/timeElapsed)*2*np.pi
        wr=((readR/CONST.shaft_pulse_per_rotation_r)/timeElapsed)*2*np.pi
        print('r: ',wl,',',wr)
        self.w[0]=wl
        self.w[1]=wr
        v = (wr*CONST.wheel_r + wl*CONST.wheel_r)/2.0
        _w = (wr*CONST.wheel_r - wl*CONST.wheel_r)/CONST.wheel_sep
        print("_w",_w)
        self.pos[0] = self.pos[0] + timeElapsed*v*np.cos(self.pos[2])
        self.pos[1] = self.pos[1] + timeElapsed*v*np.sin(self.pos[2])
        self.pos[2] = self.pos[2] + timeElapsed*_w #check the unit here, this is from michael's code, we need it to be true bearing
        
        wl=readL/CONST.shaft_pulse_per_rotation/timeElapsed*2*np.pi
        wr=readR/CONST.shaft_pulse_per_rotation/timeElapsed*2*np.pi
        self.w[0]=wl
        self.w[1]=wr
        v = (wl*CONST.wheel_r + wr*CONST.wheel_r)/2.0
        _w = (wl*CONST.wheel_r - wr*CONST.wheel_r)/CONST.wheel_sep
        self.pos[0] = self.pos[0] - timeElapsed*v*np.cos(self.pos[2])
        self.pos[1] = self.pos[1] - timeElapsed*v*np.sin(self.pos[2])
        self.pos[2] = self.pos[2] + timeElapsed*_w #check the unit here, this is from michael's code, we need it to be true bearing
        self.pos[2] = (self.pos[2] + np.pi) % (2 * np.pi) -np.pi 
        #print("thread1: ",*pos)
        self._prevT = datetime.datetime.now()
        self._encoderL.steps = 0
        self._encoderR.steps = 0

    def calibrate(self, expected_ori): 
        # diff_sensor = (self.sensorFront_Right.distance*100-self.sensorFront_Left.distance*100)
        r_distance = self.sensorFront_Right.distance*100
        l_distance = self.sensorFront_Left.distance*100
        sensor_diff = r_distance - l_distance
        #print("DisFront_Right:",r_distance)
        ##print("DisFront_Left:",l_distance)
        #print("Difference:",sensor_diff)
        self._prevT = datetime.datetime.now()

        while abs(sensor_diff) > 0.25:
            if abs(sensor_diff)<0.25:
                break
            time.sleep(CONST.dt)
            self._updateLoc()
            v_desired = 0
            w_desired = min(max(-0.5,1*(sensor_diff)),0.5) #desired w to face to the target
            duty_cycle_l, duty_cycle_r = self.robot_controller.drive(v_desired,w_desired,self.w[0],self.w[1]) #contorl
            self._pwmL.value = abs(duty_cycle_l)
            self._pwmR.value = abs(duty_cycle_r)
            self._setDirL(duty_cycle_l>0)
            self._setDirR(duty_cycle_r>0)
            #print(duty_cycle_l)
            #print(*self.pos)
            time.sleep(0.1)
            self._pwmL.value =0
            self._pwmR.value =0
            time.sleep(0.25)
            #check error (dis to target)
            r_distance = self.sensorFront_Right.distance*100
            l_distance = self.sensorFront_Left.distance*100

            sensor_diff = r_distance - l_distance
            #print("sensor_diff:",sensor_diff)
                
            CONST.time_out-=CONST.dt
            if CONST.time_out<=0:
                break
        self._pwmL.value =0
        self._pwmR.value =0
        time.sleep(CONST.dt)
        self._updateLoc()
        #print(*self.pos)

        # Set the new position
        # CONST.left_offset
        # CONST.front

        time.sleep(1)

        # self.pos[0] =  sensorFront_Left
        print("Current Posision")
        print(*self.pos)

        front_measurement =  (r_distance+l_distance)/2 + CONST.front_offset
        left_measurement = self.sensorLeft.distance*100 + CONST.left_offset
        
        if(expected_ori == 0 or expected_ori == 2*np.pi):
            self.pos[0] = 120 - front_measurement
            self.pos[1] = 120 - left_measurement
        elif(expected_ori == np.pi or expected_ori == -np.pi):
            self.pos[0] = front_measurement
            self.pos[1] = left_measurement
        elif(expected_ori == np.pi/2):
            self.pos[0] = left_measurement
            self.pos[1] = 120 - front_measurement
        elif(expected_ori == -np.pi/2):
            self.pos[0] = 120 - left_measurement
            self.pos[1] = front_measurement
            
        # self.pos[2] = expected_ori

        print("Updated position")
        print(*self.pos)

    def rotate(self,ori):
        e=100
        self._prevT = datetime.datetime.now()
        while e>0.02*2*np.pi:
            time.sleep(CONST.dt)
            threshold=10
            distance_fl = (self._sensorFront_Left.distance*100)<threshold
            distance_fr = (self._sensorFront_Right.distance*100)<threshold
            distance_r =  (self._sensorLeft.distance*100)<threshold
            distance_l =  (self._sensorRight.distance*100)<threshold
            if (distance_r  or distance_l or distance_fl or distance_fr):
                #print("STOP")
                self._pwmL.value=0.0
                self._pwmR.value=0.0
                #self._pwmL.off()
                #self._pwmR.off()
                return -1
            self._updateLoc()
            v_desired = 0
            w_desired = min(max(-0.02,CONST.Kw*((ori-self.pos[2]))),0.02) #desired w to face to the target
            print(w_desired)
            print("e:    ",e)
            print(*self.pos)
            #print("desired val",v_desired,w_desired)
            duty_cycle_l, duty_cycle_r = self.robot_controller.drive(v_desired,w_desired,self.w[0],self.w[1]) #contorl
            print("duty cycle: ",duty_cycle_l,duty_cycle_r)
        while e>0.005*2*np.pi:
            time.sleep(CONST.dt)
            l_distance = self.sensorFront_Left.distance*100
            r_distance = self.sensorFront_Left.distance*100
            if (l_distance <20  or r_distance<20 ):
                print("OBSTICAL")
            
            self._updateLoc()
            v_desired = 0
            w_desired = min(max(-0.5,CONST.Kw*((ori-self.pos[2]))),0.5) #desired w to face to the target
            #print("desired val",v_desired,w_desired)
            duty_cycle_l, duty_cycle_r = self.robot_controller.drive(v_desired,w_desired,self.w[0],self.w[1]) #contorl
            self._pwmL.value = abs(duty_cycle_l)
            self._pwmR.value = abs(duty_cycle_r)
            self._setDirL(duty_cycle_l>0)
            self._setDirR(duty_cycle_r>0)
            #print(*self.pos)
                
            #check error (dis to target)
            e=abs(ori-self.pos[2])
            # print("errs:",e)
                
            CONST.time_out-=CONST.dt
            if CONST.time_out<=0:
                break
        self._pwmL.off()
        self._pwmR.off()
        self._pwmL.value =0
        self._pwmR.value =0
        time.sleep(CONST.dt)
        self._updateLoc()
        #print(*self.pos)


    def moveTo(self,pos):
        e_dis=999
        e_ori=2*np.pi
        print("#####################################################################")
        print("target:",pos)
        self._prevT = datetime.datetime.now()
        while e_dis>CONST.arena_dim[0]*CONST.tol or e_ori>CONST.tol*2*np.pi:
            #print(id(self))
            time.sleep(CONST.dt)
            self._updateLoc()
            v_desired = min(max(-10,-1* CONST.Kw*self._computeDistance(pos)),10)  ## currently pass full power until at goal
            w_desired = min(max(-0.5,CONST.Kw*self._computeHeading(pos)),0.5) #desired w to face to the target
            #print("desired val",v_desired,w_desired)
            duty_cycle_l, duty_cycle_r = self.robot_controller.drive(v_desired,w_desired,self.w[0],self.w[1]) #contorl
            #print(duty_cycle_l,type(duty_cycle_l))
            #print(duty_cycle_r,type(duty_cycle_r))
            #print("dutyL",duty_cycle_l,"dutyR",duty_cycle_r)
            #duty_cycle_r=duty_cycle_r.item()
            #duty_cycle_l=duty_cycle_l.item()
            self._pwmL.value = abs(duty_cycle_l)
            self._pwmR.value = abs(duty_cycle_r)
            self._setDirL(duty_cycle_l>0)
            self._setDirR(duty_cycle_r>0)
            
            #print("pwmL",self._pwmL.value,"pwmR",self._pwmR.value)
            #print("dirL: (",self._dirL1.value,self._dirL2.value,") dirR: (",self._dirR1.value,self._dirR2.value,")")
            #time.sleep(CONST.dt) #ensure at least one loc update between 2 iteration
            #self._updateLoc()
            #print(*self.pos)
            
            l_distance = self.sensorFront_Left.distance*100
            r_distance = self.sensorFront_Left.distance*100
            if (l_distance <20  or r_distance<20 ):
                print("OBSTICAL")

            #check error (dis to target)
            e_dis = self._computeDistance(pos)
            if len(pos)<3:
                e_ori=0
            else:
                e_ori=abs(self.pos[2]-pos[2])
            #print("errs:",e_dis,e_ori)
                
            CONST.time_out-=CONST.dt
            #self._updateLoc()
            if CONST.time_out<=0:
                break
        self._pwmL.value =0
        self._pwmR.value =0
        self._updateLoc()
        print(*self.pos)

    def pickWP(self):
        dis=0
        while dis<10000:
            newx=randint(0,120)
            newy=randint(0,120)
            dis=(self.pos[0]-newx)**2+(self.pos[1]-newy)**2
        print(newx,newy)
        return self._computeAbsHeading([newx,newy])


    def moveUntilHit(self):
        e_dis=999
        e_ori=2*np.pi
        v_desired=5
        w_desired=0
        while 1:
            duty_cycle_l, duty_cycle_r = self.robot_controller.drive(v_desired,w_desired,self.w[0],self.w[1]) #contorl
            self._pwmL.value = abs(duty_cycle_l)
            self._pwmR.value = abs(duty_cycle_r)
            self._setDirL(duty_cycle_l>0)
            self._setDirR(duty_cycle_r>0)  
            
            time.sleep(CONST.dt)
            self._updateLoc()
            print(*self.pos)
            threshold=20
            distance_fl = (self._sensorFront_Left.distance*100)<threshold
            distance_fr = (self._sensorFront_Right.distance*100)<threshold
            distance_r =  (self._sensorLeft.distance*100)<threshold
            distance_l =  (self._sensorRight.distance*100)<threshold
            if (distance_r  or distance_l or distance_fl or distance_fr):
                print("STOP")
                if distance_fl: print("fl:", round(self._sensorFront_Left.distance*100,2))
                if distance_fr: print("fr:", round(self._sensorFront_Right.distance*100,2))
                if distance_l: print("l:", round(self._sensorRight.distance*100,2))
                if distance_r: print("r:", round(self._sensorLeft.distance*100,2))
                self._pwmL.value=0.0
                self._pwmR.value=0.0
                #self._pwmL.off()
                #self._pwmR.off()
                return


    
    def _step2(self):
        target=(90,20)
        flag=True
        while e_dis>CONST.arena_dim[0]*CONST.tol or e_ori>CONST.tol*2*np.pi:
            v_desired = 1 ## currently pass full power until at goal
            w_desired = CONST.Kw*self._computeHeading(pos) #desired w to face to the target
            duty_cycle_l, duty_cycle_r = self.robot_controller.drive(v_desired,w_desired,self.w[0],self.w[1]) #contorl
            
            #duty_cycle_r=duty_cycle_r.item()
            #duty_cycle_l=duty_cycle_l.item()
            #drive
            self._pwmL.value = abs(duty_cycle_l)
            self._pwmR.value = abs(duty_cycle_r)
            self._setDirL(duty_cycle_l>0)
            self._setDirR(duty_cycle_r>0)
            time.sleep(2*CONST.dt) #ensure at least one loc update between 2 iteration
            
            if self._sensor_right.distance>50 and flag:
                target=(90,max(self.pos[1]-20,20))
                flag=False
            
            e_dis = self._computeDistance(pos)
            if len(pos)<=3:
                e_ori=0
            else:
                e_ori=abs(self.pos[2]-pos[2])
                
            CONT.time_out-=2*CONST.dt
            if CONST.time_out<=0:
                break
        self.rotate(np.pi)
        self.moveTo((30,self.pos[1],np.pi))
        self.moveTo((30,self.pos[1],np.pi/2))
        self.moveTo((30,60))


    def _computeHeading(self,pos):
        x_diff = pos[0]-self.pos[0]
        y_diff = pos[1]-self.pos[1]
        #print("############",np.arctan2(y_diff,x_diff),x_diff,y_diff)
        theta = np.arctan2(y_diff,x_diff)-self.pos[2]
        temp=(theta + np.pi) % (2 * np.pi) -np.pi #make the angle between -pi and pi / -180 and 180
        return temp.item()

    def _computeAbsHeading(self,pos):
        x_diff = pos[0]-self.pos[0]
        y_diff = pos[1]-self.pos[1]
        #print("############",np.arctan2(y_diff,x_diff),x_diff,y_diff)
        theta = np.arctan2(y_diff,x_diff)
        temp=(theta + np.pi) % (2 * np.pi) -np.pi #make the angle between -pi and pi / -180 and 180
        return temp.item()

    def _computeDistance(self,pos):
        temp=np.sqrt((self.pos[0]-pos[0])**2+(self.pos[1]-pos[1])**2)
        return temp.item()
            
    def _setDirL(self,d:bool):
        assert type(d)== bool, type(d)
        self._dirL2.value = int(d)
        self._dirL1.value = not self._dirL2.value
    
    def _setDirR(self,d:int):
        assert type(d)== bool, type(d)
        self._dirR2.value = int(d)
        self._dirR1.value = not self._dirR2.value
        
    def run(self,f,args):
        #set up process
        #p_loc=Process(target=self._updateLoc,args=(self.w,self.pos))
        #p_main=Process(target=f,args=args)
        #p_loc.start()
        f(*args)
        #p_main.start()
        
        #p_main.join()
        #p_loc.terminate() 

        #turn off the driver
        self._pwmL.value =0 
        self._pwmL.off()
        self._pwmR.value =0 
        self._pwmR.off()
    




