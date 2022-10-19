'''
Control the Brightness of LED using PWM on Raspberry Pi
http://www.electronicwings.com
'''

import gpiozero
import time
#ledpin = 12
class testPWM:
    def __init__(self):
        self.pwmL = gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=100)
        self.pwmR = gpiozero.PWMOutputDevice(pin=13,active_high=True,initial_value=0,frequency=100)

        self.dirL1 = gpiozero.OutputDevice(pin=24);self.dirL2 = gpiozero.OutputDevice(pin=23)
        self.dirR1 = gpiozero.OutputDevice(pin=5);self.dirR2 = gpiozero.OutputDevice(pin=6)

        self.dirL1.value=0;self.dirL2.value=1
        self.dirR1.value=0;self.dirR2.value=1
        self.encoderL = gpiozero.RotaryEncoder(a=20, b=16,max_steps=100000)
        self.encoderR = gpiozero.RotaryEncoder(a=19, b=26,max_steps=100000)

    def run(self):
        for _ in range(1):
            for i in range(1):
                self.pwmL.value = 0.5
                self.pwmR.value = 0.45
                self.encoderL.steps = 0
                self.encoderR.steps = 0
                time.sleep(0.5)
                print("encoderL:",self.encoderL.steps)
                print("encoderR:",self.encoderR.steps)
                print('Duty cycleL:',self.pwmL.value,'Duty cycleR:',self.pwmR.value)
            self.dirL1.value = not self.dirL1.value
            self.dirL2.value = not self.dirL2.value
            self.dirR1.value = not self.dirR1.value
            self.dirR2.value = not self.dirR2.value

        self.pwmL.value =0 
        self.pwmL.off()
        self.pwmR.value =0 
        self.pwmR.off()


if __name__=="__main__":
    n=testPWM()
    n.run()
