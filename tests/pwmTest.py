'''
Control the Brightness of LED using PWM on Raspberry Pi
http://www.electronicwings.com
'''

import gpiozero
import time
#ledpin = 12
pwmL = gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=100)
pwmR = gpiozero.PWMOutputDevice(pin=10,active_high=True,initial_value=0,frequency=100)

dirL1 = gpiozero.OutputDevice(pin=5);dirL2 = gpiozero.OutputDevice(pin=6)
dirR1 = gpiozero.OutputDevice(pin=25);dirR2 = gpiozero.OutputDevice(pin=9)

dirL1.value=0;dirL2.value=1
dirR1.value=0;dirR2.value=1
encoderL = gpiozero.RotaryEncoder(a=3, b=4,max_steps=100000)
encoderR = gpiozero.RotaryEncoder(a=17, b=27,max_steps=100000)
#encoder = gpiozero.RotaryEncoder(a=21, b=20,max_steps=100000) 
# This class has a lot more functionality,so worth reading up on it
# Step through duty cycle values, slowly increasing the speed and changing the direction of motion#
#encoder.steps = 0
# for _ in range(10):
#     for i in range(1,11):
#         pwm.value = i*0.1
#         time.sleep(1)
#         print('Duty cycle:',pwm.value)
#         #print('Counter:',encoder.steps,'Speed:',(encoder.steps)/5.0,'steps per second\n')
#         #encoder.steps = 0
    
#     print('Direction:',dir1.value)
#     dir1.value = not dir1.value
#     dir2.value = not dir1.value
#pwm.value =0 
#pwm.off()
#for i in range(10):
#        pwm.value = i*0.1
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

pwmL.value =0 
pwmL.off()
pwmR.value =0 
pwmR.off()
