'''
Control the Brightness of LED using PWM on Raspberry Pi
http://www.electronicwings.com
'''

import gpiozero
import time
#ledpin = 12
pwm = gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=100)
dir1 = gpiozero.OutputDevice(pin=5)
dir2 = gpiozero.OutputDevice(pin=6)
dir1.value=0
dir2.value=1
#encoder = gpiozero.RotaryEncoder(a=21, b=20,max_steps=100000) 
# This class has a lot more functionality,so worth reading up on it
# Step through duty cycle values, slowly increasing the speed and changing the direction of motion#
#encoder.steps = 0
for _ in range(10):
    for i in range(1,11):
        pwm.value = i*0.1
        time.sleep(1)
        print('Duty cycle:',pwm.value)
        #print('Counter:',encoder.steps,'Speed:',(encoder.steps)/5.0,'steps per second\n')
        #encoder.steps = 0
    
    print('Direction:',dir1.value)
    dir1.value = not dir1.value
    dir2.value = not dir1.value
    
        

pwm.value =0 
pwm.off()