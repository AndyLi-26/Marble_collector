import gpiozero
import time
#ledpin = 12
pwmL = gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=100)
pwmR = gpiozero.PWMOutputDevice(pin=10,active_high=True,initial_value=0,frequency=100)
pwmL.value =0 
pwmL.off()
pwmR.value =0 
pwmR.off()