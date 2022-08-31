import gpiozero
import time
sensorBack = gpiozero.DistanceSensor(echo=26,trigger=20, max_distance=2)
sensorFront = gpiozero.DistanceSensor(echo=19,trigger=16, max_distance=2)
sensorLeft = gpiozero.DistanceSensor(echo=0,trigger=1, max_distance=2)
sensorRight = gpiozero.DistanceSensor(echo=11,trigger=8, max_distance=2)
pin5,6
pwm=12
while 1:
	print("DisBack:",sensorBack.distance*100)
	print("DisFront:",sensorFront.distance*100)
	print("DisLeft:",sensorLeft.distance*100)
	print("DisRight:",sensorRight.distance*100)
	time.sleep(1)
