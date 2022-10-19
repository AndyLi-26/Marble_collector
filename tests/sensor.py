import gpiozero
import time
sensorFront_Right = gpiozero.DistanceSensor(echo=7,trigger=18, max_distance=2,queue_len=15)
sensorLeft = gpiozero.DistanceSensor(echo=11,trigger=9, max_distance=2,queue_len=15)
sensorRight = gpiozero.DistanceSensor(echo=1,trigger=25, max_distance=2,queue_len=15)
sensorFront_Left = gpiozero.DistanceSensor(echo=0,trigger=10, max_distance=2,queue_len=15)
while 1:
    time.sleep(1.0)
    print("DisFront_Right:",sensorFront_Right.distance*100) #Right
    print("DisLeft:",sensorLeft.distance*100) #front left
    print("DisFront_Left:",sensorFront_Left.distance*100) # left
    print("DisRight:",sensorRight.distance*100) #right
	