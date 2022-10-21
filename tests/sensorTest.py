import gpiozero
import time

#set up the sensor pins    
sensorFront_Left = gpiozero.DistanceSensor(echo=0,trigger=10, max_distance=2,queue_len=15)
sensorRight = gpiozero.DistanceSensor(echo=1,trigger=25, max_distance=2,queue_len=15)
sensorLeft = gpiozero.DistanceSensor(echo=11,trigger=9, max_distance=2,queue_len=15)
sensorFront_Right = gpiozero.DistanceSensor(echo=7,trigger=18, max_distance=2,queue_len=15)
while 1:
    time.sleep(1)
    print("sensorFront_Left:",sensorFront_Left.distance*100)
    print("sensorLeft:",sensorLeft.distance*100)
    print("sensorFront_Right:",sensorFront_Right.distance*100)
    print("sensorRight:",sensorRight.distance*100)
	
	
