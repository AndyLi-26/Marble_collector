import gpiozero
import time
sensorFront_Right = gpiozero.DistanceSensor(echo=19,trigger=20, max_distance=2,queue_len=15)
sensorLeft = gpiozero.DistanceSensor(echo=11,trigger=16, max_distance=2,queue_len=15)
#sensorRight = gpiozero.DistanceSensor(echo=0,trigger=8, max_distance=2,queue_len=15)
sensorFront_Left = gpiozero.DistanceSensor(echo=26,trigger=1, max_distance=2,queue_len=15)
while 1:
    time.sleep(1)
    print("DisFront_Right:",sensorFront_Right.distance*100)
    # print("DisLeft:",sensorLeft.distance*100) 
    print("DisFront_Left:",sensorFront_Left.distance*100)
    #print("DisRight:",sensorRight.distance*100)
	   
	
