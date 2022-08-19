import gpiozero
import time
sensor = gpiozero.DistanceSensor(echo=1,trigger=7, max_distance=2)
while 1:
	print("Dis:",sensor.distance*100)
	time.sleep(1)
