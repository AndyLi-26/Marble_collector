# Marble_collector
This repository contains all the essential code for a robot which is driven by a raspberry pi, including:
* Camera to recgnize circular object, such as marbles
* Differential drive controller (PID controller for 2 PWM controlled motor)
* Shaftencoder reading from each of the motor
* Ultrasound distance sensors
* Parallel programming
* real time camera with ssh connection (require VScode)
---
run.py will move randomly in a 120cm x 120cm area if there is no marble in the area. If there is marble, it will only move to the marbles
parallel_version.py will run some important components in parallel, such as updata location function, marble recgonition (CV) etc...
---
Note, this code is designed for a competition which was held by Monash University ECE4191. It may looks weird if you are not fimilar with the competition.
If you have any further question please Email 1953812182@qq.com
