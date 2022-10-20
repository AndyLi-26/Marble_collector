class CONST:
    # wheel_r=2.65
    wheel_r=3.0
    wheel_sep=23.0*wheel_r/3.025
    #wheel_sep=25
    car_dim=(26,30) #width,length
    arena_dim=(120,120)
    sensor_offset=(-2,0) #left,right sensor is 20mm inwards than the boundry of the car
    shaft_pulse_per_rotation_l=900
    shaft_pulse_per_rotation_r=900
    dt=0.1
    tol=0.005
    time_out=300
    Kw=5
    left_offset = 5
    front_offset = 22.5
    pi=3.1415926