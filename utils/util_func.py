from utils.const import CONST

def updateLoc(pos,w):
    encoderL = gpiozero.RotaryEncoder(a=3, b=4,max_steps=100000)
    encoderR = gpiozero.RotaryEncoder(a=17, b=27,max_steps=100000)
    
    encoderL.steps = 0
    encoderR.steps = 0
    sleep(CONST.dt)
    readL=encoderL.steps
    readR=encoderR.steps
    #print("encoder reading", readL,readR)
    wl=readL/CONST.shaft_pulse_per_rotation/CONST.dt*2*CONST.pi
    wr=readR/CONST.shaft_pulse_per_rotation/CONST.dt*2*CONST.pi
    w[0]=wl;w[1]=wr
    v = (wl*CONST.wheel_r + wr*CONST.wheel_r)/2.0
    _w = (wl*CONST.wheel_r - wr*CONST.wheel_r)/CONST.wheel_sep
    pos[0] = pos[0] - CONST.dt*v*np.cos(pos[2])
    pos[1] = pos[1] - CONST.dt*v*np.sin(pos[2])
    pos[2] = pos[2] + CONST.dt*_w 
    
def setDirL(d):
    _dirL2.value = int(d)
    _dirL1.value = not _dirL2.value
    dir2=d==True
    return (not dir2,dir2)