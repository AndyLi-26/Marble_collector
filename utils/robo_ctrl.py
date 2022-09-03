class RobotController:
    ''' modified based on @Michael's 
    '''
    def __init__(self,Kp=0.1,Ki=0.01):
        self.Kp = Kp
        self.Ki = Ki
        self.e_sum_l = 0
        self.e_sum_r = 0
        
    def p_control(self,w_desired,w_measured,e_sum):
        duty_cycle = min(max(-1,self.Kp*(w_desired-w_measured) + self.Ki*e_sum),1)
        e_sum = e_sum + (w_desired-w_measured)
        return duty_cycle, e_sum
        
    def drive(self,v_desired,w_desired,wl,wr):
        wl_desired = (v_desired + self.wheel_sep*w_desired/2)/self.wheel_r
        wr_desired = (v_desired - self.wheel_sep*w_desired/2)/self.wheel_r
        duty_cycle_l,self.e_sum_l = self.p_control(wl_desired,wl,self.e_sum_l)
        duty_cycle_r,self.e_sum_r = self.p_control(wr_desired,wr,self.e_sum_r)
        return duty_cycle_l, duty_cycle_r