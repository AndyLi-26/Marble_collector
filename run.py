from utils.MarbalRunner import MarbalRunner
if __name__=="__main__":
    init_pos=[30.0,20.0,0.0]
    init_w=[0.0,0.0]
    robot=MarbalRunner(init_pos,init_w)
    def prelim(robot):
        robot.moveTo((90,80)) 
        robot.moveTo((90,80,-np.pi/2)) 
    robot.run(prelim,(robot,))
