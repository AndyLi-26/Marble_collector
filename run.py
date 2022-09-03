from utils.MarbalRunner import MarbalRunner
if __name__=="__main__":
    init_pos=[30.0,20.0,0.0]
    init_w=[0.0,0.0]
    robot=MarbalRunner(init_pos,init_w)
    locs=[(90,80),(90,30)]
    
    def moveToGoals(robot,locs):
        for loc in locs:
            robot.moveTo(loc)
    
    robot.run(moveToGoals,[robot,locs])