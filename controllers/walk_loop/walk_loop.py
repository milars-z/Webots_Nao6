from controller import Robot
from GaitController import GaitController


NAO6_robot = Robot()
timestep = int(NAO6_robot.getBasicTimeStep())

class Robot():
    
    def __init__(self,NAO6_robot):

        self.gait_ctrl   = GaitController(NAO6_robot)

        #设置模拟时间
        #Set simulation time
        total_time = 10  # in seconds
        steps_per_second = int(1000 / timestep)
        self.total_steps = total_time * steps_per_second
        
    def run(self):
        
        current_step = 0
        self.gait_ctrl.manage_state_('walk_start')
        while (NAO6_robot.step(timestep) != -1)&(current_step <self.total_steps) :
            current_simulation_time = NAO6_robot.getTime()
            current_step = int(current_simulation_time*1000/timestep)
            self.gait_ctrl.manage_state_('walk_loop')
        while (NAO6_robot.step(timestep) != -1):
            self.gait_ctrl.manage_state_('walk_stop')
             

Robot = Robot(NAO6_robot)
Robot.run()