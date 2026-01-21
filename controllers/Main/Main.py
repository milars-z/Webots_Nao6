from controller import Robot

from GaitController import GaitController
from LogicLib import Logic
from PosrLib import PoseLib

NAO6_robot = Robot()
timestep = int(NAO6_robot.getBasicTimeStep())

class Robot():
    
    def __init__(self,NAO6_robot):

        self.robot_name  = NAO6_robot.getName() 
        self.gait_ctrl   = GaitController(NAO6_robot)
        self.Nao_logic   = Logic(NAO6_robot)
        self.Pos         = PoseLib(NAO6_robot)

        total_time       = 3000
        steps_per_second = int(1000 / timestep)
        self.total_steps = total_time * steps_per_second
        
    def run(self):

        current_step = 0
        self.Pos.stand_up()
        #V1.2根据名字调用不同代码
        #太过繁琐后期需修改
        if(self.robot_name == 'Black_Striker') or (self.robot_name == 'Red_Striker'):
            while (NAO6_robot.step(timestep) != -1)&(current_step <self.total_steps) :
                current_simulation_time = NAO6_robot.getTime()
                current_step = int(current_simulation_time*1000/timestep)
                #state = self.Nao_logic.find_state()
                state = self.Nao_logic.state_tree()
                #state = self.Nao_logic.check_andgllll()
                #state = 'kickl'
                self.gait_ctrl.manage_state_(state)

        elif(self.robot_name == 'Red_Defender_1') or (self.robot_name == 'Red_Defender_2') or (self.robot_name == 'Black_Defender_1') or (self.robot_name == 'Black_Defender_2'):
            while (NAO6_robot.step(timestep) != -1)&(current_step <self.total_steps) :
                current_simulation_time = NAO6_robot.getTime()
                current_step = int(current_simulation_time*1000/timestep)
                #state = 'Hand_Wave'
                state = self.Nao_logic.get_state_defender()
                self.gait_ctrl.manage_state_(state)

        elif(self.robot_name == 'Red_Keeper') or (self.robot_name == 'Black_Keeper'):
            while (NAO6_robot.step(timestep) != -1)&(current_step <self.total_steps) :                
                state = self.Nao_logic.get_state_keeper()
                self.gait_ctrl.manage_state_(state)




        # else:
        #     while (NAO6_robot.step(timestep) != -1)&(current_step <self.total_steps) :
        #         state = 'Hand_Wave'
        #         self.gait_ctrl.manage_state_(state)

     
Robot = Robot(NAO6_robot)
Robot.run()