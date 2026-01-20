## For Vision part
## Now use reviever to get the position of ball and robot

from Receiver import receiver
import math
           

class VisionReviewer:
    def __init__(self, Nao):
        timestep = int(Nao.getBasicTimeStep())

        self.receiver = receiver(Nao)

        self.ball_x = None
        self.ball_y = None
        self.robot_x = None
        self.robot_y = None
        self.robot_z = None
        self.robot_x_val = 0.0
        self.robot_y_val = 0.0  
        self.robot_z_val = 0.0
        self.robot_x_arr = [0] * 3
        self.robot_y_arr = [0] * 3
        self.robot_z_arr = [0] * 3

        self.setball = 0
        self.setrobot = 0
        self.setavoid = 0

        #处理进球后重置
        self.flag = 0

        #处理避障
        self.avoid_flag = 0


        
        self.iu = Nao.getDevice("inertial_unit")
        self.iu.enable(timestep)
        
        self.old_yaw = 0
        self.standard_yaw = 0

    def update(self,robot_name):

        ball_x, ball_y, robot_x, robot_y, robot_z, flag, avoid_flag= self.receiver.get_all_positions(robot_name)

        if ball_x is not None:
            self.ball_x = ball_x
            self.ball_y = ball_y
            self.setball = 1
            self.flag = flag

        if robot_x is not None and robot_y is not None and robot_z is not None:
            self.setrobot = 1
            self.robot_x_val = robot_x
            self.robot_y_val = robot_y
            self.robot_z_val = robot_z

        if  avoid_flag is not None:
            self.avoid_flag = avoid_flag
            self.setavoid = 1

    def get_ball_position(self):
        if self.setball == 0:
            return None, None
        return self.ball_x, self.ball_y

    def get_robot_position(self):
        if self.setrobot == 0:
            return None, None, None
        return self.robot_x_val, self.robot_y_val, self.robot_z_val
    
    def get_robot_avoid_message(self):
        if self.setavoid == 0:
            return None
        return self.avoid_flag
    
    def get_robot_angle(self):
        roll_pitch_yaw = self.iu.getRollPitchYaw()
        yaw = roll_pitch_yaw[2] 
        return yaw
    
    def get_fall_orientation(self):
        roll_pitch_yaw = self.iu.getRollPitchYaw()
        pitch = roll_pitch_yaw[1]

        if pitch < -1.0: 
            return "FRONT" 
        elif pitch > 1.0: 
            return "BACK" 

    def get_flag(self):
        return self.flag
    

    def check_obstacle(self,team,target_x,target_y,threshold):
        return self.receiver.check_obstacle(team,target_x,target_y,threshold)   
        
    
