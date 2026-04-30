import struct
import math
import config as cfg

class receiver:
    def __init__(self,Nao):
        timestep = int(Nao.getBasicTimeStep())
        # Initialize communication receiver
        #Set receiver
        self.receiver = Nao.getDevice("robot_receiver")
        self.robot_name  = Nao.getName() 
        if self.receiver is None:
            print("cant find 'robot_receiver'")
        else:
            #If successfully found, set the receiver
            self.receiver.enable(timestep)
            if(self.robot_name == 'Black_Striker' or
               self.robot_name == 'Black_Defender_1' or
               self.robot_name == 'Black_Defender_2' or
               self.robot_name == 'Black_Keeper'):
                self.receiver.setChannel(1) 
            if(self.robot_name == 'Red_Striker' or
               self.robot_name == 'Red_Defender_1' or
               self.robot_name == 'Red_Defender_2' or
               self.robot_name == 'Red_Keeper'):
                self.receiver.setChannel(1) 

        #V1.4 new
        #dic for all data
        self.role_indices = {
            'Black_Striker': 2, 'Red_Striker': 5,
            'Black_Defender_1': 8, 'Red_Defender_1': 11,
            'Black_Defender_2': 14, 'Red_Defender_2': 17,
            'Black_Keeper': 20, 'Red_Keeper': 23
        }

        self.positions = {
            'ball': (0, 0),
            'flag': 0,
            'avoid_flag':0
        }

        for role in self.role_indices:
            self.positions[role] = (0, 0, 0) 
    
    def get_all_positions(self,robot_name):
        
        latest_message = None
        while self.receiver.getQueueLength() > 0:
            latest_message = self.receiver.getBytes()
            self.receiver.nextPacket()

        if latest_message is not None:
            try:
                count = len(latest_message) // 8
                coords = struct.unpack('d' * count , latest_message)
                
                #update positions of ball
                self.positions['ball'] = (coords[0], coords[1])
                
                #update positions of robots
                for role, idx in self.role_indices.items():
                    self.positions[role] = (coords[idx], coords[idx+1], coords[idx+2])

                self.positions['flag'] = coords[26]
                self.positions['avoid_flag'] = coords[27]

                position = self.positions[robot_name]
                ball_x,ball_y = self.positions['ball']
                flag = self.positions['flag']
                avoid_flag = self.positions['avoid_flag']

                return ball_x,ball_y,position[0],position[1],position[2],flag,avoid_flag

            except struct.error:
                print("Received data size mismatch.")
                return None, None, None, None, None, None, None
        print("Warning!No new message received. detail:Receiver/get_all_positon",robot_name)
        return None, None, None, None, None, None, None



    def check_obstacle(self,team,target_x,target_y,threshold):


        ball_x, ball_y = self.positions['ball']

        if team == 'Black':
            obstacle_x_1, obstacle_y_1, obstacle_z_1 = self.positions['Red_Striker']
            obstacle_x_2, obstacle_y_2, obstacle_z_2 = self.positions['Red_Defender_1']
            obstacle_x_3, obstacle_y_3, obstacle_z_3 = self.positions['Red_Defender_2']
            if self.if_obstacle(ball_x,ball_y,obstacle_x_1,obstacle_y_1,target_x,target_y,threshold) or \
                self.if_obstacle(ball_x,ball_y,obstacle_x_2,obstacle_y_2,target_x,target_y,threshold) or \
                self.if_obstacle(ball_x,ball_y,obstacle_x_3,obstacle_y_3,target_x,target_y,threshold):
                return True

        elif team == 'Red': 
            obstacle_x_1, obstacle_y_1, obstacle_z_1 = self.positions['Black_Striker']
            obstacle_x_2, obstacle_y_2, obstacle_z_2 = self.positions['Black_Defender_1']
            obstacle_x_3, obstacle_y_3, obstacle_z_3 = self.positions['Black_Defender_2']
            if self.if_obstacle(ball_x,ball_y,obstacle_x_1,obstacle_y_1,target_x,target_y,threshold) or \
                self.if_obstacle(ball_x,ball_y,obstacle_x_2,obstacle_y_2,target_x,target_y,threshold) or \
                self.if_obstacle(ball_x,ball_y,obstacle_x_3,obstacle_y_3,target_x,target_y,threshold):
                return True 
        
        return False

            

    def if_obstacle(self,ball_x,ball_y,obstacle_x,obstacle_y,target_x,target_y,threshold):
        
        #get positions
        x1, y1 = ball_x, ball_y
        x2, y2 = obstacle_x, obstacle_y
        x3, y3 = target_x, target_y

        dx = x3 - x1
        dy = y3 - y1
        line_len_sq = dx**2 + dy**2

        # Preventing division by 0
        if line_len_sq == 0:
            dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            return dist <= threshold

        t = ((x2 - x1) * dx + (y2 - y1) * dy) / line_len_sq
        t = max(0, min(1, t))

        nearest_x = x1 + t * dx
        nearest_y = y1 + t * dy

        dist = (x2 - nearest_x)**2 + (y2 - nearest_y)**2

        # Compute distance between obstacle and ball
        # If obstacle is too far from the ball (e.g., > 2* max kick distance),
        # kicking is still considered feasible
        dist_ball_to_obs = ( x1 - x2 )**2 + ( y1 - y2 )**2

        if ( dist <= threshold ** 2 ) and  (dist_ball_to_obs < ((2 * cfg.MAX_KICK_DIS)**2)):
            return True
        
        return False
    
