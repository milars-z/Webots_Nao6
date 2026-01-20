from controller import Supervisor

import struct

import math
import csv
import os

DATA_FILE = "kick_learning_data.csv"

INITIAL_POSITIONS = {
        "ball": ([0, 0, 0.1], [0, 0, 1, 0]),
        "striker_Red": ([-1.0, 0, 0.35], [0, 0, 1, 0]),
        "striker_Black": ([1.0, 0, 0.35], [0, 0, 1, 3.13]),
        "defender_1_Red": ([-2.25, 1.5, 0.35], [0, 0, 1, 0]),
        "defender_1_Black": ([2.25, 1.5, 0.35], [0, 0, 1, 3.13]),
        "defender_2_Red": ([-2.25, -1.5, 0.35], [0, 0, 1, 0]),
        "defender_2_Black": ([2.25, -1.5, 0.35], [0, 0, 1, 3.13]),
        "keeper_Red": ([-4.5, 0, 0.35], [0, 0, 1, 0]),
        "keeper_Black": ([4.5, 0, 0.35], [0, 0, 1, 3.13])
}


class SoccerSupervisor(Supervisor):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

        self.pre = 0
        self.B_score = 0
        self.R_score = 0

        self.ball_node = None
        self.striker_Black_node = None
        self.striker_Red_node = None
        self.defender_1_Black_node = None
        self.defender_1_Red_node = None
        self.defender_2_Black_node = None
        self.defender_2_Red_node = None
        self.keeper_Black_node = None
        self.keeper_Red_node = None

        self.ball_x = None
        self.ball_y = None

        self.kick_flag = 0
        self.ball_x_start = 0.0
        self.ball_y_start = 0.0

        self.striker_Black_x = None
        self.striker_Black_y = None
        self.striker_Black_z = None
        self.striker_Red_x = None
        self.striker_Red_y = None
        self.striker_Red_z = None
        self.defender_1_Black_x = None
        self.defender_1_Black_y = None
        self.defender_1_Black_z = None
        self.defender_1_Red_x = None
        self.defender_1_Red_y = None
        self.defender_1_Red_z = None
        self.defender_2_Black_x = None
        self.defender_2_Black_y = None
        self.defender_2_Black_z = None
        self.defender_2_Red_x = None
        self.defender_2_Red_y = None
        self.defender_2_Red_z = None
        self.keeper_Black_x = None
        self.keeper_Black_y = None
        self.keeper_Black_z = None
        self.keeper_Red_x = None
        self.keeper_Red_y = None
        self.keeper_Red_z = None

        self.emitter = None
        self.reset_flag = 0

        self.dx = 0
        self.dy = 0
        self.ang = 0
        self.total_dis = 0

        if not os.path.exists(DATA_FILE):
            with open(DATA_FILE, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["rel_x", "rel_y","angle","distance"]) # 表头

    def get_nodes(self):
        self_node = self.getSelf()
        if self_node is None:
            print("Error: Could not find self node")
        else:
            my_def_name = self_node.getDef()

        ball_name = "BALL"
        self.ball_node = self.getFromDef(ball_name)
        if self.ball_node is None:
            print(f"cant find {ball_name}")

        #全传递
        striker_Black_def_name = "Black_Striker"  
        self.striker_Black_node = self.getFromDef(striker_Black_def_name)
        striker_Red_def_name = "Red_Striker"  
        self.striker_Red_node = self.getFromDef(striker_Red_def_name)

        defender_1_Black_def_name = "Black_Defender_1"  
        self.defender_1_Black_node = self.getFromDef(defender_1_Black_def_name)
        defender_1_Red_def_name = "Red_Defender_1"  
        self.defender_1_Red_node = self.getFromDef(defender_1_Red_def_name)

        defender_2_Black_def_name = "Black_Defender_2"  
        self.defender_2_Black_node = self.getFromDef(defender_2_Black_def_name)
        defender_2_Red_def_name = "Red_Defender_2"  
        self.defender_2_Red_node = self.getFromDef(defender_2_Red_def_name)

        keeper_Black_def_name = "Black_Keeper"  
        self.keeper_Black_node = self.getFromDef(keeper_Black_def_name)
        keeper_Red_def_name = "Red_Keeper"  
        self.keeper_Red_node = self.getFromDef(keeper_Red_def_name)

        if self.striker_Black_node is None:
            print(f"cant find black strick ")
        if self.striker_Red_node is None:
            print(f"cant find red strick ")

        if self.defender_1_Black_node is None:
            print(f"cant find black defender_1 ")
        if self.defender_1_Red_node is None:
            print(f"cant find red defender_1 ")

        if self.defender_2_Black_node is None:
            print(f"cant find black defender_2 ")
        if self.defender_2_Red_node is None:
            print(f"cant find red defender_2 ")

        if self.keeper_Black_node is None:
            print(f"cant find black keeper ")
        if self.keeper_Red_node is None:
            print(f"cant find red keeper ")

        self.emitter = self.getDevice("supervisor_emitter")
        if self.emitter is None:
            print("cant find'Black_emitter'!")
        else:
            self.emitter.setChannel(1) 

        if( self.ball_node and
            self.striker_Black_node and self.striker_Red_node and 
            self.defender_1_Black_node and self.defender_1_Red_node and 
            self.defender_2_Black_node and self.defender_2_Red_node and
            self.keeper_Black_node and self.keeper_Red_node):
            self.pre = 1

        else:
            print("wrong!")

    def reset_game(self):
        for name, (pos, rot) in INITIAL_POSITIONS.items():
            node = getattr(self, f"{name}_node", None)

            if node:
                node.getField("translation").setSFVec3f(pos)
                node.getField("rotation").setSFRotation(rot)
                node.resetPhysics()
                #if "ball" not in name:
                #    node.restartController()

        print("--- 场上物体已复位 ---")
        self.reset_flag = 0


    def send_message(self):
            if self.pre:
   
                ball_position             = self.ball_node.getPosition()
                striker_Black_position    = self.striker_Black_node.getPosition()
                striker_Red_position      = self.striker_Red_node.getPosition()
                defender_1_Black_position = self.defender_1_Black_node.getPosition()
                defender_1_Red_position   = self.defender_1_Red_node.getPosition()
                defender_2_Black_position = self.defender_2_Black_node.getPosition()
                defender_2_Red_position   = self.defender_2_Red_node.getPosition()
                keeper_Black_position     = self.keeper_Black_node.getPosition()
                keeper_Red_position       = self.keeper_Red_node.getPosition()

                self.ball_x, self.ball_y = ball_position[0], ball_position[1]
                self.striker_Black_x, self.striker_Black_y, self.striker_Black_z = striker_Black_position[0], striker_Black_position[1],striker_Black_position[2]
                self.striker_Red_x, self.striker_Red_y, self.striker_Red_z = striker_Red_position[0], striker_Red_position[1],striker_Red_position[2]
                self.defender_1_Black_x, self.defender_1_Black_y, self.defender_1_Black_z = defender_1_Black_position[0], defender_1_Black_position[1],defender_1_Black_position[2]
                self.defender_1_Red_x, self.defender_1_Red_y, self.defender_1_Red_z = defender_1_Red_position[0], defender_1_Red_position[1],defender_1_Red_position[2]
                self.defender_2_Black_x, self.defender_2_Black_y, self.defender_2_Black_z = defender_2_Black_position[0], defender_2_Black_position[1],defender_2_Black_position[2]
                self.defender_2_Red_x, self.defender_2_Red_y, self.defender_2_Red_z = defender_2_Red_position[0], defender_2_Red_position[1],defender_2_Red_position[2]
                self.keeper_Black_x, self.keeper_Black_y, self.keeper_Black_z = keeper_Black_position[0], keeper_Black_position[1],keeper_Black_position[2]
                self.keeper_Red_x, self.keeper_Red_y, self.keeper_Red_z = keeper_Red_position[0], keeper_Red_position[1],keeper_Red_position[2]

                code = self.avoid_obs()

                data_to_send = [
                    self.ball_x,self.ball_y,                                                    #0/1
                    self.striker_Black_x, self.striker_Black_y, self.striker_Black_z,           #2/3/4
                    self.striker_Red_x, self.striker_Red_y, self.striker_Red_z,                 #5/6/7
                    self.defender_1_Black_x, self.defender_1_Black_y, self.defender_1_Black_z,  #8/9/10
                    self.defender_1_Red_x, self.defender_1_Red_y, self.defender_1_Red_z,        #11/12/13
                    self.defender_2_Black_x, self.defender_2_Black_y, self.defender_2_Black_z,  #14/15/16
                    self.defender_2_Red_x, self.defender_2_Red_y, self.defender_2_Red_z,        #17/18/19
                    self.keeper_Black_x, self.keeper_Black_y, self.keeper_Black_z,              #20/21/22
                    self.keeper_Red_x, self.keeper_Red_y, self.keeper_Red_z,                    #23/24/25
                    float(self.reset_flag),                                                     #26
                    float(code)
                ]

                pack_format = 'd' * len(data_to_send)
                message = struct.pack(pack_format, *data_to_send)
                self.emitter.send(message)

    #该函数用来避障
    #管理员遍历所有坐标，求出两者差值
    #生成特征数并返回
    #特征数为6位3进制数
    #每一位代表一个球员
    #0.1.2三种状态表明无障碍，左侧有障碍，右侧有障碍
    def avoid_obs(self):
        
        players = [
            (self.striker_Black_x    , self.striker_Black_y   ), 
            (self.striker_Red_x      , self.striker_Red_y     ),   
            (self.defender_1_Black_x , self.defender_1_Black_y), 
            (self.defender_1_Red_x   , self.defender_1_Red_y  ), 
            (self.defender_2_Black_x , self.defender_2_Black_y), 
            (self.defender_2_Red_x   , self.defender_2_Red_y  )  
        ]

        avoid_code = 0
        threshold = 0.42 # 距离阈值

        for i in range(len(players)):
            status = 0  
            me_x, me_y = players[i]

            for j in range(len(players)):
                if i == j: continue 
                other_x, other_y = players[j]
                dist = ((me_x - other_x)**2 + (me_y - other_y)**2)**0.5
                #state:左右判断
                if dist < threshold:
                    status = 1 if other_y > me_y else 2
                    break 
            avoid_code += status * (3 ** i)
        return avoid_code

    def check_goal(self):
        if abs( self.ball_x ) > 4.0:
            print("game over")
            if(self.ball_x <0): 
                self.B_score +=1
                self.reset_flag = 1
            else: 
                self.R_score +=1
                self.reset_flag = 2
            print("Black team score:",self.B_score," Red team score:",self.R_score)
            self.send_message()
            #self.step(1000)
            self.reset_game()

    def save_to_csv(self, dx, dy, ang, dist):
        with open(DATA_FILE, 'a', newline='') as f:
            writer = csv.writer(f)
            row = [f"{dx:.4f}", f"{dy:.4f}", f"{ang:.4f}", f"{dist:.4f}"]
            writer.writerow(row)

    def ball_dis_cal(self):
        vel = self.ball_node.getVelocity()
        speed = (vel[0]**2 + vel[1]**2)**0.5 

        if self.kick_flag == 0:
            if speed > 0.5: 
                self.kick_flag = 1
                self.ball_x_start = self.ball_x
                self.ball_y_start = self.ball_y
                striker_Black_position = self.striker_Black_node.getPosition()
                self.dx = self.ball_x_start - striker_Black_position[0]
                self.dy = self.ball_y_start - striker_Black_position[1]
                
                rotation_field  = self.striker_Black_node.getField("rotation")
                self.rot        = rotation_field.getSFRotation()
                self.angle = self.rot[3] 
                print(f"检测到击球！开始测距...相对位置: dx={self.dx:.3f}, dy={self.dy:.3f},angle={self.angle:.3f}")

        elif self.kick_flag == 1:
            if speed < 0.02: 
                self.kick_flag = 0
                self.total_dis = ((self.ball_x - self.ball_x_start)**2 + 
                             (self.ball_y - self.ball_y_start)**2)**0.5
                
                if self.total_dis > 0.1: 
                    print(f"球已停稳。踢球距离: {self.total_dis:.3f} 米")
                    self.save_to_csv(self.dx, self.dy, self.angle, self.total_dis)
        
        


    def run(self):
        self.get_nodes()
        while self.step(self.timestep) != -1:
            self.send_message()
            #self.check_goal()
            self.ball_dis_cal()


if __name__ == "__main__":
    sv = SoccerSupervisor()
    sv.run()