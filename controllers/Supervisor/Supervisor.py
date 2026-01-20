from controller import Supervisor
import struct 


supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

B_score = 0
R_score = 0

INITIAL_POSITIONS = {
    "ball": ([0, 0, 0.1], [0, 0, 1, 0]),
    "striker_Red": ([-1.0, 0, 0.35], [0, 0, 1, 0]),
    "striker_Black": ([1.0, 0, 0.35], [0, 0, 1, 3.14159]),
    "defender_1_Red": ([-2.25, 1.5, 0.35], [0, 0, 1, 0]),
    "defender_1_Black": ([2.25, 1.5, 0.35], [0, 0, 1, 3.14159]),
    "defender_2_Red": ([-2.25, -1.5, 0.35], [0, 0, 1, 0]),
    "defender_2_Black": ([2.25, -1.5, 0.35], [0, 0, 1, 3.14159]),
    "keeper_Red": ([-4.5, 0, 0.35], [0, 0, 1, 0]),
    "keeper_Black": ([4.5, 0, 0.35], [0, 0, 1, 3.14159])
}



self_node = supervisor.getSelf()
if self_node is None:
    print("Error: Could not find self node")
else:
    my_def_name = self_node.getDef()

ball_name = "BALL"
ball_node = supervisor.getFromDef(ball_name)
if ball_node is None:
    print(f"cant find {ball_name}")



    #全传递
striker_Black_def_name = "Black_Striker"  
striker_Black_node = supervisor.getFromDef(striker_Black_def_name)
striker_Red_def_name = "Red_Striker"  
striker_Red_node = supervisor.getFromDef(striker_Red_def_name)

defender_1_Black_def_name = "Black_Defender_1"  
defender_1_Black_node = supervisor.getFromDef(defender_1_Black_def_name)
defender_1_Red_def_name = "Red_Defender_1"  
defender_1_Red_node = supervisor.getFromDef(defender_1_Red_def_name)

defender_2_Black_def_name = "Black_Defender_2"  
defender_2_Black_node = supervisor.getFromDef(defender_2_Black_def_name)
defender_2_Red_def_name = "Red_Defender_2"  
defender_2_Red_node = supervisor.getFromDef(defender_2_Red_def_name)

keeper_Black_def_name = "Black_Keeper"  
keeper_Black_node = supervisor.getFromDef(keeper_Black_def_name)
keeper_Red_def_name = "Red_Keeper"  
keeper_Red_node = supervisor.getFromDef(keeper_Red_def_name)

if striker_Black_node is None:
    print(f"cant find black strick ")
if striker_Red_node is None:
    print(f"cant find red strick ")

if defender_1_Black_node is None:
    print(f"cant find black defender_1 ")
if defender_1_Red_node is None:
    print(f"cant find red defender_1 ")

if defender_2_Black_node is None:
    print(f"cant find black defender_2 ")
if defender_2_Red_node is None:
    print(f"cant find red defender_2 ")

if keeper_Black_node is None:
    print(f"cant find black keeper ")
if keeper_Red_node is None:
    print(f"cant find red keeper ")

emitter = supervisor.getDevice("supervisor_emitter")
if emitter is None:
    print("cant find'Black_emitter'!")
else:
    emitter.setChannel(1) 

if( ball_node and
    striker_Black_node and striker_Red_node and 
    defender_1_Black_node and defender_1_Red_node and 
    defender_2_Black_node and defender_2_Red_node and
    keeper_Black_node and keeper_Red_node):
    pre = 1
else:
    print("wrong!")

def reset_game():
    for node, (pos, rot) in RESET_MAP:
        # 1. 设置位置
        trans_field = node.getField("translation")
        trans_field.setSFVec3f(pos)
        
        # 2. 设置旋转
        rot_field = node.getField("rotation")
        rot_field.setSFRotation(rot)
        
        # 3. 关键：清除物理动量（防止复位后还在飞）
        node.resetPhysics()
    
    print("--- 场上物体已复位 ---")


while supervisor.step(timestep) != -1:
    if pre:
 
        ball_position             = ball_node.getPosition()
        striker_Black_position    = striker_Black_node.getPosition()
        striker_Red_position      = striker_Red_node.getPosition()
        defender_1_Black_position = defender_1_Black_node.getPosition()
        defender_1_Red_position   = defender_1_Red_node.getPosition()
        defender_2_Black_position = defender_2_Black_node.getPosition()
        defender_2_Red_position   = defender_2_Red_node.getPosition()
        keeper_Black_position     = keeper_Black_node.getPosition()
        keeper_Red_position       = keeper_Red_node.getPosition()

        RESET_MAP = [
        (ball_node, INITIAL_POSITIONS["ball"]),
        (striker_Black_node, INITIAL_POSITIONS["striker_Black"]),
        (striker_Red_node, INITIAL_POSITIONS["striker_Red"]),
        (defender_1_Black_node, INITIAL_POSITIONS["defender_1_Black"]),
        (defender_1_Red_node, INITIAL_POSITIONS["defender_1_Red"]),
        (defender_2_Black_node, INITIAL_POSITIONS["defender_2_Black"]),
        (defender_2_Red_node, INITIAL_POSITIONS["defender_2_Red"]),
        (keeper_Black_node, INITIAL_POSITIONS["keeper_Black"]),
        (keeper_Red_node, INITIAL_POSITIONS["keeper_Red"]),
                ]

        ball_x, ball_y = ball_position[0], ball_position[1]
        striker_Black_x, striker_Black_y, striker_Black_z = striker_Black_position[0], striker_Black_position[1],striker_Black_position[2]
        striker_Red_x, striker_Red_y, striker_Red_z = striker_Red_position[0], striker_Red_position[1],striker_Red_position[2]
        defender_1_Black_x, defender_1_Black_y, defender_1_Black_z = defender_1_Black_position[0], defender_1_Black_position[1],defender_1_Black_position[2]
        defender_1_Red_x, defender_1_Red_y, defender_1_Red_z = defender_1_Red_position[0], defender_1_Red_position[1],defender_1_Red_position[2]
        defender_2_Black_x, defender_2_Black_y, defender_2_Black_z = defender_2_Black_position[0], defender_2_Black_position[1],defender_2_Black_position[2]
        defender_2_Red_x, defender_2_Red_y, defender_2_Red_z = defender_2_Red_position[0], defender_2_Red_position[1],defender_2_Red_position[2]
        keeper_Black_x, keeper_Black_y, keeper_Black_z = keeper_Black_position[0], keeper_Black_position[1],keeper_Black_position[2]
        keeper_Red_x, keeper_Red_y, keeper_Red_z = keeper_Red_position[0], keeper_Red_position[1],keeper_Red_position[2]

        data_to_send = [
            ball_x,ball_y,                                               #0/1
            striker_Black_x, striker_Black_y, striker_Black_z,           #2/3/4
            striker_Red_x, striker_Red_y, striker_Red_z,                 #5/6/7
            defender_1_Black_x, defender_1_Black_y, defender_1_Black_z,  #8/9/10
            defender_1_Red_x, defender_1_Red_y, defender_1_Red_z,        #11/12/13
            defender_2_Black_x, defender_2_Black_y, defender_2_Black_z,  #14/15/16
            defender_2_Red_x, defender_2_Red_y, defender_2_Red_z,        #17/18/19
            keeper_Black_x, keeper_Black_y, keeper_Black_z,              #20/21/22
            keeper_Red_x, keeper_Red_y, keeper_Red_z                     #23/24/25
        ]

        pack_format = 'd' * len(data_to_send)
        message = struct.pack(pack_format, *data_to_send)
        emitter.send(message)

        if abs( ball_x ) > 4.0:
            print("game over")
            if(ball_x <0): B_score +=1
            else: R_score +=1
            print("Black team score:",B_score," Red team score:",R_score)
            
            reset_game()







    


