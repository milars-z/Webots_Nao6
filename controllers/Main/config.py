
#-----------------角度相关--------------------------------------------------------#
#V1.1新增，作为角度容忍值，偏离3.14 +-0.15都是可以接受的，也是转向8度能补偿的最小值
ANGLE_TOLERANCE = 0.20
#V1.4新增，最大角度容忍，避免机器人在斜线行走过程中频繁改变角度,大概40度
ANGLE_TOLERANCE_MAX = 0.8
#V1.1 新增，作为阈值配置指导机器人寻找正确的motion
BIG_DEGREEN   = 2.5
SIXTY_DEGREEN = 1.2
FORTY_DEGREEN = 0.75

#-----------------坐标相关--------------------------------------------------------#
DOOR_POS_X_B = -4.25
DOOR_POS_X_R = 4.25
DOOR_POS_Y_B = 0
DOOR_POS_Y_R = 0

#-----------------微调参数--------------------------------------------------------#
TOLERANCE_Y_R = 0.07
TOLERANCE_Y_L = -0.05
TOLERANCE_X_B = 0.05
TOLERANCE_X_F = -0.06


RIGHT_LEG = 0
LEFT_LEG  = 1

TRAINING_TOLERANCE = 0.1

#阻挡判断相关
OBSTACLE_THRESHOLD = 0.15

#-----------------状态1相关值-----------------------------------------------------#
#左右移动和向左转后前进的判断
#当小球在机器人的左右两侧超过1m，则不用左右移动来调整位置
#直接左右转再前进
SIDE_THRESHOLD = 1

#转向后移动和直接倒退的判断
#当小球在机器人后方超过一定距离后机器人调头在走向目标点
BACK_THRESHOLD = 1

#保持球在机器人前方至少0.15m的距离
MINDIS_BALL_ROBOT      = 0.15 
#保持机器人与球的横向水平距离大于0.2，以防倒推碰到球
MINDIS_BALL_ROBOT_SIDE = 0.2

#精细踢球半径
ACCU_KICK_THRESHOLD = 0.5

#踢球点距离球的位置
DISTANCE_TO_KICK = 0.3

#避障相关
NO_OBS    = 0
LEFT_OBS  = 1
RIGHT_OBS = 2

STATE_ITEM = 3
ROBOT_SB  = 3**1
ROBOT_SR  = 3**2
ROBOT_D1B = 3**3
ROBOT_D1R = 3**4
ROBOT_D2B = 3**5
ROBOT_D2R = 3**6

#状态名称
FIND_BACK_BALL = 0
CLOSE_TO_BALL  = 1
KICK_BALL      = 2 

BALL_IS_BACK     = 0
BALL_IS_NOT_BACK = 1

#机器人摔倒检测
ROBOT_FALL_THRESHOLD = 0.18

#球体运动检测
BALL_IS_MOVE = 0.01

#--------------------------------------------DEFENDER相关
#DEFENDER界限
DEFENDER_MAX_LINE = 0
DEFENDER_MIN_LINE_R = -3.25
DEFENDER_MIN_LINE_B = 3.25

#球门相关定量
DOOR_R_LEFT_X = -4
DOOR_R_LEFT_Y = 1
DOOR_R_RIGHT_X = -4
DOOR_R_RIGHT_Y = -1

DOOR_B_LEFT_X = 4
DOOR_B_LEFT_Y = 1
DOOR_B_RIGHT_X = 4
DOOR_B_RIGHT_Y = -1

#DEFENDER状态
DEFENDER_FALL_CHACK = 0
DEFENDER_IN_POS = 2

#
DEFENDER_POS_THRESHOLD = 0.5
DEFENDER_XY_THRESHOLD = 0.3

#最远踢球距离
MAX_KICK_DIS = 1.0

TRAINING_FILE_NAME = 'Lkick_data.csv'

PRIORITY_LIST =['goal_c', 
                'goal_l1', 'goal_r1', 
                'goal_l2', 'goal_r2',
                'goal_l3', 'goal_r3',
                'area_c',
                'area_l1', 'area_r1', 
                'area_l2', 'area_r2',
                'mid_atk_c',
                'mid_atk_l', 'mid_atk_r',
                'mid_edge_l', 'mid_edge_r',
                'center_l','center_r',
                'abandon']

#传球判断相关
PASSBALL_POS = {
    'goal_c':    (-4.5,  0.0),
    'goal_l1':   (-4.5, -0.4), 
    'goal_r1':   (-4.5,  0.4),
    'goal_l2':   (-4.5, -0.8), 
    'goal_r2':   (-4.5,  0.8),
    'goal_l3':   (-4.5, -1.2), 
    'goal_r3':   (-4.5,  1.2),

    'area_c':    (-3.8,  0.0),
    'area_l1':   (-3.8, -1.5),
    'area_r1':   (-3.8,  1.5),
    'area_l2':   (-3.8, -2.5), 
    'area_r2':   (-3.8,  2.5), 

    'mid_atk_c': (-2.5,  0.0),
    'mid_atk_l': (-2.5, -1.2),
    'mid_atk_r': (-2.5,  1.2),
    'mid_edge_l':(-2.5, -2.8), 
    'mid_edge_r':(-2.5,  2.8),

    'center_l':  (-1.0, -1.5),
    'center_r':  (-1.0,  1.5),

    'abandon':   ( 4.5, -3.0) 
}

#defender传球判断相关,设置多一点值给更多角度
PRIORITY_LIST_DEF =['fwd_deep_c', 
                    'fwd_deep_l', 'fwd_deep_r', 
                    'diag_l1', 'diag_r1',
                    'diag_l2', 'diag_r2',
                    'side_l1', 'side_r1',
                    'side_l2', 'side_r2',
                    'last_l', 'last_r',
                    'deadline_l', 'deadline_r']

PASSBALL_POS_DEF = {
    'fwd_deep_c': (0.0,  0.0),
    'fwd_deep_l': (0.0, -1.5),
    'fwd_deep_r': (0.0,  1.5),

    'diag_l1':    (1.0, -2.5),
    'diag_r1':    (1.0,  2.5),
    'diag_l2':    (2.5, -3.0),
    'diag_r2':    (2.5,  3.0),

    'side_l1':    (-2.0, -3.0),
    'side_r1':    (-2.0,  3.0),
    'side_l2':    (-3.5, -3.0),
    'side_r2':    (-3.5,  3.0),

    'last_l':     (-4.5, -2.0),
    'last_r':     (-4.5,  2.0),
    'deadline_l': (-4.5, -3.0),
    'deadline_r': (-4.5,  3.0)
}


