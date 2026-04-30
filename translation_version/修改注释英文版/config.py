
#-----------------Angle Parameters--------------------------------------------------------#
# V1.1 Newly added Angle tolerance:
# deviation from pi (3.14) within +-0.15 rad is acceptable.
# This is also the minimum angle that an 8-degree turn can compensate.
ANGLE_TOLERANCE = 0.20
#V1.4 Newly added Maximum angle tolerance:
# prevents frequent heading corrections during diagonal movement.
# (~40 degrees in radians)
ANGLE_TOLERANCE_MAX = 0.8
# V1.1 Newly added Thresholds for motion selection:
# used to guide the robot to choose appropriate turning behavior.
BIG_DEGREEN   = 2.5
SIXTY_DEGREEN = 1.2
FORTY_DEGREEN = 0.75

#-----------------Goal Positions--------------------------------------------------------#
DOOR_POS_X_B = -4.25
DOOR_POS_X_R = 4.25
DOOR_POS_Y_B = 0
DOOR_POS_Y_R = 0

#-----------------Fine-tuning Parameters--------------------------------------------------------#
TOLERANCE_Y_R = 0.07
TOLERANCE_Y_L = -0.05
TOLERANCE_X_B = 0.05
TOLERANCE_X_F = -0.06


RIGHT_LEG = 0
LEFT_LEG  = 1

TRAINING_TOLERANCE = 0.1

# Distance threshold for obstacle detection
OBSTACLE_THRESHOLD = 0.15

#-----------------State 1 Motion Thresholds-----------------------------------------------------#
# Side movement decision:
# If the ball is more than 1m to the left or right of the robot,
# skip lateral adjustment and turn directly toward the target.
SIDE_THRESHOLD = 1

# Backward movement decision:
# If the ball is sufficiently behind the robot,
# rotate first and then move toward the target.
BACK_THRESHOLD = 1

# Maintain at least 0.15m distance in front of the robot
MINDIS_BALL_ROBOT      = 0.15 
# Maintain at least 0.2m lateral distance from the ball
# to avoid accidental collision
MINDIS_BALL_ROBOT_SIDE = 0.2

# Fine kick radius threshold
ACCU_KICK_THRESHOLD = 0.5

# Desired distance between robot and ball before kicking
DISTANCE_TO_KICK = 0.3

# Obstacle Relevent
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

# Striker States
FIND_BACK_BALL = 0
CLOSE_TO_BALL  = 1
KICK_BALL      = 2 

BALL_IS_BACK     = 0
BALL_IS_NOT_BACK = 1

# Fall Detection
ROBOT_FALL_THRESHOLD = 0.18

# Ball movement detection
BALL_IS_MOVE = 0.01

#--------------------------------------------Defender Parameters
# Defender Boundaries
DEFENDER_MAX_LINE = 0
DEFENDER_MIN_LINE_R = -3.25
DEFENDER_MIN_LINE_B = 3.25

# Goal posts / goal line reference points
DOOR_R_LEFT_X = -4
DOOR_R_LEFT_Y = 1
DOOR_R_RIGHT_X = -4
DOOR_R_RIGHT_Y = -1

DOOR_B_LEFT_X = 4
DOOR_B_LEFT_Y = 1
DOOR_B_RIGHT_X = 4
DOOR_B_RIGHT_Y = -1

# Defender state 
DEFENDER_FALL_CHACK = 0
DEFENDER_IN_POS = 2

#
DEFENDER_POS_THRESHOLD = 0.5
DEFENDER_XY_THRESHOLD = 0.3

# Maximum kickable distance
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

# Predefined passing target positions
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

# Defender passing target priority list:
# include more candidate points to cover more angles.
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


