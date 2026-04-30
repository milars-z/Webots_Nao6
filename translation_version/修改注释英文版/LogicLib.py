from controller import Robot, Keyboard, Motion
from VisionLib import VisionReviewer
import config as cfg
import math
import os



STATE_CHECK = False

class Logic:
    
    def __init__(self,Nao):

        # V1.2: Use robot name to determine team/role
        self.name = Nao.getName()
        self.visionReviewer = VisionReviewer(Nao)
        self.timeStep = int(Nao.getBasicTimeStep())
        # V1.1: Default setup to keep the robot facing forward

        # Store previous state (used for alternating sidestep left/right)
        self.last_state = None

        # Store ball position before kick to determine whether the ball was kicked
        self.last_ball_x = None
        self.last_ball_y = None

        if self.name[:5] == "Black":
            self.team = "Black"
            self.door_x       = cfg.DOOR_POS_X_B
            self.door_y       = cfg.DOOR_POS_Y_B
            self.normal_angle = -math.pi
        elif self.name[:3] == "Red":
            self.team = "Red"
            self.door_x = cfg.DOOR_POS_X_R
            self.door_y = cfg.DOOR_POS_Y_R
            self.normal_angle = 0
        else:
            print("illegal_name!!")

        # Assign robot role based on its name
        if self.name == 'Red_Defender_1' or self.name == 'Black_Defender_1':
            self.robot_func = 'atc'
        elif self.name == 'Red_Defender_2' or self.name == 'Black_Defender_2':
            self.robot_func = 'def'
        elif self.name == 'Black_Striker' or self.name == 'Red_Striker':
            self.robot_func = 'striker'
        elif self.name == 'Black_Keeper' or self.name == 'Red_Keeper':
            self.robot_func = 'keeper'
        else: 
            print("defender wrong name")
        


        # Load supervised learning data
        self.kick_data = []
        self.get_trainning_file(cfg.TRAINING_FILE_NAME)

        # State flags
        self.back_state_flag = cfg.BALL_IS_BACK
        # High-level state machine flags
        self.striker_state = cfg.FIND_BACK_BALL

        self.defender_state = cfg.DEFENDER_FALL_CHACK

    def zx_tese(self):

        path_state = 'Hand_Wave'
        self.visionReviewer.update(self.name)

        robot_x,robot_y,robot_z = self.visionReviewer.get_robot_position()
        ball_x,ball_y = self.visionReviewer.get_ball_position()
        robot_angle = self.visionReviewer.get_robot_angle()

        if (ball_x is None) or (robot_x is None):
            path_state = 'Hand_Wave'
            return path_state

        angle = self.calculate_angle_to_ball(robot_x,robot_y,ball_x,ball_y)
        
        if self.check_angle(robot_angle,angle,cfg.ANGLE_TOLERANCE) == False:
                path_state = self.change_angle_state(robot_angle,angle)
                return path_state
        
        else:
            return path_state

   
    def state_tree(self):

        # Default values to avoid uninitialized states
        path_state = 'Hand_Wave'
        finish_state = 'OK'

        # Default goal target position
        aim_x = self.door_x
        aim_y = self.door_y

        # Update all sensor/vision data
        self.visionReviewer.update(self.name)

        # Fall detection / recovery
        fall_state = self.check_robot_fall()
        if fall_state == True:
            orientation = self.visionReviewer.get_fall_orientation()
            if orientation == 'BACK':
                return 'StandUp'
            else:
                return 'StandUpB'
        else :
            
            obstacle_state = self.check_obstacle_in_path()
            if obstacle_state == True:
                aim_x, aim_y = self.update_door_position()
            else:
                pass
            # Detect large ball displacement (e.g., interference); reset state machine if triggered
            if self.check_ball_move(cfg.BALL_IS_MOVE) == True:
                self.striker_state = cfg.FIND_BACK_BALL

            # Obstacle avoidance
            avoid_code = self.visionReviewer.get_robot_avoid_message()
            if avoid_code != None:
                check_code = self.check_rob_obs(avoid_code,self.name)
                # Avoidance has high priority, except during fine kicking alignment
                if (self.striker_state != cfg.KICK_BALL) and (check_code != cfg.NO_OBS):
                    path_state = self.avoid_robot(check_code)
                    return path_state
                # Due to code reuse, defenders always perform avoidance to reduce teammate collisions
                if (self.robot_func == 'atc') and (check_code != cfg.NO_OBS):
                    path_state = self.avoid_robot(check_code)
                    return path_state
            
            if self.striker_state == cfg.FIND_BACK_BALL:
                if STATE_CHECK: print('state 0',self.name)
                path_state = self.find_back_ball()
            if path_state == finish_state or self.striker_state == cfg.CLOSE_TO_BALL:
                if STATE_CHECK:print('state 1',self.name)
                path_state = self.close_to_ball(aim_x,aim_y)
            if path_state == finish_state or self.striker_state == cfg.KICK_BALL:
                if STATE_CHECK:print('state 2',self.name)
                path_state = self.kick_ball(aim_x,aim_y)

            if path_state == finish_state :
                print('wrong!!!')
                path_state = 'Hand_Wave'

            return path_state
        

    # Robot fall check
    def check_robot_fall(self):
        robot_x,robot_y,robot_z = self.visionReviewer.get_robot_position()
        if(robot_z != None ):
            if( robot_z < cfg.ROBOT_FALL_THRESHOLD ):
                return True 
        return False
    
    # Obstacle check
    def check_obstacle_in_path(self):
        return self.visionReviewer.check_obstacle(self.team,self.door_x,self.door_y,cfg.OBSTACLE_THRESHOLD)
    
    # Update target kick point (V1.7)
    # - Dynamically plan candidate kick points
    # - Ignore obstacles that are too far to be a threat
    # - Choose the reachable point with minimum movement cost to speed up attack
    # - Current cost favors minimizing rotation (turning is time-consuming)
    def update_door_position(self):

        best_tx = 0
        best_ty = 0
        min_angle_change = math.pi
        if_valid_pos = False
        is_red = (self.team == 'Red')

        if self.robot_func == 'atc':
            p_list = cfg.PRIORITY_LIST_DEF
            p_pos  = cfg.PASSBALL_POS_DEF
        elif self.robot_func == 'striker':
            p_list = cfg.PRIORITY_LIST
            p_pos  = cfg.PASSBALL_POS
        else:
            p_list = cfg.PRIORITY_LIST
            p_pos  = cfg.PASSBALL_POS
            print("func_update_door_position: Illegal call!")

        robot_angle = self.visionReviewer.get_robot_angle()
        ball_x, ball_y = self.visionReviewer.get_ball_position()

        if ball_x is not None :
            for target_name in p_list:
                raw_x, raw_y = p_pos[target_name]
                if is_red:
                    tx, ty = -raw_x, -raw_y
                else:
                    tx, ty = raw_x, raw_y
                if not self.visionReviewer.check_obstacle(self.team, tx, ty, cfg.OBSTACLE_THRESHOLD):
                    # If blocked, compute cost (mainly to reduce turning)
                    # Compute the turning angle required for this candidate point
                    ideal_kick_angle = self.calculate_angle_to_ball(ball_x,ball_y,tx,ty)
                    turn_diff = self.get_changed_angle(robot_angle, ideal_kick_angle)
                    if min(min_angle_change,turn_diff) == turn_diff:
                        min_angle_change = turn_diff
                        best_tx = tx
                        best_ty = ty
                        if_valid_pos = True
            if if_valid_pos:
                return best_tx, best_ty
        else:
            print("func_update_door_position: cant get ball_x")

        print('Warning: No clear path found, using default priority target!')
        default_name = p_list[0]
        dx, dy = p_pos[default_name]
        return (-dx, -dy) if is_red else (dx, dy)
    


    
    # V1.6: Obstacle avoidance
    # Parse avoid_code to detect obstacles within ~0.35 m
    # avoid_code is provided by the Supervisor and should not be manually modified here
    # Codes: SB, SR, D1B, D1R, D2B, D2R (3^0 ... 3^5)
    # Input: code, robot_name
    # Output: obstacle state
    #   0: no obstacle
    #   1: obstacle on the left
    #   2: obstacle on the right
    def check_rob_obs(self,code,robot_name):
        check_code = cfg.ROBOT_SB
        obs_state  = cfg.NO_OBS 
        if robot_name == 'Black_Striker':
            check_code = cfg.ROBOT_SB
        elif robot_name == 'Red_Striker':
            check_code = cfg.ROBOT_SR
        elif robot_name == 'Black_Defender_1':
            check_code = cfg.ROBOT_D1B
        elif robot_name == 'Red_Defender_1':
            check_code = cfg.ROBOT_D1R
        elif robot_name == 'Black_Defender_2':
            check_code = cfg.ROBOT_D2B
        elif robot_name == 'Red_Defender_2':
            check_code = cfg.ROBOT_D2R
        else: 
            print("check_rob_obs_find illgal name input!")
        if (code // check_code) % cfg.STATE_ITEM == cfg.NO_OBS:
            obs_state = cfg.NO_OBS
        if (code // check_code) % cfg.STATE_ITEM == cfg.LEFT_OBS:
            obs_state = cfg.LEFT_OBS
        if (code // check_code) % cfg.STATE_ITEM == cfg.RIGHT_OBS:
            obs_state = cfg.RIGHT_OBS
        return obs_state
    
    def avoid_robot(self,obs_state):
        state = 'Hand_Wave'
        if obs_state == cfg.RIGHT_OBS:
            state = 'sidestepleft'
        if obs_state == cfg.LEFT_OBS:
            state = 'sidestepright'
        return state
        

    # V1.5: Split plan_path_to_ball into smaller functions
    # Handle the case where the ball is behind the robot
    # See detailed notes in V1.4 (01/17)

    # Return state=1 if the ball is in front
    def find_back_ball(self):

        state = 'Hand_Wave'
        finish_state = 'OK'

        if self.team == 'Black':
            door_pos_x = cfg.DOOR_POS_X_B
            normal_angle = math.pi
            back_angle = 0
        else : 
            door_pos_x = cfg.DOOR_POS_X_R
            normal_angle = 0
            back_angle = math.pi

        robot_x,robot_y,robot_z = self.visionReviewer.get_robot_position()
        ball_x,ball_y = self.visionReviewer.get_ball_position()
        robot_angle = self.visionReviewer.get_robot_angle()

        if (ball_x is None) or (robot_x is None):
            state = 'Hand_Wave'
            return state
    
        if (abs(ball_x - door_pos_x) - abs(robot_x - door_pos_x ) > -cfg.MINDIS_BALL_ROBOT):

            if abs(ball_x - robot_x) < cfg.BACK_THRESHOLD and self.back_state_flag !=cfg.BALL_IS_NOT_BACK:
                if self.check_angle(robot_angle,normal_angle,cfg.ANGLE_TOLERANCE) == False:
                        state = self.change_angle_state(robot_angle,normal_angle)
                        return state
                if abs(ball_y - robot_y) > cfg.MINDIS_BALL_ROBOT_SIDE :
                    state = 'back'
                    return state
                else:
                    if(ball_y > robot_y):
                        state = 'sidestepleft'
                        return state
                    else:
                        state = 'sidestepright'
                        return state
            else:
                self.back_state_flag = cfg.BALL_IS_NOT_BACK
                if self.check_angle(robot_angle,back_angle,cfg.ANGLE_TOLERANCE) == False:
                    state = self.change_angle_state(robot_angle,back_angle)
                    return state
                else:
                    state = 'forward'
                    return state

        else:
            self.striker_state = cfg.CLOSE_TO_BALL
            self.back_state_flag = cfg.BALL_IS_BACK

        if self.striker_state == cfg.CLOSE_TO_BALL:
            if self.check_angle(robot_angle,normal_angle,cfg.ANGLE_TOLERANCE) == False:
                dis = math.sqrt((robot_x - ball_x)**2 + (robot_y - ball_y)**2)
                if dis >cfg.SIDE_THRESHOLD:
                    if ball_y > robot_y:
                        aim_angle = 0.5 * math.pi 
                    else:
                        aim_angle = - 0.5 * math.pi
                    state = self.change_angle_state(robot_angle,aim_angle)
                return state
            else:
                self.striker_state = cfg.CLOSE_TO_BALL
        return finish_state

    # V1.5: Split plan_path_to_ball into smaller functions
    # Approach the ball and bring it into a controllable range
    # See detailed notes in V1.4 (01/17)

    # Only enter this function when state==1
    # Return state=2 when the robot is in ball-control mode
    def close_to_ball(self,aim_x,aim_y):
        
        state = 'Hand_Wave'
        finish_state = 'OK'

        robot_x,robot_y,robot_z = self.visionReviewer.get_robot_position()
        ball_x,ball_y = self.visionReviewer.get_ball_position()
        robot_angle = self.visionReviewer.get_robot_angle()

        if (ball_x is None) or (robot_x is None):
            state = 'Hand_Wave'
            return state

        dis = math.sqrt((robot_x - ball_x)**2 + (robot_y - ball_y)**2)
        if dis > cfg.ACCU_KICK_THRESHOLD:
            # If far from the ball, first rotate to face the ball direction
            striker_x,striker_2 = self.get_striking_point(aim_x,aim_y,ball_x,ball_y,cfg.DISTANCE_TO_KICK)
            deg = self.calculate_angle_to_ball(robot_x,robot_y,striker_x,striker_2)

            if self.check_angle(robot_angle,deg,cfg.ANGLE_TOLERANCE_MAX) == False:
                state = self.change_angle_state(robot_angle,deg)
                return state
            else:
                
                state = 'forward'
                return state
        self.striker_state = cfg.KICK_BALL
        return finish_state 


    # V1.5: Split from plan_path_to_ball
    # Called when the robot is in ball-control phase
    # Handles fine positioning before kicking
    # See V1.4 (01/17) for detailed logic notes

    # Only executed when state == 2
    # Return state = 2 while processing kick behavior
    def kick_ball(self,aim_x,aim_y):

        state = 'Hand_Wave'

        robot_x,robot_y,robot_z = self.visionReviewer.get_robot_position()
        ball_x,ball_y = self.visionReviewer.get_ball_position()
        robot_angle = self.visionReviewer.get_robot_angle()

        if (ball_x is None) or (robot_x is None):
            state = 'Hand_Wave'
            return state

        deg = self.calculate_angle_to_ball(ball_x,ball_y,aim_x,aim_y)


        idx = min(range(len(self.kick_data)), key=lambda i: abs(self.kick_data[i][2] - deg))
        if abs(self.kick_data[idx][2] - deg) > cfg.TRAINING_TOLERANCE:
            idx = None 

        if idx != None:
            if STATE_CHECK: print("Kick data found, validating...")
            dx = self.kick_data[idx][0]
            dy = self.kick_data[idx][1] 
        else:
            if STATE_CHECK: print("No kick data found, generating default values...")
            dx = 0.27 * math.cos(deg + 0.3)
            dy = 0.27 * math.sin(deg + 0.3)
        dx = 0.27 * math.cos(deg + 0.3)
        dy = 0.27 * math.sin(deg + 0.3)
            
        # Prioritize angle alignment first
        if self.check_angle(robot_angle,deg,cfg.ANGLE_TOLERANCE) == False:
            state = self.change_angle_state(robot_angle,deg)
            return state
        dx_n = ball_x - robot_x
        dy_n = ball_y - robot_y
        if STATE_CHECK:print(f"dx:{dx},dy:{dy},nowdx:{dx_n},nowdy:{dy_n}")

        if dx_n - dx > cfg.TOLERANCE_X_B:
            state = 'back'
            if self.team == 'Red':
                state = 'forward'
            return state
        if dx_n - dx < cfg.TOLERANCE_X_F:
            state = 'forward'
            if self.team == 'Red':
                state = 'back'
            return state
        
        if dy_n - dy < cfg.TOLERANCE_Y_L:
            state = 'sidestepleft'
            if self.team == 'Red':
                state = 'sidestepright'
            return state
        if dy_n - dy > cfg.TOLERANCE_Y_R:
            state = 'sidestepright'
            if self.team == 'Red':
                state = 'sidestepleft'
            return state

        state = 'kickl'
        return state
                   

    # ---------------- Ball displacement check ----------------
    # Input:
    #   threshold: max allowed ball movement per frame
    # Output:
    #   True  - ball moved more than threshold in this frame
    #   False - ball moved within threshold in this frame   
    def check_ball_move(self,threshold):

        ball_x,ball_y = self.visionReviewer.get_ball_position()
        if (ball_x is None) :
            return True
        
        if self.last_ball_x == None :
            self.last_ball_x = ball_x
            self.last_ball_y = ball_y
            return True
        else:
            dis = math.sqrt( (self.last_ball_x - ball_x)**2 + (self.last_ball_y - ball_y)**2 )
            self.last_ball_x = ball_x
            self.last_ball_y = ball_y

        if dis > threshold:
            return True
        else: 
            return False
        
    # ---------------- Angle alignment check ----------------
    # Input:
    #   robot_angle : current robot heading
    #   aim_angle   : desired target heading
    #   tolerance   : allowable angular error
    # Output:
    #   True  - within tolerance
    #   False - outside tolerance
    def check_angle(self,robot_angle,aim_angle,tolerance):

        adjust_ang = abs(robot_angle - aim_angle)
        if adjust_ang > math.pi :
            adjust_ang = 2*math.pi - adjust_ang
        if( adjust_ang > tolerance ):
            return False
        else:
            return True
        
    # ---------------- Angle adjustment state ----------------
    # Input:
    #   robot_angle : current heading
    #   aim_angle   : target heading
    # Output:
    #   motion_state : rotation direction/state
    def change_angle_state(self,robot_angle,aim_angle):
        
        turn_flag = False
        adjust_ang = abs(robot_angle - aim_angle)
        
        if adjust_ang > math.pi :
            adjust_ang = 2*math.pi - adjust_ang
            turn_flag = True

        if  (robot_angle - aim_angle > 0 and turn_flag == False) or \
            (robot_angle - aim_angle < 0 and turn_flag == True) :

            if adjust_ang > cfg.BIG_DEGREEN:
                now_state = 'turn_left_180'
            elif adjust_ang > cfg.SIXTY_DEGREEN:
                now_state = 'turn_right_60'
            elif adjust_ang > cfg.FORTY_DEGREEN:
                now_state = 'turn_right_40'
            else :
                now_state = 'turn_right_08'

        if  (robot_angle - aim_angle < 0 and turn_flag == False) or \
            (robot_angle - aim_angle > 0 and turn_flag == True) :

            if adjust_ang > cfg.BIG_DEGREEN:
                now_state = 'turn_left_180'
            elif adjust_ang > cfg.SIXTY_DEGREEN:
                now_state = 'turn_left_60'
            elif adjust_ang > cfg.FORTY_DEGREEN:
                now_state = 'turn_left_40'
            else :
                now_state = 'turn_left_08'

        return now_state
        
    # ---------------- Compute heading angle ----------------
    # Input:
    #   robot_pos_x, robot_pos_y : robot position
    #   ball_pos_x, ball_pos_y   : target position (ball or goal)
    # Output:
    #   angle (radians)
    def calculate_angle_to_ball(self, robot_pos_x, robot_pos_y, ball_pos_x, ball_pos_y):
        rx, ry = robot_pos_x, robot_pos_y
        bx, by = ball_pos_x, ball_pos_y
        dx = bx - rx
        dy = by - ry
        angle = math.atan2(dy, dx)
        return angle

    # ---------------- Compute optimal striking point ----------------
    # Ensures robot, ball, and target are approximately collinear
    # Input:
    #   target_x, target_y : goal/target position
    #   ball_x, ball_y     : ball position
    #   distance           : offset distance from ball toward target
    # Output:
    #   striking point coordinates (x, y)
    def get_striking_point(self,target_x, target_y, ball_x, ball_y, distance):
        dx = target_x - ball_x
        dy = target_y - ball_y
        line_len = math.sqrt(dx**2 + dy**2)
        if line_len == 0:
            return ball_x, ball_y
        
        strike_x = ball_x - (dx / line_len) * distance
        strike_y = ball_y - (dy / line_len) * distance
        
        return strike_x, strike_y
    
    # ---------------- Load training data ----------------
    # Read supervised learning data from file
    # Input:
    #   filename : training file name
    # Output:
    #   None (data stored in self.kick_data)
    def get_trainning_file(self,filename):
    
        current_dir = os.path.dirname(__file__)
        file_path = os.path.join(current_dir, filename)

        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        row = [float(parts[0]), float(parts[1]), float(parts[2])]
                        self.kick_data.append(row)
            print(f"read {len(self.kick_data)} data")
        else:
            print("wrong!cant find file")

    # ---------------- Compute angular difference ----------------
    # Input:
    #   robot_angle : current heading
    #   aim_angle   : desired heading
    # Output:
    #   minimal rotation angle required 
    def get_changed_angle(self,robot_angle,aim_angle):

        adjust_ang = abs(robot_angle - aim_angle)
        if adjust_ang > math.pi :
            adjust_ang = 2*math.pi - adjust_ang
        return adjust_ang


##################################################################################################
###############-----------------------------Goalkeeper Logic-----------------############################
##################################################################################################
    def get_state_keeper(self):

        self.visionReviewer.update(self.name)
        robot_x,robot_y,robot_z = self.visionReviewer.get_robot_position()
        ball_x,ball_y = self.visionReviewer.get_ball_position()
        robot_angle = self.visionReviewer.get_robot_angle()

        now_state = 'Hand_Wave'

        if(robot_y != None ):
            if((robot_y - ball_y) < 0.0):
                now_state = 'turn_left'
            elif((robot_y - ball_y) > 0.12):
                now_state = 'turn_right'

        return now_state
    
##################################################################################################
###############-----------------------------Defender Logic-----------------##############################
##################################################################################################
    def get_state_defender(self):

        self.defender_state = cfg.DEFENDER_FALL_CHACK
        now_state = 'Hand_Wave'

        # Initialize movement boundaries based on team
        # Attacking-type robots may enter opponent half
        # Y-axis is not restricted (field boundary assumed safe)
        # Note: Black team field direction is mirrored
        if self.team == 'Red':
            x_min = cfg.DEFENDER_MIN_LINE_R
            if self.robot_func == 'atc' :
                x_max = -cfg.DEFENDER_MIN_LINE_R
            else:
                x_max = cfg.DEFENDER_MAX_LINE
        else:
            x_min = cfg.DEFENDER_MIN_LINE_B
            if self.robot_func == 'atc' :
                x_max = -cfg.DEFENDER_MIN_LINE_B
            else:
                x_max = cfg.DEFENDER_MAX_LINE
            
        # As a defensive-type defender:
        # - Stay within own half
        # - Position between the ball and the goal
        # Compute an optimal guarding point based on ball and goal geometry
        # (Approximate circumcenter / strategic interception point)

        # Logic overview:
        # 1. Check fall state
        # 2. Perform obstacle avoidance if needed
        # 3. Move toward guarding/interception point
        # 
        # If movement between frames is below threshold, remain still
        # Once positioned at guarding point, skip avoidance
        # Large tolerance used to reduce unnecessary oscillation
        if self.robot_func == 'def' :

            self.visionReviewer.update(self.name)
            robot_x,robot_y,robot_z = self.visionReviewer.get_robot_position()
            ball_x,ball_y = self.visionReviewer.get_ball_position()
            robot_angle = self.visionReviewer.get_robot_angle()

            if (ball_x is None) or (robot_x is None):
                state = 'Hand_Wave'
                return state
            # Fall recovery
            fall_state = self.check_robot_fall()
            if fall_state == True:
                orientation = self.visionReviewer.get_fall_orientation()
                if orientation == 'BACK':
                    return 'StandUp'
                else:
                    return 'StandUpB'
            else:
                # If not yet positioned, perform avoidance
                # Detect large ball displacement (possible interference) and reset state if triggered
                if self.check_ball_move(cfg.BALL_IS_MOVE) == True:
                    self.defender_state = cfg.DEFENDER_FALL_CHACK
                    aim_x,aim_y = self.calculate_circumcenter(robot_x,robot_y)

                if self.defender_state != cfg.DEFENDER_IN_POS:
                    avoid_code = self.visionReviewer.get_robot_avoid_message()
                    if avoid_code != None:
                        check_code = self.check_rob_obs(avoid_code,self.name)
                        if (check_code != cfg.NO_OBS):
                            path_state = self.avoid_robot(check_code)
                            return path_state
                    else:
                        print("warning! no avoid_code")
                        # Not positioned and no valid avoidance code received
                        # Do nothing
                # Reached guarding position, update state and hold position (idle gesture)     
                else:
                    now_state = 'Hand_Wave'
            # Prioritize angle alignment
            # Defender maintains a fixed forward-facing orientation
            # Movement restricted to forward/backward/side steps
            # self.normal_angle is initialized based on team orientation
            if self.check_angle(robot_angle,self.normal_angle,cfg.ANGLE_TOLERANCE) == False:
                state = self.change_angle_state(robot_angle,self.normal_angle)
                return state
            # If orientation is correct:
            # Compute distance to target guarding point
            # Move if distance exceeds threshold
            # (Distance calculation should be standardized/refactored later)
            else:
                aim_x,aim_y = self.calculate_circumcenter(ball_x,ball_y)
                # Within threshold, update state and wave the hand
                if math.sqrt( (aim_x-robot_x)**2 + (aim_y-robot_y)**2 ) < cfg.DEFENDER_POS_THRESHOLD:
                    self.defender_state = cfg.DEFENDER_IN_POS
                    now_state = 'Hand_Wave'
                    return now_state
                # Beyond threshold, update state and move toward target
                else:
                    self.defender_state = cfg.DEFENDER_FALL_CHACK
                    if abs( aim_y - robot_y ) >cfg.DEFENDER_XY_THRESHOLD:
                        
                        if self.team == 'Red':
                            if aim_y > robot_y: 
                                now_state = 'sidestepleft'
                            else: 
                                now_state = 'sidestepright'
                            
                            return now_state
                        else:
                            if aim_y > robot_y: 
                                now_state = 'sidestepright'
                            else: 
                                now_state = 'sidestepleft'  
                            return now_state
                    # Enforce field boundary constraints before movement
                    if self.team == 'Red':
                        if robot_x < x_min :
                            now_state = 'Hand_Wave'
                            return now_state
                    else:
                        if robot_x > x_min :
                            now_state = 'Hand_Wave'
                            return now_state
                        
                    if abs( aim_x - robot_x ) >cfg.DEFENDER_XY_THRESHOLD:
                        if self.team == 'Red':
                            if aim_x > robot_x:
                                now_state = 'forward'
                            else :
                                now_state = 'back'
                        else:
                            if aim_x > robot_x:
                                now_state = 'back'
                            else :
                                now_state = 'forward'
                        return now_state
                    

            # Tactical enhancement idea:
            # - Make attacking-type defender more aggressive
            # - When ball is in own half, clear toward midfield strategic points
            # - When ball is in opponent half, block opponent advance
            # - If interception fails and ball far from our striker (>1m),
            #   consider transition to offensive support

            # To implement dynamic kick target switching:
            # - Reuse striker overall logic (fall check, avoidance, ball orientation)
            # - Only modify target aim position
            # self.defender_atc_aim should be initialized in __init__
            # PASSBALL_POS_DEF and priority configured in config
        if self.robot_func == 'atc' :

            now_state = self.state_tree()

        return now_state
    
    def calculate_circumcenter(self,ball_x,ball_y):
        if self.team == 'Red' :
            doorlx = cfg.DOOR_R_LEFT_X
            doorly = cfg.DOOR_R_LEFT_Y
            doorrx = cfg.DOOR_R_RIGHT_X
            doorry = cfg.DOOR_R_RIGHT_Y
        else :
            doorlx = cfg.DOOR_B_LEFT_X
            doorly = cfg.DOOR_B_LEFT_Y
            doorrx = cfg.DOOR_B_RIGHT_X
            doorry = cfg.DOOR_B_RIGHT_Y
        
        a = abs(doorly - doorry)
        b = math.sqrt((ball_x - doorlx)**2 + (ball_y - doorly)**2)
        c = math.sqrt((ball_x - doorrx)**2 + (ball_y - doorry)**2)

        perimeter = a + b + c

        ix = (a * ball_x + b * doorlx + c * doorrx) / perimeter
        iy = (a * ball_y + b * doorly + c * doorry) / perimeter

        return ix,iy

    


        
    


