from controller import Robot, Keyboard, Motion
from VisionLib import VisionReviewer
import config as cfg
import math
import os



STATE_CHECK = False

class Logic:
    
    def __init__(self,Nao):

        #V1.2 新增获取机器人名字决定队伍
        self.name = Nao.getName()
        self.visionReviewer = VisionReviewer(Nao)
        self.timeStep = int(Nao.getBasicTimeStep())
        #V1.1 新增，作为机器人正常配置，保持机器人面朝前方

        #存储上一次的状态，用来循环右移左移
        self.last_state = None

        #存储上一次踢球前球的位置，用来判断球是否踢到了
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

        #根据命名设置不同的门的位置
        if self.name == 'Red_Defender_1' or self.name == 'Black_Defender_1':
            self.robot_func = 'atc'
        elif self.name == 'Red_Defender_2' or self.name == 'Black_Defender_2':
            self.robot_func = 'def'
        elif self.name == 'Black_Striker' or self.name == 'Red_Striker':
            self.robot_func = 'stricker'
        elif self.name == 'Black_Keeper' or self.name == 'Red_Keeper':
            self.robot_func = 'keeper'
        else: 
            print("defender wrong name")
        


        #监督学习数据导入
        self.kick_data = []
        self.get_trainning_file(cfg.TRAINING_FILE_NAME)
        self.kick_len = cfg.KICK_LEN

        #目标击球点动态规划
        #相关项目还在编写中
        self.can_i_kick = [False] * self.kick_len

        #状态flag
        self.back_state_flag = cfg.BALL_IS_BACK
        #状态树整体状态flag
        self.stricker_state = cfg.FIND_BACK_BALL

        self.defender_state = cfg.DEFENDER_FALL_CHACK

   
    def state_tree(self):

        #默认赋值防止出错
        path_state = 'Hand_Wave'
        finish_state = 'OK'

        #默认球门位置
        aim_x = self.door_x
        aim_y = self.door_y

        #更新所有数据
        self.visionReviewer.update(self.name)

        #防跌倒
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
            #判断球是否出现大幅度移动（被干扰）如果出现则重新开始逻辑树
            if self.check_ball_move(cfg.BALL_IS_MOVE) == True:
                self.stricker_state = cfg.FIND_BACK_BALL

            #避障相关
            avoid_code = self.visionReviewer.get_robot_avoid_message()
            if avoid_code != None:
                check_code = self.check_rob_obs(avoid_code,self.name)
                #避障优先级较高，但是在精细踢球阶段不会避障
                if (self.stricker_state != cfg.KICK_BALL) and (check_code != cfg.NO_OBS):
                    path_state = self.avoid_robot(check_code)
                    return path_state
                #由于代码复用，当自身为defender时一直需要避障，否则容易撞到自己人
                if self.robot_func == 'atc':
                    path_state = self.avoid_robot(check_code)
                    return path_state
            
            if self.stricker_state == cfg.FIND_BACK_BALL:
                if STATE_CHECK: print('state 0')
                path_state = self.find_back_ball()
            if path_state == finish_state or self.stricker_state == cfg.CLOSE_TO_BALL:
                if STATE_CHECK:print('state 1')
                path_state = self.close_to_ball(aim_x,aim_y)
            if path_state == finish_state or self.stricker_state == cfg.KICK_BALL:
                if STATE_CHECK:print('state 2')
                path_state = self.kick_ball(aim_x,aim_y)

            if path_state == finish_state :
                print('wrong!!!')
                path_state = 'Hand_Wave'
            return path_state
        

    #机器人跌倒检查
    def check_robot_fall(self):
        robot_x,robot_y,robot_z = self.visionReviewer.get_robot_position()
        if(robot_z != None ):
            if( robot_z < cfg.ROBOT_FALL_THRESHOLD ):
                return True 
        return False
    
    #障碍物判断
    def check_obstacle_in_path(self):
        return self.visionReviewer.check_obstacle(self.team,self.door_x,self.door_y,cfg.OBSTACLE_THRESHOLD)
    
    #更新目标击球点
    def update_door_position(self):

        self.can_i_kick = [True] * self.kick_len
        is_red = (self.team == 'Red')
        if self.robot_func == 'stricker':
            priority_list = cfg.PRIORITY_LIST
            passball_pos  = cfg.PASSBALL_POS
        if self.robot_func == 'atc':
            priority_list = cfg.PRIORITY_LIST_DEF
            passball_pos  = cfg.PASSBALL_POS_DEF
        for idx, target_name in enumerate(priority_list):
            raw_x, raw_y = passball_pos[target_name]
            if is_red:
                tx, ty = -raw_x, -raw_y
            else:
                tx, ty = raw_x, raw_y
            if not self.visionReviewer.check_obstacle(self.team, tx, ty, cfg.OBSTACLE_THRESHOLD):
                self.can_i_kick[idx] = False  
                return tx, ty
        print('Warning: No clear path found, using default priority target!')
        default_name = cfg.PRIORITY_LIST[0]
        dx, dy = cfg.PASSBALL_POS[default_name]
        return (-dx, -dy) if is_red else (dx, dy)
    
    #V1.6 新增避障
    #该函数解析avoid_code判断周围0.35m处是否有障碍
    #该参数来源于supervisor端口，无法作为参数修改
    #SB, SR, D1B, D1R, D2B, D2R (3^0 到 3^5)
    #输入为code,robotname
    #输出为障碍状态
    #0：无障碍
    #1：左侧障碍
    #2：右侧障碍
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
        

    #V1.5 将plan_path_to_ball函数拆分
    #该函数用来解决球到机器人身后的情况
    #具体注释参照V1.4——01/17版本

    #返回state = 1 表明球在身前
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
                        state = 'turn_left'
                        return state
                    else:
                        state = 'turn_right'
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
            self.stricker_state = cfg.CLOSE_TO_BALL
            self.back_state_flag = cfg.BALL_IS_BACK

        if self.stricker_state == cfg.CLOSE_TO_BALL:
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
                self.stricker_state = cfg.CLOSE_TO_BALL
        return finish_state

    #V1.5 将plan_path_to_ball函数拆分
    #该函数用来接近球，使球在机器人可控范围内
    #具体注释参照V1.4——01/17版本

    #仅当state = 1时进入该函数
    #返回state = 2 表明处于控球状态
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
            #距离很远的情况，优先找到机器人距离球的方向进行转向
            striker_x,striker_2 = self.get_striking_point(aim_x,aim_y,ball_x,ball_y,cfg.DISTANCE_TO_KICK)
            deg = self.calculate_angle_to_ball(robot_x,robot_y,striker_x,striker_2)

            if self.check_angle(robot_angle,deg,cfg.ANGLE_TOLERANCE_MAX) == False:
                state = self.change_angle_state(robot_angle,deg)
                return state
            else:
                
                state = 'forward'
                return state
        self.stricker_state = cfg.KICK_BALL
        return finish_state 


    #V1.5 将plan_path_to_ball函数拆分
    #该函数在机器人处于控球阶段时调用，用于精细走位进行踢球
    #具体注释参照V1.4——01/17版本

    #仅当state = 2 时进入该函数
    #返回state = 2 表明正在处理踢球动作
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
            if STATE_CHECK: print('已找到相关踢球数据,正在进行校对')
            dx = self.kick_data[idx][0]
            dy = self.kick_data[idx][1] 
            
            #优先对准角度
            if self.check_angle(robot_angle,deg,cfg.ANGLE_TOLERANCE) == False:
                state = self.change_angle_state(robot_angle,deg)
                return state
            dx_n = ball_x - robot_x
            dy_n = ball_y - robot_y
            if STATE_CHECK:print(f"dx:{dx},dy:{dy},nowdx:{dx_n},nowdy:{dy_n}")

            if dx_n - dx > cfg.TOLERANCE_X_B:
                state = 'back'
                return state
            if dx_n - dx < cfg.TOLERANCE_X_F:
                state = 'forward'
                return state
            
            if dy_n - dy < cfg.TOLERANCE_Y_L:
                state = 'sidestepleft'
                return state
            if dy_n - dy > cfg.TOLERANCE_Y_R:
                state = 'sidestepright'
                return state

            state = 'kickl'
            return state
                   
        else:
            print("未找到合适的踢球数据,请补充,当前deg为",deg)
            print("下面开始随机踢球")

            if self.check_angle(robot_angle,deg,cfg.ANGLE_TOLERANCE) == False:
                state = self.change_angle_state(robot_angle,deg)
                return state
            else:
                state = 'kickl'
        return state

    #-------------球位移检测
    #---------输入
    #threshold：球在一帧中滚动的上限
    #---------输出
    #True  球在这一帧中滚动超过上限
    #False 球在这一帧中滚动小于上限
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
        
    #-------------------机器人角度检测
    #---------输入
    #robot_angle：机器人现在的角度
    #aim_angle：目标角度
    #tolerance：容忍量
    #---------输出
    #False 机器人角度与目标角度相差大于容忍度
    #True  机器人角度与目标角度相差小于容忍度
    def check_angle(self,robot_angle,aim_angle,tolerance):

        adjust_ang = abs(robot_angle - aim_angle)
        if adjust_ang > math.pi :
            adjust_ang = 2*math.pi - adjust_ang
        if( adjust_ang > tolerance ):
            return False
        else:
            return True
        
    #-------------机器人角度旋转
    #--------输入
    #robot_angle：机器人现在的角度
    #aim_angle：目标角度
    #--------输出
    #motion_state
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
        
    #--------------------计算机器人朝球或者目标点的角度
    #-------------输入
    #robot_pos_x, robot_pos_y ：机器人坐标
    #ball_pos_x, ball_pos_y：目标点坐标
    #-------------输出
    #具体角度
    def calculate_angle_to_ball(self, robot_pos_x, robot_pos_y, ball_pos_x, ball_pos_y):
        rx, ry = robot_pos_x, robot_pos_y
        bx, by = ball_pos_x, ball_pos_y
        dx = bx - rx
        dy = by - ry
        angle = math.atan2(dy, dx)
        return angle

    #----------------------保证击球点与机器人球三点一线的情况下，计算最佳击球点
    #-------------输入
    #target_x, target_y：击球目标点坐标
    #ball_x, ball_y：球的坐标
    #distance：击球点与目标点的绝对距离
    #-------------输出
    #击球点坐标
    def get_striking_point(self,target_x, target_y, ball_x, ball_y, distance):
        dx = target_x - ball_x
        dy = target_y - ball_y
        line_len = math.sqrt(dx**2 + dy**2)
        if line_len == 0:
            return ball_x, ball_y
        
        strike_x = ball_x - (dx / line_len) * distance
        strike_y = ball_y - (dy / line_len) * distance
        
        return strike_x, strike_y
    
    #--------监督学习相关，获取训练文件数据，写入self.kick_data中
    #-------------输入
    #文件名称
    #-------------输出
    #None
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

##################################################################################################
##################################################################################################
###############-----------------------------待开发区域-----------------############################
##################################################################################################
##################################################################################################
    def get_state_keeper(self):

        self.visionReviewer.update(self.name)
        robot_x,robot_y,robot_z = self.reciever.get_robot_position()
        ball_x,ball_y = self.reciever.get_ball_position()
        robot_angle = self.reciever.get_robot_angle()

        now_state = 'Hand_Wave'

        if(robot_y != None ):
            if((robot_y - ball_y) < 0.0):
                now_state = 'turn_left'
            elif((robot_y - ball_y) > 0.12):
                now_state = 'turn_right'

        return now_state
    
    def get_state_defender(self):


        self.defender_state = cfg.DEFENDER_FALL_CHACK

        self.visionReviewer.update(self.name)
        robot_x,robot_y,robot_z = self.visionReviewer.get_robot_position()
        ball_x,ball_y = self.visionReviewer.get_ball_position()
        robot_angle = self.visionReviewer.get_robot_angle()

        if (ball_x is None) or (robot_x is None):
            state = 'Hand_Wave'
            return state

        now_state = 'Hand_Wave'



        #根据队伍初始化每个机器人的边界
        #进攻型可以走到对面半场去
        #y不用管，不可能走出去
        #注意黑队是反的
        if self.team == 'RED':
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
            

        #作为一个防御型的defender，需要做的事情就是在自己的半场待机，并且随时卡在球和球门之间的位置
        #首先需要写一个函数来找到这个点
        #已知球的位置，球门两边的位置，求三角形的内接圆心

        #逻辑很简单，优先检测摔倒，其次进行避障，最后到内接圆圆心
        #如果两次变动的距离小于threshold则原地不动，如果到达了圆心则不进行避障。
        #设置的可接受范围挺大的，两次出现变动也不一定会动，这里少用一个变量
        if self.robot_func == 'def' :
            #跌倒起立
            fall_state = self.check_robot_fall()
            if fall_state == True:
                orientation = self.visionReviewer.get_fall_orientation()
                if orientation == 'BACK':
                    return 'StandUp'
                else:
                    return 'StandUpB'
            else:
                #如果没有到位则避障
                #判断球是否出现大幅度移动（被干扰）如果出现则重新开始逻辑树
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
                        #没有到位也没有接收到有效的避障码
                        #do nothing
                #到位了，状态为cfg.DEFENDER_IN_POS,开始招手      
                else:
                    now_state = 'Hand_Wave'
            #优先检查角度，这里让defender笨一点，永远保持自己面朝前方
            #后续也是只进行前进后退左右移
            #self.normal_angle在最开始通过队名进行配置的值
            if self.check_angle(robot_angle,self.normal_angle,cfg.ANGLE_TOLERANCE) == False:
                state = self.change_angle_state(robot_angle,self.normal_angle)
                return state
            #如果角度正确
            #求出指定坐标，计算自身坐标到指定坐标的距离，如果大于设定值就移动
            #竟然没写计算距离的公式，后续需要全部替换
            else:
                aim_x,aim_y = self.calculate_circumcenter(ball_x,ball_y)
                #距离小于阈值，刷新状态并且招手
                if math.sqrt( (aim_x-robot_x)**2 + (aim_y-robot_y)**2 ) < cfg.DEFENDER_POS_THRESHOLD:
                    self.defender_state = cfg.DEFENDER_IN_POS
                    now_state = 'Hand_Wave'
                    return now_state
                #距离大于阈值，刷新状态并移动
                else:
                    self.defender_state = cfg.DEFENDER_FALL_CHACK
                    if abs( aim_y - robot_y ) >cfg.DEFENDER_XY_THRESHOLD:
                        if aim_y > robot_y: 
                            now_state = 'sidestepleft'
                        else: 
                            now_state = 'sidestepright'   
                        return now_state
                    #在移动之前先判断自己的位置，不能超过自己所属的边界
                    if self.team == 'red':
                        if robot_x < x_min :
                            now_state = 'Hand_Wave'
                            return now_state
                    else:
                        if robot_x > x_min :
                            now_state = 'Hand_Wave'
                            return now_state
                    if abs( aim_x - robot_x ) >cfg.DEFENDER_XY_THRESHOLD:
                        if aim_x > robot_x:
                            now_state = 'forward'
                        else :
                            now_state = 'back'
                        return now_state
                    

            # 我想让进攻型的Defender更具攻击性一点，当球出现在自己半场时，朝中场两个点或者另外几个点进行击球
            # 当球在对面半场的时候阻拦对方球员的防守
            # 不防守了，直接进攻，判断球与自家进攻手的距离，如果超过1m判断为被截断进攻，否则站在场边待机

            #怎么实现换点踢球，可以直接使用stricker的整体逻辑，判断摔倒避障，球的方位，只需要把aim值修改即可
            #self.defender_atc_aim 应该在init初始化
            #PASSBALL_POS_DEF和优先级在config进行初始配置
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

    


        
    


