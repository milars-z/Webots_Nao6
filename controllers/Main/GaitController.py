import math
import numpy as np
from controller import Motion

class GaitController:

    def __init__(self, Nao):
        self.robot = Nao
        self.time_step = int(self.robot.getBasicTimeStep())
        
        # 内部时钟
        self.time = 0.0

        self.tick = 0

        #初始化flag
        #Initialize flag
        self.currentlyPlaying = None
        self.last_state = None

        self.now_motion_start = False

        #V1.2更新分布走路的motion
        self.motions = {
            'forward': Motion('../../motions/Forwards.motion'),
            'back': Motion("../../motions/Backwards.motion"),
            'sidestepleft': Motion('../../motions/SideStepLeft.motion'),
            'sidestepright': Motion('../../motions/SideStepRight.motion'),
            #'kick': Motion('../../motions/fast_shoot.motion'),
            'kickr': Motion('../../motions/testshootr.motion'),
            'kickl': Motion('../../motions/testshootl.motion'),
            'StandUp': Motion('../../motions/StandUpFromFront_m.motion'),
            'StandUpB':Motion('../../motions/StandUpFromBack.motion'),
            'turn_left_08':Motion('../../motions/TurnLeft08.motion'),
            'turn_left_40': Motion('../../motions/TurnLeft40.motion'),
            'turn_left_60': Motion('../../motions/TurnLeft60.motion'),
            'turn_right_08': Motion('../../motions/TurnRight08.motion'),
            'turn_right_40': Motion('../../motions/TurnRight40.motion'),
            'turn_right_60': Motion('../../motions/TurnRight60.motion'),
            'walk_start': Motion('../../motions/walk_start.motion'),
            'walk_loop': Motion("../../motions/walk_loop.motion"),
            'walk_stop': Motion('../../motions/walk_stop.motion'),
            'Hand_Wave': Motion('../../motions/HandWave.motion'),
            'SideRight_Start': Motion('../../motions/SideRight_Start_.motion'),
            'SideRight_Loop': Motion('../../motions/SideRight_Loop_.motion'),
            'SideRight_Stop': Motion('../../motions/SideRight_Stop_.motion'),
            'SideLeft_Start': Motion('../../motions/SideLeft_Start.motion'),
            'SideLeft_Loop': Motion('../../motions/SideLeft_Loop.motion'),
            'SideLeft_Stop': Motion('../../motions/SideLeft_Stop.motion'),
            'turn_left_180': Motion('../../motions/TurnLeft180.motion'),
            'back_Start':Motion('../../motions/Backwards_Start.motion'),
            'back_Loop':Motion('../../motions/Backwards_Loop.motion'),
            'back_Stop':Motion('../../motions/Backwards_Stop.motion'),
        }

        self.motion_sum = len(self.motions)
        #按位操作，记录flag状态
        self.flag = 0
        #配置最大时间和现在的时间
        self.motion_time    = np.zeros(self.motion_sum)
        self.timemax = np.zeros(self.motion_sum)
        
        #最大时间设置
        #注意！新增motion文件需要在此配置时间，且顺序和motion一致
        self.timemax = [
            2.6,  #forward
            2.6,  #back
            4.96, #turn_left
            5.76, #turn_right
            1.36, #kickr
            1.36, #kickl
            5.0,  #StandUp
            3.4,  #StandUpB
            1.56, #turn_left_08
            2.88, #turn_left_40
            4.52, #turn_left_60
            1.56, #turn_right_08
            2.88, #turn_right_40
            4.52, #turn_right_60
            1.40, #walk_start
            1.12, #walk_loop
            1.36, #walk_stop
            5.00, #Hand_Wave
            2.60, #SideRight_Start
            1.76, #SideRight_Loop
            1.36, #SideRight_Stop
            1.60, #SideLeft_Start
            1.84, #SideLeft_Loop
            1.48, #SideLeft_Stop
            9.00, #turnleft_180
            0.96, #back_Start
            0.80, #back_Loop
            0.84, #back_Stop
        ]

        #获取index用
        self.motion_indices = {k: i for i, k in enumerate(self.motions)}

        self.Init = 0
        self.InputCheck = 1
        self.FlagCheck = 2
        self.NewmotionCheck = 3
        self.PlayNewMotion = 4
        self.fist_run = True

    #V1.1 主要更新点，更新了state轮换的逻辑减少相关bug
    def manage_state_(self, new_state):

        input_state = self.Init
        
        #排除错误输入
        if new_state not in self.motions:
            return
        else:
            input_state = self.InputCheck

        #动作后停顿 确保当前动作结束 V1.2更换函数
        if input_state == self.InputCheck:
            check_flags = self.check_flagss()
            if check_flags:
                input_state = self.FlagCheck
            else:
                return
            

        if input_state == self.FlagCheck or self.fist_run:
            self.fist_run = False
            #print("Changing state to:", new_state)
            #上一帧不是前进且下一帧是前进，则walk_start

            #walk dispatch
            if self.last_state == 'walk_start'   and  new_state == 'forward':
                new_state = 'walk_loop'
            elif self.last_state == 'walk_loop'   and  new_state == 'forward':
                new_state = 'walk_loop'
            elif (self.last_state == 'walk_loop'  or self.last_state == 'walk_start') and new_state != 'forward':
                new_state = 'walk_stop'
            elif self.last_state != 'forward' and  new_state == 'forward':
                new_state = 'walk_start'

            #Right dispatch
            if self.last_state == 'SideRight_Start'   and  new_state == 'sidestepright':
                new_state = 'SideRight_Loop'
            elif self.last_state == 'SideRight_Loop'   and  new_state == 'sidestepright':
                new_state = 'SideRight_Loop'
            elif (self.last_state == 'SideRight_Loop' or self.last_state == 'SideRight_Start') and new_state != 'sidestepright':
                new_state = 'SideRight_Stop'
            elif self.last_state != 'sidestepright' and  new_state == 'sidestepright':
                new_state = 'SideRight_Start'

            #Left dispatch
            if self.last_state == 'SideLeft_Start'   and  new_state == 'sidestepleft':
                new_state = 'SideLeft_Loop'
            elif self.last_state == 'SideLeft_Loop'   and  new_state == 'sidestepleft':
                new_state = 'SideLeft_Loop'
            elif (self.last_state == 'SideLeft_Loop' or self.last_state == 'SideLeft_Start') and new_state != 'sidestepleft':
                new_state = 'SideLeft_Stop'
            elif self.last_state != 'sidestepleft' and  new_state == 'sidestepleft':
                new_state = 'SideLeft_Start'

            #Back dispatch
            if self.last_state == 'back_Start'   and  new_state == 'back':
                new_state = 'back_Loop'
            elif self.last_state == 'back_Loop'   and  new_state == 'back':
                new_state = 'back_Loop'
            elif (self.last_state == 'back_Loop' or self.last_state == 'back_Start') and new_state != 'back':
                new_state = 'back_Stop'
            elif self.last_state != 'back' and  new_state == 'back':
                new_state = 'back_Start'

            
            self.currentlyPlaying = self.motions[new_state]
            self.currentlyPlaying.play()
            self.last_state = new_state
            self.now_motion_start = True

            #V1.2新增，flag设置
            index = self.motion_indices.get(new_state)
            mask = 1 << index 
            self.flag = self.flag | mask
            self.motion_time[index] = self.robot.getTime()
        
    #V1.2新增 flag_check
    def check_flagss(self):
        if self.flag != 0:
            index = self.flag.bit_length() - 1
            if (self.robot.getTime() - self.motion_time[index]) < self.timemax[index]:
                return 0
            else:
                self.flag = self.flag & (0)
                return 1
        else:
            return 1