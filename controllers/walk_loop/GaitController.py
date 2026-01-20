import math
import numpy as np
from controller import Motion

'''
start_time:0 0
loop_time: 1 2
stop_time: 2 4
loopp_time:3 8
'''

class GaitController:

    def __init__(self, Nao):
        self.robot = Nao
        self.time_step = int(self.robot.getBasicTimeStep())
        
        # 内部时钟，用于生成正弦波
        self.time = 0.0
        self.motion_sum = 4
        #按位操作，记录flag状态
        self.flag = 0
        #配置最大时间和现在的时间
        self.time    = np.zeros(self.motion_sum)
        self.timemax = np.zeros(self.motion_sum)
        #初始化flag
        #Initialize flag
        #V1.1新增所有motion的flag
        #后续此部分需要更新，用输出存储状态比较简洁
        self.currentlyPlaying = None
        self.last_state = None
        self.start_time = 0
        self.start_flag = 0
        self.loop_time = 0
        self.loop_flag = 0
        self.stop_time = 0
        self.stop_flag = 0
        self.loopp_time = 0
        self.loopp_flag = 0

        self.now_motion_start = False


        #V1.1更新旋转运动的motion路径
        self.motions = {
            'walk_start': Motion('../../motions/walk_start.motion'),
            'walk_loop': Motion("../../motions/walk_loop.motion"),
            'walk_stop': Motion('../../motions/walk_stop.motion'),
            'loopp':Motion('../../motions/Forwards50.motion'),
        }

        self.timemax = [
            1.36, #walk_start
            1.12, #walk_loop
            1.32, #walk_stop
            6.76  #Forwards50
        ]

        #获取index用
        self.motion_indices = {k: i for i, k in enumerate(self.motions)}
             
        #V1.1更新，用来设置state
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

        #动作后停顿 确保当前动作结束
        if input_state == self.InputCheck:
            check_flags = self.check_flagss()
            if check_flags:
                input_state = self.FlagCheck
            else:
                return
        
        #确认下一帧是否是新动作
        if input_state == self.FlagCheck:
            if new_state == self.last_state:
                input_state = self.NewmotionCheck
            else:
                input_state = self.NewmotionCheck


        if input_state == self.NewmotionCheck or self.fist_run:
            self.fist_run = False
            #print("Changing state to:", new_state)
            self.currentlyPlaying = self.motions[new_state]
            self.currentlyPlaying.play()
            self.last_state = new_state
            self.now_motion_start = True

            index = self.motion_indices.get(new_state)
            mask = 1 << index 
            self.flag = self.flag | mask
            self.time[index] = self.robot.getTime()

            '''
            #V1.1 此处flag设置的操作后续也需要更新成数组异或操作
            if new_state == 'walk_start':
                print("start")
                self.start_time = self.robot.getTime()
                self.start_flag = 1
            elif new_state == 'walk_loop':
                print("loop")
                self.loop_time = self.robot.getTime()
                self.loop_flag = 1
            elif new_state == 'walk_stop':
                print("stop")
                self.stop_time = self.robot.getTime()
                self.stop_flag = 1
            elif new_state == 'loopp':
                print("loopp")
                self.stop_time = self.robot.getTime()
                self.stop_flag = 1
            '''
                
    #V1.1更新，flag检查
    #具体时间配置见motion文件
    #后续需修改
    def check_flags(self):
        if self.start_time == 1 and ((self.robot.getTime() - self.start_time) < 1.36):
            return 0
        else:
            self.start_time = 0 

        if self.loop_flag == 1 and ((self.robot.getTime() - self.loop_time) < 1.12):
            return 0
        else:
            self.loop_flag = 0 

        if self.stop_flag == 1 and ((self.robot.getTime() - self.stop_time) < 1.32):
            return 0
        else:
            self.stop_flag = 0 

        if self.loopp_flag == 1 and ((self.robot.getTime() - self.loopp_time) < 6.76):
            return 0
        else:
            self.loopp_flag = 0 
        
        
            

        if ((self.start_time == 0) and
            (self.loop_flag == 0) and 
            (self.stop_flag == 0) and
            (self.loopp_flag == 0) ):              
            return 1
        

    def check_flagss(self):
        if self.flag != 0:
            index = self.flag.bit_length() - 1
            if (self.robot.getTime() - self.time[index]) < self.timemax[index]:
                return 0
            else:
                self.flag = self.flag & (0)
                return 1
        else:
            return 1

