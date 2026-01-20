from controller import Robot, Motion

#该控制器仅用来测试motion函数
# This controller is only used to test the motion function
robot = Robot()
timestep = int(robot.getBasicTimeStep())

iu = robot.getDevice("inertial_unit2")
iu.enable(timestep)
try:
    back_motion = Motion('../../motions/TurnRight08.motion')

except Exception as e:
    back_motion = None

if back_motion:
    
    back_motion.play()


while robot.step(timestep) != -1:

    if robot.getTime() > 10.0:
        break

