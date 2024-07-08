#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/')
import time
import signal
import HiwonderSDK.mecanum as mecanum

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！ Press Ctrl+C to exit the program, please try few more times if fail to exit!
----------------------------------------------------------
''')

chassis = mecanum.MecanumChassis()

start = True
#关闭前处理 Processing before exit 
def Stop(signum, frame):
    global start

    start = False
    print('关闭中...')
    chassis.set_velocity(0,0,0)  # 关闭所有电机 Turn off all motors 
    

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    while start:
        chassis.set_velocity(0,90,0.3)# 顺时针旋转,控制机器人移动函数,线速度0(0~100)，方向角90(0~360)，偏航角速度0.3(-2~2)  Clockwise turning. Movement control function. The linear velocity is 50 (0~100). The direcion angle is 180 (0-350). The jaw velocity is 0.3 (-2~2).
        time.sleep(3)
        chassis.set_velocity(0,90,-0.3)# 逆时针旋转 Counterclockwise turning
        time.sleep(3)
    chassis.set_velocity(0,0,0)  # 关闭所有电机 Turn off all motors
    print('Closed')

        
