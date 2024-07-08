#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/')
import cv2
import time
import math
import signal
import Camera
import argparse
import threading
import numpy as np
import yaml_handle
import SDK.PID as PID
import SDK.Misc as Misc
import SDK.Board as Board
import SDK.mecanum as mecanum

# 颜色追踪 Color tracking

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
car = mecanum.MecanumChassis()

servo1 = 1500
servo2 = 1500
servo_x = servo2
servo_y = servo1

color_radius = 0
color_center_x = -1
color_center_y = -1

car_en = False
wheel_en = False
size = (640, 480)
target_color = ()
__isRunning = False

car_x_pid = PID.PID(P=0.15, I=0.001, D=0.0001) # pid初始化 Pid Initialization
car_y_pid = PID.PID(P=1.00, I=0.001, D=0.0001)
servo_x_pid = PID.PID(P=0.06, I=0.0003, D=0.0006)  
servo_y_pid = PID.PID(P=0.06, I=0.0003, D=0.0006)

lab_data = None
servo_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)


# 初始位置  Initial Position 
def initMove():
    Board.setPWMServoPulse(1, servo1, 1000)
    Board.setPWMServoPulse(2, servo2, 1000)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# 设置蜂鸣器  Set Buzzer
def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

# 关闭电机 Turn off motor
def car_stop():
    car.set_velocity(0,90,0)  # 关闭所有电机 Turn off all motors


#设置扩展板的RGB灯颜色使其跟要追踪的颜色一致 The color of RGB light on expansion board is set to consistent with the tracked color 
def set_rgb(color):
    if color == "red":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        Board.RGB.show()
    elif color == "green":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
        Board.RGB.show()
    elif color == "blue":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
        Board.RGB.show()
    else:
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()


# 变量重置 Reset Variables
def reset():
    global target_color, car_en
    global servo1, servo2, wheel_en
    global servo_x, servo_y, color_radius
    global color_center_x, color_center_y
    
    car_en = False
    wheel_en = False
    servo1 = servo_data['servo1']
    servo2 = servo_data['servo2']
    servo_x = servo2
    servo_y = servo1
    target_color = ()
    car_x_pid.clear()
    car_y_pid.clear()
    servo_x_pid.clear()
    servo_y_pid.clear()
    color_radius = 0
    color_center_x = -1
    color_center_y = -1
    

# app初始化调用 APP Initialization
def init():
    print("ColorTracking Init")
    load_config()
    reset()
    initMove()

# app开始玩法调用  App starts calling game program
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorTracking Start")

# app停止玩法调用 App stops calling game program  
def stop():
    global __isRunning
    reset()
    initMove()
    car_stop()
    __isRunning = False
    set_rgb('None')
    print("ColorTracking Stop")

# app退出玩法调用 Exit the game
def exit():
    global __isRunning
    reset()
    initMove()
    car_stop()
    __isRunning = False
    set_rgb('None')
    print("ColorTracking Exit")

# 设置检测颜色 Set the target color
def setTargetColor(color):
    global target_color

    print("COLOR", color)
    target_color = color
    return (True, ())

# 设置车辆跟随 Set car following
def setVehicleFollowing(state):
    global wheel_en
    
    print("wheel_en", state)
    wheel_en = state
    if not wheel_en:
        car_stop()
    return (True, ())

# 找出面积最大的轮廓 Find the maximum contour 
# 参数为要比较的轮廓的列表 Parameters are the list of contours to be compared 
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    areaMaxContour = None
    for c in contours:  # 历遍所有轮廓 Loop over the contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积 Calculate the contour area 
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰 Only when the area is greater than 300, the maximum contour takes effect to filter the interference
                areaMaxContour = c
    return areaMaxContour, contour_area_max  # 返回最大的轮廓 Return the maximum contour

# 机器人移动逻辑处理 Movement Processing
def move():
    global __isRunning, car_en, wheel_en
    global servo_x, servo_y, color_radius
    global color_center_x, color_center_y
    
    img_w, img_h = size[0], size[1]
    
    while True:
        if __isRunning:
            if color_center_x != -1 and color_center_y != -1:
                # 摄像头云台追踪 Camera pan-tilt tracking
                # 根据摄像头X轴坐标追踪 Track based on the camera x-axis coordiate
                if abs(color_center_x - img_w/2.0) < 15: # 移动幅度比较小，则不需要动  The movement is quite small, so do not need to move
                    color_center_x = img_w/2.0
                servo_x_pid.SetPoint = img_w/2.0    # 设定  Set 
                servo_x_pid.update(color_center_x)  # 当前  Current
                servo_x += int(servo_x_pid.output)  # 获取PID输出值 Access PID output
                
                servo_x = 800 if servo_x < 800 else servo_x  # 设置舵机范围 Set servo range
                servo_x = 2200 if servo_x > 2200 else servo_x
                
                # 根据摄像头Y轴坐标追踪  Track based on the y-axis coordinate 
                if abs(color_center_y - img_h/2.0) < 10: # 移动幅度比较小，则不需要动 The movement is quite small, so do not need to move
                    color_center_y = img_h/2.0
                servo_y_pid.SetPoint = img_h/2.0   # 设定 Set 
                servo_y_pid.update(color_center_y) # 当前 Get the current centre position
                servo_y -= int(servo_y_pid.output) # 获取PID输出值 Access PID output
                
                servo_y = 1200 if servo_y < 1200 else servo_y # 设置舵机范围 Set servo range
                servo_y = 1900 if servo_y > 1900 else servo_y
                
                Board.setPWMServoPulse(1, servo_y, 20) # 设置舵机移动 Set servo pulse
                Board.setPWMServoPulse(2, servo_x, 20)
                time.sleep(0.01)
                
                # 车身跟随追踪 Car following
                if wheel_en:
                    # 根据目标大小进行远近追踪 Track according the target distance
                    if abs(color_radius - 100) < 10: 
                        car_y_pid.SetPoint = color_radius
                    else:
                        car_y_pid.SetPoint = 100
                    car_y_pid.update(color_radius)
                    dy = car_y_pid.output   # 获取PID输出值 Access PID output value
                    dy = 0 if abs(dy) < 15 else dy # 设置速度范围 set the speed range
                    
                    # 根据X轴舵机值进行追踪  Track according to the x-axis value of the servo 
                    if abs(servo_x - servo2) < 15:
                        car_x_pid.SetPoint = servo_x
                    else:
                        car_x_pid.SetPoint = servo2
                    car_x_pid.update(servo_x)
                    dx = car_x_pid.output   # 获取PID输出值 Access PID output value
                    dx = 0 if abs(dx) < 15 else dx # 设置速度范围 Set the speed range
                    
                    car.translation(dx, dy) # 设置机器人移动（X轴速度，Y轴速度） Robot is set to move (a-axis speed, y-axis speed)
                    car_en = True
                
                time.sleep(0.01)
                
            else:
                if car_en:
                    car_stop()
                    car_en = False
        else:
            if car_en:
                car_stop()
                car_en = False
            time.sleep(0.01)

# 运行子线程 Run child thread 
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# 机器人图像处理 Image processing
def run(img):
    global __isRunning, color_radius
    global color_center_x, color_center_y
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning:   # 检测是否开启玩法，没有开启则返回原图像 Detect whether the game is started, if not, the orginal image will be returned
        return img
     
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)   
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间 Convert image into LAB space
    
    area_max = 0
    areaMaxContour = 0
    for i in target_color:
        if i in lab_data:
            frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0],
                                          lab_data[i]['min'][1],
                                          lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0],
                                          lab_data[i]['max'][1],
                                          lab_data[i]['max'][2]))  #对原图像和掩模进行位运算  Bitwise operation of the original 
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 Opening 
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 Closing
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓  Find contour
            areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓 Find the maximum contour
    if area_max > 1000:  # 有找到最大面积 Find the maximum contour
        (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)  # 获取最小外接圆 Obtain the minimum circumcircle of an object
        color_radius = int(Misc.map(radius, 0, size[0], 0, img_w))
        color_center_x = int(Misc.map(center_x, 0, size[0], 0, img_w))
        color_center_y = int(Misc.map(center_y, 0, size[1], 0, img_h))
        if color_radius > 300:
            color_radius = 0
            color_center_x = -1
            color_center_y = -1
            return img
        
        cv2.circle(img, (color_center_x, color_center_y), color_radius, range_rgb[i], 2)
        
    else:
        color_radius = 0
        color_center_x = -1
        color_center_y = -1
            
    return img


#关闭前处理 Processing before exit
def manual_stop(signum, frame):
    global __isRunning
    
    print('Closing...')
    __isRunning = False
    car_stop()  # 关闭所有电机 Turn off all motors 
    initMove()  # 舵机回到初始位置 Motor returns to the initial position

if __name__ == '__main__':
    init()
    start()
    target_color = ('red',)
	wheel_en = True
    camera = Camera.Camera()
    camera.camera_open(correction=True) # 开启畸变矫正,默认不开启 Enable distortion correction which is not enabled by default
    signal.signal(signal.SIGINT, manual_stop)
    while __isRunning:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)  
            frame_resize = cv2.resize(Frame, (320, 240)) # 画面缩放到320*240  Resize the frame to 230*240
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()
