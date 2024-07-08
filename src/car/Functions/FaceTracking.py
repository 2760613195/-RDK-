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
import mediapipe as mp
import SDK.PID as PID
import SDK.Misc as Misc
import SDK.Board as Board
import SDK.mecanum as mecanum

# 人脸追踪 Face Tracking

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
 
car = mecanum.MecanumChassis()
# 导入人脸识别模块 Import face recognition module 
Face = mp.solutions.face_detection
# 自定义人脸识别方法，最小的人脸检测置信度0.5   Customize the face recognition method. The minimum detection confidence is 0.5
faceDetection = Face.FaceDetection(min_detection_confidence=0.5)

servo1 = 1500
servo2 = 1500
servo_x = servo2
servo_y = servo1

size = (640, 480)
__isRunning = False
center_x, center_y, area = -1, -1, 0

car_x_pid = PID.PID(P=0.150, I=0.001, D=0.0001)
car_y_pid = PID.PID(P=0.002, I=0.001, D=0.0001)
servo_x_pid = PID.PID(P=0.05, I=0.0001, D=0.0005)  # pid初始化  PID initialization
servo_y_pid = PID.PID(P=0.05, I=0.0001, D=0.0005)

servo_data = None
def load_config():
    global servo_data
    
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)


# 初始位置 Initial Position 
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

# 设置蜂鸣器 Set Buzzer
def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

# 关闭电机 Turn off motor 
def car_stop():
    car.set_velocity(0,90,0)  # 关闭所有电机 Turn off all motors  


# 变量重置 Reset Variables
def reset():
    global servo1, servo2
    global servo_x, servo_y
    global center_x, center_y, area
    
    servo1 = servo_data['servo1'] - 350
    servo2 = servo_data['servo2']
    servo_x = servo2
    servo_y = servo1
    car_x_pid.clear()
    car_y_pid.clear()
    servo_x_pid.clear()
    servo_y_pid.clear()
    center_x, center_y, area = -1, -1, 0
    

# app初始化调用  APP Initialization
def init():
    print("FaceTracking Init")
    load_config()
    reset()
    initMove()

# app开始玩法调用  App starts calling game program 
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("FaceTracking Start")

# app停止玩法调用  App stops calling game program 
def stop():
    global __isRunning
    reset()
    initMove()
    car_stop()
    __isRunning = False
    print("FaceTracking Stop")

# app退出玩法调用 Exit the game
def exit():
    global __isRunning
    reset()
    initMove()
    car_stop()
    __isRunning = False
    print("FaceTracking Exit")

# 机器人移动逻辑处理  Robot Movement Processing
car_en = False
def move():
    global __isRunning,car_en
    global servo_x, servo_y
    global center_x, center_y, area
    
    img_w, img_h = size[0], size[1]
    
    while True:
        if __isRunning:
            if center_x != -1 and center_y != -1:
                # 摄像头云台追踪 Camera pan-tilt tracking
                # 根据摄像头X轴坐标追踪 Track based on the camera x-axis coordiate
                if abs(center_x - img_w/2.0) < 15: # 移动幅度比较小，则不需要动 The movement is quite small, so do not need to move
                    center_x = img_w/2.0
                servo_x_pid.SetPoint = img_w/2.0 # 设定 Set 
                servo_x_pid.update(center_x)     # 当前 Get the current centre position
                servo_x += int(servo_x_pid.output)  # 获取PID输出值 Access PID output
                
                servo_x = 800 if servo_x < 800 else servo_x # 设置舵机范围  Set servo range
                servo_x = 2200 if servo_x > 2200 else servo_x
                
                # 根据摄像头Y轴坐标追踪  Track based on the camera y-axis coordinate 
                if abs(center_y - img_h/2.0) < 10: # 移动幅度比较小，则不需要动  The movement is quite small, so do not need to move
                    center_y = img_h/2.0
                servo_y_pid.SetPoint = img_h/2.0  
                servo_y_pid.update(center_y)
                servo_y -= int(servo_y_pid.output) # 获取PID输出值 Access PID output
                
                servo_y = 1000 if servo_y < 1000 else servo_y # 设置舵机范围 Set servo range
                servo_y = 1900 if servo_y > 1900 else servo_y
                
                Board.setPWMServoPulse(1, servo_y, 20) # 设置舵机移动 Set servo pulse
                Board.setPWMServoPulse(2, servo_x, 20)
                
                # 车身跟随追踪 Car following
                # 根据目标大小进行远近追踪 Track according the target distance
                if abs(area - 30000) < 2000 or servo_y < 1100:
                    car_y_pid.SetPoint = area
                else:
                    car_y_pid.SetPoint = 30000
                car_y_pid.update(area)
                dy = car_y_pid.output   # 获取PID输出值 Access PID output value
                dy = 0 if abs(dy) < 20 else dy # 设置速度范围 set the speed range
                
                # 根据X轴舵机值进行追踪 Track according to the x-axis value of the servo 
                if abs(servo_x - servo2) < 15:
                    car_x_pid.SetPoint = servo_x
                else:
                    car_x_pid.SetPoint = servo2
                car_x_pid.update(servo_x)
                dx = car_x_pid.output   # 获取PID输出值 Access PID output value
                dx = 0 if abs(dx) < 20 else dx # 设置速度范围 Set the speed range
                                
                car.translation(dx, dy) # 设置机器人移动（X轴速度，Y轴速度） Robot is set to move (a-axis speed, y-axis speed)
                car_en = True
                
            else:
                if car_en:
                    car_stop()
                    car_en = False
                time.sleep(0.01)
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
    global __isRunning, area
    global center_x, center_y
    global center_x, center_y, area
    
    if not __isRunning:   # 检测是否开启玩法，没有开启则返回原图像 Detect whether the game is started, if not, the orginal image will be returned
        return img
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    imgRGB = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB) # 将BGR图像转为RGB图像 Convert BGR image to RGB image
    results = faceDetection.process(imgRGB) # 将每一帧图像传给人脸识别模块  Transfer each frame of image to the module
    if results.detections:   # 如果检测不到人脸那就返回None    Return None 
	if face is not detected
        for index, detection in enumerate(results.detections): # 返回人脸索引index(第几张脸)，和关键点的坐标信息  return the face index and coordinate information of the kep point
            bboxC = detection.location_data.relative_bounding_box # 设置一个边界框，接收所有的框的xywh及关键点信息  Set a bounding box to receive the xywh of all boxes and ke poiny information
            
			# 将边界框的坐标点,宽,高从比例坐标转换成像素坐标 Convert the coordinate, width and height of bouding box to pixel coordinate
            bbox = (int(bboxC.xmin * img_w), int(bboxC.ymin * img_h),  
                   int(bboxC.width * img_w), int(bboxC.height * img_h))
            cv2.rectangle(img, bbox, (0,255,0), 2)  # 在每一帧图像上绘制矩形框 draw rectangle frame on each frame
            x, y, w, h = bbox  # 获取识别框的信息,xy为左上角坐标点  Access the information of recognition box. xy is the coordiate in the upper left corner 
            center_x =  int(x + (w/2))
            center_y =  int(y + (h/2))
            area = int(w * h)
    else:
        center_x, center_y, area = -1, -1, 0
            
    return img


#关闭前处理 Processing before exit
def manual_stop(signum, frame):
    global __isRunning
    
    print('Closings...')
    __isRunning = False
    car_stop()  # 关闭所有电机 Turn off all motors
    initMove()  # 舵机回到初始位置 Servo returns to the initial position

if __name__ == '__main__':
    init()
    start()
    camera = Camera.Camera()
    camera.camera_open(correction=True) # 开启畸变矫正,默认不开启 Enable distortion correction which is not enabled by default.
    signal.signal(signal.SIGINT, manual_stop)
    while __isRunning:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)  
            frame_resize = cv2.resize(Frame, (320, 240)) # 画面缩放到320*240 Resize the frame to 230*240
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()
