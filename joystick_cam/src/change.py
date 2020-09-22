#!/usr/bin/env python
# -*- coding: utf-8 -*

####################################################################
# 프로그램명 : joy_cam.py
# 작 성 자 : 자이트론
# 생 성 일 : 2020년 07월 23일
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import rospy, rospkg
import time
import numpy as np
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image

ack_msg = xycar_motor()
ack_publisher = None
bridge = CvBridge()
image = np.empty(shape=[0])
#Height, Width = image.shape[:2]
Height = 640
Width = 480

def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def callback_speed(msg_android_speed):
    global ack_msg
    #global speed
    ack_msg.speed = msg_android_speed.linear.x*50

def callback_steering(msg_android_steering):
    global ack_msg
    ack_msg.angle = msg_android_steering.angular.z*50

#Must Delete
#Detection Crosswalk  example
def detection_lane(img, speed):
    flag = False
    for y in range(360, 400):
    #for y in range(Height / 2, 2 * Height / 3):
        for x in range(250, 400):
        #for x in range(Width / 6, 5 * Width / 6):
            #check image, if lightness == 0 flag = False
            #print(img[y][x])
            if img[y][x] != 0:
                flag = True

        #flag == True is find crosswalk
        if flag == True:
            print("Flag is True")
            return 0
    if flag == False:
        return speed

def start():
    global image
    global Height, Width
    global ack_msg
    global ack_publisher
    global bridge
    rospy.init_node("joystick_cam")
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)

    rospy.Subscriber("android_motor_speed",Twist, callback_speed)
    rospy.Subscriber("android_motor_steering",Twist, callback_steering)
    ack_publisher = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    speed = 5

    while not rospy.is_shutdown():
        while not image.size == (640*480*3):
            continue
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gau = cv2.GaussianBlur(gray, (5, 5), 10)
        ret, th = cv2.threshold(gau, 127, 255, cv2.THRESH_BINARY)

        #cv2.imshow("frame", th)

    #print("speed : ", speed)
        speed = detection_lane(th, speed)
        if speed == 0:
            ack_msg.speed = 0

        #print("ack_speed : ", ack_msg.speed)
        #print("speed : ", speed)
        
        ack_publisher.publish(ack_msg)
        time.sleep(0.001)

if __name__ == '__main__':

    start()
