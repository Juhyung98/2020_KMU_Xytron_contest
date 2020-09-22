#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : hough_drive_a2.py
# 작 성 자 : 자이트론
# 생 성 일 : 2020년 08월 12일
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import copy

import sys
import os
import signal

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()

pub = None
Width = 640
Height = 480
breaking_speed = 0

def img_callback(data):
	global image    
	image = bridge.imgmsg_to_cv2(data, "bgr8")    

# publish xycar_motor msg
def drive(Angle, Speed): 
	global pub

	msg = xycar_motor()
	msg.angle = Angle
	msg.speed = Speed

	pub.publish(msg)

class BirdEyeView() :
	def __init__(self, img) :
		self.__img = img
		self.img_h = self.__img.shape[0]
		self.img_w = self.__img.shape[1]
		self.__src = np.float32([[-50, self.img_h], [195, 230], [460, 230], [self.img_w+150 , self.img_h]]) ## 원본이미지의 warping 포인트
		self.__dst = np.float32([[100,480] , [100,0] , [540, 0],[540,480]]) ## 결과 이미지에서 src가 매칭될 점들
	def setROI(self,frame) :
		self.__roi = np.array([self.__src]).astype(np.int32)
		return cv2.polylines(frame, np.int32(self.__roi),True,(255,0,0),10) ## 10 두께로 파란선 그림
	def warpPerspect(self,frame) :
		M = cv2.getPerspectiveTransform(self.__src,self.__dst) ## 시점변환 메트릭스 얻어옴.
		return cv2.warpPerspective(frame, M, (self.img_w, self.img_h), flags=cv2.INTER_LINEAR) ## 버드아이뷰로 전환
	@property
	def src(self):
		return self.__src
	@property
	def dst(self):
		return self.__dst

def image_processing(bev, img):
    warped_frame = bev.warpPerspect(img)

    gray = cv2.cvtColor(warped_frame, cv2.COLOR_BGR2GRAY)
    gau = cv2.GaussianBlur(gray, (5, 5), 10)
    ret, th = cv2.threshold(gau, 127, 255, cv2.THRESH_BINARY)
    canny = cv2.Canny(th, 80, 90)

    return th, warped_frame
    #return canny, warped_frame

def detection_cross(img, detect_flag):
    #75cm (130, 240), (510, 320) (130, 240), (510, 320)
    global y_height
    flag = True
    for y in range(240, 320):
        for x in range(200, 400):
            if img[y][x] == 0:
                flag = False

        if flag == True:
            y_height = y
            print("cross is True")
            return True
        else:
            flag = True
    return detect_flag

def detection_across(img, detect_flag):
    #250cm (220, 30), (415, 80) 
    global x_height2
    global y_last2
    x_height2 = []
    y_last2 = []
    cnt = 0

    for x in range(220, 415):
        for y in range(30, 80):
            if img[y][x] == 255:
                cnt = cnt + 1
                #print(cnt)
                if cnt == 40:
                    y_last2.append(y)
                    x_height2.append(x)
                    print("across is True")
                elif cnt > 0:
                    break
        cnt = 0

    if len(x_height2) > 0:
        return True
    else:
        return detect_flag

def detection_across2(img, detect_flag):
    global x_height
    global y_last
    x_height = []
    y_last = []
    cnt = 0

    for x in range(220, 415):
        for y in range(220, 400):
            if img[y][x] == 255:
                cnt = cnt + 1
                #print(cnt)
                if cnt == 70:
                    y_last.append(y)
                    x_height.append(x)
                    print("across is True")
                elif cnt > 0:
                    break
        cnt = 0

    if len(x_height) > 0:
        return True
    else:
        return detect_flag

def start():
    global pub
    global image
    global cap
    global Width, Height
    global y_height
    global breaking_speed
    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(2)

    angle = 0
    speed = 4
    detect_cross = False
    detect_across = False

    #record Video code 
    #fps = image.get(cv2.CAP_PROP_FPS)
    #fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    #filename = 'auto_move_detection_crosswalk.avi'
    #out = cv2.VideoWriter(filename, fourcc, 20.0, (int(Width), int(Height)))

    while True:
        while not image.size == (640*480*3):
            continue
        detect_image = image

        bev = BirdEyeView(image)
        warped_frame, origin_img = image_processing(bev, image)

        drive(angle, speed)
        print("speed : ", speed)

        #횡단보도를 먼저 감지, 감지시 정지선을 감지하게 설계
        #정지선과 유사한 모습이 많기때문에 횡단보도를 감지하고 나서만 정지선을 인지할 수 있게 만듬.
        if detect_across == False:#한번 감지시 더이상 반복 X
            detect_across = detection_across(warped_frame, detect_across)

        if detect_across == True:
            speed = breaking_speed
            drive(angle, speed)
            for i in range(len(x_height)):
                #cv2.line(origin_img, (x_height[i], y_last[i] - 100), (x_height[i], y_last[i]), (0,0,255), 3)
                cv2.circle(origin_img, (x_height[i], y_last[i]), 2, (255, 0, 0), 2)
            detect_cross = detection_cross(warped_frame, detect_cross)

        detect_cross = detection_cross(warped_frame, detect_cross)
        if detect_cross == True:
            speed = breaking_speed
            drive(angle, speed)
            print("detect_cross")
            cv2.line(origin_img,(250, y_height),(400, y_height), (255, 0, 0), 3)
            #cv2.line(origin_img,(250, 250),(400, 250), (255, 255, 0), 3)
    
        cv2.rectangle(detect_image,(150, 290), (550, 320), (0,0,255), 2)
        cv2.putText(detect_image, "90cm", ((150, 288)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
        cv2.rectangle(detect_image,(260, 230), (390, 245), (0,255,0), 2)
        cv2.putText(detect_image, "210cm", ((260, 228)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)

        #cv2.rectangle(warped_frame,(130, 240), (510, 320), (255,255,255), 2)
        cv2.rectangle(warped_frame,(220, 30), (415, 80), (255,255,255), 2)

        cv2.imshow("image", detect_image)
        cv2.imshow("origin_img", origin_img)
        cv2.imshow("warped", warped_frame)

        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    break
        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            break
    rospy.spin()


    #cv2.destroyAllWindows()

if __name__ == '__main__':

    start()



