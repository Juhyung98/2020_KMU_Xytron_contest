#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
across_bar = 18

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

def image_processing(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    gau = cv2.GaussianBlur(gray, (5, 5), 10)

    canny = cv2.Canny(gray, 80, 90)

    return canny

def region_of_interest(img, vertices, color3=(255,255,255), color1=255): # ROI 셋팅

    mask = np.zeros_like(img) # mask = img와 같은 크기의 빈 이미지
    
    if len(img.shape) > 2: # Color 이미지(3채널)라면 :
        color = color3
    else: # 흑백 이미지(1채널)라면 :
        color = color1
        
    # vertices에 정한 점들로 이뤄진 다각형부분(ROI 설정부분)을 color로 채움 
    cv2.fillPoly(mask, vertices, color)
    
    # 이미지와 color로 채워진 ROI를 합침
    ROI_image = cv2.bitwise_and(img, mask)
    return ROI_image

def draw_lines(img, lines, color=[0, 0, 255], thickness=2): # 선 그리기
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)

def hough_lines_across(img, rho, theta, threshold, min_line_len, max_line_gap): # 허프 변환
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_arr = np.squeeze(lines)

    slope_degree = (np.arctan2(line_arr[:,1] - line_arr[:,3], line_arr[:,0] - line_arr[:,2]) * 180) / np.pi

    # 수평 기울기 제한
    line_arr = line_arr[np.abs(slope_degree)<170]
    slope_degree = slope_degree[np.abs(slope_degree)<170]
    # 수직 기울기 제한
    line_arr = line_arr[np.abs(slope_degree)>95]
    slope_degree = slope_degree[np.abs(slope_degree)>95]
    # 필터링된 직선 버리기
    L_lines, R_lines = line_arr[(slope_degree>0),:], line_arr[(slope_degree<0),:]
    L_lines, R_lines = L_lines[:,None], R_lines[:,None]


    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, L_lines)
    draw_lines(line_img, R_lines)
    '''
    print("L_lines")
    print(len(L_lines))
    print(L_lines)
    print("R_lines")
    print(len(R_lines))
    '''
    lines_cnt = len(L_lines) + len(R_lines)
    return line_img, lines_cnt

def hough_lines_cross(img, rho, theta, threshold, min_line_len, max_line_gap): # 허프 변환
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_arr = np.squeeze(lines)

    slope_degree = (np.arctan2(line_arr[:,1] - line_arr[:,3], line_arr[:,0] - line_arr[:,2]) * 180) / np.pi

    # 수평 기울기 제한
    line_arr = line_arr[np.abs(slope_degree)<=180]
    slope_degree = slope_degree[np.abs(slope_degree)<=180]
    # 수직 기울기 제한
    line_arr = line_arr[np.abs(slope_degree)>=170]
    slope_degree = slope_degree[np.abs(slope_degree)>=170]
    # 필터링된 직선 버리기
    L_lines, R_lines = line_arr[(slope_degree>0),:], line_arr[(slope_degree<0),:]
    L_lines, R_lines = L_lines[:,None], R_lines[:,None]


    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, L_lines)
    draw_lines(line_img, R_lines)
    #cv2.imshow("line_img", line_img)
    lines_cnt = len(L_lines) + len(R_lines)
    return line_img, lines_cnt

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

def weighted_img(img, initial_img, a=1, b=1, c=0): # 두 이미지 operlap 하기
    return cv2.addWeighted(initial_img, a, img, b, c)

def start():
    global pub
    global image
    global Width, Height
    global breaking_speed
    global across_bar
    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(2)

    angle = 0
    speed = 4
    detect_cross = False
    detect_across = False

    while True:
        while not image.size == (640*480*3):
            continue

        canny_img = image_processing(image)
    
        vertices = np.array([[(-50, Height - 60),(Width / 4, Height / 2 + 20), (3 * Width / 4, Height / 2 + 20), (Width + 50, Height - 60)]], dtype=np.int32)
        roi_img = region_of_interest(canny_img, vertices) # ROI 설정

        drive(angle, speed)
        #print("speed : ", speed)

        #횡단보도를 먼저 감지, 감지시 정지선을 감지하게 설계
        #정지선과 유사한 모습이 많기때문에 횡단보도를 감지하고 나서만 정지선을 인지할 수 있게 만듬.
        if detect_across == False and detect_cross == False:#한번 감지시 더이상 반복 X
            hough_across, cnt = hough_lines_across(roi_img, 1, 1 * np.pi/180, 30, 30, 20) # 허프 변환
            cv2.imshow("hough_across", hough_across)
            across_img = weighted_img(hough_across, image) # 원본 이미지에 검출된 선 overlap
            cv2.imshow("across_origin", across_img)
            if cnt > across_bar:#across_bar is 18 now
                detect_across = True
                #speed = 4
                #drive(angle, speed)

        if detect_across == True and detect_cross == False:
            speed = 4#-(speed - speed / 2)
            drive(angle, speed)
            hough_cross, cnt = hough_lines_cross(roi_img, 1, 1 * np.pi/180, 30, 60, 20) # 허프 변환
            cv2.imshow("hough_cross", hough_cross)
            cross_img = weighted_img(hough_cross, image) # 원본 이미지에 검출된 선 overlap
            cv2.imshow("cross_origin", cross_img)
            if cnt > 0:#across_bar is crosswalk
                detect_cross = True

        
        if detect_across == True and detect_cross == True:
            speed = breaking_speed
            drive(angle, speed)
            rospy.sleep(5)
            speed = 4
            drive(angle, speed)
            detect_across = False
            detect_cross = False

        cv2.imshow("image", image)
        cv2.imshow("canny_img", canny_img)

        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            break
    rospy.spin()


    #cv2.destroyAllWindows()

if __name__ == '__main__':

    start()



