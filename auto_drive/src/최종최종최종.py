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
from std_msgs.msg import Float64
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
from sensor_msgs.msg import LaserScan
from simple_pid import PID
import time
import sys
import os
import signal
import enum

class drive_state(enum.Enum):
	init = -1
	drive = 0
	obstacle = 1
	stop = 2
	move_parking_spot = 3
	parking_moving = 4

class obstacle_state(enum.Enum):
	straight = 0
	left = 1
	right = 2

image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
Offset = 340 #origin340
Offset2 = 310
Gap = 40
speed_rpm = 0
breaking_speed = 0
across_bar = 18
right_max = 50
left_max = -50
find_parking_flag = False

def signal_handler(sig, frame):
	os.system('killall -9 python rosout')
	sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def img_callback(data):
	global image    
	image = bridge.imgmsg_to_cv2(data, "bgr8")

def speed_callback(msg):
	global speed_rpm
	speed_rpm = msg.data / 500.0 # if speed max 5000 constant will be 440
	#print("speed : ", speed_rpm)
	
def lidar_callback(msg) :
	global lidar_scan
	lidar_scan = msg.ranges

def pose_callback(msg):
	global ar_pose_x, ar_pose_y, ar_pose_z, ar_pose_ox, ar_pose_oy, ar_pose_oz, ar_pose_ow, ar_pose_id
	ar_pose_x = 0
	ar_pose_y = 0
	ar_pose_z = 0
	ar_pose_ox = 0
	ar_pose_oy = 0
	ar_pose_oz = 0
	ar_pose_ow = 0
	ar_pose_id = 0
	ar_pose_id = msg.markers[0].id
	ar_pose_x = msg.markers[0].pose.pose.position.x
	ar_pose_y = msg.markers[0].pose.pose.position.y
	ar_pose_z = msg.markers[0].pose.pose.position.z
	ar_pose_ox = msg.markers[0].pose.pose.orientation.x
	ar_pose_oy = msg.markers[0].pose.pose.orientation.y
	ar_pose_oz = msg.markers[0].pose.pose.orientation.z
	ar_pose_ow = msg.markers[0].pose.pose.orientation.w


def find_parking_spot(state):
	point =[]
	for i in lidar_scan[142:180]:#Angle : 38.9
			if i <= 0.75 :#INF is float("inf")
				point.append(i)
	if np.average(point) <= 0.75 :
		speed = 0
		return drive_state.move_parking_spot #move_parking_spot
	return state

def turn_parking():
	point =[]
	for i in lidar_scan[170:180]:
			if i <= 0.55 :#INF is float("inf")
				point.append(i)
	if np.average(point) <= 0.55 :
		return True
	return False

# publish xycar_motor msg
def drive(Angle, Speed): 
	global pub

	msg = xycar_motor()
	msg.angle = Angle
	msg.speed = Speed

	pub.publish(msg)

#pid
def calculate_pid(reference_input, feedback_input, proportional_gain, intergral_gain, derivative_gain):
	global errorList
	error = feedback_input - reference_input
	errorList.insert(0,error)
	errorPrev = errorList[1]
	errorControl = error - errorPrev
	errorList.pop()
	# error_sum = 0
	# error_sum = error_sum + error
	intergral_output = 0
	proportional_output = proportional_gain * error
	intergral_output = intergral_output + intergral_gain * error
	derivative_output = derivative_gain * errorControl

	presaturated_output = proportional_output + intergral_output + derivative_output
	return presaturated_output

# draw lines
def draw_lines(img, lines, Offset):
	#global Offset
	for line in lines:
		x1, y1, x2, y2 = line[0]
		color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
		img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
	return img

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
	center = (lpos + rpos) / 2

	cv2.rectangle(img, (lpos - 5, 15 + offset),
						(lpos + 5, 25 + offset),
						(0, 255, 0), 2)
	cv2.rectangle(img, (rpos - 5, 15 + offset),
						(rpos + 5, 25 + offset),
						(0, 255, 0), 2)
	cv2.rectangle(img, (center-5, 15 + offset),
						(center+5, 25 + offset),
						(0, 255, 0), 2)    
	# cv2.rectangle(img, (315, 15 + offset),
	# 					(325, 25 + offset),
	# 					(0, 0, 255), 2)
	return img

# left lines, right lines
def divide_left_right(lines):
	global Width

	low_slope_threshold = 0
	high_slope_threshold = 10

	# calculate slope & filtering with threshold
	slopes = []
	new_lines = []

	for line in lines:
		x1, y1, x2, y2 = line[0]

		if x2 - x1 == 0:
			slope = 0
		else:
			slope = float(y2-y1) / float(x2-x1)
        
		if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
			slopes.append(slope)
			new_lines.append(line[0])

	# divide lines left to right
	left_lines = []
	right_lines = []

	for j in range(len(slopes)):
		Line = new_lines[j]
		slope = slopes[j]

		x1, y1, x2, y2 = Line

		if (slope < 0) and (x2 < Width/2 - 90):
			left_lines.append([Line.tolist()])
		elif (slope > 0) and (x1 > Width/2 + 90):
			right_lines.append([Line.tolist()])

	return left_lines, right_lines

# get average m, b of lines
def get_line_params(lines):
	# sum of x, y, m
	x_sum = 0.0
	y_sum = 0.0
	m_sum = 0.0

	size = len(lines)
	if size == 0:
		return 0, 0

	for line in lines:
		x1, y1, x2, y2 = line[0]

		x_sum += x1 + x2
		y_sum += y1 + y2
		m_sum += float(y2 - y1) / float(x2 - x1)

	x_avg = x_sum / (size * 2)
	y_avg = y_sum / (size * 2)
	m = m_sum / size
	b = y_avg - m * x_avg

	return m, b

# get lpos, rpos
def get_line_pos(img, lines, Offset, left=False, right=False):
	global Width, Height, Gap
	#global Offset 

	m, b = get_line_params(lines)

	if m == 0 and b == 0:
		if left:
			pos = 0
		if right:
			pos = Width
	else:
		y = Gap / 2
		pos = (y - b) / m

		b += Offset
		x1 = (Height - b) / float(m)
		x2 = ((Height/2) - b) / float(m)

		cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), (255, 0,0), 3)

	return img, int(pos)

# show image and return lpos, rpos
def process_image(frame):
	global Width
	global Offset, Offset2, Gap
	# gray
	gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

	# blur
	kernel_size = 5
	blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

	# canny edge
	low_threshold = 50
	high_threshold = 150
	edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

	# HoughLinesP
	roi = edge_img[Offset : Offset+Gap, 0 : Width]
	roi2 = edge_img[Offset2 : Offset2+Gap, 0 : Width]
	all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)
	all_lines2 = cv2.HoughLinesP(roi2,1,math.pi/180,30,30,10)

	# divide left, right lines
	if all_lines is None and all_lines2 is None :
		lpos, rpos, lpos2, rpos2 = 0,640,0,640
	elif all_lines is None :
		lpos, rpos = 0, 640
		lpos2, rpos2 = lane_detection(frame, all_lines2, Offset2)		
	elif all_lines2 is None :
		lpos2,rpos2 = 0, 640
		lpos, rpos = lane_detection(frame, all_lines, Offset)		
	else : 
		lpos, rpos = lane_detection(frame, all_lines, Offset)
		lpos2, rpos2 = lane_detection(frame, all_lines2, Offset2)

	center = (lpos + rpos) / 2
	center2 = (lpos2 + rpos2) / 2
	real_center = center * 0.75 + center2 * 0.25

	frame = cv2.line(frame, (320, Offset+Gap), (320, Offset2),(0,0,255), 3)
	frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)

	# show image
	cv2.imshow('calibration', frame)

	return real_center, center, edge_img

def binary_process(frame):
	# gray
	gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

	# blur
	kernel_size = 5
	blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

	# binarization
	ret, binary_img = cv2.threshold(blur_gray, 190, 255, cv.THRESH_BINARY)

	return binary_img

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


def weighted_img(img, initial_img, a=1, b=1, c=0): # 두 이미지 operlap 하기
	return cv2.addWeighted(initial_img, a, img, b, c)


def hough_lines_across(img, rho, theta, threshold, min_line_len, max_line_gap): # 허프 변환
	global Width, Height
	#vertices = np.array([[(-50, Height - 60),(190, Height / 2 + 60), (450, Height / 2 + 60), (Width+50, Height - 60)]], dtype=np.int32)
	vertices = np.array([[(-50, Height - 60),(190, Height / 2), (330, Height / 2), (Width+50, Height - 60)]], dtype=np.int32)
	roi_img = region_of_interest(img, vertices) # ROI 설정
#	cv2.imshow("roi", roi_img)
	#line_img = np.zeros((roi_img.shape[0], roi_img.shape[1], 3), dtype=np.uint8)
	lines = cv2.HoughLinesP(roi_img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
	#line_arr = lines
	line_arr = np.squeeze(lines)
	# print("Error : ", line_arr)
	# print("Error type : ", type(line_arr))
	# print("Error size : ", line_arr.size)
	# print("Error dimen : ", line_arr.ndim)
	if line_arr.size == 1:
		return 0
	elif line_arr.ndim == 1:
		return 0
	#line_arr = lines
	#print("Error : ", (np.arctan2(line_arr[:,1] - line_arr[:,3], line_arr[:,0] - line_arr[:,2]) * 180) / np.pi)
	slope_degree = (np.arctan2(line_arr[:,1] - line_arr[:,3], line_arr[:,0] - line_arr[:,2]) * 180) / np.pi

	# 수평 기울기 제한
	line_arr = line_arr[np.abs(slope_degree)<160]
	slope_degree = slope_degree[np.abs(slope_degree)<160]
	# 수직 기울기 제한
	line_arr = line_arr[np.abs(slope_degree)>90]
	slope_degree = slope_degree[np.abs(slope_degree)>90]
	# 필터링된 직선 버리기
	L_lines, R_lines = line_arr[(slope_degree>0),:], line_arr[(slope_degree<0),:]
	L_lines, R_lines = L_lines[:,None], R_lines[:,None]


	#draw_lines(line_img, L_lines, 0)
	#draw_lines(line_img, R_lines, 0)

	#    print("len(L_lines) : ", len(L_lines))
	#    print("len(R_lines) : ", len(R_lines))
	lines_cnt = len(L_lines) + len(R_lines)
	#    cv2.imshow("across roi_img", roi_img)
	return lines_cnt


def hough_lines_cross(img, rho, theta, threshold, min_line_len, max_line_gap): # 허프 변환
	'''
	vertices = np.array([[(80, 400), (80, 360), (555, 360), (555, 400)]], dtype=np.int32)
	roi_th = region_of_interest(img, vertices) # ROI 설정

	white_cnt = 0
	black_cnt = 0
	for i in range(80, 555 + 1):
		for j in range(360, 400 + 1):
			if (roi_th[j][i] == 255):
				white_cnt += 1
			elif (roi_th[j][i] == 0):
				black_cnt += 1

	print("white_cnt : ", white_cnt)
	print("black_cnt : ", black_cnt)
	#print("roi_th[390][326] : ", roi_th[390][326])
	if (white_cnt > 5500):
		print("pass")
		stop_flag = True
	else:
		print("don't pass")
		stop_flag = False

	#cv2.rectangle(roi_th, (80, 400), (555, 360), (255,255,255), 3)
	'''
	global Width, Height
	stop_flag = False
	vertices = np.array([[(-50,  Height), (100, 2 *  Height / 3), (540, 2 *  Height / 3), (Width + 50,  Height)]], dtype=np.int32)
	roi_img = region_of_interest(img, vertices) # ROI 설정
#	cv2.imshow("roi", roi_img)
	#line_img = np.zeros((roi_img.shape[0], roi_img.shape[1], 3), dtype=np.uint8)

	lines = cv2.HoughLinesP(roi_img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
	line_arr = np.squeeze(lines)
	if (line_arr.size == 1):
		return False
	elif (line_arr.ndim == 1):
		return False

	slope_degree = (np.arctan2(line_arr[:,1] - line_arr[:,3], line_arr[:,0] - line_arr[:,2]) * 180) / np.pi

	# 수평 기울기 제한
	line_arr = line_arr[np.abs(slope_degree)<=180]
	slope_degree = slope_degree[np.abs(slope_degree)<=180]
	# 수직 기울기 제한
	line_arr = line_arr[np.abs(slope_degree)>=180]
	slope_degree = slope_degree[np.abs(slope_degree)>=180]
	# 필터링된 직선 버리기
	L_lines, R_lines = line_arr[(slope_degree>0),:], line_arr[(slope_degree<0),:]
	L_lines, R_lines = L_lines[:,None], R_lines[:,None]

	#draw_lines(line_img, L_lines, 0)
	#draw_lines(line_img, R_lines, 0)

    
	L_y_max = 0
	for line in L_lines:
		x1, y1, x2, y2 = line[0]
		L_y_max = max(y1, y2, L_y_max)
		#print("cross L_lines y_max : ", L_y_max)
	R_y_max = 0
	for line in R_lines:
		x1, y1, x2, y2 = line[0]
		R_y_max = max(y1, y2, R_y_max)
		#print("cross R_lines y_max : ", R_y_max)	
	#print("cross final y_max : ", max(R_y_max, L_y_max))
	if max(R_y_max, L_y_max) > 405:
		stop_flag = True
	#print("L : ", L_lines)
	#print("L len : ", len(L_lines))
	#print("R : ", R_lines)
	#print("R len : ", len(R_lines))
	#print("===================================================================================")
	lines_cnt = len(L_lines) + len(R_lines)

	#cv2.imshow("line_img", line_img)
#	cv2.imshow("roi", roi_img)
	
	return stop_flag

def lane_detection(frame, all_lines, off, draw = True):
	left_lines, right_lines = divide_left_right(all_lines)

	# get center of lines
	frame, lpos = get_line_pos(frame, left_lines, off, left=True)
	frame, rpos = get_line_pos(frame, right_lines, off, right=True)
	# draw lines
	frame = draw_lines(frame, left_lines, off)
	frame = draw_lines(frame, right_lines, off)
	# draw rectangle
	frame = draw_rectangle(frame, lpos, rpos, offset=off)

	return lpos, rpos


def check_obstacle_avoiding() :
	global lidar_scan

	left_point =[]
	right_point =[]

	for i in lidar_scan[60:90]:
		if i < 0.5 :
			left_point.append(i)
	
	for i in lidar_scan[90:120]:	
		if i < 0.5 :
			right_point.append(i)

	right_average = np.average(right_point)
	left_average = np.average(left_point)
	if len(right_point) == 0 :
		right_average = 0
	if len(left_point) == 0 :
		left_average = 0

	print("right, left : ",right_average, left_average)
	if right_average >  left_average :
		Direction = obstacle_state.left
	elif right_average < left_average :
		Direction = obstacle_state.right
	else :
		Direction = obstacle_state.straight

	return Direction


def go_left(speed, sec) :
	start = time.time()
	while True :
		drive(-25, speed)
		print("왼쪽으로 트는중")
		if time.time() - start > sec :
			break

def go_right(speed, sec) :
	start = time.time()
	while True :
		drive(25, speed)
		print("오른쪽으로트는중")
		if time.time() - start > sec :
			break

def start():
	global pub
	global image
	global cap
	global Width, Height, Offset
	global speed_rpm, lidar_scan
	global ar_pose_x, ar_pose_y, ar_pose_z, ar_pose_ox, ar_pose_oy, ar_pose_oz, ar_pose_ow, ar_pose_id
	global obstacle_no_detection_time
	global breaking_speed
	global across_bar
	global errorList
	global right_max, left_max
	State = drive_state.drive
	errorList = [0]
	angle = 0
	speed = 0
	count = 0 
	ar_count, ar_time = 0, 0

	rospy.init_node('auto_drive')
	pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
	speed_sub = rospy.Subscriber("/commands/motor/speed", Float64, speed_callback)
	image_sub = rospy.Subscriber("/camera/image_raw", Image, img_callback)
	lidar_sub = rospy.Subscriber("/scan", LaserScan, lidar_callback)
	ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, pose_callback)
	print "---------- Xycar A2 v1.0 ----------"
	rospy.sleep(2)

	detect_cross = False
	detect_across = False
	obstacle_finish = False

	while True:
		a = time.time()
		while not image.size == (640*480*3):
			continue
		copy_frame = image.copy()

		real_center,center, copy_image = process_image(image)
		error = center - Width/2
		lidar_calibration = int( np.arctan2(float(error), float(Offset)) *180/np.pi )
		#---------------------Lane Keeping----------------------#

		if(State == drive_state.drive):

			#--------- staright ---------#
			if abs(angle) <= 20:
				angle = -(Width/2 - center)/2.5 #lidar_calibration#(calculate_pid (Width/2, center,3.5,0.05,1.3) / 13.0)
				#angle = (calculate_pid (Width/2, real_center,3.0,0.12,1.1) / 16.0) # 5.5,0.1,1.1 //17.0    next time input this 3.5 0.1 1.1 !!!!!  원래 내코드 PID 3.0,0.05,1.3) / 7.0
			#--------- curve ---------#
			else:
				angle = (calculate_pid (Width/2, center,3.5,0.05,1.3) / 7.0)

			speed = abs(calculate_pid(50,abs(angle),3.0,0.3,0.5)/3) # set value more precisely 
			if speed <= 15.0 :
				speed = 15.0

			for i in lidar_scan[60:120]:
				if i < 0.6:
					State = drive_state.obstacle
					Direction = check_obstacle_avoiding()
					#speed = 0
			
			if obstacle_finish == True and detect_across == False and detect_cross == False:#한번 감지시 더이상 반복 X
				#drive(angle, speed)
				cnt = hough_lines_across(copy_image, 1, np.pi/180, 30, 40, 70) # 허프 변환
				#cv2.imshow("hough_across", hough_across)
				#across_img = weighted_img(hough_across, image) # 원본 이미지에 검출된 선 overlap
				#cv2.imshow("across_origin", across_img)
				if cnt > across_bar:#across_bar is 18 now
					detect_across = True
					speed = 3
					State = drive_state.stop
					drive(angle, speed)

			#-------------ar tag-----------------#
			

			if ar_pose_id > 0 and time.time() - ar_time >= 2.5 and ar_pose_z < 0.01 and ar_pose_z > 0.000000001:
				ar_count += 1
				ar_time = time.time()
				if ar_count == 1:  #4:
					State = drive_state.move_parking_spot


		#---------------------Obstacle Avoiding----------------------#

		if(State == drive_state.obstacle):
			speed = 7
			print("하드코딩 디렉션: ",Direction)
			if Direction == obstacle_state.right :
				go_right(speed, 0.15)
			elif Direction == obstacle_state.left :
				go_left(speed, 0.15)
			else :
				real_center,center, copy_image = process_image(image)
				angle = -(Width/2 - center)/2
				
			

			State = drive_state.drive
			obstacle_finish = True
		#---------------------Stop Line----------------------#

		if(State == drive_state.stop) :
			if detect_across == True and detect_cross == False:
				real_center,center, copy_image = process_image(image)
				speed = 3#-(speed - speed / 2)
				angle = -(Width/2 - center)/2
				#angle= right_max
				#drive(angle, speed)
				#copy_frame = binary_process(copy_frame)
				stop_flag = hough_lines_cross(copy_image, 1, np.pi/180, 30, 40, 5) # 허프 변환
				#cv2.imshow("hough_cross", hough_cross)
				#cross_img = weighted_img(hough_cross, image) # 원본 이미지에 검출된 선 overlap
				#cv2.imshow("cross_origin", cross_img)
				if stop_flag > 0:#across_bar is crosswalk
					detect_cross = True
			
			if detect_across == True and detect_cross == True :
				obstacle_finish = False
				real_center,center, copy_image = process_image(image)
				angle = -(Width/2 - center)/3
				drive(angle, breaking_speed)
				print("sleep")
				rospy.sleep(5)
				start = time.time()
				speed = 0
				while True :
					speed = 10
					drive(0, speed)
					print("왼쪽으로트는중")
					if time.time() - start > 0.5:
						break
				detect_across = False
				detect_cross = False
				State = drive_state.drive


		#---------------------move parking spot----------------------#

		if(State == drive_state.move_parking_spot):
			
			start_time = time.time()
			while True :
				real_center,center, copy_image = process_image(image)
				speed = 4#-(speed - speed / 2)
				angle = -(Width/2 - center)/2 
				drive(angle, speed)	
				point =[]
				for i in lidar_scan[165:175]:#Angle : 38.9
					if i <= 0.55 :#INF is float("inf")
						print("hihi")
						point.append(i)
				if np.average(point) <= 0.5 and time.time()-start_time > 4.5:
					drive(angle, 0)
					break
			# start = time.time()
			# while True :
			# 	real_center,center, copy_image = process_image(image)
			# 	angle = -(Width/2 - center)
			# 	drive(angle, 3)
			# 	print("직진중")
			# 	if time.time() - start > 5:
			# 		break

			start = time.time()
			while True :
				drive(0, 3)
				print("직진중")
				if time.time() - start > 1.9:
					break	

			start = time.time()
			while True :
				drive(50, -3)
				print("후진중")
				if time.time() - start > 2.3:
					break

			start = time.time()
			while True :
				drive(-50, -3)
				print("후진중")
				if time.time() - start > 2.3:
					break	
			State = drive_state.parking_moving
						

		#---------------------parking_moving----------------------#

		if(State == drive_state.parking_moving):
			# start = time.time()
			# while True :
			# 	drive(0, 0)
			# 	print("2전진을 위한 1보 후퇴")
			# 	if time.time() - start > 1:
			# 		break	
			if ar_pose_x < -0.05 and ar_pose_z < 0.0015:
				angle = 10
				speed = -calculate_pid(0.002, ar_pose_z,5.5,0.05,1.1) * 800
				if speed >= 4:
					speed = 4
			elif ar_pose_x > 0.05 and ar_pose_z < 0.0015:
				angle = -10
				speed = -calculate_pid(0.002, ar_pose_z,5.5,0.05,1.1) * 800
				if speed >= 4:
					speed = 4
			elif ar_pose_x < -0.05 and ar_pose_z > 0.0025:
				angle = 10
				speed = calculate_pid(0.002, ar_pose_z,5.5,0.05,1.1) * 800
				if speed >= 4:
					speed = 4
			elif ar_pose_x > 0.05 and ar_pose_z > 0.0025:
				angle = -10
				speed = calculate_pid(0.002, ar_pose_z,5.5,0.05,1.1) * 800
				if speed >= 4:
					speed = 4
			else:
				angel = 0
				speed = 0
					







		drive(angle, speed)


		print("\n\n---------------------------------------------------")
		print("Current State : ", State)
		print("angle, speed : ", angle, speed)
		print("lidar value : ", lidar_scan[90])
		print("ar_value : ")
		print("x : ", ar_pose_x)
		print("y : ", ar_pose_y)
		print("z : ", ar_pose_z)
		print("ox : ", ar_pose_ox)
		print("oy : ", ar_pose_oy)
		print("oz : ", ar_pose_oz)
		print("ow : ", ar_pose_ow)
		print("ar_count : ", ar_count)
		#print("detect_across : ", detect_across)
		#print("detect_cross : ", detect_cross)
		print("find_parking_flag : ", find_parking_flag)
		b = time.time()
		print("timestamp : ",b-a)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break


	rospy.spin()

if __name__ == '__main__':

	start()

