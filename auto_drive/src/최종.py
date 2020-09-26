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
	global ar_pose_x, ar_pose_y, ar_pose_z, ar_pose_ox, ar_pose_oy, ar_pose_oz, ar_pose_ow, ar_time_recent, ar_time_now, ar_detect_flag
	global find_parking_flag
	ar_pose_x = 0
	ar_pose_y = 0
	ar_pose_z = 0
	ar_pose_ox = 0
	ar_pose_oy = 0
	ar_pose_oz = 0
	ar_pose_ow = 0
	#find_parking_flag = True
	if (msg.markers[0].id > 0):
		ar_time_now = time.time()
		if ((now - recent) >= 10 and ar_detect_flag <= 3):
			ar_detect_flag = ar_detect_flag + 1
		elif (ar_detect_flag == 4):
			find_parking_flag = True
			ar_detect_flag = ar_detect_flag + 1
		elif (ar_detect_flag > 4 and find_parking_flag == True):
			ar_pose_x = msg.markers[0].pose.pose.position.x
			ar_pose_y = msg.markers[0].pose.pose.position.y
			ar_pose_z = msg.markers[0].pose.pose.position.z
			ar_pose_ox = msg.markers[0].pose.pose.orientation.x
			ar_pose_oy = msg.markers[0].pose.pose.orientation.y
			ar_pose_oz = msg.markers[0].pose.pose.orientation.z
			ar_pose_ow = msg.markers[0].pose.pose.orientation.w
			state = drive_state.parking_moving

	ar_time_recent = time.time()

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
	vertices = np.array([[(-50, Height - 60),(240, Height / 2 + 60), (330, Height / 2 + 60), (Width+50, Height - 60)]], dtype=np.int32)
	roi_img = region_of_interest(img, vertices) # ROI 설정
	cv2.imshow("roi", roi_img)
	lines = cv2.HoughLinesP(roi_img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
	#line_arr = lines
	line_arr = np.squeeze(lines)
	print("Error : ", line_arr)

	print("Error size : ", type(line_arr))
	if line_arr.size() == 0:
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


	line_img = np.zeros((roi_img.shape[0], roi_img.shape[1], 3), dtype=np.uint8)
	draw_lines(line_img, L_lines, 0)
	draw_lines(line_img, R_lines, 0)

	#    print("len(L_lines) : ", len(L_lines))
	#    print("len(R_lines) : ", len(R_lines))
	lines_cnt = len(L_lines) + len(R_lines)
	#    cv2.imshow("across roi_img", roi_img)
	return lines_cnt


def hough_lines_cross(img, rho, theta, threshold, min_line_len, max_line_gap): # 허프 변환
	
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
	if (white_cnt > 7500):
		print("pass")
		stop_flag = True
	else:
		print("don't pass")
		stop_flag = False

	#cv2.rectangle(roi_th, (80, 400), (555, 360), (255,255,255), 3)
	'''
	global Width, Height
	vertices = np.array([[(-50,  Height), (100, 2 *  Height / 3), (540, 2 *  Height / 3), (Width + 50,  Height)]], dtype=np.int32)
	roi_img = region_of_interest(img, vertices) # ROI 설정
	cv2.imshow("roi", roi_img)

	lines = cv2.HoughLinesP(roi_img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
	line_arr = np.squeeze(lines)

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


	line_img = np.zeros((roi_img.shape[0], roi_img.shape[1], 3), dtype=np.uint8)
	draw_lines(line_img, L_lines, 0)
	draw_lines(line_img, R_lines, 0)

    
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
	print("cross final y_max : ", max(R_y_max, L_y_max))
	if max(R_y_max, L_y_max) > 400:
		stop_flag = True
	#print("L : ", L_lines)
	print("L len : ", len(L_lines))
	#print("R : ", R_lines)
	print("R len : ", len(R_lines))
	print("===================================================================================")
	lines_cnt = len(L_lines) + len(R_lines)

	#cv2.imshow("line_img", line_img)
	cv2.imshow("roi", roi_img)
	'''
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

	for i in lidar_scan[70:90]:
		if i < 0.65 :
			left_point.append(i)
	
	for i in lidar_scan[90:110]:	
		if i < 0.65 :
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


def go_left(center) :
	global lidar_scan
	angle = -((Width*6/10) - center)#(calculate_pid (Width*6/10, center,2.5,0,1))
	print("go left")
	right_point = []
	for i in lidar_scan[135:170]:	
		if i < 0.3 :
			right_point.append(i)

	if len(right_point) == 0 :
		Direction = check_obstacle_avoiding()
	else :
		Direction = obstacle_state.left
		print("라인유지")

	return Direction, angle

def go_right(center) :
	global lidar_scan
	angle = -((Width*4/10) - center)##(calculate_pid (Width*4/10, center,2.5,0,1))
	print("go right")
	left_point = []
	for i in lidar_scan[10:45]:	
		if i < 0.3 :
			left_point.append(i)

	if len(left_point) == 0 :
		Direction = check_obstacle_avoiding()
	else : 
		Direction = obstacle_state.right
		print("라인유지")

	return Direction, angle


def start():
	global pub
	global image
	global cap
	global Width, Heightgo_left
	global speed_rpm, lidar_scan
	global ar_pose_x, ar_pose_y, ar_pose_z, ar_pose_ox, ar_pose_oy, ar_pose_oz, ar_pose_ow
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

		#---------------------Lane Keeping----------------------#

		if(State == drive_state.drive):

			#--------- staright ---------#
			if abs(angle) <= 15:
				angle = -(Width/2 - center)
				#angle = (calculate_pid (Width/2, real_center,3.0,0.12,1.1) / 16.0) # 5.5,0.1,1.1 //17.0    next time input this 3.5 0.1 1.1 !!!!!  원래 내코드 PID 3.0,0.05,1.3) / 7.0
			#--------- curve ---------#
			else:
				angle = (calculate_pid (Width/2, center,3.5,0.05,1.3) / 7.0)

			speed = abs(calculate_pid(50,abs(angle),3.0,0.3,0.5)/3) # set value more precisely 
			if speed <= 15.0 :
				speed = 15.0

			for i in lidar_scan[60:120]:
				if i < 0.5:
					State = drive_state.obstacle
					Direction = check_obstacle_avoiding()
					#speed = 0


		#---------------------Obstacle Avoiding----------------------#

		if(State == drive_state.obstacle):
			speed = 10
			# print("obstacle ", Direction)
			# if Direction == obstacle_state.left:
			# 	Direction, angle = go_left(center)
			# 	count = 0
			# 	#obstacle_no_detection_time = time.time()
			# elif Direction == obstacle_state.right:
			# 	Direction, angle = go_right(center)
			# 	count = 0
			# 	#obstacle_no_detection_time = time.time()
			# else :
			# 	angle = -(Width/2 - center)
			# 	#angle = (calculate_pid (Width/2, center,2.5,0.1,1.1) / 12.0)
			# 	count += 1
			# print(count)
			# if count == 120 : #time.time() - obstacle_no_detection_time > 2 :
			# 	State = drive_state.drive
			# 	print("탈출")
			# 	count = 0
			# 	obstacle_finish = True
			print("하드코딩 디렉션: ",Direction)
			if Direction == obstacle_state.right :
				start = time.time()
				while True :
					drive(30, speed)
					print("오른쪽으로트는중")
					if time.time() - start > 0.3 :
						break
					rospy.sleep(0.01)
				start = time.time()
				while True :
					drive(-30, speed)
					print("왼쪽으로 트는중")
					if time.time() - start > 0.9:
						break
					rospy.sleep(0.01)
				start = time.time()
				while True :
					drive(30, speed)
					print("오른쪽으로트는중")
					if time.time() - start > 0.9 :
						break
					rospy.sleep(0.01)
				start = time.time()
				while True :
					drive(-30, speed)
					print("왼쪽으로트는중")
					if time.time() - start > 0.2:
						break
					rospy.sleep(0.01)
				State = drive_state.drive
		drive(angle, speed)


		print("\n\n---------------------------------------------------")
		print("Current State : ", State)
		print("angle, speed : ", angle, speed)
		# print("lidar value : ", lidar_scan[90])
		# print("ar_value : ")
		# print("x : ", ar_pose_x)
		# print("y : ", ar_pose_y)
		# print("z : ", ar_pose_z)
		# print("ox : ", ar_pose_ox)
		# print("oy : ", ar_pose_oy)
		# print("oz : ", ar_pose_oz)
		# print("ow : ", ar_pose_ow)
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

