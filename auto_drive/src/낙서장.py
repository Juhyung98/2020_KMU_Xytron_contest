#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
from geometry_msgs.msg import PoseStamped
import time
import sys
import os
import signal

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
Alvar = AlvarMarker()

pub = None
Width = 640
Height = 480
Offset = 340
Gap = 40

def img_callback(data):
	global image    
	image = bridge.imgmsg_to_cv2(data, "bgr8")

def pose_callback(msg):

	ar_pose_x = msg.markers[0].pose.pose.position.x
	ar_pose_y = msg.markers[0].pose.pose.position.y
	ar_pose_z = msg.markers[0].pose.pose.position.z
	ar_pose_ox = msg.markers[0].pose.pose.orientation.x
	ar_pose_oy = msg.markers[0].pose.pose.orientation.y
	ar_pose_oz = msg.markers[0].pose.pose.orientation.z
	ar_pose_ow = msg.markers[0].pose.pose.orientation.w

	print("x : ", ar_pose_x)
	print("y : ", ar_pose_y)
	print("z : ", ar_pose_z)
	print("ox : ", ar_pose_ox)
	print("oy : ", ar_pose_oy)
	print("oz : ", ar_pose_oz)
	print("ow : ", ar_pose_ow)

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
		# ㅈㅗㅂㅇㅡㄴㄱㅓ
		#self.__src = np.float32([[2, 379], [139, 314], [515, 315], [639 , 374]]) ## 원본이미지의 warping 포인트
		#self.__dst = np.float32([[100,480] , [100,0] , [540, 0],[540,480]]) ## 결과 이미지에서 src가 매칭될 점들

		#self.__src = np.float32([[0, 379], [175, 290], [503, 290], [640 , 379]]) ## 원본이미지의 warping 포인트
		#self.__dst = np.float32([[100,480] , [100,0] , [540, 0],[540,480]]) ## 결과 이미지에서 src가 매칭될 점들

		self.__src = np.float32([[-50, self.img_h], [195, 280], [460, 280], [self.img_w+150 , self.img_h]]) ## 원본이미지의 warping 포인트
		self.__dst = np.float32([[0,480] , [0,0] , [640, 0],[640,480]]) ## 결과 이미지에서 src가 매칭될 점들

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



class LaneDetector() :
	def __init__(self,bev) :
		self.__bev = bev

	def hough_line(self, binary_img, color_img, draw = True):
		global Offset, Gap, Width
		roi = binary_img[Offset : Offset+Gap, 0 : Width]
		all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,30)
		cv2.imshow("r", roi)

		if all_lines is None:
			return 0, 640

		left_lines, right_lines = self.divide_left_right(all_lines)
		# get center of lines
		binary_img, lpos = self.get_line_pos(binary_img, left_lines, left=True)
		binary_img, rpos = self.get_line_pos(binary_img, right_lines, right=True)

		# draw lines
		color_img = self.draw_lines(color_img, left_lines)
		color_img = self.draw_lines(color_img, right_lines)
		color_img = cv2.line(color_img, (230, 235), (410, 235), (255,255,255), 2)
									
		# draw rectangle
		color_img = self.draw_rectangle(color_img, lpos, rpos, offset=Offset)
		# show image
		cv2.imshow('calibration', color_img)

		return lpos, rpos


	# left lines, right lines
	def divide_left_right(self, lines):
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
	def get_line_params(self, lines):
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
	def get_line_pos(self, img, lines, left=False, right=False):
		global Width, Height
		global Offset, Gap

		m, b = self.get_line_params(lines)

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

	# draw lines
	def draw_lines(self, img, lines):
		global Offset
		for line in lines:
			x1, y1, x2, y2 = line[0]
			color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
			img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
		return img

	# draw rectangle
	def draw_rectangle(self, img, lpos, rpos, offset=0):
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
		cv2.rectangle(img, (315, 15 + offset),
						(325, 25 + offset),
						(0, 0, 255), 2)
		return img



def get_errorControl(valueForCentering):
		global error, errorList, error_i         ## record error
	
		desired_valueForCentering = 320
	
		error = valueForCentering - desired_valueForCentering
		error_i = error
		error_i = error_i + error
		#print(error)

		# if error > 500:                 ## compressed error (help D control)
		# 	error = error * 0.5

		errorList.insert(0, error)
		errorPrev = errorList[1]

		errorControl = error - errorPrev

		errorList.pop()                 ## errorList[1] is always errorPrev, errorList[[0] is always error(current error)
	
		return errorControl

def get_errorControl_speed(valueForCentering):
		global error_speed, errorList_speed          ## record error
	
		desired_speedForCentering = 50 #valueForCentering
	
		error_speed = valueForCentering - desired_speedForCentering
		#print(error_speed)

		# if error > 500:                 ## compressed error (help D control)
		# 	error = error * 0.5

		errorList_speed.insert(0, error_speed)
		errorPrev_speed = errorList_speed[1]

		errorControl_speed = error_speed - errorPrev_speed

		errorList_speed.pop()                 ## errorList[1] is always errorPrev, errorList[[0] is always error(current error)
	
		return errorControl_speed

def calculate_PID_speed(error_speed, errorControl_speed):
	global constant_PresaturatedToSpeed, presaturated_output_speed
	
	kp = 3.5                                               ## golden ratio kp for P control
	ki = 1    								## golden ratio ki for I control                         ###
	kd = 2                                                ## golden ratio kd for D control
	#constant_PresaturatedToAngle =  float(1) / 46           ## constant_PresaturatedToAngle size is related with kp

	proportional_output = kp * error_speed                        ## P control
	derivative_output = kd * (errorControl_speed)                 ## D control
	integral_output = 0                                                              ###
	integral_output = integral_output + ki * error_speed                                   ###
	presaturated_output_speed = proportional_output + integral_output + derivative_output   ###
	
	return presaturated_output_speed


def calculate_PID(error, errorControl):
	global constant_PresaturatedToAngle, presaturated_output,error_i
	
	kp = 6.0                                               ## golden ratio kp for P control
	ki = 0.5    								## golden ratio ki for I control                         ###
	kd = 1.2                                                ## golden ratio kd for D control
	#constant_PresaturatedToAngle =  float(1) / 46           ## constant_PresaturatedToAngle size is related with kp

	proportional_output = kp * error                        ## P control
	derivative_output = kd * (errorControl)                 ## D control
	integral_output = 0                                                              ###
	integral_output = integral_output + ki * error_i                                   ###
	presaturated_output = proportional_output + integral_output + derivative_output   ###
	
	return presaturated_output

def image_processing(img, bev) :
    # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # # define range of white color in HSV
    # # change it according to your need !
    # lower_white = np.array([0,0,0], dtype=np.uint8)
    # upper_white = np.array([0,0,255], dtype=np.uint8)

    # # Threshold the HSV image to get only white colors
    # mask = cv2.inRange(hsv, lower_white, upper_white)
    # # Bitwise-AND mask and original image
    # res = cv2.bitwise_and(img,img, mask= mask)


    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # gau = cv2.GaussianBlur(gray, (5, 5), 10)
    # ret, th = cv2.threshold(gau, 210, 255, cv2.THRESH_TOZERO)


    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # blur = cv2.GaussianBlur(gray, (5, 5), 3)
    # canny = cv2.Canny(blur, 50, 50) 

    # gray
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 60
    high_threshold = 70
    canny = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    warped_frame = bev.warpPerspect(canny)

    return warped_frame



def start():
	global pub
	global image
	global cap
	global Width, Height
	global error, errorList, error_speed, errorList_speed
	#global ar_pose_x,ar_pose_y,ar_pose_z

	rospy.init_node('auto_drive')
	pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
	error = 0.0
	error_speed = 0.0
	errorList = [0]
	errorList_speed = [0]
	image_sub = rospy.Subscriber("/camera/image_raw", Image, img_callback)
	ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, pose_callback)
	print "---------- Xycar A2 v1.0 ----------"
	rospy.sleep(2)

	bev = BirdEyeView(image)
	ldt = LaneDetector(bev)

	while True:
		while not image.size == (640*480*3):
			continue
		a=time.time()
		warped_frame = image_processing(image, bev)
		warped_frame2 = bev.warpPerspect(image)
		lpos, rpos =ldt.hough_line(warped_frame, warped_frame2)

		center = (lpos + rpos) / 2
		middle_x = -(Width/2 - center)
		print(middle_x)
		#pid control
		valueForCentering = middle_x
		errorControl = get_errorControl(valueForCentering)
		presaturated_output = calculate_PID(error,errorControl) / 18.0 # presaturated_output can be a angel!!

		cv2.imshow("warped_frame", warped_frame)
		cv2.imshow("warped_frame2",warped_frame2)
		#cv2.putText(final_frame, "Radius of curvature : " + str(presaturated_output), (10,  30), cv2.FONT_HERSHEY_SIMPLEX, 1,  (0, 255, 0),  2)
		#cv2.imshow('image',final_frame)

		speedForCentering = 0 

		if np.abs(presaturated_output) <= 10:
			speedForCentering = 40
		elif np.abs(presaturated_output) <= 20:
			speedForCentering = 30
		elif np.abs(presaturated_output) <= 30:
			speedForCentering = 20
		else :
			speedForCentering = 10	

		# errorControl_speed = get_errorControl_speed(speedForCentering)
		# presaturated_output_speed = calculate_PID_speed(error_speed,errorControl_speed) / 3
		# print(presaturated_output_speed)
		speedForCentering = 0
		drive(middle_x, speedForCentering)
		b=time.time()
		print("pipeline cost : ", b-a)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	rospy.spin()

if __name__ == '__main__':

    start()


	# wroking prototype #
	# 1. progress state : IMAGE -> BIRD_EYE_VIEW -> LANE_DECTECTING -> DRAW_REFERENCE_LINE -> PID_CONTROL
	# 2. TO_DO_LIST : MAKE ANGLE BY CURVATURE(USE PROFILE : IF ANGLE IS 50 WHAT THE HACK WILL BE THE APPROPERIATE CURVATURE)
	# 3. TEST & TUNING TIME
	# 4. MAKE TWO LOOKAHEAD POINT ONE FOR PID, SECOND FOR PURE_PURSUIT(ACTUALLY NOT REAL JUST BRING ALGORITHM : LET'S SEE)
	# 5. ALWAYS NEED BACKUP YALL SHOULD TAKE CARE OF YOUR OWN CODE 