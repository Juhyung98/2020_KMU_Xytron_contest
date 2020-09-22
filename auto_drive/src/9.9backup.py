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
from ar_track_alvar_msgs.msg import AlvarMarkers
#from geometry_msgs.msg import PoseStamped
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
#poseStamp = PoseStamped()
pub = None
Width = 640
Height = 480
Offset = 340
Gap = 40
ar_pose_x=0.0
ar_pose_y=0.0
ar_pose_z=0.0

def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def pose_callback(msg):
    #global ar_pose_x,ar_pose_y,ar_pose_z
    ar_pose_x = msg.markers[0].pose.pose.position.x
    ar_pose_y = msg.markers.PoseStamped.pose.position.y
    ar_pose_z = msg.markers.PoseStamped.pose.position.z
    print(msg)
    

# publish xycar_motor msg
def drive(Angle, Speed): 
    global pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    pub.publish(msg)


WEIGHT = 800
LIMIT_ANGLE = 50
n_win = 60 # 좌,우 차선별 탐지 윈도우의 개수, 적어지면 샘플링이 적어지는 샘이라서 급커브 같은데서 영역을 정확히 못잡아냄
margin = 25 # 윈도우 margin
min_pix = 10 # 유효하다고 판단할 때 윈도우 박스 안 최소 픽셀
rate_of_validWindow = 0.65		# 유효한 차선이라고 판단하는 기준 


class BirdEyeView() :
	def __init__(self, img) :
		self.__img = img
		self.img_h = self.__img.shape[0]
		self.img_w = self.__img.shape[1]
		self.__src = np.float32([[-50, self.img_h], [195, 280], [460, 280], [self.img_w+150 , self.img_h]]) ## 원본이미지의 warping 포인트
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

class LaneDetector() :
	def __init__(self,bev) :
		self.__bev = bev

	def slidingWindows(self, binary_img, draw = True) :
		## sliding windows 방식으로 좌 우 차선의 영역을 탐지함.
		histogram = np.sum(binary_img[binary_img.shape[0]*3//5:,:], axis=0) # 영상의 3/5 이상에서 각 픽셀들의 같은 열 성분들을 합함
		width_mid_pt = np.int(histogram.shape[0]/2) ## 이미지의 width의 중점
		left_x_base = np.argmax(histogram[:width_mid_pt]) ## 히스토그램을 반으로 나눠서 히스토그램의 값이 첫번째로 높아지는 구간을 좌측 레인 탐지의 베이스로 잡는다.
		right_x_base = np.argmax(histogram[width_mid_pt:]) + width_mid_pt ## 히스토그램을 반으로 나눠서 우측 영역에서 히스토그램이 높이자는 구간을 우측 레인 탐지의 베이스로 잡는다.
		
		window_height = np.int(binary_img.shape[0]/n_win) ## 윈도우 높이
		non_zero = binary_img.nonzero() ## binary_img에서 값이 0 이 아닌 픽셀들의 좌표를 x 좌표 y 좌표로 각각 인덱싱해서 배출. 예를들어 0,0의 픽셀값이 0이 아니라면 array([array[0], array[0]]) 형태 
		non_zero_y = np.array(non_zero[0]) ## 0이아닌 y좌표 
		non_zero_x = np.array(non_zero[1]) ## 0이아닌 x좌표

		left_x_current = left_x_base 
		right_x_current = right_x_base 
		valid_left_line = True
		valid_right_line = True
		left_count = 0
		right_count = 0
		left_lane_indices = []
		right_lane_indices = []
		half_left_lane_indices = []
		half_right_lane_indices = []

		for window in range(n_win):
			## 각 윈도우는 버드아이뷰 상단점을 기준으로 y 윈도우들의 좌표값을 구한다 .
			## win_y_low는 이미지 최상단 y좌표 (height)에서 window+1 에 heght를 곱하면 그만큼 아래쪽이며
			## win_y_high 는 그 low 좌표와 짝을 이루는 위쪽 좌표이므로 window 에 height를 곱한다.
			win_y_low = binary_img.shape[0] - (window+1)*window_height
			win_y_high = binary_img.shape[0] - window*window_height

			## 좌측차선의 윈도우 위 아래 x좌표 
			win_x_left_low = left_x_current - margin
			win_x_left_high = left_x_current + margin

			## 우측 차선의 윈도우 위 아래 x 좌표 
			win_x_right_low = right_x_current - margin
			win_x_right_high = right_x_current + margin

			"""
			다음 아래 두 식은 다음과 같은 연산을 진행함.
			non_zero_y 의 모든 좌표 중 현재 window의 y 최소값, 최대값 보다 큰값에 대한 판별을 진행한 TF 테이블을 만들고
			x에 대해서도 같은 방식을 진행하여 TF 테이블을 만든다. 이 값들이 모두 T인 지점들은 1이 나오므로
			해당 점들을 non_zero 로 뽑아내고 x축 값만을 취함
			"""

			good_left_indices = ((non_zero_y >= win_y_low) & (non_zero_y < win_y_high) & (non_zero_x >= win_x_left_low) &  (non_zero_x < win_x_left_high)).nonzero()[0]
			good_right_indices = ((non_zero_y >= win_y_low) & (non_zero_y < win_y_high) & (non_zero_x >= win_x_right_low) &  (non_zero_x < win_x_right_high)).nonzero()[0]

			cv2.rectangle(binary_img, (win_x_left_low, win_y_low), ( win_x_left_high, win_y_high), (255,0,0),1)
			cv2.rectangle(binary_img, (win_x_right_low, win_y_low), ( win_x_right_high, win_y_high), (255,0,0),1)

			##위에서 추려낸 값을 append
			left_lane_indices.append(good_left_indices)
			right_lane_indices.append(good_right_indices)
			if window < n_win//2 :
				half_left_lane_indices.append(good_left_indices)
				half_right_lane_indices.append(good_right_indices)

			## 다음 윈도우 위치 업데이트 (주석 처리 되있는 것은 )
			if len(good_left_indices) > min_pix :
				pre_left_x_current = copy.deepcopy(left_x_current)
				left_x_current = np.int(np.mean(non_zero_x[good_left_indices]))
				left_count += 1	
			else :
				try:
					diff =int((left_x_current - pre_left_x_current )*1.2)
				except:
					diff = 0
				pre_left_x_current = copy.deepcopy(left_x_current)
				if np.abs(left_x_current + diff) < binary_img.shape[1]:
					left_x_current += diff
			if len(good_right_indices) > min_pix :     
				pre_right_x_current = copy.deepcopy(right_x_current)   
				right_x_current = np.int(np.mean(non_zero_x[good_right_indices]))
				right_count += 1
			else :
				try:
					diff = int((right_x_current - pre_right_x_current )*1.2)
				except:
					diff = 0
				pre_right_x_current = copy.deepcopy(right_x_current)   
				if np.abs(right_x_current + diff) < binary_img.shape[1]:
					right_x_current += diff

		## 배열 합치기   이 부분은 디텍팅 된 차선의 픽셀의 좌표 집합임.
		left_lane_indices = np.concatenate(left_lane_indices)
		right_lane_indices = np.concatenate(right_lane_indices)
		half_left_lane_indices = np.concatenate(half_left_lane_indices)
		half_right_lane_indices = np.concatenate(half_right_lane_indices)

		# 좌 우측 라인의 픽셀 위치들을 추출
		left_x = non_zero_x[left_lane_indices]
		left_y = non_zero_y[left_lane_indices] 
		right_x = non_zero_x[right_lane_indices]
		right_y = non_zero_y[right_lane_indices] 

		## 다항식으로 피팅한 좌표들을 2차다항식으로 피팅
		left_fit = np.polyfit(left_y, left_x, 2)
		right_fit = np.polyfit(right_y, right_x, 2)

		# 좌 우측 차선이 유효하지 않을 땐 유효한 차선을 가져다 씀 (화면 표시만을 위한 것)
		if left_count < n_win*rate_of_validWindow  :
			valid_left_line = False
			left_fit[:] = right_fit[:]
			left_fit[0] *= 1.1
			left_fit[2] -= 490
		if right_count < n_win*rate_of_validWindow  :
			valid_right_line = False
			right_fit[:] = left_fit[:]
			right_fit[0] *= 1.1
			right_fit[2] += 490

		info = {}
		info['left_fit'] = left_fit
		info['right_fit'] = right_fit
		info['non_zero_x'] = non_zero_x
		info['non_zero_y'] = non_zero_y
		info['left_lane_indices'] = left_lane_indices
		info['right_lane_indices'] = right_lane_indices
		info['half_left_lane_indices'] = half_left_lane_indices
		info['half_right_lane_indices'] = half_right_lane_indices
		info['valid_left_line'] = valid_left_line
		info['valid_right_line'] = valid_right_line

		return info

	def drawFitLane(self, frame, binary_warped_frame, info) :
		height,width = binary_warped_frame.shape

		left_fit = info['left_fit']
		right_fit = info['right_fit']	 
		nonzerox = info['non_zero_x']
		nonzeroy = info['non_zero_y'] 
		left_lane_inds = info['left_lane_indices']
		right_lane_inds = info['right_lane_indices']
		half_left_lane_indices = info['half_left_lane_indices']
		half_right_lane_indices = info['half_right_lane_indices'] 

		M = cv2.getPerspectiveTransform(self.__bev.dst,self.__bev.src) ## 시점변환용 메트릭스. 
		##Bird Eye View 에서는 src -> dst 로의 시점 전환을 수행하였으므로
		##원본 좌표로 복구를 위해서 dst->src 로 변환을 해야함

		plot_y = np.linspace(0,binary_warped_frame.shape[0]-1, binary_warped_frame.shape[0])

		left_fit_x = left_fit[0] * plot_y**2 + left_fit[1] * plot_y +left_fit[2]
		right_fit_x = right_fit[0] * plot_y**2 + right_fit[1] * plot_y + right_fit[2]

		warp_zero = np.zeros_like(binary_warped_frame).astype(np.uint8) 
		color_warp = np.dstack((warp_zero, warp_zero, warp_zero)) 
		## np.dstack => 양쪽 행렬의 element wise 로 값을 짝지어 열벡터를 만든다.

		lefts = np.array([np.transpose(np.vstack([left_fit_x, plot_y]))])
		rights = np.array([np.flipud(np.transpose(np.vstack([right_fit_x, plot_y])))]) 
		## np.flidud을 row기준으로 위아래 순서를 뒤바꿔버림.
		points = np.hstack((lefts,rights))

		cv2.fillPoly(color_warp, np.int_([points]),(0,0,255))
		cv2.polylines(color_warp, np.int32([lefts]), isClosed=False, color = (255,255,0),thickness = 10)
		cv2.polylines(color_warp, np.int32([rights]), isClosed=False, color = (255,0,255),thickness = 10)
		cv2.imshow("color_warp", color_warp )
		new = cv2.warpPerspective(color_warp, M, (width, height))
		output = cv2.addWeighted(frame,1,new,0.5,0)

		"""
		Calculate radius of curvature in meters
		"""
		y_eval = 480  # 이미지의 y 크기

		# 1픽셀당 몇 미터인지 환산
		ym_per_pix = 1.8/280
		xm_per_pix = 0.845/610 

		# 좌우측 차선의 좌표 추출
		leftx = nonzerox[left_lane_inds]
		lefty = nonzeroy[left_lane_inds]
		rightx = nonzerox[right_lane_inds]
		righty = nonzeroy[right_lane_inds]
	
		# 다항식으로 피팅한 좌표들을 2차다항식으로 피팅
		left_fit_cr = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
		right_fit_cr = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)

		# 2차원 그래프 visualizingc
		# plt.plot(leftx*xm_per_pix, lefty*ym_per_pix)
		# plt.plot(rightx*xm_per_pix, righty*ym_per_pix)
		# plt.xlabel('x - axis') 
		# # naming the y axis 
		# plt.ylabel('y - axis') 
		# # giving a title to my graph 
		# plt.title('My first graph!') 
		# # function to show the plot 
		# plt.show() 

		# 반지름을 이용한 곡률 계산
		left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix+ left_fit_cr[1])**2)**1.5) / (2*left_fit_cr[0])
		right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / (2*right_fit_cr[0])

		return output, left_curverad, right_curverad


def image_processing(img, bev) :
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	gau = cv2.GaussianBlur(gray, (5, 5), 10)
	ret, th = cv2.threshold(gau, 127, 255, cv2.THRESH_BINARY)

    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # blur = cv2.GaussianBlur(gray, (5, 5), 3)
    # canny = cv2.Canny(blur, 80, 70) 
    warped_frame = bev.warpPerspect(th)
    return warped_frame



def start():
    global pub
    global image
    global cap
    global Width, Height
    #global ar_pose_x,ar_pose_y,ar_pose_z

    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/camera/image_raw", Image, img_callback)
    ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, pose_callback)
    print "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(2)

    bev = BirdEyeView(image)
    ldt = LaneDetector(bev)

    while True:
        while not image.size == (640*480*3):
            continue
		
	warped_frame = image_processing(image, bev)
        warped_frame2 = bev.warpPerspect(image)

        info = ldt.slidingWindows(warped_frame)
        final_frame, left_curverad, right_curverad = ldt.drawFitLane(image, warped_frame, info)
        left_curvature, right_curvature= 1/left_curverad, 1/right_curverad

	print(ar_pose_x)

	if info['valid_left_line'] & info['valid_right_line'] :
            final_curvature = WEIGHT*(left_curvature + right_curvature)/2
        elif info['valid_left_line'] :
            final_curvature = WEIGHT*left_curvature
        elif info['valid_right_line'] :
            final_curvature = WEIGHT*right_curvature
        else :
            final_curvature = 0

        if final_curvature >LIMIT_ANGLE :
            final_curvature = LIMIT_ANGLE
        elif final_curvature < -LIMIT_ANGLE :
            final_curvature = -LIMIT_ANGLE

        #cv2.imshow("roi_frame", roi_frame)
        cv2.imshow("warped_frame", warped_frame)
        cv2.imshow("warped_frame2",warped_frame2)
        cv2.putText(final_frame, "Radius of curvature : " + str(final_curvature), (10,  30), cv2.FONT_HERSHEY_SIMPLEX, 1,  (0, 255, 0),  2)
        cv2.imshow('image',final_frame)

        drive(final_curvature, 4)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rospy.spin()

if __name__ == '__main__':

    start()



