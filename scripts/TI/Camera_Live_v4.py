# import the necessary packages
import time
import cv2
import math as m
import numpy as np
import json


# camera Calibration givin the calibrated image
def cam_calib(orig_frame_, cam_matrix_, cam_disto_):
	#h,	w = orig_frame_.shape[:2]

	#map1, map2 = cv2.fisheye.initUndistortRectifyMap(cam_matrix_, cam_disto_, np.eye(3), cam_matrix_, (w,h), cv2.CV_16SC2)
	#undistorted_img = cv2.remap(orig_frame_, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
	center_image_ = np.array([orig_frame_.shape[1]/2, orig_frame_.shape[0]/2])
	return orig_frame_, center_image_

# calculate height of the airship
def center_camera(gyro_, height_, center_image_):
	shift_ =	height_*np.tan(gyro_)
	center_camera_ = center_image_-shift_[0:2]
	#center_camera_ = np.array([center_image_[0], center_image_[1]]) 
	return center_camera_

# closest distnce to a point in the line
def dis(lines_, center_image_, cam_matrix_, height_):
	center_image_ = np.array(center_image_).astype(float)
	# if point a_y is smaller than b_y invert values of a and b to have same triangle
	if lines_[1] < lines_[3]:
			b = np.array([lines_[0], lines_[1]])
			a = np.array([lines_[2], lines_[3]])
	else:
			a = np.array([lines_[0], lines_[1]])
			b = np.array([lines_[2], lines_[3]])

	norm_a = cv2.norm(center_image_, a, cv2.NORM_L2)
	norm_b = cv2.norm(b, center_image_, cv2.NORM_L2)
	norm_base = cv2.norm(b, a, cv2.NORM_L2)
	dir_ = b - a
	if center_image_[1] > a[1]:
			coor_pix_ = a
	elif center_image_[1] < b[1]:
			coor_pix_ = b
	else:
			lenght = ((dir_[0]*(center_image_[0] - a[0])) + (dir_[1]* (center_image_[1] - a[1]))) / (dir_[0]**2 + dir_[1]**2)
			coor_pix_ = np.array(a + dir_ * lenght)
			
	norm_pix_ = cv2.norm(coor_pix_, center_image_, cv2.NORM_L2)
	unit_dir = dir_ / norm_pix_
	
	# direction of coor_pix_ to point b in line
	norm_fix_ = 70
	slope_ = b - a
	norm_p = cv2.norm(b, a, cv2.NORM_L2)
	if norm_p == 0:
		norm_p = 0.001
	unit_line_dir = slope_ / norm_p
	coor_line_dir = unit_line_dir* norm_fix_ + coor_pix_

	# sum vector
	slope_sum = coor_line_dir - center_image_
	norm_sum = cv2.norm(center_image_, coor_line_dir, cv2.NORM_L2)
	if norm_sum == 0:
		norm_sum = 0.00001
	unit_sum = slope_sum / norm_sum
	sum_vect = unit_sum*norm_sum + center_image_
	
	# world units
	focal_lenght_ = np.array([cam_matrix_[0,0], cam_matrix_[1,1]])
	wu_coor_ = coor_pix_*height_/focal_lenght_
	wu_ci_ = center_image_*height_/focal_lenght_
	norm_wu = cv2.norm(wu_coor_, wu_ci_, cv2.NORM_L2)

	# inverse the y axis because of the image
	unit_sum[1] = unit_sum[1]		
	unit_dir[1] = -unit_dir[1]
	unit_line_dir[1] = -unit_line_dir[1]
	return coor_pix_.astype(int), coor_line_dir.astype(int), norm_wu, unit_dir, unit_line_dir, sum_vect.astype(int), unit_sum

def org_hsv2cvhsc(low_color_, up_color_):
	##	 For HSV, Hue range is [0,179], Saturation range is [0,255]
	##	 and Value range is [0,255]. Different software use different scales.
	##	 So if you are comparing
	##	 OpenCV values with them, you need to normalize these ranges.
	low_color_ = np.array(low_color_)
	up_color_ = np.array(up_color_)
	hsv_orig = np.array([360, 100, 100])
	hsv_cv = np.array([179, 255, 255])
	low_color_cv = np.divide((low_color_*hsv_cv),hsv_orig)
	up_color_cv = np.divide((up_color_*hsv_cv),hsv_orig)
	return low_color_cv.astype(int), up_color_cv.astype(int)

class LineTracking:
	def __init__(self):
		print('Line identification')
		# black
		self.low_color_hsv = [0, 0, 0]
		self.up_color_hsv = [50, 50, 50]
		# green
		#self.low_color_hsv = [76, 20, 20]
		#self.up_color_hsv = [180, 90, 90]

		self.low_color, self.up_color = org_hsv2cvhsc(self.low_color_hsv, self.up_color_hsv)
		self.rad = 10
		# line inialization
		self.line = None

				# path for camera matrix parameters
		self.file_cam_matrix = 'Camera_Matrix_v640x480.txt'
		# path for camera distortion parameters
		self.file_cam_distortion = 'Camera_Distortion_v640x480.txt'
		# load text files as float
		self.cam_matrix = np.array([[1.0e+00,0.0e+00,0.0e+00],[0.0e+00,1.0e+00,0.0e+00],[0.0e+00,0.0e+00,1.0e+00]])
		self.cam_disto = np.array([0,0,0,0])


		# Create figures
		self.img_frame = 'Frame'
		# cv2.namedWindow(self.img_frame)
		# cv2.moveWindow(self.img_frame, 30, 30)
		self.img_edges = 'Edges'
		# cv2.namedWindow(self.img_edges)
		# cv2.moveWindow(self.img_edges, 680,30)
		# create text
		self.corner_text = (20, 20)
		self.corner_text_height = (20, 50)
		self.corner_text_accel = (20, 80)
		self.font = cv2.FONT_HERSHEY_SIMPLEX
		self.font_scale = 0.5
		self.font_color = (255, 255, 255)
		self.line_type = 2

 # capture frames from the camera
	def update(self, orig_frame, pose, height):
		# image treatmen
		self.orig_frame, self.center_image = cam_calib(orig_frame, self.cam_matrix, self.cam_disto)
		self.frame = cv2.GaussianBlur(self.orig_frame, (5, 5), 0)
		self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
		self.lower = np.array(self.low_color)
		self.upper = np.array(self.up_color)
		self.mask = cv2.inRange(self.hsv, self.lower, self.upper)
		self.edges = cv2.Canny(self.mask, 50, 150)
		self.thershold = 50
		self.lines = cv2.HoughLinesP(self.edges, 1, np.pi / 180, self.thershold, minLineLength=5, maxLineGap=30)			
		self.camera_pos = center_camera(pose, height, self.center_image)
		self.max_norms = np.array([0, 0, 0, 0])
		if self.lines is not None:
			if self.lines.shape == (4,):
				self.line == self.lines
			else:
				for l in self.lines:
					self.a = np.array([l[0,0], l[0,1]])
					self.b = np.array([l[0,2], l[0,3]])
					self.c = np.array([self.max_norms[0], self.max_norms[1]])
					self.d = np.array([self.max_norms[2], self.max_norms[3]])
					self.n1 = cv2.norm(self.a, self.b, cv2.NORM_L2)
					self.n2 = cv2.norm(self.c, self.d, cv2.NORM_L2)
					if self.n1 > self.n2:
						self.max_norms = np.concatenate((self.a, self.b), 0)
					else:
						self.max_norms = np.concatenate((self.c, self.d), 0)
				self.line = self.max_norms.astype(float)
		else:
				if self.line == None:
					self.lim = np.array([30, 30, -30, 30])
					self.line = (np.concatenate((self.camera_pos, self.camera_pos), 0) + self.lim).astype(float)
		self.coor, self.direction, self.norm_wu, self.unit_dir, self.unit_line_dir, self.sum_vect, self.unit_sum = dis(self.line, self.camera_pos, self.cam_matrix, height)
		self.x1, self.y1, self.x2, self.y2 = self.line.astype(int)

		
		# Lines for vizualitation
		self.cx, self.cy = self.camera_pos.astype(int)
		# Camera center
		cv2.circle(self.frame, (self.cx, self.cy), self.rad, (0, 0, 225), -1)
		# Lines for visualisation
		cv2.line(self.frame, (self.x1, self.y1), (self.x2, self.y2), (0, 255, 0), 5)
		cv2.line(self.frame, (self.cx, self.cy), (self.coor[0], self.coor[1]), (255, 0, 0))
		cv2.line(self.frame, (self.coor[0], self.coor[1]), (self.direction[0], self.direction[1]), (255, 0, 0))
		cv2.line(self.frame, (self.cx, self.cy), (self.sum_vect[0], self.sum_vect[1]), (255, 0, 0))

		 # show the frame
		cv2.putText(self.edges, 'Norm: '+ str(self.norm_wu), self.corner_text, self.font, self.font_scale, self.font_color, self.line_type)
		cv2.putText(self.edges, 'Height: '+ str(height/1000), self.corner_text_height, self.font, self.font_scale, self.font_color, self.line_type)
		self.str_accel = str(pose[0]) + ' ' + str(pose[1]) + ' ' + str(pose[2])
		cv2.putText(self.edges, 'Pose: '+ self.str_accel, self.corner_text_accel, self.font, self.font_scale, self.font_color, self.line_type)

		
		return self.unit_dir, self.unit_line_dir, self.unit_sum, self.frame, self.edges