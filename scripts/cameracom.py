#!/usr/bin/env python
import rospy
import socket
import sys
import copy
import numpy as np
import cv2

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from airship_keyboard.srv import SetRate
from airship_keyboard.srv import SetSize
from sensor_msgs.msg import CameraInfo

MAX_RATE = 30

class Core(object):
	'''
	The to method call_* are used to subscribe to morse topic (the data coming from sensors emulation
	in morse.) They are useless for real system.
	'''
	#
	# BEGIN TO REMOVE FOR NOSIM SYSTEM 
	#
	def call_image(self, msg):
		'''
		This function is called when an image arrived from ros to change by RPi camera 
		:param msg: message sended by command publisher
		'''
		self.pubImage = msg
	#
	# END TO REMOVE FOR NOSIM SYSTEM 
	#

	def call_sleep(self, msg):
		'''
		This function is called when a is_sleep message is received.
		:param msg: message sanded by sleep publisher
		'''
		self.is_sleep_state = msg.data

	def set_rate(self, srv):
		'''
		This function allow us to change publish rate of our node
		:param srv: contain the new rate sended
		'''
		#To be modify if not simulation
		if srv.rate > MAX_RATE:
			self.rate = MAX_RATE
		else:
			self.rate = srv.rate

		rospy.loginfo("airship/" + self.topic_root + "/rate = " + str(srv.rate) + "Hz")
		return True

	def set_size(self, srv):
		self.img_width = srv.size[0]
		self.img_height = ser.size[1]
		#self.cam_infos.K = ...


	def publish_onShutdown(self):
		'''
		This method contain all the thing to publish on shutdown. It recommanded to publish 0 for 
		everything on shutdown.
		'''
		rospy.loginfo("node : " + self.topic_root + " is shutdown.")

	def __init__(self):
		'''
		This class is used to transmit data from morse node to a transparent node used in ros airship you will have to modify it.
		'''
		# Get the ~private namespace parameters from command line or launch file.
		init_message = rospy.get_param('~message', 'hello')
		self.rate = float(rospy.get_param('~rate', '30.0'))
		self.rate_before_sleep = self.rate
		self.topic_root = rospy.get_param('~topic', 'hardcom')
		self.topic_command = rospy.get_param('~topic_command', 'command')
		rospy.loginfo("airship/" + self.topic_root + "/rate = " + str(self.rate) + "Hz")

		#Defining size of output image for croping (TODO)
		self.img_width = 1280
		self.img_height = 720


		'''
		http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html

		Header header
		uint32 height
		uint32 width
		string distortion_model
		float64[9]  K
			# Intrinsic camera matrix for the raw (distorted) images.
			#     [fx  0 cx]
			# K = [ 0 fy cy]
			#     [ 0  0  1]
		float64[9]  R #Rectification matrix (stereo cameras only)
		float64[12] P
			# Projection/camera matrix
			#     [fx'  0  cx' Tx]
			# P = [ 0  fy' cy' Ty]
			#     [ 0   0   1   0]
		uint32 binning_x
		uint32 binning_y
		RegionOfInterest roi
		'''
		#Defining published messages
		#TOFILL
		self.cam_infos = CameraInfo()
		self.cam_infos.height = self.img_height
		self.cam_infos.width = self.img_width
		self.cam_infos.D = [0.0]
		self.cam_infos.K = [2000.0, 0.0, 640.0, 0.0, 2000.0, 360.0, 0.0, 0.0, 1.0]
		self.cam_infos.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		self.cam_infos.P = [2000.0, 0.0, 640.0, 0.0, 0.0, 2000.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0]
		self.cam_infos.binning_x = 0.
		self.cam_infos.binning_y = 0.

		'''
		IMAGE FIELDS
		Header header        

		uint32 height         # image height, that is, number of rows
		uint32 width          # image width, that is, number of columns

		string encoding       # Encoding of pixels -- channel meaning, ordering, size
		                      # taken from the list of strings in include/sensor_msgs/image_encodings.h
		uint8 is_bigendian    # is this data bigendian?
		uint32 step           # Full row length in bytes
		uint8[] data          # actual matrix data, size is (step * rows)
		'''
		self.pubImage = Image()
		self.is_sleep_state = False


		#Defining the subscriber (not necessary for final node)

		#
		# BEGIN TO REMOVE FOR NOSIM SYSTEM 
		#
		rospy.Subscriber("/raw/cam/image", Image, self.call_image)
		#
		# END TO REMOVE FOR NOSIM SYSTEM 
		#

		rospy.Subscriber("airship/"+self.topic_command+"/isSleep", Bool, self.call_sleep)

		#Defining all the publisher
		self.img_pub = rospy.Publisher("airship/"+self.topic_root+"/CameraImg", Image, queue_size=10)
		self.info_pub = rospy.Publisher("airship/"+self.topic_root+"/CameraInfo", CameraInfo, queue_size=10)

		#Defining service for setting publish rate 
		self.s = rospy.Service("airship/"+self.topic_root+"/set_rate", SetRate, self.set_rate)

		while not rospy.is_shutdown():
			if not self.is_sleep_state:
				#In this loop you can put all your code to use RPicam.
				self.img_pub.publish(self.pubImage)
				self.info_pub.publish(self.cam_infos)

				#this is the rate of the publishment.
				if self.rate:
					rospy.sleep(1/self.rate)
				else:
					rospy.sleep(1.0)

		self.publish_onShutdown()		

		
if __name__ == '__main__':
	rospy.init_node("camercom")
	try:
		core = Core()
	except rospy.ROSInterruptException:
		pass