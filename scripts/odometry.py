#!/usr/bin/env python
import rospy
import socket
import sys
import copy
import numpy as np
import cv2

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu

MAX_RATE = 60

class Core(object):
	'''
	The to method call_* are used to subscribe to morse topic (the data coming from sensors emulation
	in morse.) They are useless for real system.
	'''
	def call_range(self, msg):
		'''
		This function is called when a motor_command message is received from command nide. This function have to be
		replaced by the I2C com
		:param msg: message sended by command publisher
		'''
		self.range = msg

	def call_camera_info(self, msg):
		'''
		This function is called when a motor_command message is received from command nide. This function have to be
		replaced by the I2C com
		:param msg: message sended by command publisher
		'''
		self.camera_info = msg

	def call_imu(self, msg):
		'''
		This function is called when a imu message is received. This function have to be
		replaced by the I2C com
		:param msg: message sanded by imu publisher
		'''
		self.imu = msg

	def call_sleep(self, msg):
		'''
		This function is called when a is_sleep message is received.
		:param msg: message sanded by sleep publisher
		'''
		self.is_sleep_state = msg.data


	def call_image(self, msg):
		'''
		This function is called when an image is published by the camera publishing module.
		:param msg: Image message received
		'''
		self.image = msg

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
		self.topic_root = rospy.get_param('~topic', 'odometry')
		self.topic_hardcom = rospy.get_param('~topic_hardcom', 'hardcom')
		self.topic_camcom = rospy.get_param('~topic_camcom', 'cameracom')
		self.topic_command = rospy.get_param('~topic_command', 'command')
		rospy.loginfo("airship/" + self.topic_root + "/rate = " + str(self.rate) + "Hz")

		#Defining published messages
		'''
		RANGE FIELDS:
		Header header
		uint8 ULTRASOUND=0
		uint8 INFRARED=1
		uint8 radiation_type
		float32 field_of_view
		float32 min_range
		float32 max_range
		float32 range
		'''
		self.range = Range()

		'''
		IMU FIELDS:
		Header header

		geometry_msgs/Quaternion orientation
		float64[9] orientation_covariance # Row major about x, y, z axes

		geometry_msgs/Vector3 angular_velocity
		float64[9] angular_velocity_covariance # Row major about x, y, z axes

		geometry_msgs/Vector3 linear_acceleration
		float64[9] linear_acceleration_covariance # Row major x, y z 
		'''
		self.imu = Imu()

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
		self.image = Image()

		self.camera_info = CameraInfo()

		'''
		ODOMETRY FIELDS:
		std_msgs/Header header
		  uint32 seq
		  time stamp
		  string frame_id
		string child_frame_id
		geometry_msgs/PoseWithCovariance pose
		  geometry_msgs/Pose pose
		    geometry_msgs/Point position
		      float64 x
		      float64 y
		      float64 z
		    geometry_msgs/Quaternion orientation
		      float64 x
		      float64 y
		      float64 z
		      float64 w
		  float64[36] covariance
		geometry_msgs/TwistWithCovariance twist
		  geometry_msgs/Twist twist
		    geometry_msgs/Vector3 linear
		      float64 x
		      float64 y
		      float64 z
		    geometry_msgs/Vector3 angular
		      float64 x
		      float64 y
		      float64 z
		  float64[36] covariance

		'''
		self.estimated_pose = Odometry()
		self.is_sleep_state = False

		#Defining the subscriber
		rospy.Subscriber(self.topic_hardcom+"/LidarRange", Range, self.call_range)
		rospy.Subscriber(self.topic_hardcom+"/InertialData", Imu, self.call_imu)
		rospy.Subscriber(self.topic_camcom+"/CameraImg", Image, self.call_image)
		rospy.Subscriber(self.topic_camcom+"/CameraInfo", CameraInfo, self.call_camera_info)
		rospy.Subscriber(self.topic_command+"/isSleep", Bool, self.call_sleep)

		#Defining all the publisher
		self.estimated_pose_pub = rospy.Publisher(self.topic_root+"/EstimatedPose", Odometry, queue_size=10)

		while not rospy.is_shutdown():
			if not self.is_sleep_state:
				#######################################
				# This is where we can compute the estimated
				# pose
				#######################################

				self.estimated_pose_pub.publish(self.estimated_pose)

				#this is the rate of the publishment.
				if self.rate:
					rospy.sleep(1/self.rate)
				else:
					rospy.sleep(1.0)

		self.publish_onShutdown()		

		
if __name__ == '__main__':
	rospy.init_node("odometry")
	try:
		core = Core()
	except rospy.ROSInterruptException:
		pass