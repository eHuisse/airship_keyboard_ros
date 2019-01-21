#!/usr/bin/env python
import rospy
import socket
import sys
import copy
import numpy as np
import cv2
import roslib
roslib.load_manifest('tf')
import tf
from tf.transformations import *

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion


MAX_RATE = 60

class Core(object):
	'''
	The to method call_* are used to subscribe to morse topic (the data coming from sensors emulation
	in morse.) They are useless for real system.
	'''
	def call_sleep(self, msg):
		'''
		This function is called when a is_sleep message is received.
		:param msg: message sanded by sleep publisher
		'''

		self.is_sleep_state = msg.data

	def call_imu(self, msg):
		'''
		This function is called when a is_sleep message is received.
		:param msg: message sanded by sleep publisher
		'''

		self.imu_data = msg

	def call_range(self, msg):
		'''
		This function is called when a is_sleep message is received.
		:param msg: message sanded by sleep publisher
		'''
		self.lidar_range = msg

	def call_pose(self, msg):
		'''
		TOREMOVE BEFORE USING IN REAL
		This function is called when a pose is published by the morse publishing module.
		:param msg: PoseWithCovarianceStamped message received
		'''
		self.in_pose = msg


	def qv_mult(self, q1, v1):

		# we want to rotate the point using the x-axis ("roll")
		rot = [q1.x, q1.y, q1.z, q1.w]
		# the object is located in the y=1. We use the format [x,y,z,w] and w is allways 0 for vectors
		vec = v1 
		 
		#now we apply the mathematical operation res=q*v*q'. We use this function for multiplication but internally it is just a complex multiplication operation.
		return quaternion_multiply(quaternion_multiply(rot, vec),quaternion_conjugate(rot))

		'''
			now = self.in_pose.header.stamp.secs + self.in_pose.header.stamp.nsecs * 10e-9 
			before = self.prev_in_pose.header.stamp.secs + self.prev_in_pose.header.stamp.nsecs * 10e-9
			deltaT = now - before

			x = msg.pose.pose.position.x
			y = msg.pose.pose.position.y
			z = msg.pose.pose.position.z

			prev_x = self.prev_in_pose.pose.pose.position.x
			prev_y = self.prev_in_pose.pose.pose.position.y
			prev_z = self.prev_in_pose.pose.pose.position.z

			i = msg.pose.pose.orientation.x
			j = msg.pose.pose.orientation.y
			k = msg.pose.pose.orientation.z
			w = msg.pose.pose.orientation.w

			prev_i =self.prev_in_pose.pose.pose.orientation.x
			prev_j =self.prev_in_pose.pose.pose.orientation.y
			prev_k =self.prev_in_pose.pose.pose.orientation.z
			prev_w =self.prev_in_pose.pose.pose.orientation.w

			self.out_odom.pose = msg.pose
			try:
				self.out_odom.twist.twist.linear = Vector3((x - prev_x) / deltaT,(y - prev_y) / deltaT,(z - prev_z) / deltaT)
			except ZeroDivisionError:
				self.out_odom.twist.twist.linear = Vector3(0, 0, 0)

			(prev_r, prev_p, prev_y) = tf.transformations.euler_from_quaternion([self.prev_in_pose.pose.pose.orientation.x, self.prev_in_pose.pose.pose.orientation.y, 
																	self.prev_in_pose.pose.pose.orientation.z, self.prev_in_pose.pose.pose.orientation.w])
			(r, p, y) = tf.transformations.euler_from_quaternion([self.in_pose.pose.pose.orientation.x, self.in_pose.pose.pose.orientation.y, 
																	self.in_pose.pose.pose.orientation.z, self.in_pose.pose.pose.orientation.w])

			self.out_odom.twist.covariance = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]

			try:
			    self.out_odom.twist.twist.angular = Vector3((r - prev_r) / deltaT,(p - prev_p) / deltaT,(y - prev_y) / deltaT)

			except ZeroDivisionError:
				self.out_odom.twist.twist.linear = Vector3(0, 0, 0)
			'''


			
		self.prev_in_pose = copy.deepcopy(msg)




	def publish_onShutdown(self):
		'''
		This method contain all the thing to publish on shutdown. It recommanded to publish 0 for 
		everything on shutdown.
		'''
		rospy.loginfo("node : " + self.topic_root + " is shutdown.")

	def twist_from_pose(self):
			now = self.out_odom.header.stamp.secs + self.out_odom.header.stamp.nsecs * 10e-9 
			before = self.prev_out_odom.header.stamp.secs + self.prev_out_odom.header.stamp.nsecs * 10e-9
			deltaT = now - before

			x = self.out_odom.pose.pose.position.x
			y = self.out_odom.pose.pose.position.y
			z = self.out_odom.pose.pose.position.z

			prev_x = self.prev_out_odom.pose.pose.position.x
			prev_y = self.prev_out_odom.pose.pose.position.y
			prev_z = self.prev_out_odom.pose.pose.position.z

			i = self.out_odom.pose.pose.orientation.x
			j = self.out_odom.pose.pose.orientation.y
			k = self.out_odom.pose.pose.orientation.z
			w = self.out_odom.pose.pose.orientation.w

			prev_i =self.prev_out_odom.pose.pose.orientation.x
			prev_j =self.prev_out_odom.pose.pose.orientation.y
			prev_k =self.prev_out_odom.pose.pose.orientation.z
			prev_w =self.prev_out_odom.pose.pose.orientation.w

			try:
				self.out_odom.twist.twist.linear = Vector3((x - prev_x) / deltaT,(y - prev_y) / deltaT,(z - prev_z) / deltaT)
			except ZeroDivisionError:
				self.out_odom.twist.twist.linear = Vector3(0, 0, 0)

			(prev_r, prev_p, prev_y) = tf.transformations.euler_from_quaternion([self.prev_out_odom.pose.pose.orientation.x, self.prev_out_odom.pose.pose.orientation.y, 
																	self.prev_out_odom.pose.pose.orientation.z, self.prev_out_odom.pose.pose.orientation.w])
			(r, p, y) = tf.transformations.euler_from_quaternion([self.out_odom.pose.pose.orientation.x, self.out_odom.pose.pose.orientation.y, 
																	self.out_odom.pose.pose.orientation.z, self.out_odom.pose.pose.orientation.w])

			self.out_odom.twist.covariance = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]

			try:
			    self.out_odom.twist.twist.angular = Vector3((r - prev_r) / deltaT,(p - prev_p) / deltaT,(y - prev_y) / deltaT)

			except ZeroDivisionError:
				self.out_odom.twist.twist.linear = Vector3(0, 0, 0)

	def calibrate_odom(self, srv):
		#Rescaling we increment mean_scale
		increment = (self.real_range - self.init_lidar_z)/(self.in_pose.pose.pose.position.z + 1e-6)
		if np.isnan(increment):
			pass
			return False, ""
		else:
			self.mean_scale = abs(increment)
			rospy.loginfo("mean : " + str(self.mean_scale))
			rospy.loginfo("NOM : " + str((self.in_pose.pose.pose.position.z)))
			rospy.loginfo("DENOM : " + str((self.real_range - self.init_lidar_z)))
			return True, ""






	def __init__(self):
		'''
		This class is used to transmit data from morse node to a transparent node used in ros airship you will have to modify it.
		'''
		# Get the ~private namespace parameters from command line or launch file.
		init_message = rospy.get_param('~message', 'hello')
		self.rate = float(rospy.get_param('~rate', '30.0'))
		self.rate_before_sleep = self.rate
		self.topic_root = rospy.get_param('~topic', 'pose2odom')
		self.topic_svo = rospy.get_param('~topic_svo', 'svo')
		self.topic_command = rospy.get_param('~topic_command', 'command')
		self.topic_hardcom = rospy.get_param('~topic_hardcom', 'hardcom')
		self.topic_imu = rospy.get_param('~topic_imu', 'imu')
		rospy.loginfo("airship/" + self.topic_root + "/rate = " + str(self.rate) + "Hz")

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
		self.out_odom = Odometry()
		self.prev_out_odom = Odometry()
		self.lidar_odom = Odometry()
		self.lidar_odom.pose.covariance[35] = 0.1

		'''
		Header header           # timestamp in the header is the time the ranger
		                        # returned the distance reading

		# Radiation type enums
		# If you want a value added to this list, send an email to the ros-users list
		uint8 ULTRASOUND=0
		uint8 INFRARED=1

		uint8 radiation_type

		float32 field_of_view

		float32 min_range       
		float32 max_range       
		                        

		float32 range           # range data [m]
				self.lidar_range = Range()
		'''

		self.lidar_range = Range()
		'''
		[geometry_msgs/PoseWithCovarianceStamped]:
		std_msgs/Header header
		  uint32 seq
		  time stamp
		  string frame_id
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

		'''
		self.imu_data = Imu()
		self.in_pose = PoseWithCovarianceStamped()
	

		self.is_sleep_state = False
		self.before = rospy.get_rostime()
		self.n = 0
		self.real_range = 0

		self.mean_scale = 1


		#Defining the subscriber
		rospy.Subscriber(self.topic_svo+"/pose", PoseWithCovarianceStamped, self.call_pose)
		rospy.Subscriber(self.topic_command+"/isSleep", Bool, self.call_sleep)
		rospy.Subscriber(self.topic_hardcom+"/LidarRange", Range, self.call_range)
		rospy.Subscriber(self.topic_imu+"/datacooked", Imu, self.call_imu)

		#rospy.Subscriber("raw/pose", PoseStamped, self.call_pose)

		#Defining all the publisher
		self.estimated_pose_pub = rospy.Publisher(self.topic_root+"/EstimatedPose", Odometry, queue_size=10)
		self.lidar_pose_pub = rospy.Publisher(self.topic_root+"/LidarPose", Odometry, queue_size=10)

		self.calibrate_odometry_service = rospy.Service(self.topic_root+"/calibrate_odom", Trigger, self.calibrate_odom)

		self.lidar_odom.pose.covariance = [99999,0,0,0,0,0, 0,99999,0,0,0,0, 0,0,0.03,0,0,0, 0,0,0,99999,0,0, 0,0,0,0,99999,0, 0,0,0,0,0,99999]
		self.lidar_odom.twist.covariance = [99999,0,0,0,0,0, 0,99999,0,0,0,0, 0,0,0.03,0,0,0, 0,0,0,99999,0,0, 0,0,0,0,99999,0, 0,0,0,0,0,99999]
		#self.lidar_odom.header.frame_id = "/cam"
		#self.lidar_odom.child_frame_id = "/cam"

		while not rospy.is_shutdown():
			if self.n < 10:
				self.init_odom_z = self.in_pose.pose.pose.position.z
				self.init_lidar_z = self.real_range
				self.n = self.n + 1
			#######################################
			# This is where we can compute the estimated
			# pose
			#####################################
			self.out_odom.header = self.in_pose.header
			self.out_odom.pose = self.in_pose.pose
			#self.out_odom.header.frame_id = ""
			#self.out_odom.child_frame_id = ""


			lidarRange = [0., 0., self.lidar_range.range, 0.]

			self.real_range = abs(self.qv_mult(self.imu_data.orientation, lidarRange))[2]

			self.out_odom.pose.pose.position.x = self.in_pose.pose.pose.position.x * self.mean_scale
			self.out_odom.pose.pose.position.y = self.in_pose.pose.pose.position.y * self.mean_scale
			self.out_odom.pose.pose.position.z = self.real_range

			self.twist_from_pose()
			#self.lidar_odom.header.stamp = rospy.get_rostime()
			#self.out_odom.header.stamp = rospy.get_rostime()
			self.estimated_pose_pub.publish(self.out_odom)
			#self.lidar_pose_pub.publish(self.lidar_odom)


			#this is the rate of the publishment.
			if self.rate:
				rospy.sleep(1/self.rate)
			else:
				rospy.sleep(1.0)

			self.prev_out_odom = copy.deepcopy(self.out_odom)

		self.publish_onShutdown()		

		
if __name__ == '__main__':
	rospy.init_node("pose2odom")
	try:
		core = Core()
	except rospy.ROSInterruptException:
		pass
