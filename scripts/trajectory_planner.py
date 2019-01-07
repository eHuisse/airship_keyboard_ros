#!/usr/bin/env python
import rospy
import socket
import sys
import copy
import numpy as np
import cv2

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from airship_keyboard.srv import TrigPointTracking
from airship_keyboard.srv import SetRate

MAX_RATE = 60

class Core(object):
	'''
	The to method call_* are used to subscribe to morse topic (the data coming from sensors emulation
	in morse.) They are useless for real system.
	'''
	def call_range(self, msg):
		'''
		This function is called when a range is publish by hardcom
		:param msg: message sended by command publisher
		'''
		self.range = msg

	def call_imu(self, msg):
		'''
		This function is called when a imu is publish by harcom
		:param msg: message sanded by imu publisher
		'''
		self.imu = msg

	def call_sleep(self, msg):
		'''
		This function is called when a is_sleep message is received.
		:param msg: message sanded by sleep publisher
		'''
		self.is_sleep_state = msg.data

	def call_estimated_pose(self, msg):
		'''
		This function is called when an estimated pose is published by the odometry node.
		:param msg: Image message received
		'''
		self.estimated_pose = msg

	def publish_onShutdown(self):
		'''
		This method contain all the thing to publish on shutdown. It recommanded to publish 0 for 
		everything on shutdown.
		'''
		rospy.loginfo("node : " + self.topic_root + " is shutdown.")

	def line_track(self, srv):

		#########################
		#	Code here
		#########################

		#self.wish_pose = ...
		return True, "Succeed"

	def point_track(self, srv):
		self.point_x = srv.position.x
		self.point_y = srv.position.y
		self.point_z = srv.position.z
		self.quaternion_x = srv.orientation.x
		self.quaternion_y = srv.orientation.y
		self.quaternion_z = srv.orientation.z
		self.quaternion_w = srv.orientation.w

		#########################
		#	Code here
		#########################

		#self.wish_pose = ...
		return True, "Succeed"

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

	def __init__(self):
		'''
		This class is used to transmit data from morse node to a transparent node used in ros airship you will have to modify it.
		'''
		# Get the ~private namespace parameters from command line or launch file.
		init_message = rospy.get_param('~message', 'hello')
		self.rate = float(rospy.get_param('~rate', '30.0'))
		self.rate_before_sleep = self.rate
		self.topic_root = rospy.get_param('~topic', 'trejectory_planner')
		self.topic_hardcom = rospy.get_param('~topic_hardcom', 'hardcom')
		self.topic_camcom = rospy.get_param('~topic_camcom', 'cameracom')
		self.topic_command = rospy.get_param('~topic_command', 'command')
		self.topic_odometry = rospy.get_param('~topic_odometry', 'odometry')
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
		ODOMETRY FIELDS
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

		'''
		POSE FIELDS:
		# A representation of pose in free space, composed of position and orientation.

		Point position
		POINT FIELDS:
		# This contains the position of a point in free space
			float64 x
			float64 y
			float64 z

		Quaternion orientation
		QUATERNION FIELDS
			float64 x
			float64 y
			float64 z
			float64 w
		'''
		self.wish_pose = Pose()
		self.is_sleep_state = False

		#Defining the subscriber
		rospy.Subscriber(self.topic_hardcom+"/LidarRange", Range, self.call_range)
		rospy.Subscriber(self.topic_hardcom+"/InertialData", Imu, self.call_imu)
		rospy.Subscriber(self.topic_command+"/isSleep", Bool, self.call_sleep)
		rospy.Subscriber(self.topic_odometry+"/EstimatedPose", Odometry, self.call_estimated_pose)

		#Defining all the publisher
		self.wish_pose_pub = rospy.Publisher(self.topic_root+"/WishPose", Pose, queue_size=10)

		#Defining service to set tracking mode
		self.proceed_line_tracking = rospy.Service(self.topic_root+"/proceed_line_tracking", Trigger, self.line_track)
		self.proceed_point_tracking = rospy.Service(self.topic_root+"/proceed_point_tracking", TrigPointTracking, self.point_track)

		#Defining service to set publish rate
		self.set_rate_service = rospy.Service(self.topic_root+"/set_rate", SetRate, self.set_rate)

		while not rospy.is_shutdown():
			if not self.is_sleep_state:
				#Publish the wishpose
				self.wish_pose_pub.publish(self.wish_pose)

				#this is the rate of the publishment.
				if self.rate:
					rospy.sleep(1/self.rate)
				else:
					rospy.sleep(1.0)

		self.publish_onShutdown()		

		
if __name__ == '__main__':
	rospy.init_node("trajectory_planner")
	try:
		core = Core()
	except rospy.ROSInterruptException:
		pass