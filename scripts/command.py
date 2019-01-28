#!/usr/bin/env python
import rospy
import socket
import sys
import copy
import numpy as np
import cv2
import pid as PID

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from airship_keyboard.srv import SetSleep
from airship_keyboard.srv import SetRate
from airship_keyboard.srv import SetGain
from airship_keyboard.msg import MotorCommand
from airship_keyboard.msg import Gain
from keyboard.msg import Key

MAX_RATE = 60

class Core(object):
	'''
	The to method call_* are used to subscribe to morse topic (the data coming from sensors emulation
	in morse.) They are useless for real system.
	'''

	def call_estimated_pose(self, msg):
		'''
		This function is called when an estimated pose is published by the odometry node.
		:param msg: Image message received
		'''
		self.estimated_pose = msg

	def call_wish_pose(self, msg):
		'''
		This function is called when an estimated pose is published by the odometry node.
		:param msg: Image message received
		'''
		self.wish_pose = msg

	def call_keydown(self, msg):
		'''
		This function is called when an estimated pose is published by the odometry node.
		:param msg: Image message received
		'''
		self.keydown = msg
		if msg.code == 122:
			self.thrust += 0.1
		elif msg.code == 115:
			self.thrust -= 0.1
		elif msg.code == 100:
			self.pitch -= 0.1
		elif msg.code == 113:
			self.pitch += 0.1
		elif msg.code == 116:
			self.yaw += 0.1
		elif msg.code == 103:
			self.yaw -= 0.1
		rospy.loginfo("test")


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

	def set_gain(self, srv):
		'''
		airship_keyboard/Gain gain1
		  float32 Kp
		  float32 Kd
		  float32 Ki
		airship_keyboard/Gain gain2
		  float32 Kp
		  float32 Kd
		  float32 Ki
		airship_keyboard/Gain gain3
		  float32 Kp
		  float32 Kd
		  float32 Ki
		---
		bool success
		'''
		self.gain[0].Kp = srv.gain1.Kp
		self.gain[0].Kd = srv.gain1.Kd
		self.gain[0].Ki = srv.gain1.Ki

		self.gain[1].Kp = srv.gain2.Kp
		self.gain[1].Kd = srv.gain2.Kd
		self.gain[1].Ki = srv.gain2.Ki

		self.gain[2].Kp = srv.gain3.Kp
		self.gain[2].Kd = srv.gain3.Kd
		self.gain[2].Ki = srv.gain3.Ki

	def set_sleep(self, srv):
		self.sleep.data = srv.set_sleep
		return True

	def publish_onShutdown(self):
		'''
		This method contain all the thing to publish on shutdown. It recommanded to publish 0 for 
		everything on shutdown.
		'''
		self.motor_command.header.stamp = rospy.Time.now()
		self.motor_command.motor_gimbal_angle = 0
		self.motor_command.left_motor = 0
		self.motor_command.right_motor = 0
		self.motor_command.tail_yaw = 0
		self.motor_command.tail_pitch = 0

		self.motor_command_pub.publish(self.motor_command)

		rospy.loginfo("node : " + self.topic_root + " is shutdown.")

	def __init__(self):
		'''
		This class is used to transmit data from morse node to a transparent node used in ros airship you will have to modify it.
		'''
		# Get the ~private namespace parameters from command line or launch file.
		init_message = rospy.get_param('~message', 'hello')
		self.rate = float(rospy.get_param('~rate', '30.0'))
		self.rate_before_sleep = self.rate
		self.topic_root = rospy.get_param('~topic', 'command')
		self.topic_odometry = rospy.get_param('~topic_odometry', 'odometry')
		self.topic_trajectory_planner = rospy.get_param('~topic_trajectory_planner', 'trajectory_planner')
		self.topic_hardcom = rospy.get_param('~topic_hardcom', 'hardcom')

		rospy.loginfo("airship/" + self.topic_root + "/rate = " + str(self.rate) + "Hz")

		#Defining published messages
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

		'''
		MOTORCOMMAND FIELDs:
		Header header            	# timestamp in the header is the acquisition time of command

		float32 motor_gimbal_angle	# The gimbal of motor is mooving (rad)

		float32 left_motor			# Left motor thrust (N)
		float32 right_motor			# Right motor thrust (N)

		float32 tail_yaw			# Tail_yaw angle (rad)
		float32 tail_pitch			# Tail_pitch angle (rad)
		'''
		self.motor_command = MotorCommand()

		'''
		GAIN FIELDS:
		airship_keyboard/Gain gain
  			float32 Kp
  			float32 Kd
  			float32 Ki
		'''

		self.sleep = Bool()
		self.sleep.data = False

		self.keydown = Key()

		self.gain = [Gain(), Gain(), Gain()]
		self.gain[0].Kp = 1
		self.gain[0].Kd = 0.1
		self.gain[0].Ki = 1

		self.pitch = 0
		self.yaw = 0
		self.thrust = 0

		#Defining the subscriber
		rospy.Subscriber("pose2odom/EstimatedPose", Odometry, self.call_estimated_pose)
		rospy.Subscriber(self.topic_trajectory_planner+"/WishPose", Pose, self.call_wish_pose)
		rospy.Subscriber("keyboard/keydown", Key, self.call_keydown)

		#Defining all the publisher
		self.motor_command_pub = rospy.Publisher(self.topic_root+"/MotorCommand", MotorCommand, queue_size=10)
		self.is_sleep_pub = rospy.Publisher(self.topic_root+"/isSleep", Bool, queue_size=10)

		#Defining service to set publish rate
		self.set_rate_service = rospy.Service(self.topic_root+"/set_rate", SetRate, self.set_rate)
		self.set_gain_service = rospy.Service(self.topic_root+"/set_gain", SetGain, self.set_gain)
		self.set_sleep_service = rospy.Service(self.topic_root+"/set_sleep", SetSleep, self.set_sleep)

		while not rospy.is_shutdown():
			####
			# Altitude command
			####
			#altitude_pid = PID.PID()
			altitude_error = self.wish_pose.position.z - self.estimated_pose.pose.pose.position.z

			

			#######################################
			# This is where we can compute MotorCommand
			#######################################

			self.motor_command.header.stamp = rospy.Time.now()

			self.motor_command.left_motor = self.thrust / 2
			self.motor_command.right_motor = self.thrust / 2
			self.motor_command.tail_yaw = self.yaw
			self.motor_command.tail_pitch = self.pitch 

			self.motor_command_pub.publish(self.motor_command)
			self.is_sleep_pub.publish(self.sleep)

			#this is the rate of the publishment.
			if self.rate:
				rospy.sleep(1/self.rate)
			else:
				rospy.sleep(1.0)

		self.publish_onShutdown()		

		
if __name__ == '__main__':
	rospy.init_node("command")
	try:
		core = Core()
	except rospy.ROSInterruptException:
		pass
