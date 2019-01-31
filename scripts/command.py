#!/usr/bin/env python
import rospy
import socket
import sys
import copy
import numpy as np
import cv2
import pid
import tf

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
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

	def call_wish_command(self, msg):
		'''
		This function is called when an estimated pose is published by the odometry node.
		:param msg: Image message received
		'''
		self.wish_command = msg

		self.speed_pid.setSetPoint(msg.x)
		self.elevation_pid.setSetPoint(msg.y)
		self.rudder_pid.setSetPoint(msg.z)

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

		self.speed_pid.setKp(self.gain[0].Kp)
		self.speed_pid.setKd(self.gain[0].Kd)
		self.speed_pid.setKi(self.gain[0].Ki)

		self.gain[1].Kp = srv.gain2.Kp
		self.gain[1].Kd = srv.gain2.Kd
		self.gain[1].Ki = srv.gain2.Ki

		self.elevation_pid.setKp(self.gain[1].Kp)
		self.elevation_pid.setKd(self.gain[1].Kd)
		self.elevation_pid.setKi(self.gain[1].Ki)

		self.gain[2].Kp = srv.gain3.Kp
		self.gain[2].Kd = srv.gain3.Kd
		self.gain[2].Ki = srv.gain3.Ki

		self.rudder_pid.setKp(self.gain[2].Kp)
		self.rudder_pid.setKd(self.gain[2].Kd)
		self.rudder_pid.setKi(self.gain[2].Ki)

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
		Vector3 FIELDS:

			float64 x
			float64 y
			float64 z

		'''
		self.wish_command = Vector3()

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
		self.gain[0].Kp = 1.
		self.gain[0].Kd = 0.
		self.gain[0].Ki = 0.

		self.gain[1].Kp = 1.
		self.gain[1].Kd = 0.
		self.gain[1].Ki = 0.

		self.gain[2].Kp = 1.
		self.gain[2].Kd = 0.
		self.gain[2].Ki = 0.

		self.pitch = 0
		self.yaw = 0
		self.thrust = 0

		self.speed_pid = pid.PID(rospy.get_time(), P = 0.3, I = 0., D=0.)
		self.elevation_pid = pid.PID(rospy.get_time(), P = 0.2, I = 0., D=0.1)
		self.rudder_pid = pid.PID(rospy.get_time(), P = 10, I = 1., D=0.1)

		#Defining the subscriber
		rospy.Subscriber("pose2odom/EstimatedPose", Odometry, self.call_estimated_pose)
		rospy.Subscriber(self.topic_trajectory_planner+"/WishCommand", Vector3, self.call_wish_command)
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

			(r, p, y) = tf.transformations.euler_from_quaternion([self.estimated_pose.pose.pose.orientation.x, self.estimated_pose.pose.pose.orientation.y, 
																	self.estimated_pose.pose.pose.orientation.z, self.estimated_pose.pose.pose.orientation.w])

			estimated_total_speed = np.sqrt(self.estimated_pose.twist.twist.linear.x ** 2 + self.estimated_pose.twist.twist.linear.y ** 2 + self.estimated_pose.twist.twist.linear.z ** 2)

			speed_command = self.speed_pid.update(estimated_total_speed, rospy.get_time())
			elevation_command = self.elevation_pid.update(p, rospy.get_time())
			rudder_command = self.rudder_pid.update(y, rospy.get_time())
			print("sssssssssssssssspeeeeeeeeeeeeeddddddddddddddddddd", r,",",p,",",y)

			self.motor_command.header.stamp = rospy.Time.now()

			if speed_command > 1:
				speed_command = 1
			if speed_command < -1:
				speed_command = -1
			if elevation_command > np.pi/4:
				elevation_command = np.pi/4
			if elevation_command < -np.pi/4:
				elevation_command = -np.pi/4
			if rudder_command > np.pi/4:
				rudder_command = np.pi/4
			if rudder_command < -np.pi/4:
				rudder_command = -np.pi/4


			self.motor_command.left_motor = speed_command / 2
			self.motor_command.right_motor = speed_command / 2
			self.motor_command.tail_yaw = elevation_command
			self.motor_command.tail_pitch = rudder_command

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
