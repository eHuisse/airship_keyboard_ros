#!/usr/bin/env python
import rospy
import socket
import sys
import copy
import numpy as np

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from airship_keyboard.srv import SetRate
from airship_keyboard.srv import SetSleep
from airship_keyboard.msg import MotorCommand

MAX_RATE = 60

class Core(object):
	'''
	The to method call_* are used to subscribe to morse topic (the data coming from sensors emulation
	in morse.) They are useless for real system.
	'''

	#
	# BEGIN TO REMOVE FOR NOSIM SYSTEM 
	#
	def call_lidar(self, msg):
		'''
		This function is called when a motor_command message is received from command nide. This function have to be
		replaced by the I2C com
		:param msg: message sended by command publisher
		'''
		self.pubRange.range = min(msg.ranges)
		self._sendRange = True


	def call_motor_command(self, msg):
		'''
		This function is called when a lidar message is received. This function have to be
		replaced by the I2C com
		:param msg: message sanded by lidar publisher
		'''
		self.motor_command = msg		

	def call_imu(self, msg):
		'''
		This function is called when a imu message is received. This function have to be
		replaced by the I2C com
		:param msg: message sanded by imu publisher
		'''
		self.pubImu.header = msg.header
		self.pubImu.angular_velocity = msg.angular_velocity
		self.pubImu.angular_velocity_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
		self.pubImu.linear_acceleration = msg.linear_acceleration
		self.pubImu.linear_acceleration_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
		self._sendImu = True

	#
	# END TO REMOVE FOR NOSIM SYSTEM 
	#

	def call_sleep(self, msg):
		'''
		This function is called when a is_sleep message is received.
		:param msg: message sanded by sleep publisher
		'''
		self.is_sleep_state = msg.data

	def sleep_service_calling(self):
		'''
		This function call the command service to put asleep all node.
		'''
		rospy.wait_for_service('airship/command/set_sleep')
		sleep = SetSleep()
		sleep.set_sleep = self.set_sleep
		try:
			set_sleep = rospy.ServiceProxy('airship/command/set_sleep', SetSleep)
			resp1 = set_sleep(sleep)
			rospy.loginfo("airship/" + self.topic_root + "/set_sleep to " + str(self.set_sleep) + "->" + str(resp1))

		except rospy.ServiceException, e:
			print "Service call failed: " + str(e)


	#def call_magnetic(self, msg):

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


	def publish_onShutdown(self):
		'''
		This method contain all the thing to publish on shutdown. It recommanded to publish 0 for 
		everything on shutdown.
		'''
		############################################
		# DON'T FORGET TO SEND 0. TO ALL MOTOR
		# BEFORE SHUTTING DOWN OTHERWISE IT'S 
		# DANGEROUS
		############################################
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
		self.pubRange = Range()
		self.pubRange.field_of_view = 0.0001
		self.pubRange.min_range = 0.3
		self.pubRange.max_range = 4.0
		self.pubRange.radiation_type = 1

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
		self.pubImu = Imu()

		'''
		MAGNETIC FIELDS:
 		Header header

 		geometry_msgs/Vector3 magnetic_field # x, y, and z components of the

 		float64[9] magnetic_field_covariance
		'''
		self.pubMagnetic = MagneticField()

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


		#
		# BEGIN TO REMOVE FOR NOSIM SYSTEM 
		#

		#Defining some kind of semaphore (not necessary for final node)
		self._sendRange = False
		self._sendImu = False
		self._sendMagnetic = True
		self.is_sleep_state = False

		#This variable is activate when the user take control
		self.set_sleep = False
		self.prev_set_sleep = False

		#Defining the subscriber (not necessary for final node)
		rospy.Subscriber("/raw/lidar", LaserScan, self.call_lidar)
		rospy.Subscriber("/raw/imu", Imu, self.call_imu)
		rospy.Subscriber(self.topic_command+"/isSleep", Bool, self.call_sleep)
		rospy.Subscriber(self.topic_command+"/MotorCommand", MotorCommand, self.call_motor_command)
		#rospy.Subscriber("", Vector3, self.call_magnetic)

		#
		# END TO REMOVE FOR NOSIM SYSTEM 
		#

		#Defining all the publisher
		self.range_pub = rospy.Publisher(self.topic_root+"/LidarRange", Range, queue_size=10)
		self.imu_pub = rospy.Publisher(self.topic_root+"/InertialDataRaw", Imu, queue_size=10)
		self.magnetic_pub = rospy.Publisher(self.topic_root+"/MagneticData", MagneticField, queue_size=10)

		#Defining service to set publish rate
		self.set_rate_service = rospy.Service(self.topic_root+"/set_rate", SetRate, self.set_rate)

		while not rospy.is_shutdown():
			#In this loop you can put all your code to use I2C bus.
			if not self.is_sleep_state:
				if self._sendRange and self._sendImu and self._sendMagnetic :

					####
					#### Here you can put tour I2C communication.
					####

					#This function allow us to publish the data collected on hardware
					#before, we need to convert it and package in their message contener (self.pubRange etc...)
					self.range_pub.publish(self.pubRange)
					self.imu_pub.publish(self.pubImu)
					self.magnetic_pub.publish(self.pubMagnetic)

					#
					# BEGIN TO REMOVE FOR NOSIM SYSTEM 
					#

					#The "semaphores" are placed on False
					self._sendRange=False
					self._sendImu=False
					self._sendMagnetic=True

					#
					# END TO REMOVE FOR NOSIM SYSTEM 
					#

			#This is the set_sleep part asking to command node for broadcasting a sleep
			if self.set_sleep and not self.prev_set_sleep:
				resp = self.sleep_service_calling()
			if not self.set_sleep and self.prev_set_sleep:
				resp = self.sleep_service_calling()

			self.prev_set_sleep = self.set_sleep


				#this is the rate of publishment.
			if self.rate:
				rospy.sleep(1/self.rate)
			else:
				rospy.sleep(1.0)

		self.publish_onShutdown()		

		
if __name__ == '__main__':
	rospy.init_node("hardcom")
	try:
		core = Core()
	except rospy.ROSInterruptException:
		pass
