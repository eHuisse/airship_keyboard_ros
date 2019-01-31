#!/usr/bin/env python
import rospy
import socket
import sys
import copy
import numpy as np
import cv2
import tf
from cv_bridge import CvBridge, CvBridgeError

sys.path.insert(0, "/home/edouard/catkin_ws/src/airship_keyboard/scripts/TI")
import Camera_Live_v4 as lt

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
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

	def call_image(self, msg):

		self.image = msg

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

		self.is_line_tracking = True

		return True, "Succeed"

	def point_track(self, srv):
		self.point_x = srv.position.x
		self.point_y = srv.position.y
		self.point_z = srv.position.z

		self.is_trajectory_tracking = True

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
		self.image = Image()
		self.viz_img = Image()

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
		#self.wish_pose = Pose()

		'''
		Vector3 FIELDS:

		float64 x
		float64 y
		float64 z
		'''
		self.wish_command = Vector3() # x : speed command ; y : elevation command ; z : rudder command 

		self.is_line_tracking = False
		self.is_trajectory_tracking = False
		self.is_sleep_state = False

		self.bridge = CvBridge()

		#Defining the subscriber
		rospy.Subscriber(self.topic_hardcom+"/LidarRange", Range, self.call_range)
		rospy.Subscriber(self.topic_hardcom+"/InertialData", Imu, self.call_imu)
		rospy.Subscriber(self.topic_command+"/isSleep", Bool, self.call_sleep)
		rospy.Subscriber("cameracom/CameraImg", Image, self.call_image)
		rospy.Subscriber("pose2odom/EstimatedPose", Odometry, self.call_estimated_pose)

		#Defining all the publisher
		self.wish_command_pub = rospy.Publisher(self.topic_root+"/WishCommand", Vector3, queue_size=10)
		self.vizualisation_pub = rospy.Publisher("planner/linetrack_viz", Image, queue_size=10)

		#Defining service to set tracking mode
		self.proceed_line_tracking = rospy.Service(self.topic_root+"/proceed_line_tracking", Trigger, self.line_track)
		self.proceed_point_tracking = rospy.Service(self.topic_root+"/proceed_point_tracking", TrigPointTracking, self.point_track)

		#Defining service to set publish rate
		self.set_rate_service = rospy.Service(self.topic_root+"/set_rate", SetRate, self.set_rate)
		self.linetracker = lt.LineTracking()

		while not rospy.is_shutdown():
			if not self.is_sleep_state:
				#Publish the wishpose
				
				if self.is_line_tracking:
					(r, p, y) = tf.transformations.euler_from_quaternion([self.estimated_pose.pose.pose.orientation.x, self.estimated_pose.pose.pose.orientation.y, 
																	self.estimated_pose.pose.pose.orientation.z, self.estimated_pose.pose.pose.orientation.w])

					img = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
					unit_dir, unit_line_dir, sum_vect, frame, edge = self.linetracker.update(img, [r,p,y], self.estimated_pose.pose.pose.position.z, 150)
					self.viz_img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

					unit = sum_vect

					yaw = (np.arctan(unit[1]/unit[0])) + y

					self.wish_command.x = 0.5
					self.wish_command.y = 0.
					self.wish_command.z = yaw
					self.vizualisation_pub.publish(self.viz_img)
					self.wish_command_pub.publish(self.wish_command)

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