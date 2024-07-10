#!/usr/bin/env python3
import rospy
import math
import tf
import tf2_ros
import sys
from utils import *
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from line_planner.cfg import AltitudeControllerConfig

class AltitudeController:
	def __init__(self):
		rospy.init_node("altitude_controller")

		self.ROBOT_FRAME = rospy.get_param('~robot_frame', 'base_link')
		self.PLANNING_FRAME = rospy.get_param('~planning_frame', 'map')
		self.MAX_VEL = rospy.get_param('~max_velocity', 1.0)
		self.TIMEOUT_DURATION = rospy.get_param('~timeout_duration', 1.0) 

		self.tf2_buffer = tf2_ros.Buffer()
		self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

		self.target = 1.0
		self.target_sub = rospy.Subscriber("altitude_target", Float32, self.altitude_callback)

		self.cmd_vel_sub = rospy.Subscriber("cmd_vel_in", Twist, self.cmd_vel_callback)
		self.cmd_vel_pub = rospy.Publisher("cmd_vel_out", Twist, queue_size=1)

		self.pid = PID(
			rospy.get_param('P', 5.0),
			rospy.get_param('I', 0.005), 
			rospy.get_param('D', 2.0)
		)

		self.reconfigure_server = DynamicReconfigureServer(AltitudeControllerConfig, self.dynamic_reconfigure_callback)

		self.active = False
		self.last_cmd_vel_time = rospy.Time.now()
		self.timeout_timer = rospy.Timer(rospy.Duration(1.0), self.check_timeout)

	def dynamic_reconfigure_callback(self, config, level):
		self.pid.kp = config.P
		self.pid.ki = config.I
		self.pid.kd = config.D
		self.MAX_VEL = config.max_velocity
		return config

	def altitude_callback(self, msg):
		self.target = msg.data

	def cmd_vel_callback(self, msg):
		self.active = True
		self.last_cmd_vel_time = rospy.Time.now()

		pose = transform_to_pose(self.tf2_buffer.lookup_transform(self.PLANNING_FRAME, self.ROBOT_FRAME, rospy.Time(0)))        

		delta_error = pose.position.z - self.target
		msg.linear.z = clamp(self.pid.compute(delta_error), -self.MAX_VEL, self.MAX_VEL)

		self.cmd_vel_pub.publish(msg)

	def check_timeout(self, event):
		if self.active and rospy.Time.now() - self.last_cmd_vel_time > rospy.Duration(self.TIMEOUT_DURATION):
			rospy.logwarn("Altitude controller timeout, sending zero twist.")
			zero_twist = Twist()
			self.cmd_vel_pub.publish(zero_twist)
			self.active = False

ctrl = AltitudeController()
rospy.spin()
