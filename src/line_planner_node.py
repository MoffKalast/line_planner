#!/usr/bin/env python3
import rospy
import math
import tf
import tf2_ros
from utils import *

from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray

from tf.transformations import euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from line_planner.cfg import LinePlannerConfig

goals = [
	Point(0.0, 0.0, 0.0),
	Point(5.0, 0.0, 0.0),
	Point(5.0, 5.0, 0.0),
	Point(0.0, 5.0, 0.0)
]

class LineFollowingController:
	def __init__(self):
		rospy.init_node("line_following_controller")
		
		self.ABORT_TIMEOUT = rospy.get_param('abort_timeout', 30.0)
		self.MIN_GOAL_DIST = rospy.get_param('goal_distance_threshold', 0.6)

		self.MAX_ANGULAR_SPD = rospy.get_param('max_turning_velocity', 0.9)
		self.MAX_LINEAR_SPD = rospy.get_param('max_linear_velocity', 0.45)

		self.LINE_DIVERGENCE = rospy.get_param('max_line_divergence', 1.0)
		self.MIN_PROJECT_DIST = rospy.get_param('min_project_dist', 0.15)
		self.MAX_PROJECT_DIST = rospy.get_param('max_project_dist', 1.2)

		self.ROBOT_FRAME = rospy.get_param('robot_frame', 'base_link')
		self.PLANNING_FRAME = rospy.get_param('planning_frame', 'map')

		self.DEBUG_MARKERS = rospy.get_param('publish_debug_markers', True)

		self.tf_listener = tf.TransformListener()

		self.tf2_buffer = tf2_ros.Buffer()
		self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

		self.simple_goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.simple_goal_callback)
		self.clear_goals_sub = rospy.Subscriber("/move_base_simple/clear", Empty, self.clear_goal_callback)

		if self.DEBUG_MARKERS:
			self.marker_pub = rospy.Publisher("goal_markers", MarkerArray, queue_size=1)

		self.prev_goal = None
		self.current_goal = None
		self.goal_queue = []
		self.active = False

		self.pid = PID(
			rospy.get_param('P', 3.0),
			rospy.get_param('I', 0.001), 
			rospy.get_param('D', 65.0)
		)

		self.reconfigure_server = DynamicReconfigureServer(LinePlannerConfig, self.dynamic_reconfigure_callback)

		self.print_index = 0

		rospy.loginfo("Line planner started.")
		rospy.loginfo("Robot frame: "+self.ROBOT_FRAME)
		rospy.loginfo("Planning frame: "+self.PLANNING_FRAME)

	def clear_goal_callback(self, msg):
		self.prev_goal = None
		self.current_goal = None
		self.goal_queue = []
		self.active = False

	def simple_goal_callback(self, goal):

		if self.PLANNING_FRAME == goal.header.frame_id:
			rospy.loginfo("------------------")
			rospy.loginfo("Received goal in planning ("+self.PLANNING_FRAME+") frame.")
			rospy.loginfo("Position: X: %f, Y: %f, Z: %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)
			rospy.loginfo("Orientation: X: %f, Y: %f, Z: %f, W: %f", goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w)
			self.goal_queue.append(goal.pose)
		else:

			try:
				transform = self.tf2_buffer.lookup_transform(self.PLANNING_FRAME, goal.header.frame_id, goal.header.stamp, rospy.Duration(1.0))
				goal_transformed = do_transform_pose(goal, transform)
				self.goal_queue.append(goal_transformed.pose)
				rospy.loginfo("------------------")
				rospy.loginfo("Received goal in "+goal.header.frame_id+" frame, transformed to "+self.PLANNING_FRAME+".")
				rospy.loginfo("Position: X: %f, Y: %f, Z: %f", goal_transformed.pose.position.x, goal_transformed.pose.position.y, goal_transformed.pose.position.z)
				rospy.loginfo("Orientation: X: %f, Y: %f, Z: %f, W: %f", goal_transformed.pose.orientation.x, goal_transformed.pose.orientation.y, goal_transformed.pose.orientation.z, goal_transformed.pose.orientation.w)

			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				rospy.logwarn("TF2 exception: %s", e)
				return
		

	def dynamic_reconfigure_callback(self, config, level):
		self.pid.kp = config.P
		self.pid.ki = config.I
		self.pid.kd = config.D

		self.ABORT_TIMEOUT = config.abort_timeout
		self.MIN_GOAL_DIST = config.goal_distance_threshold

		self.MAX_ANGULAR_SPD = config.max_turning_velocity
		self.MAX_LINEAR_SPD = config.max_linear_velocity

		self.LINE_DIVERGENCE = config.max_line_divergence
		self.MIN_PROJECT_DIST = config.min_project_dist
		self.MAX_PROJECT_DIST = config.max_project_dist

		self.DEBUG_MARKERS = config.publish_debug_markers

		return config

	def create_marker(self, position, marker_id, r, g, b, size):
		marker = Marker()
		marker.header.frame_id = self.PLANNING_FRAME
		marker.type = marker.SPHERE
		marker.pose.position = position
		marker.scale.x = size
		marker.scale.y = size
		marker.scale.z = size
		marker.color.a = 0.5
		marker.color.r = r
		marker.color.g = g
		marker.color.b = b
		marker.id = marker_id
		return marker


	def process_goal_queue(self):
		if self.current_goal == None:
			try:
				self.prev_goal = transform_to_pose(self.tf2_buffer.lookup_transform(self.PLANNING_FRAME, self.ROBOT_FRAME, rospy.Time(0)))
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				rospy.logwarn("TF2 exception: %s", e)
				return
		else:
			self.prev_goal = self.current_goal
		
		self.current_goal = self.goal_queue.pop(0)
		self.active = True

	def update(self):

		if not self.active:
			if len(self.goal_queue) > 0:
				self.process_goal_queue()
			else:
				return
		
		try:
			
			pose = transform_to_pose(self.tf2_buffer.lookup_transform(self.PLANNING_FRAME, self.ROBOT_FRAME, rospy.Time(0)))
			_, _, current_yaw = euler_from_quaternion([
				pose.orientation.x,
				pose.orientation.y,
				pose.orientation.z,
				pose.orientation.w,
			])

			target_position = project_position(
				self.prev_goal,
				self.current_goal,
				pose,
				self.MIN_PROJECT_DIST,
				self.MAX_PROJECT_DIST,
				self.LINE_DIVERGENCE
			)
			
			target_unit_vector = get_dir(pose.position, target_position)
			current_unit_vector = [math.cos(current_yaw), math.sin(current_yaw)]

			dot_product = target_unit_vector[0] * current_unit_vector[0] + target_unit_vector[1] * current_unit_vector[1]
			cross_product = current_unit_vector[0] * target_unit_vector[1] - current_unit_vector[1] * target_unit_vector[0]

			angle_error = -math.acos(dot_product) * math.copysign(1, cross_product)
			angular_velocity = clamp(self.pid.compute(angle_error), -self.MAX_ANGULAR_SPD, self.MAX_ANGULAR_SPD)

			deltax = self.current_goal.position.x - pose.position.x
			deltay = self.current_goal.position.y - pose.position.y

			target_distance = math.sqrt(deltax** 2 + deltay ** 2)

			if target_distance > self.MIN_GOAL_DIST:
				linear_velocity = clamp((self.MAX_LINEAR_SPD+0.1) - math.fabs(angle_error) * self.MAX_LINEAR_SPD * 0.65, 0.0, self.MAX_LINEAR_SPD)
			else:
				linear_velocity = 0
				angular_velocity = 0
				self.active = False

			twist = Twist()
			twist.linear.x = linear_velocity
			twist.angular.z = angular_velocity

			self.cmd_vel_pub.publish(twist)

			if self.DEBUG_MARKERS:

				if self.print_index == 0:
					print("Angle:",angle_error,"Vel:",linear_velocity, "Tgt:",target_distance)

					markerArray = MarkerArray()
					markerArray.markers.append(self.create_marker(self.prev_goal.position, 0, 1.0, 0.0, 0.0, self.MIN_GOAL_DIST))
					markerArray.markers.append(self.create_marker(self.current_goal.position, 1, 0.0, 0.0, 1.0, self.MIN_GOAL_DIST))
					markerArray.markers.append(self.create_marker(target_position, 2, 0.0, 1.0, 0.0, 0.25))
					self.marker_pub.publish(markerArray)
				
				self.print_index = (self.print_index +1)%5


		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logwarn("TF Exception")


ctrl = LineFollowingController()
rate = rospy.Rate(rospy.get_param('rate', 30))

while not rospy.is_shutdown():
	ctrl.update()
	rate.sleep()
	