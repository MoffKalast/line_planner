#!/usr/bin/env python3
import rospy
import math
import tf
import tf2_ros

from utils import *
from projector import *

from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

from tf.transformations import euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose

from actionlib import SimpleActionServer
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from line_planner.cfg import LinePlannerConfig

ROBOT_FRAME = "base_link"
PLANNING_FRAME = "map"

class PathServer:
	def __init__(self, tf2_buffer, active_server_callback=None):
		self.tf2_buffer = tf2_buffer

class ActionGoalServer:
	def __init__(self, tf2_buffer, active_server_callback=None):
		self.start_goal = None
		self.end_goal = None
		self.tf2_buffer = tf2_buffer
		self.active_server_callback = active_server_callback
		self.action_server = SimpleActionServer('move_base', MoveBaseAction, execute_cb=self.goal_callback, auto_start=True)

	def get_goals(self):
		return self.start_goal, self.end_goal
	
	def goal_reached(self):
		self.start_goal = None
		self.end_goal = None

	def set_goal_pair(self, endgoal):
		try:
			self.start_goal = transform_to_pose(self.tf2_buffer.lookup_transform(PLANNING_FRAME, ROBOT_FRAME, rospy.Time(0)))
			self.end_goal = endgoal
			self.active_server_callback(self)
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			rospy.logwarn("TF2 exception: %s", e)

	def goal_callback(self, goal):

		print(goal)

""" 		if PLANNING_FRAME == goal.header.frame_id:
			rospy.loginfo("------------------")
			rospy.loginfo("Received simple goal in planning ("+PLANNING_FRAME+") frame.")
			rospy.loginfo("Position: X: %f, Y: %f, Z: %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)
			rospy.loginfo("Orientation: X: %f, Y: %f, Z: %f, W: %f", goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w)
			self.set_goal_pair(goal.pose)
		else:
			try:
				transform = self.tf2_buffer.lookup_transform(PLANNING_FRAME, goal.header.frame_id, goal.header.stamp, rospy.Duration(1.0))
				goal_transformed = do_transform_pose(goal, transform)
				self.set_goal_pair(goal_transformed.pose)
				rospy.loginfo("------------------")
				rospy.loginfo("Received simple goal in "+goal.header.frame_id+" frame, transformed to "+PLANNING_FRAME+".")
				rospy.loginfo("Position: X: %f, Y: %f, Z: %f", goal_transformed.pose.position.x, goal_transformed.pose.position.y, goal_transformed.pose.position.z)
				rospy.loginfo("Orientation: X: %f, Y: %f, Z: %f, W: %f", goal_transformed.pose.orientation.x, goal_transformed.pose.orientation.y, goal_transformed.pose.orientation.z, goal_transformed.pose.orientation.w)

			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				rospy.logwarn("TF2 exception: %s", e)
				return """

class SimpleGoalServer:
	def __init__(self, tf2_buffer, active_server_callback=None):
		self.start_goal = None
		self.end_goal = None
		self.tf2_buffer = tf2_buffer
		self.active_server_callback = active_server_callback

		self.simple_goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
		self.clear_goals_sub = rospy.Subscriber("/move_base_simple/clear", Empty, self.clear_callback)

	def get_goals(self):
		return self.start_goal, self.end_goal
	
	def goal_reached(self):
		self.start_goal = None
		self.end_goal = None

	def set_goal_pair(self, endgoal):
		try:
			self.start_goal = transform_to_pose(self.tf2_buffer.lookup_transform(PLANNING_FRAME, ROBOT_FRAME, rospy.Time(0)))
			self.end_goal = endgoal
			self.active_server_callback(self)
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			rospy.logwarn("TF2 exception: %s", e)

	def goal_callback(self, goal):

		if PLANNING_FRAME == goal.header.frame_id:
			rospy.loginfo("------------------")
			rospy.loginfo("Received simple goal in planning ("+PLANNING_FRAME+") frame.")
			rospy.loginfo("Position: X: %f, Y: %f, Z: %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)
			rospy.loginfo("Orientation: X: %f, Y: %f, Z: %f, W: %f", goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w)
			self.set_goal_pair(goal.pose)
		else:

			try:
				transform = self.tf2_buffer.lookup_transform(PLANNING_FRAME, goal.header.frame_id, goal.header.stamp, rospy.Duration(1.0))
				goal_transformed = do_transform_pose(goal, transform)
				self.set_goal_pair(goal_transformed.pose)
				rospy.loginfo("------------------")
				rospy.loginfo("Received simple goal in "+goal.header.frame_id+" frame, transformed to "+PLANNING_FRAME+".")
				rospy.loginfo("Position: X: %f, Y: %f, Z: %f", goal_transformed.pose.position.x, goal_transformed.pose.position.y, goal_transformed.pose.position.z)
				rospy.loginfo("Orientation: X: %f, Y: %f, Z: %f, W: %f", goal_transformed.pose.orientation.x, goal_transformed.pose.orientation.y, goal_transformed.pose.orientation.z, goal_transformed.pose.orientation.w)

			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				rospy.logwarn("TF2 exception: %s", e)
				return

	def clear_callback(self, msg):
		self.start_goal = None
		self.end_goal = None

class LineFollowingController:
	def __init__(self):
		rospy.init_node("line_following_controller")
		global ROBOT_FRAME, PLANNING_FRAME

		ROBOT_FRAME = rospy.get_param('robot_frame', 'base_link')
		PLANNING_FRAME = rospy.get_param('planning_frame', 'map')
		
		self.ABORT_TIMEOUT = rospy.get_param('abort_timeout', 30.0)
		self.MIN_GOAL_DIST = rospy.get_param('goal_distance_threshold', 0.6)

		self.MAX_ANGULAR_SPD = rospy.get_param('max_turning_velocity', 0.9)
		self.MAX_LINEAR_SPD = rospy.get_param('max_linear_velocity', 0.45)

		self.LINE_DIVERGENCE = rospy.get_param('max_line_divergence', 1.0)
		self.MIN_PROJECT_DIST = rospy.get_param('min_project_dist', 0.15)
		self.MAX_PROJECT_DIST = rospy.get_param('max_project_dist', 1.2)

		self.SIDE_OFFSET_MULT = rospy.get_param('side_offset_mult', 0.5)

		self.ROBOT_FRAME = rospy.get_param('robot_frame', 'base_link')
		self.PLANNING_FRAME = rospy.get_param('planning_frame', 'map')

		self.DEBUG_MARKERS = rospy.get_param('publish_debug_markers', True)

		self.tf_listener = tf.TransformListener()

		self.tf2_buffer = tf2_ros.Buffer()
		self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
		self.marker_pub = rospy.Publisher("goal_markers", MarkerArray, queue_size=1)

		self.pid = PID(
			rospy.get_param('P', 3.0),
			rospy.get_param('I', 0.001), 
			rospy.get_param('D', 65.0)
		)

		self.active_server = None
		self.simple_server = SimpleGoalServer(self.tf2_buffer, self.set_active_server)
		#self.actiongoal_server = ActionGoalServer(self.set_active_server)
		#self.path_server = PathServer(self.set_active_server)

		self.reconfigure_server = DynamicReconfigureServer(LinePlannerConfig, self.dynamic_reconfigure_callback)

		self.marker_publish_skip = 0

		rospy.loginfo("Line planner started.")
		rospy.loginfo("Robot frame: "+ROBOT_FRAME)
		rospy.loginfo("Planning frame: "+PLANNING_FRAME)
		
	def set_active_server(self, server):
		self.active_server = server

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

	def get_angle_error(self, current_pose, target_position):
		_, _, current_yaw = euler_from_quaternion([
			current_pose.orientation.x,
			current_pose.orientation.y,
			current_pose.orientation.z,
			current_pose.orientation.w,
		])

		target_unit_vector = get_dir(current_pose.position, target_position)
		current_unit_vector = [math.cos(current_yaw), math.sin(current_yaw)]

		#get the angle cosine
		dot_product = target_unit_vector[0] * current_unit_vector[0] + target_unit_vector[1] * current_unit_vector[1]
		
		#get the direction
		cross_product = current_unit_vector[0] * target_unit_vector[1] - current_unit_vector[1] * target_unit_vector[0]

		return -math.copysign(math.acos(dot_product), cross_product)

	def get_distance(self, pose, goal):
		deltax = goal.position.x - pose.position.x
		deltay = goal.position.y - pose.position.y
		return math.sqrt(deltax** 2 + deltay ** 2)

	def update(self):

		if self.active_server == None:
			return
		
		start_goal, end_goal = self.active_server.get_goals()

		if start_goal == None or end_goal == None:
			self.active_server = None
			if self.DEBUG_MARKERS:
				self.delete_debug_markers()
			return
		
		try:
			pose = transform_to_pose(self.tf2_buffer.lookup_transform(PLANNING_FRAME, ROBOT_FRAME, rospy.Time(0)))

			target_position = project_position(
				start_goal,
				end_goal,
				pose,
				self.MIN_PROJECT_DIST,
				self.MAX_PROJECT_DIST,
				self.LINE_DIVERGENCE,
				self.SIDE_OFFSET_MULT
			)
			
			angle_error = self.get_angle_error(pose, target_position)
			angular_velocity = clamp(self.pid.compute(angle_error), -self.MAX_ANGULAR_SPD, self.MAX_ANGULAR_SPD)
			target_distance = self.get_distance(end_goal, pose)

			if target_distance > self.MIN_GOAL_DIST:
				linear_velocity = clamp((self.MAX_LINEAR_SPD+0.1) - math.fabs(angle_error) * self.MAX_LINEAR_SPD * 0.65, 0.0, self.MAX_LINEAR_SPD)
			else:
				linear_velocity = 0
				angular_velocity = 0
				self.active_server.goal_reached()

			self.send_twist(linear_velocity, angular_velocity)

			if self.DEBUG_MARKERS:
				self.draw_debug_markers(target_position, start_goal, end_goal)

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logwarn("TF Exception")

	def send_twist(self, vel_x, vel_z):
		twist = Twist()
		twist.linear.x = vel_x
		twist.angular.z = vel_z
		self.cmd_vel_pub.publish(twist)

	def delete_debug_markers(self):

		def delete_marker(marker_id):
			marker = Marker()
			marker.action = 2
			marker.id = marker_id
			return marker

		markerArray = MarkerArray()
		markerArray.markers.append(delete_marker(0))
		markerArray.markers.append(delete_marker(1))
		markerArray.markers.append(delete_marker(2))
		self.marker_pub.publish(markerArray)

	def draw_debug_markers(self, target_position, start_goal, end_goal):

		def delete_marker(marker_id):
			marker = Marker()
			marker.action = 2
			marker.id = marker_id
			return marker
		
		def set_marker(position, marker_id, r, g, b, size):
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

		# to avoid flooding
		if self.marker_publish_skip == 0:
			markerArray = MarkerArray()
			markerArray.markers.append(set_marker(start_goal.position, 0, 1.0, 0.0, 0.0, self.MIN_GOAL_DIST))
			markerArray.markers.append(set_marker(end_goal.position, 1, 0.0, 0.0, 1.0, self.MIN_GOAL_DIST))
			markerArray.markers.append(set_marker(target_position, 2, 0.0, 1.0, 0.0, 0.25))
			self.marker_pub.publish(markerArray)
		
		self.marker_publish_skip = (self.marker_publish_skip +1)%5



ctrl = LineFollowingController()
rate = rospy.Rate(rospy.get_param('rate', 30))

while not rospy.is_shutdown():
	ctrl.update()
	rate.sleep()
	