#!/usr/bin/env python3
import rospy
import math
import tf
import tf2_ros
import sys

from utils import *
from navigator import Navigator
from markers import DebugMarkers

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Empty, Bool

from tf.transformations import euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from line_planner.cfg import LinePlannerConfig

from nav_msgs.msg import Path

ROBOT_FRAME = "base_link"
PLANNING_FRAME = "map"
GRID_SIZE = 1.0

class GoalServer:

	def __init__(self, tf2_buffer, max_path_width, update_plan):
		self.start_goal = None
		self.end_goal = None
		self.tf2_buffer = tf2_buffer
		self.update_plan = update_plan
		self.max_path_width = max_path_width

		self.simple_goal_sub = rospy.Subscriber("/move_base_simple/waypoints", Path, self.route_callback)
		self.simple_goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
		self.clear_goals_sub = rospy.Subscriber("/move_base_simple/clear", Empty, self.reset)

		self.original_start_goal = None
		self.original_end_goal = None

		self.start_goal = None
		self.end_goal = None

		self.route = []
		self.route_index = 0

		self.subroute = []
		self.navigator = Navigator(tf2_buffer, PLANNING_FRAME, GRID_SIZE, self.obstacle_change_callback)

	def reset(self, msg):
		self.original_start_goal = None
		self.original_end_goal = None
		self.start_goal = None
		self.end_goal = None
		self.route = []
		self.route_index = 0
		self.subroute = []
		self.update_plan()
		self.navigator.obstacles.clear_grid(None)

	def obstacle_change_callback(self):
		if self.original_start_goal is None or self.original_end_goal is None:
			return
		
		try:
			robot_pos = transform_to_pose(self.tf2_buffer.lookup_transform(PLANNING_FRAME, ROBOT_FRAME, rospy.Time(0)))
			self.subroute = self.navigator.plan(robot_pos, robot_pos, self.original_end_goal, self.max_path_width)

			if len(self.subroute) >= 2:
				self.start_goal = self.subroute[0]
				self.end_goal = self.subroute[1]
				self.subroute = self.subroute[1:]
				self.update_plan()
			else:
				self.reset(None)

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			rospy.logwarn("TF2 exception: %s", e)
			self.reset(None)

	def goal_callback(self, goal):
		self.reset(None)
		#self.route = []
		#self.route_index = 0
		self.process_goal(goal)
		self.update_plan()

	def route_callback(self, msg):
		self.reset(None)
		rospy.loginfo("New route received.")

		if len(msg.poses) == 0:
			rospy.loginfo("Empty route, stopping.")
			#self.reset(None)
			return

		self.route = msg.poses
		self.route_index = 0
		self.process_goal(self.route[0])
		self.update_plan()

	def get_goals(self):
		return self.start_goal, self.end_goal
	
	def goal_reached(self):
		subroute_len = len(self.subroute) 

		#if we're following a subroute, continue
		if subroute_len > 0:
			if subroute_len == 1:
				self.navigator.publish_local_plan([])
			else:
				self.start_goal = self.subroute[0]
				self.end_goal = self.subroute[1]
				self.subroute = self.subroute[1:]
				self.update_plan()

				rospy.loginfo("Continuing detour, "+str(len(self.subroute)-1)+" local subgoals left.")
				return

		if len(self.route) > 0:
			rospy.loginfo("Goal #%i reached.",self.route_index)
			if self.route_index < len(self.route)-1:
				self.route_index +=1
				self.process_goal(self.route[self.route_index])
			else:
				rospy.loginfo("-> Route finished.")
				self.reset(None)
		else:
			rospy.loginfo("Simple goal reached.")
			self.reset(None)

		self.update_plan()

	def set_goal_pair(self, endgoal):
		# set up the two currently active points to follow a line between
		try:
			robot_pos = transform_to_pose(self.tf2_buffer.lookup_transform(PLANNING_FRAME, ROBOT_FRAME, rospy.Time(0)))

			if len(self.route) == 0 or self.route_index == 0 or self.subroute:
				self.start_goal = robot_pos
				self.end_goal = endgoal
			else:
				self.start_goal = self.end_goal
				self.end_goal = endgoal

			self.original_start_goal = self.start_goal
			self.original_end_goal = self.end_goal

			self.subroute = self.navigator.plan(robot_pos, self.start_goal, self.end_goal, self.max_path_width)

			if len(self.subroute) >= 2:
				self.start_goal = self.subroute[0]
				self.end_goal = self.subroute[1]
				self.subroute = self.subroute[1:]
				self.update_plan()
			else:
				self.reset(None)			

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			rospy.logwarn("TF2 exception: %s", e)

	def process_goal(self, goal):
		# transform goal into planning frame, print debug info
		if PLANNING_FRAME == goal.header.frame_id:
			rospy.loginfo("------------------")
			rospy.loginfo("Received goal in planning ("+PLANNING_FRAME+") frame.")
			rospy.loginfo("Position: X: %f, Y: %f, Z: %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)
			rospy.loginfo("Orientation: X: %f, Y: %f, Z: %f, W: %f", goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w)
			self.set_goal_pair(goal.pose)
		else:
			try:
				transform = self.tf2_buffer.lookup_transform(PLANNING_FRAME, goal.header.frame_id, rospy.Time(0))
				goal_transformed = do_transform_pose(goal, transform)
				self.set_goal_pair(goal_transformed.pose)
				rospy.loginfo("------------------")
				rospy.loginfo("Received goal in "+goal.header.frame_id+" frame, transformed to "+PLANNING_FRAME+".")
				rospy.loginfo("Position: X: %f, Y: %f, Z: %f", goal_transformed.pose.position.x, goal_transformed.pose.position.y, goal_transformed.pose.position.z)
				rospy.loginfo("Orientation: X: %f, Y: %f, Z: %f, W: %f", goal_transformed.pose.orientation.x, goal_transformed.pose.orientation.y, goal_transformed.pose.orientation.z, goal_transformed.pose.orientation.w)

			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				rospy.logwarn("TF2 exception: %s", e)
				return

class LineFollowingController:
	def __init__(self):
		rospy.init_node("line_following_controller")
		global ROBOT_FRAME, PLANNING_FRAME, GRID_SIZE

		ROBOT_FRAME = rospy.get_param('~robot_frame', 'base_link')
		PLANNING_FRAME = rospy.get_param('~planning_frame', 'map')
		GRID_SIZE = rospy.get_param('~obstacle_grid_size', 1.0)
		
		self.MIN_GOAL_DIST = rospy.get_param('~goal_distance_threshold', 0.6)
		self.MAX_ANGULAR_SPD = rospy.get_param('~max_turning_velocity', 0.9)

		self.LINEAR_ACCEL = rospy.get_param('~linear_acceleration', 0.1)
		self.MIN_LINEAR_SPD = rospy.get_param('~min_linear_velocity', 0.1)
		self.MAX_LINEAR_SPD = rospy.get_param('max_linear_velocity', 0.45)

		self.ROBOT_WIDTH = rospy.get_param('~robot_width', 0.3)
		self.LINE_DIVERGENCE = rospy.get_param('~max_line_divergence', 1.0)
		self.MIN_PROJECT_DIST = rospy.get_param('~min_project_dist', 0.15)
		self.MAX_PROJECT_DIST = rospy.get_param('~max_project_dist', 1.2)

		self.SIDE_OFFSET_MULT = rospy.get_param('~side_offset_mult', 0.5)

		self.DEBUG_MARKERS = rospy.get_param('~publish_debug_markers', True)
		self.markers = DebugMarkers(PLANNING_FRAME)

		self.tf_listener = tf.TransformListener()

		self.tf2_buffer = tf2_ros.Buffer()
		self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

		self.status_pub = rospy.Publisher("line_planner/active", Bool, queue_size=1, latch=True)
		self.plan_pub = rospy.Publisher("line_planner/plan", Path, queue_size=1, latch=True)

		self.pid = PID(
			rospy.get_param('P', 3.0),
			rospy.get_param('I', 0.001), 
			rospy.get_param('D', 65.0)
		)

		self.goal_server = GoalServer(self.tf2_buffer, self.LINE_DIVERGENCE+self.ROBOT_WIDTH, self.update_plan)
		self.active = False

		self.reconfigure_server = DynamicReconfigureServer(LinePlannerConfig, self.dynamic_reconfigure_callback)

		self.status_pub.publish(False)

		rospy.loginfo("Line planner started.")
		rospy.loginfo("Robot frame: "+ROBOT_FRAME)
		rospy.loginfo("Planning frame: "+PLANNING_FRAME)

	def dynamic_reconfigure_callback(self, config, level):
		self.pid.kp = config.P
		self.pid.ki = config.I
		self.pid.kd = config.D

		self.MIN_GOAL_DIST = config.goal_distance_threshold

		self.LINEAR_ACCEL = config.linear_acceleration
		self.MAX_LINEAR_SPD = config.max_linear_velocity

		self.MAX_ANGULAR_SPD = config.max_turning_velocity
		self.MAX_LINEAR_SPD = config.max_linear_velocity

		self.ROBOT_WIDTH = config.robot_width
		self.LINE_DIVERGENCE = config.max_line_divergence
		self.MIN_PROJECT_DIST = config.min_project_dist
		self.MAX_PROJECT_DIST = config.max_project_dist

		self.DEBUG_MARKERS = config.publish_debug_markers

		self.goal_server.max_path_width = self.LINE_DIVERGENCE+self.ROBOT_WIDTH

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

	def get_linear_velocity(self, distance, angle_error):
		vel = self.MAX_LINEAR_SPD

		# check for correct orientation
		abserr = math.fabs(angle_error)

		if abserr > 2.0:
			vel *= clamp(-1.598 * abserr + 3.196, -1.0, 0.0) # gradually reverse from 120 to 180 deg heading
			return clamp(vel, -self.MAX_LINEAR_SPD, 0.0)
		
		if abserr > 0.52:
			vel *= clamp((-1.0 / 0.52) * abserr + 2, 0.0, 1.0) # gradually decrease velocity from 30 to 60 deg heading
		
		return clamp(vel, 0.0, self.MAX_LINEAR_SPD)


	def update(self):
		start_goal, end_goal = self.goal_server.get_goals()

		if start_goal == None or end_goal == None:

			if self.active:
				self.send_twist(0, 0)
				if self.DEBUG_MARKERS:
					self.markers.delete_debug_markers()
				self.active = False
				self.status_pub.publish(False)
			return
		
		try:
			self.active = True
			self.status_pub.publish(True)
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
				linear_velocity = self.get_linear_velocity(target_distance, angle_error)
			else:
				linear_velocity = 0
				angular_velocity = 0
				self.goal_server.goal_reached()

			self.send_twist(linear_velocity, angular_velocity)

			if self.DEBUG_MARKERS:
				self.markers.draw_debug_markers(target_position, start_goal, end_goal, self.MIN_GOAL_DIST, self.LINE_DIVERGENCE)

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logwarn("TF Exception")
		except Exception as e:
			rospy.logerr('An exception occurred: {}'.format(e))
		except:
			rospy.logerr("Unexpected error: "+str(sys.exc_info()[0]))

	def update_plan(self):

		msg = Path()
		msg.header.frame_id = PLANNING_FRAME

		if len(self.goal_server.route) > 1:
			msg.poses = self.goal_server.route[self.goal_server.route_index:]
		else:
			start_goal, end_goal = self.goal_server.get_goals()

			if start_goal == None or end_goal == None:
				self.plan_pub.publish(Path())
				return

			start_stamped = PoseStamped()
			start_stamped.header.frame_id = PLANNING_FRAME
			start_stamped.pose = start_goal
			msg.poses.append(start_stamped)

			end_stamped = PoseStamped()
			end_stamped.header.frame_id = PLANNING_FRAME
			end_stamped.pose = end_goal		

			msg.poses = [start_stamped, end_stamped]

		self.plan_pub.publish(msg)

	def send_twist(self, vel_x, vel_z):
		#sanity check, just in case
		if math.isnan(vel_x):
			vel_x = 0

		if math.isnan(vel_z):
			vel_z = 0

		twist = Twist()
		twist.linear.x = vel_x
		twist.angular.z = vel_z
		self.cmd_vel_pub.publish(twist)

	def cleanup(self):
		self.send_twist(0,0)
		self.markers.delete_debug_markers()

ctrl = LineFollowingController()
rate = rospy.Rate(rospy.get_param('rate', 30))
rospy.on_shutdown(ctrl.cleanup)

while not rospy.is_shutdown():
	ctrl.update()
	rate.sleep()
