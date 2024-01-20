#!/usr/bin/env python3
import rospy
import math

from utils import find_closest_segment

from search import WishUponAStar
from obstacles import Obstacles
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped

class Navigator:
	def __init__(self, planning_frame, tf2_buffer, grid_size, obstacle_change_callback):
		self.planning_frame = planning_frame
		self.tf2_buffer = tf2_buffer
		self.grid_size = grid_size

		self.obstacles = Obstacles(planning_frame, tf2_buffer, grid_size, obstacle_change_callback)
		self.astar = WishUponAStar(self.obstacles)

		self.global_plan_pub = rospy.Publisher("line_planner/local_plan", Path, queue_size=1, latch=True)

	def get_pose(self, p):
		pose = Pose()
		pose.position.x = p[0] * self.grid_size
		pose.position.y = p[1] * self.grid_size
		pose.position.z = 0
		pose.orientation.w = 1.0
		return pose
	
	def get_pose_stamped(self, p):
		pose = PoseStamped()
		pose.header.frame_id = self.planning_frame
		pose.pose.position.x = p[0] * self.grid_size
		pose.pose.position.y = p[1] * self.grid_size
		pose.pose.position.z = 0
		pose.pose.orientation.w = 1.0
		return pose

	def publish_local_plan(self, poses):
		path = Path()
		path.header.frame_id = self.planning_frame
		path.poses = poses
		self.global_plan_pub.publish(path)

	def dist(self, p1, p2):
		return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

	def plan(self, robot_pos, start, goal, path_width):

		#no need to plan if there's no obstacles
		if self.obstacles.obstacle_count() == 0:
			self.publish_local_plan([])
			return [start, goal]
		
		#transform goals into grid space
		start_scaled = (start.position.x / self.grid_size, start.position.y / self.grid_size)
		goal_scaled = (goal.position.x / self.grid_size, goal.position.y / self.grid_size)

		#find path of max deviation width
		astarpath = self.astar.search(
			(round(start_scaled[0]), round(start_scaled[1])),
			(round(goal_scaled[0]), round(goal_scaled[1])),
			path_width / self.grid_size
		)

		#no path, but maybe we can make one ;)
		if len(astarpath) == 0:
			self.publish_local_plan([])
			return [start, goal]
		
		# continue path smoothly from a later segment in case of on the fly replanning
		if robot_pos != start:
			robot_scaled = (robot_pos.position.x / self.grid_size, robot_pos.position.y / self.grid_size)
			i = find_closest_segment(robot_scaled, astarpath)
			astarpath = astarpath[i:]

		
		diagonal = math.hypot(self.grid_size, self.grid_size)
		#print("start:",self.dist(astarpath[0], start_scaled)," end:", self.dist(astarpath[-1], goal_scaled)," dist:", diagonal)
		
		if self.dist(astarpath[0], start_scaled) < diagonal:
			astarpath[0] = start_scaled

		if self.dist(astarpath[-1], goal_scaled) < diagonal:
			astarpath[-1] = goal_scaled
		

		poses = []
		poses_stamped = []
		for point in astarpath:
			poses.append(self.get_pose(point))
			poses_stamped.append(self.get_pose_stamped(point))
		self.publish_local_plan(poses_stamped)

		return poses
	