#!/usr/bin/env python3
import rospy

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

	def plan(self, start, goal):

		if self.obstacles.obstacle_count() == 0:
			self.publish_local_plan([])
			return [start, goal]
		
		x0 = round(start.position.x / self.grid_size)
		y0 = round(start.position.y / self.grid_size)

		x1 = round(goal.position.x / self.grid_size)
		y1 = round(goal.position.y / self.grid_size)

		astarpath = self.astar.search((x0,y0),(x1,y1))

		if len(astarpath) == 0:
			self.publish_local_plan([])
			return [start, goal]

		#astarpath[0] = (start.position.x / self.grid_size, start.position.y / self.grid_size)
		#astarpath[-1] = (goal.position.x / self.grid_size, goal.position.y / self.grid_size)

		poses = []
		poses_stamped = []
		for point in astarpath:
			poses.append(self.get_pose(point))
			poses_stamped.append(self.get_pose_stamped(point))
		self.publish_local_plan(poses_stamped)

		return poses