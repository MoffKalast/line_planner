#!/usr/bin/env python3
import rospy
import math

from utils import find_closest_segment

from search import WishUponAStar
from obstacles import Obstacles
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped

class Navigator:
	def __init__(self, tf2_buffer, PLANNING_FRAME, GRID_SIZE, obstacle_change_callback):

		self.tf2_buffer = tf2_buffer
		self.PLANNING_FRAME = PLANNING_FRAME
		self.GRID_SIZE = GRID_SIZE

		self.obstacles = Obstacles(tf2_buffer, PLANNING_FRAME, GRID_SIZE, obstacle_change_callback)
		self.astar = WishUponAStar(self.obstacles)

		self.global_plan_pub = rospy.Publisher("line_planner/local_plan", Path, queue_size=1, latch=True)

	def get_pose(self, p):
		pose = Pose()
		pose.position.x = p[0] * self.GRID_SIZE
		pose.position.y = p[1] * self.GRID_SIZE
		pose.position.z = 0
		pose.orientation.w = 1.0
		return pose
	
	def get_pose_stamped(self, p):
		pose = PoseStamped()
		pose.header.frame_id = self.PLANNING_FRAME
		pose.pose.position.x = p[0] * self.GRID_SIZE
		pose.pose.position.y = p[1] * self.GRID_SIZE
		pose.pose.position.z = 0
		pose.pose.orientation.w = 1.0
		return pose

	def publish_local_plan(self, poses):
		path = Path()
		path.header.frame_id = self.PLANNING_FRAME
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
		start_scaled = (start.position.x / self.GRID_SIZE, start.position.y / self.GRID_SIZE)
		goal_scaled = (goal.position.x / self.GRID_SIZE, goal.position.y / self.GRID_SIZE)

		#find path of max deviation width
		astarpath = self.astar.search(
			(round(start_scaled[0]), round(start_scaled[1])),
			(round(goal_scaled[0]), round(goal_scaled[1])),
			path_width / self.GRID_SIZE
		)

		#no path, we're screwed
		if astarpath is None:
			rospy.logerr("Path not found!")
			print("astarpath",astarpath)
			print("path_width",path_width)
			print("start",(round(start_scaled[0]), round(start_scaled[1])))
			print("goal",(round(goal_scaled[0]), round(goal_scaled[1])))
			print("obstacles",self.obstacles.obstacle_count())
			self.publish_local_plan([])
			return []
		
		# continue path smoothly from a later segment in case of on the fly replanning
		if robot_pos != start:
			robot_scaled = (robot_pos.position.x / self.GRID_SIZE, robot_pos.position.y / self.GRID_SIZE)
			i = find_closest_segment(robot_scaled, astarpath)
			rospy.loginfo("Continuing from segment ",i)
			astarpath = astarpath[i:]

		
		diagonal = math.hypot(self.GRID_SIZE, self.GRID_SIZE)
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
	