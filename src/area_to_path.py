#!/usr/bin/env python3

import math
import rospy
import itertools

from nav_msgs.msg import Path
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point 

class BoundingBox3DListener:
	def __init__(self):
		rospy.init_node("area_to_path_node")

		self.path_pub = rospy.Publisher("/move_base_simple/waypoints", Path, queue_size=10, latch=True)

		self.lawnmower_sub = rospy.Subscriber("/area_to_path/lawnmower", PolygonStamped, self.lawnmower_callback)
		self.square_sub = rospy.Subscriber("/area_to_path/expanding_square", PolygonStamped, self.square_callback)
		self.sierra_sub = rospy.Subscriber("/area_to_path/victor_sierra", PolygonStamped, self.sierra_callback)
		self.home_sub = rospy.Subscriber("/area_to_path/home", PolygonStamped, self.home_callback)

		self.planner_state_sub = rospy.Subscriber("/line_planner/active", Bool, self.planner_callback)

		self.step_size = rospy.get_param("~step_size", 2.0)  # The step size (X meters) in the path

		self.home_point = None
		self.home_header = None

	def planner_callback(self, msg):
		# clear path info, since the planner has finished
		if not msg.data:
			path = Path()
			self.path_pub.publish(path)

	def send_path(self, header, poses):
		if len(poses) == 0:
			rospy.logwarn("Send a larger polygon or reduce step size.")
			return

		path = Path()
		path.header = header
		path.poses = poses
		self.path_pub.publish(path)

	def home_callback(self, msg):
		p = msg.polygon.points

		if len(p) != 4:
			rospy.logwarn("Invalid Polygon, path creation requires a 4 vertex square.")
			return

		self.home_point = Point(
			x=(p[0].x + p[2].x) / 2,
			y=(p[0].y + p[2].y) / 2
		)

		self.home_header = msg.header

		rospy.loginfo("Home point set.")

	def norm(self, point):
		p = Point()

		psum = math.sqrt(point.x **2 + point.y **2 + point.z **2)
		if psum == 0:
			return p

		p.x = point.x / psum
		p.y = point.y / psum
		p.z = point.z / psum

		return p

	def get_pose(self, header, x, y):
		pose = PoseStamped()
		pose.header = header
		pose.pose.position.x = x
		pose.pose.position.y = y
		pose.pose.position.z = 0
		pose.pose.orientation.w = 1.0
		return pose
	
	def sierra_callback(self, msg):
		p = msg.polygon.points

		if len(p) != 4:
			rospy.logwarn("Invalid Polygon, path creation requires a 4 vertex square.")
			return

		poses = []

		# Calculate vectors between points
		vec_side_1 = Point(x=p[1].x - p[0].x, y=p[1].y - p[0].y)
		vec_side_2 = Point(x=p[3].x - p[0].x, y=p[3].y - p[0].y)

		# Compute the lengths of the two sides and the average (which will be our radius)
		length_side_1 = math.sqrt(vec_side_1.x**2 + vec_side_1.y**2)
		length_side_2 = math.sqrt(vec_side_2.x**2 + vec_side_2.y**2)
		radius = (length_side_1 + length_side_2) / 4

		# Calculate the center point of the search area
		center = Point(
			x=(p[0].x + p[2].x) / 2,
			y=(p[0].y + p[2].y) / 2
		)

		poses.append(self.get_pose(msg.header, center.x, center.y))

		angle = math.radians(0)
		poses.append(self.get_pose(msg.header, 
			center.x + radius * math.cos(angle), 
			center.y + radius * math.sin(angle)
		))

		angle = math.radians(60)
		poses.append(self.get_pose(msg.header, 
			center.x + radius * math.cos(angle), 
			center.y + radius * math.sin(angle)
		))

		angle = math.radians(240)
		poses.append(self.get_pose(msg.header, 
			center.x + radius * math.cos(angle), 
			center.y + radius * math.sin(angle)
		))

		angle = math.radians(300)
		poses.append(self.get_pose(msg.header, 
			center.x + radius * math.cos(angle), 
			center.y + radius * math.sin(angle)
		))

		angle = math.radians(120)
		poses.append(self.get_pose(msg.header, 
			center.x + radius * math.cos(angle), 
			center.y + radius * math.sin(angle)
		))

		angle = math.radians(180)
		poses.append(self.get_pose(msg.header, 
			center.x + radius * math.cos(angle), 
			center.y + radius * math.sin(angle)
		))

		poses.append(self.get_pose(msg.header, center.x, center.y))

		if self.home_point != None:
			poses.append(self.get_pose(self.home_header, self.home_point.x, self.home_point.y))

		self.send_path(msg.header, poses)

	def square_callback(self, msg):
		p = msg.polygon.points

		if len(p) != 4:
			rospy.logwarn("Invalid Polygon, path creation requires a 4 vertex square.")
			return

		poses = []

		# Calculate vectors between points
		vec_side_1 = Point(x=p[1].x - p[0].x, y=p[1].y - p[0].y)
		vec_side_2 = Point(x=p[3].x - p[0].x, y=p[3].y - p[0].y)

		# Compute the lengths of the two sides
		length_side_1 = math.sqrt(vec_side_1.x**2 + vec_side_1.y**2)
		length_side_2 = math.sqrt(vec_side_2.x**2 + vec_side_2.y**2)

		# Determine the center of the square
		center = Point(x=p[0].x + vec_side_1.x/2 + vec_side_2.x/2, y=p[0].y + vec_side_1.y/2 + vec_side_2.y/2)

		# The spiral is drawn as a series of line segments, starting from the center of the square
		pos = [center.x, center.y]
		dir = [0, -1]  # Initial direction: up

		# Continue drawing the spiral until we reach the edge of the square
		for step in itertools.count(start=1, step=1):
			if self.step_size * step > max(length_side_1, length_side_2):  # If we would go beyond the edge of the square, end the spiral
				break

			for _ in range(2):  # Each "layer" of the spiral consists of two steps
				for _ in range(step):
					if abs(pos[0] - center.x) <= length_side_1 / 2 and abs(pos[1] - center.y) <= length_side_2 / 2:
						pos[0] += dir[0] * self.step_size
						pos[1] += dir[1] * self.step_size

				
				poses.append(self.get_pose(msg.header, pos[0], pos[1]))
				dir = [-dir[1], dir[0]]  # Rotate direction 90 degrees to the right

		if self.home_point != None:
			poses.append(self.get_pose(self.home_header, self.home_point.x, self.home_point.y))

		self.send_path(msg.header, poses)


	def lawnmower_callback(self, msg):
		p = msg.polygon.points

		if len(p) != 4:
			rospy.logwarn("Invalid Polygon, path creation requires a 4 vertex square.")
			return

		poses = []

		# Calculate vectors between points
		vec_side_1 = Point(x=p[1].x - p[0].x, y=p[1].y - p[0].y)
		vec_side_2 = Point(x=p[3].x - p[0].x, y=p[3].y - p[0].y)

		# Compute the lengths of the two sides
		length_side_1 = math.sqrt(vec_side_1.x**2 + vec_side_1.y**2)
		length_side_2 = math.sqrt(vec_side_2.x**2 + vec_side_2.y**2)

		# Decide which side to move along based on their lengths
		if length_side_1 > length_side_2:
			vec_step = self.norm(vec_side_2)
			vec_side = vec_side_1
			steps = math.ceil(length_side_2 / self.step_size)
		else:
			vec_step = self.norm(vec_side_1)
			vec_side = vec_side_2
			steps = math.ceil(length_side_1 / self.step_size)

		vec_step.x *= self.step_size
		vec_step.y *= self.step_size

		for i in range(steps):
			if i % 2 == 0:  # For every other line, the direction should be reversed
				# Start of the line
				poses.append(self.get_pose(
					msg.header,
					p[0].x + i * vec_step.x,
					p[0].y + i * vec_step.y
				))

				# End of the line
				poses.append(self.get_pose(
					msg.header,
					p[0].x + i * vec_step.x + vec_side.x,
					p[0].y + i * vec_step.y + vec_side.y,
				))
			else:
				# Start of the line
				poses.append(self.get_pose(
					msg.header,
					p[0].x + i * vec_step.x + vec_side.x,
					p[0].y + i * vec_step.y + vec_side.y,
				))

				# End of the line
				poses.append(self.get_pose(
					msg.header,
					p[0].x + i * vec_step.x,
					p[0].y + i * vec_step.y,
				))

		if self.home_point != None:
			poses.append(self.get_pose(self.home_header, self.home_point.x, self.home_point.y))

		self.send_path(msg.header, poses)


box = BoundingBox3DListener()
rospy.spin()


