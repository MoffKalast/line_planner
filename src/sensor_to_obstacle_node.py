#!/usr/bin/env python3
import math
import rospy
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import PyKDL

from std_msgs.msg import Empty
from geometry_msgs.msg import PolygonStamped, Point32 
from nav_msgs.msg import GridCells
from sensor_msgs.msg import Range, LaserScan
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point, PointStamped

from tf2_geometry_msgs import do_transform_point

from dynamic_reconfigure.server import Server as DynamicReconfigureServer

def transform_to_kdl(t):
	return PyKDL.Frame(
		PyKDL.Rotation.Quaternion(
			t.transform.rotation.x, 
			t.transform.rotation.y,
			t.transform.rotation.z,
			t.transform.rotation.w
		),
		PyKDL.Vector(
			t.transform.translation.x, 
			t.transform.translation.y, 
			t.transform.translation.z
		)
	)

class SensorObstacleNode:
	def __init__(self):
		rospy.init_node("sensor_to_obstacle_node")

		self.PLANNING_FRAME = rospy.get_param('~planning_frame', 'map')
		self.GRID_SIZE = rospy.get_param('~obstacle_grid_size', 1.0)

		self.tf2_buffer = tf2_ros.Buffer()
		self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

		self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
		self.sonar_sub = rospy.Subscriber('/sonars', Range, self.range_callback)

		self.cells_pub = rospy.Publisher('/obstacle_grid/add_cells', GridCells, queue_size=1)

	def transform_points(self, points, source_frame, target_frame):
		try:
			transform = self.tf2_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			rospy.logerr('Error finding transform: %s' % e)
			return None
		
		kdl_tf = transform_to_kdl(transform)

		transformed_set = set()
		for point in points:
			p = kdl_tf * PyKDL.Vector(point[0], point[1], point[2])
			transformed_set.add((round(p[0] / self.GRID_SIZE), round(p[1] / self.GRID_SIZE)))

		return transformed_set

	def scan_callback(self, msg):
		points = []
		for index, distance in enumerate(msg.ranges):
			if distance >= msg.range_min and distance <= msg.range_max:
				angle = msg.angle_min + index * msg.angle_increment
				points.append((distance * math.cos(angle), distance * math.sin(angle), 0))
			
		world_points = self.transform_points(points, msg.header.frame_id, self.PLANNING_FRAME)

		if world_points is None:
			return

		grid = GridCells()
		grid.header.frame_id = self.PLANNING_FRAME
		grid.cell_width = self.GRID_SIZE
		grid.cell_height = self.GRID_SIZE
		grid.cells = []
		for (x,y) in world_points:
			point = Point32()
			point.x = x * self.GRID_SIZE
			point.y = y * self.GRID_SIZE
			grid.cells.append(point)
			
		self.cells_pub.publish(grid)		

	def range_callback(self, msg):
		if msg.range < msg.max_range:
			# Project the point into the "odom" frame
			sonar_point = PointStamped()
			sonar_point.header = msg.header
			sonar_point.point.x = msg.range  # x-coordinate is the range
			sonar_point.point.y = 0.0  # y-coordinate can be set based on your requirement
			sonar_point.point.z = 0.0  # z-coordinate, assuming it's in 2D

			try:
				# Transform the point to the "odom" frame
				transform_stamped = self.tf2_buffer.lookup_transform(self.PLANNING_FRAME, sonar_point.header.frame_id, rospy.Time(0))
				point = do_transform_point(sonar_point, transform_stamped).point
				tuple = (int(round(point.x / self.GRID_SIZE)),int(round(point.y / self.GRID_SIZE)))

				grid = GridCells()
				grid.header.frame_id = self.PLANNING_FRAME
				grid.cell_width = self.GRID_SIZE
				grid.cell_height = self.GRID_SIZE
				grid.cells = [
					Point32(
						tuple[0] * self.GRID_SIZE,
						tuple[1] * self.GRID_SIZE,
						0
					),
					Point32(
						(tuple[0]+1) * self.GRID_SIZE,
						tuple[1] * self.GRID_SIZE,
						0
					),
					Point32(
						tuple[0] * self.GRID_SIZE,
						(tuple[1]+1) * self.GRID_SIZE,
						0
					),
					Point32(
						(tuple[0]-1) * self.GRID_SIZE,
						tuple[1] * self.GRID_SIZE,
						0
					),
										Point32(
						tuple[0] * self.GRID_SIZE,
						(tuple[1]-1) * self.GRID_SIZE,
						0
					)
				]
					
				self.cells_pub.publish(grid)

			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				rospy.logerr("Transform failed: %s", str(e))
		

sensor_node = SensorObstacleNode()
rospy.spin()