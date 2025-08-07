#!/usr/bin/env python3
import math
import rospy
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import PyKDL
from collections import defaultdict

from std_msgs.msg import Empty
from geometry_msgs.msg import PolygonStamped, Point32 
from nav_msgs.msg import GridCells
from sensor_msgs.msg import Range, LaserScan
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point, PointStamped
from rospy.exceptions import ROSTimeMovedBackwardsException

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
		self.MIN_HITS = rospy.get_param('~min_hits_threshold', 3)

		self.tf2_buffer = tf2_ros.Buffer()
		self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

		self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
		self.sonar_sub = rospy.Subscriber('/sonars', Range, self.range_callback)

		self.cells_pub = rospy.Publisher('/obstacle_grid/add_cells', GridCells, queue_size=1)

		self.message = self.new_grid()
		
		# Hit counting for scan data
		self.scan_hit_counts = defaultdict(int)

	def new_grid(self):
		grid = GridCells()
		grid.header.frame_id = self.PLANNING_FRAME
		grid.cell_width = self.GRID_SIZE
		grid.cell_height = self.GRID_SIZE
		grid.cells = []
		return grid

	def send(self):
		# Add scan cells that meet the minimum hit threshold
		for (x, y), count in self.scan_hit_counts.items():
			if count >= self.MIN_HITS:
				point = Point32()
				point.x = x * self.GRID_SIZE
				point.y = y * self.GRID_SIZE
				point.z = 0
				self.message.cells.append(point)
		
		# Clear hit counts after processing
		self.scan_hit_counts.clear()

		if len(self.message.cells) == 0:
			return
		
		self.cells_pub.publish(self.message)
		self.message = self.new_grid()

	def transform_points_vectorized(self, points_array, header, target_frame):
		"""
		Vectorized point transformation and gridding
		points_array: Nx3 numpy array of points
		"""
		try:
			transform = self.tf2_buffer.lookup_transform(target_frame, header.frame_id, rospy.Time(0), rospy.Duration(1.0))
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			rospy.logerr('Error finding transform: %s' % e)
			return None
		
		kdl_tf = transform_to_kdl(transform)
		
		# Transform points using KDL (could be optimized further with pure numpy transforms)
		transformed_points = []
		for i in range(points_array.shape[0]):
			point = points_array[i]
			p = kdl_tf * PyKDL.Vector(point[0], point[1], point[2])
			transformed_points.append([p[0], p[1]])
		
		# Vectorized gridding
		transformed_array = np.array(transformed_points)
		grid_coords = np.round(transformed_array / self.GRID_SIZE).astype(int)
		
		# Convert to set of tuples for deduplication
		return set(map(tuple, grid_coords))

	def scan_callback(self, msg):
		# Convert to numpy and handle NaN values
		ranges = np.array(msg.ranges)
		
		# Filter out NaN, inf, and out-of-range values
		valid_mask = (
			~np.isnan(ranges) & 
			~np.isinf(ranges) & 
			(ranges >= msg.range_min) & 
			(ranges <= msg.range_max)
		)
		
		valid_indices = np.where(valid_mask)[0]
		
		if len(valid_indices) == 0:
			return
		
		valid_ranges = ranges[valid_indices]
		
		# Vectorized angle calculation
		angles = msg.angle_min + valid_indices * msg.angle_increment
		
		# Vectorized coordinate transformation
		x_coords = valid_ranges * np.cos(angles)
		y_coords = valid_ranges * np.sin(angles)
		z_coords = np.zeros(len(valid_ranges))
		
		# Stack into Nx3 array
		points_array = np.column_stack([x_coords, y_coords, z_coords])
		
		# Transform and grid the points
		world_points = self.transform_points_vectorized(points_array, msg.header, self.PLANNING_FRAME)

		if world_points is None:
			return
		
		# Increment hit counts for each grid cell
		for (x, y) in world_points:
			self.scan_hit_counts[(x, y)] += 1

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

				# Range sensors are added immediately (no hit counting)
				self.message.cells += [
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

			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				rospy.logerr("Transform failed: %s", str(e))
		

sensor_node = SensorObstacleNode()
rate = rospy.Rate(rospy.get_param('rate', 3.0))

while not rospy.is_shutdown():
	try:
		sensor_node.send()
		rate.sleep()
	except ROSTimeMovedBackwardsException as e:
		print(e)