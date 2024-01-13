#!/usr/bin/env python3
import rospy
import tf2_ros
import laser_geometry
import numpy as np

from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import PolygonStamped, Point32, PointStamped 
from nav_msgs.msg import GridCells
from sensor_msgs.msg import Range

from tf2_geometry_msgs import do_transform_point 

class Obstacles:

	def __init__(self, planning_frame, tf2_buffer, grid_size, obstacle_change_callback):

		self.planning_frame = planning_frame
		self.tf2_buffer = tf2_buffer
		self.grid_size = grid_size
		self.obstacle_change_callback = obstacle_change_callback

		self.entries = set()
		self.remove_sub = rospy.Subscriber('/obstacle_grid/remove_polygon', PolygonStamped, self.remove_obstacle) 
		self.add_sub = rospy.Subscriber('/obstacle_grid/add_polygon', PolygonStamped, self.add_obstacle)
		self.add_sub = rospy.Subscriber('/obstacle_grid/add_points', PolygonStamped, self.add_obstacle)
		self.add_sub = rospy.Subscriber('/obstacle_grid/clear', Empty, self.clear_obstacles)
		self.grid_pub = rospy.Publisher('/obstacle_grid', GridCells, queue_size=10, latch=True)

		self.sonar_sub = rospy.Subscriber('/sonars', Range, self.range_callback)

		# Create a LaserProjection object to convert LaserScan to PointCloud2
		self.laser_projector = laser_geometry.LaserProjection()
		self.publish_grid()
		
	def clear_obstacles(self, msg):
		self.entries.clear()
		self.publish_grid()
	
	def obstacle_count(self):
		return len(self.entries)

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
				transform_stamped = self.tf2_buffer.lookup_transform(self.planning_frame, sonar_point.header.frame_id, rospy.Time(0))
				point = do_transform_point(sonar_point, transform_stamped).point
				tuple = (int(round(point.x / self.grid_size)),int(round(point.y / self.grid_size)))

				if not tuple in self.entries:
					self.entries.add(tuple)
					self.entries.add((tuple[0]+1,tuple[1]))
					self.entries.add((tuple[0],tuple[1]+1))
					self.entries.add((tuple[0]-1,tuple[1]))
					self.entries.add((tuple[0],tuple[1]-1))

					self.publish_grid()

			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				rospy.logerr("Transform failed: %s", str(e))

	def remove_obstacle(self, msg):
		self.update_grid_polygon(msg.polygon)

	def add_obstacle(self, msg):
		self.update_grid_polygon(msg.polygon, delete=True)

	def update_grid_polygon(self, polygon, delete=False):
		
		#TODO transform polygon into self.planning_frame

		min_x = min(p.x for p in polygon.points)
		max_x = max(p.x for p in polygon.points)
		min_y = min(p.y for p in polygon.points)
		max_y = max(p.y for p in polygon.points)
		
		# Discretize bounding box into grid coordinates
		x_coords = np.arange(round(min_x / self.grid_size), round(max_x / self.grid_size), 1)
		y_coords = np.arange(round(min_y / self.grid_size), round(max_y / self.grid_size), 1)

		# Mark grid cells inside polygon as occupied
		if delete:
			for x in x_coords:
				for y in y_coords:
					self.entries.add((int(x),int(y)))
		else:
			for x in x_coords:
				for y in y_coords:
					self.entries.discard((int(x),int(y)))

		self.publish_grid()

	def publish_grid(self):
		grid = GridCells()
		grid.header.frame_id = self.planning_frame
		grid.cell_width = self.grid_size
		grid.cell_height = self.grid_size
		grid.cells = []
		for (x,y) in self.entries:
			point = Point32()
			point.x = x * self.grid_size
			point.y = y * self.grid_size
			grid.cells.append(point)
			
		self.grid_pub.publish(grid)
		self.obstacle_change_callback()