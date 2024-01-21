#!/usr/bin/env python3
import rospy
import numpy as np

from std_msgs.msg import Empty
from geometry_msgs.msg import PolygonStamped, Point32 
from nav_msgs.msg import GridCells

class Obstacles:

	def __init__(self, tf2_buffer, PLANNING_FRAME, GRID_SIZE, obstacle_change_callback):

		self.tf2_buffer = tf2_buffer
		self.PLANNING_FRAME = PLANNING_FRAME
		self.GRID_SIZE = GRID_SIZE
		self.obstacle_change_callback = obstacle_change_callback

		self.entries = set()
		self.remove_sub = rospy.Subscriber('/obstacle_grid/remove_polygon', PolygonStamped, self.remove_obstacle) 
		self.add_sub = rospy.Subscriber('/obstacle_grid/add_polygon', PolygonStamped, self.add_obstacle)
		self.add_sub = rospy.Subscriber('/obstacle_grid/add_cells', GridCells, self.add_cells)
		self.add_sub = rospy.Subscriber('/obstacle_grid/clear', Empty, self.clear_grid)
		self.grid_pub = rospy.Publisher('/obstacle_grid', GridCells, queue_size=10, latch=True)

		self.publish_grid()
		
	def clear_grid(self, msg):
		self.entries.clear()
		self.publish_grid()
	
	def obstacle_count(self):
		return len(self.entries)

	def remove_obstacle(self, msg):
		self.update_grid_polygon(msg.polygon)

	def add_obstacle(self, msg):
		self.update_grid_polygon(msg.polygon, delete=True)

	def add_cells(self, msg):
		if msg.cell_width != self.GRID_SIZE or msg.cell_height != self.GRID_SIZE:
			rospy.logerr("Obstacle grid size doesn't match, it should be "+str(self.GRID_SIZE))
			return

		grid_changed = False
		for point in msg.cells:
			point_tup = (round(point.x / self.GRID_SIZE),round(point.y / self.GRID_SIZE))
			if not point_tup in self.entries:
				self.entries.add(point_tup)
				grid_changed = True

		if grid_changed:
			self.publish_grid()

	def update_grid_polygon(self, polygon, delete=False):
		#TODO transform polygon into self.planning_frame

		min_x = min(p.x for p in polygon.points)
		max_x = max(p.x for p in polygon.points)
		min_y = min(p.y for p in polygon.points)
		max_y = max(p.y for p in polygon.points)
		
		# Discretize bounding box into grid coordinates
		x_coords = np.arange(round(min_x / self.GRID_SIZE), round(max_x / self.GRID_SIZE), 1)
		y_coords = np.arange(round(min_y / self.GRID_SIZE), round(max_y / self.GRID_SIZE), 1)

		# Mark grid cells inside polygon as occupied
		if delete:
			for x in x_coords:
				for y in y_coords:
					self.entries.add((x,y))
		else:
			for x in x_coords:
				for y in y_coords:
					self.entries.discard((x,y))

		self.publish_grid()

	def publish_grid(self):
		grid = GridCells()
		grid.header.frame_id = self.PLANNING_FRAME
		grid.cell_width = self.GRID_SIZE
		grid.cell_height = self.GRID_SIZE
		grid.cells = []
		for (x,y) in self.entries:
			point = Point32()
			point.x = x * self.GRID_SIZE
			point.y = y * self.GRID_SIZE
			grid.cells.append(point)
			
		self.grid_pub.publish(grid)
		self.obstacle_change_callback()