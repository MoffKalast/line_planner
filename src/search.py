#!/usr/bin/env python3
import heapq
import math

from obstacles import Obstacles

class WishUponAStar:

	def __init__(self, obstacles):
		self.obstacles = obstacles

	def heuristic(self, start, goal):
		# Using Euclidean distance
		return math.hypot(start[0] - goal[0], start[1] - goal[1])

	def get_neighbors(self, node, max_width, max_width_sq):
		def get_obstacle_hits(p):
			x0, y0 = p
			hits = 0
			for x in range(-max_width, max_width + 1):
				for y in range(-max_width, max_width + 1):
					if (x0 + x, y0 + y) in self.obstacles.entries: #x**2 + y**2 <= max_width_sq and 
						hits +=1
			return hits

		x, y = node
		directions = [
			(x, y - 1),(x - 1, y), (x + 1, y),(x, y + 1)
		]

		overlap_points = [(get_obstacle_hits(p), p) for p in directions]
		neighbors = [p for hits, p in overlap_points if hits == 0]

		if len(neighbors) == 0:
			#find the least obstructed point and go that way to get unstuck
			return [sorted(overlap_points, key=lambda x: x[0])[0][1]]

		return neighbors

	def search(self, start, goal, path_width, max_distance=20.0):

		max_width = math.ceil(path_width)
		max_width_sq = math.ceil(path_width**2)
		max_distance = self.heuristic(start, goal) * 2 + max_distance

		open_set = []
		heapq.heappush(open_set, (0, start))
		came_from = {}
		g_score = {start: 0}
		f_score = {start: self.heuristic(start, goal)}

		# Initialize with the starting node as the best so far if the goal is unreachable
		closest_node = start
		closest_distance = self.heuristic(start, goal)

		while open_set:
			current = heapq.heappop(open_set)[1]

			# Check if the current node is within the max_distance allowed
			if self.heuristic(current, goal) > max_distance:
				continue  # Skip processing this node as it's beyond the max distance

			if current == goal:
				return self.reconstruct_path(came_from, current, max_width)

			for neighbor in self.get_neighbors(current, max_width, max_width_sq):
				tentative_g_score = g_score[current] + math.sqrt(self.heuristic(current, neighbor))

				if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
					came_from[neighbor] = current
					g_score[neighbor] = tentative_g_score
					f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
					heapq.heappush(open_set, (f_score[neighbor], neighbor))

					# Update the closest node if this is a new closest point to the goal
					distance_to_goal = self.heuristic(neighbor, goal)
					if distance_to_goal < closest_distance:
						closest_node = neighbor
						closest_distance = distance_to_goal

		# If the goal was unreachable within the max_distance, return the path to the closest node found
		if closest_node != goal:
			return self.reconstruct_path(came_from, closest_node, max_width)

		return None  # No path found and start was the closest

	def reconstruct_path(self, came_from, current, max_width):
		path = [current]
		while current in came_from:
			current = came_from[current]
			path.append(current)
		path.reverse()
		return self.optimize_path(path, max_width)


	def has_line_of_sight(self, point1, point2):
		# Check if there's a clear line of sight between point1 and point2 with Bresenham
		x0 = round(point1[0])
		y0 = round(point1[1])

		x1 = round(point2[0])
		y1 = round(point2[1])

		dx = x1 - x0
		dy = y1 - y0
		sx = -1 if dx < 0 else 1
		sy = -1 if dy < 0 else 1

		dx = abs(dx)
		dy = abs(dy)

		if dx > dy:
			err = dx / 2.0
			while x0 != x1:
				if (x0, y0) in self.obstacles.entries:
					return False
				err -= dy
				if err <= 0:
					y0 += sy
					err += dx
				x0 += sx
		else:
			err = dy / 2.0
			while y0 != y1:
				if (x0, y0) in self.obstacles.entries:
					return False
				err -= dx
				if err <= 0:
					x0 += sx
					err += dy
				y0 += sy

		return True

	def has_clear_corridor(self, point1, point2, max_width):

		if point1 == point2:
			return True

		x1, y1 = point1
		x2, y2 = point2

		delta_x = x2 - x1
		delta_y = y2 - y1

		# Normalize the direction vector
		magnitude = math.hypot(delta_x,delta_y)
		delta_x = (delta_x / magnitude) * max_width
		delta_y = (delta_y / magnitude) * max_width

		# Calculate perpendicular vectors
		start_left = (x1 + delta_y, y1 - delta_x)
		start_right = (x1 - delta_y, y1 + delta_x)

		end_left = (x2 + delta_y, y2 - delta_x)
		end_right = (x2 - delta_y, y2 + delta_x)

		return self.has_line_of_sight(start_left, end_left) and self.has_line_of_sight(start_right, end_right)

	def optimize_path(self, path, max_width):
		# Reduce number of points in path by skipping unnecessary waypoints
		if not path:
			return []

		optimized_path = [path[0]]
		skip = 0

		for i in range(1, len(path)):
			if skip > 0:
				skip -= 1
				continue
			
			for j in range(len(path) - 1, i, -1):
				if self.has_clear_corridor(optimized_path[-1], path[j], max_width):
					optimized_path.append(path[j])
					skip = j - i - 1
					break
			else:
				optimized_path.append(path[i])

		return optimized_path[:-1]