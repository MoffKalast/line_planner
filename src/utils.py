#!/usr/bin/env python3
import math
import numpy as np

from geometry_msgs.msg import Pose, Point

def find_closest_segment(point, path):
	min_distance = distance_point_to_line_segment(point, path[0], path[1])
	closest_segment_index = 0

	for i in range(1, len(path) - 1):
		segment_start = path[i]
		segment_end = path[i + 1]
		distance = distance_point_to_line_segment(point, segment_start, segment_end)
		if distance < min_distance:
			min_distance = distance
			closest_segment_index = i

	return closest_segment_index

def distance_point_to_line_segment(point, segment_start, segment_end):
	px, py = point
	x1, y1 = segment_start
	x2, y2 = segment_end

	dx = x2 - x1
	dy = y2 - y1
	length_squared = dx * dx + dy * dy

	# Avoid division by zero if the line segment is a point
	if length_squared == 0:
		return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)

	t = ((px - x1) * dx + (py - y1) * dy) / length_squared
	t = max(0, min(1, t))

	closest_x = x1 + t * dx
	closest_y = y1 + t * dy

	distance_squared = (px - closest_x) ** 2 + (py - closest_y) ** 2

	return math.sqrt(distance_squared)

def project_position(start, end, current, mindist, maxdist, line_divergence, side_offset):
	
	def unit_vector(vector):
		return vector / np.linalg.norm(vector + 1e-10)  # A small constant to prevent division by zero
	
	def pose_to_np(p):
		return np.array([p.position.x, p.position.y])

	def np_to_point(np_array):
		return Point(np_array[0], np_array[1], 0)

	start_pos = pose_to_np(start)
	end_pos = pose_to_np(end)
	current_pos = pose_to_np(current)

	# Calculate the vector representing the line segment
	line = end_pos - start_pos
	unit_line = unit_vector(line)
	
	# Calculate the projection of the current point onto the line
	current_to_start = current_pos - start_pos
	projection_length = np.dot(current_to_start, unit_line)
	projection = start_pos + unit_line * projection_length

	# Clamp the projection to the start point if necessary
	if np.dot(projection - start_pos, start_pos - end_pos) > 0:
		projection = start_pos

	# Calculate the distance and new position based on input parameters
	deltadist = current_pos - projection 
	value = 0 if line_divergence == 0 else (line_divergence - np.sqrt(deltadist.dot(deltadist))) / line_divergence
	distance = mindist + (maxdist - mindist) * clamp(value, 0.0, 1.0)
	new_pos = projection + unit_vector(end_pos - start_pos) * distance

	# Clamp the new position to the end point if necessary
	if np.dot(new_pos - end_pos, end_pos - start_pos) > 0:
		new_pos = end_pos

	# Add the side vector for more aggressive tracking
	additional_vector = -side_offset * (current_pos - projection)

	return np_to_point(new_pos+ additional_vector)

def clamp(num, min, max):
	return min if num < min else max if num > max else num

def transform_to_pose(t):
	pose = Pose()
	pose.position.x = t.transform.translation.x
	pose.position.y = t.transform.translation.y
	pose.position.z = t.transform.translation.z
	pose.orientation.x = t.transform.rotation.x
	pose.orientation.y = t.transform.rotation.y
	pose.orientation.z = t.transform.rotation.z
	pose.orientation.w = t.transform.rotation.w

	return pose

def get_dir(from_vec, to_vec):
	target_direction = [
		to_vec.x - from_vec.x,
		to_vec.y - from_vec.y,
	]
	target_distance = math.sqrt(target_direction[0] ** 2 + target_direction[1] ** 2)

	return [target_direction[0] / target_distance, target_direction[1] / target_distance]

class PID:
	def __init__(self, kp, ki, kd, target=0, windup_guard=20.0):
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.target = target

		self.windup_guard = windup_guard

		self.prev_error = 0.0
		self.integral = 0.0
		self.derivative = 0.0

	def reset(self):
		self.prev_error = 0.0
		self.integral = 0.0
		self.derivative = 0.0

	def compute(self, current_value):
		error = self.target - current_value

		self.integral += error
		self.integral = max(min(self.integral, self.windup_guard), -self.windup_guard)

		self.derivative = error - self.prev_error

		output = self.kp * error + self.ki * self.integral + self.kd * self.derivative

		self.prev_error = error

		return output