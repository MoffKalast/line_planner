#!/usr/bin/env python3

import math
import numpy as np

from geometry_msgs.msg import Point, Pose

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

def project_position(start, end, current, mindist, maxdist, line_divergence):
	
	def unit_vector(vector):
		return vector / np.linalg.norm(vector)
	
	def pose_to_np(p):
		return np.array([p.position.x, p.position.y])

	def np_to_point(np_array):
		return Point(np_array[0], np_array[1], 0)

	start_pos = pose_to_np(start)
	end_pos = pose_to_np(end)
	current_pos = pose_to_np(current)

	line = end_pos - start_pos
	unit_line = unit_vector(line)
	
	current_to_start = current_pos - start_pos
	projection_length = np.dot(current_to_start, unit_line)
	projection = start_pos + unit_line * projection_length

	if np.dot(projection - start_pos, start_pos - end_pos) > 0:
		projection = start_pos

	deltadist = current_pos - projection 
	distance = mindist + (maxdist - mindist) * clamp((line_divergence - np.sqrt(deltadist.dot(deltadist)))/line_divergence,0.0, 1.0)

	new_pos = projection + unit_vector(end_pos - start_pos) * distance

	if np.dot(new_pos - end_pos, end_pos - start_pos) > 0:
		new_pos = end_pos

	additional_vector = -0.5 * (current_pos - projection)

	return np_to_point(new_pos+ additional_vector)

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