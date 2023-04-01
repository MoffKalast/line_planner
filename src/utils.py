#!/usr/bin/env python3

import math

from geometry_msgs.msg import Pose

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