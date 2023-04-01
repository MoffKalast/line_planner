 #!/usr/bin/env python3

import numpy as np

from utils import *
from geometry_msgs.msg import Point

def project_position(start, end, current, mindist, maxdist, line_divergence, side_offset):
	
	def unit_vector(vector):
		return vector / np.linalg.norm(vector)
	
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
	distance = mindist + (maxdist - mindist) * clamp((line_divergence - np.sqrt(deltadist.dot(deltadist)))/line_divergence,0.0, 1.0)
	new_pos = projection + unit_vector(end_pos - start_pos) * distance

    # Clamp the new position to the end point if necessary
	if np.dot(new_pos - end_pos, end_pos - start_pos) > 0:
		new_pos = end_pos

    # Add the side vector for more aggressive tracking
	additional_vector = -side_offset * (current_pos - projection)

	return np_to_point(new_pos+ additional_vector)