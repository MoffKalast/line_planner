import rospy
import math

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Pose

class DebugMarkers:

	def __init__(self, planning_frame):
		self.planning_frame = planning_frame
		self.marker_pub = rospy.Publisher("line_planner/markers", MarkerArray, queue_size=1)
		self.marker_publish_skip = 0

	def delete_debug_markers(self):

		marker = Marker()
		marker.action = 3

		markerArray = MarkerArray()
		markerArray.markers.append(marker)
		self.marker_pub.publish(markerArray)

	def get_side_vec(self, start_goal, end_goal, max_divergence):
		x1, y1 = start_goal.position.x, start_goal.position.y
		x2, y2 = end_goal.position.x, end_goal.position.y

		delta_x = x2 - x1
		delta_y = y2 - y1

		# Normalize the direction vector
		magnitude = math.hypot(delta_x,delta_y)
		delta_x /= magnitude
		delta_y /= magnitude

		# Calculate perpendicular vectors
		start_left = Point()
		start_left.x = x1 + max_divergence * delta_y
		start_left.y = y1 - max_divergence * delta_x

		start_right = Point()
		start_right.x = x1 - max_divergence * delta_y
		start_right.y = y1 + max_divergence * delta_x

		end_left = Point()
		end_left.x = x2 + max_divergence * delta_y
		end_left.y = y2 - max_divergence * delta_x

		end_right = Point()
		end_right.x = x2 - max_divergence * delta_y
		end_right.y = y2 + max_divergence * delta_x

		return start_left, start_right, end_left, end_right

	def draw_debug_markers(self, target_position, start_goal, end_goal, min_goal_dist, max_divergence):
		
		def sphere_marker(position, marker_id, r, g, b, size):
			marker = Marker()
			marker.header.frame_id = self.planning_frame
			marker.type = Marker.SPHERE
			marker.pose.position = position
			marker.pose.orientation.w = 1.0
			marker.scale.x = size
			marker.scale.y = size
			marker.scale.z = size
			marker.color.a = 0.5
			marker.color.r = r
			marker.color.g = g
			marker.color.b = b
			marker.id = marker_id
			return marker
		
		def line_marker(p_from, p_to, marker_id, r, g, b):
			marker = Marker()
			marker.header.frame_id = self.planning_frame
			marker.type = Marker.LINE_STRIP
			marker.pose.orientation.w = 1.0

			marker.points = [
				Point(p_from.x, p_from.y, p_from.z),
				Point(p_to.x, p_to.y, p_to.z),
			]

			c = ColorRGBA(r,g,b, 1.0)

			marker.colors = [c, c]

			marker.scale.x = 0.05
			marker.id = marker_id
			return marker

		# to avoid flooding
		if self.marker_publish_skip == 0:

			start_left, start_right, end_left, end_right = self.get_side_vec(start_goal, end_goal, max_divergence)

			markerArray = MarkerArray()
			markerArray.markers.append(line_marker(start_left, end_left, 0, 0.1, 0.6, 0.1))
			markerArray.markers.append(line_marker(start_right, end_right, 1, 0.6, 0.2, 0.2))
			markerArray.markers.append(line_marker(start_goal.position, end_goal.position, 2, 0.575, 0.870, 0.0261))
			markerArray.markers.append(sphere_marker(start_goal.position, 3, 1.0, 0.0, 0.0, 0.2))
			markerArray.markers.append(sphere_marker(end_goal.position, 4, 0.0, 0.0, 1.0, min_goal_dist*2))
			markerArray.markers.append(sphere_marker(target_position, 5, 0.0, 1.0, 0.0, 0.2))
			self.marker_pub.publish(markerArray)
		
		self.marker_publish_skip = (self.marker_publish_skip +1)%5