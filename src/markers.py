#!/usr/bin/env python3

import rospy
import math
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Pose

class DebugMarkers:

    def __init__(self, planning_frame):
        self.planning_frame = planning_frame
        self.marker_pub = rospy.Publisher("line_planner/markers", MarkerArray, queue_size=1)

        self.queued_marker_array = None
        self.publish_timer = rospy.Timer(rospy.Duration(0.2), self._timer_callback)  # 200 ms throttle

    def _timer_callback(self, event):
        if self.queued_marker_array is not None:
            self.marker_pub.publish(self.queued_marker_array)
            self.queued_marker_array = None

    def delete_debug_markers(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        markerArray = MarkerArray()
        markerArray.markers.append(marker)
        self.queued_marker_array = markerArray

    def get_side_vec(self, start_goal, end_goal, max_divergence):
        start = start_goal.position
        end = end_goal.position

        delta_x = end.x - start.x
        delta_y = end.y - start.y

        # Normalize the direction vector
        magnitude = math.hypot(delta_x, delta_y) + 0.0001
        delta_x = (delta_x / magnitude) * max_divergence
        delta_y = (delta_y / magnitude) * max_divergence

        # Calculate perpendicular vectors
        start_left = Point(start.x + delta_y, start.y - delta_x, start.z)
        start_right = Point(start.x - delta_y, start.y + delta_x, start.z)

        end_left = Point(end.x + delta_y, end.y - delta_x, end.z)
        end_right = Point(end.x - delta_y, end.y + delta_x, end.z)

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
            marker.color = ColorRGBA(r, g, b, 0.5)
            marker.id = marker_id
            return marker

        def line_marker(p_from, p_to, marker_id, r, g, b):
            marker = Marker()
            marker.header.frame_id = self.planning_frame
            marker.type = Marker.LINE_STRIP
            marker.pose.orientation.w = 1.0
            marker.points = [p_from, p_to]
            marker.colors = [ColorRGBA(r, g, b, 1.0), ColorRGBA(r, g, b, 1.0)]
            marker.scale.x = 0.03
            marker.id = marker_id
            return marker

        start_left, start_right, end_left, end_right = self.get_side_vec(start_goal, end_goal, max_divergence)

        markerArray = MarkerArray()
        markerArray.markers.append(line_marker(start_left, end_left, 0, 0.1, 0.6, 0.1))
        markerArray.markers.append(line_marker(start_right, end_right, 1, 0.6, 0.2, 0.2))
        markerArray.markers.append(line_marker(start_goal.position, end_goal.position, 2, 0.870, 0.870, 0.870))
        markerArray.markers.append(sphere_marker(start_goal.position, 3, 1.0, 0.0, 0.0, 0.2))
        markerArray.markers.append(sphere_marker(end_goal.position, 4, 0.0, 0.0, 1.0, min_goal_dist * 2))
        markerArray.markers.append(sphere_marker(target_position, 5, 0.0, 1.0, 0.0, 0.2))
        self.queued_marker_array = markerArray
