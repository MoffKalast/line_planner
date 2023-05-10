#!/usr/bin/env python

import rospy
from vision_msgs.msg import BoundingBox3D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from dynamic_reconfigure.server import Server as DynamicReconfigureServer

class BoundingBox3DListener:
    def __init__(self):
        rospy.init_node("box_path_generator")

        self.path_pub = rospy.Publisher("/move_base_simple/waypoints", Path, queue_size=10)
        self.bbox_sub = rospy.Subscriber("/area_path_demo", BoundingBox3D, self.bbox_callback)
        self.step_size = rospy.get_param("~step_size", 2.0)  # The step size (X meters) in the path

    def bbox_callback(self, bbox):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        min_x = bbox.center.position.x - bbox.size.x
        max_x = bbox.center.position.x + bbox.size.x
        min_y = bbox.center.position.y - bbox.size.y
        max_y = bbox.center.position.y + bbox.size.y

        x = min_x
        while x <= max_x:
            y_points = [min_y, max_y] if (x - min_x) // self.step_size % 2 == 0 else [max_y, min_y]

            for y in y_points:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = bbox.center.position.z
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)

            x += self.step_size

        self.path_pub.publish(path)



box = BoundingBox3DListener()
rospy.spin()